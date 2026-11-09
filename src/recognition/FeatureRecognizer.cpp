// FeatureRecognizer 实现 — 参见 FeatureRecognizer.h。
// 模块: oap_recognition。
#include "FeatureRecognizer.h"
#include "recognition/detail/HoleToolpathBuilder.h"

#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>


#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepAdaptor_Surface.hxx>

#include "AAG.h"
#include "Recognition.h"
#include "RecognitionGeometryCache.h"

namespace oap {

namespace {
// 特征类别配色（RGB，0~1，sRGB 线性分量），用于识别结果视图高亮。
const Quantity_Color kColorThroughHole    (0.0, 1.0, 0.0, Quantity_TOC_RGB);  // 通孔=绿
const Quantity_Color kColorBlindHole      (0.0, 0.0, 1.0, Quantity_TOC_RGB);  // 盲孔=蓝
const Quantity_Color kColorCountersinkHole(1.0, 1.0, 0.0, Quantity_TOC_RGB);  // 沉孔=黄
const Quantity_Color kColorSlot           (0.55, 0.0, 0.0, Quantity_TOC_RGB); // 槽=深红
const Quantity_Color kColorPocket         (0.0, 0.5, 0.5, Quantity_TOC_RGB);  // 腔=深青
const Quantity_Color kColorDefault        (0.78, 0.78, 0.80, Quantity_TOC_RGB); // 未识别=中性灰

} // namespace

// 特征类别 -> 识别结果视图颜色。
// 当前方案：通孔=绿、盲孔=蓝、沉孔=黄；槽=红、腔=青。
Quantity_Color featureColor(FeatureKind kind) {
    switch (kind) {
        case FeatureKind::ThroughHole:     return kColorThroughHole;
        case FeatureKind::BlindHole:       return kColorBlindHole;
        case FeatureKind::CountersinkHole: return kColorCountersinkHole;
        case FeatureKind::Slot:            return kColorSlot;
        case FeatureKind::Pocket:          return kColorPocket;
    }
    return kColorDefault;
}

namespace {
// 按 face_id 的数字后缀比较(F2 < F10),与项目其余排序约定一致。
long long faceSuffixNum(const std::string& id) {
    size_t i = 0;
    while (i < id.size() && (id[i] < '0' || id[i] > '9')) ++i;
    long long n = 0;
    for (; i < id.size() && id[i] >= '0' && id[i] <= '9'; ++i) n = n * 10 + (id[i] - '0');
    return n;
}

} // namespace

// 实例去重：删除面集完全重复或真包含的子集实例，保留人工新建实例。
void RecognitionResult::dedupInstances() {
    // 先把每个实例的面集规范化(排序+去重),使下面的集合比较稳定。
    for (auto& inst : instances) {
        std::sort(inst.faceIds.begin(), inst.faceIds.end(),
                  [](const std::string& a, const std::string& b) {
                      return faceSuffixNum(a) < faceSuffixNum(b);
                  });
        inst.faceIds.erase(std::unique(inst.faceIds.begin(), inst.faceIds.end()),
                           inst.faceIds.end());
    }

    // 用规范化后的面集做查找键(已排序去重 -> 用 set 比较包含关系)。
    std::vector<std::unordered_set<std::string>> sets;
    sets.reserve(instances.size());
    for (const auto& inst : instances)
        sets.emplace_back(inst.faceIds.begin(), inst.faceIds.end());

    // 标记要丢弃的实例:面集完全相同的重复(保留下标较小者),
    // 以及被另一非空实例真包含的子集(走廊递归产生的小集合)。
    // 人工新建(manual)的实例不参与去重,既不被删也不删除别人 —— 保留人工意图。
    std::vector<bool> drop(instances.size(), false);
    for (size_t i = 0; i < instances.size(); ++i) {
        if (drop[i] || instances[i].manual || sets[i].empty()) continue;
        for (size_t j = 0; j < instances.size(); ++j) {
            if (i == j || drop[j] || instances[j].manual || sets[j].empty()) continue;
            // sets[j] 是否被 sets[i] 包含(i 覆盖 j)。
            if (sets[j].size() > sets[i].size()) continue;
            bool contained = true;
            for (const auto& id : sets[j])
                if (!sets[i].count(id)) { contained = false; break; }
            if (!contained) continue;
            // 完全相同:保留下标较小者,丢弃较大者(孔先于槽加入,故孔优先得以保持)。
            if (sets[j].size() == sets[i].size()) {
                if (j > i) {
                    drop[j] = true;
                }
            } else {
                // 真子集：丢弃较小的 j。
                drop[j] = true;
            }
        }
    }

    std::vector<Instance> kept;
    kept.reserve(instances.size());
    for (size_t i = 0; i < instances.size(); ++i)
        if (!drop[i]) kept.push_back(std::move(instances[i]));
    instances = std::move(kept);
}

// 重建派生数据：统计孔/槽/腔计数并建立面→类别映射（孔优先，不被后续类别覆盖）。
void RecognitionResult::rebuildDerived() {
    faceKind.clear();
    holeCount = slotCount = pocketCount = 0;
    for (auto& inst : instances) {
        // 实例内面去重并按数字后缀排序,树显示与导出稳定。
        std::sort(inst.faceIds.begin(), inst.faceIds.end(),
                  [](const std::string& a, const std::string& b) {
                      return faceSuffixNum(a) < faceSuffixNum(b);
                  });
        inst.faceIds.erase(std::unique(inst.faceIds.begin(), inst.faceIds.end()),
                           inst.faceIds.end());
        for (const auto& id : inst.faceIds) {
            // 孔优先:已归为孔的面不被后来的槽/腔实例覆盖类别。
            auto it = faceKind.find(id);
            if (it == faceKind.end()) faceKind[id] = inst.kind;
        }
        switch (inst.kind) {
            case FeatureKind::ThroughHole:
            case FeatureKind::BlindHole:
            case FeatureKind::CountersinkHole: ++holeCount;   break;
            case FeatureKind::Slot:            ++slotCount;   break;
            case FeatureKind::Pocket:          ++pocketCount; break;
        }
    }
}

namespace {

// 把 model 中每个 unit 的 solid 收进一个 compound,作为识别算法的输入形状。
// 用同一批 solid 保证 AAG 内的 TopoDS_Face 与 StepIndexer 索引时的面是同一拓扑,
// 从而能经 idForFace() 反查回 "F<n>"。
TopoDS_Compound buildCompound(const Model& model, const StepIndexer& indexer) {
    TopoDS_Compound comp;
    BRep_Builder builder;
    builder.MakeCompound(comp);
    for (const auto& u : model.units) {
        const TopoDS_Solid* solid = indexer.solidForUnit(u.unit_id);
        if (solid && !solid->IsNull())
            builder.Add(comp, *solid);
    }
    return comp;
}

bool finiteBox(const Bbox& box)
{
    return box.valid() && std::isfinite(box.xmin) && std::isfinite(box.ymin) &&
        std::isfinite(box.zmin) && std::isfinite(box.xmax) &&
        std::isfinite(box.ymax) && std::isfinite(box.zmax);
}

Bnd_Box indexedBox(const Bbox& box)
{
    Bnd_Box result;
    if (finiteBox(box)) {
        result.Update(box.xmin, box.ymin, box.zmin,
            box.xmax, box.ymax, box.zmax);
        result.SetGap(0.0);
    }
    return result;
}

bool hasNativeCylinder(const AAG& graph)
{
    for (int index = 1; index <= graph.faceMap.Extent(); ++index) {
        try {
            if (BRepAdaptor_Surface(TopoDS::Face(graph.faceMap(index))).GetType() == GeomAbs_Cylinder)
                return true;
        } catch (const Standard_Failure&) {
            // Match the existing surface classifier's invalid-face behavior.
        }
    }
    return false;
}

// 一次特征识别事务只建立一份 Compound、AAG 和孔识别轻量几何缓存。
// 完整识别与孔专用识别共用该路径，避免两个入口的构建逻辑继续分叉。
class HoleRecognitionSession
{
public:
    void build(
        const Model& model,
        const StepIndexer& indexer)
    {
        m_shape =
            buildCompound(model, indexer);
        if (!m_shape.IsNull())
            m_graph.build(m_shape);
    }

    bool valid() const
    {
        return !m_shape.IsNull() &&
               m_graph.faceMap.Extent() > 0;
    }

    AAG& graph()
    {
        return m_graph;
    }

    const std::vector<HoleResult>& holes(const Model& model, const StepIndexer& indexer)
    {
        if (m_holesReady)
            return m_holes;

        // Hole recognition supports analytic cylinders only. Plane regression
        // cannot create a cylinder candidate, so freeform reliefs need neither
        // precise bounding-box reconstruction nor the outer-face cache here.
        if (!hasNativeCylinder(m_graph)) {
            m_holesReady = true;
            return m_holes;
        }

        Bnd_Box shapeBox;
        bool allUnitBoxesKnown = true;
        for (const auto& unit : model.units) {
            const auto* solid = indexer.solidForUnit(unit.unit_id);
            if (!solid || solid->IsNull()) continue;
            if (!finiteBox(unit.bbox)) {
                allUnitBoxesKnown = false;
                break;
            }
            shapeBox.Add(indexedBox(unit.bbox));
        }
        // A partial union is not a valid bound for a compound. Void requests
        // the original precise AddOptimal fallback for the complete shape.
        if (!allUnitBoxesKnown) shapeBox.SetVoid();

        std::unordered_map<std::string, const Face*> records;
        records.reserve(model.faces.size());
        for (const auto& face : model.faces) records.emplace(face.face_id, &face);
        std::vector<Bnd_Box> faceBoxes(static_cast<std::size_t>(m_graph.faceMap.Extent() + 1));
        for (int index = 1; index <= m_graph.faceMap.Extent(); ++index) {
            const auto found = records.find(indexer.idForFace(TopoDS::Face(m_graph.faceMap(index))));
            if (found != records.end())
                faceBoxes[static_cast<std::size_t>(index)] = indexedBox(found->second->bbox);
        }

        m_geometryCache.build(
            m_shape, m_graph, shapeBox, faceBoxes,
            RecognitionGeometryCacheMode::
                MainOuterFacesOnly);
        m_holes = RecognizeHolesOCCT(
            m_graph, m_geometryCache);
        m_holesReady = true;
        return m_holes;
    }

private:
    TopoDS_Compound m_shape;
    AAG m_graph;
    RecognitionGeometryCache m_geometryCache;
    std::vector<HoleResult> m_holes;
    bool m_holesReady = false;
};

} // namespace

// 完整识别入口：构建 compound + AAG -> 运行孔识别 -> 经 faceIdOf 反查
// "F<n>" 面 id -> 生成 instances / holeGeometries -> 去重并重建派生数据。
// 全程 try/catch 兜底，退化拓扑的异常不会冒泡到 Qt 事件循环。
RecognitionResult FeatureRecognizer::recognize(const Model& model,
                                               const StepIndexer& indexer) {
    RecognitionResult out;

    // 构建 compound 与 AAG 邻接图。这一前置阶段同样会做 OCCT 拓扑遍历
    // (TopExp::MapShapesAndAncestors 等),退化/非流形 STEP 数据可能抛
    // Standard_Failure —— 必须与下方孔/槽识别一样兜底,否则异常会冒泡到
    // Qt 事件循环触发 abort 弹窗(GUI 的三个识别入口都经此路径)。
    HoleRecognitionSession session;
    try {
        session.build(model, indexer);
    } catch (const Standard_Failure& e) {
        std::cerr << "[FeatureRecognizer] AAG build failed: "
                  << e.GetMessageString() << std::endl;
        return out;
    } catch (const std::exception& e) {
        std::cerr << "[FeatureRecognizer] AAG build failed: "
                  << e.what() << std::endl;
        return out;
    }
    if (!session.valid()) return out;
    AAG& graph = session.graph();

    // AAG 1-based 面索引 -> "F<n>"。同一拓扑面经 StepIndexer 反查得到稳定 id。
    auto faceIdOf = [&](int aagIdx) -> std::string {
        if (aagIdx < 1 || aagIdx > graph.faceMap.Extent()) return std::string();
        const TopoDS_Face f = TopoDS::Face(graph.faceMap(aagIdx));
        return indexer.idForFace(f);
    };

    // 当前自动识别仅保留孔：把孔涉及的面按通/盲/沉细分标记。
    // 整体用 try/catch 兜底:大模型的退化拓扑可能让识别算法抛 OCCT/STL 异常,
    // 不能让它冒泡到 Qt 事件循环触发 abort 弹窗 —— 失败时返回已识别的部分。
    try {
        const std::vector<HoleResult>& holes =
            session.holes(model, indexer);
        for (const auto& h : holes) {
            FeatureKind kind;
            switch (h.type) {
                case HoleType::THROUGH:     kind = FeatureKind::ThroughHole;     break;
                case HoleType::BLIND:       kind = FeatureKind::BlindHole;       break;
                case HoleType::COUNTERSINK: kind = FeatureKind::CountersinkHole; break;
                default: continue; // UNKNOWN 不上色
            }
            RecognitionResult::Instance inst;
            inst.kind = kind;
            for (int fid : h.faces) {
                const std::string id = faceIdOf(fid);
                if (!id.empty()) inst.faceIds.push_back(id);
            }
            if (!inst.faceIds.empty()) out.instances.push_back(std::move(inst));
        }
        out.holeGeometries = recognition::detail::buildHoleGeometries(
            holes, graph, model, indexer);
    } catch (const Standard_Failure& e) {
        // OCCT 异常:记录后继续(孔识别可能只完成一部分)。
        std::cerr << "[FeatureRecognizer] hole recognition failed: "
                  << e.GetMessageString() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[FeatureRecognizer] hole recognition failed: "
                  << e.what() << std::endl;
    }

    // instances 为权威数据；派生 faceKind 与各类别计数。
    out.dedupInstances();
    out.rebuildDerived();
    return out;
}

// 仅孔加工几何入口：与 recognize() 共用同一 Session 构建路径，返回顺序
// 与 recognize() 中的孔顺序一致（通孔在前、盲孔在后）。
HoleGeometryRecognitionResult FeatureRecognizer::recognizeHoleGeometries(
    const Model& model, const StepIndexer& indexer) {
    HoleGeometryRecognitionResult out;
    using Clock = std::chrono::steady_clock;

    HoleRecognitionSession session;
    const auto aagStart = Clock::now();
    try {
        session.build(model, indexer);
    } catch (const Standard_Failure& e) {
        std::cerr << "[FeatureRecognizer] hole-only AAG build failed: "
                  << e.GetMessageString() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[FeatureRecognizer] hole-only AAG build failed: "
                  << e.what() << std::endl;
    }
    out.aagBuildMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                         Clock::now() - aagStart)
                         .count();
    if (!session.valid())
        return out;
    AAG& graph = session.graph();

    try {
        const auto classificationStart = Clock::now();
        const std::vector<HoleResult>& holes =
            session.holes(model, indexer);
        out.holeClassificationMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - classificationStart)
                .count();

        const auto geometryStart = Clock::now();
        out.geometries = recognition::detail::buildHoleGeometries(
            holes, graph, model, indexer);
        out.geometryBuildMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  Clock::now() - geometryStart)
                                  .count();
    } catch (const Standard_Failure& e) {
        std::cerr << "[FeatureRecognizer] hole-only recognition failed: "
                  << e.GetMessageString() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[FeatureRecognizer] hole-only recognition failed: "
                  << e.what() << std::endl;
    }
    return out;
}

// 从人工选中的完整圆柱壁重建局部孔几何；继续复用自动识别的 AAG 和构建算法。
std::vector<RecognizedHoleGeometry>
FeatureRecognizer::recognizeSelectedHoleGeometries(
    const Model& model,
    const StepIndexer& indexer,
    const std::vector<std::string>& faceIds,
    std::string* error)
{
    if (error)
        error->clear();
    std::vector<RecognizedHoleGeometry> result;
    if (faceIds.empty()) {
        if (error)
            *error = "没有选中用于构建人工孔的面";
        return result;
    }

    TopoDS_Compound compound;
    AAG graph;
    try {
        compound = buildCompound(model, indexer);
        if (!compound.IsNull())
            graph.build(compound);
    } catch (const Standard_Failure& failure) {
        if (error) {
            const char* message = failure.GetMessageString();
            *error = std::string("人工孔邻接图构建失败: ") +
                (message ? message : "OCCT failure");
        }
        return result;
    } catch (const std::exception& failure) {
        if (error) {
            *error = std::string("人工孔邻接图构建失败: ") + failure.what();
        }
        return result;
    }
    if (compound.IsNull() || graph.faceMap.Extent() == 0) {
        if (error)
            *error = "当前模型无法建立人工孔邻接图";
        return result;
    }

    std::set<int> selectedCylinderIndices;
    for (const std::string& faceId : faceIds) {
        const TopoDS_Face* face = indexer.faceForId(faceId);
        if (!face || face->IsNull())
            continue;
        const int index = graph.faceMap.FindIndex(*face);
        if (index > 0 && GetSurfaceType(*face) == GeomAbs_Cylinder)
            selectedCylinderIndices.insert(index);
    }
    if (selectedCylinderIndices.empty()) {
        if (error)
            *error = "所选面中没有完整圆柱孔壁";
        return result;
    }

    std::vector<HoleResult> holes;
    holes.reserve(selectedCylinderIndices.size());
    for (const int faceIndex : selectedCylinderIndices) {
        HoleResult hole = ClassifyHoleOCCT(graph, faceIndex);
        if (hole.type != HoleType::UNKNOWN)
            holes.push_back(std::move(hole));
    }
    if (holes.empty()) {
        if (error) {
            *error =
                "所选圆柱面不是可加工内孔，或其入口、底面拓扑无法可靠解析";
        }
        return result;
    }

    result = recognition::detail::buildHoleGeometries(
        holes, graph, model, indexer);
    if (result.empty() && error)
        *error = "所选圆柱壁未形成具有有效孔轴和深度的孔段";
    return result;
}

} // namespace oap
