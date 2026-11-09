// StepIndexer 实现 —— 参见 StepIndexer.h。
#include "StepIndexer.h"
#include "Naming.h"
#include "detail/StepIndexGeometry.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cctype>

#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_Tool.hxx>

#include <STEPCAFControl_Reader.hxx>
#include <XSControl_WorkSession.hxx>
#include <XSControl_TransferReader.hxx>
#include <Transfer_TransientProcess.hxx>
#include <TransferBRep.hxx>
#include <Interface_InterfaceModel.hxx>
#include <StepRepr_RepresentationItem.hxx>
#include <TCollection_HAsciiString.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAbs_CurveType.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopExp.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

namespace oap {
namespace {

// OCC AABB -> 我们的 Bbox。用 AddOptimal 计算紧致包围盒(基于真实曲面/曲线几何,
// 而非控制点多边形),使详细信息栏的 size 贴近 NX 等 CAD 的读数。Add() 会用样条
// 控制点定界,导致盒子明显偏大(中心不变、尺寸偏大),正是 size 与 NX 对不上的根因。
// 代价:AddOptimal 比 Add 慢(逐面采样几何),大模型加载会变慢,但 size 更准确。
Bbox computeBbox(const TopoDS_Shape& shape) {
    Bbox out;
    Bnd_Box box;
    try { BRepBndLib::AddOptimal(shape, box, Standard_False, Standard_False); } catch (...) { return out; }
    if (box.IsVoid()) return out;
    box.Get(out.xmin, out.ymin, out.zmin, out.xmax, out.ymax, out.zmax);
    return out;
}

// 表面类型枚举 -> 输出字符串(与 Face.surface_type 契约一致)。
const char* surfaceTypeName(GeomAbs_SurfaceType t) {
    switch (t) {
        case GeomAbs_Plane:               return "plane";
        case GeomAbs_Cylinder:            return "cylinder";
        case GeomAbs_Cone:                return "cone";
        case GeomAbs_Sphere:              return "sphere";
        case GeomAbs_Torus:               return "torus";
        case GeomAbs_BezierSurface:       return "bezier";
        case GeomAbs_BSplineSurface:      return "bspline";
        case GeomAbs_SurfaceOfRevolution: return "revolution";
        case GeomAbs_SurfaceOfExtrusion:  return "extrusion";
        default:                          return "other";
    }
}

// 曲线类型枚举 -> 输出字符串(与 Edge.curve_type 契约一致)。
const char* curveTypeName(GeomAbs_CurveType t) {
    switch (t) {
        case GeomAbs_Line:        return "line";
        case GeomAbs_Circle:      return "circle";
        case GeomAbs_Ellipse:     return "ellipse";
        case GeomAbs_Hyperbola:   return "hyperbola";
        case GeomAbs_Parabola:    return "parabola";
        case GeomAbs_BezierCurve: return "bezier";
        case GeomAbs_BSplineCurve:return "bspline";
        default:                  return "other";
    }
}

// 填充一条 edge 记录的几何事实:curve_type / length / bbox / 端点。
void fillEdgeGeometry(const TopoDS_Edge& e, Edge& rec) {
    rec.bbox = computeBbox(e);
    try {
        BRepAdaptor_Curve crv(e);
        rec.curve_type = curveTypeName(crv.GetType());
        GProp_GProps props;
        BRepGProp::LinearProperties(e, props);
        rec.length = props.Mass();
    } catch (...) {
        rec.curve_type = "other";
    }
    // 端点取自首末顶点(退化边可能缺失,此时保持 has_* = false)。
    TopoDS_Vertex v1, v2;
    TopExp::Vertices(e, v1, v2);
    if (!v1.IsNull()) {
        const gp_Pnt p = BRep_Tool::Pnt(v1);
        rec.has_start = true;
        rec.start_point = {p.X(), p.Y(), p.Z()};
    }
    if (!v2.IsNull()) {
        const gp_Pnt p = BRep_Tool::Pnt(v2);
        rec.has_end = true;
        rec.end_point = {p.X(), p.Y(), p.Z()};
    }
}

} // namespace

const TopoDS_Face* StepIndexer::faceForId(const std::string& faceId) const {
    auto it = m_idToFace.find(faceId);
    return it == m_idToFace.end() ? nullptr : &it->second;
}

std::string StepIndexer::idForFace(const TopoDS_Face& face) const {
    if (m_faceToId.IsBound(face)) return m_faceToId.Find(face);
    return std::string();
}

const TopoDS_Edge* StepIndexer::edgeForId(const std::string& edgeId) const {
    auto it = m_idToEdge.find(edgeId);
    return it == m_idToEdge.end() ? nullptr : &it->second;
}

std::string StepIndexer::idForEdge(const TopoDS_Edge& edge) const {
    if (m_edgeToId.IsBound(edge)) return m_edgeToId.Find(edge);
    return std::string();
}

const TopoDS_Wire* StepIndexer::wireForId(const std::string& wireId) const {
    auto it = m_idToWire.find(wireId);
    return it == m_idToWire.end() ? nullptr : &it->second;
}

std::string StepIndexer::idForWire(const TopoDS_Wire& wire) const {
    if (m_wireToId.IsBound(wire)) return m_wireToId.Find(wire);
    return std::string();
}

const TopoDS_Solid* StepIndexer::solidForFaceId(const std::string& faceId) const {
    const auto faceIt = m_faceIdToUnitId.find(faceId);
    if (faceIt == m_faceIdToUnitId.end()) return nullptr;
    return solidForUnit(faceIt->second);
}

const TopoDS_Solid* StepIndexer::solidForUnit(const std::string& unitId) const {
    auto it = m_unitToSolid.find(unitId);
    return it == m_unitToSolid.end() ? nullptr : &it->second;
}

const TopoDS_Shape* StepIndexer::shapeForUnit(const std::string& unitId) const {
    auto it = m_unitToShape.find(unitId);
    return it == m_unitToShape.end() ? nullptr : &it->second;
}

StepIndexer::SolidNameMap StepIndexer::solidNamesFromReader(STEPCAFControl_Reader& reader) {
    // OCCT 的 STEPCAF 不会把 solid/shell 实体名搬进 XCAF(子形状标签全部无名,已实测确认)。
    // 名字在原始 STEP 的 representation_item 上,通过同一工作会话的 TransientProcess
    // 把带名实体映射回它 transfer 出的 TopoDS 形状。已实测:枚举命中率 100%。
    SolidNameMap map;
    Handle(XSControl_WorkSession) ws = reader.ChangeReader().WS();
    if (ws.IsNull()) return map;
    Handle(Interface_InterfaceModel) model = ws->Model();
    Handle(Transfer_TransientProcess) tp =
        ws->TransferReader().IsNull() ? nullptr : ws->TransferReader()->TransientProcess();
    if (model.IsNull() || tp.IsNull()) return map;

    const int nb = model->NbEntities();
    for (int i = 1; i <= nb; ++i) {
        Handle(StepRepr_RepresentationItem) ri =
            Handle(StepRepr_RepresentationItem)::DownCast(model->Value(i));
        if (ri.IsNull() || ri->Name().IsNull()) continue;
        const std::string nm(ri->Name()->ToCString());
        if (nm.empty() || nm == "NONE") continue;
        TopoDS_Shape sh = TransferBRep::ShapeResult(tp, model->Value(i));
        // 实体(solid)与片体(open shell,NX 片体)都登记体名;键为各自的 TopoDS 形状。
        if (!sh.IsNull() &&
            (sh.ShapeType() == TopAbs_SOLID || sh.ShapeType() == TopAbs_SHELL) &&
            !map.IsBound(sh))
            map.Bind(sh, nm);
    }
    return map;
}

bool StepIndexer::build(const Handle(TDocStd_Document)& doc, Model& out,
                        const SolidNameMap* solidNames, const StepIndexBuildControl* control,
                        StepIndexBuildStatistics* statistics) {
    *this = StepIndexer{};
    out = Model{};
    if (statistics) *statistics = {};
    if (control && control->cancelled()) return false;
    StepIndexer pending;
    Model pendingModel;
    if (!pending.buildInPlace(doc, pendingModel, solidNames, control, statistics) ||
        (control && control->cancelled())) return false;
    *this = std::move(pending);
    out = std::move(pendingModel);
    return true;
}
bool StepIndexer::buildInPlace(const Handle(TDocStd_Document)& doc, Model& out,
                        const SolidNameMap* solidNames, const StepIndexBuildControl* control,
                        StepIndexBuildStatistics* statistics) {
    m_idToFace.clear();
    m_faceToId.Clear();
    m_idToEdge.clear();
    m_edgeToId.Clear();
    m_idToWire.clear();
    m_wireToId.Clear();
    m_unitToSolid.clear();
    m_unitToShape.clear();
    m_faceIdToUnitId.clear();
    out = Model();
    if (doc.IsNull()) return false;

    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.IsEmpty()) return false;

    FaceBoundsMap bounds;
    StepIndexBuildStatistics measured;
    if (control && !detail::buildFaceBounds(doc, *control, measured, bounds)) return false;
    if (statistics) *statistics = measured;
    // 收集每个自由形状下所有 solid 的(solid shape, 原始名称)。我们将每个
    // TopoDS_Solid 视为一个 unit;名称在存在时取自其自身的 XCAF 标签。
    int unitCounter = 0, faceCounter = 0, edgeCounter = 0, wireCounter = 0;

    // 将标签的 TDataStd_Name 读取为 UTF-8 std::string(不存在则为空)。
    auto labelName = [](const TDF_Label& lab) -> std::string {
        Handle(TDataStd_Name) nameAttr;
        if (!lab.IsNull() && lab.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
            const TCollection_ExtendedString& es = nameAttr->Get();
            TCollection_AsciiString utf8(es);
            // ExtendedString -> UTF-8 往返转换;OCCT 会原样保留 STEP 的转义。
            return std::string(utf8.ToCString());
        }
        return std::string();
    };

    // 取某个 solid 自身的体名:优先查 solidNames(从原始 STEP 实体提取,已实测命中率
    // 100%);该 map 缺失或未提供时返回空,由调用方回退到 product 标签名。
    auto solidOwnName = [&](const TopoDS_Shape& solid) -> std::string {
        if (solidNames && solidNames->IsBound(solid)) return solidNames->Find(solid);
        return std::string();
    };

    // 收集所有形状标签(自由形状 + 组件),然后索引其下找到的每个 solid。
    TDF_LabelSequence allShapes;
    shapeTool->GetShapes(allShapes);

    // GetShapes() 会同时返回装配体的节点标签与其组件子标签,二者常解析到
    // 同一个(同位置)TopoDS_Solid。若不去重,同一实体会被建成两个 unit,
    // 导致单元与拓扑记录重复,并破坏下游 AAG 邻接
    // (同一 TShape 进 compound 两遍 → 每条边被 4 个面共享 → 邻接表塌缩,
    //  孔/槽识别全部失效)。按 IsSame(TShape+Location)去重:合法的"同一
    //  零件实例化到不同位置"是 IsSame 不同的,会被正确保留为独立 unit。
    TopTools_MapOfShape seenSolids;
    // 片体(shell)去重 + 排除"属于某 solid 的 shell"。NX 片体在 STEP 里是不隶属任何
    // 实体的 open shell;solid 内部的 shell 不应再被当作独立片体 unit。先在 solid 扫描中
    // 把每个 solid 的 shell 全部登记进来(即便该 solid 因去重被跳过也登记),随后扫描
    // shell 时只对未登记的(=游离片体)建 unit。
    TopTools_MapOfShape seenShells;

    for (TDF_LabelSequence::Iterator it(allShapes); it.More(); it.Next()) {
        const TDF_Label& label = it.Value();
        TopoDS_Shape shape = shapeTool->GetShape(label);
        if (shape.IsNull()) continue;

        const std::string rawName = labelName(label);

        // (1) 实体:每个 solid 一个 unit。一个标签可能携带多个 solid(compound),故迭代。
        TopExp_Explorer solidExp(shape, TopAbs_SOLID);
        for (; solidExp.More(); solidExp.Next()) {
            const TopoDS_Solid solid = TopoDS::Solid(solidExp.Current());

            // 无论该 solid 是否新建 unit,都把它的 shell 标记为"已属于实体",
            // 避免后续 shell 扫描把实体内壳误判为游离片体。
            for (TopExp_Explorer se(solid, TopAbs_SHELL); se.More(); se.Next())
                seenShells.Add(se.Current());

            // 同一 located solid 已被其它标签建过 unit:跳过,避免实体翻倍。
            if (!seenSolids.Add(solid)) continue;

            // 体名优先取 solid 自身的实体名(从原始 STEP 提取,如 LP-01/DIE-06/体);
            // 取不到再回退到 product 标签名(常等于文件名)。
            std::string solidName = solidOwnName(solid);
            if (solidName.empty()) solidName = rawName;

            Unit unit;
            unit.unit_id = "U" + std::to_string(++unitCounter);
            unit.unit_name = solidName;
            unit.unit_kind = "solid";
            NameInfo ni = parseUnitName(solidName);
            unit.decoded_name = ni.decoded_name;
            unit.name_prefix = ni.name_prefix;
            unit.name_family = ni.name_family;
            unit.unit_role_hint = ni.unit_role_hint;
            unit.role_hint_source = solidName.empty() ? "fallback" : "name_rule";
            unit.role_hint_confidence = solidName.empty() ? 0.0 : ni.confidence;
            if (solidName.empty())
                unit.role_hint_diagnostics.push_back("unit_name_missing");

            if (control && control->cancelled()) return false;
            indexShape(solid, unit, out, faceCounter, edgeCounter, wireCounter, bounds);
            m_unitToSolid[unit.unit_id] = solid; // 保留该 solid 以便按 unit 导出 STEP
            m_unitToShape[unit.unit_id] = solid; // 通用形状(颜色继承/导出)
            out.units.push_back(std::move(unit));
        }

        // (2) 片体:不隶属任何 solid 的 open shell(NX 片体)。每个游离 shell 一个 unit,
        //     unit_kind="shell",无 solid(solidForUnit 返回 nullptr,消费方已有守卫)。
        for (TopExp_Explorer shellExp(shape, TopAbs_SHELL); shellExp.More(); shellExp.Next()) {
            const TopoDS_Shell shell = TopoDS::Shell(shellExp.Current());
            if (!seenShells.Add(shell)) continue; // 已属于某 solid 或已建过 unit

            std::string shellName = solidOwnName(shell); // 同一映射也含 shell 名
            if (shellName.empty()) shellName = rawName;

            Unit unit;
            unit.unit_id = "U" + std::to_string(++unitCounter);
            unit.unit_name = shellName;
            unit.unit_kind = "shell";
            NameInfo ni = parseUnitName(shellName);
            unit.decoded_name = ni.decoded_name;
            unit.name_prefix = ni.name_prefix;
            unit.name_family = ni.name_family;
            unit.unit_role_hint = ni.unit_role_hint;
            unit.role_hint_source = shellName.empty() ? "fallback" : "name_rule";
            unit.role_hint_confidence = shellName.empty() ? 0.0 : ni.confidence;
            if (shellName.empty())
                unit.role_hint_diagnostics.push_back("unit_name_missing");

            if (control && control->cancelled()) return false;
            indexShape(shell, unit, out, faceCounter, edgeCounter, wireCounter, bounds);
            m_unitToShape[unit.unit_id] = shell; // 片体无 solid,仅登记通用形状
            out.units.push_back(std::move(unit));
        }
    }

    classifyAndSort(out);
    return !out.units.empty();
}

// 缓存命中路径:仅重建 id<->shape 映射,跳过几何计算。遍历顺序必须与 build()/
// indexSolid()/addEdge() 完全一致(solid -> FACE -> WIRE -> WireExplorer edge ->
// 兜底 EDGE),才能复现相同的 F/E/W/U 编号。末尾按计数与 model 校验,不符则返回 false。
bool StepIndexer::restoreMaps(const Handle(TDocStd_Document)& doc, const Model& model) {
    m_idToFace.clear(); m_faceToId.Clear();
    m_idToEdge.clear(); m_edgeToId.Clear();
    m_idToWire.clear(); m_wireToId.Clear();
    m_unitToSolid.clear();
    m_unitToShape.clear();
    m_faceIdToUnitId.clear();
    if (doc.IsNull()) return false;

    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.IsEmpty()) return false;

    int unitCounter = 0, faceCounter = 0, edgeCounter = 0, wireCounter = 0;
    // edge 去重(shape -> edge_id),对应 build() 的 m_edgeToIndex 语义。
    NCollection_DataMap<TopoDS_Shape, std::string, TopTools_ShapeMapHasher> edgeSeen;

    // 首次遇到某条 edge 时分配 E 编号并绑定映射(与 addEdge 的去重一致)。
    auto bindEdge = [&](const TopoDS_Edge& e) {
        if (edgeSeen.IsBound(e)) return;
        std::string eid = "E" + std::to_string(++edgeCounter);
        edgeSeen.Bind(e, eid);
        m_idToEdge[eid] = e;
        if (!m_edgeToId.IsBound(e)) m_edgeToId.Bind(e, eid);
    };

    TDF_LabelSequence allShapes;
    shapeTool->GetShapes(allShapes);
    TopTools_MapOfShape seenSolids;
    TopTools_MapOfShape seenShells;

    // 复刻 indexShape 的遍历:FACE -> (新面 F++ + WIRE W++ + WireExplorer edge)
    //                                  -> 兜底 EDGE explorer。仅绑定映射,不算几何。
    auto bindShapeFaces = [&](const TopoDS_Shape& container) {
        for (TopExp_Explorer faceExp(container, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
            const TopoDS_Face f = TopoDS::Face(faceExp.Current());
            if (m_faceToId.IsBound(f)) continue; // 共享面已编号
            const std::string faceId = "F" + std::to_string(++faceCounter);
            m_idToFace[faceId] = f;
            m_faceToId.Bind(f, faceId);

            for (TopExp_Explorer wireExp(f, TopAbs_WIRE); wireExp.More(); wireExp.Next()) {
                const TopoDS_Wire w = TopoDS::Wire(wireExp.Current());
                const std::string wireId = "W" + std::to_string(++wireCounter);
                m_idToWire[wireId] = w;
                if (!m_wireToId.IsBound(w)) m_wireToId.Bind(w, wireId);
                for (BRepTools_WireExplorer we(w); we.More(); we.Next())
                    bindEdge(we.Current());
            }
            for (TopExp_Explorer ee(f, TopAbs_EDGE); ee.More(); ee.Next())
                bindEdge(TopoDS::Edge(ee.Current()));
        }
    };

    for (TDF_LabelSequence::Iterator it(allShapes); it.More(); it.Next()) {
        TopoDS_Shape shape = shapeTool->GetShape(it.Value());
        if (shape.IsNull()) continue;

        // (1) solid —— 与 build() 同序。
        for (TopExp_Explorer solidExp(shape, TopAbs_SOLID); solidExp.More(); solidExp.Next()) {
            const TopoDS_Solid solid = TopoDS::Solid(solidExp.Current());
            for (TopExp_Explorer se(solid, TopAbs_SHELL); se.More(); se.Next())
                seenShells.Add(se.Current());
            if (!seenSolids.Add(solid)) continue;
            const std::string unitId = "U" + std::to_string(++unitCounter);
            m_unitToSolid[unitId] = solid;
            m_unitToShape[unitId] = solid;
            bindShapeFaces(solid);
        }

        // (2) 游离片体(shell)—— 与 build() 同序。
        for (TopExp_Explorer shellExp(shape, TopAbs_SHELL); shellExp.More(); shellExp.Next()) {
            const TopoDS_Shell shell = TopoDS::Shell(shellExp.Current());
            if (!seenShells.Add(shell)) continue;
            const std::string unitId = "U" + std::to_string(++unitCounter);
            m_unitToShape[unitId] = shell;
            bindShapeFaces(shell);
        }
    }
    if ((size_t)unitCounter != model.units.size()) return false;
    if ((size_t)faceCounter != model.faces.size()) return false;
    if ((size_t)edgeCounter != model.edges.size()) return false;
    if ((size_t)wireCounter != model.wires.size()) return false;
    for (const Face& face : model.faces)
        m_faceIdToUnitId[face.face_id] = face.unit_id;
    return true;
}

void StepIndexer::indexShape(const TopoDS_Shape& shape, Unit& unit, Model& out,
                             int& faceCounter, int& edgeCounter, int& wireCounter,
                             const FaceBoundsMap& bounds) {
    // 面:每个 TopoDS_Face 一条记录 + AABB + surface_type。通过映射对共享面去重。
    // 对封闭实体(solid)与开放片体(shell)同样有效——仅依赖 FACE/WIRE/EDGE 遍历。
    TopExp_Explorer faceExp(shape, TopAbs_FACE);
    for (; faceExp.More(); faceExp.Next()) {
        const TopoDS_Face f = TopoDS::Face(faceExp.Current());
        if (m_faceToId.IsBound(f)) {
            // 共享面已在另一个 solid 下被索引;仍然记录该引用。
            const std::string faceId = m_faceToId.Find(f);
            unit.face_ids.push_back(faceId);
            m_faceIdToUnitId.try_emplace(faceId, unit.unit_id);
            continue;
        }
        Face rec;
        rec.face_id = "F" + std::to_string(++faceCounter);
        rec.unit_id = unit.unit_id;
        m_faceIdToUnitId[rec.face_id] = unit.unit_id; // 便于 solidForFaceId 查询
        rec.bbox = bounds.IsBound(f) ? bounds.Find(f) : computeBbox(f);

        // 表面类型/参数提取:畸形面上 OCCT 可能抛 Standard_Failure,
        // 兜底为 surface_type="other"/无法线,与 computeBbox/fillEdgeGeometry 一致,
        // 避免单个坏面 abort 整个 build()。
        try {
            BRepAdaptor_Surface surf(f, Standard_True);
            rec.surface_type = surfaceTypeName(surf.GetType());
            if (surf.GetType() == GeomAbs_Plane) {
                const gp_Dir n = surf.Plane().Axis().Direction();
                rec.has_normal = true;
                rec.normal = {n.X(), n.Y(), n.Z()};
            } else if (surf.GetType() == GeomAbs_Cylinder) {
                rec.radius = surf.Cylinder().Radius();
            } else if (surf.GetType() == GeomAbs_Cone) {
                rec.radius = surf.Cone().RefRadius();
            } else if (surf.GetType() == GeomAbs_Sphere) {
                rec.radius = surf.Sphere().Radius();
            }
        } catch (const Standard_Failure&) {
            rec.surface_type = "other";
        }
        if (rec.bbox.valid()) {
            auto c = rec.bbox.center();
            rec.has_center = true;
            rec.center = c;
        }
        // area 在首次查看属性(ResultPanel)时惰性计算。

        m_idToFace[rec.face_id] = f;
        m_faceToId.Bind(f, rec.face_id);
        unit.bbox.merge(rec.bbox);
        unit.face_ids.push_back(rec.face_id);

        // 索引该 face 的 wire 与 edge,并建立交叉引用(doc 07 §5-6)。
        // edge 在整个模型范围内按拓扑去重;wire 与 face 绑定不去重。
        // OuterWire 在退化面上可能抛 Standard_Failure,兜底为空 wire
        // (下游 !outerW.IsNull() 守卫会将全部 wire 视为 inner,不崩)。
        TopoDS_Wire outerW;
        try {
            outerW = BRepTools::OuterWire(f);
        } catch (const Standard_Failure&) {
            // outerW 保持为空
        }
        TopExp_Explorer wireExp(f, TopAbs_WIRE);
        for (; wireExp.More(); wireExp.Next()) {
            const TopoDS_Wire w = TopoDS::Wire(wireExp.Current());
            Wire wrec;
            wrec.wire_id = "W" + std::to_string(++wireCounter);
            wrec.unit_id = unit.unit_id;
            wrec.face_id = rec.face_id;
            wrec.wire_role = (!outerW.IsNull() && w.IsSame(outerW)) ? "outer" : "inner";
            wrec.is_closed = w.Closed() == Standard_True;
            wrec.bbox = computeBbox(w);
            rec.wire_ids.push_back(wrec.wire_id);

            // wire_id <-> shape(wire 不去重,直接绑定)。
            m_idToWire[wrec.wire_id] = w;
            if (!m_wireToId.IsBound(w)) m_wireToId.Bind(w, wrec.wire_id);

            // BRepTools_WireExplorer 给出有序且去重的 edge 序列。
            for (BRepTools_WireExplorer we(w); we.More(); we.Next()) {
                const TopoDS_Edge e = we.Current();
                addEdge(e, rec.face_id, wrec.wire_id, unit.unit_id, out, edgeCounter);
                const int idx = m_edgeToIndex.Find(e);
                wrec.edge_ids.push_back(out.edges[idx].edge_id);
                if (std::find(rec.edge_ids.begin(), rec.edge_ids.end(),
                              out.edges[idx].edge_id) == rec.edge_ids.end())
                    rec.edge_ids.push_back(out.edges[idx].edge_id);
            }
            out.wires.push_back(std::move(wrec));
        }
        // 安全兜底:WireExplorer 偶尔会跳过退化/接缝 edge,确保它们也进入 edge 目录。
        for (TopExp_Explorer ee(f, TopAbs_EDGE); ee.More(); ee.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ee.Current());
            addEdge(e, rec.face_id, std::string(), unit.unit_id, out, edgeCounter);
            const int idx = m_edgeToIndex.Find(e);
            if (std::find(rec.edge_ids.begin(), rec.edge_ids.end(),
                          out.edges[idx].edge_id) == rec.edge_ids.end())
                rec.edge_ids.push_back(out.edges[idx].edge_id);
        }

        out.faces.push_back(std::move(rec));
    }
}

void StepIndexer::addEdge(const TopoDS_Edge& edge, const std::string& faceId,
                          const std::string& wireId, const std::string& unitId,
                          Model& out, int& edgeCounter) {
    int idx;
    if (m_edgeToIndex.IsBound(edge)) {
        idx = m_edgeToIndex.Find(edge); // 共享边:复用已有记录,仅追加引用
    } else {
        Edge erec;
        erec.edge_id = "E" + std::to_string(++edgeCounter);
        erec.unit_id = unitId;
        fillEdgeGeometry(edge, erec);
        idx = (int)out.edges.size();
        // edge_id <-> shape(去重后唯一,仅首次绑定)。
        m_idToEdge[erec.edge_id] = edge;
        if (!m_edgeToId.IsBound(edge)) m_edgeToId.Bind(edge, erec.edge_id);
        out.edges.push_back(std::move(erec));
        m_edgeToIndex.Bind(edge, idx);
    }
    Edge& e = out.edges[idx];
    if (!faceId.empty() &&
        std::find(e.face_ids.begin(), e.face_ids.end(), faceId) == e.face_ids.end())
        e.face_ids.push_back(faceId);
    if (!wireId.empty() &&
        std::find(e.wire_ids.begin(), e.wire_ids.end(), wireId) == e.wire_ids.end())
        e.wire_ids.push_back(wireId);
}

namespace {

// 后缀解析使用的十进制基数。
constexpr int kDecimalBase = 10;
// 几何分类使用的长宽比阈值。
constexpr double kPlateAspectRatio = 4.0;
constexpr double kColumnAspectRatio = 4.0;
// 除零保护 epsilon。
constexpr double kDivZeroEpsilon = 1e-9;

// 形如 "F12" 的 ID 的数字后缀 -> 12,用于按数值排序(F2 < F10)—— doc 09 §17。
long suffixNum(const std::string& id) {
    size_t i = 0;
    while (i < id.size() && !std::isdigit((unsigned char)id[i])) ++i;
    if (i >= id.size()) return 0;
    return std::strtol(id.c_str() + i, nullptr, kDecimalBase);
}

// 按 AABB 比例对 unit 分类(doc 08 §6)。阈值为保守的默认值。
std::string geometryClass(const Bbox& b) {
    if (!b.valid()) return "unknown_unit";
    auto s = b.size();
    double sx = s[0], sy = s[1], sz = s[2];
    double maxXY = std::max(sx, sy);
    if (sz <= 0 || maxXY <= 0) return "unknown_unit";
    if (sz <= maxXY && maxXY / sz >= kPlateAspectRatio) return "plate_like_unit";
    if (sz >= maxXY && sz / std::max(kDivZeroEpsilon, maxXY) >= kColumnAspectRatio) return "column_like_unit";
    return "block_like_unit";
}

} // namespace

// 单元几何分类 + 名称/几何冲突诊断，并按数字后缀对 unit/face/edge/wire 排序。
void StepIndexer::classifyAndSort(Model& out) {
    // 几何分类 + 名称/几何冲突诊断(doc 08 §7 —— 仅记录,绝不覆盖)。
    for (auto& u : out.units) {
        u.unit_geometry_class = geometryClass(u.bbox);
        const bool nameSaysPlate = u.unit_role_hint == "plate_layer_candidate";
        const bool nameSaysColumn = u.unit_role_hint == "support_column_candidate";
        if (nameSaysPlate && u.unit_geometry_class == "column_like_unit") {
            u.role_hint_diagnostics.push_back("role_geometry_conflict");
            if (u.role_hint_source == "name_rule") u.role_hint_source = "conflict";
        } else if (nameSaysColumn && u.unit_geometry_class == "plate_like_unit") {
            u.role_hint_diagnostics.push_back("role_geometry_conflict");
            if (u.role_hint_source == "name_rule") u.role_hint_source = "conflict";
        }
    }

    auto byId = [](auto a, auto b) { return suffixNum(a) < suffixNum(b); };
    std::sort(out.units.begin(), out.units.end(),
              [&](const Unit& a, const Unit& b) { return byId(a.unit_id, b.unit_id); });
    std::sort(out.faces.begin(), out.faces.end(),
              [&](const Face& a, const Face& b) { return byId(a.face_id, b.face_id); });
    std::sort(out.edges.begin(), out.edges.end(),
              [&](const Edge& a, const Edge& b) { return byId(a.edge_id, b.edge_id); });
    std::sort(out.wires.begin(), out.wires.end(),
              [&](const Wire& a, const Wire& b) { return byId(a.wire_id, b.wire_id); });
}

std::optional<std::array<double, 6>> StepIndexer::exactBoundsForShape(
    const TopoDS_Shape& shape, const Model& model) const
{
    if (shape.IsNull()) return std::nullopt;
    Bbox bounds;
    if (shape.ShapeType() == TopAbs_COMPOUND) {
        for (TopoDS_Iterator child(shape); child.More(); child.Next()) {
            const auto box = exactBoundsForShape(child.Value(), model);
            if (!box) return std::nullopt;
            bounds.merge({(*box)[0], (*box)[1], (*box)[2], (*box)[3], (*box)[4], (*box)[5]});
        }
    } else {
        const auto unit = std::find_if(model.units.begin(), model.units.end(), [&](const Unit& item) {
            const auto* indexed = shapeForUnit(item.unit_id);
            return indexed && indexed->IsEqual(shape);
        });
        if (unit == model.units.end() || !unit->bbox.valid() || unit->face_ids.empty())
            return std::nullopt;
        // A failed face bound must not silently turn a partial unit box into an
        // exact target bound. The geometric path remains the fallback.
        std::size_t validFaces = 0;
        for (const auto& face : model.faces) {
            if (face.unit_id != unit->unit_id) continue;
            if (!face.bbox.valid()) return std::nullopt;
            ++validFaces;
        }
        if (validFaces != unit->face_ids.size()) return std::nullopt;
        bounds = unit->bbox;
    }
    if (!bounds.valid()) return std::nullopt;
    return std::array<double, 6>{bounds.xmin, bounds.ymin, bounds.zmin,
        bounds.xmax, bounds.ymax, bounds.zmax};
}

} // namespace oap
