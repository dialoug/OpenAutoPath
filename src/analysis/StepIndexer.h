// StepIndexer — 将 XDE/XCAF 文档遍历构建为分析用 Model(docs/06 §4-5, doc 02 oap_geometry)。
// 同时维护一份 TopoDS_Face <-> face_id 映射,使 3D 视图能将拾取关联到记录。
#pragma once

#include "Model.h"

#include <map>
#include <functional>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>
#include <string>

#include <TDocStd_Document.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shape.hxx>
#include <NCollection_DataMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>

class STEPCAFControl_Reader; // 仅按引用传递,前向声明即可(实现见 .cpp)

namespace oap {

// Executor 必须同步完成并 join 所有任务；取消查询可从计算线程调用。
// 线程预算由调用层注入，analysis 不拥有线程池或依赖 CAM 调度器。
struct StepIndexBuildControl {
    std::function<void(std::size_t, const std::function<void(std::size_t)>&)> executeRange;
    std::function<bool()> isCancellationRequested;
    // 回调按完成数串行调用，但可能运行于任意计算线程。
    std::function<void(std::size_t, std::size_t)> reportProgress;
    bool cancelled() const { return isCancellationRequested && isCancellationRequested(); }
};

struct StepIndexBuildStatistics {
    std::int64_t boundsPreparationMillis = 0;
    std::int64_t faceBoundsMillis = 0;
    std::size_t faceCount = 0;
    std::size_t isolatedFaceCount = 0;
};

// 将已加载的 XCAF 文档索引为分析 Model:遍历 solid / shell 生成 U/F/E/W 记录,
// 计算 AABB 与几何事实,并维护 face/edge/wire/unit <-> TopoDS 形状的双向映射。
// 生命周期:由分析管线创建并持有,供查看器拾取同步与场景展示使用;
// 映射仅在 build() / restoreMaps() 成功后有效。
class StepIndexer {
public:
    // 实体形状 -> 名称映射(按拓扑哈希)。OCCT 的 STEPCAF 不会把 solid 的实体名
    // (如 LP-01 / DIE-06 / 体)搬进 XCAF,需直接从原始 STEP 实体提取——见
    // solidNamesFromReader()。键为 transfer 出的 TopoDS_Solid。
    using SolidNameMap = NCollection_DataMap<TopoDS_Shape, std::string, TopTools_ShapeMapHasher>;

    // 从刚完成 Transfer 的 STEPCAF reader 提取“solid -> 实体名”映射。
    // 通过工作会话的 TransientProcess 把带名的 representation_item 映射回其 TopoDS_Solid。
    static SolidNameMap solidNamesFromReader(STEPCAFControl_Reader& reader);

    // 从已加载的 XCAF 文档构建 unit/face/edge/wire 索引(含 AABB)。
    // solidNames 非空时,优先用它作为 unit 体名来源(回退顺序见实现)。
    // 无可分析形状或取消时返回 false，不发布部分索引/Model。
    bool build(const Handle(TDocStd_Document)& doc, Model& out,
               const SolidNameMap* solidNames = nullptr,
               const StepIndexBuildControl* control = nullptr,
               StepIndexBuildStatistics* statistics = nullptr);

    // 缓存命中路径:几何来自缓存的 .xbf 文档、分析 Model 来自缓存的 model.bin,
    // 此处仅按与 build() 完全一致的遍历顺序重建 id<->shape 映射(F/E/W/unit->solid),
    // 跳过所有几何计算。要求 doc 与生成该 Model 时是同一份几何(同序遍历 -> 同序 ID)。
    // 安全校验:重建出的 face/edge/wire/unit 计数与 model 不符时返回 false,
    // 调用方据此丢弃缓存、回退到完整 build()。
    bool restoreMaps(const Handle(TDocStd_Document)& doc, const Model& model);

    // build() 期间填充的查找辅助方法 —— 供查看器进行拾取 <-> 记录同步。
    const TopoDS_Face* faceForId(const std::string& faceId) const;
    std::string idForFace(const TopoDS_Face& face) const;

    // edge / wire 的 id <-> shape 查找(与 face 对称),供 edge/wire 选择模式与显示隐藏使用。
    const TopoDS_Edge* edgeForId(const std::string& edgeId) const;
    std::string idForEdge(const TopoDS_Edge& edge) const;
    const TopoDS_Wire* wireForId(const std::string& wireId) const;
    std::string idForWire(const TopoDS_Wire& wire) const;

    // 某个 unit_id 对应的实体(仅当该 unit 是 solid 时存在)。片体(shell)unit
    // 没有 solid,返回 nullptr。颜色继承/导出请用 shapeForUnit(对 solid/shell 通用)。
    const TopoDS_Solid* solidForUnit(const std::string& unitId) const;

    // 返回某个面的所属封闭实体；面属于 shell 或未知 unit 时返回 nullptr。
    const TopoDS_Solid* solidForFaceId(const std::string& faceId) const;

    // 某个 unit_id 对应的几何形状(solid 或 shell,每个 unit 一个)。若 id 未知返回
    // nullptr。供 SceneController 颜色继承——对片体同样有效。
    const TopoDS_Shape* shapeForUnit(const std::string& unitId) const;

    // Exact indexed bounds for a unit or a compound of complete indexed units.
    // Unknown/modified geometry returns nullopt, requiring a geometric fallback.
    std::optional<std::array<double, 6>> exactBoundsForShape(
        const TopoDS_Shape& shape, const Model& model) const;

    // 按 face_id 顺序遍历所有已索引的面。
    // 回调签名:void(const std::string& face_id, const TopoDS_Face&)
    template<typename Fn>
    void forEachFace(Fn&& fn) const {
        for (const auto& kv : m_idToFace) fn(kv.first, kv.second);
    }

    // 按 edge_id 顺序遍历所有去重后的唯一 edge(供 3D 视图构建边线叠加层)。
    // 回调签名:void(const std::string& edge_id, const TopoDS_Edge&)
    template<typename Fn>
    void forEachEdge(Fn&& fn) const {
        for (const auto& kv : m_idToEdge) fn(kv.first, kv.second);
    }

private:
    using FaceBoundsMap = NCollection_DataMap<TopoDS_Shape, Bbox, TopTools_ShapeMapHasher>;
    bool buildInPlace(const Handle(TDocStd_Document)& doc, Model& out,
                      const SolidNameMap* solidNames,
                      const StepIndexBuildControl* control,
                      StepIndexBuildStatistics* statistics);
    // 将一个 shape(solid 或 shell)索引到 unit 中:faces/edges/wires + AABB
    // (合并进 unit.bbox)。仅做 FACE/WIRE/EDGE 遍历,对封闭实体与开放片体均成立。
    void indexShape(const TopoDS_Shape& shape, Unit& unit, Model& out,
                    int& faceCounter, int& edgeCounter, int& wireCounter,
                    const FaceBoundsMap& bounds);

    // 将一条 edge 登记到 edge 目录(已去重)并补充 face/wire/unit 交叉引用。
    // 首次遇到时计算几何事实并追加到 out.edges;重复出现时仅追加引用。
    void addEdge(const TopoDS_Edge& edge, const std::string& faceId,
                 const std::string& wireId, const std::string& unitId,
                 Model& out, int& edgeCounter);

    // 根据 AABB 比例推导 unit_geometry_class,并标记名称/几何冲突;
    // 然后按各自数字后缀的键对每个 vector 做稳定排序(doc 09 §17)。
    void classifyAndSort(Model& out);

    // face_id <-> shape。该 shape 映射使用 OCCT 的拓扑哈希,以便拾取到的面能被解析。
    std::map<std::string, TopoDS_Face> m_idToFace;
    NCollection_DataMap<TopoDS_Shape, std::string, TopTools_ShapeMapHasher> m_faceToId;

    // edge_id <-> shape(去重后的唯一 edge)。供 edge 选择模式拾取与显示隐藏使用。
    std::map<std::string, TopoDS_Edge> m_idToEdge;
    NCollection_DataMap<TopoDS_Shape, std::string, TopTools_ShapeMapHasher> m_edgeToId;

    // wire_id <-> shape(wire 与 face 绑定,不跨 face 去重)。
    std::map<std::string, TopoDS_Wire> m_idToWire;
    NCollection_DataMap<TopoDS_Shape, std::string, TopTools_ShapeMapHasher> m_wireToId;

    // edge 去重:同一条 edge 被多个 face/solid 共享时只生成一条记录。
    // 值为 Model.edges 中的下标,便于回填 face_ids / wire_ids 交叉引用。
    NCollection_DataMap<TopoDS_Shape, int, TopTools_ShapeMapHasher> m_edgeToIndex;

    // unit_id -> 构建该 unit 的 solid(仅 solid unit 存在;片体不在此表)。
    // 纯查找(仅 find/insert),不依赖有序迭代,故用 unordered_map。
    std::unordered_map<std::string, TopoDS_Solid> m_unitToSolid;

    // unit_id -> 构建该 unit 的 shape(solid 或 shell),使每个 unit——含片体——
    // 都能被重新导出为各自的 STEP 并继承部件颜色。
    std::unordered_map<std::string, TopoDS_Shape> m_unitToShape;

    std::unordered_map<std::string, std::string> m_faceIdToUnitId;
};

} // namespace oap
