// PocketRegionBuilder —— 将识别实例转换为 PocketClearing 使用的归一化几何结构。
// 它不持有模型数据：OCCT 形状以句柄方式复制，识别结果保持不变。
#pragma once

#include <optional>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include "toolpath/pocket/geometry/OcctAreaAdapter.h"
#include "toolpath/core/ToolpathDiagnostics.h"
#include "analysis/FeatureTypes.h"
#include "analysis/Model.h"
#include "analysis/StepIndexer.h"

namespace oap::toolpath {

// 区域构建选项：离散公差、绕向归一化开关和局部重算时的底面偏好。
// 由 PocketClearing 调用方（PocketToolpathService 等）一次性配置后传入 build()。
struct PocketRegionBuildOptions {
    double discretizationDeflection = 0.01;
    double tolerance = 1.0e-7;
    bool normalizeOrientation = true; // 在区域坐标系中，外环逆时针、岛环顺时针。
    // 局部重算（例如腔/槽互转）时沿用旧操作已经确认的底面，避免重新按面积
    // 猜测后误选上表面，导致刀路坐标系翻转或向实体外投射。
    std::string preferredFloorFaceId;
};

// 归一化后的清腔加工区域：底面、开口方向、局部坐标系、二维边界/岛环
// 与逐层截面实体。由 PocketRegionBuilder 构建，后续 PocketClearing /
// PocketLayerSlicer 只读消费，不修改其中的 OCCT 形状。
struct PocketRegion {
    FeatureKind kind = FeatureKind::Pocket;
    std::vector<std::string> sourceFaceIds;

    std::string floorFaceId;
    TopoDS_Face floorFace;
    gp_Pln floorPlane;
    gp_Dir openDir; // 局部 +Z，从底面指向特征开口。
    gp_Ax3 frame;   // 局部 XY 位于底面上，所有 Loop2d 均使用该坐标系。
    double floorArea = 0.0;
    double depth = 0.0;

    // 用于逐层截面的已识别面组只读副本。
    // 它属于刀路区域，不会修改识别结果。
    TopoDS_Compound sectionShape;
    // 所属最终工件实体。逐层切片时从候选清腔区域中扣除这些真实材料，
    // 使未进入特征面组的凸台、圆角凸起和残留实体仍会形成禁入岛。
    TopoDS_Shape protectedMaterialShape;
    double sectionTolerance = 1.0e-7;
    double sectionDeflection = 0.01;
    int sectionFaceCount = 0;
    bool filteredDisconnectedFaces = false;
    // 与通槽/通结构联合的腔允许截面在实体外部出口处开放。切片器会
    // 对该出口建立虚拟封口供二维区域运算使用，普通区域仍严格拒绝
    // 开放轮廓。
    bool allowOpenSectionContours = false;
    int openBoundaryFaceCount = 0;
    // 主外表面内环追踪结构没有传统“腔底面”；根面内环位于
    // depth 偏移的开口层，offset=0 位于沿主外表面反法向得到的最深层。
    // 该标记强制所有图案使用真实逐层截面。
    bool useSectionBoundaryAtEveryDepth = false;

    Loop2d boundary;
    std::vector<Loop2d> islands;
};

// 由主外表面某条内环追踪得到的独立结构。rootWireId 必须指向
// rootFaceId 上的真实内环；sourceFaceIds 只包含从该内环走出的结构面，
// 不包含主外表面根面。
struct RootInnerWirePocketSource {
    FeatureKind kind = FeatureKind::Pocket;
    std::vector<std::string> sourceFaceIds;
    std::string rootFaceId;
    std::string rootWireId;
    bool allowOpenSectionContours = false;
    int openBoundaryFaceCount = 0;
};

// 一个被 build() 跳过的槽/腔实例及其原因(如底面非平面、无可用外边界)。
// 供 UI 把静默丢弃变为可见的诊断,便于用户理解为何某些特征未生成刀路。
struct SkippedRegion {
    FeatureKind kind = FeatureKind::Pocket;
    std::vector<std::string> sourceFaceIds;
    std::string reason;
    ToolpathDiagnostic diagnostic;
};

// 槽/腔识别结果 -> 归一化 PocketRegion 的转换器。不持有模型数据，
// 每次 build()/buildInstance() 是独立的纯函数调用；输出结构拥有
// OCCT 形状句柄副本，与识别结果（Model/StepIndexer）互不共享可变状态。
class PocketRegionBuilder {
public:
    // 只构建 Slot/Pocket 实例。不支持或格式错误的实例会被忽略并记录到 skipped，
    // 不会中止整批识别结果的处理。
    // skipped 非空时,逐个记录被跳过的槽/腔实例及原因;为 nullptr 时行为不变。
    std::vector<PocketRegion> build(const std::vector<FeatureInstance>& instances,
                                    const StepIndexer& indexer,
                                    const PocketRegionBuildOptions& options = {},
                                    std::vector<SkippedRegion>* skipped = nullptr) const;

    std::optional<PocketRegion> buildInstance(
        const FeatureInstance& instance,
        const StepIndexer& indexer,
        const PocketRegionBuildOptions& options = {},
        std::string* error = nullptr) const;

    // 新代码使用结构化诊断；上面的字符串重载仅为兼容旧调用方保留。
    std::optional<PocketRegion> buildInstanceDetailed(
        const FeatureInstance& instance,
        const StepIndexer& indexer,
        const PocketRegionBuildOptions& options = {},
        ToolpathDiagnostic* diagnostic = nullptr) const;

    // 为主外表面内环追踪结构建立清腔区域。根面内环作为开口层边界，
    // 下刀方向固定为根面有效法向的反向；其余各层只使用
    // sourceFaceIds 的真实截面。
    std::vector<PocketRegion> buildRootInnerWireRegions(
        const std::vector<RootInnerWirePocketSource>& sources,
        const StepIndexer& indexer,
        const PocketRegionBuildOptions& options = {},
        std::vector<SkippedRegion>* skipped = nullptr) const;

    std::optional<PocketRegion> buildRootInnerWireRegionDetailed(
        const RootInnerWirePocketSource& source,
        const StepIndexer& indexer,
        const PocketRegionBuildOptions& options = {},
        ToolpathDiagnostic* diagnostic = nullptr) const;

    // 人工补腔入口:seedFaceId 必须是平面底面。返回底面以及与其边界边直接共享
    // 拓扑的同 Unit 面(侧壁/岛壁),供 UI 建立一个完整的人工槽/腔实例。
    // 不调用也不修改 FeatureRecognizer；独立拓扑或无邻接面时严格失败。
    static std::vector<std::string> expandManualFloorSeed(
        const std::string& seedFaceId,
        const Model& model,
        std::string* error = nullptr)
    {
        ToolpathDiagnostic diagnostic;
        std::vector<std::string> expanded = expandManualFloorSeedDetailed(
            seedFaceId, model, &diagnostic);
        if (error)
            *error = diagnostic.message;
        return expanded;
    }

    static std::vector<std::string> expandManualFloorSeedDetailed(
        const std::string& seedFaceId,
        const Model& model,
        ToolpathDiagnostic* diagnostic = nullptr)
    {
        auto fail = [diagnostic](ToolpathDiagnosticCode code,
                                 const char* message) {
            if (diagnostic) {
                diagnostic->code = code;
                diagnostic->severity = ToolpathDiagnosticSeverity::Error;
                diagnostic->message = message;
            }
            return std::vector<std::string>{};
        };
        const Face* seed = nullptr;
        for (const Face& face : model.faces) {
            if (face.face_id == seedFaceId) {
                seed = &face;
                break;
            }
        }
        if (!seed)
            return fail(ToolpathDiagnosticCode::ManualFloorNotFound,
                        "manual floor seed face was not found");
        if (seed->surface_type != "plane")
            return fail(ToolpathDiagnosticCode::ManualFloorNotPlanar,
                        "manual floor seed is not planar");

        std::unordered_map<std::string, const Edge*> edgesById;
        edgesById.reserve(model.edges.size());
        for (const Edge& edge : model.edges)
            edgesById.emplace(edge.edge_id, &edge);

        std::vector<std::string> expanded{seedFaceId};
        for (const std::string& edgeId : seed->edge_ids) {
            const auto edgeIt = edgesById.find(edgeId);
            if (edgeIt == edgesById.end()) continue;
            for (const std::string& neighborId : edgeIt->second->face_ids) {
                if (neighborId == seedFaceId) continue;
                const auto faceIt = std::find_if(
                    model.faces.begin(), model.faces.end(),
                    [&](const Face& face) { return face.face_id == neighborId; });
                if (faceIt != model.faces.end() && faceIt->unit_id == seed->unit_id)
                    expanded.push_back(neighborId);
            }
        }
        std::sort(expanded.begin(), expanded.end());
        expanded.erase(std::unique(expanded.begin(), expanded.end()), expanded.end());
        if (expanded.size() <= 1)
            return fail(ToolpathDiagnosticCode::ManualFloorDisconnected,
                        "manual floor seed has no topologically adjacent faces");
        if (diagnostic)
            *diagnostic = {};
        return expanded;
    }
};

} // namespace oap::toolpath
