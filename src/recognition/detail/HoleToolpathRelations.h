// 解析孔段父子关系并执行跨识别结果的稳定去重。
#pragma once

#include <string>
#include <vector>

#include "analysis/FeatureTypes.h"

class AAG;

namespace oap {
struct Bbox;
struct MoldCoordinateFrame;
class StepIndexer;
}

namespace oap::recognition::detail {

// 孔识别内部工作记录。加工决策只用于稳定关系解析与几何定向，不越过
// recognition 公共边界；公开结果会投影为 RecognizedHoleGeometry。
struct HoleRecognitionRecord {
    FeatureKind kind = FeatureKind::BlindHole;
    gp_Pnt entry;
    gp_Pnt bottom;
    double radius = 0.0;
    HoleBottomProfile bottomProfile = HoleBottomProfile::Unknown;
    HoleEntryResolution entryResolution = HoleEntryResolution::Unresolved;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    double setupFaceEntryDistance = -1.0;
    double setupFaceOtherEndpointDistance = -1.0;
    MoldSetupFaceDecision setupFaceDecision = MoldSetupFaceDecision::None;
    bool preserveEntrySide = false;
    std::vector<std::string> sourceFaceIds;
    std::vector<std::string> stageWallFaceIds;
    HoleStageRole stageRole = HoleStageRole::Standard;
    HoleParentRelation parentRelation = HoleParentRelation::None;
    std::string parentCounterboreFaceId;
    std::string parentCounterboreFloorFaceId;
    double parentCounterboreRadius = 0.0;
};

// 一个孔段候选：加工记录 + 父级沉孔半径、壁面/平底/终止面索引，
// 供关系解析阶段的几何去重与父子匹配使用。
struct HoleToolpathCandidate {
    HoleRecognitionRecord toolpath;
    double countersinkOwnerRadius = 0.0;
    std::vector<int> cylinderFaceIds;
    std::vector<int> planarBoundaryFaceIds;
    std::vector<int> terminalPlanarFaceIds;
    std::vector<int> terminalConicalFaceIds;
    std::string stableCylinderFaceId;
    bool largestCountersinkStage = false;
};

// 模块私有实现细节，不对外暴露；由 HoleToolpathBuilder 在构建加工几何时调用。
// 解析孔段父子关系（沉头大/小径、平底子孔）并跨识别结果稳定去重；
// candidates 按引用被消耗（返回其 toolpath），bounds/moldFrame 用于定向
// 与容差，graph/indexer 提供拓扑与 STEP id。
std::vector<HoleRecognitionRecord> resolveHoleToolpathRelations(
    std::vector<HoleToolpathCandidate> candidates,
    const Bbox& bounds,
    const MoldCoordinateFrame& moldFrame,
    const AAG& graph,
    const StepIndexer& indexer);

} // namespace oap::recognition::detail
