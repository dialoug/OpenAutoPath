// 构造孔候选之间的父子、沉头阶段和共享通道关系，供最终孔特征组装使用。
#include "HoleToolpathRelations.h"

#include <algorithm>
#include <cmath>

#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>

#include "analysis/Model.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/StepIndexer.h"
#include "recognition/detail/CountersinkRelationMatcher.h"
#include "analysis/MoldSurfaceClassifier.h"
#include "recognition/AAG.h"

namespace oap::recognition::detail {

namespace {

long long faceSuffixNum(const std::string& id)
{
    std::size_t index = 0;
    while (index < id.size() && (id[index] < '0' || id[index] > '9'))
        ++index;
    long long value = 0;
    for (; index < id.size() && id[index] >= '0' && id[index] <= '9'; ++index)
        value = value * 10 + (id[index] - '0');
    return value;
}

std::string stableFaceId(int faceIndex,
                         const AAG& graph,
                         const StepIndexer& indexer)
{
    if (faceIndex < 1 || faceIndex > graph.faceMap.Extent())
        return {};
    return indexer.idForFace(TopoDS::Face(graph.faceMap(faceIndex)));
}

// 应用模具表面分类到孔记录并锁定入口侧。
void applyClassification(HoleRecognitionRecord& toolpath,
                         const MoldSurfaceClassification& surface,
                         HoleEntryResolution resolution)
{
    toolpath.entry = surface.entry;
    toolpath.bottom = surface.bottom;
    toolpath.entryResolution = resolution;
    toolpath.setupFace = surface.setupFace;
    toolpath.setupFaceEntryDistance = surface.entryDistance;
    toolpath.setupFaceOtherEndpointDistance = surface.otherEndpointDistance;
    toolpath.setupFaceDecision = surface.decision;
    toolpath.preserveEntrySide = true;
}

// 在候选的最终 bottom 端重新核对孔底拓扑：平底/锥底终止面须与该端
// 直接邻接且轴向距离在容差内，用于继承父阶段后复核孔底轮廓。
HoleBottomProfile terminalProfileAtFinalBottom(
    const HoleToolpathCandidate& candidate,
    const AAG& graph,
    double tolerance)
{
    gp_Vec axis(
        candidate.toolpath.entry,
        candidate.toolpath.bottom);
    if (axis.Magnitude() <= tolerance)
        return HoleBottomProfile::Unknown;
    axis.Normalize();

    const auto touchesFinalBottom =
        [&](const std::vector<int>& faceIds) {
            for (int faceId : faceIds) {
                if (faceId < 1 ||
                    faceId > graph.faceMap.Extent())
                    continue;
                bool adjacentToStage = false;
                for (int cylinderFaceId :
                     candidate.cylinderFaceIds) {
                    const auto adjacent =
                        graph.adjacency.find(cylinderFaceId);
                    if (adjacent != graph.adjacency.end() &&
                        std::find(
                            adjacent->second.begin(),
                            adjacent->second.end(), faceId) !=
                            adjacent->second.end()) {
                        adjacentToStage = true;
                        break;
                    }
                }
                if (!adjacentToStage)
                    continue;

                const TopoDS_Face face =
                    TopoDS::Face(graph.faceMap(faceId));
                for (TopExp_Explorer vertex(
                         face, TopAbs_VERTEX);
                     vertex.More(); vertex.Next()) {
                    const gp_Pnt point = BRep_Tool::Pnt(
                        TopoDS::Vertex(vertex.Current()));
                    const double axialDistance = std::abs(
                        gp_Vec(
                            candidate.toolpath.bottom,
                            point).Dot(axis));
                    if (axialDistance <= tolerance)
                        return true;
                }
            }
            return false;
        };

    if (touchesFinalBottom(
            candidate.terminalPlanarFaceIds))
        return HoleBottomProfile::Planar;
    if (touchesFinalBottom(
            candidate.terminalConicalFaceIds))
        return HoleBottomProfile::Conical;
    return HoleBottomProfile::Unknown;
}

} // namespace

std::vector<HoleRecognitionRecord> resolveHoleToolpathRelations(
    std::vector<HoleToolpathCandidate> candidates,
    const Bbox& bounds,
    const MoldCoordinateFrame& moldFrame,
    const AAG& graph,
    const StepIndexer& indexer)
{
    const double diagonal = bounds.valid()
        ? std::sqrt(std::pow(bounds.xmax - bounds.xmin, 2.0) +
                    std::pow(bounds.ymax - bounds.ymin, 2.0) +
                    std::pow(bounds.zmax - bounds.zmin, 2.0))
        : 0.0;
    const double tolerance = std::clamp(
        diagonal * 1.0e-6, 1.0e-4, 1.0e-2);
    // 把候选投影为只含稳定几何的匹配输入，避免关系解析依赖刀路构建细节。
    std::vector<CountersinkStageMatchInput> inputs;
    inputs.reserve(candidates.size());
    for (const HoleToolpathCandidate& candidate : candidates) {
        inputs.push_back(CountersinkStageMatchInput{
            candidate.toolpath.kind, candidate.toolpath.radius,
            candidate.toolpath.entry, candidate.toolpath.bottom,
            candidate.toolpath.setupFace, candidate.cylinderFaceIds,
            candidate.toolpath.bottomProfile,
            candidate.largestCountersinkStage,
            candidate.planarBoundaryFaceIds});
    }

    // 先求每个候选的沉头父级，再沿父链上溯到最大径段，把小径段折叠为
    // CoaxialStage 并继承父段装夹面。
    std::vector<std::size_t> ownerIndices(candidates.size(), candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index)
        ownerIndices[index] =
            findCountersinkStageOwner(inputs, index, graph, tolerance);

    for (std::size_t index = 0; index < candidates.size(); ++index) {
        HoleToolpathCandidate& candidate = candidates[index];
        std::size_t ownerIndex = ownerIndices[index];
        if (ownerIndex >= candidates.size())
            continue;
        for (std::size_t hop = 0; hop < candidates.size(); ++hop) {
            const std::size_t next = ownerIndices[ownerIndex];
            if (next >= candidates.size())
                break;
            ownerIndex = next;
        }
        const HoleToolpathCandidate& owner = candidates[ownerIndex];
        const MoldSurfaceClassification inherited =
            orientHoleSegmentToMoldFace(
                bounds, moldFrame, owner.toolpath.setupFace,
                candidate.toolpath.entry, candidate.toolpath.bottom);
        if (!inherited.valid)
            continue;
        candidate.toolpath.kind = FeatureKind::CountersinkHole;
        candidate.toolpath.stageRole =
            HoleStageRole::CountersinkSmallDiameter;
        if (candidates[ownerIndex].toolpath.stageRole ==
            HoleStageRole::Standard) {
            candidates[ownerIndex].toolpath.stageRole =
                HoleStageRole::CountersinkLargeDiameter;
        }
        applyClassification(
            candidate.toolpath, inherited,
            HoleEntryResolution::CountersinkLargestStage);
        candidate.countersinkOwnerRadius = owner.toolpath.radius;
        candidate.toolpath.parentRelation = HoleParentRelation::CoaxialStage;
        candidate.toolpath.parentCounterboreFaceId =
            owner.stableCylinderFaceId;
        candidate.toolpath.parentCounterboreRadius = owner.toolpath.radius;
        // 父阶段确定最终加工方向后重新核对 bottom 端拓扑。这样既能保留
        // 另一端真实存在的独占锥底，也不会把反向后的内部锥形过渡当作孔底。
        const HoleBottomProfile finalProfile =
            terminalProfileAtFinalBottom(
                candidate, graph, 10.0 * tolerance);
        if (finalProfile != HoleBottomProfile::Unknown ||
            candidate.toolpath.bottomProfile != HoleBottomProfile::Open) {
            candidate.toolpath.bottomProfile = finalProfile;
        }
        candidate.toolpath.sourceFaceIds.insert(
            candidate.toolpath.sourceFaceIds.end(),
            owner.toolpath.sourceFaceIds.begin(),
            owner.toolpath.sourceFaceIds.end());
    }

    // 平底子孔：继承最大沉头平底段的装夹面，父级记录平底/半径供去重优先。
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const CounterboreFloorOwner floorOwner =
            findCounterboreFloorOwner(inputs, index, graph, tolerance);
        if (!floorOwner.valid || floorOwner.ownerIndex >= candidates.size())
            continue;
        HoleToolpathCandidate& candidate = candidates[index];
        const HoleToolpathCandidate& owner = candidates[floorOwner.ownerIndex];
        const MoldSurfaceClassification inherited =
            orientHoleSegmentToMoldFace(
                bounds, moldFrame, owner.toolpath.setupFace,
                candidate.toolpath.entry, candidate.toolpath.bottom);
        if (!inherited.valid)
            continue;
        applyClassification(
            candidate.toolpath, inherited,
            HoleEntryResolution::CountersinkFloorOpening);
        candidate.toolpath.parentRelation =
            HoleParentRelation::CounterboreFloorChild;
        if (candidate.toolpath.stageRole !=
            HoleStageRole::CountersinkSmallDiameter) {
            candidate.toolpath.stageRole =
                HoleStageRole::CounterboreFloorChild;
        }
        candidate.toolpath.parentCounterboreFaceId =
            owner.stableCylinderFaceId;
        candidate.toolpath.parentCounterboreFloorFaceId =
            stableFaceId(floorOwner.floorFaceId, graph, indexer);
        candidate.toolpath.parentCounterboreRadius =
            owner.toolpath.radius;
        // 去重时保留已经继承父沉孔装夹面的候选，避免被同几何的普通孔覆盖。
        candidate.countersinkOwnerRadius = owner.toolpath.radius;
    }

    // 稳定去重：半径与端点（含反向）相同视为同一几何；已继承沉头装夹面的
    // 候选在合并时优先保留，面集合并后按 STEP id 稳定排序。
    std::vector<HoleToolpathCandidate> deduplicated;
    deduplicated.reserve(candidates.size());
    for (HoleToolpathCandidate& candidate : candidates) {
        auto duplicate = std::find_if(
            deduplicated.begin(), deduplicated.end(),
            [&](const HoleToolpathCandidate& existing) {
                if (std::abs(existing.toolpath.radius -
                             candidate.toolpath.radius) > tolerance)
                    return false;
                const bool sameDirection =
                    existing.toolpath.entry.Distance(candidate.toolpath.entry) <=
                        tolerance &&
                    existing.toolpath.bottom.Distance(candidate.toolpath.bottom) <=
                        tolerance;
                const bool reverseDirection =
                    existing.toolpath.entry.Distance(candidate.toolpath.bottom) <=
                        tolerance &&
                    existing.toolpath.bottom.Distance(candidate.toolpath.entry) <=
                        tolerance;
                return sameDirection || reverseDirection;
            });
        if (duplicate == deduplicated.end()) {
            deduplicated.push_back(std::move(candidate));
            continue;
        }

        std::vector<std::string> mergedFaces = duplicate->toolpath.sourceFaceIds;
        mergedFaces.insert(mergedFaces.end(),
                           candidate.toolpath.sourceFaceIds.begin(),
                           candidate.toolpath.sourceFaceIds.end());
        if (candidate.countersinkOwnerRadius >
            duplicate->countersinkOwnerRadius + tolerance)
            *duplicate = std::move(candidate);
        duplicate->toolpath.sourceFaceIds = std::move(mergedFaces);
        std::sort(duplicate->toolpath.sourceFaceIds.begin(),
                  duplicate->toolpath.sourceFaceIds.end(),
                  [](const std::string& first, const std::string& second) {
                      return faceSuffixNum(first) < faceSuffixNum(second);
                  });
        duplicate->toolpath.sourceFaceIds.erase(
            std::unique(duplicate->toolpath.sourceFaceIds.begin(),
                        duplicate->toolpath.sourceFaceIds.end()),
            duplicate->toolpath.sourceFaceIds.end());
    }

    std::vector<HoleRecognitionRecord> paths;
    paths.reserve(deduplicated.size());
    for (HoleToolpathCandidate& candidate : deduplicated)
        paths.push_back(std::move(candidate.toolpath));
    return paths;
}

} // namespace oap::recognition::detail
