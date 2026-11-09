// ToolpathRequestBuilder 实现（oap_app_workflow_cam 的 detail 私有目录）。
// 无窗口的桌面刀路请求装配辅助函数（孔障碍物上下文、槽/腔装夹上下文），
// 见 detail/ToolpathRequestBuilder.h。
#include "app/workflows/shared/detail/ToolpathRequestBuilder.h"
#include "app/workflows/profile/ProfileSetupContextAssigner.h"

#include <algorithm>

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>

#include "analysis/MoldSurfaceClassifier.h"

namespace oap::app {

std::vector<toolpath::ProfileRegion> orderProfileRegionsBySetup(
    const std::vector<toolpath::ProfileRegion>& regions,
    const MoldCoordinateFrame& moldFrame)
{
    std::vector<toolpath::ProfileRegion> ordered = regions;
    std::stable_sort(
        ordered.begin(), ordered.end(),
        [&](const toolpath::ProfileRegion& left,
            const toolpath::ProfileRegion& right) {
            const MoldSetupFace leftFace = classifyMoldSetupFace(
                moldFrame, gp_Vec(left.openDir));
            const MoldSetupFace rightFace = classifyMoldSetupFace(
                moldFrame, gp_Vec(right.openDir));
            return static_cast<int>(leftFace) <
                static_cast<int>(rightFace);
        });
    return ordered;
}

// 装配碰撞场景:所有障碍体合并为 compound 并逐一加入碰撞场景,
// 便于后续孔刀路做整体避让计算。
HoleRapidPlanningContextBuildResult buildHoleRapidPlanningContext(
    const std::vector<HoleObstacleSource>& obstacles,
    const MoldCoordinateFrame* moldFrame)
{
    HoleRapidPlanningContextBuildResult result;
    result.context.moldFrame = moldFrame;

    TopoDS_Compound obstacleShape;
    BRep_Builder builder;
    builder.MakeCompound(obstacleShape);
    for (const HoleObstacleSource& obstacle : obstacles) {
        if (obstacle.shape.IsNull())
            continue;
        builder.Add(obstacleShape, obstacle.shape);
        result.context.collisionScene.addBody(
            obstacle.stableId,
            toolpath::CollisionBodyRole::Workpiece,
            obstacle.shape);
        ++result.obstacleBodyCount;
    }
    if (result.obstacleBodyCount > 0)
        result.context.obstacleShape = obstacleShape;
    return result;
}

// 槽腔工序通过 sourceRegionIndex 找到对应区域,再按开方向的
// 几何分类(基于模具坐标系)写入装夹面与序号。
void assignPocketSetupContexts(
    toolpath::PocketToolpathBatch& batch,
    const std::vector<toolpath::PocketRegion>& regions,
    const MoldCoordinateFrame& moldFrame)
{
    for (toolpath::PocketToolpathOperation& operation :
         batch.operations) {
        const int regionIndex = operation.sourceRegionIndex;
        if (regionIndex < 0 ||
            regionIndex >= static_cast<int>(regions.size())) {
            continue;
        }
        operation.setupFace = classifyMoldSetupFace(
            moldFrame,
            gp_Vec(regions[static_cast<std::size_t>(regionIndex)].openDir));
        operation.setupIndex = static_cast<int>(operation.setupFace);
    }
}

void assignProfileSetupContexts(
    toolpath::ProfileToolpathBatch& batch,
    const std::vector<toolpath::ProfileRegion>& regions,
    const MoldCoordinateFrame& moldFrame)
{
    for (toolpath::ProfileToolpathOperation& operation : batch.operations) {
        const int regionIndex = operation.sourceRegionIndex;
        if (regionIndex < 0 ||
            regionIndex >= static_cast<int>(regions.size())) {
            continue;
        }
        operation.setupFace = classifyMoldSetupFace(
            moldFrame,
            gp_Vec(regions[static_cast<std::size_t>(regionIndex)].openDir));
        operation.setupIndex = operation.setupFace == MoldSetupFace::Unresolved
            ? 0 : static_cast<int>(operation.setupFace);
    }
}

} // namespace oap::app
