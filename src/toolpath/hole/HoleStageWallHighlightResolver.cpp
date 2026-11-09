// HoleStageWallHighlightResolver —— 按孔段父子关系解析下刀路径需要高亮的孔壁。
// 属于 oap_toolpath_hole 模块的内部实现：小径钻孔段/沉孔平底子孔在选中时
// 会把父级沉孔段的壁面一并纳入高亮，帮助用户确认下刀时可能干涉的位置。
#include "toolpath/hole/HoleStageWallHighlightResolver.h"

#include <algorithm>

namespace oap::toolpath {
namespace {

void appendUniqueFaceId(
    std::vector<std::string>& faceIds,
    const std::string& faceId)
{
    if (faceId.empty() ||
        std::find(faceIds.begin(), faceIds.end(), faceId) != faceIds.end()) {
        return;
    }
    faceIds.push_back(faceId);
}

void appendStageWalls(
    std::vector<std::string>& faceIds,
    const HoleMachiningInput& hole)
{
    for (const std::string& faceId : hole.stageWallFaceIds)
        appendUniqueFaceId(faceIds, faceId);
}

void appendResolvedStageWalls(
    std::vector<std::string>& faceIds,
    const std::vector<HoleMachiningInput>& holes,
    std::size_t selectedHoleIndex)
{
    if (selectedHoleIndex >= holes.size())
        return;

    const HoleMachiningInput& selected = holes[selectedHoleIndex];
    appendStageWalls(faceIds, selected);

    // 只有作为沉孔内段的角色（小径钻孔段/沉孔平底子孔）才需要回溯父级壁面；
    // 标准段和大径扩大段自身就是最外侧，无需再向上合并。
    const HoleStageRole role = resolvedHoleStageRole(selected);
    if (role != HoleStageRole::CountersinkSmallDiameter &&
        role != HoleStageRole::CounterboreFloorChild) {
        return;
    }

    appendUniqueFaceId(faceIds, selected.parentCounterboreFaceId);
    const auto parent = std::find_if(
        holes.begin(), holes.end(),
        [&selected](const HoleMachiningInput& candidate) {
            return std::find(
                       candidate.stageWallFaceIds.begin(),
                       candidate.stageWallFaceIds.end(),
                       selected.parentCounterboreFaceId) !=
                   candidate.stageWallFaceIds.end();
        });
    if (parent != holes.end())
        appendStageWalls(faceIds, *parent);
}

} // namespace

std::vector<std::string> resolveHoleStageWallHighlightFaceIds(
    const std::vector<HoleMachiningInput>& holes,
    std::size_t selectedHoleIndex)
{
    std::vector<std::string> faceIds;
    appendResolvedStageWalls(faceIds, holes, selectedHoleIndex);
    return faceIds;
}

std::vector<std::string> resolveHoleStageWallHighlightFaceIds(
    const std::vector<HoleMachiningInput>& holes,
    const std::vector<std::size_t>& selectedHoleIndices)
{
    std::vector<std::string> faceIds;
    for (const std::size_t index : selectedHoleIndices)
        appendResolvedStageWalls(faceIds, holes, index);
    return faceIds;
}

} // namespace oap::toolpath
