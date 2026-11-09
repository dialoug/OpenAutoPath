// 把识别几何与装夹决策合并为刀路输入;根据入口/装夹是否已解析
// 标记 mappingState,供后续选刀规划器决定是否纳入批次。
#include "tooling/adapter/hole/HoleMachiningInputMapper.h"

#include "analysis/MoldSurfaceClassifier.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace oap::tooling {

toolpath::HoleMachiningInput HoleMachiningInputMapper::map(
    RecognizedHoleGeometry geometry,
    toolpath::HoleMachiningDecision decision) const
{
    toolpath::HoleMachiningInput input;
    static_cast<RecognizedHoleGeometry&>(input) = std::move(geometry);
    static_cast<toolpath::HoleMachiningDecision&>(input) = decision;
    if (decision.entryResolution == HoleEntryResolution::Unresolved) {
        input.mappingState =
            toolpath::HoleMachiningMappingState::UnresolvedEntry;
    } else if (decision.setupFace == MoldSetupFace::Unresolved) {
        input.mappingState =
            toolpath::HoleMachiningMappingState::UnresolvedSetup;
    } else {
        input.mappingState = toolpath::HoleMachiningMappingState::Ready;
    }
    return input;
}

// 自动解析入口与装夹面:优先用模具表面分类(考虑毛坯包络面投影),
// 分类失败但孔段有长度时退回按轴线方向分类装夹面。
toolpath::HoleMachiningInput HoleMachiningInputMapper::resolve(
    RecognizedHoleGeometry geometry,
    const Bbox& bounds,
    const MoldCoordinateFrame& moldFrame) const
{
    toolpath::HoleMachiningDecision decision;
    const gp_Pnt topologyOpening = geometry.entry;
    const MoldSurfaceClassification surface =
        classifyHoleSegmentByMoldSurface(
            bounds, moldFrame, geometry.entry, geometry.bottom,
            &topologyOpening);
    if (surface.valid) {
        geometry.entry = surface.entry;
        geometry.bottom = surface.bottom;
        decision.entryResolution = HoleEntryResolution::TopologyOpening;
        decision.setupFace = surface.setupFace;
        decision.preserveEntrySide = true;
        decision.setupFaceEntryDistance = surface.entryDistance;
        decision.setupFaceOtherEndpointDistance =
            surface.otherEndpointDistance;
        decision.setupFaceDecision = surface.decision;
    } else if (geometry.entry.Distance(geometry.bottom) > 0.0) {
        decision.entryResolution = HoleEntryResolution::TopologyOpening;
        decision.setupFace = classifyMoldSetupFace(
            moldFrame, gp_Vec(geometry.bottom, geometry.entry));
        decision.preserveEntrySide =
            decision.setupFace != MoldSetupFace::Unresolved;
    }
    return map(std::move(geometry), decision);
}

std::vector<toolpath::HoleMachiningInput>
HoleMachiningInputMapper::resolve(
    const std::vector<RecognizedHoleGeometry>& geometries,
    const Bbox& bounds,
    const MoldCoordinateFrame& moldFrame) const
{
    std::vector<toolpath::HoleMachiningInput> inputs;
    inputs.reserve(geometries.size());
    std::transform(
        geometries.begin(), geometries.end(), std::back_inserter(inputs),
        [&](const RecognizedHoleGeometry& geometry) {
            return resolve(geometry, bounds, moldFrame);
        });
    return inputs;
}

} // namespace oap::tooling
