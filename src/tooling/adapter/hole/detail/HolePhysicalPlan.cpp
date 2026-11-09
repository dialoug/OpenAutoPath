// 将识别到的同轴孔段归并成物理孔计划，并提取直径、深度和装夹约束。
// 属于 hole 模块 detail 私有实现;依赖 toolpath/hole 的物理分组算法
// (HolePhysicalGrouping)做同轴归并,输出为各阶段规划器统一消费的几何。
#include "tooling/adapter/hole/detail/HolePhysicalPlan.h"

#include <cmath>
#include <limits>
#include <utility>

#include <Precision.hxx>

#include "toolpath/hole/HolePhysicalGrouping.h"

namespace oap::tooling::detail {

std::vector<PlannedPhysicalHole> buildPhysicalHolePlan(
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    oap::MoldSetupFace face,
    std::size_t& nextStableIndex)
{
    std::vector<oap::toolpath::HolePhysicalSegmentInput>
        inputs;
    for (std::size_t index = 0; index < holes.size(); ++index) {
        if (holes[index].setupFace != face)
            continue;
        gp_Vec inward(holes[index].entry, holes[index].bottom);
        if (inward.Magnitude() <= Precision::Confusion())
            continue;

        oap::toolpath::HolePhysicalSegmentInput input;
        input.localIndex = inputs.size();
        input.originalIndex = index;
        input.entry = holes[index].entry;
        input.bottom = holes[index].bottom;
        input.inwardAxis = inward;
        input.radius = holes[index].radius;
        inputs.push_back(std::move(input));
    }

    std::vector<PlannedPhysicalHole> result;
    const auto groups =
        oap::toolpath::groupPhysicalHoleSegments(inputs, 1.0e-4);
    // 对每组同轴孔段:锚定外端段作为入口与轴向;遍历组内各段求
    // 最小直径(等径段取最深的底),记录最小直径段集合供钻削/铣削使用。
    for (const auto& group : groups) {
        if (group.segmentIndices.empty())
            continue;

        PlannedPhysicalHole physical;
        physical.stableIndex = nextStableIndex++;
        physical.setupFace = face;
        physical.outerSegmentIndex =
            inputs[group.anchorSegmentIndex].originalIndex;
        physical.outerEntry =
            holes[physical.outerSegmentIndex].entry;
        physical.inwardAxis = gp_Vec(
            holes[physical.outerSegmentIndex].entry,
            holes[physical.outerSegmentIndex].bottom);
        physical.inwardAxis.Normalize();
        physical.minimumDiameter =
            std::numeric_limits<double>::infinity();

        for (const std::size_t groupedIndex : group.segmentIndices) {
            const std::size_t sourceIndex =
                inputs[groupedIndex].originalIndex;
            physical.segmentIndices.push_back(sourceIndex);
            const double diameter = 2.0 * holes[sourceIndex].radius;
            if (diameter < physical.minimumDiameter - 1.0e-5) {
                physical.minimumDiameter = diameter;
                physical.minimumSegmentIndex = sourceIndex;
                physical.deepestBottom = holes[sourceIndex].bottom;
                physical.minimumDiameterSegmentIndices.clear();
                physical.minimumDiameterSegmentIndices.push_back(sourceIndex);
            } else if (
                std::abs(diameter - physical.minimumDiameter) <=
                1.0e-5) {
                physical.minimumDiameterSegmentIndices.push_back(sourceIndex);
                physical.minimumSegmentIndex = std::min(
                    physical.minimumSegmentIndex, sourceIndex);
                const double currentDepth =
                    gp_Vec(physical.outerEntry,
                           physical.deepestBottom)
                        .Dot(physical.inwardAxis);
                const double candidateDepth =
                    gp_Vec(physical.outerEntry,
                           holes[sourceIndex].bottom)
                        .Dot(physical.inwardAxis);
                if (candidateDepth > currentDepth)
                    physical.deepestBottom =
                        holes[sourceIndex].bottom;
            }
        }
        result.push_back(std::move(physical));
    }
    return result;
}

oap::toolpath::HoleMachiningInput makePilotHole(
    const PlannedPhysicalHole& physical,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes)
{
    oap::toolpath::HoleMachiningInput result =
        holes[physical.minimumSegmentIndex];
    result.entry = physical.outerEntry;
    result.bottom = physical.deepestBottom;
    result.radius = 0.5 * physical.minimumDiameter;
    result.setupFace = physical.setupFace;
    result.preserveEntrySide = true;
    return result;
}

} // namespace oap::tooling::detail
