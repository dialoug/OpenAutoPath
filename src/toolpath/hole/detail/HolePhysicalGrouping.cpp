// 使用共轴、共线和轴向区间相接规则，把沉头/台阶孔的圆柱段合并为物理孔。
// 属于 oap_toolpath_hole 模块内部实现；通过并查集传递归组，保证任意
// 两段间接同轴的孔段最终也落入同一个物理孔。
#include "toolpath/hole/HolePhysicalGrouping.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace oap::toolpath {
namespace {

constexpr double kAxisDotTolerance = 0.9998476951563913; // cos(1 度)

// 路径压缩并查集：用于把满足两两条件的孔段传递性归入同一物理孔。
class DisjointSet {
public:
    explicit DisjointSet(std::size_t count) : m_parent(count) {
        std::iota(m_parent.begin(), m_parent.end(), 0);
    }

    std::size_t find(std::size_t value) {
        if (m_parent[value] != value)
            m_parent[value] = find(m_parent[value]);
        return m_parent[value];
    }

    void unite(std::size_t first, std::size_t second) {
        first = find(first);
        second = find(second);
        if (first != second)
            m_parent[second] = first;
    }

private:
    std::vector<std::size_t> m_parent;
};

double projection(const gp_Pnt& point, const gp_Vec& axis)
{
    return gp_Vec(gp_Pnt(0.0, 0.0, 0.0), point).Dot(axis);
}

// 两个孔段是否属于同一物理孔：轴向夹角不超过 1 度、入口侧向偏移在容差
// 内，且两者在孔轴上的投影区间相接或重叠。
bool sharesPhysicalHole(const HolePhysicalSegmentInput& first,
                        const HolePhysicalSegmentInput& second,
                        double tolerance)
{
    gp_Vec axis = first.inwardAxis;
    gp_Vec otherAxis = second.inwardAxis;
    if (axis.Magnitude() <= tolerance || otherAxis.Magnitude() <= tolerance)
        return false;
    axis.Normalize();
    otherAxis.Normalize();
    if (axis.Dot(otherAxis) < kAxisDotTolerance)
        return false;

    gp_Vec between(first.entry, second.entry);
    between -= axis.Multiplied(between.Dot(axis));
    if (between.Magnitude() > tolerance)
        return false;

    const double firstEntry = projection(first.entry, axis);
    const double firstBottom = projection(first.bottom, axis);
    const double secondEntry = projection(second.entry, axis);
    const double secondBottom = projection(second.bottom, axis);
    const double firstMin = std::min(firstEntry, firstBottom);
    const double firstMax = std::max(firstEntry, firstBottom);
    const double secondMin = std::min(secondEntry, secondBottom);
    const double secondMax = std::max(secondEntry, secondBottom);
    return std::max(firstMin, secondMin) <= std::min(firstMax, secondMax) + tolerance;
}

} // namespace

std::vector<HolePhysicalGroup> groupPhysicalHoleSegments(
    const std::vector<HolePhysicalSegmentInput>& segments,
    double linearTolerance)
{
    const double tolerance = std::max(1.0e-6, linearTolerance);
    const std::size_t count = segments.size();
    std::vector<HolePhysicalGroup> groups;
    if (count == 0)
        return groups;

    DisjointSet connected(count);
    for (std::size_t first = 0; first < count; ++first) {
        for (std::size_t second = first + 1; second < count; ++second) {
            if (sharesPhysicalHole(segments[first], segments[second], tolerance))
                connected.unite(first, second);
        }
    }

    std::vector<std::size_t> roots;
    roots.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const std::size_t root = connected.find(index);
        auto found = std::find(roots.begin(), roots.end(), root);
        if (found == roots.end()) {
            roots.push_back(root);
            groups.push_back(HolePhysicalGroup{});
            found = std::prev(roots.end());
        }
        groups[static_cast<std::size_t>(found - roots.begin())].segmentIndices.push_back(index);
    }

    for (HolePhysicalGroup& group : groups) {
        const HolePhysicalSegmentInput& reference = segments[group.segmentIndices.front()];
        gp_Vec axis = reference.inwardAxis;
        axis.Normalize();
        std::stable_sort(group.segmentIndices.begin(), group.segmentIndices.end(),
                         [&](std::size_t first, std::size_t second) {
            const HolePhysicalSegmentInput& a = segments[first];
            const HolePhysicalSegmentInput& b = segments[second];
            const double firstEntry = projection(a.entry, axis);
            const double secondEntry = projection(b.entry, axis);
            if (std::abs(firstEntry - secondEntry) > tolerance)
                return firstEntry < secondEntry;
            if (std::abs(a.radius - b.radius) > tolerance)
                return a.radius > b.radius;
            return a.originalIndex < b.originalIndex;
        });
        group.anchorSegmentIndex = group.segmentIndices.front();
        group.originalIndex = std::numeric_limits<std::size_t>::max();
        for (std::size_t index : group.segmentIndices)
            group.originalIndex = std::min(group.originalIndex, segments[index].originalIndex);
    }

    return groups;
}

} // namespace oap::toolpath
