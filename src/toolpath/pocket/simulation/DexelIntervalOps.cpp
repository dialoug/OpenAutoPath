#include "toolpath/pocket/simulation/DexelIntervalOps.h"
#include "toolpath/pocket/simulation/detail/DexelCellIntervals.h"

// 实现细节说明：本文件只做「一列材料减一个 removal」的区间集合差，
// 不包含刀具扫掠几何。两个分支都严格维持调用方传入的
// 「按 lower 递增、互不重叠、厚度大于 tolerance」不变式。

#include <algorithm>
#include <cmath>
#include <utility>

namespace oap::toolpath {

namespace {
template<class Intervals>
bool subtractInterval(
    Intervals& material,
    const DexelInterval& removal,
    double tolerance)
{
    if (!std::isfinite(removal.lower) ||
        !std::isfinite(removal.upper) ||
        !std::isfinite(tolerance) ||
        tolerance < 0.0 ||
        removal.upper <= removal.lower ||
        removal.upper - removal.lower <= tolerance ||
        material.empty()) {
        return false;
    }

    if (material.size() == 1u) {
        // 单区间快速路径：最多把原区间切成两段（保留下段/上段），
        // 或整体清除，避免构造中间容器。
        DexelInterval& interval = material.front();
        if (removal.upper <= interval.lower + tolerance ||
            removal.lower >= interval.upper - tolerance) {
            return false;
        }

        const bool keepLower =
            removal.lower > interval.lower + tolerance;
        const bool keepUpper =
            removal.upper < interval.upper - tolerance;
        if (keepLower && keepUpper) {
            const double originalUpper = interval.upper;
            interval.upper =
                std::min(removal.lower, originalUpper);
            material.push_back({
                std::max(removal.upper, interval.lower),
                originalUpper});
        } else if (keepLower) {
            interval.upper =
                std::min(removal.lower, interval.upper);
        } else if (keepUpper) {
            interval.lower =
                std::max(removal.upper, interval.lower);
        } else {
            material.clear();
        }
        return true;
    }

    const bool intersectsMaterial =
        std::any_of(
            material.begin(), material.end(),
            [&](const DexelInterval& interval) {
                return removal.upper >
                           interval.lower + tolerance &&
                       removal.lower <
                           interval.upper - tolerance;
            });
    if (!intersectsMaterial)
        return false;

    Intervals next;
    next.reserve(material.size() + 1u);
    // 通用多区间路径：逐段检查 material 与 removal 的重叠（含容差），
    // 命中区间拆成 removal 上/下两侧的保留段；完全不相交的区间原样保留。
    // removal 自身比所有区间都宽时，最终 next 为空，表示该列被全部切除。
    for (const DexelInterval& interval : material) {
        if (removal.upper <= interval.lower + tolerance ||
            removal.lower >= interval.upper - tolerance) {
            next.push_back(interval);
            continue;
        }

        if (removal.lower > interval.lower + tolerance) {
            next.push_back({
                interval.lower,
                std::min(removal.lower, interval.upper)});
        }
        if (removal.upper < interval.upper - tolerance) {
            next.push_back({
                std::max(removal.upper, interval.lower),
                interval.upper});
        }
    }

    material = std::move(next);
    return true;
}

} // namespace

bool subtractDexelInterval(
    std::vector<DexelInterval>& material,
    const DexelInterval& removal,
    double tolerance)
{
    return subtractInterval(material, removal, tolerance);
}

bool multi_dexel_detail::subtractDexelInterval(
    DexelCellIntervals& material,
    const DexelInterval& removal,
    double tolerance)
{
    const bool changed = subtractInterval(material, removal, tolerance);
    if (changed)
        material.compact();
    return changed;
}

} // namespace oap::toolpath
