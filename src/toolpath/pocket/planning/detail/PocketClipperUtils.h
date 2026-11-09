// Clipper2 内部表示及归一化辅助函数。
#pragma once

#include <vector>

#include "clipper2/clipper.h"

#include "toolpath/pocket/geometry/OcctAreaAdapter.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"

namespace oap::toolpath::detail {

using Clipper2Lib::PathD;
using Clipper2Lib::PathsD;
using Clipper2Lib::PointD;

// 一条路径的排序元数据：有符号面积、嵌套深度（被多少个更大环包含）
// 与包围盒最小点，用于图案环的稳定排序和岛环识别。
struct PathInfo {
    PathD path;
    double area = 0.0;
    double absArea = 0.0;
    int nesting = 0;
    double minX = 0.0;
    double minY = 0.0;
};

double signedArea(const PathD& path);
PathsD cleanPaths(const PathsD& paths, int precision);
// 对已归一化的 NonZero 区域做圆角偏置并重新归一化。正值向材料外扩，
// 负值向区域内部收缩；供生成和安全校验共享同一偏置语义。
PathsD offsetArea(const PathsD& paths,
                  double delta,
                  int precision,
                  double arcTolerance);
PathsD normalizedClearArea(const PocketRegion& region, int precision);
PathsD normalizedClearArea(const std::vector<Loop2d>& contours, int precision);
// 保留任意嵌套层级的正负轮廓：实体外环、腔孔和腔内凸台分别对应
// 正、负、正面积，不能简化成单次 outer-islands 差集。
PathsD normalizedNestedArea(const std::vector<Loop2d>& contours, int precision);
std::vector<Loop2d> nestedAreaContours(const PathsD& paths);
// 对加工约束的两侧做预偏置,使图案生成器消费真实的刀心区域,
// 并必须以零作为其首次内缩量。
PathsD cutterCenterReachableArea(
    const PathsD& clearArea,
    const PathsD& protectedMaterial,
    double requiredMargin,
    int precision,
    double arcTolerance);
// 计算较大平底刀完整清腔后，当前较小刀仍需覆盖的刀心区域。
// 返回空表示当前刀相对上一刀没有新增可去除材料。
PathsD restMachiningCenterArea(
    const PathsD& clearArea,
    const PathsD& protectedMaterial,
    double currentRadius,
    double previousRadius,
    double extraOffset,
    int precision,
    double arcTolerance);
// 将虚拟封口沿实体外侧扩展，使刀具中心可越过真实通槽出口，而不是
// 被统一内偏置限制在虚拟封口内侧。
PathsD extendThroughVirtualOpenBoundaries(
    const PathsD& area,
    const std::vector<Loop2d>& contours,
    double extension,
    int precision);
std::vector<PathInfo> sortedPathInfos(const PathsD& paths);

} // namespace oap::toolpath::detail
