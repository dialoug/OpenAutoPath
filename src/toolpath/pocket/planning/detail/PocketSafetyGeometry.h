// 槽腔刀心二维安全判定。入刀规划与最终刀路验证共享同一套边界/岛余量语义。
#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/pocket/planning/detail/PocketClipperUtils.h"

namespace oap::toolpath::detail {

// 将任意 3D 点投影到区域 frame 的 XY 平面（局部 2D 坐标）。
gp_Pnt2d projectToPocketPlane(const gp_Pnt& point, const gp_Ax3& frame);

// 点到环的最小距离（含环边线段，环按隐式闭合处理）。
double distanceToPocketLoop(const gp_Pnt2d& point, const Loop2d& loop);

// 点在环内判定：tolerance 内视为“在环上”，通过射线法判断包含关系。
bool pointInPocketLoop(const gp_Pnt2d& point,
                       const Loop2d& loop,
                       double tolerance);

// 综合判定刀心点是否安全：位于根边界内、距边界至少 requiredMargin、
// 且不在任何岛的 requiredMargin 邻域内。
bool isPocketCutterCenterSafe(const gp_Pnt2d& point,
                              const PocketRegion& region,
                              double requiredMargin,
                              double tolerance);

// 将根边界安全区和各深度材料禁入区预偏置一次。大量移动/圆弧采样随后只需
// 做包围盒过滤和 PointInPolygon，避免对每个采样点重复扫描源轮廓全部边。
class PocketSafetyArea {
public:
    PocketSafetyArea(
        const PocketRegion& region,
        const ToolpathParams& params,
        const std::vector<ProtectedMaterialLayer>& protectedLayers);

    bool rootClear(const gp_Pnt2d& point) const;
    bool protectedMaterialClear(const gp_Pnt2d& point,
                                int depthIndex) const;
    bool clear(const gp_Pnt2d& point,
               int depthIndex,
               bool checkRootBoundary = true) const;

private:
    struct Bounds2d {
        double minX = 0.0;
        double minY = 0.0;
        double maxX = 0.0;
        double maxY = 0.0;
        bool valid = false;

        bool contains(const gp_Pnt2d& point) const noexcept;
    };
    struct PreparedArea {
        PathsD paths;
        std::vector<Bounds2d> bounds;
        // Path orientation never changes after construction.  Point queries
        // are extremely hot during final safety validation, so do not
        // recompute the signed area of every polygon for every sample.
        std::vector<std::int8_t> windingDirections;
    };
    struct PreparedProtectedArea {
        PreparedArea material;
        PreparedArea forbidden;
    };

    static PreparedArea prepare(PathsD paths);
    static bool contains(const PreparedArea& area,
                         const gp_Pnt2d& point,
                         bool includeBoundary);
    static double distanceToAreaBoundary(
        const PreparedArea& area,
        const gp_Pnt2d& point);

    PreparedArea m_rootSafeArea;
    std::unordered_map<int, PreparedProtectedArea>
        m_forbiddenMaterialByDepth;
    double m_checkedMargin = 0.0;
};

} // namespace oap::toolpath::detail
