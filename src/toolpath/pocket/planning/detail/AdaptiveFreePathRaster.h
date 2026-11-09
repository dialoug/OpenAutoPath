// Raster remaining-material model used only by Adaptive FreePath planning.
// Continuous Clipper geometry remains authoritative for cutter-center safety.
#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include <gp_Pnt2d.hxx>

#include "PocketClipperUtils.h"

namespace oap::toolpath::detail {

struct AdaptiveAngularEngagement {
    double contactFraction = 0.0;
    double maximumArcAngle = 0.0; // radians
};

struct AdaptiveRasterComponent {
    std::vector<std::size_t> cells;
    std::size_t minX = 0u;
    std::size_t minY = 0u;
    std::size_t maxX = 0u;
    std::size_t maxY = 0u;
    gp_Pnt2d centroid;
    double principalAngle = 0.0;
};

class AdaptiveFreePathRaster {
public:
    AdaptiveFreePathRaster(const PathsD& materialArea,
                           const PathsD& safeCenterArea,
                           double requestedCellSize,
                           std::size_t maximumCells = 260000u);

    bool valid() const { return m_initialCount > 0u && m_safeCount > 0u; }
    double cellSize() const { return m_cellSize; }
    double dx() const { return m_dx; }
    double dy() const { return m_dy; }
    std::size_t nx() const { return m_nx; }
    std::size_t ny() const { return m_ny; }
    std::size_t initialCount() const { return m_initialCount; }
    std::size_t remainingCount() const { return m_remainingCount; }
    double remainingFraction() const;

    gp_Pnt2d point(std::size_t x, std::size_t y) const;
    bool safe(const gp_Pnt2d& point) const;
    bool segmentSafe(const gp_Pnt2d& start,
                     const gp_Pnt2d& end,
                     double sampleSpacing) const;

    double engagementRatio(const gp_Pnt2d& center,
                           double radius,
                           const std::unordered_set<std::size_t>* ignored = nullptr) const;
    AdaptiveAngularEngagement angularEngagement(
        const gp_Pnt2d& center,
        double radius,
        int samples,
        const std::unordered_set<std::size_t>* ignored = nullptr) const;

    std::vector<std::size_t> segmentRemainingIndices(
        const gp_Pnt2d& start,
        const gp_Pnt2d& end,
        double radius,
        const std::unordered_set<std::size_t>* ignored = nullptr) const;
    std::size_t polylineRemainingCount(
        const std::vector<gp_Pnt2d>& points,
        double radius) const;
    std::size_t removeSegment(const gp_Pnt2d& start,
                              const gp_Pnt2d& end,
                              double radius);
    std::size_t removePolyline(const std::vector<gp_Pnt2d>& points,
                               double radius);
    std::vector<AdaptiveRasterComponent> components(
        std::size_t minimumCells = 1u) const;

private:
    bool inArea(const PathsD& area, const gp_Pnt2d& point) const;
    bool remainingAt(const gp_Pnt2d& point,
                     const std::unordered_set<std::size_t>* ignored) const;
    double distanceSquaredToSegment(double x, double y,
                                    const gp_Pnt2d& start,
                                    const gp_Pnt2d& end) const;
    void cellRange(double minX, double minY, double maxX, double maxY,
                   std::size_t& x0, std::size_t& y0,
                   std::size_t& x1, std::size_t& y1) const;

    PathsD m_safeCenterArea;
    double m_minX = 0.0;
    double m_minY = 0.0;
    double m_maxX = 0.0;
    double m_maxY = 0.0;
    std::size_t m_nx = 0u;
    std::size_t m_ny = 0u;
    double m_dx = 0.0;
    double m_dy = 0.0;
    double m_cellSize = 0.0;
    double m_halfDiagonal = 0.0;
    std::vector<std::uint8_t> m_initial;
    std::vector<std::uint8_t> m_remaining;
    std::vector<std::uint8_t> m_safe;
    std::size_t m_initialCount = 0u;
    std::size_t m_remainingCount = 0u;
    std::size_t m_safeCount = 0u;
};

} // namespace oap::toolpath::detail
