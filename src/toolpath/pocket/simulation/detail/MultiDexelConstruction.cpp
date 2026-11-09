// Independently compiled private MultiDexelConstruction implementation.
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/pocket/simulation/StockTargetCache.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_Orientation.hxx>

#include "toolpath/pocket/simulation/detail/MultiDexelImpl.h"
#include "toolpath/pocket/simulation/detail/MultiDexelRasterization.h"
#include "toolpath/core/ToolpathOcctParallel.h"

namespace oap::toolpath {
using namespace oap::toolpath::multi_dexel_detail;

MultiDexelStock::MultiDexelStock(
    std::shared_ptr<const Impl> impl)
    : m_impl(std::move(impl))
{
}

std::shared_ptr<const MultiDexelStock>
MultiDexelStock::fromShape(
    const TopoDS_Shape& shape,
    const gp_Vec& outwardAxis,
    double referenceCutterDiameter,
    const StockSimulationOptions& options,
    MultiDexelBuildMetrics* metrics,
    std::string* error,
    std::function<bool()> cancellationRequested)
{
    const SteadyClock::time_point start =
        SteadyClock::now();
    auto impl = std::make_shared<Impl>();
    auto finish = [&](std::string message) {
        impl->error = std::move(message);
        if (error)
            *error = impl->error;
        if (metrics)
            metrics->milliseconds =
                elapsedMilliseconds(start);
        return std::shared_ptr<const MultiDexelStock>{};
    };
    const double tolerance = std::max(
        options.geometricTolerance,
        Precision::Confusion());
    impl->tolerance = tolerance;
    if (shape.IsNull())
        return finish("multi-dexel requires a non-null stock");
    if (!std::isfinite(referenceCutterDiameter) ||
        referenceCutterDiameter <= tolerance) {
        return finish(
            "multi-dexel requires a positive reference cutter diameter");
    }
    impl->wAxis =
        normalizedAxis(outwardAxis, tolerance);
    if (impl->wAxis.Magnitude() <= tolerance)
        return finish("multi-dexel requires a valid fixed axis");

    try {
        {
            ToolpathOcctParallelLease lease(options.runParallel);
            if (!BRepCheck_Analyzer(
                     shape, Standard_True,
                     lease.enabled())
                     .IsValid()) {
                return finish(
                    "multi-dexel stock is not a valid OCCT shape");
            }
        }
        const double exactVolume =
            exactShapeVolume(shape);
        if (!std::isfinite(exactVolume) ||
            exactVolume <= tolerance) {
            return finish(
                "multi-dexel stock has no positive volume");
        }

        Bnd_Box bounds;
        BRepBndLib::AddOptimal(
            shape, bounds, Standard_False, Standard_False);
        if (bounds.IsVoid())
            return finish("multi-dexel stock has no finite bounds");
        double xMin = 0.0;
        double yMin = 0.0;
        double zMin = 0.0;
        double xMax = 0.0;
        double yMax = 0.0;
        double zMax = 0.0;
        bounds.Get(
            xMin, yMin, zMin,
            xMax, yMax, zMax);
        impl->origin = gp_Pnt(
            0.5 * (xMin + xMax),
            0.5 * (yMin + yMax),
            0.5 * (zMin + zMax));
        const gp_Ax3 frame(
            impl->origin, gp_Dir(impl->wAxis));
        impl->uAxis = gp_Vec(frame.XDirection());
        impl->vAxis = gp_Vec(frame.YDirection());

        impl->uMin = impl->vMin =
            std::numeric_limits<double>::infinity();
        impl->uMax = impl->vMax =
            -std::numeric_limits<double>::infinity();
        for (int xi = 0; xi < 2; ++xi) {
            for (int yi = 0; yi < 2; ++yi) {
                for (int zi = 0; zi < 2; ++zi) {
                    const LocalPoint point =
                        impl->project(gp_Pnt(
                            xi == 0 ? xMin : xMax,
                            yi == 0 ? yMin : yMax,
                            zi == 0 ? zMin : zMax));
                    impl->uMin =
                        std::min(impl->uMin, point.u);
                    impl->uMax =
                        std::max(impl->uMax, point.u);
                    impl->vMin =
                        std::min(impl->vMin, point.v);
                    impl->vMax =
                        std::max(impl->vMax, point.v);
                }
            }
        }
        const double uSpan =
            impl->uMax - impl->uMin;
        const double vSpan =
            impl->vMax - impl->vMin;
        if (!std::isfinite(uSpan) ||
            !std::isfinite(vSpan) ||
            uSpan <= tolerance ||
            vSpan <= tolerance) {
            return finish(
                "multi-dexel stock projection is degenerate");
        }

        const int maxCells = std::clamp(
            options.zMapMaxCellCount,
            4096, 16000000);
        const double memoryLimitedResolution =
            resolutionForCellBudget(
                uSpan, vSpan, maxCells);
        const double automaticResolution = std::max({
            // 无显式质量配置的调用方采用平衡档：每刀径 40 个 XY
            // 单元、长轴最多约 2048 个单元。总单元预算仍是最终内存
            // 保护，不能为了曲线外观绕过 memoryLimitedResolution。
            referenceCutterDiameter / 40.0,
            std::max(uSpan, vSpan) / 2048.0,
            memoryLimitedResolution,
            10.0 * tolerance,
            1.0e-3});
        impl->resolution =
            std::isfinite(options.zMapResolution) &&
                    options.zMapResolution > 0.0
            ? std::max(
                  options.zMapResolution,
                  memoryLimitedResolution)
            : automaticResolution;
        impl->nx = std::max(
            1, static_cast<int>(
                   std::ceil(
                       uSpan / impl->resolution)));
        impl->ny = std::max(
            1, static_cast<int>(
                   std::ceil(
                       vSpan / impl->resolution)));
        const std::size_t cellCount =
            static_cast<std::size_t>(impl->nx) *
            static_cast<std::size_t>(impl->ny);
        if (cellCount == 0 ||
            cellCount >
                static_cast<std::size_t>(
                    maxCells)) {
            return finish(
                "multi-dexel cell budget was exceeded");
        }
        impl->tileSize = std::clamp(
            options.zMapTileSize, 8, 256);
        impl->tilesX =
            (impl->nx + impl->tileSize - 1) /
            impl->tileSize;
        impl->tilesY =
            (impl->ny + impl->tileSize - 1) /
            impl->tileSize;

        const double deflection = std::max(
            10.0 * tolerance,
            0.25 * impl->resolution);
        BRepMesh_IncrementalMesh mesher(
            shape, deflection,
            Standard_False, 0.35,
            options.runParallel);
        mesher.Perform();
        if (!mesher.IsDone()) {
            return finish(
                "multi-dexel could not triangulate the stock");
        }

        std::vector<std::vector<Crossing>> crossings(
            cellCount);
        for (TopExp_Explorer explorer(
                 shape, TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            if (cancellationRequested &&
                cancellationRequested()) {
                return finish(
                    "multi-dexel initialization was cancelled");
            }
            const TopoDS_Face face =
                TopoDS::Face(explorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation =
                BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull())
                continue;
            const gp_Trsf transform =
                location.Transformation();
            const int faceSign =
                face.Orientation() == TopAbs_REVERSED
                ? -1
                : 1;
            for (int triangleIndex = 1;
                 triangleIndex <=
                     triangulation->NbTriangles();
                 ++triangleIndex) {
                int n1 = 0;
                int n2 = 0;
                int n3 = 0;
                triangulation->Triangle(
                    triangleIndex).Get(n1, n2, n3);
                const LocalPoint a = impl->project(
                    triangulation->Node(n1)
                        .Transformed(transform));
                const LocalPoint b = impl->project(
                    triangulation->Node(n2)
                        .Transformed(transform));
                const LocalPoint c = impl->project(
                    triangulation->Node(n3)
                        .Transformed(transform));
                const double projectedNormal =
                    (b.u - a.u) * (c.v - a.v) -
                    (b.v - a.v) * (c.u - a.u);
                const int sign =
                    projectedNormal * faceSign >= 0.0
                    ? 1
                    : -1;
                rasterizeTriangle(
                    *impl, a, b, c, sign,
                    crossings);
            }
        }

        // Reuse one conversion buffer rather than allocating one vector per ray.
        std::vector<DexelInterval> columnIntervals;
        const double crossingTolerance =
            std::max(
                10.0 * tolerance,
                impl->resolution * 1.0e-8);
        impl->tiles.reserve(
            static_cast<std::size_t>(
                impl->tilesX * impl->tilesY));
        for (int ty = 0;
             ty < impl->tilesY; ++ty) {
            for (int tx = 0;
                 tx < impl->tilesX; ++tx) {
                auto tile =
                    std::make_shared<Impl::Tile>();
                tile->iMin = tx * impl->tileSize;
                tile->jMin = ty * impl->tileSize;
                tile->width = std::min(
                    impl->tileSize,
                    impl->nx - tile->iMin);
                tile->height = std::min(
                    impl->tileSize,
                    impl->ny - tile->jMin);
                tile->cells.resize(
                    static_cast<std::size_t>(
                        tile->width * tile->height));
                for (int localJ = 0;
                     localJ < tile->height; ++localJ) {
                    for (int localI = 0;
                         localI < tile->width; ++localI) {
                        const int i =
                            tile->iMin + localI;
                        const int j =
                            tile->jMin + localJ;
                        intervalsFromCrossings(
                            crossings[
                                static_cast<std::size_t>(j) *
                                    static_cast<std::size_t>(
                                        impl->nx) +
                                static_cast<std::size_t>(i)],
                            crossingTolerance, columnIntervals);
                        tile->cell(i, j).intervals.assign(columnIntervals);
                    }
                }
                impl->tiles.push_back(
                    std::move(tile));
            }
        }
        impl->refreshDerivedData();
        if (impl->activeCellCount == 0 ||
            impl->intervalCount == 0 ||
            impl->volume <= tolerance) {
            return finish(
                "multi-dexel projection contains no material intervals");
        }
        impl->valid = true;
        if (metrics) {
            metrics->milliseconds =
                elapsedMilliseconds(start);
            metrics->cellCount =
                impl->nx * impl->ny;
            metrics->activeCellCount =
                impl->activeCellCount;
            metrics->intervalCount =
                impl->intervalCount;
            metrics->tileCount =
                impl->tilesX * impl->tilesY;
        }
        if (error)
            error->clear();
        return std::shared_ptr<const MultiDexelStock>(
            new MultiDexelStock(std::move(impl)));
    } catch (const Standard_Failure& failure) {
        const char* message =
            failure.GetMessageString();
        return finish(
            std::string(
                "OCCT failed while initializing multi-dexel: ") +
            (message ? message : "unknown OCCT failure"));
    } catch (const std::exception& failure) {
        return finish(
            std::string(
                "failed to initialize multi-dexel: ") +
            failure.what());
    } catch (...) {
        return finish(
            "unknown failure while initializing multi-dexel");
    }
}

std::shared_ptr<const MultiDexelStock>
MultiDexelStock::fromShapeOnGrid(
    const TopoDS_Shape& shape,
    const MultiDexelStock& referenceGrid,
    const StockSimulationOptions& options,
    MultiDexelBuildMetrics* metrics,
    std::string* error,
    std::function<bool()> cancellationRequested)
{
    DexelDisplayGrid rasterized;
    if (!referenceGrid.rasterizeShapeOnGrid(
            shape, rasterized, metrics, error,
            std::move(cancellationRequested),
            options.runParallel,
            options.geometricTolerance)) {
        return {};
    }

    auto impl = std::make_shared<Impl>(
        *referenceGrid.m_impl);
    impl->valid = false;
    impl->error.clear();
    impl->tolerance = std::max(
        impl->tolerance, rasterized.tolerance);
    impl->tiles.clear();
    impl->tiles.reserve(
        static_cast<std::size_t>(
            impl->tilesX * impl->tilesY));
    for (int ty = 0; ty < impl->tilesY; ++ty) {
        for (int tx = 0; tx < impl->tilesX; ++tx) {
            auto tile = std::make_shared<Impl::Tile>();
            tile->iMin = tx * impl->tileSize;
            tile->jMin = ty * impl->tileSize;
            tile->width = std::min(
                impl->tileSize,
                impl->nx - tile->iMin);
            tile->height = std::min(
                impl->tileSize,
                impl->ny - tile->jMin);
            tile->cells.resize(
                static_cast<std::size_t>(
                    tile->width * tile->height));
            for (int localJ = 0;
                 localJ < tile->height; ++localJ) {
                for (int localI = 0;
                     localI < tile->width; ++localI) {
                    const int i = tile->iMin + localI;
                    const int j = tile->jMin + localJ;
                    const std::span<const DexelInterval>
                        intervals =
                            rasterized.intervalsAt(i, j);
                    tile->cell(i, j).intervals.assign(intervals);
                }
            }
            impl->tiles.push_back(std::move(tile));
        }
    }
    impl->refreshDerivedData();
    if (impl->activeCellCount == 0 ||
        impl->intervalCount == 0 ||
        impl->volume <= impl->tolerance) {
        if (error) {
            *error =
                "shape rasterization contains no positive material volume";
        }
        return {};
    }
    impl->valid = true;
    if (error)
        error->clear();
    return std::shared_ptr<const MultiDexelStock>(
        new MultiDexelStock(std::move(impl)));
}

bool MultiDexelStock::rasterizeShapeOnGrid(
    const TopoDS_Shape& shape,
    DexelDisplayGrid& output,
    MultiDexelBuildMetrics* metrics,
    std::string* error,
    std::function<bool()> cancellationRequested,
    bool runParallel,
    double geometricTolerance,
    bool allowProjectionClipping,
    const StockTargetCache* targetCache) const
{
    const SteadyClock::time_point start =
        SteadyClock::now();
    output = {};
    if (metrics)
        *metrics = {};
    auto fail = [&](std::string message) {
        output = {};
        if (error)
            *error = std::move(message);
        if (metrics)
            metrics->milliseconds =
                elapsedMilliseconds(start);
        return false;
    };
    if (!valid()) {
        return fail(
            "shape rasterization requires a valid multi-dexel grid");
    }
    if (shape.IsNull()) {
        return fail(
            "shape rasterization requires a non-null shape");
    }
    const Impl& grid = *m_impl;
    const double tolerance = std::max({
        std::isfinite(geometricTolerance)
            ? geometricTolerance
            : 1.0e-5,
        grid.tolerance,
        Precision::Confusion()});

    try {
        std::array<double, 6> cachedBounds{};
        const bool reuseBounds = targetCache && targetCache->matchesShape(shape) &&
            targetCache->bounds(cachedBounds);
        Bnd_Box bounds;
        if (!reuseBounds) {
            BRepBndLib::AddOptimal(shape, bounds, Standard_False, Standard_False);
            if (bounds.IsVoid()) return fail("shape rasterization could not determine bounds");
            bounds.Get(cachedBounds[0], cachedBounds[1], cachedBounds[2],
                cachedBounds[3], cachedBounds[4], cachedBounds[5]);
        }
        double xMin = 0.0;
        double yMin = 0.0;
        double zMin = 0.0;
        double xMax = 0.0;
        double yMax = 0.0;
        double zMax = 0.0;
        xMin = cachedBounds[0]; yMin = cachedBounds[1]; zMin = cachedBounds[2];
        xMax = cachedBounds[3]; yMax = cachedBounds[4]; zMax = cachedBounds[5];
        double projectedUMin =
            std::numeric_limits<double>::infinity();
        double projectedUMax =
            -std::numeric_limits<double>::infinity();
        double projectedVMin =
            std::numeric_limits<double>::infinity();
        double projectedVMax =
            -std::numeric_limits<double>::infinity();
        for (int xi = 0; xi < 2; ++xi) {
            for (int yi = 0; yi < 2; ++yi) {
                for (int zi = 0; zi < 2; ++zi) {
                    const LocalPoint point = grid.project(
                        gp_Pnt(
                            xi == 0 ? xMin : xMax,
                            yi == 0 ? yMin : yMax,
                            zi == 0 ? zMin : zMax));
                    projectedUMin =
                        std::min(projectedUMin, point.u);
                    projectedUMax =
                        std::max(projectedUMax, point.u);
                    projectedVMin =
                        std::min(projectedVMin, point.v);
                    projectedVMax =
                        std::max(projectedVMax, point.v);
                }
            }
        }
        const double projectionGuard =
            std::max(tolerance, 0.5 * grid.resolution);
        if (!allowProjectionClipping &&
            (projectedUMin <
                grid.uMin - projectionGuard ||
            projectedUMax >
                grid.uMax + projectionGuard ||
            projectedVMin <
                grid.vMin - projectionGuard ||
            projectedVMax >
                grid.vMax + projectionGuard)) {
            return fail(
                "shape projection extends outside the multi-dexel grid");
        }

        const double deflection = std::max(
            10.0 * tolerance,
            0.25 * grid.resolution);
        BRepMesh_IncrementalMesh mesher(
            shape, deflection,
            Standard_False, 0.35,
            runParallel);
        mesher.Perform();
        if (!mesher.IsDone()) {
            return fail(
                "shape rasterization could not triangulate the shape");
        }

        const std::size_t cellCount =
            static_cast<std::size_t>(grid.nx) *
            static_cast<std::size_t>(grid.ny);
        std::vector<std::vector<Crossing>> crossings(
            cellCount);
        for (TopExp_Explorer explorer(
                 shape, TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            if (cancellationRequested &&
                cancellationRequested()) {
                return fail(
                    "shape rasterization was cancelled");
            }
            const TopoDS_Face face =
                TopoDS::Face(explorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation =
                BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull())
                continue;
            const gp_Trsf transform =
                location.Transformation();
            const int faceSign =
                face.Orientation() == TopAbs_REVERSED
                ? -1
                : 1;
            for (int triangleIndex = 1;
                 triangleIndex <=
                     triangulation->NbTriangles();
                 ++triangleIndex) {
                int n1 = 0;
                int n2 = 0;
                int n3 = 0;
                triangulation->Triangle(
                    triangleIndex).Get(n1, n2, n3);
                const LocalPoint a = grid.project(
                    triangulation->Node(n1)
                        .Transformed(transform));
                const LocalPoint b = grid.project(
                    triangulation->Node(n2)
                        .Transformed(transform));
                const LocalPoint c = grid.project(
                    triangulation->Node(n3)
                        .Transformed(transform));
                const double projectedNormal =
                    (b.u - a.u) * (c.v - a.v) -
                    (b.v - a.v) * (c.u - a.u);
                const int sign =
                    projectedNormal * faceSign >= 0.0
                    ? 1
                    : -1;
                rasterizeTriangle(
                    grid, a, b, c, sign, crossings);
            }
        }

        output.origin = grid.origin;
        output.uAxis = grid.uAxis;
        output.vAxis = grid.vAxis;
        output.wAxis = grid.wAxis;
        output.uMin = grid.uMin;
        output.uMax = grid.uMax;
        output.vMin = grid.vMin;
        output.vMax = grid.vMax;
        output.resolution = grid.resolution;
        output.tolerance = tolerance;
        output.nx = grid.nx;
        output.ny = grid.ny;
        output.offsets.reserve(cellCount + 1u);
        output.offsets.push_back(0u);
        const double crossingTolerance = std::max(
            10.0 * tolerance,
            grid.resolution * 1.0e-8);
        int activeCellCount = 0;
        std::vector<DexelInterval> intervals;
        for (std::size_t cell = 0;
             cell < cellCount; ++cell) {
            if (cancellationRequested &&
                cell % static_cast<std::size_t>(
                           std::max(1, grid.nx)) == 0u &&
                cancellationRequested()) {
                return fail(
                    "shape rasterization was cancelled");
            }
            intervalsFromCrossings(crossings[cell], crossingTolerance, intervals);
            if (!intervals.empty())
                ++activeCellCount;
            if (output.intervals.size() +
                    intervals.size() >
                std::numeric_limits<std::uint32_t>::max()) {
                return fail(
                    "shape rasterization interval count exceeds capacity");
            }
            output.intervals.insert(
                output.intervals.end(),
                intervals.begin(), intervals.end());
            output.offsets.push_back(
                static_cast<std::uint32_t>(
                    output.intervals.size()));
        }
        if (!output.valid() ||
            output.intervals.empty()) {
            return fail(
                "shape rasterization contains no material intervals");
        }
        if (metrics) {
            metrics->milliseconds =
                elapsedMilliseconds(start);
            metrics->cellCount =
                grid.nx * grid.ny;
            metrics->activeCellCount =
                activeCellCount;
            metrics->intervalCount =
                static_cast<int>(
                    output.intervals.size());
            metrics->tileCount =
                grid.tilesX * grid.tilesY;
        }
        if (error)
            error->clear();
        return true;
    } catch (const Standard_Failure& failure) {
        const char* message =
            failure.GetMessageString();
        return fail(
            std::string(
                "OCCT failed while rasterizing shape on multi-dexel grid: ") +
            (message ? message : "unknown OCCT failure"));
    } catch (const std::exception& failure) {
        return fail(
            std::string(
                "failed to rasterize shape on multi-dexel grid: ") +
            failure.what());
    } catch (...) {
        return fail(
            "unknown failure while rasterizing shape on multi-dexel grid");
    }
}

} // namespace oap::toolpath
