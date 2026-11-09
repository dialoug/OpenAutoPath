// Independently compiled private MultiDexelDisplay implementation.
#include "toolpath/pocket/simulation/MultiDexelStock.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <BRep_Builder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include "toolpath/pocket/simulation/DexelDisplayMesh.h"
#include "toolpath/pocket/simulation/detail/MultiDexelImpl.h"
#include "toolpath/pocket/simulation/detail/MultiDexelOrientedSweep.h"

namespace oap::toolpath {
using namespace oap::toolpath::multi_dexel_detail;

bool MultiDexelStock::buildSnapshot(
    TopoDS_Shape& output,
    int* solidCount,
    std::string* error,
    std::function<bool()> cancellationRequested) const
{
    output.Nullify();
    if (solidCount)
        *solidCount = 0;
    auto fail = [&](std::string message) {
        if (error)
            *error = std::move(message);
        return false;
    };
    if (!valid())
        return fail("cannot snapshot an invalid multi-dexel stock");

    try {
        const std::size_t cellCount =
            static_cast<std::size_t>(m_impl->nx) *
            static_cast<std::size_t>(m_impl->ny);
        std::vector<std::vector<DexelInterval>> columns(
            cellCount);
        std::vector<std::vector<unsigned char>> used(
            cellCount);
        for (int j = 0; j < m_impl->ny; ++j) {
            for (int i = 0; i < m_impl->nx; ++i) {
                const std::size_t cell =
                    static_cast<std::size_t>(j) *
                        static_cast<std::size_t>(
                            m_impl->nx) +
                    static_cast<std::size_t>(i);
                const auto& intervals = m_impl->cell(i, j).intervals;
                columns[cell].assign(intervals.begin(), intervals.end());
                used[cell].assign(
                    columns[cell].size(), 0u);
            }
        }

        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);
        int count = 0;
        const double mergeTolerance =
            std::max(
                10.0 * m_impl->tolerance,
                m_impl->resolution * 1.0e-7);
        for (int j = 0; j < m_impl->ny; ++j) {
            if (cancellationRequested &&
                cancellationRequested()) {
                return fail(
                    "multi-dexel snapshot was cancelled");
            }
            for (int i = 0; i < m_impl->nx; ++i) {
                const std::size_t firstCell =
                    static_cast<std::size_t>(j) *
                        static_cast<std::size_t>(
                            m_impl->nx) +
                    static_cast<std::size_t>(i);
                for (std::size_t intervalIndex = 0;
                     intervalIndex <
                         columns[firstCell].size();
                     ++intervalIndex) {
                    if (used[firstCell][intervalIndex] != 0u)
                        continue;
                    const DexelInterval first =
                        columns[firstCell][intervalIndex];
                    int width = 1;
                    while (i + width < m_impl->nx) {
                        const std::size_t candidate =
                            static_cast<std::size_t>(j) *
                                static_cast<std::size_t>(
                                    m_impl->nx) +
                            static_cast<std::size_t>(
                                i + width);
                        if (intervalIndex >=
                                columns[candidate].size() ||
                            used[candidate][intervalIndex] != 0u ||
                            !sameInterval(
                                first,
                                columns[candidate][intervalIndex],
                                mergeTolerance)) {
                            break;
                        }
                        ++width;
                    }
                    int height = 1;
                    for (; j + height < m_impl->ny;
                         ++height) {
                        bool matches = true;
                        for (int x = 0; x < width; ++x) {
                            const std::size_t candidate =
                                static_cast<std::size_t>(
                                    j + height) *
                                    static_cast<std::size_t>(
                                        m_impl->nx) +
                                static_cast<std::size_t>(
                                    i + x);
                            if (intervalIndex >=
                                    columns[candidate].size() ||
                                used[candidate][intervalIndex] != 0u ||
                                !sameInterval(
                                    first,
                                    columns[candidate][intervalIndex],
                                    mergeTolerance)) {
                                matches = false;
                                break;
                            }
                        }
                        if (!matches)
                            break;
                    }
                    for (int y = 0; y < height; ++y) {
                        for (int x = 0; x < width; ++x) {
                            const std::size_t cell =
                                static_cast<std::size_t>(j + y) *
                                    static_cast<std::size_t>(
                                        m_impl->nx) +
                                static_cast<std::size_t>(i + x);
                            used[cell][intervalIndex] = 1u;
                        }
                    }
                    const double u0 =
                        m_impl->cellU0(i);
                    const double u1 =
                        m_impl->cellU1(i + width - 1);
                    const double v0 =
                        m_impl->cellV0(j);
                    const double v1 =
                        m_impl->cellV1(j + height - 1);
                    const double du = u1 - u0;
                    const double dv = v1 - v0;
                    const double dw =
                        first.upper - first.lower;
                    if (!std::isfinite(du) ||
                        !std::isfinite(dv) ||
                        !std::isfinite(dw) ||
                        du <= m_impl->tolerance ||
                        dv <= m_impl->tolerance ||
                        dw <= m_impl->tolerance) {
                        continue;
                    }
                    const gp_Pnt corner = localToWorld(
                        m_impl->origin,
                        m_impl->uAxis,
                        m_impl->vAxis,
                        m_impl->wAxis,
                        u0, v0, first.lower);
                    BRepPrimAPI_MakeBox box(
                        gp_Ax2(
                            corner,
                            gp_Dir(m_impl->wAxis),
                            gp_Dir(m_impl->uAxis)),
                        du, dv, dw);
                    box.Build();
                    if (!box.IsDone() ||
                        box.Shape().IsNull()) {
                        std::ostringstream message;
                        message.precision(17);
                        message
                            << "failed to construct a multi-dexel snapshot prism"
                            << " (du=" << du
                            << ", dv=" << dv
                            << ", dw=" << dw
                            << ", u0=" << u0
                            << ", v0=" << v0
                            << ", w0=" << first.lower
                            << ", corner=("
                            << corner.X() << ','
                            << corner.Y() << ','
                            << corner.Z() << ')'
                            << ", uAxis=("
                            << m_impl->uAxis.X() << ','
                            << m_impl->uAxis.Y() << ','
                            << m_impl->uAxis.Z() << ')'
                            << ", wAxis=("
                            << m_impl->wAxis.X() << ','
                            << m_impl->wAxis.Y() << ','
                            << m_impl->wAxis.Z() << ')'
                            << ')';
                        return fail(message.str());
                    }
                    builder.Add(
                        compound, box.Shape());
                    ++count;
                    if (count > 250000) {
                        return fail(
                            "multi-dexel snapshot exceeded its merged-prism budget");
                    }
                }
            }
        }
        if (count > 0)
            output = compound;
        if (solidCount)
            *solidCount = count;
        if (error)
            error->clear();
        return true;
    } catch (const Standard_Failure& failure) {
        const char* message =
            failure.GetMessageString();
        return fail(
            std::string(
                "OCCT failed while building multi-dexel snapshot: ") +
            (message ? message : "unknown OCCT failure"));
    } catch (const std::exception& failure) {
        return fail(
            std::string(
                "failed to build multi-dexel snapshot: ") +
            failure.what());
    } catch (...) {
        return fail(
            "unknown failure while building multi-dexel snapshot");
    }
}

bool MultiDexelStock::buildDisplayMesh(
    Handle(Poly_Triangulation)& output,
    MultiDexelDisplayMeshMetrics* metrics,
    std::string* error,
    std::function<bool()> cancellationRequested,
    const DexelDisplayMeshOptions& options) const
{
    const SteadyClock::time_point start = SteadyClock::now();
    output.Nullify();
    if (metrics)
        *metrics = {};

    auto fail = [&](std::string message) {
        if (metrics) {
            *metrics = {};
            metrics->milliseconds = elapsedMilliseconds(start);
        }
        if (error)
            *error = std::move(message);
        return false;
    };
    if (!valid())
        return fail("cannot mesh an invalid multi-dexel stock");

    const std::size_t cellCount =
        static_cast<std::size_t>(m_impl->nx) *
        static_cast<std::size_t>(m_impl->ny);
    if (static_cast<std::size_t>(m_impl->intervalCount) >
        std::numeric_limits<std::uint32_t>::max()) {
        return fail(
            "multi-dexel interval count exceeds display-grid capacity");
    }

    DexelDisplayGrid grid;
    grid.origin = m_impl->origin;
    grid.uAxis = m_impl->uAxis;
    grid.vAxis = m_impl->vAxis;
    grid.wAxis = m_impl->wAxis;
    grid.uMin = m_impl->uMin;
    grid.uMax = m_impl->uMax;
    grid.vMin = m_impl->vMin;
    grid.vMax = m_impl->vMax;
    grid.resolution = m_impl->resolution;
    grid.tolerance = m_impl->tolerance;
    grid.nx = m_impl->nx;
    grid.ny = m_impl->ny;
    grid.offsets.reserve(cellCount + 1u);
    grid.intervals.reserve(
        static_cast<std::size_t>(m_impl->intervalCount));
    grid.offsets.push_back(0u);

    for (int j = 0; j < m_impl->ny; ++j) {
        if (cancellationRequested &&
            cancellationRequested()) {
            return fail(
                "multi-dexel display-grid conversion was cancelled");
        }
        for (int i = 0; i < m_impl->nx; ++i) {
            const auto& intervals =
                m_impl->cell(i, j).intervals;
            grid.intervals.insert(
                grid.intervals.end(),
                intervals.begin(), intervals.end());
            grid.offsets.push_back(
                static_cast<std::uint32_t>(
                    grid.intervals.size()));
        }
    }

    const bool built = buildDexelDisplayMesh(
        grid, output, metrics, error,
        std::move(cancellationRequested), options);
    if (metrics)
        metrics->milliseconds = elapsedMilliseconds(start);
    return built;
}

} // namespace oap::toolpath
