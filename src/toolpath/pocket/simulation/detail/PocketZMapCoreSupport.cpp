// Independently compiled private PocketZMapCoreSupport implementation.
#include "toolpath/pocket/simulation/detail/PocketZMapCoreSupport.h"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <limits>
#include <ratio>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
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
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopAbs_ShapeEnum.hxx>

namespace oap::toolpath::zmap_detail {

double elapsedMilliseconds(const SteadyClock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               SteadyClock::now() - start)
        .count();
}

ScopedElapsedMilliseconds::ScopedElapsedMilliseconds(double& output)
        : m_output(output), m_start(SteadyClock::now())
{
    }

ScopedElapsedMilliseconds::~ScopedElapsedMilliseconds()
{
        m_output = elapsedMilliseconds(m_start);
    }

void ContentFingerprint::add(std::uint64_t value) noexcept
{
        for (int shift = 0; shift < 64; shift += 8) {
            const std::uint8_t byte =
                static_cast<std::uint8_t>(
                    (value >> shift) & 0xffu);
            m_first ^= byte;
            m_first *= 1099511628211ull;
            m_second +=
                byte + 0x9e3779b97f4a7c15ull;
            m_second ^= m_second >> 29;
            m_second *= 0xbf58476d1ce4e5b9ull;
            m_second ^= m_second >> 32;
        }
    }

void ContentFingerprint::add(double value) noexcept
{
        add(std::bit_cast<std::uint64_t>(value));
    }

void ContentFingerprint::add(std::string_view value) noexcept
{
        add(static_cast<std::uint64_t>(value.size()));
        for (const unsigned char byte : value) {
            add(static_cast<std::uint64_t>(byte));
        }
    }

std::string ContentFingerprint::finish() const
{
        std::ostringstream output;
        output << std::hex << std::setfill('0')
               << std::setw(16) << m_first
               << std::setw(16) << m_second;
        return output.str();
    }

std::string displaySourceFingerprint(
    const PocketZMapSimulationData& data,
    const TargetField& target)
{
    ContentFingerprint hash;
    hash.add(std::uint64_t{0x7a6d617064697370ull});
    hash.add(data.origin.X());
    hash.add(data.origin.Y());
    hash.add(data.origin.Z());
    for (const gp_Vec* axis :
         {&data.uAxis, &data.vAxis, &data.wAxis}) {
        hash.add(axis->X());
        hash.add(axis->Y());
        hash.add(axis->Z());
    }
    hash.add(data.uMin);
    hash.add(data.uMax);
    hash.add(data.vMin);
    hash.add(data.vMax);
    hash.add(data.resolution);
    hash.add(static_cast<std::uint64_t>(data.nx));
    hash.add(static_cast<std::uint64_t>(data.ny));
    auto addValues = [&](const std::vector<double>& values) {
        hash.add(
            static_cast<std::uint64_t>(
                values.size()));
        for (const double value : values)
            hash.add(value);
    };
    addValues(data.bottoms);
    addValues(data.initialTops);
    addValues(data.finalTops);
    addValues(target.bottoms);
    addValues(target.tops);
    hash.add(static_cast<std::uint64_t>(
        data.operationDeltas.size()));
    for (const PocketZMapOperationDelta& operation :
         data.operationDeltas) {
        hash.add(operation.operationId);
        hash.add(static_cast<std::uint64_t>(
            operation.sequenceIndex));
        hash.add(static_cast<std::uint64_t>(
            operation.cells.size()));
        for (const PocketZMapCellDelta& cell :
             operation.cells) {
            hash.add(static_cast<std::uint64_t>(
                cell.cellIndex));
            hash.add(cell.beforeTop);
            hash.add(cell.afterTop);
        }
    }
    return hash.finish();
}

bool cancelled(const ToolpathGenerationControl* control)
{
    return control && control->cancelled();
}

void addDiagnostic(
    StockComparisonResult& result,
    ToolpathDiagnosticCode code,
    ToolpathDiagnosticSeverity severity,
    std::string message)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = std::move(message);
    result.diagnostics.push_back(std::move(diagnostic));
}

void addAttributionDiagnostic(
    StockOvercutAttributionResult& result,
    ToolpathDiagnosticCode code,
    ToolpathDiagnosticSeverity severity,
    std::string message,
    std::string operationId)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = std::move(message);
    diagnostic.operationId = std::move(operationId);
    result.diagnostics.push_back(std::move(diagnostic));
}

std::size_t offset(
    const PocketZMapSimulationData& data,
    int i,
    int j)
{
    return static_cast<std::size_t>(j) *
               static_cast<std::size_t>(data.nx) +
           static_cast<std::size_t>(i);
}

double cellU0(const PocketZMapSimulationData& data, int i)
{
    return data.uMin +
           static_cast<double>(i) * data.resolution;
}

double cellU1(const PocketZMapSimulationData& data, int i)
{
    return std::min(
        data.uMax, cellU0(data, i) + data.resolution);
}

double cellV0(const PocketZMapSimulationData& data, int j)
{
    return data.vMin +
           static_cast<double>(j) * data.resolution;
}

double cellV1(const PocketZMapSimulationData& data, int j)
{
    return std::min(
        data.vMax, cellV0(data, j) + data.resolution);
}

double cellU(const PocketZMapSimulationData& data, int i)
{
    return 0.5 * (cellU0(data, i) + cellU1(data, i));
}

double cellV(const PocketZMapSimulationData& data, int j)
{
    return 0.5 * (cellV0(data, j) + cellV1(data, j));
}

double cellArea(
    const PocketZMapSimulationData& data,
    int i,
    int j)
{
    return (cellU1(data, i) - cellU0(data, i)) *
           (cellV1(data, j) - cellV0(data, j));
}

LocalPoint project(
    const PocketZMapSimulationData& data,
    const gp_Pnt& point)
{
    const gp_Vec relative(data.origin, point);
    return {
        relative.Dot(data.uAxis),
        relative.Dot(data.vAxis),
        relative.Dot(data.wAxis)};
}

int coordinateToI(
    const PocketZMapSimulationData& data,
    double u)
{
    return static_cast<int>(
        std::floor((u - data.uMin) / data.resolution));
}

int coordinateToJ(
    const PocketZMapSimulationData& data,
    double v)
{
    return static_cast<int>(
        std::floor((v - data.vMin) / data.resolution));
}

bool finiteInterval(
    double lower,
    double upper,
    double tolerance)
{
    return std::isfinite(lower) &&
           std::isfinite(upper) &&
           upper - lower > tolerance;
}

bool sameFiniteInterval(
    double leftLower,
    double leftUpper,
    double rightLower,
    double rightUpper,
    double tolerance)
{
    const bool leftFinite =
        finiteInterval(
            leftLower, leftUpper, tolerance);
    const bool rightFinite =
        finiteInterval(
            rightLower, rightUpper, tolerance);
    if (leftFinite != rightFinite)
        return false;
    if (!leftFinite)
        return true;
    return std::abs(leftLower - rightLower) <= tolerance &&
           std::abs(leftUpper - rightUpper) <= tolerance;
}

bool targetBoundaryCell(
    const PocketZMapSimulationData& data,
    const TargetField& target,
    std::size_t cell,
    double tolerance)
{
    const int i = static_cast<int>(
        cell % static_cast<std::size_t>(data.nx));
    const int j = static_cast<int>(
        cell / static_cast<std::size_t>(data.nx));
    for (int dj = -1; dj <= 1; ++dj) {
        for (int di = -1; di <= 1; ++di) {
            if (di == 0 && dj == 0)
                continue;
            const int neighborI = i + di;
            const int neighborJ = j + dj;
            if (neighborI < 0 ||
                neighborJ < 0 ||
                neighborI >= data.nx ||
                neighborJ >= data.ny) {
                return true;
            }
            const std::size_t neighbor =
                offset(data, neighborI, neighborJ);
            if (!sameFiniteInterval(
                    target.bottoms[cell],
                    target.tops[cell],
                    target.bottoms[neighbor],
                    target.tops[neighbor],
                    tolerance)) {
                return true;
            }
        }
    }
    return false;
}

void setInterval(
    Interval& output,
    double lower,
    double upper,
    double tolerance)
{
    if (!finiteInterval(lower, upper, tolerance))
        return;
    output.lower = lower;
    output.upper = upper;
}

bool shapeBounds(
    const TopoDS_Shape& shape,
    std::array<double, 6>& values)
{
    if (shape.IsNull())
        return false;
    try {
        Bnd_Box bounds;
        BRepBndLib::AddOptimal(
            shape, bounds, Standard_False, Standard_False);
        if (bounds.IsVoid())
            return false;
        bounds.Get(
            values[0], values[1], values[2],
            values[3], values[4], values[5]);
        return std::all_of(
            values.begin(), values.end(),
            [](double value) {
                return std::isfinite(value);
            });
    } catch (...) {
        return false;
    }
}

bool targetProjectionFitsGrid(
    const PocketZMapSimulationData& data,
    const std::array<double, 6>& bounds,
    double tolerance)
{
    double uMin = std::numeric_limits<double>::infinity();
    double uMax = -std::numeric_limits<double>::infinity();
    double vMin = std::numeric_limits<double>::infinity();
    double vMax = -std::numeric_limits<double>::infinity();
    for (int xi = 0; xi < 2; ++xi) {
        for (int yi = 0; yi < 2; ++yi) {
            for (int zi = 0; zi < 2; ++zi) {
                const LocalPoint point = project(
                    data,
                    gp_Pnt(
                        bounds[xi == 0 ? 0 : 3],
                        bounds[yi == 0 ? 1 : 4],
                        bounds[zi == 0 ? 2 : 5]));
                uMin = std::min(uMin, point.u);
                uMax = std::max(uMax, point.u);
                vMin = std::min(vMin, point.v);
                vMax = std::max(vMax, point.v);
            }
        }
    }
    const double guard =
        std::max(tolerance, 0.5 * data.resolution);
    return uMin >= data.uMin - guard &&
           uMax <= data.uMax + guard &&
           vMin >= data.vMin - guard &&
           vMax <= data.vMax + guard;
}

void rasterizeTriangle(
    const PocketZMapSimulationData& data,
    const LocalPoint& a,
    const LocalPoint& b,
    const LocalPoint& c,
    TargetField& target)
{
    const double denominator =
        (b.v - c.v) * (a.u - c.u) +
        (c.u - b.u) * (a.v - c.v);
    const double projectedScale = std::max({
        std::abs(a.u), std::abs(a.v),
        std::abs(b.u), std::abs(b.v),
        std::abs(c.u), std::abs(c.v), 1.0});
    if (std::abs(denominator) <=
        std::numeric_limits<double>::epsilon() *
            projectedScale * projectedScale * 32.0) {
        return;
    }
    int iMin = coordinateToI(
                   data, std::min({a.u, b.u, c.u})) -
               1;
    int iMax = coordinateToI(
                   data, std::max({a.u, b.u, c.u})) +
               1;
    int jMin = coordinateToJ(
                   data, std::min({a.v, b.v, c.v})) -
               1;
    int jMax = coordinateToJ(
                   data, std::max({a.v, b.v, c.v})) +
               1;
    iMin = std::clamp(iMin, 0, data.nx - 1);
    iMax = std::clamp(iMax, 0, data.nx - 1);
    jMin = std::clamp(jMin, 0, data.ny - 1);
    jMax = std::clamp(jMax, 0, data.ny - 1);
    constexpr double barycentricTolerance = 1.0e-10;
    for (int j = jMin; j <= jMax; ++j) {
        const double v = cellV(data, j);
        for (int i = iMin; i <= iMax; ++i) {
            const double u = cellU(data, i);
            const double alpha =
                ((b.v - c.v) * (u - c.u) +
                 (c.u - b.u) * (v - c.v)) /
                denominator;
            const double beta =
                ((c.v - a.v) * (u - c.u) +
                 (a.u - c.u) * (v - c.v)) /
                denominator;
            const double gamma = 1.0 - alpha - beta;
            if (alpha < -barycentricTolerance ||
                beta < -barycentricTolerance ||
                gamma < -barycentricTolerance) {
                continue;
            }
            const double w =
                alpha * a.w + beta * b.w + gamma * c.w;
            const std::size_t cell = offset(data, i, j);
            target.bottoms[cell] =
                std::min(target.bottoms[cell], w);
            target.tops[cell] =
                std::max(target.tops[cell], w);
        }
    }
}

bool rasterizeTarget(
    const PocketZMapSimulationData& data,
    const TopoDS_Shape& targetShape,
    const StockComparisonOptions& options,
    const ToolpathGenerationControl* control,
    TargetField& target,
    std::array<double, 6>& targetBounds,
    std::string& error)
{
    const double tolerance = std::max(
        options.geometricTolerance, Precision::Confusion());
    if (targetShape.IsNull()) {
        error = "Z-map comparison requires a target shape";
        return false;
    }
    const auto cache = options.targetCache && options.targetCache->matchesShape(targetShape)
        ? options.targetCache : nullptr;
    if (!(cache ? cache->bounds(targetBounds) : shapeBounds(targetShape, targetBounds))) {
        error = "Z-map comparison could not determine target bounds";
        return false;
    }
    if (!targetProjectionFitsGrid(
            data, targetBounds, tolerance)) {
        error =
            "target projection extends outside the pocket Z-map grid";
        return false;
    }
    const auto cacheKey = stockTargetGridKey(data, tolerance);
    if (cancelled(control)) {
        error = "Z-map target rasterization was cancelled";
        return false;
    }
    if (cache) {
        if (const auto cached = cache->heightField(cacheKey)) {
            target = *cached;
            return true;
        }
    }
    try {
        target.bottoms.assign(
            data.cellCount(),
            std::numeric_limits<double>::infinity());
        target.tops.assign(
            data.cellCount(),
            -std::numeric_limits<double>::infinity());
        const double deflection = std::max(
            10.0 * tolerance, 0.25 * data.resolution);
        BRepMesh_IncrementalMesh mesher(
            targetShape, deflection, Standard_False,
            0.35, options.runParallel);
        mesher.Perform();
        if (!mesher.IsDone()) {
            error =
                "Z-map comparison could not triangulate the target";
            return false;
        }
        for (TopExp_Explorer explorer(
                 targetShape, TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            if (cancelled(control)) {
                error =
                    "Z-map target rasterization was cancelled";
                return false;
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
            for (int triangleIndex = 1;
                 triangleIndex <=
                 triangulation->NbTriangles();
                 ++triangleIndex) {
                int n1 = 0;
                int n2 = 0;
                int n3 = 0;
                triangulation->Triangle(triangleIndex).Get(
                    n1, n2, n3);
                rasterizeTriangle(
                    data,
                    project(
                        data,
                        triangulation->Node(n1)
                            .Transformed(transform)),
                    project(
                        data,
                        triangulation->Node(n2)
                            .Transformed(transform)),
                    project(
                        data,
                        triangulation->Node(n3)
                            .Transformed(transform)),
                    target);
            }
        }
        target.volume = 0.0;
        for (int j = 0; j < data.ny; ++j) {
            if (cancelled(control)) {
                error =
                    "Z-map target rasterization was cancelled";
                return false;
            }
            for (int i = 0; i < data.nx; ++i) {
                const std::size_t cell =
                    offset(data, i, j);
                if (!finiteInterval(
                        target.bottoms[cell],
                        target.tops[cell],
                        tolerance)) {
                    target.bottoms[cell] =
                        std::numeric_limits<double>::infinity();
                    target.tops[cell] =
                        -std::numeric_limits<double>::infinity();
                    continue;
                }
                target.volume +=
                    (target.tops[cell] -
                     target.bottoms[cell]) *
                    cellArea(data, i, j);
            }
        }
        if (!std::isfinite(target.volume) ||
            target.volume <= tolerance) {
            error =
                "Z-map target projection contains no material";
            return false;
        }
        if (cancelled(control)) {
            error = "Z-map target rasterization was cancelled";
            return false;
        }
        if (cache) cache->store(cacheKey, target);
        return true;
    } catch (const Standard_Failure& failure) {
        const char* message = failure.GetMessageString();
        error =
            std::string(
                "OCCT failed while rasterizing Z-map target: ") +
            (message ? message : "unknown OCCT failure");
        return false;
    } catch (const std::exception& failure) {
        error =
            std::string(
                "failed to rasterize Z-map target: ") +
            failure.what();
        return false;
    } catch (...) {
        error = "unknown failure while rasterizing Z-map target";
        return false;
    }
}

double intervalVolume(
    const PocketZMapSimulationData& data,
    const std::vector<Interval>& intervals)
{
    double volume = 0.0;
    for (int j = 0; j < data.ny; ++j) {
        for (int i = 0; i < data.nx; ++i) {
            const Interval& interval =
                intervals[offset(data, i, j)];
            if (std::isfinite(interval.lower) &&
                std::isfinite(interval.upper) &&
                interval.upper > interval.lower) {
                volume +=
                    (interval.upper - interval.lower) *
                    cellArea(data, i, j);
            }
        }
    }
    return volume;
}

bool sameMultiDexelGrid(
    const MultiDexelStock& left,
    const MultiDexelStock& right,
    double tolerance)
{
    return left.valid() && right.valid() &&
           left.nx() == right.nx() &&
           left.ny() == right.ny() &&
           std::abs(
               left.resolution() -
               right.resolution()) <= tolerance &&
           std::abs(left.uMin() - right.uMin()) <=
               tolerance &&
           std::abs(left.uMax() - right.uMax()) <=
               tolerance &&
           std::abs(left.vMin() - right.vMin()) <=
               tolerance &&
           std::abs(left.vMax() - right.vMax()) <=
               tolerance &&
           gp_Vec(
               left.origin(),
               right.origin()).Magnitude() <= tolerance &&
           left.uAxis().IsEqual(
               right.uAxis(), tolerance, tolerance) &&
           left.vAxis().IsEqual(
               right.vAxis(), tolerance, tolerance) &&
           left.wAxis().IsEqual(
               right.wAxis(), tolerance, tolerance);
}

DexelDisplayGrid gridLike(
    const MultiDexelStock& stock,
    double tolerance)
{
    DexelDisplayGrid grid;
    grid.origin = stock.origin();
    grid.uAxis = stock.uAxis();
    grid.vAxis = stock.vAxis();
    grid.wAxis = stock.wAxis();
    grid.uMin = stock.uMin();
    grid.uMax = stock.uMax();
    grid.vMin = stock.vMin();
    grid.vMax = stock.vMax();
    grid.resolution = stock.resolution();
    grid.tolerance = tolerance;
    grid.nx = stock.nx();
    grid.ny = stock.ny();
    grid.offsets.reserve(
        static_cast<std::size_t>(grid.nx) *
            static_cast<std::size_t>(grid.ny) +
        1u);
    grid.offsets.push_back(0u);
    return grid;
}

double multiDexelCellArea(
    const MultiDexelStock& stock,
    int i,
    int j)
{
    const double u0 =
        stock.uMin() +
        static_cast<double>(i) * stock.resolution();
    const double u1 =
        std::min(
            stock.uMax(),
            u0 + stock.resolution());
    const double v0 =
        stock.vMin() +
        static_cast<double>(j) * stock.resolution();
    const double v1 =
        std::min(
            stock.vMax(),
            v0 + stock.resolution());
    return (u1 - u0) * (v1 - v0);
}

void subtractIntervals(
    std::span<const DexelInterval> material,
    std::span<const DexelInterval> cover,
    double tolerance,
    std::vector<DexelInterval>& output)
{
    output.clear();
    for (const DexelInterval& source : material) {
        double cursor = source.lower;
        for (const DexelInterval& blocking : cover) {
            if (blocking.upper <= cursor + tolerance)
                continue;
            if (blocking.lower >=
                source.upper - tolerance) {
                break;
            }
            if (blocking.lower >
                cursor + tolerance) {
                output.push_back({
                    cursor,
                    std::min(
                        source.upper,
                        blocking.lower)});
            }
            cursor =
                std::max(cursor, blocking.upper);
            if (cursor >= source.upper - tolerance)
                break;
        }
        if (cursor < source.upper - tolerance) {
            output.push_back(
                {cursor, source.upper});
        }
    }
}

void intersectIntervals(
    std::span<const DexelInterval> left,
    std::span<const DexelInterval> right,
    double tolerance,
    std::vector<DexelInterval>& output)
{
    output.clear();
    std::size_t leftIndex = 0u;
    std::size_t rightIndex = 0u;
    while (leftIndex < left.size() &&
           rightIndex < right.size()) {
        const double lower = std::max(
            left[leftIndex].lower,
            right[rightIndex].lower);
        const double upper = std::min(
            left[leftIndex].upper,
            right[rightIndex].upper);
        if (upper - lower > tolerance)
            output.push_back({lower, upper});
        if (left[leftIndex].upper <
            right[rightIndex].upper) {
            ++leftIndex;
        } else {
            ++rightIndex;
        }
    }
}

bool sameIntervals(
    std::span<const DexelInterval> left,
    std::span<const DexelInterval> right,
    double tolerance)
{
    if (left.size() != right.size())
        return false;
    for (std::size_t index = 0;
         index < left.size(); ++index) {
        if (std::abs(
                left[index].lower -
                right[index].lower) > tolerance ||
            std::abs(
                left[index].upper -
                right[index].upper) > tolerance) {
            return false;
        }
    }
    return true;
}

bool multiDexelTargetBoundaryCell(
    const DexelDisplayGrid& target,
    int i,
    int j,
    double tolerance)
{
    const auto intervals =
        target.intervalsAt(i, j);
    for (int dj = -1; dj <= 1; ++dj) {
        for (int di = -1; di <= 1; ++di) {
            if (di == 0 && dj == 0)
                continue;
            const int neighborI = i + di;
            const int neighborJ = j + dj;
            if (neighborI < 0 ||
                neighborJ < 0 ||
                neighborI >= target.nx ||
                neighborJ >= target.ny) {
                return true;
            }
            if (!sameIntervals(
                    intervals,
                    target.intervalsAt(
                        neighborI, neighborJ),
                    tolerance)) {
                return true;
            }
        }
    }
    return false;
}

bool appendCellIntervals(
    DexelDisplayGrid& grid,
    std::span<const DexelInterval> intervals,
    double cellArea,
    double& volume,
    std::string& error)
{
    if (grid.intervals.size() + intervals.size() >
        std::numeric_limits<std::uint32_t>::max()) {
        error =
            "multi-dexel difference interval count exceeds display capacity";
        return false;
    }
    for (const DexelInterval& interval : intervals) {
        grid.intervals.push_back(interval);
        volume +=
            (interval.upper - interval.lower) *
            cellArea;
    }
    grid.offsets.push_back(
        static_cast<std::uint32_t>(
            grid.intervals.size()));
    return true;
}

} // namespace oap::toolpath::zmap_detail
