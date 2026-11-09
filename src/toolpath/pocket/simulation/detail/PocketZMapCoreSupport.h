// Private PocketZMapCoreSupport contract; not part of the module public API.
#pragma once

#include <chrono>
#include <array>
#include <cstdint>
#include <limits>
#include <string_view>
#include "toolpath/pocket/simulation/PocketZMapComparison.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/pocket/simulation/StockTargetCache.h"

namespace oap::toolpath::zmap_detail {

using SteadyClock = std::chrono::steady_clock;

double elapsedMilliseconds(const SteadyClock::time_point& start);

class ScopedElapsedMilliseconds {
public:
    explicit ScopedElapsedMilliseconds(double& output);

    ~ScopedElapsedMilliseconds();

private:
    double& m_output;
    SteadyClock::time_point m_start;
};

struct LocalPoint {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
};

struct Interval {
    double lower = std::numeric_limits<double>::infinity();
    double upper = -std::numeric_limits<double>::infinity();
};

using TargetField = StockTargetHeightField;

class ContentFingerprint {
public:
    void add(std::uint64_t value) noexcept;

    void add(double value) noexcept;

    void add(std::string_view value) noexcept;

    std::string finish() const;

private:
    std::uint64_t m_first =
        1469598103934665603ull;
    std::uint64_t m_second =
        0x6a09e667f3bcc909ull;
};

std::string displaySourceFingerprint(
    const PocketZMapSimulationData& data,
    const TargetField& target);

bool cancelled(const ToolpathGenerationControl* control);

void addDiagnostic(
    StockComparisonResult& result,
    ToolpathDiagnosticCode code,
    ToolpathDiagnosticSeverity severity,
    std::string message);

void addAttributionDiagnostic(
    StockOvercutAttributionResult& result,
    ToolpathDiagnosticCode code,
    ToolpathDiagnosticSeverity severity,
    std::string message,
    std::string operationId = {});

std::size_t offset(
    const PocketZMapSimulationData& data,
    int i,
    int j);

double cellU0(const PocketZMapSimulationData& data, int i);

double cellU1(const PocketZMapSimulationData& data, int i);

double cellV0(const PocketZMapSimulationData& data, int j);

double cellV1(const PocketZMapSimulationData& data, int j);

double cellU(const PocketZMapSimulationData& data, int i);

double cellV(const PocketZMapSimulationData& data, int j);

double cellArea(
    const PocketZMapSimulationData& data,
    int i,
    int j);

LocalPoint project(
    const PocketZMapSimulationData& data,
    const gp_Pnt& point);

int coordinateToI(
    const PocketZMapSimulationData& data,
    double u);

int coordinateToJ(
    const PocketZMapSimulationData& data,
    double v);

bool finiteInterval(
    double lower,
    double upper,
    double tolerance);

bool sameFiniteInterval(
    double leftLower,
    double leftUpper,
    double rightLower,
    double rightUpper,
    double tolerance);

bool targetBoundaryCell(
    const PocketZMapSimulationData& data,
    const TargetField& target,
    std::size_t cell,
    double tolerance);

void setInterval(
    Interval& output,
    double lower,
    double upper,
    double tolerance);

bool shapeBounds(
    const TopoDS_Shape& shape,
    std::array<double, 6>& values);

bool targetProjectionFitsGrid(
    const PocketZMapSimulationData& data,
    const std::array<double, 6>& bounds,
    double tolerance);

void rasterizeTriangle(
    const PocketZMapSimulationData& data,
    const LocalPoint& a,
    const LocalPoint& b,
    const LocalPoint& c,
    TargetField& target);

bool rasterizeTarget(
    const PocketZMapSimulationData& data,
    const TopoDS_Shape& targetShape,
    const StockComparisonOptions& options,
    const ToolpathGenerationControl* control,
    TargetField& target,
    std::array<double, 6>& targetBounds,
    std::string& error);

double intervalVolume(
    const PocketZMapSimulationData& data,
    const std::vector<Interval>& intervals);

bool sameMultiDexelGrid(
    const MultiDexelStock& left,
    const MultiDexelStock& right,
    double tolerance);

DexelDisplayGrid gridLike(
    const MultiDexelStock& stock,
    double tolerance);

double multiDexelCellArea(
    const MultiDexelStock& stock,
    int i,
    int j);

void subtractIntervals(
    std::span<const DexelInterval> material,
    std::span<const DexelInterval> cover,
    double tolerance,
    std::vector<DexelInterval>& output);

void intersectIntervals(
    std::span<const DexelInterval> left,
    std::span<const DexelInterval> right,
    double tolerance,
    std::vector<DexelInterval>& output);

bool sameIntervals(
    std::span<const DexelInterval> left,
    std::span<const DexelInterval> right,
    double tolerance);

bool multiDexelTargetBoundaryCell(
    const DexelDisplayGrid& target,
    int i,
    int j,
    double tolerance);

bool appendCellIntervals(
    DexelDisplayGrid& grid,
    std::span<const DexelInterval> intervals,
    double cellArea,
    double& volume,
    std::string& error);

} // namespace oap::toolpath::zmap_detail
