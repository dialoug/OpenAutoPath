#include "SurfaceFinishing.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <list>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

#include <Standard_Failure.hxx>
#include <gp_Vec.hxx>
#include <ballcutter.hpp>
#include <bullcutter.hpp>
#include <clpoint.hpp>
#include <cylcutter.hpp>
#include <kdtree.hpp>

#include "detail/SurfaceFinishingMesh.h"
#include "toolpath/core/ToolpathParallel.h"

namespace oap::toolpath {
namespace {

using Clock = std::chrono::steady_clock;
using finishing_detail::checkCancellation;

double milliseconds(Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

bool finitePositive(double value)
{
    // OCL uses squared radii and ellipse constructions. Finite input alone is
    // insufficient: extremely large/small dimensions can silently overflow or
    // underflow intermediate contact calculations.
    return std::isfinite(value) && value >= 1.e-7 && value <= 1.e9;
}

void validate(const TopoDS_Shape& target, const gp_Ax3& frame,
              const SurfaceFinishingParams& p)
{
    if (target.IsNull())
        throw std::runtime_error("Engraving requires a non-null target.");
    if (!frame.Direct() || !std::isfinite(frame.Location().X()) ||
        !std::isfinite(frame.Location().Y()) || !std::isfinite(frame.Location().Z()) ||
        std::abs(frame.Location().X()) > 1.e9 || std::abs(frame.Location().Y()) > 1.e9 ||
        std::abs(frame.Location().Z()) > 1.e9)
        throw std::runtime_error("Engraving requires a finite right-handed machining frame.");
    for (const gp_Dir axis : {frame.XDirection(), frame.YDirection(), frame.Direction()})
        if (!std::isfinite(axis.X()) || !std::isfinite(axis.Y()) || !std::isfinite(axis.Z()))
            throw std::runtime_error("Machining frame contains a non-finite axis.");
    if (!finitePositive(p.diameter) || !finitePositive(p.cuttingLength) ||
        !finitePositive(p.stepOver) || !finitePositive(p.sampleSpacing) ||
        !finitePositive(p.meshDeflection) || !finitePositive(p.safeClearance) ||
        !finitePositive(p.feed) || !finitePositive(p.plungeFeed) ||
        !std::isfinite(p.allowance) || p.allowance < 0.0 || p.allowance > 1.e9 ||
        !std::isfinite(p.cornerRadius) || p.cornerRadius < 0.0 || p.cornerRadius > 1.e9 ||
        !finitePositive(p.maxStepDown) || !std::isfinite(p.roughingAllowance) ||
        p.roughingAllowance < 0 || p.roughingAllowance > 1.e9)
        throw std::runtime_error("Engraving dimensions, spacing, clearance and feeds must be within [1e-7, 1e9]; allowance and corner radius must be within [0, 1e9].");
    // Metadata indices are signed ints, and budgets must stay representable.
    if (p.maxSamples < 4 || p.maxSamples > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        p.maxTriangles == 0 || p.maxTriangles > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        p.maxMoves < 4 || p.maxMoves > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("Engraving sample/triangle budgets are invalid.");
    if (p.stepOver > p.diameter)
        throw std::runtime_error("Engraving step-over must not exceed cutter diameter.");
    if (p.mode != EngravingMachiningMode::FinishOnly &&
        p.mode != EngravingMachiningMode::StockRoughingAndFinishing)
        throw std::runtime_error("Unsupported engraving machining mode.");
    if (p.mode == EngravingMachiningMode::StockRoughingAndFinishing &&
        (p.maxStepDown > p.cuttingLength || p.roughingAllowance > p.maxStepDown))
        throw std::runtime_error("Stock engraving requires roughing allowance <= step-down <= cutting length.");
    switch (p.cutterKind) {
    case FinishingCutterKind::Ball:
        if (p.cuttingLength < p.diameter * 0.5)
            throw std::runtime_error("Ball cutter cutting length must cover its hemispherical tip.");
        break;
    case FinishingCutterKind::Flat:
        break;
    case FinishingCutterKind::Bull:
        if (!finitePositive(p.cornerRadius) || p.cornerRadius >= p.diameter * 0.5 ||
            p.cuttingLength < p.cornerRadius)
            throw std::runtime_error("Bull cutter requires 0 < corner radius < tool radius and sufficient cutting length.");
        break;
    default:
        throw std::runtime_error("Unsupported engraving cutter kind.");
    }
}

std::size_t sampleCount(double length, double spacing, std::size_t budget)
{
    const double ratio = length / spacing;
    // A rigid frame transformation can turn an exact integer ratio into
    // integer + a few ulps. Do not add a whole scan row for that roundoff.
    const double roundoff = 64.0 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::abs(ratio));
    const double intervals = std::ceil(ratio - roundoff);
    if (!std::isfinite(intervals) || intervals >= static_cast<double>(budget))
        throw std::runtime_error("Engraving sample budget exceeded; increase sampling spacing.");
    return static_cast<std::size_t>(std::max(1.0, intervals)) + 1u;
}

gp_Pnt worldPoint(const gp_Pnt& local, const gp_Ax3& frame)
{
    return frame.Location().Translated(gp_Vec(frame.XDirection()) * local.X() +
                                      gp_Vec(frame.YDirection()) * local.Y() +
                                      gp_Vec(frame.Direction()) * local.Z());
}

std::unique_ptr<ocl::MillingCutter> makeCutter(const SurfaceFinishingParams& p)
{
    switch (p.cutterKind) {
    case FinishingCutterKind::Ball:
        return std::make_unique<ocl::BallCutter>(p.diameter, p.cuttingLength);
    case FinishingCutterKind::Flat:
        return std::make_unique<ocl::CylCutter>(p.diameter, p.cuttingLength);
    case FinishingCutterKind::Bull:
        return std::make_unique<ocl::BullCutter>(p.diameter, p.cornerRadius, p.cuttingLength);
    }
    throw std::runtime_error("Unsupported engraving cutter kind.");
}

// The mesh outlives every query. OCL's KDTree copies bucket entries during
// construction and search, so store references instead of copying triangles.
struct TriangleRef {
    const ocl::Triangle& triangle;
    const ocl::Bbox& bb;
};

class ProjectionIndex {
public:
    explicit ProjectionIndex(const ocl::STLSurf& surface)
    {
        std::list<TriangleRef> entries;
        for (const auto& triangle : surface.tris)
            entries.push_back({triangle, triangle.bb});
        tree.setXYDimensions();
        tree.setBucketSize(8);
        tree.build(entries);
    }

    void drop(ocl::CLPoint& point, const ocl::MillingCutter& cutter)
    {
        // The pinned OCL search only reads the built tree and returns a new
        // list per query. Preserve PointDropCutter's candidate order and tests.
        const std::unique_ptr<std::list<TriangleRef>> hits(tree.search_cutter_overlap(&cutter, &point));
        for (const auto& hit : *hits) {
            if (cutter.overlaps(point, hit.triangle) && point.below(hit.triangle))
                cutter.dropCutter(point, hit.triangle);
        }
    }

private:
    ocl::KDTree<TriangleRef> tree;
};

struct RasterNode {
    double x = 0;
    std::optional<double> contact;
    bool startsRun = false;
};

struct AssembledRow {
    std::vector<ToolpathMove> moves;
    std::optional<gp_Pnt> firstSafe, lastSafe, incomingSafe;
    std::size_t segments = 0;
    std::size_t moveOffset = 0, segmentOffset = 0;
};

void mergeRows(std::vector<AssembledRow>& batch, std::size_t first, const gp_Ax3& frame,
               std::optional<gp_Pnt>& previousSafe, SurfaceFinishingResult& result,
               const std::function<void()>& poll)
{
    auto count = result.moves.size();
    for (auto& out : batch) {
        out.moveOffset = count;
        out.segmentOffset = result.metrics.segmentCount;
        if (out.firstSafe) {
            if (previousSafe && previousSafe->SquareDistance(*out.firstSafe) > 1.e-18) {
                out.incomingSafe = previousSafe;
                ++count;
            }
            previousSafe = out.lastSafe;
        }
        count += out.moves.size();
        if (count > result.effectiveParams.maxMoves)
            throw std::runtime_error("Engraving move budget exceeded; reduce layer or raster density.");
        result.metrics.segmentCount += out.segments;
    }
    result.moves.resize(count);
    result.primitives.resize(count);
    toolpathParallelFor(batch.size(), [&](std::size_t i) {
        const auto& out = batch[i];
        auto dest = out.moveOffset;
        const auto write = [&](ToolpathMove move) {
            move.loopSequence += static_cast<int>(out.segmentOffset);
            move.from = worldPoint(move.from, frame);
            move.to = worldPoint(move.to, frame);
            result.moves[dest] = move;
            result.primitives[dest++] = makeLinearPrimitive(move);
        };
        if (out.incomingSafe) {
            ToolpathMove rapid;
            rapid.type = ToolpathMoveType::Rapid;
            rapid.ring = static_cast<int>(first + i);
            rapid.loopSequence = 0;
            rapid.from = *out.incomingSafe;
            rapid.to = *out.firstSafe;
            write(rapid);
        }
        for (const auto& move : out.moves) {
            if (dest % 256 == 0) poll();
            write(move);
        }
    });
}

void assembleRaster(std::vector<std::vector<RasterNode>>& projectedRows, std::size_t layers,
                    std::size_t desiredWorkers, const finishing_detail::SurfaceMesh& domain,
                    const gp_Ax3& frame, const ToolpathGenerationControl* control,
                    SurfaceFinishingResult& result)
{
    const std::size_t rows = projectedRows.size();
    const std::size_t totalRows = rows + rows * (layers + 1);
    const double safeZ = result.safePlaneOffset;
    const auto& params = result.effectiveParams;
    using Coverage = std::vector<std::vector<std::pair<double, double>>>;
    Coverage previousCoverage;
    std::optional<gp_Pnt> previousSafe;
    ToolpathParallelScope assemblyScope(control);
    ToolpathParallelLease assemblyLease(control, desiredWorkers - 1, desiredWorkers - 1);
    const auto workers = assemblyLease.parallelism();
    result.metrics.assemblyWorkerCount = workers;
    std::mutex cancellationMutex;
    const auto poll = [&] {
        if (control && control->isCancellationRequested) {
            std::lock_guard<std::mutex> lock(cancellationMutex);
            checkCancellation(control);
        }
    };
    for (std::size_t pass = 0; pass <= layers; ++pass) {
        const bool roughing = pass < layers;
        const double layerZ = roughing ? std::max(domain.minZ,
            domain.maxZ - params.maxStepDown * static_cast<double>(pass + 1)) : 0;
        Coverage currentCoverage(roughing ? rows : 0);
        const auto passStartSegments = result.metrics.segmentCount;
        // Bound temporary row storage. Allocate final slices only after the
        // preceding safe point, segment IDs and move budget are known.
        for (std::size_t first = 0; first < rows; first += workers) {
            checkCancellation(control);
            std::vector<AssembledRow> batch(std::min(workers, rows - first));
            const auto remaining = params.maxMoves - result.moves.size();
            std::atomic_size_t staged{0};
            toolpathParallelFor(batch.size(), [&](std::size_t i) {
                const auto row = first + i;
                auto& out = batch[i];
                const auto append = [&](ToolpathMoveType type, const gp_Pnt& a, const gp_Pnt& b, double feed) {
                    if (a.SquareDistance(b) <= 1.e-18) return;
                    if (staged.fetch_add(1, std::memory_order_relaxed) >= remaining)
                        throw std::runtime_error("Engraving move budget exceeded; reduce layer or raster density.");
                    ToolpathMove move;
                    move.type = type;
                    move.from = a;
                    move.to = b;
                    move.feed = feed;
                    move.loopSequence = static_cast<int>(out.segments);
                    move.ring = static_cast<int>(row);
                    out.moves.push_back(move);
                };
                std::vector<gp_Pnt> run;
                const auto flush = [&] {
                    if (run.size() >= 2) {
                        const std::pair<double, double> extent = std::minmax(run.front().X(), run.back().X());
                        if (!previousCoverage.empty()) {
                            const auto& covered = previousCoverage[row];
                            if (std::none_of(covered.begin(), covered.end(), [&](const auto& interval) {
                                return extent.first >= interval.first - 1.e-8 && extent.second <= interval.second + 1.e-8;
                            }))
                                throw std::runtime_error("Layered engraving cannot establish prior-layer entry coverage across a discontinuity; no path was published.");
                        }
                        const gp_Pnt start(run.front().X(), run.front().Y(), safeZ);
                        const gp_Pnt end(run.back().X(), run.back().Y(), safeZ);
                        if (out.lastSafe) append(ToolpathMoveType::Rapid, *out.lastSafe, start, 0);
                        else out.firstSafe = start;
                        append(ToolpathMoveType::Plunge, start, run.front(), params.plungeFeed);
                        for (std::size_t n = 1; n < run.size(); ++n) {
                            if (n % 256 == 0) poll();
                            append(ToolpathMoveType::Cut, run[n - 1], run[n], params.feed);
                        }
                        append(ToolpathMoveType::Retract, run.back(), end, 0);
                        out.lastSafe = end;
                        ++out.segments;
                        if (roughing) currentCoverage[row].push_back(extent);
                    }
                    run.clear();
                };
                const double y = domain.minY + (domain.maxY - domain.minY) *
                    (static_cast<double>(row) / static_cast<double>(rows - 1u));
                const auto& nodes = projectedRows[row];
                for (std::size_t n = 0; n < nodes.size(); ++n) {
                    if (n % 256 == 0) poll();
                    const auto& node = nodes[n];
                    if (node.startsRun || (!roughing && !node.contact)) flush();
                    if (!roughing && !node.contact) continue;
                    const double targetZ = node.contact ? *node.contact + params.allowance +
                        (roughing ? params.roughingAllowance : 0.0) : domain.minZ;
                    const double z = roughing ? std::max(layerZ, targetZ) : targetZ;
                    if (!std::isfinite(z) || z >= safeZ)
                        throw std::runtime_error("Projected cutter contact exceeds the target safe plane.");
                    run.emplace_back(node.x, y, z);
                }
                flush();
                if (!roughing) std::vector<RasterNode>().swap(projectedRows[row]);
            });
            mergeRows(batch, first, frame, previousSafe, result, poll);
            result.metrics.scanLineCount += batch.size();
            if (control) control->progress(rows + result.metrics.scanLineCount, totalRows);
        }
        if (result.metrics.segmentCount == passStartSegments)
            throw std::runtime_error("An engraving pass has no continuous safe runs; no path was published.");
        if (roughing) {
            previousCoverage = std::move(currentCoverage);
            ++result.metrics.roughingLayerCount;
            result.metrics.roughingMoveCount = result.moves.size();
        }
    }
}

// Independent raster rows own their OCL state; layers are assembled in order.
void projectRaster(const finishing_detail::SurfaceMesh& mesh,
                   const gp_Ax3& frame, const SurfaceFinishingParams& params,
                   const ToolpathGenerationControl* control,
                   const finishing_detail::SurfaceMesh* stock,
                   SurfaceFinishingResult& result)
{
    const bool fromStock = params.mode == EngravingMachiningMode::StockRoughingAndFinishing;
    if (fromStock && !stock)
        throw std::runtime_error("Stock engraving requires declared planning stock.");
    if (fromStock) {
        const double tolerance = std::max(1.e-6, params.meshDeflection);
        if (mesh.minX < stock->minX - tolerance || mesh.maxX > stock->maxX + tolerance ||
            mesh.minY < stock->minY - tolerance || mesh.maxY > stock->maxY + tolerance ||
            mesh.minZ < stock->minZ - tolerance || mesh.maxZ > stock->maxZ + tolerance)
            throw std::runtime_error("Planning stock envelope does not contain the complete engraving target.");
    }
    const auto& domain = fromStock ? *stock : mesh;
    const std::size_t columns = sampleCount(domain.maxX - domain.minX, params.sampleSpacing, params.maxSamples);
    const std::size_t rows = sampleCount(domain.maxY - domain.minY, params.stepOver, params.maxSamples);
    if (rows > params.maxSamples / columns)
        throw std::runtime_error("Engraving sample budget exceeded; increase step-over or sample spacing.");
    // One midpoint per base edge is the minimum verification cost.
    const std::size_t baseSamples = rows * columns;
    if (rows * (columns - 1u) > params.maxSamples - baseSamples)
        throw std::runtime_error("Engraving verification sample budget exceeded; increase sample spacing.");
    std::size_t layers = 0;
    if (fromStock) {
        const double required = std::ceil((stock->maxZ - stock->minZ) / params.maxStepDown);
        // Each layer must traverse the full envelope; reject amplification
        // before allocating or projecting a potentially enormous layer list.
        if (!std::isfinite(required) || required < 1 ||
            required > static_cast<double>(params.maxMoves / baseSamples))
            throw std::runtime_error("Stock engraving layer/move budget exceeded; increase step-down or spacing.");
        layers = static_cast<std::size_t>(required);
    }

    checkCancellation(control);
    const double seedZ = mesh.minZ - std::max(params.diameter, 1.0);
    const double targetSafe = mesh.maxZ + params.allowance + (fromStock ? params.roughingAllowance : 0.0);
    const double safeZ = std::max(targetSafe, stock ? stock->maxZ : targetSafe) + params.safeClearance;
    if (!std::isfinite(seedZ) || !std::isfinite(safeZ) || safeZ <= mesh.maxZ + params.allowance)
        throw std::runtime_error("Engraving safe height or cutter dimensions exceed numeric range.");
    result.safePlaneOffset = safeZ;

    // Midpoint refinement reduces chord gouging on curved profiles. This remains
    // discrete verification, not a swept-tool or assembly collision certificate.
    // Unresolved discontinuities are separated by full safe-plane retractions.
    const double chordTolerance = std::max(1.e-7, params.meshDeflection);

    // Every stock layer uses the same XY partition. A clipped linear slope can
    // acquire a moving corner at each layer; independently refining those
    // corners produces incompatible gaps even though the target is unchanged.
    // Discover all phase-specific gaps first and preserve them at every height.
    const std::size_t desiredWorkers = std::min(toolpathParallelismFor(rows),
        std::max<std::size_t>(1u, baseSamples / 1024u));
    std::vector<std::vector<RasterNode>> projectedRows;
    const std::size_t templateRows = rows;
    const std::size_t totalRows = templateRows + rows * (layers + 1u);
    if (control) control->progress(0, totalRows);
    {
        projectedRows.resize(rows);
        std::atomic_uint64_t verificationWork{0};
        std::atomic_size_t samples{0}, nextRow{0};
        std::atomic_bool failed{false};
        std::mutex callbackMutex;
        std::size_t completedRows = 0;
        ToolpathGenerationControl rowControl;
        rowControl.isCancellationRequested = [&] {
            if (failed.load(std::memory_order_relaxed)) return true;
            if (!control || !control->isCancellationRequested) return false;
            // Serialize callbacks as callers may share state between progress
            // and cancellation. OCL work runs outside this short critical section.
            std::lock_guard<std::mutex> lock(callbackMutex);
            return control->cancelled();
        };
        const std::uint64_t verificationBudget =
            4u * (static_cast<std::uint64_t>(params.maxSamples) + params.maxMoves);
        const auto requiresRefinement = [&](double a, double middle, double b, double distance) {
            return std::abs(middle - (a + b) * 0.5) > chordTolerance ||
                std::abs(b - a) > distance * 4.0;
        };
        ToolpathParallelScope scope(control);
        ProjectionIndex index(mesh.surface);
        ToolpathParallelLease lease(control, desiredWorkers - 1u, desiredWorkers - 1u);
        result.metrics.projectionWorkerCount = lease.parallelism();
        std::exception_ptr projectionFailure;
        std::mutex failureMutex;
        toolpathParallelFor(lease.parallelism(), [&](std::size_t) {
          try {
            checkCancellation(&rowControl);
            auto workerCutter = makeCutter(params);
            std::map<double, std::optional<double>> heights;
            auto rowContact = [&](double x, double y) -> std::optional<double> {
                checkCancellation(&rowControl);
                if (fromStock) {
                    const auto cached = heights.find(x);
                    if (cached != heights.end()) return cached->second;
                }
                if (samples.fetch_add(1u, std::memory_order_relaxed) >= params.maxSamples)
                    throw std::runtime_error("Engraving adaptive verification sample budget exceeded.");
                ocl::CLPoint point(x, y, seedZ);
                index.drop(point, *workerCutter);
                const auto contact = point.getCC();
                if (!fromStock && contact.type == ocl::NONE) return std::nullopt;
                if (contact.type == ocl::ERROR || !std::isfinite(point.z))
                    throw std::runtime_error("OpenCAMLib returned an invalid cutter contact.");
                const std::optional<double> height = contact.type == ocl::NONE
                    ? std::nullopt : std::optional<double>(point.z);
                if (fromStock) heights.emplace(x, height);
                return height;
            };
          while (true) {
            const auto templateRow = nextRow.fetch_add(1u, std::memory_order_relaxed);
            if (templateRow >= rows) break;
            checkCancellation(&rowControl);
            heights.clear();
            const double y = domain.minY + (domain.maxY - domain.minY) *
                (static_cast<double>(templateRow) / static_cast<double>(rows - 1u));
            auto& nodes = projectedRows[templateRow];
            // Finish-only refinement retains the original midpoint decisions,
            // including safe retractions across missing/steep contact. Each row
            // writes only its own nodes; movement metadata is assigned at merge.
            if (!fromStock) {
                const auto finishNode = [&](double x) {
                    RasterNode node{x, rowContact(x, y), false};
                    if (node.contact && (!std::isfinite(*node.contact + params.allowance) ||
                        *node.contact + params.allowance >= safeZ))
                        throw std::runtime_error("Projected cutter contact exceeds the target safe plane.");
                    return node;
                };
                std::function<void(const RasterNode&, const RasterNode&, unsigned)> connect;
                connect = [&](const RasterNode& a, const RasterNode& b, unsigned depth) {
                    const auto middle = finishNode((a.x + b.x) * 0.5);
                    RasterNode endpoint = b;
                    if (!middle.contact) {
                        endpoint.startsRun = true;
                        nodes.push_back(endpoint);
                        return;
                    }
                    const bool split = requiresRefinement(*a.contact + params.allowance,
                        *middle.contact + params.allowance, *b.contact + params.allowance,
                        std::hypot(b.x - a.x, 0.0));
                    if (split && depth < 10u) {
                        connect(a, middle, depth + 1u);
                        connect(middle, b, depth + 1u);
                    } else {
                        endpoint.startsRun = split;
                        nodes.push_back(endpoint);
                    }
                };
                bool startsRun = true;
                for (std::size_t column = 0; column < columns; ++column) {
                    const auto ordered = templateRow % 2u ? columns - 1u - column : column;
                    const double x = domain.minX + (domain.maxX - domain.minX) *
                        (static_cast<double>(ordered) / static_cast<double>(columns - 1u));
                    auto node = finishNode(x);
                    if (!node.contact) {
                        startsRun = true;
                    } else if (startsRun) {
                        node.startsRun = true;
                        nodes.push_back(node);
                        startsRun = false;
                    } else {
                        const auto previous = nodes.back();
                        connect(previous, node, 0);
                    }
                }
            } else {
                std::function<void(const RasterNode&, const RasterNode&, unsigned)> refine;
                refine = [&](const RasterNode& a, const RasterNode& b, unsigned depth) {
                    checkCancellation(&rowControl);
                    const double middleX = (a.x + b.x) * 0.5;
                    const RasterNode middle{middleX, rowContact(middleX, y), false};
                    const double distance = std::abs(b.x - a.x);
                    // Contact/no-contact boundaries are shared by roughing and
                    // finishing. Never remove the no-contact nodes and then join
                    // across the resulting gap in the final finishing pass.
                    bool split = a.contact.has_value() != middle.contact.has_value() ||
                        b.contact.has_value() != middle.contact.has_value();
                    if (!split && a.contact)
                        split = requiresRefinement(*a.contact, *middle.contact, *b.contact, distance);
                    const auto protectedHeight = [&](const RasterNode& node) {
                        return node.contact ? *node.contact + params.allowance + params.roughingAllowance : stock->minZ;
                    };
                    const double protectedA = protectedHeight(a);
                    const double protectedMiddle = protectedHeight(middle);
                    const double protectedB = protectedHeight(b);
                    for (std::size_t layer = 0; layer < layers && !split; ++layer) {
                        checkCancellation(&rowControl);
                        if (verificationWork.fetch_add(1u, std::memory_order_relaxed) >= verificationBudget)
                            throw std::runtime_error("Engraving shared-template verification budget exceeded; reduce layer or raster density.");
                        const double z = std::max(stock->minZ,
                            stock->maxZ - params.maxStepDown * static_cast<double>(layer + 1u));
                        split = requiresRefinement(std::max(z, protectedA),
                            std::max(z, protectedMiddle), std::max(z, protectedB), distance);
                    }
                    if (split && depth < 10u) {
                        refine(a, middle, depth + 1u);
                        refine(middle, b, depth + 1u);
                    } else {
                        RasterNode endpoint = b;
                        endpoint.startsRun = split;
                        nodes.push_back(std::move(endpoint));
                    }
                };
                for (std::size_t column = 0; column < columns; ++column) {
                    const std::size_t ordered = templateRow % 2u ? columns - 1u - column : column;
                    const double x = domain.minX + (domain.maxX - domain.minX) *
                        (static_cast<double>(ordered) / static_cast<double>(columns - 1u));
                    RasterNode node{x, rowContact(x, y), nodes.empty()};
                    if (nodes.empty()) nodes.push_back(std::move(node));
                    else {
                        // Recursive refinement may reallocate the destination.
                        const RasterNode previous = nodes.back();
                        refine(previous, node, 0);
                    }
                }
            }
            std::lock_guard<std::mutex> lock(callbackMutex);
            ++completedRows;
            if (control) control->progress(completedRows, totalRows);
          }
          } catch (...) {
            // Record the originating failure before waking peers, so a budget
            // error cannot be replaced by a peer's cooperative cancellation.
            std::lock_guard<std::mutex> lock(failureMutex);
            if (!projectionFailure) projectionFailure = std::current_exception();
            failed.store(true, std::memory_order_relaxed);
          }
        });
        result.metrics.sampleCount = std::min(params.maxSamples, samples.load(std::memory_order_relaxed));
        if (projectionFailure) std::rethrow_exception(projectionFailure);
    }
    assembleRaster(projectedRows, layers, desiredWorkers, domain, frame, control, result);
    checkCancellation(control);
    if (result.metrics.segmentCount == 0 ||
        std::none_of(result.moves.begin(), result.moves.end(), [](const ToolpathMove& move) {
            return move.type == ToolpathMoveType::Cut;
        }))
        throw std::runtime_error("No continuous cutter-contact runs were found for this target and cutter.");
}

} // namespace

SurfaceFinishingResult SurfaceFinishing::generate(
    const TopoDS_Shape& target, const gp_Ax3& machiningFrame,
    const SurfaceFinishingParams& params, const ToolpathGenerationControl* control,
    const TopoDS_Shape* stock) const
{
    SurfaceFinishingResult result;
    result.effectiveParams = params;
    const auto start = Clock::now();
    try {
        checkCancellation(control);
        validate(target, machiningFrame, params);
        if (params.mode == EngravingMachiningMode::StockRoughingAndFinishing && (!stock || stock->IsNull()))
            throw std::runtime_error("Stock engraving requires declared planning stock.");
        std::optional<finishing_detail::SurfaceMesh> stockBounds;
        if (stock) stockBounds.emplace(finishing_detail::stockEnvelope(*stock, machiningFrame));
        checkCancellation(control);
        finishing_detail::SurfaceMesh mesh;
        const auto meshStart = Clock::now();
        finishing_detail::meshTarget(target, machiningFrame, params, control, mesh, result.metrics);
        result.metrics.meshMilliseconds = milliseconds(meshStart);
        result.metrics.triangleCount = mesh.surface.tris.size();
        const auto projectionStart = Clock::now();
        projectRaster(mesh, machiningFrame, params, control, stockBounds ? &*stockBounds : nullptr, result);
        result.metrics.projectionMilliseconds = milliseconds(projectionStart);
        checkCancellation(control);
        result.warning = "Sampled OpenCAMLib engraving candidate for reference NC only. "
            "Midpoint refinement is not continuous swept-tool collision verification. "
            "Exact stock occupancy, fixtures, shank and holder are not checked; vertical walls, undercuts "
            "and sub-sample features may remain unmachined. Allowance is along local +Z, "
            "not surface-normal allowance. Start from the declared safe position.";
        if (params.mode == EngravingMachiningMode::StockRoughingAndFinishing)
            result.warning += " Layered passes scan the declared stock bounding envelope, not exact residual stock. "
                "The final rough pass follows target plus roughing allowance before finishing. "
                "Step-down limits the nominal sampled envelope; inter-row scallops and residual engagement require IPW verification.";
        result.ok = true;
    } catch (const finishing_detail::Cancelled&) {
        result.cancelled = true;
        result.error = "Engraving cancelled.";
    } catch (const Standard_Failure& failure) {
        const char* message = failure.GetMessageString();
        result.error = std::string("Engraving geometry failed: ") +
            (message ? message : "OCCT failure");
    } catch (const std::exception& failure) {
        result.error = failure.what();
    } catch (...) {
        result.error = "Engraving failed with an unknown error.";
    }
    if (!result.ok) {
        result.moves.clear();
        result.primitives.clear();
    }
    result.metrics.totalMilliseconds = milliseconds(start);
    return result;
}

ToolpathParams SurfaceFinishingResult::toolpathParams() const
{
    ToolpathParams params;
    params.tool.diameter = effectiveParams.diameter;
    params.tool.cornerRadius = effectiveParams.cutterKind == FinishingCutterKind::Ball
        ? effectiveParams.diameter * 0.5 : effectiveParams.cutterKind == FinishingCutterKind::Bull
        ? effectiveParams.cornerRadius : 0.0;
    params.tool.cuttingEdgeLength = effectiveParams.cuttingLength;
    params.tool.horizFeed = effectiveParams.feed;
    params.tool.vertFeed = effectiveParams.plungeFeed;
    params.depth.safeHeight = effectiveParams.safeClearance;
    params.depth.clearanceHeight = effectiveParams.safeClearance;
    params.clearing.pattern = ClearingPattern::ZigZag;
    params.clearing.stepOverPercent = 100.0 * effectiveParams.stepOver / effectiveParams.diameter;
    return params;
}

ToolpathProgram SurfaceFinishingResult::toProgram(
    const std::string& operationId, const ToolpathToolContext& tool) const
{
    ToolpathProgram program;
    if (!ok || cancelled || moves.empty())
        return program;
    ToolpathOperation operation;
    operation.operationId = operationId;
    operation.type = ToolpathOperationType::SurfaceFinishing;
    operation.tool = tool;
    operation.metadata["algorithm"] = effectiveParams.mode == EngravingMachiningMode::StockRoughingAndFinishing
        ? kOpenCamLibStockEngravingAlgorithmId : kOpenCamLibSurfaceFinishingAlgorithmId;
    operation.metadata["machiningMode"] = effectiveParams.mode == EngravingMachiningMode::StockRoughingAndFinishing
        ? "stock-layered-envelope-then-finishing" : "finishing-only";
    operation.metadata["roughingLayers"] = std::to_string(metrics.roughingLayerCount);
    operation.metadata["roughingMoves"] = std::to_string(metrics.roughingMoveCount);
    // FeatureKind currently describes recognized holes/pockets only. Its
    // default field is unused here; operation.type is the authoritative kind.
    operation.metadata["sourceScope"] = "complete-target-shape-not-recognized-feature";
    operation.metadata["quality"] = "sampled-candidate-reference-only";
    operation.metadata["allowanceMode"] = "local-positive-z";
    operation.metadata["collisionValidation"] = "sampled-midpoint-only-no-stock-fixture-holder";
    operation.metadata["warning"] = warning;
    operation.moves = moves;
    operation.primitives = primitives;
    program.operations.push_back(std::move(operation));
    return program;
}

} // namespace oap::toolpath
