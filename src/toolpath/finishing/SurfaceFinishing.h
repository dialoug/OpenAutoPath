// OpenCAMLib fixed-axis surface finishing. Generates sampled candidate paths;
// it does not certify stock/fixture/holder clearance or publish a formal Job.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>
#include <gp_Ax3.hxx>

#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathProgram.h"

namespace oap::toolpath {

inline constexpr char kOpenCamLibSurfaceFinishingAlgorithmId[] =
    "opencamlib.surface.parallel";
inline constexpr char kOpenCamLibStockEngravingAlgorithmId[] =
    "opencamlib.engraving.stock-raster.v1";

enum class FinishingCutterKind { Ball, Flat, Bull };
enum class EngravingMachiningMode { FinishOnly = 0, StockRoughingAndFinishing = 1 };

struct SurfaceFinishingParams {
    EngravingMachiningMode mode = EngravingMachiningMode::FinishOnly;
    FinishingCutterKind cutterKind = FinishingCutterKind::Ball;
    double diameter = 6.0;
    double cuttingLength = 30.0;
    double cornerRadius = 1.0; // Bull only, strictly between zero and radius.
    double stepOver = 1.0;
    double sampleSpacing = 0.5;
    double meshDeflection = 0.05;
    double allowance = 0.0; // Vertical stock allowance along local +Z.
    double safeClearance = 5.0; // Above the complete target's local Z maximum.
    double feed = 600.0;
    double plungeFeed = 150.0;
    double maxStepDown = 1.0;
    double roughingAllowance = 0.2; // Additional local +Z allowance before finishing.
    std::size_t maxMoves = 2000000u;
    std::size_t maxSamples = 1000000u;
    std::size_t maxTriangles = 1000000u;
};

struct SurfaceFinishingMetrics {
    std::size_t projectionWorkerCount = 1;
    std::size_t triangleCount = 0;
    std::size_t sampleCount = 0; // Unique OCL queries, including midpoint verification.
    std::size_t scanLineCount = 0;
    std::size_t segmentCount = 0; // Separate contact runs; each retracts fully.
    std::size_t roughingLayerCount = 0;
    std::size_t roughingMoveCount = 0;
    double meshMilliseconds = 0;
    double projectionMilliseconds = 0;
    double totalMilliseconds = 0;
};

struct SurfaceFinishingResult {
    bool ok = false;
    bool cancelled = false;
    std::string error;
    std::string warning;
    SurfaceFinishingParams effectiveParams;
    SurfaceFinishingMetrics metrics;
    std::vector<ToolpathMove> moves;
    std::vector<ToolpathPrimitive> primitives;
    double safePlaneOffset = 0; // In the request's machining frame.

    ToolpathParams toolpathParams() const;
    ToolpathProgram toProgram(
        const std::string& operationId = "OCL_FINISH1",
        const ToolpathToolContext& tool = {}) const;
};

class SurfaceFinishing {
public:
    // All target faces participate in tool projection. Meshing uses a deep copy
    // and never changes the input's cached triangulation. Local XY raster rows
    // run along X, alternate direction and retract above target and supplied stock.
    // StockRoughingAndFinishing requires world-space stock. Its conservative
    // envelope is scanned in bounded layers; this is not exact residual clipping.
    SurfaceFinishingResult generate(
        const TopoDS_Shape& target,
        const gp_Ax3& machiningFrame,
        const SurfaceFinishingParams& params,
        const ToolpathGenerationControl* control = nullptr,
        const TopoDS_Shape* stock = nullptr) const;
};

} // namespace oap::toolpath
