#pragma once

#include <limits>
#include <stlsurf.hpp>

#include "toolpath/finishing/SurfaceFinishing.h"

namespace oap::toolpath::finishing_detail {

struct Cancelled {};

inline void checkCancellation(const ToolpathGenerationControl* control)
{
    if (control && control->cancelled())
        throw Cancelled{};
}

struct SurfaceMesh {
    ocl::STLSurf surface;
    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double minZ = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    double maxZ = -std::numeric_limits<double>::infinity();
};

void meshTarget(const TopoDS_Shape& target,
                const gp_Ax3& frame,
                const SurfaceFinishingParams& params,
                const ToolpathGenerationControl* control,
                SurfaceMesh& output);

// Geometry-only conservative stock bounds in the requested machining frame.
// The returned object contains bounds only; no stock triangles are generated.
SurfaceMesh stockEnvelope(const TopoDS_Shape& stock, const gp_Ax3& frame);

} // namespace oap::toolpath::finishing_detail
