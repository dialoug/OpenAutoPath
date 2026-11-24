#include "SurfaceFinishingMesh.h"
#include "toolpath/core/ToolpathOcctParallel.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <mutex>
#include <vector>
#ifdef OAP_FINISHING_TBB
#include <oneapi/tbb/task_arena.h>
#endif

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath::finishing_detail {
namespace {

gp_Pnt localPoint(const gp_Pnt& point, const gp_Ax3& frame)
{
    const gp_Vec displacement(frame.Location(), point);
    const gp_Pnt local(displacement.Dot(gp_Vec(frame.XDirection())),
                       displacement.Dot(gp_Vec(frame.YDirection())),
                       displacement.Dot(gp_Vec(frame.Direction())));
    if (!std::isfinite(local.X()) || !std::isfinite(local.Y()) ||
        !std::isfinite(local.Z()) || std::abs(local.X()) > 1.e9 ||
        std::abs(local.Y()) > 1.e9 || std::abs(local.Z()) > 1.e9)
        throw std::runtime_error("Target coordinates are non-finite or outside the supported range.");
    return local;
}

void includePoint(SurfaceMesh& mesh, const gp_Pnt& p)
{
    mesh.minX = std::min(mesh.minX, p.X());
    mesh.minY = std::min(mesh.minY, p.Y());
    mesh.minZ = std::min(mesh.minZ, p.Z());
    mesh.maxX = std::max(mesh.maxX, p.X());
    mesh.maxY = std::max(mesh.maxY, p.Y());
    mesh.maxZ = std::max(mesh.maxZ, p.Z());
}

} // namespace

SurfaceMesh stockEnvelope(const TopoDS_Shape& stock, const gp_Ax3& frame)
{
    if (stock.IsNull())
        throw std::runtime_error("Stock engraving requires a non-null planning stock.");
    Bnd_Box bounds;
    BRepBndLib::Add(stock, bounds, Standard_False);
    if (bounds.IsVoid() || bounds.IsOpen())
        throw std::runtime_error("Engraving stock requires a finite bounding envelope.");
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    SurfaceMesh result;
    for (int corner = 0; corner < 8; ++corner)
        includePoint(result, localPoint(gp_Pnt(corner & 1 ? xmax : xmin,
            corner & 2 ? ymax : ymin, corner & 4 ? zmax : zmin), frame));
    if (result.maxX - result.minX <= 1.e-9 || result.maxY - result.minY <= 1.e-9 ||
        result.maxZ - result.minZ <= 1.e-9)
        throw std::runtime_error("Engraving stock envelope must have positive XYZ dimensions.");
    return result;
}

void meshTarget(const TopoDS_Shape& target,
                const gp_Ax3& frame,
                const SurfaceFinishingParams& params,
                const ToolpathGenerationControl* control,
                SurfaceMesh& output, SurfaceFinishingMetrics& metrics)
{
    checkCancellation(control);
    // Copy geometry as well as topology: cleaning/meshing must never replace
    // cached triangulations shared by the interactive document.
    BRepBuilderAPI_Copy copier(target, Standard_True, Standard_False);
    if (!copier.IsDone() || copier.Shape().IsNull())
        throw std::runtime_error("Could not copy the target for surface meshing.");
    TopoDS_Shape snapshot = copier.Shape();
    BRepTools::Clean(snapshot);
    checkCancellation(control);
    // A tessellation can undershoot a curved surface's exact maximum. Use a
    // geometry-only conservative world box for the safe plane and seed height.
    // Add uses control-pole bounds for Bezier/BSpline surfaces, whereas
    // AddOptimal performs costly numerical extrema searches on dense reliefs.
    // Clearance needs containment rather than the smallest possible box;
    // XY raster bounds still come from the detailed mesh below.
    Bnd_Box bounds;
    BRepBndLib::Add(snapshot, bounds, Standard_False);
    if (bounds.IsVoid() || bounds.IsOpen())
        throw std::runtime_error("Engraving requires a finite target bounding box.");
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    for (int corner = 0; corner < 8; ++corner) {
        const gp_Pnt local = localPoint(
            gp_Pnt(corner & 1 ? xmax : xmin, corner & 2 ? ymax : ymin,
                   corner & 4 ? zmax : zmin), frame);
        output.minZ = std::min(output.minZ, local.Z());
        output.maxZ = std::max(output.maxZ, local.Z());
    }
    {
        std::size_t faceCount = 0;
        for (TopExp_Explorer faces(snapshot, TopAbs_FACE); faces.More(); faces.Next()) ++faceCount;
        const auto mesh = [&](bool parallel) {
            BRepMesh_IncrementalMesh mesher(snapshot, params.meshDeflection, Standard_False, 0.3, parallel);
            if (!mesher.IsDone()) throw std::runtime_error("Target surface meshing failed.");
        };
#ifdef OAP_FINISHING_TBB
        if (faceCount > 1 && !OSD_Parallel::ToUseOcctThreads()) {
            const auto extra = toolpathParallelismFor(faceCount) - 1;
            ToolpathParallelLease lease(control, extra, extra);
            metrics.meshingWorkerCount = lease.parallelism();
            oneapi::tbb::task_arena arena(static_cast<int>(lease.parallelism()));
            arena.execute([&] { mesh(lease.count() > 0); });
        } else
#endif
        {
            ToolpathOcctParallelLease lease(faceCount > 1, control);
            if (lease.enabled()) metrics.meshingWorkerCount = toolpathOcctParallelism();
            mesh(lease.enabled());
        }
        checkCancellation(control);
    }

    struct TriangleBlock {
        Handle(Poly_Triangulation) mesh;
        gp_Trsf transform;
        bool reversed;
        Standard_Integer first, last;
    };
    std::vector<TriangleBlock> blocks;
    std::size_t triangleCount = 0;
    for (TopExp_Explorer faces(snapshot, TopAbs_FACE); faces.More(); faces.Next()) {
        checkCancellation(control);
        const TopoDS_Face face = TopoDS::Face(faces.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation) mesh = BRep_Tool::Triangulation(face, location);
        // Silently omitting a failed face would also omit its collision envelope.
        if (mesh.IsNull() || mesh->NbTriangles() == 0)
            throw std::runtime_error("A target face could not be triangulated; no path was published.");
        if (static_cast<std::size_t>(mesh->NbTriangles()) > params.maxTriangles - triangleCount)
            throw std::runtime_error("Engraving triangle budget exceeded; increase mesh deflection.");
        triangleCount += static_cast<std::size_t>(mesh->NbTriangles());
        constexpr std::size_t blockSize = 1024;
        const auto count = static_cast<std::size_t>(mesh->NbTriangles());
        for (std::size_t begin = 0; begin < count; begin += blockSize)
            blocks.push_back({mesh, location.Transformation(), face.Orientation() == TopAbs_REVERSED,
                static_cast<Standard_Integer>(begin + 1), static_cast<Standard_Integer>(std::min(begin + blockSize, count))});
    }
    std::vector<SurfaceMesh> converted(blocks.size());
    const auto desired = std::max<std::size_t>(1, std::min(toolpathParallelismFor(blocks.size()),
        (triangleCount + 1023) / 1024));
    ToolpathParallelLease lease(control, desired - 1, desired - 1);
    metrics.conversionWorkerCount = lease.parallelism();
    std::mutex cancellationMutex;
    toolpathParallelForWorkers(blocks.size(), lease.parallelism(), [&](std::size_t blockIndex) {
        const auto& [mesh, transform, reversed, first, last] = blocks[blockIndex];
        auto& part = converted[blockIndex];
        if (control && control->isCancellationRequested) {
            std::lock_guard<std::mutex> lock(cancellationMutex);
            checkCancellation(control);
        }
        for (Standard_Integer offset = 0; offset <= last - first; ++offset) {
            Standard_Integer a, b, c;
            mesh->Triangle(first + offset).Get(a, b, c);
            if (reversed)
                std::swap(b, c);
            const gp_Pnt p = localPoint(mesh->Node(a).Transformed(transform), frame);
            const gp_Pnt q = localPoint(mesh->Node(b).Transformed(transform), frame);
            const gp_Pnt r = localPoint(mesh->Node(c).Transformed(transform), frame);
            // Include even degenerate vertices in the complete-model safe plane.
            includePoint(part, p);
            includePoint(part, q);
            includePoint(part, r);
            if (gp_Vec(p, q).Crossed(gp_Vec(p, r)).SquareMagnitude() <= 1.e-24)
                continue;
            part.surface.addTriangle(ocl::Triangle(
                ocl::Point(p.X(), p.Y(), p.Z()), ocl::Point(q.X(), q.Y(), q.Z()),
                ocl::Point(r.X(), r.Y(), r.Z())));
        }
    });
    for (auto& part : converted) {
        checkCancellation(control);
        includePoint(output, gp_Pnt(part.minX, part.minY, part.minZ));
        includePoint(output, gp_Pnt(part.maxX, part.maxY, part.maxZ));
        if (part.surface.tris.empty()) continue;
        output.surface.bb.addPoint(part.surface.bb.minpt);
        output.surface.bb.addPoint(part.surface.bb.maxpt);
        output.surface.tris.splice(output.surface.tris.end(), part.surface.tris);
    }
    if (output.surface.tris.empty())
        throw std::runtime_error("Target has no usable surface triangles.");
    if (output.maxX - output.minX <= 1.e-9 || output.maxY - output.minY <= 1.e-9)
        throw std::runtime_error("Target has no raster area in the machining XY plane.");
}

} // namespace oap::toolpath::finishing_detail
