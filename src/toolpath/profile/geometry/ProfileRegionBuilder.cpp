// ProfileRegionBuilder 实现（oap_toolpath_profile）。
// 识别外轮廓源线框、加工平面与逐层截面，见 ProfileRegionBuilder.h。
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <Bnd_Box.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>

#include "analysis/StepIndexer.h"

namespace oap::toolpath {
namespace {

bool fail(std::string* error, const char* message)
{
    if (error)
        *error = message;
    return false;
}

bool planeAndOutwardNormal(
    const TopoDS_Face& face, gp_Pln& plane, gp_Dir& normal)
{
    BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Plane)
        return false;
    plane = surface.Plane();
    normal = plane.Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED)
        normal.Reverse();
    return true;
}

bool belongsAsOuterWire(
    const TopoDS_Face& face, const TopoDS_Wire& wire)
{
    TopoDS_Wire outer;
    try {
        outer = BRepTools::OuterWire(face);
    } catch (const Standard_Failure&) {
        return false;
    }
    return !outer.IsNull() && outer.IsSame(wire);
}

bool isSupportedWall(
    const TopoDS_Face& face, const gp_Dir& toolAxis, double tolerance)
{
    try {
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() == GeomAbs_Plane) {
            const double alignment = std::abs(
                surface.Plane().Axis().Direction().Dot(toolAxis));
            // 水平终端面由深度计算消费，不属于侧壁；真正的侧壁必须与刀轴平行。
            return alignment <= tolerance || alignment >= 1.0 - tolerance;
        }
        if (surface.GetType() == GeomAbs_Cylinder) {
            return std::abs(surface.Cylinder().Axis().Direction().Dot(toolAxis)) >=
                1.0 - tolerance;
        }
    } catch (const Standard_Failure&) {
        return false;
    }
    // 第一版明确拒绝锥面、自由曲面和倒扣，避免把根轮廓直接投影后过切。
    return false;
}

bool isBoundingOuterPlane(
    const TopoDS_Face& face,
    const Bnd_Box& materialBounds,
    double tolerance)
{
    if (materialBounds.IsVoid())
        return false;
    try {
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() != GeomAbs_Plane)
            return false;
        const gp_Dir normal = surface.Plane().Axis().Direction();
        int axis = 0;
        double alignment = std::abs(normal.X());
        if (std::abs(normal.Y()) > alignment) {
            axis = 1;
            alignment = std::abs(normal.Y());
        }
        if (std::abs(normal.Z()) > alignment) {
            axis = 2;
            alignment = std::abs(normal.Z());
        }
        if (alignment < 1.0 - tolerance)
            return false;

        Standard_Real xmin = 0.0;
        Standard_Real ymin = 0.0;
        Standard_Real zmin = 0.0;
        Standard_Real xmax = 0.0;
        Standard_Real ymax = 0.0;
        Standard_Real zmax = 0.0;
        materialBounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const gp_Pnt location = surface.Plane().Location();
        const double coordinate = axis == 0
            ? location.X() : axis == 1 ? location.Y() : location.Z();
        const double minimum = axis == 0 ? xmin : axis == 1 ? ymin : zmin;
        const double maximum = axis == 0 ? xmax : axis == 1 ? ymax : zmax;
        return std::abs(coordinate - minimum) <= tolerance ||
            std::abs(coordinate - maximum) <= tolerance;
    } catch (const Standard_Failure&) {
        return false;
    }
}

double projectedDepth(
    const std::vector<TopoDS_Face>& faces,
    const gp_Pnt& origin,
    const gp_Dir& downDir,
    double deflection)
{
    double depth = 0.0;
    const gp_Vec axis(downDir);
    const double defl = std::max(deflection, Precision::Confusion());
    const auto consider = [&](const gp_Pnt& point) {
        depth = std::max(depth, gp_Vec(origin, point).Dot(axis));
    };
    for (const TopoDS_Face& face : faces) {
        for (TopExp_Explorer explorer(face, TopAbs_EDGE);
             explorer.More(); explorer.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
            try {
                BRepAdaptor_Curve curve(edge);
                GCPnts_QuasiUniformDeflection discretized(curve, defl);
                if (discretized.IsDone()) {
                    for (int index = 1; index <= discretized.NbPoints(); ++index)
                        consider(discretized.Value(index));
                    continue;
                }
            } catch (const Standard_Failure&) {
                // 回退到拓扑顶点。
            }
            TopoDS_Vertex first;
            TopoDS_Vertex last;
            TopExp::Vertices(edge, first, last, Standard_True);
            if (!first.IsNull()) consider(BRep_Tool::Pnt(first));
            if (!last.IsNull()) consider(BRep_Tool::Pnt(last));
        }
    }
    return depth;
}

Loop2d rectangularStockBoundary(
    const ProfileRegion& region,
    const ProfileRegionBuildOptions& options)
{
    Loop2d result;
    if (region.boundary.points.size() < 3)
        return result;

    double minX = region.boundary.points.front().X();
    double maxX = minX;
    double minY = region.boundary.points.front().Y();
    double maxY = minY;
    const auto consider = [&](const gp_Pnt2d& point) {
        minX = std::min(minX, point.X());
        maxX = std::max(maxX, point.X());
        minY = std::min(minY, point.Y());
        maxY = std::max(maxY, point.Y());
    };
    for (const gp_Pnt2d& point : region.boundary.points)
        consider(point);

    // 根外环可能比后续“只增不减”的累计截面更小。毛坯外框必须至少
    // 覆盖整个成品实体的固定刀轴投影，否则扩大的下层轮廓会落到毛坯外。
    Bnd_Box materialBounds;
    if (!region.protectedMaterialShape.IsNull())
        BRepBndLib::Add(region.protectedMaterialShape, materialBounds);
    if (!materialBounds.IsVoid()) {
        Standard_Real xmin = 0.0;
        Standard_Real ymin = 0.0;
        Standard_Real zmin = 0.0;
        Standard_Real xmax = 0.0;
        Standard_Real ymax = 0.0;
        Standard_Real zmax = 0.0;
        materialBounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        for (double x : {static_cast<double>(xmin),
                         static_cast<double>(xmax)}) {
            for (double y : {static_cast<double>(ymin),
                             static_cast<double>(ymax)}) {
                for (double z : {static_cast<double>(zmin),
                                 static_cast<double>(zmax)}) {
                    consider(OcctAreaAdapter::projectPoint(
                        gp_Pnt(x, y, z), region.frame));
                }
            }
        }
    }

    // X/Y/Z 毛坯余量在世界坐标中定义。把局部截面两个基向量上的投影
    // 累加为保守外扩量，适用于六个正交装夹方向及自定义刚性工作坐标系。
    const gp_Dir xDirection = region.frame.XDirection();
    const gp_Dir yDirection = region.frame.YDirection();
    const double marginU =
        std::abs(xDirection.X()) * std::max(0.0, options.stockMarginX) +
        std::abs(xDirection.Y()) * std::max(0.0, options.stockMarginY) +
        std::abs(xDirection.Z()) * std::max(0.0, options.stockMarginZ);
    const double marginV =
        std::abs(yDirection.X()) * std::max(0.0, options.stockMarginX) +
        std::abs(yDirection.Y()) * std::max(0.0, options.stockMarginY) +
        std::abs(yDirection.Z()) * std::max(0.0, options.stockMarginZ);
    minX -= marginU;
    maxX += marginU;
    minY -= marginV;
    maxY += marginV;
    result.points = {
        gp_Pnt2d(minX, minY), gp_Pnt2d(maxX, minY),
        gp_Pnt2d(maxX, maxY), gp_Pnt2d(minX, maxY)};
    result.closed = true;
    result.signedArea = (maxX - minX) * (maxY - minY);
    return result;
}

} // namespace

std::optional<ProfileRegion> ProfileRegionBuilder::buildOne(
    const RootOuterWireProfileSource& source,
    const StepIndexer& indexer,
    const ProfileRegionBuildOptions& options,
    std::string* error) const
{
    if (error) error->clear();
    if (source.rootFaceId.empty() || source.rootWireId.empty()) {
        fail(error, "profile root face or outer wire id is missing");
        return std::nullopt;
    }
    const TopoDS_Face* rootFace = indexer.faceForId(source.rootFaceId);
    const TopoDS_Wire* rootWire = indexer.wireForId(source.rootWireId);
    if (!rootFace || !rootWire || rootFace->IsNull() || rootWire->IsNull()) {
        fail(error, "profile root face or outer wire geometry was not found");
        return std::nullopt;
    }
    if (!belongsAsOuterWire(*rootFace, *rootWire)) {
        fail(error, "profile root wire is not the root face outer wire");
        return std::nullopt;
    }

    gp_Pln rootPlane;
    gp_Dir outward;
    if (!planeAndOutwardNormal(*rootFace, rootPlane, outward)) {
        fail(error, "profile root face is not planar");
        return std::nullopt;
    }

    std::vector<TopoDS_Face> faces;
    faces.reserve(source.sourceFaceIds.size());
    for (const std::string& faceId : source.sourceFaceIds) {
        if (faceId == source.rootFaceId)
            continue;
        const TopoDS_Face* face = indexer.faceForId(faceId);
        if (face && !face->IsNull())
            faces.push_back(*face);
    }
    if (faces.empty()) {
        fail(error, "profile source has no side-wall geometry");
        return std::nullopt;
    }

    TopoDS_Shape protectedMaterialShape;
    Bnd_Box materialBounds;
    if (const TopoDS_Solid* solid =
            indexer.solidForFaceId(source.rootFaceId)) {
        protectedMaterialShape = *solid;
        BRepBndLib::Add(protectedMaterialShape, materialBounds);
    }

    TopoDS_Compound sectionShape;
    BRep_Builder sectionBuilder;
    sectionBuilder.MakeCompound(sectionShape);
    for (const TopoDS_Face& face : faces)
        sectionBuilder.Add(sectionShape, face);

    const double angularTolerance = std::max(1.0e-6, options.tolerance * 10.0);
    const double boundsTolerance = std::max(
        options.discretizationDeflection,
        options.tolerance * 10.0);
    std::vector<TopoDS_Face> featureFaces;
    featureFaces.reserve(faces.size());
    const std::unordered_set<std::string> machiningFaceIds(
        source.machiningFaceIds.begin(), source.machiningFaceIds.end());
    const std::unordered_set<std::string> mainOuterFaceIds(
        source.mainOuterFaceIds.begin(), source.mainOuterFaceIds.end());
    for (const std::string& faceId : source.sourceFaceIds) {
        if (faceId == source.rootFaceId ||
            mainOuterFaceIds.contains(faceId) ||
            (!machiningFaceIds.empty() &&
             !machiningFaceIds.contains(faceId))) {
            continue;
        }
        const TopoDS_Face* face = indexer.faceForId(faceId);
        const bool useBoundsFallback = mainOuterFaceIds.empty();
        if (face && !face->IsNull() &&
            (!useBoundsFallback ||
             !isBoundingOuterPlane(
                 *face, materialBounds, boundsTolerance))) {
            featureFaces.push_back(*face);
        }
    }
    // 其它包围盒主面只是定义基础块体，不应把某个外环局部特征的加工深度
    // 拉到零件全跨度。简单直壁件没有额外特征时仍回退到全部相邻面。
    const std::vector<TopoDS_Face>& machiningFaces =
        featureFaces.empty() ? faces : featureFaces;
    const bool requiresMaterialSections = !featureFaces.empty() ||
        std::any_of(
            machiningFaces.begin(), machiningFaces.end(),
            [&](const TopoDS_Face& face) {
                return !isSupportedWall(face, outward, angularTolerance);
            });

    const gp_Dir downDir = outward.Reversed();
    const double depth = projectedDepth(
        machiningFaces, rootPlane.Location(), downDir,
        options.discretizationDeflection);
    if (!std::isfinite(depth) ||
        depth <= std::max(options.tolerance, Precision::Confusion())) {
        fail(error, "profile side walls have no positive depth");
        return std::nullopt;
    }

    gp_Pnt deepestOrigin = rootPlane.Location();
    deepestOrigin.Translate(gp_Vec(downDir) * depth);
    ProfileRegion region;
    region.source = source;
    region.openDir = outward;
    region.frame = gp_Ax3(
        deepestOrigin, outward, rootPlane.Position().XDirection());
    region.depth = depth;
    region.useMaterialSectionBoundaryAtEveryDepth =
        requiresMaterialSections;
    region.sectionTolerance = options.tolerance;
    region.sectionDeflection = options.discretizationDeflection;
    region.boundary = OcctAreaAdapter::wireToLoop2d(
        *rootWire, region.frame,
        options.discretizationDeflection, options.tolerance);
    region.boundary.closed = true;
    if (region.boundary.points.size() < 3 ||
        std::abs(region.boundary.signedArea) <= options.tolerance) {
        fail(error, "profile outer wire has no usable closed boundary");
        return std::nullopt;
    }
    if (region.boundary.signedArea < 0.0)
        OcctAreaAdapter::reverseLoop(region.boundary);
    region.protectedMaterialShape = protectedMaterialShape;
    updateStockBoundary(region, options);
    region.sectionShape = sectionShape;
    return region;
}

void ProfileRegionBuilder::updateStockBoundary(
    ProfileRegion& region,
    const ProfileRegionBuildOptions& options)
{
    region.stockBoundary = rectangularStockBoundary(
        region, options);
}

std::vector<ProfileRegion> ProfileRegionBuilder::build(
    const std::vector<RootOuterWireProfileSource>& sources,
    const StepIndexer& indexer,
    const ProfileRegionBuildOptions& options,
    std::vector<SkippedProfileRegion>* skipped) const
{
    std::vector<ProfileRegion> regions;
    regions.reserve(sources.size());
    for (const RootOuterWireProfileSource& source : sources) {
        std::string error;
        std::optional<ProfileRegion> region =
            buildOne(source, indexer, options, &error);
        if (region) {
            regions.push_back(std::move(*region));
        } else if (skipped) {
            skipped->push_back({source, std::move(error)});
        }
    }
    return regions;
}

} // namespace oap::toolpath
