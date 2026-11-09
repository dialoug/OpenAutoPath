// 将槽/腔识别结果解析为局部二维加工区域、底面、边界、岛屿和分层截面实体。
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"

#include <algorithm>
#include <unordered_map>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath {

namespace {

bool isPocketLike(FeatureKind kind)
{
    return kind == FeatureKind::Pocket || kind == FeatureKind::Slot;
}

double faceArea(const TopoDS_Face& face)
{
    try {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        return props.Mass();
    } catch (const Standard_Failure&) {
        return -1.0;
    }
}

double edgeLength(const TopoDS_Edge& edge)
{
    try {
        GProp_GProps props;
        BRepGProp::LinearProperties(edge, props);
        return props.Mass();
    } catch (const Standard_Failure&) {
        return 0.0;
    }
}

double sharedBoundaryRatio(
    const TopoDS_Face& face,
    const TopTools_IndexedDataMapOfShapeListOfShape& edgeFaces)
{
    double totalLength = 0.0;
    double sharedLength = 0.0;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        const double length = edgeLength(edge);
        if (length <= Precision::Confusion())
            continue;
        totalLength += length;
        if (edgeFaces.Contains(edge) && edgeFaces.FindFromKey(edge).Extent() > 1)
            sharedLength += length;
    }
    return totalLength > Precision::Confusion() ? sharedLength / totalLength : 0.0;
}

double sectionTopologyTolerance(const std::vector<TopoDS_Face>& faces,
                                double requestedTolerance)
{
    double tolerance = std::max(requestedTolerance, Precision::Confusion());
    for (const TopoDS_Face& face : faces) {
        tolerance = std::max(tolerance, BRep_Tool::Tolerance(face));
        for (TopExp_Explorer edgeEx(face, TopAbs_EDGE); edgeEx.More(); edgeEx.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(edgeEx.Current());
            tolerance = std::max(tolerance, BRep_Tool::Tolerance(edge));
        }
        for (TopExp_Explorer vertexEx(face, TopAbs_VERTEX);
             vertexEx.More(); vertexEx.Next()) {
            const TopoDS_Vertex vertex = TopoDS::Vertex(vertexEx.Current());
            tolerance = std::max(tolerance, BRep_Tool::Tolerance(vertex));
        }
    }
    return std::clamp(tolerance * 2.0, Precision::Confusion(), 1.0e-3);
}

bool planeOfFace(const TopoDS_Face& face, gp_Pln& plane, gp_Dir& openDir)
{
    BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Plane)
        return false;

    plane = surface.Plane();
    openDir = plane.Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED)
        openDir.Reverse();
    return true;
}

double projectedDepth(const std::vector<TopoDS_Face>& faces,
                      const gp_Pnt& origin,
                      const gp_Dir& openDir,
                      double deflection)
{
    // 深度 = 各面沿 openDir 相对底面原点的最大投影。投影是线性泛函,其在紧致
    // 曲面片上的极值必落在边界(边/顶点)上,故只需沿边离散采样即可得到精确极值,
    // 无需扫描面内部。相比早先「每面世界轴对齐 AABB 的 8 角点」,这避免了斜/曲壁
    // 的包围盒在各轴膨胀导致的深度高估(AABB 与 openDir 不对齐时角点会超出真实延伸)。
    double depth = 0.0;
    const gp_Vec axis(openDir);
    const double defl = std::max(deflection, Precision::Confusion());

    auto consider = [&](const gp_Pnt& p) {
        depth = std::max(depth, gp_Vec(origin, p).Dot(axis));
    };

    for (const TopoDS_Face& face : faces) {
        for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
            if (edge.IsNull())
                continue;
            try {
                BRepAdaptor_Curve curve(edge);
                GCPnts_QuasiUniformDeflection disc(curve, defl);
                if (disc.IsDone() && disc.NbPoints() >= 1) {
                    for (int i = 1; i <= disc.NbPoints(); ++i)
                        consider(disc.Value(i));
                    continue;
                }
            } catch (const Standard_Failure&) {
                // 落到端点顶点。
            }
            TopoDS_Vertex first;
            TopoDS_Vertex last;
            TopExp::Vertices(edge, first, last, Standard_True);
            if (!first.IsNull())
                consider(BRep_Tool::Pnt(first));
            if (!last.IsNull())
                consider(BRep_Tool::Pnt(last));
        }
    }

    return depth > Precision::Confusion() ? depth : 0.0;
}

std::vector<TopoDS_Face> connectedFacesFromFloor(
    const TopoDS_Face& floor,
    const TopTools_IndexedDataMapOfShapeListOfShape& edgeFaces)
{
    std::vector<TopoDS_Face> connected;
    std::vector<TopoDS_Face> pending{floor};
    TopTools_MapOfShape visited;
    visited.Add(floor);

    while (!pending.empty()) {
        const TopoDS_Face current = pending.back();
        pending.pop_back();
        connected.push_back(current);

        for (TopExp_Explorer edgeEx(current, TopAbs_EDGE);
             edgeEx.More(); edgeEx.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(edgeEx.Current());
            if (!edgeFaces.Contains(edge))
                continue;
            const TopTools_ListOfShape& neighbors = edgeFaces.FindFromKey(edge);
            for (TopTools_ListIteratorOfListOfShape it(neighbors); it.More(); it.Next()) {
                const TopoDS_Shape& shape = it.Value();
                if (shape.ShapeType() != TopAbs_FACE || visited.Contains(shape))
                    continue;
                visited.Add(shape);
                pending.push_back(TopoDS::Face(shape));
            }
        }
    }
    return connected;
}

void setDiagnostic(ToolpathDiagnostic* diagnostic,
                   ToolpathDiagnosticCode code,
                   const char* message)
{
    if (!diagnostic)
        return;
    diagnostic->code = code;
    diagnostic->severity = ToolpathDiagnosticSeverity::Error;
    diagnostic->message = message;
}

} // namespace

std::vector<PocketRegion> PocketRegionBuilder::build(
    const std::vector<FeatureInstance>& instances,
    const StepIndexer& indexer,
    const PocketRegionBuildOptions& options,
    std::vector<SkippedRegion>* skipped) const
{
    std::vector<PocketRegion> regions;
    regions.reserve(instances.size());
    // 记录“自动识别且已占用某底面”的区域，供合并同底面重复实例。
    std::unordered_map<std::string, size_t> automaticFloorRegions;

    for (const FeatureInstance& instance : instances) {
        if (!isPocketLike(instance.kind))
            continue;

        ToolpathDiagnostic diagnostic;
        std::optional<PocketRegion> region =
            buildInstanceDetailed(instance, indexer, options, &diagnostic);
        if (region) {
            // 以“特征类型:底面 faceId”为键去重：识别器可能把同一物理腔
            // 拆成多个实例（不同侧壁面组）。手动实例不合并，保证 UI 中
            // 独立编辑的入口保持独立。
            const std::string key = std::to_string(static_cast<int>(region->kind)) +
                                    ":" + region->floorFaceId;
            if (!instance.manual) {
                const auto duplicate = automaticFloorRegions.find(key);
                if (duplicate != automaticFloorRegions.end()) {
                    PocketRegion& retained = regions[duplicate->second];
                    FeatureInstance mergedInstance;
                    mergedInstance.kind = instance.kind;
                    mergedInstance.faceIds = retained.sourceFaceIds;
                    mergedInstance.faceIds.insert(mergedInstance.faceIds.end(),
                                                  instance.faceIds.begin(),
                                                  instance.faceIds.end());
                    std::sort(mergedInstance.faceIds.begin(), mergedInstance.faceIds.end());
                    mergedInstance.faceIds.erase(
                        std::unique(mergedInstance.faceIds.begin(),
                                    mergedInstance.faceIds.end()),
                        mergedInstance.faceIds.end());

                    ToolpathDiagnostic mergeDiagnostic;
                    std::optional<PocketRegion> mergedRegion =
                        buildInstanceDetailed(mergedInstance, indexer, options,
                                              &mergeDiagnostic);
                    if (mergedRegion &&
                        mergedRegion->floorFaceId == retained.floorFaceId) {
                        retained = std::move(*mergedRegion);
                    }
                    if (skipped) {
                        SkippedRegion s;
                        s.kind = instance.kind;
                        s.sourceFaceIds = instance.faceIds;
                        s.reason = "duplicate automatic region shares floor face " +
                                   region->floorFaceId;
                        s.diagnostic.code = ToolpathDiagnosticCode::DuplicateRegion;
                        s.diagnostic.severity = ToolpathDiagnosticSeverity::Warning;
                        s.diagnostic.message = s.reason;
                        skipped->push_back(std::move(s));
                    }
                    continue;
                }
                automaticFloorRegions.emplace(key, regions.size());
            }
            regions.push_back(std::move(*region));
        } else if (skipped) {
            SkippedRegion s;
            s.kind = instance.kind;
            s.sourceFaceIds = instance.faceIds;
            if (diagnostic.code == ToolpathDiagnosticCode::None) {
                diagnostic.code = ToolpathDiagnosticCode::UnknownRegionBuildFailure;
                diagnostic.severity = ToolpathDiagnosticSeverity::Error;
                diagnostic.message = "unknown reason";
            }
            s.reason = diagnostic.message;
            s.diagnostic = std::move(diagnostic);
            skipped->push_back(std::move(s));
        }
    }

    return regions;
}

std::optional<PocketRegion> PocketRegionBuilder::buildInstance(
    const FeatureInstance& instance,
    const StepIndexer& indexer,
    const PocketRegionBuildOptions& options,
    std::string* error) const
{
    ToolpathDiagnostic diagnostic;
    std::optional<PocketRegion> region =
        buildInstanceDetailed(instance, indexer, options, &diagnostic);
    if (error)
        *error = diagnostic.message;
    return region;
}

std::optional<PocketRegion> PocketRegionBuilder::buildInstanceDetailed(
    const FeatureInstance& instance,
    const StepIndexer& indexer,
    const PocketRegionBuildOptions& options,
    ToolpathDiagnostic* diagnostic) const
{
    if (diagnostic)
        *diagnostic = {};
    if (!isPocketLike(instance.kind)) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::UnsupportedFeature,
                      "feature instance is not a slot or pocket");
        return std::nullopt;
    }
    const std::string explicitFloorFaceId = !instance.floorFaceId.empty()
        ? instance.floorFaceId
        : options.preferredFloorFaceId;
    if (!instance.floorFaceId.empty() &&
        std::find(instance.faceIds.begin(), instance.faceIds.end(),
                  instance.floorFaceId) == instance.faceIds.end()) {
        setDiagnostic(diagnostic,
                      ToolpathDiagnosticCode::ExplicitFloorNotInFeature,
                      "explicit floor face is not part of the feature instance");
        return std::nullopt;
    }
    if (instance.manual && instance.faceIds.size() <= 1) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::ManualFloorDisconnected,
                      "manual floor seed has no topologically adjacent faces");
        return std::nullopt;
    }

    struct Candidate {
        std::string faceId;
        TopoDS_Face face;
        gp_Pln plane;
        gp_Dir openDir;
        double area = -1.0;
        double sharedRatio = 0.0;
        double depth = 0.0;
    };

    std::vector<TopoDS_Face> faces;
    faces.reserve(instance.faceIds.size());
    std::vector<Candidate> planarFaces;

    for (const std::string& faceId : instance.faceIds) {
        const TopoDS_Face* face = indexer.faceForId(faceId);
        if (!face || face->IsNull())
            continue;

        faces.push_back(*face);

        gp_Pln plane;
        gp_Dir openDir;
        if (!planeOfFace(*face, plane, openDir))
            continue;

        const double area = faceArea(*face);
        if (area <= Precision::Confusion())
            continue;

        planarFaces.push_back({faceId, *face, plane, openDir, area});
    }

    if (faces.empty()) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingFaceGeometry,
                      "no valid TopoDS_Face found for feature instance");
        return std::nullopt;
    }

    if (planarFaces.empty()) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingPlanarFloor,
                      "slot/pocket instance has no planar floor candidate");
        return std::nullopt;
    }

    TopoDS_Compound sectionShape;
    BRep_Builder builder;
    builder.MakeCompound(sectionShape);
    for (const TopoDS_Face& face : faces)
        builder.Add(sectionShape, face);

    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
    TopExp::MapShapesAndAncestors(sectionShape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);

    for (Candidate& candidate : planarFaces) {
        // 对每个平面候选面计算：与面组内其他面共享边界的比例（衡量它
        // 是否真的是底面）以及沿其法向/反法向的投影深度，作为底面选择
        // 与开口方向的评分依据。
        candidate.sharedRatio = sharedBoundaryRatio(candidate.face, edgeFaces);
        const double forwardDepth = projectedDepth(
            faces, candidate.plane.Location(), candidate.openDir,
            options.discretizationDeflection);
        gp_Dir reverseDir = candidate.openDir.Reversed();
        const double reverseDepth = projectedDepth(
            faces, candidate.plane.Location(), reverseDir,
            options.discretizationDeflection);
        // 深度大的一侧视为开口方向（加工向下沿反方向），保证区域的
        // openDir 指向材料开放侧而非实体内部。
        if (reverseDepth > forwardDepth) {
            candidate.openDir = reverseDir;
            candidate.depth = reverseDepth;
        } else {
            candidate.depth = forwardDepth;
        }
    }

    const auto bestSharedCandidate = std::max_element(
        planarFaces.begin(), planarFaces.end(),
        [](const Candidate& a, const Candidate& b) {
            if (std::abs(a.sharedRatio - b.sharedRatio) > 1.0e-9)
                return a.sharedRatio < b.sharedRatio;
            if (std::abs(a.depth - b.depth) > 1.0e-9)
                return a.depth < b.depth;
            return a.area < b.area;
        });
    const auto largestAreaCandidate = std::max_element(
        planarFaces.begin(), planarFaces.end(),
        [](const Candidate& a, const Candidate& b) { return a.area < b.area; });
    const auto minSharedCandidate = std::min_element(
        planarFaces.begin(), planarFaces.end(),
        [](const Candidate& a, const Candidate& b) {
            return a.sharedRatio < b.sharedRatio;
        });

    // 某些 STEP/XDE 模型会将特征面暴露为独立拓扑，因此几何上重合的边
    // 不一定满足 IsSame()。此时所有 sharedRatio 都接近零。只有拓扑评分能
    // 明确区分候选面时才采用它，否则保留原有的面积回退策略。
    const bool sharedTopologyIsUseful =
        planarFaces.size() == 1 ||
        (bestSharedCandidate->sharedRatio >= 0.5 &&
         bestSharedCandidate->sharedRatio - minSharedCandidate->sharedRatio >= 0.1);
    const Candidate* floor = sharedTopologyIsUseful
        ? &*bestSharedCandidate
        : &*largestAreaCandidate;
    if (!explicitFloorFaceId.empty()) {
        const auto preferred = std::find_if(
            planarFaces.begin(), planarFaces.end(),
            [&](const Candidate& candidate) {
                return candidate.faceId == explicitFloorFaceId;
            });
        if (preferred == planarFaces.end()) {
            setDiagnostic(diagnostic, ToolpathDiagnosticCode::InvalidExplicitFloor,
                          "explicit floor face is not a valid planar candidate");
            return std::nullopt;
        }
        floor = &*preferred;
    }

    // 识别面组可能混入与底面无拓扑关系的远处面。只在确实找到“底面 + 邻接面”
    // 的连通分量时使用过滤结果；独立拓扑 STEP 无法建立邻接时保留原有回退。
    const std::vector<TopoDS_Face> connectedFaces =
        connectedFacesFromFloor(floor->face, edgeFaces);
    const bool useConnectedFaces =
        connectedFaces.size() > 1 && connectedFaces.size() < faces.size();
    const std::vector<TopoDS_Face>& sectionFaces =
        useConnectedFaces ? connectedFaces : faces;

    gp_Dir regionOpenDir = floor->plane.Axis().Direction();
    if (floor->face.Orientation() == TopAbs_REVERSED)
        regionOpenDir.Reverse();
    // 最终确定底面后，重新按“底面 + 连通面组”计算深度与开口方向；
    // 深度取两个方向投影的较大者，较大的那一侧就是实体开口侧。
    const double forwardDepth = projectedDepth(
        sectionFaces, floor->plane.Location(), regionOpenDir,
        options.discretizationDeflection);
    const gp_Dir reverseDir = regionOpenDir.Reversed();
    const double reverseDepth = projectedDepth(
        sectionFaces, floor->plane.Location(), reverseDir,
        options.discretizationDeflection);
    const double regionDepth = std::max(forwardDepth, reverseDepth);
    if (reverseDepth > forwardDepth)
        regionOpenDir = reverseDir;

    TopoDS_Compound filteredSectionShape;
    builder.MakeCompound(filteredSectionShape);
    for (const TopoDS_Face& face : sectionFaces)
        builder.Add(filteredSectionShape, face);

    PocketRegion region;
    region.kind = instance.kind;
    region.sourceFaceIds = instance.faceIds;
    region.floorFaceId = floor->faceId;
    region.floorFace = floor->face;
    region.floorPlane = floor->plane;
    region.openDir = regionOpenDir;
    region.frame = gp_Ax3(floor->plane.Location(), regionOpenDir);
    region.floorArea = floor->area;
    region.depth = regionDepth;
    region.sectionTolerance = sectionTopologyTolerance(sectionFaces, options.tolerance);
    region.sectionDeflection =
        std::max(options.discretizationDeflection, Precision::Confusion());

    region.sectionShape = filteredSectionShape;
    if (const TopoDS_Solid* solid =
            indexer.solidForFaceId(floor->faceId)) {
        region.protectedMaterialShape = *solid;
    }
    region.sectionFaceCount = static_cast<int>(sectionFaces.size());
    region.filteredDisconnectedFaces = useConnectedFaces;

    TopoDS_Wire outer;
    try {
        outer = BRepTools::OuterWire(floor->face);
    } catch (const Standard_Failure&) {
        outer.Nullify();
    }

    for (TopExp_Explorer ex(floor->face, TopAbs_WIRE); ex.More(); ex.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(ex.Current());
        if (wire.IsNull())
            continue;

        Loop2d loop = OcctAreaAdapter::wireToLoop2d(
            wire, region.frame, options.discretizationDeflection, options.tolerance);
        if (loop.points.size() < 3)
            continue;

        if (!outer.IsNull() && wire.IsSame(outer))
            region.boundary = std::move(loop);
        else
            region.islands.push_back(std::move(loop));
    }

    if (region.boundary.points.empty() && !outer.IsNull()) {
        region.boundary = OcctAreaAdapter::wireToLoop2d(
            outer, region.frame, options.discretizationDeflection, options.tolerance);
    }

    if (region.boundary.points.size() < 3) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingOuterBoundary,
                      "floor face has no usable outer boundary");
        return std::nullopt;
    }

    if (options.normalizeOrientation) {
        if (region.boundary.signedArea < 0.0)
            OcctAreaAdapter::reverseLoop(region.boundary);
        for (Loop2d& island : region.islands)
            if (island.signedArea > 0.0)
                OcctAreaAdapter::reverseLoop(island);
    }

    return region;
}

std::vector<PocketRegion> PocketRegionBuilder::buildRootInnerWireRegions(
    const std::vector<RootInnerWirePocketSource>& sources,
    const StepIndexer& indexer,
    const PocketRegionBuildOptions& options,
    std::vector<SkippedRegion>* skipped) const
{
    std::vector<PocketRegion> regions;
    regions.reserve(sources.size());
    for (const RootInnerWirePocketSource& source : sources) {
        ToolpathDiagnostic diagnostic;
        std::optional<PocketRegion> region =
            buildRootInnerWireRegionDetailed(
                source, indexer, options, &diagnostic);
        if (region) {
            regions.push_back(std::move(*region));
            continue;
        }
        if (!skipped)
            continue;
        if (diagnostic.code == ToolpathDiagnosticCode::None) {
            diagnostic.code =
                ToolpathDiagnosticCode::UnknownRegionBuildFailure;
            diagnostic.severity =
                ToolpathDiagnosticSeverity::Error;
            diagnostic.message =
                "unknown root inner-wire region build failure";
        }
        SkippedRegion item;
        item.kind = source.kind;
        item.sourceFaceIds = source.sourceFaceIds;
        item.reason = diagnostic.message;
        item.diagnostic = std::move(diagnostic);
        skipped->push_back(std::move(item));
    }
    return regions;
}

std::optional<PocketRegion>
PocketRegionBuilder::buildRootInnerWireRegionDetailed(
    const RootInnerWirePocketSource& source,
    const StepIndexer& indexer,
    const PocketRegionBuildOptions& options,
    ToolpathDiagnostic* diagnostic) const
{
    if (diagnostic)
        *diagnostic = {};
    if (!isPocketLike(source.kind)) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::UnsupportedFeature,
                      "inner-wire source is not a slot or pocket");
        return std::nullopt;
    }
    if (source.rootFaceId.empty() || source.rootWireId.empty()) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingOuterBoundary,
                      "main-outer root face or inner wire id is missing");
        return std::nullopt;
    }

    const TopoDS_Face* rootFace =
        indexer.faceForId(source.rootFaceId);
    const TopoDS_Wire* rootWire =
        indexer.wireForId(source.rootWireId);
    if (!rootFace || rootFace->IsNull() ||
        !rootWire || rootWire->IsNull()) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingFaceGeometry,
                      "main-outer root face or inner wire geometry was not found");
        return std::nullopt;
    }

    gp_Pln rootPlane;
    gp_Dir rootOpenDir;
    if (!planeOfFace(*rootFace, rootPlane, rootOpenDir)) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingPlanarFloor,
                      "main-outer inner-wire root is not planar");
        return std::nullopt;
    }

    TopoDS_Wire outerWire;
    try {
        outerWire = BRepTools::OuterWire(*rootFace);
    } catch (const Standard_Failure&) {
        outerWire.Nullify();
    }
    bool wireBelongsToRoot = false;
    for (TopExp_Explorer wireExplorer(*rootFace, TopAbs_WIRE);
         wireExplorer.More(); wireExplorer.Next()) {
        const TopoDS_Wire candidate =
            TopoDS::Wire(wireExplorer.Current());
        if (!candidate.IsNull() &&
            candidate.IsSame(*rootWire)) {
            wireBelongsToRoot = true;
            break;
        }
    }
    if (!wireBelongsToRoot ||
        (!outerWire.IsNull() && rootWire->IsSame(outerWire))) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingOuterBoundary,
                      "selected root wire is not an inner wire of the main-outer face");
        return std::nullopt;
    }

    std::vector<TopoDS_Face> sectionFaces;
    sectionFaces.reserve(source.sourceFaceIds.size());
    for (const std::string& faceId : source.sourceFaceIds) {
        const TopoDS_Face* face = indexer.faceForId(faceId);
        if (face && !face->IsNull())
            sectionFaces.push_back(*face);
    }
    if (sectionFaces.empty()) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingFaceGeometry,
                      "inner-wire structure has no valid section faces");
        return std::nullopt;
    }

    TopoDS_Compound sectionShape;
    BRep_Builder builder;
    builder.MakeCompound(sectionShape);
    for (const TopoDS_Face& face : sectionFaces)
        builder.Add(sectionShape, face);

    // 必须能从指定根内环直接进入结构面，防止把同一实体上的远处面组
    // 错配给这个开口。
    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
    TopExp::MapShapesAndAncestors(
        sectionShape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
    bool touchesRootWire = false;
    for (TopExp_Explorer edgeExplorer(*rootWire, TopAbs_EDGE);
         edgeExplorer.More() && !touchesRootWire;
         edgeExplorer.Next()) {
        const TopoDS_Edge edge =
            TopoDS::Edge(edgeExplorer.Current());
        touchesRootWire =
            edgeFaces.Contains(edge) &&
            !edgeFaces.FindFromKey(edge).IsEmpty();
    }
    if (!touchesRootWire) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::ManualFloorDisconnected,
                      "inner-wire structure faces are not adjacent to the root inner wire");
        return std::nullopt;
    }

    // 主外表面的有效法向始终作为局部开口方向；实际下刀方向严格取
    // 它的反向。不能再根据两侧投影较大者翻转 openDir，否则合并后的
    // 大型连通盲腔会把实体内部方向误当成安全退刀方向。
    const gp_Dir mainOuterNormal = rootOpenDir;
    const gp_Dir machiningDownDir =
        mainOuterNormal.Reversed();
    const double depth = projectedDepth(
        sectionFaces, rootPlane.Location(),
        machiningDownDir,
        options.discretizationDeflection);
    if (depth <= std::max(options.tolerance, Precision::Confusion())) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingFaceGeometry,
                      "inner-wire structure has no positive depth opposite "
                      "the main-outer face normal");
        return std::nullopt;
    }

    // PocketClearing 的统一深度约定是：offset=0 位于最深层，
    // offset=depth 位于开口，局部 +Z 指向开口。根内环本身在主外表面
    // 上，因此把坐标原点沿下刀方向平移到最深层。
    gp_Pnt deepestOrigin = rootPlane.Location();
    deepestOrigin.Translate(
        gp_Vec(machiningDownDir) * depth);

    PocketRegion region;
    region.kind = source.kind;
    region.sourceFaceIds = source.sourceFaceIds;
    region.floorFaceId = source.rootFaceId;
    region.floorFace = *rootFace;
    region.floorPlane =
        gp_Pln(deepestOrigin, mainOuterNormal);
    region.openDir = mainOuterNormal;
    region.frame =
        gp_Ax3(deepestOrigin, mainOuterNormal);
    region.depth = depth;
    region.sectionShape = sectionShape;
    if (const TopoDS_Solid* solid =
            indexer.solidForFaceId(source.rootFaceId)) {
        region.protectedMaterialShape = *solid;
    }
    region.sectionTolerance =
        sectionTopologyTolerance(sectionFaces, options.tolerance);
    region.sectionDeflection =
        std::max(options.discretizationDeflection,
                 Precision::Confusion());
    region.sectionFaceCount =
        static_cast<int>(sectionFaces.size());
    region.allowOpenSectionContours =
        source.allowOpenSectionContours;
    region.openBoundaryFaceCount =
        source.openBoundaryFaceCount;
    region.useSectionBoundaryAtEveryDepth = true;

    region.boundary = OcctAreaAdapter::wireToLoop2d(
        *rootWire, region.frame,
        options.discretizationDeflection,
        options.tolerance);
    if (region.boundary.points.size() < 3 ||
        std::abs(region.boundary.signedArea) <=
            std::max(options.tolerance,
                     Precision::Confusion())) {
        setDiagnostic(diagnostic, ToolpathDiagnosticCode::MissingOuterBoundary,
                      "main-outer inner wire has no usable closed boundary");
        return std::nullopt;
    }
    // 线框已经通过“属于根面且不是 OuterWire”的拓扑校验。部分 STEP
    // 边方向会使离散适配器未重复输出首点，但区域运算仍应按隐式闭环处理。
    region.boundary.closed = true;
    if (options.normalizeOrientation &&
        region.boundary.signedArea < 0.0) {
        OcctAreaAdapter::reverseLoop(region.boundary);
    }
    region.floorArea =
        std::abs(region.boundary.signedArea);
    return region;
}

} // namespace oap::toolpath
