// 将孔识别结果转换为稳定的 HoleRecognitionRecord 特征，解析入口、孔底、半径和装夹面。
#include "HoleToolpathBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Vec.hxx>

#include "analysis/Model.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/StepIndexer.h"
#include "analysis/MoldSurfaceClassifier.h"
#include "recognition/detail/HoleToolpathRelations.h"
#include "recognition/AAG.h"
#include "recognition/Recognition.h"

namespace oap::recognition::detail {

namespace {

// 提取面 id 末尾的数字后缀，用于按 STEP 面编号稳定排序。
long long faceSuffixNum(const std::string& id)
{
    std::size_t index = 0;
    while (index < id.size() && (id[index] < '0' || id[index] > '9'))
        ++index;
    long long value = 0;
    for (; index < id.size() && id[index] >= '0' && id[index] <= '9'; ++index)
        value = value * 10 + (id[index] - '0');
    return value;
}

std::string stableFaceId(const std::vector<int>& faceIds,
                         const AAG& graph,
                         const StepIndexer& indexer)
{
    std::string selected;
    for (int faceIndex : faceIds) {
        if (faceIndex < 1 || faceIndex > graph.faceMap.Extent())
            continue;
        const std::string id = indexer.idForFace(
            TopoDS::Face(graph.faceMap(faceIndex)));
        if (!id.empty() &&
            (selected.empty() || faceSuffixNum(id) < faceSuffixNum(selected)))
            selected = id;
    }
    return selected;
}

// 将一个圆柱孔段的全部拓扑面索引转换为去重、稳定排序的 STEP 面标识。
std::vector<std::string> stableFaceIds(const std::vector<int>& faceIds,
                                       const AAG& graph,
                                       const StepIndexer& indexer)
{
    std::vector<std::string> result;
    result.reserve(faceIds.size());
    for (int faceIndex : faceIds) {
        if (faceIndex < 1 || faceIndex > graph.faceMap.Extent())
            continue;
        const std::string id = indexer.idForFace(
            TopoDS::Face(graph.faceMap(faceIndex)));
        if (!id.empty())
            result.push_back(id);
    }
    std::sort(result.begin(), result.end(),
              [](const std::string& first, const std::string& second) {
                  return faceSuffixNum(first) < faceSuffixNum(second);
              });
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

// 合并全部 Unit 的包围盒，作为模具整体边界。
Bbox modelBounds(const Model& model)
{
    Bbox bounds;
    for (const Unit& unit : model.units)
        bounds.merge(unit.bbox);
    return bounds;
}

// 把整体边界八个角点变换到模具局部坐标系，得到沿 Z 的厚度区间；
// 供六面回退定向判断端点是否靠近模具上下包络面。
bool localThicknessRange(const Bbox& bounds,
                         const MoldCoordinateFrame& frame,
                         double& minimum,
                         double& maximum)
{
    if (!bounds.valid() || !frame.isValid)
        return false;
    bool initialized = false;
    for (int xi = 0; xi < 2; ++xi) {
        for (int yi = 0; yi < 2; ++yi) {
            for (int zi = 0; zi < 2; ++zi) {
                const gp_Pnt corner(xi ? bounds.xmax : bounds.xmin,
                                    yi ? bounds.ymax : bounds.ymin,
                                    zi ? bounds.zmax : bounds.zmin);
                const double z = frame.worldToLocal(corner).Z();
                if (!initialized) {
                    minimum = maximum = z;
                    initialized = true;
                } else {
                    minimum = std::min(minimum, z);
                    maximum = std::max(maximum, z);
                }
            }
        }
    }
    return initialized && maximum - minimum > Precision::Confusion();
}

// 识别结果可能包含多个同轴圆柱片或复合孔区域；沿孔轴投影所有区域顶点，
// 覆盖圆柱片、锥面、台阶面和孔底，避免只截取主圆柱的一小段。
bool holeRegionAxisEndpoints(const HoleResult& hole,
                             const AAG& graph,
                             gp_Pnt& first,
                             gp_Pnt& second)
{
    const gp_Pnt origin = hole.axis.Location();
    const gp_Vec direction(hole.axis.Direction());
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (int faceIndex : hole.faces) {
        if (faceIndex < 1 || faceIndex > graph.faceMap.Extent())
            continue;
        const TopoDS_Face face = TopoDS::Face(graph.faceMap(faceIndex));
        for (TopExp_Explorer vertex(face, TopAbs_VERTEX);
             vertex.More(); vertex.Next()) {
            const gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(vertex.Current()));
            const double parameter = gp_Vec(origin, point).Dot(direction);
            minimum = std::min(minimum, parameter);
            maximum = std::max(maximum, parameter);
        }
    }
    if (std::isfinite(minimum) && std::isfinite(maximum) &&
        maximum - minimum > Precision::Confusion()) {
        first = origin.Translated(direction * minimum);
        second = origin.Translated(direction * maximum);
        return true;
    }

    if (hole.mainFaceId < 1 || hole.mainFaceId > graph.faceMap.Extent())
        return false;
    const TopoDS_Face face = TopoDS::Face(graph.faceMap(hole.mainFaceId));
    Standard_Real uMin, uMax, vMin, vMax;
    BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
    first = origin.Translated(direction * vMin);
    second = origin.Translated(direction * vMax);
    return first.Distance(second) > Precision::Confusion();
}

struct HoleStage {
    double radius = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
    std::vector<int> cylinderFaceIds;
};

// 线性查找面是否属于给定集合；集合很小，直接遍历。
bool containsFace(const std::vector<int>& faces, int faceId)
{
    return std::find(faces.begin(), faces.end(), faceId) != faces.end();
}

HoleBottomProfile stageBottomProfile(const HoleStage& stage,
                                     const HoleResult& hole,
                                     const AAG& graph,
                                     bool entryAtMinimum)
{
    // 只有实际 bottom 端邻接的终止面才属于当前孔段；
    // 共享肩面会同时成为相邻两个圆柱段的边界，但只会终止外侧扩大段。
    const double bottomParameter = entryAtMinimum ? stage.maximum : stage.minimum;
    constexpr double kEndpointTolerance = 1.0e-5;
    for (int cylinderId : stage.cylinderFaceIds) {
        const auto adjacent = graph.adjacency.find(cylinderId);
        if (adjacent == graph.adjacency.end())
            continue;
        for (int neighborId : adjacent->second) {
            HoleBottomProfile profile = HoleBottomProfile::Unknown;
            if (containsFace(hole.planarBottomFaceIds, neighborId))
                profile = HoleBottomProfile::Planar;
            else if (containsFace(hole.conicalBottomFaceIds, neighborId))
                profile = HoleBottomProfile::Conical;
            if (profile == HoleBottomProfile::Unknown ||
                neighborId < 1 || neighborId > graph.faceMap.Extent())
                continue;

            const TopoDS_Face face = TopoDS::Face(graph.faceMap(neighborId));
            const gp_Pnt origin = hole.axis.Location();
            const gp_Vec direction(hole.axis.Direction());
            for (TopExp_Explorer vertex(face, TopAbs_VERTEX);
                 vertex.More(); vertex.Next()) {
                const gp_Pnt point = BRep_Tool::Pnt(
                    TopoDS::Vertex(vertex.Current()));
                const double parameter = gp_Vec(origin, point).Dot(direction);
                if (std::abs(parameter - bottomParameter) <= kEndpointTolerance)
                    return profile;
            }
        }
    }
    // 只有识别结果本身确认是普通通孔时，缺少终止面才足以证明该端开放。
    // 盲孔和沉头孔的拓扑可能残缺；此时不能把“没找到终止面”当成开放端，
    // 否则会掩盖平底/锥底识别失败并影响自动选刀。
    return hole.type == HoleType::THROUGH
        ? HoleBottomProfile::Open
        : HoleBottomProfile::Unknown;
}

bool cylinderFaceAxisRange(const TopoDS_Face& face,
                           const gp_Pnt& origin,
                           const gp_Vec& direction,
                           double& minimum,
                           double& maximum)
{
    minimum = std::numeric_limits<double>::infinity();
    maximum = -std::numeric_limits<double>::infinity();
    for (TopExp_Explorer vertex(face, TopAbs_VERTEX);
         vertex.More(); vertex.Next()) {
        const gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(vertex.Current()));
        const double parameter = gp_Vec(origin, point).Dot(direction);
        minimum = std::min(minimum, parameter);
        maximum = std::max(maximum, parameter);
    }
    return std::isfinite(minimum) && std::isfinite(maximum) &&
           maximum - minimum > Precision::Confusion();
}

// 判断当前圆柱壁是否已经是带同轴小径孔的真实沉孔大径段；入口斜环此时只
// 用于定位外部开口，不能再以过渡外径伪造一个更大的加工圆柱段。
bool hasSmallerCoaxialStageAcrossShoulder(
    const HoleResult& hole,
    const AAG& graph,
    double tolerance)
{
    const gp_Vec axisDirection(hole.axis.Direction());
    for (const int shoulderFaceId : hole.planarBottomFaceIds) {
        const auto adjacent = graph.adjacency.find(shoulderFaceId);
        if (adjacent == graph.adjacency.end())
            continue;
        for (const int neighborId : adjacent->second) {
            if (neighborId == hole.mainFaceId || neighborId < 1 ||
                neighborId > graph.faceMap.Extent()) {
                continue;
            }
            gp_Ax1 neighborAxis;
            double neighborRadius = 0.0;
            const TopoDS_Face neighbor =
                TopoDS::Face(graph.faceMap(neighborId));
            if (GetSurfaceType(neighbor) != GeomAbs_Cylinder ||
                !GetCylinderData(neighbor, neighborAxis, neighborRadius) ||
                neighborRadius >= hole.radius - tolerance) {
                continue;
            }

            const gp_Vec neighborDirection(neighborAxis.Direction());
            gp_Vec offset(hole.axis.Location(), neighborAxis.Location());
            offset -= axisDirection.Multiplied(offset.Dot(axisDirection));
            if (std::abs(axisDirection.Dot(neighborDirection)) >=
                    0.9998476951563913 &&
                offset.Magnitude() <= tolerance) {
                return true;
            }
        }
    }
    return false;
}

std::vector<HoleStage> splitCylinderStages(const HoleResult& hole,
                                           const AAG& graph)
{
    // 把孔的所有同轴圆柱面按半径合并为若干“段”（HoleStage），每段记录
    // 轴向区间与壁面索引；之后按区间排序，用于分阶段加工几何。
    constexpr double kStageTolerance = 1.0e-5;
    const gp_Pnt origin = hole.axis.Location();
    const gp_Vec direction(hole.axis.Direction());
    std::vector<HoleStage> stages;
    for (int faceIndex : hole.faces) {
        if (faceIndex < 1 || faceIndex > graph.faceMap.Extent())
            continue;
        const TopoDS_Face face = TopoDS::Face(graph.faceMap(faceIndex));
        gp_Ax1 axis;
        double radius = 0.0;
        if (GetSurfaceType(face) != GeomAbs_Cylinder ||
            !GetCylinderData(face, axis, radius))
            continue;
        const gp_Vec candidateDirection(axis.Direction());
        if (std::abs(candidateDirection.Dot(direction)) < 0.9998476951563913)
            continue;
        gp_Vec axisOffset(origin, axis.Location());
        axisOffset -= direction.Multiplied(axisOffset.Dot(direction));
        if (axisOffset.Magnitude() > kStageTolerance)
            continue;
        double minimum = 0.0;
        double maximum = 0.0;
        if (!cylinderFaceAxisRange(
                face, origin, direction, minimum, maximum))
            continue;

        auto stage = std::find_if(
            stages.begin(), stages.end(), [&](const HoleStage& candidate) {
                return std::abs(candidate.radius - radius) <= kStageTolerance &&
                       minimum <= candidate.maximum + kStageTolerance &&
                       maximum >= candidate.minimum - kStageTolerance;
            });
        if (stage == stages.end()) {
            HoleStage value;
            value.radius = radius;
            value.minimum = minimum;
            value.maximum = maximum;
            value.cylinderFaceIds.push_back(faceIndex);
            stages.push_back(std::move(value));
        } else {
            stage->minimum = std::min(stage->minimum, minimum);
            stage->maximum = std::max(stage->maximum, maximum);
            stage->cylinderFaceIds.push_back(faceIndex);
        }
    }

    // 没有真实沉孔壁但存在入口斜环（开口过渡）时，用其外径参数合成一个
    // 额外扩大段；已有同轴小径孔时不再伪造加工段。
    const bool hasTrueCounterboreWall =
        hasSmallerCoaxialStageAcrossShoulder(
            hole, graph, kStageTolerance);
    if (!hasTrueCounterboreWall &&
        hole.openingTransition.faceId > 0 &&
        hole.openingTransition.outerRadius > hole.radius + kStageTolerance &&
        std::abs(hole.openingTransition.outerParameter -
                 hole.openingTransition.innerParameter) > kStageTolerance) {
        HoleStage transition;
        transition.radius = hole.openingTransition.outerRadius;
        transition.minimum = std::min(
            hole.openingTransition.innerParameter,
            hole.openingTransition.outerParameter);
        transition.maximum = std::max(
            hole.openingTransition.innerParameter,
            hole.openingTransition.outerParameter);
        transition.cylinderFaceIds = {hole.openingTransition.faceId};
        stages.push_back(std::move(transition));
    }

    if (stages.empty()) {
        gp_Pnt first;
        gp_Pnt second;
        if (holeRegionAxisEndpoints(hole, graph, first, second)) {
            HoleStage fallback;
            fallback.radius = hole.radius;
            fallback.minimum = gp_Vec(origin, first).Dot(direction);
            fallback.maximum = gp_Vec(origin, second).Dot(direction);
            if (fallback.minimum > fallback.maximum)
                std::swap(fallback.minimum, fallback.maximum);
            stages.push_back(std::move(fallback));
        }
    }
    std::stable_sort(stages.begin(), stages.end(), [](const HoleStage& first,
                                                       const HoleStage& second) {
        if (std::abs(first.minimum - second.minimum) > 1.0e-5)
            return first.minimum < second.minimum;
        return first.radius > second.radius;
    });
    return stages;
}

// 盲孔/未解析段的端点定向：优先采用拓扑开口点，其次按模具局部 Z 厚度
// 选取更接近上下包络面的一侧为入口，最后回退到原顺序。
void resolveNonThroughEndpoints(const HoleResult& hole,
                                const MoldCoordinateFrame& moldFrame,
                                double localZMin,
                                double localZMax,
                                bool hasThicknessRange,
                                gp_Pnt first,
                                gp_Pnt second,
                                HoleRecognitionRecord& toolpath)
{
    const double tolerance = std::max(10.0 * Precision::Confusion(),
                                      first.Distance(second) * 1.0e-8);
    if (hole.hasOpeningPoint) {
        const double firstDistance = hole.openingPoint.Distance(first);
        const double secondDistance = hole.openingPoint.Distance(second);
        if (std::abs(firstDistance - secondDistance) > tolerance) {
            if (secondDistance < firstDistance)
                std::swap(first, second);
            toolpath.entry = first;
            toolpath.bottom = second;
            toolpath.entryResolution = HoleEntryResolution::TopologyOpening;
            return;
        }
    }

    if (hasThicknessRange) {
        gp_Vec segment(first, second);
        if (segment.Magnitude() > Precision::Confusion()) {
            segment.Normalize();
            constexpr double kParallelDot = 0.9998476951563913; // cos(1°)
            if (std::abs(segment.Dot(gp_Vec(moldFrame.zAxis))) >= kParallelDot) {
                const double firstZ = moldFrame.worldToLocal(first).Z();
                const double secondZ = moldFrame.worldToLocal(second).Z();
                const double firstBoundary = std::min(
                    std::abs(firstZ - localZMin), std::abs(firstZ - localZMax));
                const double secondBoundary = std::min(
                    std::abs(secondZ - localZMin), std::abs(secondZ - localZMax));
                if (std::abs(firstBoundary - secondBoundary) > tolerance) {
                    if (secondBoundary < firstBoundary)
                        std::swap(first, second);
                    toolpath.entry = first;
                    toolpath.bottom = second;
                    toolpath.entryResolution = HoleEntryResolution::MoldBoundaryFallback;
                    return;
                }
            }
        }
    }

    toolpath.entry = first;
    toolpath.bottom = second;
    toolpath.entryResolution = HoleEntryResolution::Unresolved;
}

// 用模具表面分类结果覆盖孔记录的入口/孔底/装夹面，并锁定入口侧。
void applyMoldSurfaceClassification(
    HoleRecognitionRecord& toolpath,
    const MoldSurfaceClassification& surface,
    HoleEntryResolution resolution = HoleEntryResolution::MoldSurfaceNearest)
{
    toolpath.entry = surface.entry;
    toolpath.bottom = surface.bottom;
    toolpath.entryResolution = resolution;
    toolpath.setupFace = surface.setupFace;
    toolpath.setupFaceEntryDistance = surface.entryDistance;
    toolpath.setupFaceOtherEndpointDistance = surface.otherEndpointDistance;
    toolpath.setupFaceDecision = surface.decision;
    toolpath.preserveEntrySide = true;
}

// 以“孔底 -> 入口”的外向轴重新定向；仅在没有真实拓扑解析时作为回退。
void orientAlongOutwardAxis(HoleRecognitionRecord& toolpath,
                            const gp_Vec& outwardAxis)
{
    if (outwardAxis.Magnitude() <= Precision::Confusion())
        return;
    gp_Vec outward = outwardAxis;
    outward.Normalize();
    if (gp_Vec(gp_Pnt(0.0, 0.0, 0.0), toolpath.bottom).Dot(outward) >
        gp_Vec(gp_Pnt(0.0, 0.0, 0.0), toolpath.entry).Dot(outward))
        std::swap(toolpath.entry, toolpath.bottom);
}

// 检查已定向孔段的另一端是否沿孔轴实际落在当前装夹面的相反模具包络面。
bool oppositeEndpointTouchesMoldSurface(
    const HoleRecognitionRecord& toolpath,
    const Bbox& bounds,
    const MoldCoordinateFrame& moldFrame)
{
    const MoldSetupFace opposite = oppositeMoldSetupFace(toolpath.setupFace);
    if (opposite == MoldSetupFace::Unresolved)
        return false;
    const gp_Vec towardOpposite(toolpath.entry, toolpath.bottom);
    if (towardOpposite.Magnitude() <= Precision::Confusion())
        return false;

    gp_Pnt projected;
    double extension = 0.0;
    if (!projectPointToMoldSurface(
            bounds, moldFrame, opposite, toolpath.bottom,
            towardOpposite, projected, extension)) {
        return false;
    }
    const double diagonal = bounds.valid()
        ? std::sqrt(std::pow(bounds.xmax - bounds.xmin, 2.0) +
                    std::pow(bounds.ymax - bounds.ymin, 2.0) +
                    std::pow(bounds.zmax - bounds.zmin, 2.0))
        : 0.0;
    const double tolerance = std::clamp(
        diagonal * 1.0e-7, 1.0e-5, 1.0e-3);
    return extension <= tolerance &&
           projected.Distance(toolpath.bottom) <= tolerance;
}

} // namespace

std::vector<RecognizedHoleGeometry> buildHoleGeometries(
    const std::vector<HoleResult>& holes,
    const AAG& graph,
    const Model& model,
    const StepIndexer& indexer)
{
    std::vector<HoleToolpathCandidate> candidates;
    candidates.reserve(holes.size());
    const Bbox bounds = modelBounds(model);
    const MoldCoordinateFrame moldFrame = MoldCoordinateFrame::fromBounds(bounds);
    double localZMin = 0.0;
    double localZMax = 0.0;
    const bool hasThicknessRange =
        localThicknessRange(bounds, moldFrame, localZMin, localZMax);

    // 每个孔识别结果可能含多个圆柱段：逐段生成 HoleRecognitionRecord，
    // 沉头孔的最大径段作为父级（ownerStage），其余段继承其装夹面。
    for (const HoleResult& hole : holes) {
        FeatureKind kind;
        switch (hole.type) {
            case HoleType::THROUGH:     kind = FeatureKind::ThroughHole; break;
            case HoleType::BLIND:       kind = FeatureKind::BlindHole; break;
            case HoleType::COUNTERSINK: kind = FeatureKind::CountersinkHole; break;
            default: continue;
        }
        if (hole.mainFaceId < 1 || hole.mainFaceId > graph.faceMap.Extent())
            continue;

        const std::vector<HoleStage> stages = splitCylinderStages(hole, graph);
        if (stages.empty())
            continue;
        const gp_Pnt origin = hole.axis.Location();
        const gp_Vec direction(hole.axis.Direction());
        const auto axisPoint = [&](double parameter) {
            return origin.Translated(direction * parameter);
        };

        std::vector<std::string> sourceFaceIds;
        sourceFaceIds.reserve(hole.faces.size());
        for (int faceIndex : hole.faces) {
            if (faceIndex < 1 || faceIndex > graph.faceMap.Extent())
                continue;
            const std::string faceId = indexer.idForFace(
                TopoDS::Face(graph.faceMap(faceIndex)));
            if (!faceId.empty())
                sourceFaceIds.push_back(faceId);
        }
        std::sort(sourceFaceIds.begin(), sourceFaceIds.end(),
                  [](const std::string& first, const std::string& second) {
                      return faceSuffixNum(first) < faceSuffixNum(second);
                  });
        sourceFaceIds.erase(
            std::unique(sourceFaceIds.begin(), sourceFaceIds.end()),
            sourceFaceIds.end());

        double ownerRadius = 0.0;
        const HoleStage* ownerStagePointer = nullptr;
        MoldSurfaceClassification countersinkOwner;
        HoleRecognitionRecord countersinkFallback;
        gp_Vec countersinkOutward;
        if (kind == FeatureKind::CountersinkHole) {
            const HoleStage& ownerStage = *std::max_element(
                stages.begin(), stages.end(),
                [](const HoleStage& first, const HoleStage& second) {
                    return first.radius < second.radius;
                });
            ownerStagePointer = &ownerStage;
            ownerRadius = ownerStage.radius;
            const gp_Pnt ownerFirst = axisPoint(ownerStage.minimum);
            const gp_Pnt ownerSecond = axisPoint(ownerStage.maximum);
            const gp_Pnt* topologyOpening = hole.hasOpeningPoint
                ? &hole.openingPoint : nullptr;
            countersinkOwner = classifyHoleSegmentByMoldSurface(
                bounds, moldFrame, ownerFirst, ownerSecond, topologyOpening);
            if (!countersinkOwner.valid) {
                resolveNonThroughEndpoints(
                    hole, moldFrame, localZMin, localZMax, hasThicknessRange,
                    ownerFirst, ownerSecond, countersinkFallback);
                if (countersinkFallback.entryResolution !=
                    HoleEntryResolution::Unresolved) {
                    countersinkOutward = gp_Vec(
                        countersinkFallback.bottom, countersinkFallback.entry);
                }
            }
        }
        const std::string ownerStableFaceId = ownerStagePointer
            ? stableFaceId(
                  ownerStagePointer->cylinderFaceIds, graph, indexer)
            : std::string();

        for (const HoleStage& stage : stages) {
            HoleRecognitionRecord toolpath;
            toolpath.kind = kind;
            toolpath.radius = stage.radius;
            toolpath.entry = axisPoint(stage.minimum);
            toolpath.bottom = axisPoint(stage.maximum);
            if (kind == FeatureKind::CountersinkHole) {
                toolpath.stageRole = &stage == ownerStagePointer
                    ? HoleStageRole::CountersinkLargeDiameter
                    : HoleStageRole::CountersinkSmallDiameter;
            }

            if (kind == FeatureKind::CountersinkHole) {
                toolpath.entryResolution = HoleEntryResolution::Unresolved;
                if (countersinkOwner.valid) {
                    if (&stage == ownerStagePointer) {
                        applyMoldSurfaceClassification(
                            toolpath, countersinkOwner);
                    } else {
                        const MoldSurfaceClassification inherited =
                            orientHoleSegmentToMoldFace(
                                bounds, moldFrame, countersinkOwner.setupFace,
                                toolpath.entry, toolpath.bottom);
                        if (inherited.valid) {
                            applyMoldSurfaceClassification(
                                toolpath, inherited,
                                HoleEntryResolution::CountersinkLargestStage);
                        }
                    }
                } else if (countersinkFallback.entryResolution !=
                           HoleEntryResolution::Unresolved) {
                    orientAlongOutwardAxis(toolpath, countersinkOutward);
                    toolpath.entryResolution =
                        countersinkFallback.entryResolution;
                    toolpath.preserveEntrySide = true;
                }
            } else if (kind == FeatureKind::ThroughHole) {
                const gp_Pnt* topologyOpening = hole.hasOpeningPoint
                    ? &hole.openingPoint : nullptr;
                const MoldSurfaceClassification surface =
                    classifyHoleSegmentByMoldSurface(
                        bounds, moldFrame, toolpath.entry, toolpath.bottom,
                        topologyOpening);
                if (surface.valid)
                    applyMoldSurfaceClassification(toolpath, surface);
            } else {
                resolveNonThroughEndpoints(
                    hole, moldFrame, localZMin, localZMax, hasThicknessRange,
                    toolpath.entry, toolpath.bottom, toolpath);
                // 盲孔优先保留拓扑开口；只有拓扑无法解析时才按六面回退。
                if (toolpath.entryResolution == HoleEntryResolution::Unresolved) {
                    const MoldSurfaceClassification surface =
                        classifyHoleSegmentByMoldSurface(
                            bounds, moldFrame, toolpath.entry, toolpath.bottom);
                    if (surface.valid)
                        applyMoldSurfaceClassification(toolpath, surface);
                }
            }

            // 拓扑已经可靠确定入口时，仅根据“孔底 -> 入口”的外向轴补齐
            // 六面提示，不再用端点距离推翻真实开口方向。
            if (kind != FeatureKind::CountersinkHole &&
                toolpath.entryResolution != HoleEntryResolution::Unresolved &&
                toolpath.setupFace == MoldSetupFace::Unresolved) {
                const gp_Vec outward(toolpath.bottom, toolpath.entry);
                toolpath.setupFace = classifyMoldSetupFace(moldFrame, outward);
            }
            toolpath.sourceFaceIds = sourceFaceIds;
            toolpath.stageWallFaceIds = stableFaceIds(
                stage.cylinderFaceIds, graph, indexer);
            const bool entryAtMinimum =
                toolpath.entry.Distance(axisPoint(stage.minimum)) <=
                toolpath.entry.Distance(axisPoint(stage.maximum));
            toolpath.bottomProfile = stageBottomProfile(
                stage, hole, graph, entryAtMinimum);
            if (toolpath.stageRole ==
                    HoleStageRole::CountersinkSmallDiameter &&
                toolpath.bottomProfile == HoleBottomProfile::Unknown &&
                oppositeEndpointTouchesMoldSurface(
                    toolpath, bounds, moldFrame)) {
                toolpath.bottomProfile = HoleBottomProfile::Open;
            }
            if (kind == FeatureKind::CountersinkHole &&
                &stage != ownerStagePointer &&
                toolpath.entryResolution ==
                    HoleEntryResolution::CountersinkLargestStage) {
                toolpath.parentRelation = HoleParentRelation::CoaxialStage;
                toolpath.parentCounterboreFaceId = ownerStableFaceId;
                toolpath.parentCounterboreRadius = ownerRadius;
            }
            if (toolpath.entry.Distance(toolpath.bottom) >= Precision::Confusion()) {
                HoleToolpathCandidate candidate;
                candidate.toolpath = std::move(toolpath);
                candidate.countersinkOwnerRadius = ownerRadius;
                candidate.cylinderFaceIds = stage.cylinderFaceIds;
                candidate.stableCylinderFaceId = stableFaceId(
                    stage.cylinderFaceIds, graph, indexer);
                candidate.terminalPlanarFaceIds =
                    hole.planarBottomFaceIds;
                candidate.terminalConicalFaceIds =
                    hole.conicalBottomFaceIds;
                candidate.largestCountersinkStage =
                    kind == FeatureKind::CountersinkHole &&
                    &stage == ownerStagePointer;
                if (candidate.toolpath.bottomProfile ==
                    HoleBottomProfile::Planar) {
                    candidate.planarBoundaryFaceIds =
                        hole.planarBottomFaceIds;
                }
                candidates.push_back(std::move(candidate));
            }
        }
    }

    std::vector<HoleRecognitionRecord> records = resolveHoleToolpathRelations(
        std::move(candidates), bounds, moldFrame, graph, indexer);
    std::vector<RecognizedHoleGeometry> geometries;
    geometries.reserve(records.size());
    for (HoleRecognitionRecord& record : records) {
        RecognizedHoleGeometry geometry;
        geometry.kind = record.kind;
        geometry.entry = record.entry;
        geometry.bottom = record.bottom;
        geometry.radius = record.radius;
        geometry.bottomProfile = record.bottomProfile;
        geometry.sourceFaceIds = std::move(record.sourceFaceIds);
        geometry.stageWallFaceIds = std::move(record.stageWallFaceIds);
        geometry.stageRole = record.stageRole;
        geometry.parentRelation = record.parentRelation;
        geometry.parentCounterboreFaceId =
            std::move(record.parentCounterboreFaceId);
        geometry.parentCounterboreFloorFaceId =
            std::move(record.parentCounterboreFloorFaceId);
        geometry.parentCounterboreRadius = record.parentCounterboreRadius;
        geometries.push_back(std::move(geometry));
    }
    return geometries;
}

} // namespace oap::recognition::detail
