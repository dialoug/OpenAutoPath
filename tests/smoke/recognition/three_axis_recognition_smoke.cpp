// 三轴结构识别特征测试：程序化盲腔固定根平面，并直接覆盖纯用例边界。
#include "analysis/ModelIndex.h"
#include "analysis/StepIndexer.h"
#include "analysis/AabbBoundaryFace.h"
#include "recognition/AAG.h"
#include "recognition/AxisParallelPlaneRecognizer.h"
#include "recognition/AxisWireTraceTracker.h"
#include "recognition/RecognitionGeometryCache.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"
#include "toolpath/core/ToolpathParallel.h"
#include "RecognitionSampleBenchmark.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

TopoDS_Shape blindRectangularPocket()
{
    const TopoDS_Shape stock =
        BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
    const TopoDS_Shape cutter = BRepPrimAPI_MakeBox(
        gp_Pnt(12.0, 12.0, 7.0), 16.0, 16.0, 4.0).Shape();
    return BRepAlgoAPI_Cut(stock, cutter).Shape();
}

bool sameGeometryCache(const oap::RecognitionGeometryCache& first,
                       const oap::RecognitionGeometryCache& second,
                       int faceCount)
{
    if (first.shapeBox().IsVoid() != second.shapeBox().IsVoid() ||
        first.edgeFaces().Extent() != second.edgeFaces().Extent())
        return false;
    for (int face = 1; face <= faceCount; ++face) {
        if (first.isMainOuterFace(face) != second.isMainOuterFace(face) ||
            first.faceNeighbors(face) != second.faceNeighbors(face))
            return false;
        gp_Pnt firstCenter, secondCenter;
        if (first.faceBoxCenter(face, firstCenter) !=
            second.faceBoxCenter(face, secondCenter) ||
            firstCenter.Distance(secondCenter) > 1.0e-9)
            return false;
        gp_Pln firstPlane, secondPlane;
        const bool hasPlane = first.effectivePlane(face, firstPlane);
        if (hasPlane != second.effectivePlane(face, secondPlane) ||
            (hasPlane &&
             (firstPlane.Location().Distance(secondPlane.Location()) > 1.0e-9 ||
              !firstPlane.Axis().Direction().IsEqual(
                  secondPlane.Axis().Direction(), 1.0e-9))))
            return false;
        double firstMinimum = 0, firstMaximum = 0;
        double secondMinimum = 0, secondMaximum = 0;
        if (first.faceBoundaryEdgeProjectionRange(face, gp::DZ(), firstMinimum, firstMaximum) !=
            second.faceBoundaryEdgeProjectionRange(face, gp::DZ(), secondMinimum, secondMaximum) ||
            std::abs(firstMinimum - secondMinimum) > 1.0e-9 ||
            std::abs(firstMaximum - secondMaximum) > 1.0e-9)
            return false;
    }
    return true;
}

bool splineClassificationAndSuppliedBoundsRemainEquivalent()
{
    TColStd_Array1OfReal knots(1, 2);
    knots.SetValue(1, 0.0);
    knots.SetValue(2, 1.0);
    TColStd_Array1OfInteger multiplicities(1, 2);
    multiplicities.Init(3);
    const auto splineFace = [&](bool curved) {
        TColgp_Array2OfPnt poles(1, 3, 1, 3);
        for (int u = 1; u <= 3; ++u)
            for (int v = 1; v <= 3; ++v)
                poles.SetValue(u, v, gp_Pnt((u - 1) * 10.0 + (curved ? 20.0 : 0.0),
                    (v - 1) * 10.0, curved && u == 2 && v == 2 ? 15.0 : 10.0));
        Handle(Geom_BSplineSurface) surface = new Geom_BSplineSurface(
            poles, knots, knots, multiplicities, multiplicities, 2, 2);
        return BRepBuilderAPI_MakeFace(surface, 1.e-7).Face();
    };
    const auto planarSpline = splineFace(false);
    const auto curvedSpline = splineFace(true);
    if (!expect(BRepAdaptor_Surface(planarSpline).GetType() == GeomAbs_BSplineSurface &&
                BRepAdaptor_Surface(curvedSpline).GetType() == GeomAbs_BSplineSurface,
                "regression fixture must contain real B-spline surfaces"))
        return false;
    BRep_Builder builder;
    TopoDS_Compound shape;
    builder.MakeCompound(shape);
    builder.Add(shape, BRepPrimAPI_MakeBox(20.0, 20.0, 10.0).Shape());
    builder.Add(shape, planarSpline);
    builder.Add(shape, curvedSpline);
    builder.Add(shape, BRepBuilderAPI_MakeEdge(gp_Pnt(60, 0, 0), gp_Pnt(70, 0, 0)).Shape());
    AAG graph;
    graph.build(shape);
    Bnd_Box exactShapeBox;
    BRepBndLib::AddOptimal(shape, exactShapeBox, Standard_False, Standard_False);
    std::vector<Bnd_Box> faceBoxes(static_cast<std::size_t>(graph.faceMap.Extent() + 1));
    for (int index = 1; index <= graph.faceMap.Extent(); ++index)
        BRepBndLib::AddOptimal(graph.faceMap(index), faceBoxes[index],
                               Standard_False, Standard_False);
    const Bnd_Box oldReferenceBox = oap::buildAxisAlignedPlanarBox(shape);
    for (const auto mode : {oap::RecognitionGeometryCacheMode::MainOuterFacesOnly,
                            oap::RecognitionGeometryCacheMode::Full})
    for (const Bnd_Box& suppliedBox : {exactShapeBox, Bnd_Box{}}) {
        oap::RecognitionGeometryCache cache;
        // An empty supplied body box retains the complete-shape exact fallback,
        // including the isolated edge outside all faces.
        std::size_t ranges = 0;
        oap::RecognitionGeometryCacheControl control;
        control.executeRange = [&](std::size_t count, const auto& task) {
            ++ranges;
            for (std::size_t index = 0; index < count; ++index)
                task(index);
        };
        if (!expect(cache.build(shape, graph, suppliedBox, faceBoxes, mode, &control) &&
                    ranges == (mode == oap::RecognitionGeometryCacheMode::Full ? 2u : 1u),
                    "cached B-spline geometry build failed"))
            return false;
        Standard_Real ax0, ay0, az0, ax1, ay1, az1;
        Standard_Real bx0, by0, bz0, bx1, by1, bz1;
        cache.shapeBox().Get(ax0, ay0, az0, ax1, ay1, az1);
        exactShapeBox.Get(bx0, by0, bz0, bx1, by1, bz1);
        if (!expect(std::abs(ax0 - bx0) < 1.e-7 && std::abs(ay0 - by0) < 1.e-7 &&
                    std::abs(az0 - bz0) < 1.e-7 && std::abs(ax1 - bx1) < 1.e-7 &&
                    std::abs(ay1 - by1) < 1.e-7 && std::abs(az1 - bz1) < 1.e-7,
                    "missing body box must retain isolated geometry in the exact fallback"))
            return false;
        gp_Pln plane;
        if (!expect(cache.effectivePlane(graph.faceMap.FindIndex(planarSpline), plane) &&
                    !cache.effectivePlane(graph.faceMap.FindIndex(curvedSpline), plane),
                    "both cache modes must preserve planar/nonplanar B-spline classification"))
            return false;
        for (int index = 1; index <= graph.faceMap.Extent(); ++index) {
            const auto face = TopoDS::Face(graph.faceMap(index));
            if (!expect(cache.isMainOuterFace(index) ==
                        oap::isFaceOnAabbBoundary(face, oldReferenceBox, &faceBoxes[index]),
                        "cached B-spline boundaries differ from shape-based classification"))
                return false;
        }
    }
    return true;
}

bool geometryCacheUsesInjectedBudgetAndPublishesOnlyCompleteBuilds()
{
    const auto shape = blindRectangularPocket();
    AAG graph;
    graph.build(shape);
    const auto caller = std::this_thread::get_id();
    std::atomic_bool wrongThread{false};
    oap::RecognitionGeometryCacheControl serialControl;
    serialControl.isCancellationRequested = [&] {
        if (std::this_thread::get_id() != caller)
            wrongThread.store(true);
        return false;
    };
    oap::RecognitionGeometryCache serial;
    if (!expect(serial.build(shape, graph, oap::RecognitionGeometryCacheMode::Full,
                             &serialControl) && !wrongThread.load(),
                "recognition without an executor must run on the caller"))
        return false;

    for (std::size_t capacity : {0u, 1u}) {
        oap::toolpath::ToolpathParallelBudget budget(capacity);
        std::size_t acquired = 0, released = 0, rangeCount = 0;
        oap::toolpath::ToolpathGenerationControl budgetControl;
        budgetControl.tryAcquireParallelWorkers = [&](std::size_t requested) {
            const auto grant = budget.tryAcquire(requested);
            acquired += grant;
            return grant;
        };
        budgetControl.releaseParallelWorkers = [&](std::size_t count) {
            released += count;
            budget.release(count);
        };
        oap::toolpath::ToolpathParallelScope scope(&budgetControl);
        oap::RecognitionGeometryCacheControl control;
        control.executeRange = [&](std::size_t count, const auto& task) {
            ++rangeCount;
            oap::toolpath::toolpathParallelForBudgeted(count, task);
        };
        control.isCancellationRequested = [&] {
            if (capacity == 0u && std::this_thread::get_id() != caller)
                wrongThread.store(true);
            return false;
        };
        oap::RecognitionGeometryCache actual;
        if (!expect(actual.build(shape, graph, oap::RecognitionGeometryCacheMode::Full,
                                 &control) && rangeCount == 2u && !wrongThread.load() &&
                        acquired == released && budget.available() == capacity &&
                        (capacity == 0u || oap::toolpath::toolpathHardwareParallelism() == 1u || acquired > 0u) &&
                        sameGeometryCache(serial, actual, graph.faceMap.Extent()),
                    "injected recognition ranges must inherit zero/partial budget and preserve geometry"))
            return false;
    }

    // Cancel after partially computing either the face or edge range. Neither
    // a previous successful build nor this partial build may remain published.
    for (int cancelRange : {1, 2}) {
        oap::RecognitionGeometryCache cache = serial;
        bool cancelled = false;
        int range = 0;
        oap::RecognitionGeometryCacheControl control;
        control.isCancellationRequested = [&] { return cancelled; };
        control.executeRange = [&](std::size_t count, const auto& task) {
            ++range;
            for (std::size_t index = 0; index < count; ++index) {
                task(index);
                if (range == cancelRange)
                    cancelled = true;
            }
        };
        if (!expect(!cache.build(shape, graph, oap::RecognitionGeometryCacheMode::Full,
                                 &control) && range == cancelRange &&
                        cache.shapeBox().IsVoid() && cache.faceBox(1) == nullptr &&
                        cache.edgeFaces().IsEmpty(),
                    "cancellation must discard all partially built recognition cache data"))
            return false;
    }
    oap::RecognitionGeometryCache cache = serial;
    oap::RecognitionGeometryCacheControl throwingControl;
    throwingControl.executeRange = [](std::size_t count, const auto& task) {
        if (count > 0u)
            task(0u);
        throw std::runtime_error("injected recognition executor failure");
    };
    bool threw = false;
    try {
        cache.build(shape, graph, oap::RecognitionGeometryCacheMode::Full, &throwingControl);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    return expect(threw && cache.shapeBox().IsVoid() && cache.faceBox(1) == nullptr,
                  "executor failure must not publish partial recognition cache data");
}

struct TraceSnapshot {
    oap::AxisParallelPlaneRecognitionResult planes;
    oap::CombinedWireTraceResult trace;
};

TraceSnapshot recognize(const TopoDS_Shape& shape)
{
    AAG graph;
    graph.build(shape);
    oap::RecognitionGeometryCache geometry;
    geometry.build(shape, graph);

    TraceSnapshot result;
    result.planes = oap::RecognizeAxisParallelPlanesOCCT(
        shape, graph, gp::DZ(), {}, geometry);
    result.trace = oap::TraceCombinedWireLoopsAlongAxisOCCT(
        shape, graph, gp::DZ(), {}, geometry);
    return result;
}

std::vector<int> sorted(std::vector<int> values)
{
    std::sort(values.begin(), values.end());
    return values;
}

bool recognizesStableBlindPocketTrace()
{
    const TopoDS_Shape shape = blindRectangularPocket();
    const TraceSnapshot first = recognize(shape);
    const TraceSnapshot second = recognize(shape);

    if (!expect(first.planes.faceIndices.size() == 1,
                "one internal Z-parallel pocket floor should remain") ||
        !expect(first.trace.rootFaceCount == 3 &&
                    first.trace.paths.size() == 3,
                "the current combined trace should retain three eligible roots") ||
        !expect(first.trace.seedWireCount == 4,
                "the current roots should contribute four seed wires") ||
        !expect(first.trace.traversedFaceIndices.size() == 4,
                "the trace should contain the four pocket walls") ||
        !expect(first.trace.terminalFaceIndices.size() == 1,
                "the stock top should remain a terminal face")) {
        std::cerr << "observed planes=" << first.planes.faceIndices.size()
                  << " roots=" << first.trace.rootFaceCount
                  << " paths=" << first.trace.paths.size()
                  << " seeds=" << first.trace.seedWireCount
                  << " traversed=" << first.trace.traversedFaceIndices.size()
                  << " terminal=" << first.trace.terminalFaceIndices.size()
                  << '\n';
        return false;
    }

    return expect(first.planes.faceIndices == second.planes.faceIndices &&
                      sorted(first.trace.traversedFaceIndices) ==
                          sorted(second.trace.traversedFaceIndices) &&
                      sorted(first.trace.terminalFaceIndices) ==
                          sorted(second.trace.terminalFaceIndices),
                  "repeated recognition should preserve stable face ordering and membership");
}

struct IndexedShape {
    Handle(TDocStd_Document) document;
    oap::Model model;
    oap::ModelIndex modelIndex;
    oap::StepIndexer stepIndexer;
    bool valid = false;

    explicit IndexedShape(const TopoDS_Shape& shape)
    {
        Handle(XCAFApp_Application) application =
            XCAFApp_Application::GetApplication();
        application->NewDocument("MDTV-XCAF", document);
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        shapeTool->AddShape(shape, Standard_False);
        valid = stepIndexer.build(document, model);
        if (valid)
            modelIndex.build(model);
    }
};

std::vector<std::string> sortedValues(
    const std::unordered_set<std::string>& values)
{
    std::vector<std::string> ordered(values.begin(), values.end());
    std::sort(ordered.begin(), ordered.end());
    return ordered;
}

std::vector<std::string> subsetSignature(
    const oap::ThreeAxisWireTraceUseCaseResult& result)
{
    std::vector<std::string> signature;
    for (const oap::ThreeAxisFeatureSubset& subset : result.featureSubsets) {
        std::vector<std::string> faces = subset.faceIds;
        std::sort(faces.begin(), faces.end());
        std::string value = subset.axisLabel + "|" + subset.rootFaceId +
            "|" + subset.rootWireId + "|" +
            std::to_string(static_cast<int>(subset.featureKind));
        for (const std::string& face : faces)
            value += "|" + face;
        signature.push_back(std::move(value));
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

std::vector<std::string> profileSourceSignature(
    const oap::ThreeAxisWireTraceUseCaseResult& result)
{
    std::vector<std::string> signature;
    for (const oap::ThreeAxisProfileSource& source : result.profileSources) {
        signature.push_back(
            source.unitId + "|" + source.axisLabel + "|" +
            source.rootFaceId + "|" + source.rootWireId);
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

bool executesStablePureUseCase()
{
    IndexedShape fixture(blindRectangularPocket());
    if (!expect(fixture.valid, "programmatic XCAF shape should be indexed"))
        return false;

    oap::ThreeAxisWireTraceRequest request;
    request.model = &fixture.model;
    request.modelIndex = &fixture.modelIndex;
    request.stepIndexer = &fixture.stepIndexer;

    std::vector<std::pair<std::size_t, std::size_t>> progress;
    oap::ThreeAxisWireTraceControl control;
    std::size_t executedRanges = 0;
    control.executeRange = [&](std::size_t count, const auto& task) {
        ++executedRanges;
        oap::toolpath::toolpathParallelForBudgeted(count, task);
    };
    control.reportProgress = [&progress](std::size_t completed,
                                         std::size_t total) {
        progress.emplace_back(completed, total);
    };

    const oap::ThreeAxisWireTraceUseCase useCase;
    const oap::ThreeAxisWireTraceUseCaseResult first =
        useCase.execute(request, &control);
    const oap::ThreeAxisWireTraceUseCaseResult second =
        useCase.execute(request);
    if (!expect(first.ok() && second.ok() && executedRanges >= 2u,
                "pure three-axis use case should complete") ||
        !expect(!first.combinedTraceFaceIds.empty(),
                "pure use case should publish traced pocket faces") ||
        !expect(!first.featureSubsets.empty(),
                "pure use case should publish final feature subsets") ||
        !expect(!first.pocketSources.empty(),
                "pure use case should publish toolpath-neutral pocket sources") ||
        !expect(!first.profileSources.empty(),
                "pure use case should publish automatic outer-profile sources") ||
        !expect(progress.size() >= 2 && progress.front().first == 0 &&
                    progress.back().first == fixture.model.units.size() &&
                    progress.back().second == fixture.model.units.size(),
                "pure use case should report initial and completed unit progress")) {
        std::cerr << "observed use-case trace="
                  << first.combinedTraceFaceIds.size()
                  << " subsets=" << first.featureSubsets.size()
                  << " sources=" << first.pocketSources.size()
                  << " error=" << first.error << '\n';
        return false;
    }

    if (!expect(completeThreeAxisRecognitionSignature(first) ==
                    completeThreeAxisRecognitionSignature(second) &&
                    sortedValues(first.combinedTraceFaceIds) ==
                    sortedValues(second.combinedTraceFaceIds) &&
                    sortedValues(first.throughStructureFaceIds) ==
                    sortedValues(second.throughStructureFaceIds) &&
                    sortedValues(first.axisParallelPlaneFaceIds) ==
                        sortedValues(second.axisParallelPlaneFaceIds) &&
                    subsetSignature(first) == subsetSignature(second) &&
                    profileSourceSignature(first) ==
                        profileSourceSignature(second),
                "repeated pure use-case execution should be stable")) {
        return false;
    }

    oap::ThreeAxisWireTraceControl cancelledControl;
    cancelledControl.isCancellationRequested = [] { return true; };
    const oap::ThreeAxisWireTraceUseCaseResult cancelled =
        useCase.execute(request, &cancelledControl);
    if (!expect(cancelled.cancelled && !cancelled.ok() &&
                      cancelled.featureSubsets.empty() &&
                      cancelled.pocketSources.empty() &&
                      cancelled.profileSources.empty(),
                  "pre-cancelled use case must not publish committed subsets"))
        return false;
    bool cancelInCache = false;
    cancelledControl.isCancellationRequested = [&] { return cancelInCache; };
    cancelledControl.executeRange = [&](std::size_t count, const auto& task) {
        if (count > 0u)
            task(0u);
        cancelInCache = true;
        for (std::size_t index = 1; index < count; ++index)
            task(index);
    };
    const auto interrupted = useCase.execute(request, &cancelledControl);
    return expect(interrupted.cancelled && interrupted.featureSubsets.empty() &&
                      interrupted.pocketSources.empty() && interrupted.profileSources.empty(),
                  "cache-stage cancellation must stop the use case before publishing features");
}

} // namespace

int main(int argc, char** argv)
{
    if (argc > 1)
        return runRecognitionSampleBenchmark(argc, argv);
    if (!splineClassificationAndSuppliedBoundsRemainEquivalent() ||
        !geometryCacheUsesInjectedBudgetAndPublishesOnlyCompleteBuilds() ||
        !recognizesStableBlindPocketTrace() ||
        !executesStablePureUseCase())
        return 1;
    std::cout << "three-axis recognition smoke tests passed\n";
    return 0;
}
