// 程序化 OCCT 几何回归测试：无需 STEP fixture，覆盖通孔与盲孔边界。
#include "recognition/AAG.h"
#include "recognition/Recognition.h"
#include "recognition/RecognitionGeometryCache.h"
#include "recognition/FeatureRecognizer.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

struct RecognitionFixture {
    AAG graph;
    oap::RecognitionGeometryCache geometry;
    std::vector<HoleResult> holes;
};

RecognitionFixture recognize(const TopoDS_Shape& shape)
{
    RecognitionFixture result;
    result.graph.build(shape);
    result.geometry.build(shape, result.graph);
    result.holes = RecognizeHolesOCCT(result.graph, result.geometry);
    return result;
}

const HoleResult* findHole(const std::vector<HoleResult>& holes,
                           HoleType type,
                           double radius)
{
    const auto found = std::find_if(
        holes.begin(), holes.end(),
        [type, radius](const HoleResult& hole) {
            return hole.type == type &&
                   std::abs(hole.radius - radius) < 1.0e-6;
        });
    return found == holes.end() ? nullptr : &*found;
}

bool recognizesProgrammaticThroughHole()
{
    const TopoDS_Shape stock =
        BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
    const TopoDS_Shape cutter = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(20.0, 20.0, -1.0), gp_Dir(0.0, 0.0, 1.0)),
        3.0,
        12.0).Shape();
    RecognitionFixture fixture = recognize(
        BRepAlgoAPI_Cut(stock, cutter).Shape());
    const HoleResult* hole = findHole(
        fixture.holes, HoleType::THROUGH, 3.0);
    return expect(hole != nullptr,
                  "a full cylindrical cut should be recognized as through") &&
           expect(hole && hole->faces.size() == 1,
                  "the through-hole structure should contain only its wall") &&
           expect(hole && !hole->hasOpeningPoint,
                  "a through hole should not require one opening hint") &&
           expect(hole && std::none_of(
                       hole->faces.begin(), hole->faces.end(),
                       [&fixture](int faceIndex) {
                           return fixture.geometry.isMainOuterFace(faceIndex);
                       }),
                  "outer stock faces must not enter the hole structure");
}

bool recognizesProgrammaticBlindHole()
{
    const TopoDS_Shape stock =
        BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
    const TopoDS_Shape cutter = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(20.0, 20.0, 7.0), gp_Dir(0.0, 0.0, 1.0)),
        3.0,
        4.0).Shape();
    RecognitionFixture fixture = recognize(
        BRepAlgoAPI_Cut(stock, cutter).Shape());
    const HoleResult* hole = findHole(
        fixture.holes, HoleType::BLIND, 3.0);
    return expect(hole != nullptr,
                  "a cylindrical cut retaining a floor should be blind") &&
           expect(hole && hole->faces.size() >= 2,
                  "a blind-hole region should retain wall and floor") &&
           expect(hole && hole->hasOpeningPoint &&
                      std::abs(hole->openingPoint.Z() - 10.0) < 1.0e-6,
                  "the opening hint should lie on the stock exterior") &&
           expect(hole && hole->planarBottomFaceIds.size() == 1 &&
                      hole->conicalBottomFaceIds.empty(),
                  "a flat blind hole should expose one planar bottom");
}

struct IndexedFixture {
    Handle(TDocStd_Document) document;
    oap::Model model;
    oap::StepIndexer indexer;
    bool valid = false;

    explicit IndexedFixture(const std::vector<TopoDS_Shape>& shapes)
    {
        auto application = XCAFApp_Application::GetApplication();
        application->NewDocument("MDTV-XCAF", document);
        const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        for (const auto& shape : shapes) shapeTool->AddShape(shape, Standard_False);
        valid = indexer.build(document, model);
    }
    ~IndexedFixture()
    {
        if (!document.IsNull()) XCAFApp_Application::GetApplication()->Close(document);
    }
};

bool sameHoles(const oap::RecognitionResult& expected, const oap::RecognitionResult& actual)
{
    if (expected.faceKind != actual.faceKind || expected.holeCount != actual.holeCount ||
        expected.holeGeometries.size() != actual.holeGeometries.size()) return false;
    for (std::size_t index = 0; index < expected.holeGeometries.size(); ++index) {
        const auto& first = expected.holeGeometries[index];
        const auto& second = actual.holeGeometries[index];
        const bool sameEndpoints = first.entry.Distance(second.entry) <= 1e-8 &&
            first.bottom.Distance(second.bottom) <= 1e-8;
        // With no model envelope, a through hole has no preferred entry end;
        // its physical axis segment must nevertheless remain identical.
        const bool reversedThrough = first.kind == oap::FeatureKind::ThroughHole &&
            first.entry.Distance(second.bottom) <= 1e-8 && first.bottom.Distance(second.entry) <= 1e-8;
        if (first.kind != second.kind || first.sourceFaceIds != second.sourceFaceIds ||
            first.stageWallFaceIds != second.stageWallFaceIds ||
            first.bottomProfile != second.bottomProfile ||
            std::abs(first.radius - second.radius) > 1e-8 ||
            (!sameEndpoints && !reversedThrough))
            return false;
    }
    return true;
}

bool featureRecognitionReusesPreciseBoxesConservatively()
{
    const auto first = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(16, 16, 10).Shape(),
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(8, 8, -1), gp_Dir(0, 0, 1)), 2, 12).Shape()).Shape();
    const auto second = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(gp_Pnt(24, 0, 0), 16, 16, 10).Shape(),
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(32, 8, 7), gp_Dir(0, 0, 1)), 3, 4).Shape()).Shape();
    IndexedFixture fixture({first, second});
    if (!expect(fixture.valid && fixture.model.units.size() == 2, "two-solid indexed fixture failed")) return false;
    auto recognizer = oap::FeatureRecognizer{};
    const auto cached = recognizer.recognize(fixture.model, fixture.indexer);
    if (!expect(cached.holeCount == 2 && cached.holeGeometries.size() == 2,
        "compound recognition must preserve both solids' holes")) return false;
    const auto holeOnly = recognizer.recognizeHoleGeometries(fixture.model, fixture.indexer);
    if (!expect(holeOnly.geometries.size() == cached.holeGeometries.size(),
        "hole-only and full feature entry points disagree")) return false;

    auto partial = fixture.model;
    partial.units.back().bbox = {};
    partial.faces.front().bbox = {};
    if (!expect(sameHoles(cached, recognizer.recognize(partial, fixture.indexer)),
        "missing one solid/face bound must use exact fallback, not a partial compound union")) return false;
    auto missing = fixture.model;
    for (auto& unit : missing.units) unit.bbox = {};
    for (auto& face : missing.faces) face.bbox = {};
    if (!expect(sameHoles(cached, recognizer.recognize(missing, fixture.indexer)),
        "fully missing indexed bounds changed hole geometry or membership")) return false;
    auto nonfinite = fixture.model;
    nonfinite.units.front().bbox.xmax = std::numeric_limits<double>::infinity();
    nonfinite.faces.front().bbox.xmax = std::numeric_limits<double>::infinity();
    return expect(sameHoles(cached, recognizer.recognize(nonfinite, fixture.indexer)),
        "nonfinite indexed bounds did not use the precise geometric fallback");
}

bool skipsNonCylindricalFreeformHoleWork()
{
    const auto shape = BRepBuilderAPI_NurbsConvert(BRepPrimAPI_MakeBox(12, 8, 3).Shape(), Standard_True).Shape();
    IndexedFixture fixture({shape});
    if (!expect(fixture.valid, "freeform no-hole fixture could not be indexed")) return false;
    // Missing bounds are legitimate for independent recognition clients. A
    // model without analytic cylinders must still produce an empty result.
    for (auto& unit : fixture.model.units) unit.bbox = {};
    for (auto& face : fixture.model.faces) face.bbox = {};
    const auto full = oap::FeatureRecognizer{}.recognize(fixture.model, fixture.indexer);
    const auto holes = oap::FeatureRecognizer{}.recognizeHoleGeometries(fixture.model, fixture.indexer);
    return expect(full.holeCount == 0 && full.instances.empty() && holes.geometries.empty(),
        "freeform faces without analytic cylinders produced false holes");
}

} // namespace

int main()
{
    if (!recognizesProgrammaticThroughHole() ||
        !recognizesProgrammaticBlindHole() ||
        !featureRecognitionReusesPreciseBoxesConservatively() ||
        !skipsNonCylindricalFreeformHoleWork()) {
        return 1;
    }
    std::cout << "OCCT recognition smoke tests passed\n";
    return 0;
}
