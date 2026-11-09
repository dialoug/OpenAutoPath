// 中立刀路程序快速回归测试：覆盖程序验证、分组和事务式 NC 导出。
#include "toolpath/program/GCodePostProcessor.h"
#include "toolpath/program/ToolpathProgramExport.h"
#include "toolpath/core/validation/ToolpathMoveValidator.h"
#include "toolpath/core/validation/ToolpathPrimitiveValidator.h"
#include "toolpath/core/ToolpathPrimitiveMapping.h"
#include "toolpath/core/simulation/StockModelLoader.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"
#include "analysis/ModelIO.h"
#include "analysis/AnalysisPipeline.h"
#include "analysis/ContentHash.h"
#include "analysis/Utf8Path.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <OSD.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <XCAFApp_Application.hxx>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

oap::toolpath::ToolpathOperation operation(
    std::string id,
    int setupIndex = 0,
    oap::MoldSetupFace setupFace = oap::MoldSetupFace::PositiveZ)
{
    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = gp_Pnt(0.0, 0.0, 0.0);
    cut.to = gp_Pnt(5.0, 0.0, 0.0);
    cut.feed = 500.0;

    oap::toolpath::ToolpathOperation result;
    result.operationId = std::move(id);
    result.type = oap::toolpath::ToolpathOperationType::PocketClearing;
    result.featureKind = oap::FeatureKind::Pocket;
    result.setupIndex = setupIndex;
    result.setupFace = setupFace;
    result.moves.push_back(cut);
    return result;
}

bool hasIssue(const oap::toolpath::ToolpathValidationReport& report,
              const std::string& code)
{
    return std::any_of(
        report.issues.begin(), report.issues.end(),
        [&code](const oap::toolpath::ToolpathIssue& issue) {
            return issue.code == code;
        });
}

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()};
}

bool programModelAndValidationAreStable()
{
    oap::toolpath::ToolpathProgram program;
    program.operations.push_back(operation("pocket-z"));
    program.operations.push_back(
        operation("pocket-x", 1, oap::MoldSetupFace::PositiveX));

    const oap::toolpath::ToolpathParams params;
    const oap::toolpath::ToolpathProgramValidator validator;
    if (!expect(!program.empty() && program.moveCount() == 2 &&
                    program.flattenedMoves().size() == 2 &&
                    program.operationById("pocket-x") != nullptr,
                "the program model should preserve operations and moves") ||
        !expect(validator.validate(program, params).ok(),
                "a finite two-setup program should validate")) {
        return false;
    }

    const auto groups =
        oap::toolpath::splitToolpathProgramBySetupToolAndPhase(program);
    if (!expect(groups.size() == 2 &&
                    groups[0].operations.front().operationId == "pocket-z" &&
                    groups[1].operations.front().operationId == "pocket-x",
                "program splitting should retain first-seen setup order")) {
        return false;
    }

    oap::toolpath::ToolpathProgram duplicate = program;
    duplicate.operations.push_back(duplicate.operations.front());
    const auto duplicateReport = validator.validate(duplicate, params);
    if (!expect(!duplicateReport.ok() &&
                    hasIssue(duplicateReport, "duplicate_operation_id"),
                "duplicate operation ids should be rejected")) {
        return false;
    }

    oap::toolpath::ToolpathProgram mixedTools = program;
    mixedTools.operations.front().tool.valid = true;
    mixedTools.operations.front().tool.toolNumber = 7;
    mixedTools.operations.front().tool.diameter = 8.0;
    const auto mixedReport = validator.validate(mixedTools, params);
    if (!expect(!mixedReport.ok() &&
                    hasIssue(mixedReport, "mixed_operation_tool_context"),
                "mixed bound and unbound tool contexts should be rejected")) {
        return false;
    }

    oap::toolpath::ToolpathProgram conflictingSetup = program;
    conflictingSetup.operations[1].setupIndex = 0;
    const auto setupReport = validator.validate(conflictingSetup, params);
    return expect(!setupReport.ok() &&
                      hasIssue(setupReport, "conflicting_setup_face"),
                  "one setup index must not map to two mold faces");
}

bool largeStructuralValidationIsDeterministic()
{
    constexpr std::size_t itemCount = 9000;
    std::vector<oap::toolpath::ToolpathMove> moves;
    moves.reserve(itemCount);
    for (std::size_t index = 0; index < itemCount; ++index) {
        oap::toolpath::ToolpathMove move;
        move.type = oap::toolpath::ToolpathMoveType::Cut;
        move.from = gp_Pnt(static_cast<double>(index), 0.0, 0.0);
        move.to = gp_Pnt(static_cast<double>(index + 1u), 0.0, 0.0);
        move.feed = 500.0;
        moves.push_back(move);
    }

    std::vector<oap::toolpath::ToolpathMove> invalidMoves = moves;
    invalidMoves[1000].from.SetY(0.25);
    invalidMoves[2000].feed = 0.0;
    invalidMoves[7000].from.SetY(0.25);
    oap::toolpath::ToolpathValidationReport moveReport;
    const oap::toolpath::ToolpathMoveStats stats =
        oap::toolpath::validateToolpathMoves(
            invalidMoves, "large-moves", moveReport);
    std::vector<int> indexedMoveIssues;
    for (const oap::toolpath::ToolpathIssue& issue : moveReport.issues) {
        if (issue.moveIndex >= 0)
            indexedMoveIssues.push_back(issue.moveIndex);
    }
    if (!expect(
            stats.cut == static_cast<int>(itemCount) &&
                indexedMoveIssues == std::vector<int>({1000, 2000, 7000}),
            "parallel move validation should preserve ordered diagnostics")) {
        return false;
    }

    std::vector<oap::toolpath::ToolpathPrimitive> primitives;
    primitives.reserve(itemCount);
    for (const oap::toolpath::ToolpathMove& move : moves)
        primitives.push_back(oap::toolpath::makeLinearPrimitive(move));
    primitives[1000].context.ring = 1;
    primitives[7000].context.ring = 1;
    oap::toolpath::ToolpathValidationReport primitiveReport;
    oap::toolpath::validateToolpathPrimitives(
        primitives, "large-primitives", primitiveReport);
    if (!expect(
            primitiveReport.issues.size() == 2u &&
                primitiveReport.issues[0].moveIndex == 1000 &&
                primitiveReport.issues[1].moveIndex == 7000,
            "parallel primitive validation should preserve source order")) {
        return false;
    }

    primitives[1000] = oap::toolpath::makeLinearPrimitive(moves[1000]);
    primitives[7000] = oap::toolpath::makeLinearPrimitive(moves[7000]);
    moves[7000].feed = 400.0;
    oap::toolpath::ToolpathValidationReport expansionReport;
    oap::toolpath::validateToolpathPrimitiveExpansion(
        primitives, moves, "large-expansion", expansionReport, true);
    return expect(
        expansionReport.errorCount == 1 &&
            expansionReport.issues.front().moveIndex == 7000,
        "parallel expansion validation should report the earliest mismatch");
}

bool primitiveMoveMappingPreservesNcSemantics()
{
    using namespace oap::toolpath;

    ToolpathMove rapid;
    rapid.type = ToolpathMoveType::Rapid;
    rapid.from = gp_Pnt(0.0, 0.0, 0.0);
    rapid.to = gp_Pnt(1.0, 0.0, 0.0);

    ToolpathMove arcFirst;
    arcFirst.type = ToolpathMoveType::Cut;
    arcFirst.from = rapid.to;
    arcFirst.to = gp_Pnt(0.7, 0.7, 0.0);
    arcFirst.feed = 450.0;
    ToolpathMove arcSecond = arcFirst;
    arcSecond.from = arcFirst.to;
    arcSecond.to = gp_Pnt(0.0, 1.0, 0.0);

    ToolpathMove drillFirst;
    drillFirst.type = ToolpathMoveType::Cut;
    drillFirst.from = arcSecond.to;
    drillFirst.to = gp_Pnt(0.0, 1.0, -2.0);
    drillFirst.feed = 120.0;
    ToolpathMove drillSecond = drillFirst;
    drillSecond.type = ToolpathMoveType::Retract;
    drillSecond.from = drillFirst.to;
    drillSecond.to = gp_Pnt(0.0, 1.0, 0.0);

    ToolpathCircularArc arc;
    arc.direction = ToolpathArcDirection::Counterclockwise;
    arc.from = arcFirst.from;
    arc.to = arcSecond.to;
    arc.center = gp_Pnt(0.0, 0.0, 0.0);
    arc.axis = gp_Vec(0.0, 0.0, 1.0);
    arc.sweepRadians = 1.5707963267948966;
    arc.feed = 450.0;

    ToolpathDrillCycle cycle;
    cycle.type = ToolpathDrillCycleType::Peck;
    cycle.entry = drillFirst.from;
    cycle.bottom = drillFirst.to;
    cycle.retract = drillSecond.to;
    cycle.peckDepth = 2.0;
    cycle.feed = 120.0;

    const std::vector<ToolpathMove> moves{
        rapid, arcFirst, arcSecond, drillFirst, drillSecond};
    const std::vector<ToolpathPrimitive> primitives{
        makeLinearPrimitive(rapid),
        makeArcPrimitive(arc, {arcFirst, arcSecond}),
        makeDrillCyclePrimitive(cycle, {drillFirst, drillSecond})};
    const std::vector<ToolpathMoveSemantic> mapped =
        mapToolpathMovesToPrimitives(moves, primitives);
    if (!expect(
            mapped.size() == moves.size() &&
                mapped[0].commandName == "G0" &&
                mapped[1].commandName == "G3" &&
                mapped[2].primitiveIndex == 1 &&
                mapped[2].primitiveMoveIndex == 1 &&
                mapped[2].primitiveMoveCount == 2 &&
                mapped[3].commandName == "G83" &&
                mapped[4].commandSummary.find("Q2.000") !=
                    std::string::npos,
            "fallback moves should map to their arc and drill-cycle commands")) {
        return false;
    }

    ToolpathMove helixA;
    helixA.type = ToolpathMoveType::Cut;
    helixA.from = gp_Pnt(1.0, 0.0, 0.0);
    helixA.to = gp_Pnt(0.0, 1.0, -0.25);
    ToolpathMove helixB = helixA;
    helixB.from = helixA.to;
    helixB.to = gp_Pnt(-1.0, 0.0, -0.5);
    ToolpathMove helixC = helixA;
    helixC.from = helixB.to;
    helixC.to = gp_Pnt(0.0, -1.0, -0.75);
    ToolpathMove helixD = helixA;
    helixD.from = helixC.to;
    helixD.to = gp_Pnt(1.0, 0.0, -1.0);
    ToolpathCircularArc helix = arc;
    helix.from = helixA.from;
    helix.to = helixD.to;
    helix.sweepRadians = 6.283185307179586;
    const std::vector<ToolpathMoveSemantic> inferred =
        mapToolpathMovesToPrimitives(
            {helixA, helixB, helixC, helixD},
            {makeArcPrimitive(helix, {})});
    return expect(
        inferred.size() == 4u &&
            inferred.front().commandName == "G3" &&
            inferred.back().primitiveMoveIndex == 3 &&
            inferred.back().primitiveMoveCount == 4,
        "analytic helix arcs should recover their sampled move span");
}

class FailingPostProcessor final : public oap::toolpath::GCodePostProcessor {
public:
    bool write(
        std::ostream& output,
        const oap::toolpath::ToolpathProgram&,
        const oap::toolpath::ToolpathParams&,
        const oap::toolpath::MachiningSetup&,
        const oap::toolpath::GCodeProgramOptions&,
        std::string& error) const override
    {
        output << "partial output that must never reach disk";
        error = "deliberate post failure";
        return false;
    }
};

bool exportValidatesAndBuffersBeforeOverwrite()
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        oap::utf8Path("oap_中文_繁體_𠮷_🙂_smoke");
    std::error_code filesystemError;
    std::filesystem::create_directories(directory, filesystemError);
    if (!expect(!filesystemError,
                "the NC test directory should be available")) {
        return false;
    }
    const std::filesystem::path outputPath =
        directory / oap::utf8Path("程序_刀路.nc");
    const std::string outputPathUtf8 = oap::pathUtf8(outputPath);
    {
        std::ofstream existing(outputPath, std::ios::binary | std::ios::trunc);
        existing << "sentinel";
    }

    oap::toolpath::ToolpathProgram invalid;
    invalid.operations.push_back(operation("invalid"));
    invalid.operations.front().moves.front().to =
        invalid.operations.front().moves.front().from;
    oap::toolpath::ToolpathParams params;
    std::string error;
    const oap::toolpath::ToolpathProgramExport exporter;
    if (!expect(!exporter.writeGCode(
                    invalid, params, outputPathUtf8, error) &&
                    error.find("validation failed") != std::string::npos &&
                    readFile(outputPath) == "sentinel",
                "invalid export must preserve an existing NC file")) {
        return false;
    }

    oap::toolpath::ToolpathProgram valid;
    valid.operations.push_back(operation("valid"));
    const FailingPostProcessor failingPost;
    oap::toolpath::MachiningSetup setup;
    oap::toolpath::GCodeProgramOptions options;
    error.clear();
    if (!expect(!exporter.writeGCode(
                    valid,
                    params,
                    setup,
                    options,
                    outputPathUtf8,
                    error,
                    &failingPost) &&
                    error == "deliberate post failure" &&
                    readFile(outputPath) == "sentinel",
                "post-processing must finish in memory before overwrite")) {
        return false;
    }

    error.clear();
    const bool exported = exporter.writeGCode(
        valid, params, outputPathUtf8, error);
    const std::string nc = readFile(outputPath);
    std::filesystem::remove_all(directory, filesystemError);
    return expect(exported && error.empty() &&
                      nc.find("M30") != std::string::npos,
                  "a valid program should produce a complete NC file");
}

bool unicodePathIoIsIndependentOfTheActiveCodePage()
{
    const std::string unicodeDirectoryName =
        "oap_路径_路徑_𠮷_🙂_io";
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        oap::utf8Path(unicodeDirectoryName);
    std::error_code filesystemError;
    std::filesystem::remove_all(directory, filesystemError);
    filesystemError.clear();
    std::filesystem::create_directories(directory, filesystemError);
    if (!expect(!filesystemError,
                "the full-Unicode I/O directory should be available")) {
        return false;
    }

    bool ok = expect(
        oap::pathUtf8(directory.filename()) == unicodeDirectoryName,
        "UTF-8 and native paths should round-trip without replacement");

    const std::string entityName = "中文零件_繁體_𠮷_🙂";
    const TCollection_ExtendedString extendedName(
        entityName.c_str(), Standard_True);
    const TCollection_AsciiString restoredName(extendedName);
    ok = expect(restoredName.IsEqual(entityName.c_str()),
                "OCCT entity names should round-trip as UTF-8") && ok;
    const std::filesystem::path sourcePath =
        directory / oap::utf8Path("源文件_繁體_𠮷_🙂.bin");
    {
        std::ofstream source(sourcePath, std::ios::binary);
        source << "unicode-path-content";
        ok = expect(static_cast<bool>(source),
                    "a native Unicode path should be writable") && ok;
    }
    std::string digest;
    ok = expect(oap::sha256FileHex(oap::pathUtf8(sourcePath), digest) &&
                    digest.size() == 64u,
                "UTF-8 path hashing should open a full-Unicode path") && ok;

    oap::Model model;
    oap::Unit unit;
    unit.unit_id = "U1";
    unit.unit_name = entityName;
    unit.decoded_name = unit.unit_name;
    unit.face_ids = {"F1"};
    model.units.push_back(unit);
    oap::Face face;
    face.face_id = "F1";
    face.unit_id = unit.unit_id;
    face.edge_ids = {"E1"};
    face.wire_ids = {"W1"};
    model.faces.push_back(face);
    oap::Edge edge;
    edge.edge_id = "E1";
    edge.unit_id = unit.unit_id;
    edge.face_ids = {face.face_id};
    model.edges.push_back(edge);
    oap::Wire wire;
    wire.wire_id = "W1";
    wire.face_id = face.face_id;
    wire.edge_ids = {edge.edge_id};
    model.wires.push_back(wire);
    const std::filesystem::path modelPath =
        directory / oap::utf8Path("缓存_快取_𠮷_🙂.bin");
    oap::Model restored;
    ok = expect(oap::writeModel(oap::pathUtf8(modelPath), model) &&
                    oap::readModel(oap::pathUtf8(modelPath), restored) &&
                    restored.units.size() == 1u &&
                    restored.units.front().decoded_name == unit.decoded_name &&
                    restored.units.front().face_ids == unit.face_ids &&
                    restored.faces.size() == 1u &&
                    restored.faces.front().edge_ids == face.edge_ids &&
                    restored.faces.front().wire_ids == face.wire_ids &&
                    restored.edges.size() == 1u &&
                    restored.edges.front().face_ids == edge.face_ids &&
                    restored.wires.size() == 1u &&
                    restored.wires.front().face_id == face.face_id &&
                    restored.wires.front().edge_ids == wire.edge_ids,
                "model cache I/O should preserve Unicode paths and content") && ok;

    {
        std::fstream legacyCache(modelPath,
            std::ios::binary | std::ios::in | std::ios::out);
        legacyCache.seekp(8); // Version follows the eight-byte magic.
        const char oldVersion[4] = {1, 0, 0, 0};
        legacyCache.write(oldVersion, sizeof(oldVersion));
        ok = expect(static_cast<bool>(legacyCache),
                    "the legacy-cache fixture should be writable") && ok;
    }
    ok = expect(!oap::readModel(oap::pathUtf8(modelPath), restored) &&
                    restored.units.size() == 1u &&
                    restored.units.front().decoded_name == unit.decoded_name,
                "legacy analysis caches must be rejected without replacing the model") && ok;

    const std::filesystem::path stepPath =
        directory / oap::utf8Path("毛坯模型_繁體_𠮷_🙂.stp");
    const std::string stepPathUtf8 = oap::pathUtf8(stepPath);
    STEPControl_Writer writer;
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 12.0, 14.0).Shape();
    const IFSelect_ReturnStatus transferred =
        writer.Transfer(box, STEPControl_AsIs);
    const IFSelect_ReturnStatus written = transferred == IFSelect_RetDone
        ? writer.Write(stepPathUtf8.c_str()) : transferred;
    const oap::toolpath::StockModelLoadResult loaded =
        written == IFSelect_RetDone
        ? oap::toolpath::StockModelLoader().loadStep(stepPathUtf8)
        : oap::toolpath::StockModelLoadResult{};
    ok = expect(transferred == IFSelect_RetDone &&
                    written == IFSelect_RetDone && loaded.ok(),
                "OCCT should write and read a full-Unicode UTF-8 path") && ok;

    Handle(TDocStd_Document) document;
    oap::StepIndexer indexer;
    oap::Model analyzedModel;
    const oap::AnalysisPipelineReport analysis =
        oap::AnalysisPipeline().execute(
            stepPathUtf8, document, indexer, analyzedModel);
    ok = expect(analysis.ok() && !analyzedModel.units.empty(),
                "the STEP loading pipeline should load a Unicode STEP path") && ok;

    if (!document.IsNull())
        XCAFApp_Application::GetApplication()->Close(document);

    std::filesystem::remove_all(directory, filesystemError);
    return ok;
}

} // namespace

int main()
{
    OSD::SetSignal(Standard_False);
    if (!programModelAndValidationAreStable() ||
        !largeStructuralValidationIsDeterministic() ||
        !primitiveMoveMappingPreservesNcSemantics() ||
        !exportValidatesAndBuffersBeforeOverwrite() ||
        !unicodePathIoIsIndependentOfTheActiveCodePage()) {
        return 1;
    }
    std::cout << "toolpath program smoke tests passed\n";
    return 0;
}
