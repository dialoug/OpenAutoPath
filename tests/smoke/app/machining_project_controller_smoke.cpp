#include "app/workflows/project/MachiningConfigurationSession.h"
#include "app/workflows/project/MachiningProjectController.h"
#include "app/workflows/simulation/StockSimulationSession.h"
#include "app/workflows/project/ToolpathParameterSettings.h"
#include "app/workflows/project/DirectToolSettingsStorage.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/MachiningOperationService.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <QFile>
#include <QSettings>
#include <QTemporaryDir>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <tuple>

namespace {

static_assert(static_cast<int>(oap::app::ToolSelectionMode::SingleTool) == 2,
              "new standalone-tool mode must not reuse the retired value zero");

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAILED: " << message << '\n';
    return false;
}

bool writes(const QString& path, const QByteArray& data)
{
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate) &&
           file.write(data) == data.size();
}

bool toolSelectionModesRoundTrip()
{
    QTemporaryDir temporary;
    if (!expect(temporary.isValid(),
                "parameter settings directory should exist")) {
        return false;
    }
    QSettings settings(
        temporary.filePath(QStringLiteral("toolpath.ini")),
        QSettings::IniFormat);
    oap::toolpath::ToolpathParams hole;
    oap::toolpath::ToolpathParams pocket;
    oap::toolpath::ToolpathParams profile;
    pocket.clearing.pattern = oap::toolpath::ClearingPattern::FreePath;
    pocket.clearing.expansionDirection =
        oap::toolpath::PocketClearingDirection::OutsideToInside;
    pocket.clearing.adaptive.targetEngagement = 0.18;
    pocket.clearing.adaptive.targetEngagementAngleDeg = 64.0;
    pocket.clearing.adaptive.maximumEngagementAngleDeg = 132.0;
    pocket.clearing.adaptive.engagementAngleSamples = 108;
    pocket.clearing.adaptive.minimumTurnRadiusFactor = 0.8;
    pocket.clearing.adaptive.remainingTolerance = 0.008;
    pocket.clearing.adaptive.materialCellSize = 0.35;
    pocket.clearing.adaptive.maximumAdaptivePasses = 51;
    pocket.clearing.adaptive.lookaheadDepth = 4;
    pocket.clearing.adaptive.beamWidth = 9;
    profile.tool.diameter = 12.0;
    profile.depth.stepDown = 1.75;
    profile.profile.sideAllowance = 0.35;
    profile.profile.machiningMode =
        oap::toolpath::ProfileMachiningMode::ExternalClearing;
    profile.profile.roughingAllowance = 0.6;
    profile.profile.finishPass = false;
    profile.profile.radialPassCount = 3;
    profile.profile.radialStepOver = 1.25;
    profile.profile.leadInLength = 2.0;
    profile.profile.leadOutLength = 1.5;
    bool coolant = false;
    QString error;
    if (!expect(
            oap::app::ToolpathParameterSettingsStorage::save(
                settings, hole, pocket, profile,
                oap::app::HoleToolSelectionMode::AutomaticLibrary,
                oap::app::PocketToolSelectionMode::AutomaticLibrary,
                oap::app::ProfileToolSelectionMode::AutomaticLibrary,
                coolant, &error),
            "automatic selection configuration should persist")) {
        std::cerr << error.toStdString() << '\n';
        return false;
    }

    oap::toolpath::ToolpathParams loadedHole;
    oap::toolpath::ToolpathParams loadedPocket;
    oap::toolpath::ToolpathParams loadedProfile;
    oap::app::HoleToolSelectionMode loadedHoleMode =
        oap::app::HoleToolSelectionMode::AutomaticLibrary;
    oap::app::PocketToolSelectionMode loadedMode =
        oap::app::PocketToolSelectionMode::AutomaticLibrary;
    oap::app::ProfileToolSelectionMode loadedProfileMode =
        oap::app::ProfileToolSelectionMode::AutomaticLibrary;
    if (!expect(oap::app::ToolpathParameterSettingsStorage::save(
                    settings, hole, pocket, profile,
                    oap::app::ToolSelectionMode::SingleTool, oap::app::ToolSelectionMode::SingleTool,
                    oap::app::ToolSelectionMode::SingleTool, coolant, &error) &&
                oap::app::ToolpathParameterSettingsStorage::load(
                    settings, loadedHole, loadedPocket, loadedProfile,
                    loadedHoleMode, loadedMode, loadedProfileMode, coolant, &error) &&
                loadedHoleMode == oap::app::ToolSelectionMode::SingleTool &&
                loadedMode == oap::app::ToolSelectionMode::SingleTool &&
                loadedProfileMode == oap::app::ToolSelectionMode::SingleTool,
                "all three single-tool selection modes must round-trip as value two"))
        return false;
    // Retired value zero still migrates to automatic selection, never to the
    // new explicit single-tool mode even when the caller previously loaded it.
    settings.setValue("toolpath/current_parameters/hole_parameters/tool_selection_mode", 0);
    settings.setValue("toolpath/current_parameters/pocket_parameters/tool_selection_mode", 0);
    settings.setValue("toolpath/current_parameters/profile_parameters/tool_selection_mode", 0);
    if (!expect(
            oap::app::ToolpathParameterSettingsStorage::load(
                settings, loadedHole, loadedPocket, loadedProfile,
                loadedHoleMode, loadedMode, loadedProfileMode, coolant,
                &error),
            "legacy local selection modes must migrate to automatic")) {
        std::cerr << error.toStdString() << '\n';
        return false;
    }
    return expect(
        loadedHoleMode ==
                oap::app::HoleToolSelectionMode::AutomaticLibrary &&
            loadedMode ==
                oap::app::PocketToolSelectionMode::AutomaticLibrary &&
            loadedPocket.clearing.pattern ==
                oap::toolpath::ClearingPattern::FreePath &&
            loadedPocket.clearing.expansionDirection ==
                oap::toolpath::PocketClearingDirection::InsideToOutside &&
            loadedPocket.clearing.adaptive.targetEngagement == 0.18 &&
            loadedPocket.clearing.adaptive.targetEngagementAngleDeg == 64.0 &&
            loadedPocket.clearing.adaptive.maximumEngagementAngleDeg == 132.0 &&
            loadedPocket.clearing.adaptive.engagementAngleSamples == 108 &&
            loadedPocket.clearing.adaptive.minimumTurnRadiusFactor == 0.8 &&
            loadedPocket.clearing.adaptive.remainingTolerance == 0.008 &&
            loadedPocket.clearing.adaptive.materialCellSize == 0.35 &&
            loadedPocket.clearing.adaptive.maximumAdaptivePasses == 51 &&
            loadedPocket.clearing.adaptive.lookaheadDepth == 4 &&
            loadedPocket.clearing.adaptive.beamWidth == 9 &&
            loadedProfileMode ==
                oap::app::ProfileToolSelectionMode::AutomaticLibrary &&
            loadedProfile.tool.diameter == 12.0 &&
            loadedProfile.depth.stepDown == 1.75 &&
            loadedProfile.profile.sideAllowance == 0.35 &&
            loadedProfile.profile.machiningMode ==
                oap::toolpath::ProfileMachiningMode::ExternalClearing &&
            loadedProfile.profile.roughingAllowance == 0.6 &&
            !loadedProfile.profile.finishPass &&
            loadedProfile.profile.radialPassCount == 3 &&
            loadedProfile.profile.radialStepOver == 1.25 &&
            loadedProfile.profile.leadInLength == 2.0 &&
            loadedProfile.profile.leadOutLength == 1.5,
        "restored independent tool selection modes should round-trip");
}

oap::toolpath::ToolpathToolContext directTool(int number)
{
    oap::toolpath::ToolpathToolContext tool;
    tool.valid = true;
    tool.toolNumber = number;
    tool.toolId = "direct-" + std::to_string(number);
    tool.toolName = "Standalone cutter " + std::to_string(number);
    tool.toolType = number == 1 ? "drill" : "end_mill";
    tool.diameter = 6.25 + number;
    tool.cornerRadius = 0.5;
    tool.spindleSpeed = 7200 + number;
    tool.horizontalFeed = 650 + number;
    tool.verticalFeed = 120 + number;
    tool.horizontalRapid = 3600 + number;
    tool.verticalRapid = 2400 + number;
    tool.cuttingEdgeLength = 22.25;
    tool.overallLength = 60.5;
    tool.shankDiameter = 12.25;
    tool.holderDiameter = 32.5;
    tool.holderLength = 45.75;
    tool.spindleEnvelopeDiameter = 80.25;
    tool.spindleEnvelopeLength = 90.5;
    tool.tipAngle = 118;
    tool.tipCompensation = 0.125;
    tool.peckDepth = 1.75;
    tool.drillCycle = oap::toolpath::ToolpathDrillCyclePreference::ChipBreak;
    tool.coolant = oap::toolpath::ToolpathCoolantMode::ThroughTool;
    return tool;
}

bool sameDirectTool(const oap::toolpath::ToolpathToolContext& a,
                    const oap::toolpath::ToolpathToolContext& b)
{
    const auto fields = [](const auto& t) {
        return std::tie(t.valid, t.toolNumber, t.toolId, t.toolName, t.toolType,
            t.diameter, t.cornerRadius, t.spindleSpeed, t.horizontalFeed, t.verticalFeed,
            t.horizontalRapid, t.verticalRapid, t.cuttingEdgeLength, t.overallLength,
            t.shankDiameter, t.holderDiameter, t.holderLength, t.spindleEnvelopeDiameter,
            t.spindleEnvelopeLength, t.tipAngle, t.tipCompensation, t.peckDepth, t.drillCycle, t.coolant);
    };
    return fields(a) == fields(b);
}

bool directToolSettingsAreCompleteAndAtomic()
{
    QTemporaryDir directory;
    if (!expect(directory.isValid(), "standalone tool settings directory unavailable")) return false;
    QSettings settings(directory.filePath("direct.ini"), QSettings::IniFormat);
    const auto hole = directTool(1), pocket = directTool(2), profile = directTool(3);
    oap::toolpath::ToolpathToolContext h, p, r;
    QString error;
    if (!expect(oap::app::DirectToolSettingsStorage::save(settings, hole, pocket, profile, &error) &&
                oap::app::DirectToolSettingsStorage::load(settings, h, p, r, &error) &&
                sameDirectTool(hole, h) && sameDirectTool(pocket, p) && sameDirectTool(profile, r),
                "all standalone tool snapshot fields must survive settings roundtrip"))
        return false;
    const QByteArray valid = settings.value("toolpath/direct_tools").toByteArray();
    // Corrupt each family, including the last decoded one. None of the three
    // caller snapshots may be overwritten by earlier successfully decoded data.
    for (const char* family : {"hole", "pocket", "profile"}) {
        auto object = QJsonDocument::fromJson(valid).object();
        auto damaged = object.value(QLatin1String(family)).toObject();
        damaged.insert("diameter", "invalid");
        object.insert(QLatin1String(family), damaged);
        settings.setValue("toolpath/direct_tools", QJsonDocument(object).toJson());
        h = directTool(11); p = directTool(12); r = directTool(13);
        if (!expect(!oap::app::DirectToolSettingsStorage::load(settings, h, p, r, &error) &&
                    !error.isEmpty() && sameDirectTool(h, directTool(11)) &&
                    sameDirectTool(p, directTool(12)) && sameDirectTool(r, directTool(13)),
                    "damaged family settings must atomically preserve all caller snapshots"))
            return false;
    }
    settings.setValue("toolpath/direct_tools", "{broken-json");
    return expect(!oap::app::DirectToolSettingsStorage::load(settings, h, p, r, &error) &&
                  sameDirectTool(h, directTool(11)) && sameDirectTool(p, directTool(12)) &&
                  sameDirectTool(r, directTool(13)), "malformed JSON must not alter standalone tools");
}

bool directToolChangesInvalidateOnlyTheirFamily()
{
    oap::app::MachiningConfigurationSession session;
    const oap::toolpath::MoldSetupWorkFrames frames;
    auto initial = session.editorState(frames, 40, 30);
    initial.holeSingleTool = directTool(1);
    initial.pocketSingleTool = directTool(2);
    initial.profileSingleTool = directTool(3);
    initial.holeToolSelectionMode = initial.pocketToolSelectionMode = initial.profileToolSelectionMode =
        oap::app::ToolSelectionMode::SingleTool;
    const auto applied = session.apply(initial);
    if (!expect(applied.persistenceError.isEmpty() && applied.holeToolSelectionModeChanged &&
                applied.pocketToolSelectionModeChanged && applied.profileToolSelectionModeChanged,
                "standalone configuration must persist all three explicit modes"))
        return false;
    for (int family = 0; family < 3; ++family) {
        auto edited = session.editorState(frames, 40, 30);
        if (family == 0) edited.holeSingleTool.tipAngle += 2;
        if (family == 1) edited.pocketSingleTool.holderDiameter += 1;
        if (family == 2) edited.profileSingleTool.toolNumber += 10;
        const auto changed = session.apply(edited);
        if (!expect(changed.persistenceError.isEmpty() &&
                    changed.holeParametersChanged == (family == 0) &&
                    changed.pocketParametersChanged == (family == 1) &&
                    changed.profileParametersChanged == (family == 2) &&
                    !changed.engravingParametersChanged && !changed.holeToolSelectionModeChanged &&
                    !changed.pocketToolSelectionModeChanged && !changed.profileToolSelectionModeChanged,
                    "changing a direct tool must invalidate only its owning family"))
            return false;
    }
    const auto unchanged = session.apply(session.editorState(frames, 40, 30));
    if (!expect(unchanged.persistenceError.isEmpty() && !unchanged.holeParametersChanged &&
                !unchanged.pocketParametersChanged && !unchanged.profileParametersChanged &&
                !unchanged.engravingParametersChanged,
                "unchanged standalone snapshots must not invalidate generated toolpaths"))
        return false;
    oap::app::MachiningConfigurationSession restored;
    return expect(restored.loadDefaults().isEmpty() &&
                  restored.holeToolSelectionMode() == oap::app::ToolSelectionMode::SingleTool &&
                  restored.pocketToolSelectionMode() == oap::app::ToolSelectionMode::SingleTool &&
                  restored.profileToolSelectionMode() == oap::app::ToolSelectionMode::SingleTool &&
                  sameDirectTool(restored.holeSingleTool(), session.holeSingleTool()) &&
                  sameDirectTool(restored.pocketSingleTool(), session.pocketSingleTool()) &&
                  sameDirectTool(restored.profileSingleTool(), session.profileSingleTool()),
                  "isolated default settings must restore family modes and complete snapshots");
}

bool migratesCombinedPocketProfileSettings()
{
    QTemporaryDir temporary;
    if (!expect(temporary.isValid(),
                "legacy parameter settings directory should exist")) {
        return false;
    }
    QSettings settings(
        temporary.filePath(QStringLiteral("legacy-toolpath.ini")),
        QSettings::IniFormat);
    settings.beginGroup(QStringLiteral("toolpath/current_parameters"));
    settings.setValue(QStringLiteral("schema_version"), 2);
    settings.beginGroup(QStringLiteral("pocket_parameters"));
    settings.setValue(QStringLiteral("tool/diameter"), 9.5);
    settings.setValue(QStringLiteral("depth/step_down"), 2.25);
    settings.setValue(QStringLiteral("profile/side_allowance"), 0.45);
    settings.endGroup();
    settings.endGroup();
    settings.sync();

    oap::toolpath::ToolpathParams hole;
    oap::toolpath::ToolpathParams pocket;
    oap::toolpath::ToolpathParams profile;
    oap::app::HoleToolSelectionMode holeMode =
        oap::app::HoleToolSelectionMode::AutomaticLibrary;
    oap::app::PocketToolSelectionMode mode =
        oap::app::PocketToolSelectionMode::AutomaticLibrary;
    oap::app::ProfileToolSelectionMode profileMode =
        oap::app::ProfileToolSelectionMode::AutomaticLibrary;
    bool coolant = false;
    QString error;
    if (!expect(
            oap::app::ToolpathParameterSettingsStorage::load(
                settings, hole, pocket, profile, holeMode, mode, profileMode,
                coolant, &error),
            "schema 2 combined pocket/profile settings should migrate")) {
        std::cerr << error.toStdString() << '\n';
        return false;
    }
    return expect(
        pocket.tool.diameter == 9.5 && profile.tool.diameter == 9.5 &&
            pocket.depth.stepDown == 2.25 &&
            profile.depth.stepDown == 2.25 &&
            profile.profile.sideAllowance == 0.45 &&
            profileMode ==
                oap::app::ProfileToolSelectionMode::AutomaticLibrary,
        "schema 2 profile values should seed the independent profile page");
}

bool ownsConfigurationAndProjectPolicies()
{
    QTemporaryDir temporary;
    if (!expect(temporary.isValid(), "temporary directory should exist"))
        return false;
    const QString sourcePath = temporary.filePath("part.step");
    if (!expect(writes(sourcePath, "part-v1"),
                "temporary source should be writable")) {
        return false;
    }

    oap::app::StockSimulationSession stock;
    stock.holeMarginX() = 1.0;
    stock.holeMarginY() = 2.0;
    stock.holeMarginZ() = 3.0;
    stock.holeSourceMode() = 2;
    stock.holeStockStepPath() = temporary.filePath("stock.step");
    const oap::manufacturing::StockDefinition stockDefinition =
        stock.holeStockDefinition();
    oap::app::StockSimulationSession restoredStock;
    restoredStock.restoreHoleStockDefinition(stockDefinition);
    if (!expect(
            restoredStock.holeMarginX() == 1.0 &&
                restoredStock.holeMarginY() == 2.0 &&
                restoredStock.holeMarginZ() == 3.0 &&
                restoredStock.holeSourceMode() == 2 &&
                restoredStock.holeStockStepPath() ==
                    stock.holeStockStepPath(),
            "stock project definition should round-trip through its session")) {
        return false;
    }

    oap::app::MachiningConfigurationSession configuration;
    oap::toolpath::ToolpathParams changed = configuration.holeParams();
    changed.tool.diameter += 1.0;
    if (!expect(
            !oap::app::MachiningConfigurationSession::parametersEqual(
                configuration.holeParams(), changed),
            "configuration session should detect a machining change")) {
        return false;
    }

    oap::manufacturing::MachiningJob job;
    oap::app::MachiningProjectDefinitionContext context;
    context.sourcePath = sourcePath;
    context.activeToolLibraryId = QStringLiteral("library-test");
    context.gcodeOptions.coolant = true;
    context.stock = stockDefinition;
    QString error;
    if (!expect(
            oap::app::MachiningProjectController::ensureDefinition(
                job, context, &error),
            "project controller should initialize a definition")) {
        std::cerr << error.toStdString() << '\n';
        return false;
    }
    if (!expect(
            !job.definition().part.sourceFingerprint.empty() &&
                job.definition().activeToolLibraryId == "library-test" &&
                job.definition().post.options.coolant,
            "initialized definition should capture source, library, and post options")) {
        return false;
    }

    const QString projectPath = temporary.filePath("part.oapjob");
    if (!expect(
            oap::manufacturing::MachiningJobStorage::save(
                projectPath, job.definition(), &error),
            "project definition should be saveable")) {
        return false;
    }
    oap::app::MachiningProjectLoadRequest missingPart;
    missingPart.filePath = projectPath;
    const auto missingPartResult =
        oap::app::MachiningProjectController::loadAndRegenerate(missingPart);
    if (!expect(
            missingPartResult.failure ==
                oap::app::MachiningProjectLoadFailure::PartRequired,
            "load should report the missing current part as a typed failure")) {
        return false;
    }

    if (!expect(writes(sourcePath, "part-v2"),
                "temporary source should be replaceable")) {
        return false;
    }
    oap::Model model;
    model.faces.emplace_back();
    oap::StepIndexer indexer;
    oap::toolpath::MoldSetupWorkFrames frames;
    oap::app::MachiningProjectLoadRequest mismatch;
    mismatch.filePath = projectPath;
    mismatch.currentSourcePath = sourcePath;
    mismatch.model = &model;
    mismatch.indexer = &indexer;
    mismatch.setupFrames = &frames;
    const auto mismatchResult =
        oap::app::MachiningProjectController::loadAndRegenerate(mismatch);
    return expect(
        mismatchResult.failure ==
            oap::app::MachiningProjectLoadFailure::PartFingerprint,
        "load should reject a different current part before regeneration");
}

} // namespace

int main()
{
    QTemporaryDir isolatedPreferences;
    if (!expect(isolatedPreferences.isValid(), "isolated preference directory unavailable")) return 1;
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, isolatedPreferences.path());
    QSettings::setPath(QSettings::IniFormat, QSettings::SystemScope, isolatedPreferences.filePath("system"));
    if (!toolSelectionModesRoundTrip() ||
        !directToolSettingsAreCompleteAndAtomic() ||
        !directToolChangesInvalidateOnlyTheirFamily() ||
        !migratesCombinedPocketProfileSettings() ||
        !ownsConfigurationAndProjectPolicies())
        return 1;
    std::cout << "machining project controller smoke tests passed\n";
    return 0;
}
