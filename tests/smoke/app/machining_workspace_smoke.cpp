#include "app/machining/MachiningWorkspaceCoordinator.h"
#include "manufacturing/job/MachiningJobStorage.h"

#include <BRepPrimAPI_MakeBox.hxx>

#include <iostream>
#include <string_view>

namespace {

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool explicitFamilyPreviewIsTransactional()
{
    namespace a = oap::app;
    namespace m = oap::manufacturing;
    namespace t = oap::toolpath;
    m::MachiningJob job;
    auto& definition = job.definition();
    m::HoleOperationDefinition hole;
    hole.id = "hole";
    hole.state = m::JobOperationState::NeedsReview;
    definition.holeOperations.push_back(hole);
    m::PocketOperationDefinition pocket;
    pocket.id = "pocket";
    pocket.state = m::JobOperationState::Generated;
    definition.pocketOperations.push_back(pocket);
    m::ProfileOperationDefinition profile;
    profile.id = "profile";
    profile.state = m::JobOperationState::Warning;
    definition.profileOperations.push_back(profile);
    t::ToolpathMove move;
    move.type = t::ToolpathMoveType::Cut;
    move.from = gp_Pnt(0, 0, 1);
    move.to = gp_Pnt(1, 0, 1);
    move.feed = 200;
    t::HoleToolpathBatch holes;
    t::HoleMachiningOperation holeRuntime;
    holeRuntime.operationId = hole.id;
    holeRuntime.moves.push_back(move);
    holes.machining.operations.push_back(holeRuntime);
    t::PocketToolpathBatch pockets;
    t::PocketToolpathOperation pocketRuntime;
    pocketRuntime.operationId = pocket.id;
    pocketRuntime.clearing.moves.push_back(move);
    pockets.operations.push_back(pocketRuntime);
    t::ProfileToolpathBatch profiles;
    t::ProfileToolpathOperation profileRuntime;
    profileRuntime.operationId = profile.id;
    profileRuntime.contour.moves.push_back(move);
    profiles.operations.push_back(profileRuntime);
    job.setHoleResult(holes, m::MachiningJobStorage::holeExecutionFingerprint(definition).toStdString());
    job.setPocketResult(pockets, m::MachiningJobStorage::pocketExecutionFingerprint(definition).toStdString());
    job.setProfileResult(profiles, m::MachiningJobStorage::profileExecutionFingerprint(definition).toStdString());
    a::MachiningWorkspaceCoordinator workspace;
    workspace.installProfileProgram(job, {}, {});
    t::ToolpathProgram program;
    QString error;
    if (!expect(workspace.currentProfileResult() && workspace.activateOperationPreview("pocket", program, &error) &&
            workspace.activePreview() == a::ActiveToolpathPreview::Pocket && program.operations.front().operationId == "pocket" &&
            workspace.activateOperationPreview("hole", program, &error) && workspace.activePreview() == a::ActiveToolpathPreview::Hole &&
            workspace.holePreviewEntries().size() == 1 && program.operations.front().operationId == "hole" &&
            workspace.activateOperationPreview("profile", program, &error) && workspace.activePreview() == a::ActiveToolpathPreview::Profile &&
            program.operations.front().operationId == "profile",
            "explicit preview must switch current families without regeneration or confirmation/audit requirements")) return false;
    if (!expect(!workspace.activateOperationPreview("missing", program, &error) && !error.isEmpty() &&
            workspace.activePreview() == a::ActiveToolpathPreview::Profile && program.operations.front().operationId == "profile",
            "missing formal operation changed the active preview or output")) return false;
    workspace.markHoleRegenerationRequired();
    if (!expect(!workspace.activateOperationPreview("hole", program, &error) &&
            workspace.activePreview() == a::ActiveToolpathPreview::Profile && workspace.currentProfileResult(),
            "pending hole edits must block only hole preview activation")) return false;
    auto stale = job;
    stale.definition().profileParameters.depth.stepDown += 1;
    workspace.installProfileProgram(stale, {}, {});
    workspace.setActivePreview(a::ActiveToolpathPreview::Pocket);
    if (!expect(!workspace.currentProfileResult() && !workspace.activateOperationPreview("profile", program, &error) &&
            workspace.activePreview() == a::ActiveToolpathPreview::Pocket,
            "profile fingerprint mismatch bypassed the preview freshness gate")) return false;
    auto disabled = job;
    disabled.definition().profileOperations.front().enabled = false;
    workspace.installProfileProgram(disabled, {}, {});
    workspace.setActivePreview(a::ActiveToolpathPreview::Pocket);
    if (!expect(!workspace.activateOperationPreview("profile", program, &error) &&
            workspace.activePreview() == a::ActiveToolpathPreview::Pocket,
            "disabled operation changed the active preview")) return false;
    auto missingRuntime = job;
    missingRuntime.clearProfileResult();
    workspace.installProfileProgram(missingRuntime, {}, {});
    workspace.setActivePreview(a::ActiveToolpathPreview::Pocket);
    return expect(!workspace.activateOperationPreview("profile", program, &error) &&
        workspace.activePreview() == a::ActiveToolpathPreview::Pocket,
        "missing runtime changed the active preview");
}

} // namespace

int main()
{
    if (!explicitFamilyPreviewIsTransactional()) return 1;
    oap::app::MachiningWorkspaceCoordinator workspace;
    oap::toolpath::ToolpathParams holeParams;
    holeParams.tool.diameter = 8.0;
    oap::manufacturing::MachiningJob holeJob;
    oap::toolpath::HoleToolpathBatch holeBatch;
    holeBatch.coverage.audited = true;
    holeBatch.coverage.inputHoleSegmentCount = 1;
    holeBatch.coverage.sixFaceHoleSegmentCount = 1;
    holeBatch.coverage.coveredHoleSegmentCount = 1;
    holeJob.setHoleResult(std::move(holeBatch), "hole-runtime");
    holeJob.definition().holeOperations.push_back({});
    oap::app::RecognizedHoleSource holeSource;
    holeSource.key = "hole-source-1";
    workspace.installHoleProgram(
        std::move(holeJob),
        {holeSource}, holeParams);
    const oap::app::MachiningWorkspaceStatus holeStatus = workspace.status();
    if (!expect(
            workspace.activePreview() ==
                oap::app::ActiveToolpathPreview::Hole &&
                workspace.hasHolePreviewParams() &&
                workspace.holePreviewParams().tool.diameter == 8.0 &&
                holeStatus.hasHoleProgram && holeStatus.hasProjectContent &&
                holeStatus.coverageAudited && holeStatus.coveragePassed &&
                workspace.holeProgram() != nullptr &&
                workspace.holeProgramSnapshot().has_value() &&
                workspace.recognizedHoleSourceCount() == 1 &&
                workspace.recognizedHoleSource(0) != nullptr &&
                workspace.recognizedHoleSource(1) == nullptr,
            "hole install must publish all preview state atomically")) {
        return 1;
    }

    workspace.markHoleRegenerationRequired();
    QString pendingError;
    if (!expect(workspace.hasPendingRegeneration() && workspace.activePreviewNeedsRegeneration() &&
                !workspace.hasHoleProgram() && !workspace.hasHolePreviewParams() &&
                workspace.holePreviewEntries().empty() && !workspace.prepareToolpathExport().ok &&
                !workspace.synchronizeProjectForSave({}, &pendingError) && !pendingError.isEmpty() &&
                !workspace.saveProject(QStringLiteral("must-not-write-pending.oapjob"), &pendingError),
                "changed hole parameters must block stale export/save and clear every runtime preview"))
        return 1;

    oap::app::PocketPreviewEntry entry;
    entry.key = "P1";
    oap::toolpath::ToolpathParams pocketParams;
    pocketParams.tool.diameter = 12.0;
    oap::manufacturing::MachiningJob pocketJob;
    pocketJob.definition().pocketOperations.push_back({});
    pocketJob.definition().pocketOperations.front().id = "pocket-formal-1";
    oap::toolpath::PocketToolpathBatch pocketBatch;
    pocketBatch.operations = {entry.operation};
    pocketBatch.effectiveParams = pocketParams;
    pocketJob.setPocketResult(pocketBatch,
        oap::manufacturing::MachiningJobStorage::pocketExecutionFingerprint(
            pocketJob.definition()).toStdString());
    workspace.installPocketProgram(
        std::move(pocketJob), {entry},
        pocketParams, pocketParams);
    const oap::app::PocketPreviewSnapshot snapshot =
        workspace.capturePocketPreview();
    workspace.clearPocketPreviewState();
    if (!expect(!workspace.pocketPreviewActive() &&
                    workspace.pocketPreviewEntries().empty(),
                "pocket clear must reset all pocket preview state")) {
        return 1;
    }
    workspace.restorePocketPreview(snapshot);
    if (!expect(
            workspace.pocketPreviewActive() &&
                workspace.pocketPreviewEntries().size() == 1 &&
                workspace.pocketPreviewProgram().operations.size() == 1 &&
                workspace.activePreview() ==
                    oap::app::ActiveToolpathPreview::Pocket,
            "pocket snapshot restore must be atomic")) {
        return 1;
    }

    workspace.retainDisplayablePocketPreviewEntries(
        [](oap::app::PocketPreviewEntry&) { return false; });
    if (!expect(workspace.pocketPreviewEntries().empty() &&
                    workspace.pocketPreviewProgram().operations.size() == 1,
                "display filtering must not trim the canonical playback program"))
        return 1;

    workspace.restorePocketPreview(snapshot);
    workspace.markPocketRegenerationRequired();
    const auto stalePocketOperations = workspace.operationDescriptors();
    if (!expect(!workspace.currentPocketResult() && !workspace.hasPocketPreviewParams() &&
                !workspace.pocketPreviewActive() && workspace.pocketPreviewEntries().empty() &&
                workspace.pocketPreviewProgram().operations.empty() &&
                !workspace.prepareToolpathExport().ok &&
                workspace.activePreview() == oap::app::ActiveToolpathPreview::Pocket &&
                stalePocketOperations.size() == 1 &&
                stalePocketOperations.front().state == oap::manufacturing::JobOperationState::Stale,
                "pocket parameter/tool changes must invalidate runtime, playback sources and export while preserving stale definitions"))
        return 1;

    oap::toolpath::ToolpathParams requestedProfileParams =
        workspace.profileParams();
    oap::toolpath::ToolpathParams effectiveProfileParams =
        requestedProfileParams;
    effectiveProfileParams.tool.diameter =
        requestedProfileParams.tool.diameter + 6.0;
    effectiveProfileParams.tool.horizFeed = 850.0;
    oap::toolpath::ProfileToolpathOperation profileOperation;
    profileOperation.operationId = "PROFILE_AUTO_1";
    workspace.installProfilePreview(
        {profileOperation}, requestedProfileParams,
        effectiveProfileParams);
    const oap::app::PocketPreviewSnapshot profileSnapshot =
        workspace.capturePocketPreview();
    if (!expect(
            workspace.profilePreviewParams().tool.diameter ==
                    requestedProfileParams.tool.diameter &&
                workspace.profilePreviewEffectiveParams().tool.diameter ==
                    effectiveProfileParams.tool.diameter &&
                oap::app::MachiningConfigurationSession::parametersEqual(
                    workspace.profilePreviewParams(),
                    requestedProfileParams) &&
                workspace.profilePreviewMatchesCurrentParameters(),
            "profile preview must keep requested and automatic effective parameters separate")) {
        return 1;
    }
    workspace.installProfilePreview(
        {}, effectiveProfileParams, effectiveProfileParams);
    workspace.restorePocketPreview(profileSnapshot);
    if (!expect(
            workspace.profilePreviewOperations().size() == 1 &&
                workspace.profilePreviewParams().tool.diameter ==
                    requestedProfileParams.tool.diameter &&
                workspace.profilePreviewEffectiveParams().tool.diameter ==
                    effectiveProfileParams.tool.diameter &&
                workspace.profilePreviewMatchesCurrentParameters(),
            "profile requested/effective parameters must survive preview snapshot restore")) {
        return 1;
    }

    oap::manufacturing::MachiningJob profileJob;
    oap::toolpath::ProfileToolpathBatch profileBatch;
    profileBatch.effectiveParams = effectiveProfileParams;
    profileBatch.operations.push_back(profileOperation);
    profileJob.setProfileResult(std::move(profileBatch), "profile-runtime");
    profileJob.definition().profileOperations.push_back({});
    workspace.installProfileProgram(
        std::move(profileJob), requestedProfileParams,
        effectiveProfileParams);
    const oap::app::MachiningWorkspaceStatus status = workspace.status();
    if (!expect(
            status.hasProfileProgram && status.hasProjectContent &&
                status.profileOperationCount == 1 &&
                workspace.profilePreviewOperations().size() == 1 &&
                workspace.profilePreviewProgram().operations.size() == 1 &&
                workspace.activePreview() ==
                    oap::app::ActiveToolpathPreview::Profile &&
                workspace.profilePreviewOperations().front().operationId ==
                    "PROFILE_AUTO_1",
            "profile install must publish Job and preview state atomically")) {
        return 1;
    }

    workspace.markProfileRegenerationRequired();
    const auto staleProfileOperations = workspace.operationDescriptors();
    if (!expect(!workspace.status().hasProfileProgram && !workspace.hasProfilePreviewParams() &&
                !workspace.hasProfilePreview() && workspace.profilePreviewProgram().operations.empty() &&
                !workspace.prepareToolpathExport().ok &&
                workspace.activePreview() == oap::app::ActiveToolpathPreview::Profile &&
                staleProfileOperations.size() == 1 &&
                staleProfileOperations.front().state == oap::manufacturing::JobOperationState::Stale,
                "profile parameter/tool changes must clear all runtime and preview copies and block export"))
        return 1;

    oap::manufacturing::MachiningJob replacementHoleJob;
    replacementHoleJob.definition().profileOperations.push_back({});
    replacementHoleJob.definition().profileOperations.front().state = oap::manufacturing::JobOperationState::Stale;
    replacementHoleJob.definition().holeOperations.push_back({});
    replacementHoleJob.setHoleResult(oap::toolpath::HoleToolpathBatch{}, "replacement-hole");
    workspace.installHoleProgram(std::move(replacementHoleJob), {}, holeParams);
    if (!expect(workspace.hasPendingRegeneration() && !workspace.activePreviewNeedsRegeneration() &&
                !workspace.saveProject(QStringLiteral("must-not-write-cross-family.oapjob"), &pendingError),
                "successful hole generation must not clear pending profile edits"))
        return 1;

    oap::app::MachiningWorkspaceSession disabledPending;
    disabledPending.job().definition().profileOperations.push_back({});
    disabledPending.job().definition().profileOperations.front().enabled = false;
    disabledPending.setToolpathNeedsRegeneration(oap::app::ActiveToolpathPreview::Profile, true);
    if (!expect(disabledPending.hasPendingRegeneration() && !disabledPending.hasPendingRegeneration(true),
                "disabled pending operations must still block saving but not unrelated enabled simulation"))
        return 1;
    disabledPending.job().definition().profileOperations.front().enabled = true;
    if (!expect(disabledPending.hasPendingRegeneration(true),
                "reenabling an operation must not lose its pending regeneration state"))
        return 1;

    workspace.resetForModel();
    if (!expect(
            workspace.activePreview() ==
                oap::app::ActiveToolpathPreview::None &&
                workspace.pocketPreviewEntries().empty() &&
                workspace.recognizedHoleSources().empty() && !workspace.hasPendingRegeneration(),
            "model reset must clear the complete machining session")) {
        return 1;
    }
    std::cout << "PASS: machining workspace facade checks\n";
    return 0;
}
