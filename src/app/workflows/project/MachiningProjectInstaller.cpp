// MachiningProjectInstaller 实现（oap_app_workflow_cam）。
// 将已验证/已重新生成的项目原子地安装到桌面会话中，见 MachiningProjectInstaller.h。
#include "app/workflows/project/MachiningProjectInstaller.h"

#include <algorithm>
#include <utility>

#include "tooling/adapter/hole/HoleStageIdentity.h"

namespace oap::app {
namespace {

// 与 MachiningProjectController 相同规则的槽腔实例键,
// 用于给预览条目生成稳定的面集合标识。
std::string pocketInstanceKey(std::vector<std::string> faceIds)
{
    std::sort(faceIds.begin(), faceIds.end());
    std::string key;
    for (const std::string& faceId : faceIds) {
        if (!key.empty())
            key += '|';
        key += faceId;
    }
    return key;
}

} // namespace

MachiningProjectInstallResult MachiningProjectInstaller::install(
    MachiningProjectLoadResult loaded,
    MachiningWorkspaceSession& workspace,
    MachiningConfigurationSession& configuration,
    ToolLibraryController& toolLibraries,
    const toolpath::MoldSetupWorkFrames& currentSetupFrames)
{
    MachiningProjectInstallResult result;
    result.hasHoleDefinitions = loaded.hasHoleDefinitions;
    result.hasPocketDefinitions = loaded.hasPocketDefinitions;
    result.hasProfileDefinitions = loaded.hasProfileDefinitions;
    result.hasFinishingDefinitions = loaded.hasFinishingDefinitions;
    result.finishingOperationCount = static_cast<int>(loaded.finishing.operations.size());
    result.holeOperationCount = static_cast<int>(
        loaded.holes.machining.operations.size());
    result.pocketOperationCount = static_cast<int>(
        loaded.pockets.operations.size());
    result.profileOperationCount = static_cast<int>(
        loaded.profiles.operations.size());
    // 先迁移 Job 所有权并同步预览参数、程序选项与刀具库,
    // 再以 Job 中的确认状态覆盖工位坐标系(先默认全部未确认)。
    workspace.job() = std::move(loaded.job);
    if (!workspace.job().definition().finishingOperations.empty()) {
        auto& settings = configuration.engravingSettings();
        const auto& operations = workspace.job().definition().finishingOperations;
        const auto first = std::min_element(operations.begin(), operations.end(),
            [](const auto& a, const auto& b) { return a.sequence < b.sequence; });
        settings.parameters = first->parameters;
        settings.projectFaces.clear();
        for (const auto& operation : operations) {
            if (const auto* setup = workspace.job().setupById(operation.setupId))
                settings.projectFaces.push_back(setup->face);
        }
        std::sort(settings.projectFaces.begin(), settings.projectFaces.end());
        settings.projectFaces.erase(std::unique(settings.projectFaces.begin(), settings.projectFaces.end()), settings.projectFaces.end());
        settings.scope = settings.projectFaces.size() == 6 ? EngravingSetupScope::AllSixFaces :
            EngravingSetupScope::PreserveProjectFaces;
        if (settings.projectFaces.size() == 1) {
            switch (settings.projectFaces.front()) {
            case MoldSetupFace::PositiveZ: settings.scope = EngravingSetupScope::PositiveZ; break;
            case MoldSetupFace::NegativeX: settings.scope = EngravingSetupScope::NegativeX; break;
            case MoldSetupFace::PositiveX: settings.scope = EngravingSetupScope::PositiveX; break;
            case MoldSetupFace::NegativeY: settings.scope = EngravingSetupScope::NegativeY; break;
            case MoldSetupFace::PositiveY: settings.scope = EngravingSetupScope::PositiveY; break;
            case MoldSetupFace::NegativeZ: settings.scope = EngravingSetupScope::NegativeZ; break;
            default: break;
            }
        }
        if (const auto* tool = workspace.job().toolControllerById(first->toolControllerId)) {
            settings.libraryId = tool->libraryId;
            settings.manualTool = tool->tool;
            settings.toolMode = tool->libraryId.empty() ? EngravingToolMode::SingleTool : EngravingToolMode::AutomaticLibrary;
        }
    }
    // A direct tool is a frozen controller snapshot, independent of the active catalog.
    const auto restoreSingle = [&](const auto& operations, auto setMode, auto& snapshot) {
        if (operations.empty()) return;
        const manufacturing::JobToolController* selected = nullptr;
        for (const auto& operation : operations) {
            const auto* tool = workspace.job().toolControllerById(operation.toolControllerId);
            if (!tool || !tool->libraryId.empty() || !tool->tool.valid ||
                (selected && toolpath::toolpathToolContextKey(selected->tool) != toolpath::toolpathToolContextKey(tool->tool))) {
                setMode(ToolSelectionMode::AutomaticLibrary);
                return;
            }
            selected = tool;
        }
        snapshot = selected->tool;
        setMode(ToolSelectionMode::SingleTool);
    };
    const auto& definition = workspace.job().definition();
    if (!definition.holeOperations.empty()) configuration.holeParams() = definition.holeParameters;
    if (!definition.pocketOperations.empty()) configuration.pocketParams() = definition.pocketParameters;
    if (!definition.profileOperations.empty()) configuration.profileParams() = definition.profileParameters;
    restoreSingle(definition.holeOperations, [&](auto mode) { configuration.setHoleToolSelectionMode(mode); }, configuration.holeSingleTool());
    restoreSingle(definition.pocketOperations, [&](auto mode) { configuration.setPocketToolSelectionMode(mode); }, configuration.pocketSingleTool());
    restoreSingle(definition.profileOperations, [&](auto mode) { configuration.setProfileToolSelectionMode(mode); }, configuration.profileSingleTool());
    result.stockDefinition = workspace.job().definition().stock;
    result.setupWorkFrames = currentSetupFrames;
    workspace.holeToolpathNeedsRegeneration() = false;
    workspace.setToolpathNeedsRegeneration(ActiveToolpathPreview::Pocket, false);
    workspace.setToolpathNeedsRegeneration(ActiveToolpathPreview::Profile, false);
    workspace.setToolpathNeedsRegeneration(ActiveToolpathPreview::SurfaceFinishing, false);
    if (result.hasHoleDefinitions) {
        workspace.holePreviewParams() = loaded.holes.effectiveParams;
        workspace.hasHolePreviewParams() = true;
    }
    if (result.hasPocketDefinitions) {
        workspace.pocketPreviewParams() =
            workspace.job().definition().pocketParameters;
        workspace.pocketPreviewEffectiveParams() =
            loaded.pockets.effectiveParams;
        workspace.hasPocketPreviewParams() = true;
    }
    if (result.hasProfileDefinitions) {
        workspace.profilePreviewParams() =
            workspace.job().definition().profileParameters;
        workspace.profilePreviewEffectiveParams() =
            loaded.profiles.effectiveParams;
        workspace.hasProfilePreviewParams() = true;
    }
    configuration.gcodeOptions() =
        workspace.job().definition().post.options;
    toolLibraries.restoreActiveLibrary(QString::fromStdString(
        workspace.job().definition().activeToolLibraryId));
    for (int faceIndex = 0; faceIndex < 6; ++faceIndex) {
        result.setupWorkFrames.setConfirmed(
            static_cast<MoldSetupFace>(faceIndex), false);
    }
    for (const manufacturing::SetupDefinition& setup :
         workspace.job().definition().setups) {
        result.setupWorkFrames.setConfirmed(
            setup.face, setup.confirmation.confirmed);
    }
    result.setupCount = static_cast<int>(
        workspace.job().definition().setups.size());

    const std::vector<toolpath::HoleMachiningInput>& recognizedHoles =
        workspace.job().definition().recognizedHoles;

    // 重建"已识别孔源"表:Job 未保存识别孔时,按工序目标里出现的
    // 最大 sourceHoleIndex 推断表大小,保证索引引用始终有效。
    std::size_t sourceCount = recognizedHoles.size();
    if (sourceCount == 0) {
        int maximumSourceIndex = -1;
        for (const manufacturing::HoleOperationDefinition& operation :
             workspace.job().definition().holeOperations) {
            for (const manufacturing::HoleTargetReference& target :
                 operation.targets) {
                maximumSourceIndex = std::max(
                    maximumSourceIndex, target.sourceHoleIndex);
            }
        }
        sourceCount = static_cast<std::size_t>(maximumSourceIndex + 1);
    }
    // 先用识别孔定义填充每个源;对只有工序目标而没有识别孔记录的
    // 槽位,回退用目标孔信息补全,保证 UI 的孔列表有完整数据。
    workspace.recognizedHoleSources().assign(sourceCount, {});
    for (std::size_t index = 0; index < recognizedHoles.size(); ++index) {
        RecognizedHoleSource& source =
            workspace.recognizedHoleSources()[index];
        source.hole = recognizedHoles[index];
        source.sourceFaceIds = recognizedHoles[index].sourceFaceIds;
        source.key = tooling::holeStageKey(source.hole);
    }
    for (const manufacturing::HoleOperationDefinition& operation :
         workspace.job().definition().holeOperations) {
        for (const manufacturing::HoleTargetReference& target :
             operation.targets) {
            if (target.sourceHoleIndex < 0 ||
                target.sourceHoleIndex >= static_cast<int>(
                    workspace.recognizedHoleSources().size())) {
                continue;
            }
            RecognizedHoleSource& source =
                workspace.recognizedHoleSources()[
                    static_cast<std::size_t>(target.sourceHoleIndex)];
            if (source.sourceFaceIds.empty()) {
                source.hole = target.hole;
                source.sourceFaceIds = target.hole.sourceFaceIds;
                source.key = tooling::holeStageKey(source.hole);
            }
        }
    }

    // 重建孔预览条目并回填其源 key,供预览面板按源分组展示。
    workspace.holePreviewEntries().clear();
    workspace.holePreviewEntries().reserve(
        loaded.holes.machining.operations.size());
    for (const toolpath::HoleMachiningOperation& operation :
         loaded.holes.machining.operations) {
        HolePreviewEntry entry;
        entry.sourceIndex = operation.sourceHoleIndex;
        entry.operation = operation;
        if (entry.sourceIndex >= 0 && entry.sourceIndex < static_cast<int>(
                workspace.recognizedHoleSources().size())) {
            entry.key = workspace.recognizedHoleSources()[
                static_cast<std::size_t>(entry.sourceIndex)].key;
        }
        workspace.holePreviewEntries().push_back(std::move(entry));
    }

    // 重建预览；仅选择有运行期刀路的领域，禁用工序的定义不抢占预览。
    workspace.pocketPreviewEntries().clear();
    workspace.pocketPreviewEntries().reserve(loaded.pockets.operations.size());
    for (const toolpath::PocketToolpathOperation& operation :
         loaded.pockets.operations) {
        PocketPreviewEntry entry;
        entry.key = pocketInstanceKey(operation.sourceFaceIds);
        entry.operation = operation;
        workspace.pocketPreviewEntries().push_back(std::move(entry));
    }
    workspace.pocketPreviewActive() = result.hasPocketDefinitions;
    workspace.profilePreviewOperations() =
        std::move(loaded.profiles.operations);
    workspace.activePreview() = workspace.job().currentFinishingResult()
            ? ActiveToolpathPreview::SurfaceFinishing
            : result.profileOperationCount > 0
            ? ActiveToolpathPreview::Profile
            : result.pocketOperationCount > 0
                ? ActiveToolpathPreview::Pocket
                : result.holeOperationCount > 0
                    ? ActiveToolpathPreview::Hole : ActiveToolpathPreview::None;
    return result;
}

} // namespace oap::app
