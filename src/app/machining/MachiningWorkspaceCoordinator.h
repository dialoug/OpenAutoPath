// 每模型加工状态、参数与刀具库的应用门面。
// 职责:作为加工工作区的门面,聚合加工会话(孔/槽程序与预览)、加工参数
//   配置与刀具库三块长期状态,并向 UI 暴露事务式安装操作与稳定查询。
// 模块归属:src/app(machining 聚焦工作区协调器),由 MainWindow 持有。
// 协作:多数修改通过 MachiningWorkspaceSession 及其工作流协调器
//   (HoleToolpathWorkbenchCoordinator)落库;跨工作区安装由
//   MachiningProjectInstaller 完成并同步到 stock/recognition/document
//   协调器;生成与导出委托给用例层和 ToolpathExportController。
#pragma once

#include <functional>
#include <optional>
#include <span>
#include <utility>

#include "app/workflows/project/ToolLibraryController.h"
#include "app/workflows/hole/HoleToolpathGenerationUseCase.h"
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "app/workflows/project/MachiningConfigurationSession.h"
#include "app/workflows/project/MachiningProjectController.h"
#include "app/workflows/project/MachiningProjectInstaller.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "app/workflows/pocket/PocketToolpathGenerationUseCase.h"
#include "app/workflows/profile/ProfileToolpathGenerationUseCase.h"
#include "app/workflows/engraving/EngravingToolpathGenerationUseCase.h"
#include "app/workflows/simulation/StockSimulationUseCases.h"
#include "app/workflows/shared/ToolpathExportController.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "manufacturing/finishing/SurfaceFinishingMachiningJobService.h"

namespace oap {
class ModelIndex;
}

namespace oap::app {

class RecognitionWorkspaceCoordinator;
class ModelDocumentWorkspaceCoordinator;
class StockSimulationWorkspaceCoordinator;

// 槽预览快照:捕获/恢复槽预览的完整状态,供临时改动前后备份还原。
struct PocketPreviewSnapshot {
    ActiveToolpathPreview activePreview = ActiveToolpathPreview::None;
    std::vector<PocketPreviewEntry> entries;
    toolpath::ToolpathParams params;
    toolpath::ToolpathParams effectiveParams;
    bool previewActive = false;
    bool hasParams = false;
    std::vector<toolpath::ProfileToolpathOperation> profileOperations;
    toolpath::ToolpathParams profileParams;
    toolpath::ToolpathParams profileEffectiveParams;
    bool hasProfileParams = false;
};

// Stable desktop projection that keeps MachiningJob storage details private.
struct MachiningWorkspaceStatus {
    bool hasHoleProgram = false;
    bool hasPocketProgram = false;
    bool hasProfileProgram = false;
    bool hasFinishingProgram = false;
    bool hasProjectContent = false;
    int holeOperationCount = 0;
    int pocketOperationCount = 0;
    int profileOperationCount = 0;
    int finishingOperationCount = 0;
    bool coverageAudited = false;
    bool coveragePassed = false;
    bool coverageHasWarnings = false;
};

// 加工工作区协调器:长期持有加工会话、参数配置与刀具库,为 UI 提供
struct SurfaceFinishingEditState {
    toolpath::SurfaceFinishingParams parameters;
    toolpath::ToolpathToolContext tool;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    std::vector<std::string> unitIds;
    std::string libraryId;
};

// 加工工作区协调器:长期持有加工会话、参数配置与刀具库,为 UI 提供
//   统一入口。修改多字段状态只允许走下列事务方法,查询走只读 getter。
class MachiningWorkspaceCoordinator final {
public:
    // 模型切换/重置:清空当前模型的加工会话状态(孔/槽程序、预览)。
    void resetForModel();
    // 原子安装孔加工程序:整体替换加工任务/孔源,并从任务
    // 重建孔预览条目、激活孔预览、写入生效参数并复位重生成标记。
    void installHoleProgram(
        manufacturing::MachiningJob job,
        std::vector<RecognizedHoleSource> sources,
        const toolpath::ToolpathParams& effectiveParams);
    // 若已安装的孔任务含有效刀路结果且无需重生成,则激活其孔预览并
    // 返回 true;否则不动预览并返回 false。
    bool activateInstalledHoleProgram();
    // Activates the selected formal operation's complete family only when its
    // current runtime is available. Failure preserves the previous preview.
    bool activateOperationPreview(const std::string& formalOperationId,
        toolpath::ToolpathProgram& program, QString* error = nullptr);
    // 标记孔程序需要重新生成(参数/刀具变更后),使后续激活失效。
    void markHoleRegenerationRequired();
    // 作废孔任务的生产安全校验结果,提醒用户在变更后重新校验。
    void invalidateHoleProductionSafety();
    // 参数或刀具变化后清除运行结果与预览，正式工序保留为待重新生成。
    void markPocketRegenerationRequired();
    void markProfileRegenerationRequired();
    void markEngravingRegenerationRequired();
    bool hasPendingRegeneration() const noexcept { return m_session.hasPendingRegeneration(); }
    bool activePreviewNeedsRegeneration() const noexcept
    {
        return m_session.toolpathNeedsRegeneration(m_session.activePreview());
    }

    // 捕获/恢复槽预览快照:用于临时改动前备份、之后还原完整预览状态。
    PocketPreviewSnapshot capturePocketPreview() const;
    void restorePocketPreview(PocketPreviewSnapshot snapshot);
    // 原子安装槽加工程序:替换加工任务与预览条目,激活槽预览并记录
    // 请求参数与生效参数。
    void installPocketProgram(
        manufacturing::MachiningJob job,
        std::vector<PocketPreviewEntry> entries,
        const toolpath::ToolpathParams& requestedParams,
        const toolpath::ToolpathParams& effectiveParams);
    // 原子安装外轮廓程序：从已提交 Job 的运行期结果重建预览，确保
    // 预览工序 ID、Job 定义与运行期批次来自同一次提交。
    void installProfileProgram(
        manufacturing::MachiningJob job,
        const toolpath::ToolpathParams& requestedParams,
        const toolpath::ToolpathParams& effectiveParams);
    // 清除槽预览条目与参数;若当前激活的是槽预览则回退到 None。
    void clearPocketPreviewState();
    // 就地过滤槽预览条目,仅保留 display 谓词接受(返回 true)的项;
    // 返回保留数量。
    int retainDisplayablePocketPreviewEntries(
        const std::function<bool(PocketPreviewEntry&)>& display);

    // 以当前会话任务为上下文生成孔/槽腔/外轮廓刀路。
    HoleToolpathGenerationResult generateHoleToolpath(
        HoleToolpathGenerationRequest request) const;
    PocketToolpathGenerationResult generatePocketToolpath(
        PocketToolpathGenerationRequest request,
        const PocketPreviewPublisher& previewPublisher = {}) const;
    ProfileToolpathGenerationResult generateProfileToolpath(
        ProfileToolpathGenerationRequest request,
        const ProfilePreviewPublisher& previewPublisher = {}) const;
    // 依据工程定义上下文补齐当前任务的项目定义,失败时通过 error 返回原因。
    bool ensureProjectDefinition(
        const MachiningProjectDefinitionContext& context,
        QString* error = nullptr);
    bool installSurfaceFinishing(
        const MachiningProjectDefinitionContext& definition,
        const manufacturing::SurfaceFinishingJobCaptureRequest& request,
        QString* error = nullptr);
    bool hasFinishingPreview() const;
    const EngravingSettings& engravingSettings() const;
    EngravingToolpathGenerationResult generateEngraving(EngravingToolpathGenerationRequest request);
    toolpath::ToolpathProgram finishingPreviewProgram() const;
    toolpath::ToolpathParams finishingPreviewParams() const;
    std::optional<SurfaceFinishingEditState> finishingOperation(const std::string& id) const;
    // 把当前槽预览几何捕获进项目定义,供保存工程时引用。
    bool capturePocketProjectOperations(
        const MachiningProjectPocketCaptureRequest& request,
        QString* error = nullptr);
    bool captureProfileProjectPreview(
        const MachiningProjectProfileCaptureRequest& request,
        QString* error = nullptr);
    std::vector<manufacturing::JobOperationDescriptor>
    operationDescriptors() const;
    MachiningProjectLoadResult regenerateProject(
        const MachiningProjectLoadRequest& request) const;
    // 保存工程前把项目定义同步回当前任务。
    bool synchronizeProjectForSave(
        const MachiningProjectSaveRequest& request,
        QString* error = nullptr);
    // 持久化当前项目定义，不向 UI 暴露 MachiningJob 的存储结构。
    bool saveProject(const QString& filePath, QString* error = nullptr) const;
    MachiningWorkspaceStatus status() const noexcept;
    bool hasHoleProgram() const noexcept;
    bool hasProjectContent() const noexcept;
    const toolpath::HoleToolpathBatch* holeProgram() const noexcept;
    std::optional<toolpath::HoleToolpathBatch>
    holeProgramSnapshot() const;
    const toolpath::HoleCoverageReport& holeCoverageReport() const noexcept;
    std::vector<std::string> validationErrors() const;
    // 准备刀路导出内容(基于当前会话)。
    ToolpathExportPreparation prepareToolpathExport() const;

    // 安装已加载的加工工程:安装器原子替换会话/配置/刀具库,随后把
    // 毛坯定义、识别孔与装夹工位坐标系分别同步到 stock、recognition、
    // document 三个协调器,保证各工作区状态一致。
    MachiningProjectInstallResult installProject(
        MachiningProjectLoadResult loaded,
        StockSimulationWorkspaceCoordinator& stockWorkspace,
        RecognitionWorkspaceCoordinator& recognitionWorkspace,
        ModelDocumentWorkspaceCoordinator& documentWorkspace);
    // 以当前任务为上下文在 stock 工作区执行孔/槽毛坯仿真。
    HoleStockRunOutcome executeHoleStockSimulation(
        StockSimulationWorkspaceCoordinator& stockWorkspace,
        HoleStockSimulationUseCaseRequest request);
    PocketStockSimulationUseCaseResult executePocketStockSimulation(
        StockSimulationWorkspaceCoordinator& stockWorkspace,
        PocketStockSimulationUseCaseRequest request);
    ProfileStockSimulationUseCaseResult executeProfileStockSimulation(
        StockSimulationWorkspaceCoordinator& stockWorkspace,
        ProfileStockSimulationUseCaseRequest request);
    ProjectStockSimulationUseCaseResult executeProjectStockSimulation(
        StockSimulationWorkspaceCoordinator& stockWorkspace,
        ProjectStockSimulationUseCaseRequest request);
    void refreshPocketSimulationDisplay(
        StockSimulationWorkspaceCoordinator& stockWorkspace,
        const gp_Trsf& modelTransform) const;
    std::vector<IpwTimelineOperation> pocketSimulationTimeline(
        const StockSimulationWorkspaceCoordinator& stockWorkspace) const;
    void displayPocketSimulationTransient(
        StockSimulationWorkspaceCoordinator& stockWorkspace,
        int operationIndex,
        int completedMoveCount,
        const gp_Trsf& modelTransform) const;
    void finishPocketSimulationTimeline(
        StockSimulationWorkspaceCoordinator& stockWorkspace,
        const gp_Trsf& modelTransform) const;

    // 加载加工参数配置默认值,返回错误信息(空串表示成功)。
    QString loadConfigurationDefaults();
    // 生成参数编辑器的初始状态(含工位坐标系与模仁尺寸约束)。
    ToolpathParameterEditorState configurationEditorState(
        const toolpath::MoldSetupWorkFrames& workFrames,
        double moldLength,
        double moldWidth) const;
    // 应用编辑后的加工参数,并把推导出的装夹工位坐标系安装到
    // document 协调器。
    MachiningConfigurationApplyResult applyConfiguration(
        ToolpathParameterEditorState edited,
        ModelDocumentWorkspaceCoordinator& documentWorkspace);
    // 配置查询:当前生效的孔/槽腔/外轮廓参数、槽刀具选择模式、加工设置与 G 代码选项。
    const toolpath::ToolpathParams& holeParams() const noexcept
    {
        return m_configuration.holeParams();
    }
    const toolpath::ToolpathParams& pocketParams() const noexcept
    {
        return m_configuration.pocketParams();
    }
    const toolpath::ToolpathParams& profileParams() const noexcept
    {
        return m_configuration.profileParams();
    }
    const toolpath::ToolpathToolContext& holeSingleTool() const noexcept { return m_configuration.holeSingleTool(); }
    const toolpath::ToolpathToolContext& pocketSingleTool() const noexcept { return m_configuration.pocketSingleTool(); }
    const toolpath::ToolpathToolContext& profileSingleTool() const noexcept { return m_configuration.profileSingleTool(); }
    HoleToolSelectionMode holeToolSelectionMode() const noexcept
    {
        return m_configuration.holeToolSelectionMode();
    }
    PocketToolSelectionMode pocketToolSelectionMode() const noexcept
    {
        return m_configuration.pocketToolSelectionMode();
    }
    ProfileToolSelectionMode profileToolSelectionMode() const noexcept
    {
        return m_configuration.profileToolSelectionMode();
    }
    const toolpath::MachiningSetup& machiningSetup() const noexcept
    {
        return m_configuration.machiningSetup();
    }
    const toolpath::GCodeProgramOptions& gcodeOptions() const noexcept
    {
        return m_configuration.gcodeOptions();
    }

    // 加载(或重新加载)全部刀具库,返回错误信息(空串表示成功)。
    QStringList loadToolLibraries();
    // 应用新的刀具库集合与激活库 id(原子替换)。
    ToolLibraryApplyResult applyToolLibraries(
        tooling::ToolAssetCatalog catalog,
        QString activeLibraryId);
    ToolLibraryApplyResult applyToolLibraries(
        QVector<tooling::ToolLibrary> libraries,
        QString activeLibraryId);
    // 按 id 恢复激活的刀具库,找不到时返回 false。
    bool restoreActiveToolLibrary(const QString& activeLibraryId);
    // 刀具库只读查询:库集合、当前激活库 id 与指针。
    const QVector<tooling::ToolLibrary>& toolLibraries() const noexcept
    {
        return m_toolLibraries.libraries();
    }
    const tooling::ToolAssetCatalog& toolCatalog() const noexcept
    {
        return m_toolLibraries.catalog();
    }
    const QString& activeToolLibraryId() const noexcept
    {
        return m_toolLibraries.activeLibraryId();
    }
    const tooling::ToolLibrary* activeToolLibrary() const noexcept
    {
        return m_toolLibraries.activeLibrary();
    }

    // 稳定的查询与工作流适配器。修改多字段的安装操作刻意只通过
    // 上面的事务方法暴露。
    const toolpath::PocketToolpathBatch* currentPocketResult() const
    {
        return m_session.job().currentPocketResult();
    }
    const toolpath::ProfileToolpathBatch* currentProfileResult(QString* error = nullptr) const;
    std::span<const PocketPreviewEntry> pocketPreviewEntries() const noexcept
    {
        return m_session.pocketPreviewEntries();
    }
    bool hasPocketPreview() const noexcept
    {
        return !m_session.pocketPreviewEntries().empty();
    }
    toolpath::ToolpathProgram pocketPreviewProgram() const;
    const toolpath::ToolpathParams& pocketPreviewParams() const noexcept
    {
        return m_session.pocketPreviewParams();
    }
    const toolpath::ToolpathParams& pocketPreviewEffectiveParams() const noexcept
    {
        return m_session.pocketPreviewEffectiveParams();
    }
    bool hasPocketPreviewParams() const noexcept
    {
        return m_session.hasPocketPreviewParams();
    }
    bool pocketPreviewActive() const noexcept
    {
        return m_session.pocketPreviewActive();
    }
    ActiveToolpathPreview activePreview() const noexcept
    {
        return m_session.activePreview();
    }
    void setActivePreview(ActiveToolpathPreview preview) noexcept
    {
        m_session.activePreview() = preview;
    }
    void installProfilePreview(
        std::vector<toolpath::ProfileToolpathOperation> operations,
        const toolpath::ToolpathParams& requestedParams,
        const toolpath::ToolpathParams& effectiveParams)
    {
        m_session.profilePreviewOperations() = std::move(operations);
        m_session.profilePreviewParams() = requestedParams;
        m_session.profilePreviewEffectiveParams() = effectiveParams;
        m_session.hasProfilePreviewParams() = true;
        m_session.activePreview() = ActiveToolpathPreview::Profile;
    }
    std::span<const toolpath::ProfileToolpathOperation>
    profilePreviewOperations() const noexcept
    {
        return m_session.profilePreviewOperations();
    }
    bool hasProfilePreview() const noexcept
    {
        return !m_session.profilePreviewOperations().empty();
    }
    toolpath::ToolpathProgram profilePreviewProgram() const;
    const toolpath::ToolpathParams& profilePreviewParams() const noexcept
    {
        return m_session.profilePreviewParams();
    }
    const toolpath::ToolpathParams&
    profilePreviewEffectiveParams() const noexcept
    {
        return m_session.profilePreviewEffectiveParams();
    }
    bool hasProfilePreviewParams() const noexcept
    {
        return m_session.hasProfilePreviewParams();
    }
    bool profilePreviewMatchesCurrentParameters() const noexcept
    {
        return m_session.hasProfilePreviewParams() &&
            MachiningConfigurationSession::parametersEqual(
                m_session.profilePreviewParams(),
                m_configuration.profileParams());
    }
    std::span<const RecognizedHoleSource> recognizedHoleSources() const noexcept
    {
        return m_session.recognizedHoleSources();
    }
    bool hasRecognizedHoleSources() const noexcept
    {
        return !m_session.recognizedHoleSources().empty();
    }
    std::size_t recognizedHoleSourceCount() const noexcept
    {
        return m_session.recognizedHoleSources().size();
    }
    const RecognizedHoleSource* recognizedHoleSource(int index) const noexcept;
    std::span<const HolePreviewEntry> holePreviewEntries() const noexcept
    {
        return m_session.holePreviewEntries();
    }
    const std::unordered_set<int>& highlightedHoleStageSources() const noexcept
    {
        return m_session.highlightedHoleStageSources();
    }
    // 切换孔源在孔加工阶段的高亮状态,返回切换后是否处于高亮。
    bool toggleHighlightedHoleStage(int sourceIndex);
    // 移除/清空孔源高亮集合。
    void removeHighlightedHoleStage(int sourceIndex);
    void clearHighlightedHoleStages();
    // 由识别孔源推导孔加工输入列表。
    std::vector<toolpath::HoleMachiningInput> sourceHoleInputs() const;
    // 根据选中面 id 推断当前选中的孔阶段下标,无匹配返回 -1。
    int selectedHoleStageIndex(
        const std::vector<std::string>& selectedFaceIds) const;
    // 依据播放移动与刀具批次起点构建刀具树模型。
    HoleToolTreeModel holeToolTreeModel(
        const std::vector<PlaybackMove>& moves,
        const std::vector<int>& toolBatchStarts) const;
    QVector<HoleCoverageOperationBinding> holeCoverageOperationBindings(
        const std::vector<PlaybackMove>& moves) const;
    QString formalOperationIdForRuntime(
        const QString& runtimeOperationId) const;
    // 孔预览参数/是否已有/是否需要重生成等查询,以及批量同步与阶段设置。
    const toolpath::ToolpathParams& holePreviewParams() const noexcept
    {
        return m_session.holePreviewParams();
    }
    bool hasHolePreviewParams() const noexcept
    {
        return m_session.hasHolePreviewParams();
    }
    bool holeToolpathNeedsRegeneration() const noexcept
    {
        return m_session.holeToolpathNeedsRegeneration();
    }
    void synchronizeHolePreviewBatch()
    {
        m_session.synchronizeHolePreviewBatch();
    }

private:
    ToolLibraryApplyResult applyToolLibraryChange(const std::function<ToolLibraryApplyResult()>& apply);
    void invalidateOperationPreviews();
    MachiningWorkspaceSession m_session;
    MachiningConfigurationSession m_configuration;
    ToolLibraryController m_toolLibraries;
};

} // namespace oap::app
