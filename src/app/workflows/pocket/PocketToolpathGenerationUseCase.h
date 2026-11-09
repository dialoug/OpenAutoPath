// 不依赖 MainWindow 状态的完整桌面槽/腔刀路候选生成。
#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include <QString>
#include <QStringList>

#include "app/workflows/pocket/PocketToolpathWorkflowController.h"
#include "app/workflows/shared/WorkflowInteraction.h"
#include "app/workflows/shared/ToolpathGenerationValidation.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "tooling/library/ToolLibrary.h"
#include "tooling/adapter/pocket/PocketAutomaticToolpath.h"

namespace oap::app {

// 生成失败分类：None 表示成功，其余对应每个提前返回的失败关口。
enum class PocketToolpathGenerationFailure {
    None,
    InvalidRequest,
    Cancelled,
    EmptyProgram,
    ValidationFailed,
    PreviewFailed,
    CommitFailed,
};

// 生成请求：指针字段由调用方持有并保证在 execute 期间存活；
// previewCutMoveBudget 限制预览阶段保留的切削移动段数量。
struct PocketToolpathGenerationRequest {
    WorkflowInteraction interaction;
    int candidateCount = 0;
    const std::vector<toolpath::PocketRegion>* regions = nullptr;
    const std::vector<toolpath::SkippedRegion>* buildSkipped = nullptr;
    bool regionCacheHit = false;
    qint64 regionBuildMilliseconds = 0;
    const toolpath::ToolpathParams* params = nullptr;
    const tooling::ToolLibrary* automaticLibrary = nullptr;
    const toolpath::ToolpathToolContext* singleTool = nullptr;
    const MoldCoordinateFrame* moldFrame = nullptr;
    const manufacturing::MachiningJob* currentJob = nullptr;
    MachiningProjectDefinitionContext definition;
    const std::vector<toolpath::RootInnerWirePocketSource>* rootSources =
        nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
    int previewCutMoveBudget = 50000;
};

// 生成与预览的统计指标：区分完整移动段（full*）与
// 抽稀后显示移动段（*MoveCount），供日志与状态栏摘要使用。
struct PocketToolpathGenerationMetrics {
    qint64 batchMilliseconds = 0;
    qint64 previewMilliseconds = 0;
    int generatedRegions = 0;
    int skippedRegions = 0;
    int sectionLayerTotal = 0;
    int protectedMaterialLayerTotal = 0;
    int protectedMaterialRegionCount = 0;
    int firstLayerPreflightRejectedCount = 0;
    int openSectionLayerTotal = 0;
    int virtualOpenBoundaryTotal = 0;
    int slotLoopCount = 0;
    int pocketLoopCount = 0;
    int depthLayerTotal = 0;
    int maxDepthLayerCount = 0;
    int fullCutMoveCount = 0;
    int fullRapidMoveCount = 0;
    int fullEntryRetractMoveCount = 0;
    int verticalEntryTotal = 0;
    int rampOnShapeEntryTotal = 0;
    int linearRampEntryTotal = 0;
    int zigzagRampEntryTotal = 0;
    int helicalRampEntryTotal = 0;
    int entryFallbackTotal = 0;
    int parallelLayerRegionCount = 0;
    int parallelPatternRegionCount = 0;
    std::size_t patternParallelBatchCount = 0u;
    std::size_t patternParallelTaskCount = 0u;
    std::size_t patternPeakParallelism = 1u;
    int cutMoveCount = 0;
    int rapidMoveCount = 0;
    int entryRetractMoveCount = 0;
    int slotCutMoveCount = 0;
    int pocketCutMoveCount = 0;
    int skippedPreviewCutMoves = 0;
    int skippedDisplaySegments = 0;
    int cutPreviewStride = 1;
    bool previewLimited = false;
    double algorithmMilliseconds = 0.0;
    double sectionMilliseconds = 0.0;
    double patternMilliseconds = 0.0;
    double moveAssemblyMilliseconds = 0.0;
    double movePreparationMilliseconds = 0.0;
    double moveLinkingMilliseconds = 0.0;
    std::size_t collinearMergedMoveCount = 0;
    std::size_t restTravelReorderedLoopCount = 0;
    std::size_t restTravelAdjustedStartCount = 0;
    std::size_t travelChoiceParallelBatchCount = 0u;
    std::size_t travelChoiceParallelTaskCount = 0u;
    std::size_t travelChoicePeakParallelism = 1u;
    std::size_t sectionWireParallelBatchCount = 0u;
    std::size_t sectionWireParallelTaskCount = 0u;
    std::size_t sectionWirePeakParallelism = 1u;
    std::size_t occtInternalParallelCount = 0u;
    std::size_t occtSerialFallbackCount = 0u;
    QString failureSummary;
    QString skipSummary;
    QStringList generationWarnings;
    QStringList generationDetails;

    int totalLoops() const noexcept
    {
        return slotLoopCount + pocketLoopCount;
    }
    int fullTotalMoves() const noexcept
    {
        return fullCutMoveCount + fullRapidMoveCount +
               fullEntryRetractMoveCount;
    }
    int displayTotalMoves() const noexcept
    {
        return cutMoveCount + rapidMoveCount + entryRetractMoveCount;
    }
};

// 预览发布回调：对每个预览条目做展示端定制（如生成显示几何），
// 返回 false 表示该条目不可显示并计入跳过统计。
using PocketPreviewPublisher =
    std::function<bool(PocketPreviewEntry&)>;

// 生成结果：ok() 为 true 时 candidateJob 已包含新的 Job。
struct PocketToolpathGenerationResult {
    PocketToolpathGenerationFailure failure =
        PocketToolpathGenerationFailure::InvalidRequest;
    QString error;
    toolpath::PocketToolpathBatch batch;
    toolpath::ToolpathProgram program;
    ToolpathGenerationValidation validation;
    tooling::PocketAutomaticToolpathResult automaticSelection;
    std::vector<PocketPreviewEntry> previewEntries;
    PocketCandidateJobResult candidateJob;
    PocketToolpathGenerationMetrics metrics;
    QString summary;
    QStringList reportLines;
    bool automatic = false;

    bool ok() const noexcept
    {
        return failure == PocketToolpathGenerationFailure::None &&
               candidateJob.ok;
    }
};

// 槽/腔刀路生成完整用例（静态门面）：不依赖 MainWindow 状态，
// 串联区域生成、自动选刀、装夹上下文分配、验证、
// 预览预算与原子提交。
class PocketToolpathGenerationUseCase {
public:
    // 执行完整生成流程；失败时 result.failure 给出分类与原因。
    static PocketToolpathGenerationResult execute(
        const PocketToolpathGenerationRequest& request,
        const PocketPreviewPublisher& previewPublisher = {});

    // 保持公开，以便在不构造 OCCT 展示对象的情况下测试
    // 预览预算控制与发布器失败。
    // 保留条目中的完整工序，按预算统计抽稀后的切削段。
    // 任一发布器拒绝则返回 false 并清空 entries，禁止提交部分工序。
    // 发布器已产生的显示副作用须由调用方恢复，清空 entries 不会撤销显示。
    static bool publishPreview(
        const std::vector<toolpath::PocketToolpathOperation>& operations,
        const toolpath::ToolpathParams& params,
        int cutMoveBudget,
        const PocketPreviewPublisher& publisher,
        std::vector<PocketPreviewEntry>& entries,
        PocketToolpathGenerationMetrics& metrics);
};

} // namespace oap::app
