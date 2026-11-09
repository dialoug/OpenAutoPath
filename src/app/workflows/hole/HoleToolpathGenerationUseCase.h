// 不依赖 MainWindow 状态的完整桌面孔刀路候选生成。
#pragma once

#include <vector>

#include <QString>
#include <QStringList>

#include "app/workflows/hole/HoleToolpathWorkflowController.h"
#include "app/workflows/shared/WorkflowInteraction.h"
#include "app/workflows/shared/ToolpathGenerationValidation.h"
#include "tooling/library/ToolLibrary.h"

namespace oap::app {

// 生成失败分类：None 表示成功，其余对应每个提前返回的失败关口。
enum class HoleToolpathGenerationFailure {
    None,
    InvalidRequest,
    Cancelled,
    EmptyProgram,
    ValidationFailed,
    CommitFailed,
};

// 生成请求：指针字段均由调用方持有并保证在 execute 期间存活。
struct HoleToolpathGenerationRequest {
    WorkflowInteraction interaction;
    const std::vector<toolpath::HoleMachiningInput>* recognized = nullptr;
    const Model* model = nullptr;
    const StepIndexer* indexer = nullptr;
    Bbox moldBounds;
    const MoldCoordinateFrame* moldFrame = nullptr;
    const toolpath::ToolpathParams* params = nullptr;
    const tooling::ToolLibrary* automaticLibrary = nullptr;
    const toolpath::ToolpathToolContext* singleTool = nullptr;
    const manufacturing::MachiningJob* currentJob = nullptr;
    MachiningProjectDefinitionContext definition;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
};

// 生成结果：ok() 为 true 时 committed 已包含新的 Job 与工作流会话。
struct HoleToolpathGenerationResult {
    HoleToolpathGenerationFailure failure =
        HoleToolpathGenerationFailure::InvalidRequest;
    QString error;
    HoleToolpathGeometryPreparation geometry;
    tooling::HoleToolSelectionPlan automaticSelection;
    toolpath::HoleToolpathBatch batch;
    toolpath::ToolpathProgram program;
    ToolpathGenerationValidation validation;
    HoleToolpathCommitResult committed;
    qint64 envelopeEntryMilliseconds = 0;
    qint64 generationMilliseconds = 0;
    int cutMoveCount = 0;
    int rapidMoveCount = 0;
    int verticalMoveCount = 0;
    bool automatic = false;
    QString automaticLibraryName;
    QString summary;

    bool ok() const noexcept
    {
        return failure == HoleToolpathGenerationFailure::None &&
               committed.ok;
    }
};

// 日志报告附加上下文：由调用方提供各阶段计时与识别来源。
struct HoleToolpathGenerationReportContext {
    QString logPath;
    QString recognitionSource;
    qint64 recognitionMilliseconds = 0;
    qint64 aagBuildMilliseconds = 0;
    qint64 holeClassificationMilliseconds = 0;
    qint64 geometryBuildMilliseconds = 0;
};

// 孔刀路生成的完整用例（静态门面）：不依赖 MainWindow 状态，
// 串联几何准备、自动生成、验证、统计与原子提交。
class HoleToolpathGenerationUseCase {
public:
    // 执行完整生成流程；失败时 result.failure 给出分类与原因。
    static HoleToolpathGenerationResult execute(
        const HoleToolpathGenerationRequest& request);

    // 生成避障/生成日志的文本行（含自动选刀、外包络、沉孔等明细）。
    static QStringList buildReport(
        const HoleToolpathGenerationResult& result,
        const HoleToolpathGenerationReportContext& context);
};

} // namespace oap::app
