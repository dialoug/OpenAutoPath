// 不依赖 MainWindow 状态的 .oapjob 初始化、捕获与加载的与 UI 无关编排。
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <QString>

#include "analysis/Model.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/StepIndexer.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::app {

// .oapjob 定义写入所需的零件上下文:源零件路径、刀具库、
// G 代码程序选项与毛坯定义,由调用方(工程会话)组装。
struct MachiningProjectDefinitionContext {
    QString sourcePath;
    QString activeToolLibraryId;
    toolpath::GCodeProgramOptions gcodeOptions;
    manufacturing::StockDefinition stock;
    // Capture-only override; empty means a standalone tool snapshot. Does not
    // change the user's active library or the persisted project preference.
    std::optional<std::string> operationToolLibraryId;
};

// 槽腔捕获请求:携带完整加工工序列表与配套有效参数;
// 各指针均为借用引用,调用期间必须保持有效。
struct MachiningProjectPocketCaptureRequest {
    MachiningProjectDefinitionContext definition;
    const std::vector<toolpath::PocketToolpathOperation>* operations = nullptr;
    const std::vector<toolpath::RootInnerWirePocketSource>* rootSources =
        nullptr;
    const toolpath::ToolpathParams* effectiveParams = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
};

// 加工项目轮廓捕获请求：捕获外轮廓工序到 Job。
struct MachiningProjectProfileCaptureRequest {
    MachiningProjectDefinitionContext definition;
    const std::vector<toolpath::ProfileToolpathOperation>* operations =
        nullptr;
    const std::vector<toolpath::RootOuterWireProfileSource>* rootSources =
        nullptr;
    const toolpath::ToolpathParams* effectiveParams = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
};

// 保存前同步请求:把当前定义上下文与工位坐标系交给保存服务。
struct MachiningProjectSaveRequest {
    MachiningProjectDefinitionContext definition;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
};

// .oapjob 加载失败原因分类,供 UI 呈现对应的恢复指引。
enum class MachiningProjectLoadFailure {
    None,
    File,
    PartRequired,
    PartFingerprint,
    EmptyProject,
    HoleRegeneration,
    PocketRegeneration,
    ProfileRegeneration,
    SurfaceFinishingRegeneration,
    PendingConfiguration,
};

// 加载请求:目标 .oapjob 路径 + 当前打开的零件模型/索引/工位,
// 用于重建时校验指纹并作为再生输入(裸指针均为借用)。
struct MachiningProjectLoadRequest {
    QString filePath;
    QString currentSourcePath;
    const Model* model = nullptr;
    const StepIndexer* indexer = nullptr;
    const MoldCoordinateFrame* moldFrame = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
};

// 加载结果:失败原因 + 重建后的完整 Job 与孔/槽腔刀路批次。
struct MachiningProjectLoadResult {
    MachiningProjectLoadFailure failure =
        MachiningProjectLoadFailure::None;
    QString error;
    manufacturing::MachiningJob job;
    toolpath::HoleToolpathBatch holes;
    toolpath::PocketToolpathBatch pockets;
    toolpath::ProfileToolpathBatch profiles;
    toolpath::SurfaceFinishingBatch finishing;
    bool hasHoleDefinitions = false;
    bool hasPocketDefinitions = false;
    bool hasProfileDefinitions = false;
    bool hasFinishingDefinitions = false;

    bool ok() const noexcept
    {
        return failure == MachiningProjectLoadFailure::None &&
               error.isEmpty();
    }
};

// 工程状态工作流的无状态控制器:所有方法均为静态函数,
// 输入输出都通过请求/结果结构体传递,不持有 UI 或会话引用,
// 因此可被工程会话、安装器等不同调用方复用。
class MachiningProjectController {
public:
    // 校验并补齐 Job 定义元数据(指纹、schema、id、零件信息、
    // 刀具库、后处理选项、毛坯);零件指纹变化时清空旧 Job 状态。
    static bool ensureDefinition(
        manufacturing::MachiningJob& job,
        const MachiningProjectDefinitionContext& context,
        QString* error = nullptr);
    // 把完整加工结果捕获为 Job 的槽腔工序，失败保留原 Job，
    // 尽可能保留既有"根内环"目标引用后再交给捕获服务。
    static bool capturePocketOperations(
        manufacturing::MachiningJob& job,
        const MachiningProjectPocketCaptureRequest& request,
        QString* error = nullptr);
    static bool captureProfilePreview(
        manufacturing::MachiningJob& job,
        const MachiningProjectProfileCaptureRequest& request,
        QString* error = nullptr);
    // 保存前同步:刷新时间戳/后处理/毛坯等字段并把工位写入 Job。
    static bool synchronizeForSave(
        manufacturing::MachiningJob& job,
        const MachiningProjectSaveRequest& request,
        QString* error = nullptr);
    // 加载 .oapjob 并依据当前零件重建孔/槽腔刀路,
    // 校验失败时返回带分类原因与提示信息的 LoadResult。
    static MachiningProjectLoadResult loadAndRegenerate(
        const MachiningProjectLoadRequest& request);
    // 在内存候选 Job 上重建全部已定义领域。IPW 由仿真工作流重新计算。
    static MachiningProjectLoadResult regenerateJob(
        manufacturing::MachiningJob job,
        const MachiningProjectLoadRequest& request);
    // 把撤销历史中的完整 Job 快照投影为安装器输入，不重新生成几何。
    static MachiningProjectLoadResult restoreSnapshot(
        manufacturing::MachiningJob job);
};

} // namespace oap::app
