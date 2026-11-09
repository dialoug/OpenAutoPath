// 加工项目的应用级协调。该服务只处理可持久化定义、Setup 重绑定和运行期
// 结果状态，不负责文件对话框、用户提示或三维场景。
// 本模块（oap_manufacturing）：配合 MachiningJobStorage 与孔/槽腔 Job 服务，
// 承担 Setup 捕获/重绑定、运行期状态投影和保存前同步三类协调职责。
#pragma once

#include <cstddef>
#include <string>

#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::manufacturing {

// Setup 同步统计报告：调用方用于展示/记录确认保留或失效的数量。
struct SetupSynchronizationReport {
    std::size_t validSetupCount = 0;
    std::size_t invalidSetupCount = 0;
    std::size_t retainedConfirmationCount = 0;
    std::size_t clearedConfirmationCount = 0;
    std::size_t operationsMarkedForReview = 0;
};

// 保存同步请求：由调用方（工作流控制器）提供保存所需的外部值；
// setupFrames 为必填，用于派生 Setup 坐标与确认状态。
struct MachiningJobSaveSynchronizationRequest {
    std::string modifiedAt;
    std::string confirmationTime;
    std::string confirmationActor = "interactive-session";
    PostProfileReference post;
    StockDefinition stock;
    std::string activeToolLibraryId;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
};

// 加工项目应用层服务（oap_manufacturing）：提供 Setup 捕获/重绑定、运行期状态
// 投影与保存前同步。全部为静态方法、无内部状态，线程安全由调用方保证。
class MachiningJobApplicationService {
public:
    // 用当前六面工作坐标覆盖定义中的派生 Setup 值。只有当前 frame 已人工
    // 确认时才保存确认；输入指纹不变时保留原确认时间和操作者。
    static SetupSynchronizationReport captureSetups(
        MachiningJobDefinition& definition,
        const toolpath::MoldSetupWorkFrames& frames,
        const std::string& confirmationTime,
        const std::string& confirmationActor = "interactive-session");

    // 加载后用当前同指纹零件重新派生 Setup。保存的确认只在新输入指纹完全
    // 相同时保留；失配 Setup 关联的孔/槽腔工序统一进入 NeedsReview。
    static SetupSynchronizationReport rebindLoadedSetups(
        MachiningJobDefinition& definition,
        const toolpath::MoldSetupWorkFrames& frames);

    // 把当前派生结果状态投影到持久化 Operation，不修改移动或几何结果。
    static void reconcileOperationStates(
        MachiningJobDefinition& definition,
        const toolpath::HoleToolpathBatch* holeResult,
        const toolpath::PocketToolpathBatch* pocketResult,
        const toolpath::ProfileToolpathBatch* profileResult = nullptr,
        const toolpath::SurfaceFinishingBatch* finishingResult = nullptr);

    // 保存前的完整定义同步。成功时同时刷新运行期执行指纹；结构无效时返回
    // false，并通过 error 返回第一条领域校验错误。
    static bool synchronizeForSave(
        MachiningJob& job,
        const MachiningJobSaveSynchronizationRequest& request,
        std::string* error = nullptr);
};

} // namespace oap::manufacturing
