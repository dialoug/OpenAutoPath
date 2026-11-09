// 为已验证槽/腔刀路构造候选 Job，不安装工作区状态。
// 属 oap_app_workflow_cam 槽/腔工作流模块（src/app/workflows/pocket），
// 由 PocketToolpathGenerationUseCase 调用，不依赖具体 UI 框架。
#pragma once

#include <vector>

#include <QString>

#include "app/workflows/project/MachiningProjectController.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"

namespace oap::app {

// 候选构造请求：指针字段由调用方持有并保证存活；
// currentJob 为基线 Job，提交不修改这些输入。
struct PocketCandidateJobRequest {
    const manufacturing::MachiningJob* currentJob = nullptr;
    MachiningProjectDefinitionContext definition;
    const toolpath::PocketToolpathBatch* batch = nullptr;
    const std::vector<toolpath::RootInnerWirePocketSource>* rootSources =
        nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
};

// 候选结果：ok 为 true 时 job 可供调用方安装；输入 Job 始终不变。
struct PocketCandidateJobResult {
    bool ok = false;
    QString error;
    manufacturing::MachiningJob job;
};

// 槽/腔候选构造器：直接捕获加工工序，与显示成功的条目数量无关。
class PocketToolpathWorkflowController {
public:
    // 在副本中构造完整候选；不发布 UI，也不提供多线程原子性保证。
    static PocketCandidateJobResult prepareCandidateJob(
        const PocketCandidateJobRequest& request);
};

} // namespace oap::app
