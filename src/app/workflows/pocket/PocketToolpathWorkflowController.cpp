// PocketToolpathWorkflowController 实现（oap_app_workflow_cam）。
// 候选构造与工作区安装分离，见 PocketToolpathWorkflowController.h。
#include "app/workflows/pocket/PocketToolpathWorkflowController.h"

namespace oap::app {

PocketCandidateJobResult PocketToolpathWorkflowController::prepareCandidateJob(
    const PocketCandidateJobRequest& request)
{
    PocketCandidateJobResult result;
    // 前置校验：任一必填输入缺失即失败，不触碰现有 Job。
    if (!request.currentJob || !request.batch ||
        !request.batch->readyForPublication() ||
        !request.setupFrames) {
        result.error = QStringLiteral("槽腔加工提交请求不完整。");
        return result;
    }
    result.job = *request.currentJob;
    // 只构造候选，不修改输入 Job；显示及工作区安装由调用方负责。
    MachiningProjectPocketCaptureRequest capture;
    capture.definition = request.definition;
    capture.operations = &request.batch->operations;
    capture.rootSources = request.rootSources;
    capture.effectiveParams = &request.batch->effectiveParams;
    capture.setupFrames = request.setupFrames;
    if (!MachiningProjectController::capturePocketOperations(
            result.job, capture, &result.error)) {
        return result;
    }
    auto* captured = result.job.pocketResult();
    if (!captured) {
        result.error = QStringLiteral("槽腔候选没有生成运行期结果。");
        return result;
    }
    captured->safetyValidation = request.batch->safetyValidation;
    captured->safetyValidationPerformed = request.batch->safetyValidationPerformed;
    captured->validation.append(request.batch->validation);
    result.ok = captured->readyForPublication();
    if (!result.ok)
        result.error = QStringLiteral("槽腔候选未满足发布条件。");
    return result;
}

} // namespace oap::app
