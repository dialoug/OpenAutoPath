// ProfileToolpathWorkflowController 实现（oap_app_workflow_cam）。
// 对已验证外轮廓生成候选的原子 Job 提交，见 ProfileToolpathWorkflowController.h。
#include "app/workflows/profile/ProfileToolpathWorkflowController.h"

namespace oap::app {

ProfileToolpathCommitResult ProfileToolpathWorkflowController::commit(
    const ProfileToolpathCommitRequest &request) {
  ProfileToolpathCommitResult result;
  if (!request.currentJob || !request.operations ||
      request.operations->empty() || !request.effectiveParams ||
      !request.setupFrames) {
    result.error = QStringLiteral("外轮廓加工提交请求不完整。");
    return result;
  }

  result.job = *request.currentJob;
  MachiningProjectProfileCaptureRequest capture;
  capture.definition = request.definition;
  capture.operations = request.operations;
  capture.rootSources = request.rootSources;
  capture.effectiveParams = request.effectiveParams;
  capture.setupFrames = request.setupFrames;
  if (!MachiningProjectController::captureProfilePreview(result.job, capture,
                                                         &result.error)) {
    return result;
  }
  result.ok = true;
  return result;
}

} // namespace oap::app
