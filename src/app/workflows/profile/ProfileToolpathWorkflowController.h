// 对已验证外轮廓生成候选的原子 Job 提交。
#pragma once

#include <vector>

#include <QString>

#include "app/workflows/project/MachiningProjectController.h"

namespace oap::app {

// 轮廓刀路提交请求：待提交的生成候选。
struct ProfileToolpathCommitRequest {
  const manufacturing::MachiningJob *currentJob = nullptr;
  MachiningProjectDefinitionContext definition;
  const std::vector<toolpath::ProfileToolpathOperation> *operations = nullptr;
  const std::vector<toolpath::RootOuterWireProfileSource> *rootSources =
      nullptr;
  const toolpath::ToolpathParams *effectiveParams = nullptr;
  const toolpath::MoldSetupWorkFrames *setupFrames = nullptr;
};

// 轮廓刀路提交结果：提交后的批次与状态。
struct ProfileToolpathCommitResult {
  bool ok = false;
  QString error;
  manufacturing::MachiningJob job;
};

// 与孔、槽腔提交控制器保持相同事务边界：只修改 Job 副本，
// 捕获全部通过后才把候选 Job 返回给工作区安装。
class ProfileToolpathWorkflowController {
public:
  static ProfileToolpathCommitResult
  commit(const ProfileToolpathCommitRequest &request);
};

} // namespace oap::app
