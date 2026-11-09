// 不依赖 MainWindow 状态的完整桌面外轮廓刀路候选生成。
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <QString>
#include <QStringList>

#include "analysis/MoldCoordinateFrame.h"
#include "app/workflows/profile/ProfileToolpathWorkflowController.h"
#include "app/workflows/shared/WorkflowInteraction.h"
#include "app/workflows/shared/ToolpathGenerationValidation.h"
#include "tooling/adapter/profile/ProfileAutomaticToolpath.h"
#include "toolpath/profile/geometry/ProfileRegion.h"

namespace oap::app {

enum class ProfileToolpathGenerationFailure {
  None,
  InvalidRequest,
  Cancelled,
  EmptyProgram,
  ValidationFailed,
  CommitFailed,
  PreviewFailed,
};

// 轮廓刀路生成请求：区域、参数与生成选项。
struct ProfileToolpathGenerationRequest {
  WorkflowInteraction interaction;
  const std::vector<toolpath::ProfileRegion> *recognizedRegions = nullptr;
  const toolpath::ToolpathParams *params = nullptr;
  const tooling::ToolLibrary *automaticLibrary = nullptr;
  const toolpath::ToolpathToolContext *singleTool = nullptr;
  const MoldCoordinateFrame *moldFrame = nullptr;
  // Empty selects all recognized directions. Selection uses the mold frame,
  // not STEP world axes; only selected regions enter planning and commit.
  std::optional<MoldSetupFace> selectedSetupFace;
  // Mutually exclusive with selectedSetupFace. Existing API callers retain
  // all-directions behavior unless they explicitly request automatic reduction.
  bool automaticSetupDirections = false;
  const manufacturing::MachiningJob *currentJob = nullptr;
  MachiningProjectDefinitionContext definition;
  const std::vector<toolpath::RootOuterWireProfileSource> *rootSources =
      nullptr;
  const toolpath::MoldSetupWorkFrames *setupFrames = nullptr;
};

using ProfilePreviewPublisher =
    std::function<bool(const toolpath::ToolpathProgram &)>;

// 轮廓刀路生成结果：批次、诊断与统计。
struct ProfileToolpathGenerationResult {
  ProfileToolpathGenerationFailure failure =
      ProfileToolpathGenerationFailure::InvalidRequest;
  QString error;
  std::vector<toolpath::ProfileRegion> regions;
  toolpath::ProfileToolpathBatch batch;
  tooling::ProfileAutomaticToolpathResult automaticSelection;
  toolpath::ToolpathProgram program;
  ToolpathGenerationValidation validation;
  ProfileToolpathCommitResult committed;
  QString summary;
  QStringList directionReport;
  bool automatic = false;

  bool ok() const noexcept {
    return failure == ProfileToolpathGenerationFailure::None && committed.ok;
  }
};

// 串联六面排序、毛坯边界、自动生成、装夹分配、验证、
// Job 副本提交与预览发布。失败时调用方当前 Job 不会被修改。
class ProfileToolpathGenerationUseCase {
public:
  static ProfileToolpathGenerationResult
  execute(const ProfileToolpathGenerationRequest &request,
          const ProfilePreviewPublisher &previewPublisher = {});

  static QStringList buildReport(const ProfileToolpathGenerationResult &result);
};

} // namespace oap::app
