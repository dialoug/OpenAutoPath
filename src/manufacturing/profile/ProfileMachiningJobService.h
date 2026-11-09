// 外轮廓 Job 桥接服务（oap_manufacturing）。
// 持久化 root outer-wire 稳定引用，并在重新打开零件后重建区域与派生刀路；
// 移动本身不进入 JSON。
#pragma once

#include <string>
#include <optional>
#include <vector>

#include "analysis/StepIndexer.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::manufacturing {

// 外轮廓 Job 捕获请求：工序、目标、参数与装夹帧的只读指针集合。
struct ProfileJobCaptureRequest {
    const std::vector<toolpath::ProfileToolpathOperation>* operations =
        nullptr;
    const std::vector<ProfileTargetReference>* targets = nullptr;
    const toolpath::ToolpathParams* parameters = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
    std::optional<std::string> toolLibraryId;
};

// Profile Job 桥接：持久化 root outer-wire 稳定引用，并在重新打开零件后
// 重新构建区域和派生刀路。移动本身不进入 JSON。
class ProfileMachiningJobService {
public:
    bool capture(MachiningJob& job,
                 const ProfileJobCaptureRequest& request,
                 std::string* error = nullptr) const;

    toolpath::ProfileToolpathBatch regenerate(
        MachiningJob& job,
        const StepIndexer& indexer,
        const toolpath::ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::manufacturing
