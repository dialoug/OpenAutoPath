// 第一阶段孔加工 Job 桥接层：把现有自动选刀孔刀路捕获为稳定定义，
// 并可在项目重新加载后从定义重建运行期 HoleToolpathBatch。
// 本模块（oap_manufacturing）：依赖 tooling 选刀/规划与 toolpath 孔生成服务，
// 捕获、选择计划和重建独立编译；共享身份/IPW 逻辑位于 hole/detail。
#pragma once

#include <string>
#include <optional>
#include <vector>

#include "manufacturing/job/MachiningJob.h"
#include "tooling/adapter/hole/HoleToolSelectionPlanner.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::manufacturing {

// 孔 Job 捕获请求：调用方提供零件/毛坯/机床等基础定义与生成的刀路结果；
// 服务补齐参数、Setup、ToolController、Operation 并绑定运行期身份。
struct HoleJobCaptureRequest {
    // 调用方填写零件、毛坯、机床、后处理和项目标识；本服务补齐孔参数、
    // Setup、ToolController、Operation 以及当前派生结果。
    MachiningJobDefinition definition;
    // 新主入口。Job JSON 暂时仍写旧 schema，但只使用显式持久化投影，
    // unresolved 输入不会进入运行期 Operation。
    const std::vector<toolpath::HoleMachiningInput>* inputs = nullptr;
    const tooling::HoleToolSelectionPlan* automaticSelection = nullptr;
    const toolpath::HoleToolpathBatch* generated = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
    std::optional<std::string> toolLibraryId;
};

// 孔加工 Job 服务（oap_manufacturing）：负责把自动孔刀路捕获为稳定定义、
// 从定义恢复选刀计划，以及加载后重建运行期刀路。不持有状态，可复用。
class HoleMachiningJobService {
public:
    MachiningJob capture(const HoleJobCaptureRequest& request) const;

    // 在已有混合 Job 上原子安装新孔定义，保留槽腔结果和可复用 IPW 前缀。
    bool capture(MachiningJob& job,
                 const HoleJobCaptureRequest& request,
                 std::string* error = nullptr) const;

    // 从持久化 Operation、ToolController 和 Setup 快照恢复可编辑计划。
    tooling::HoleToolSelectionPlan selectionPlan(
        const MachiningJob& job) const;

    // 加载后的 Job 不含运行期移动。此方法按启用顺序重新调用现有孔生成器，
    // 写回 operation 状态及运行期缓存；Job 定义结构错误时返回空批次。
    toolpath::HoleToolpathBatch regenerate(
        MachiningJob& job,
        const toolpath::HoleRapidPlanningContext* rapidContext = nullptr,
        const toolpath::ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::manufacturing
