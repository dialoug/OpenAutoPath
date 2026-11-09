// 槽腔 Job 桥接层：把当前槽腔预览捕获成稳定定义，并可在重新加载项目后
// 根据普通面组或主外表面根内环引用重建派生刀路。
//
// 本文件属于 oap_manufacturing 模块(CMake target: oap_manufacturing)。协作:
// 依赖 MachiningJob/Storage 的序列化、PocketRegionBuilder 的面组/根内环
// 重建、PocketToolpathService 生成与 PocketToolpathValidator 校验;UI 无关。
#pragma once

#include <string>
#include <optional>
#include <vector>

#include "analysis/StepIndexer.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::manufacturing {

// 槽腔捕获请求：预览生成的槽腔工序、一一对应的目标来源引用、参数与装夹
// 工作坐标；targets 提供无法从派生移动反推的稳定来源信息。
struct PocketJobCaptureRequest {
    const std::vector<toolpath::PocketToolpathOperation>* operations =
        nullptr;
    // 与 operations 一一对应，负责补充根面/根内环等不可从派生移动反推的
    // 稳定来源信息。
    const std::vector<PocketTargetReference>* targets = nullptr;
    const toolpath::ToolpathParams* parameters = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
    std::optional<std::string> toolLibraryId;
};

// 槽腔工序的服务层：capture() 把预览结果固化到 Job 定义（仅替换槽腔工序），
// regenerate() 按已捕获的稳定来源引用重建运行期刀路缓存。
class PocketMachiningJobService {
public:
    // 保留已有孔工序，只替换槽腔工序及其不再被引用的 Setup/ToolController。
    bool capture(MachiningJob& job,
                 const PocketJobCaptureRequest& request,
                 std::string* error = nullptr) const;

    // 只刷新槽腔运行期缓存，不清除已重建的孔刀路。
    toolpath::PocketToolpathBatch regenerate(
        MachiningJob& job,
        const StepIndexer& indexer,
        const toolpath::ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::manufacturing
