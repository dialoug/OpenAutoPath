// 自动规划与 Job 重建共享的中立孔批次执行循环。
//
// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)。
// 职责:把多组孔输入(每个「逻辑批次」一个刀具/一组策略)逐批生成刀路,
// 支持并行执行、进度汇总与协作取消;取消或失败时按「全有或全无」语义
// 返回,不泄漏部分批次结果。本类无状态,execute() 可并发调用。
#pragma once

#include "toolpath/hole/HoleBatchExecution.h"
#include "toolpath/workflow/hole/HoleToolpathService.h"

#include <cstddef>
#include <string>
#include <vector>

namespace oap::toolpath {

// 单个逻辑批次:同一把刀具下的一组孔输入及其策略/覆盖项。
struct HoleBatchExecutorItem {
    std::string logicalId;
    std::vector<HoleMachiningInput> inputs;
    std::vector<HoleMachiningStrategy> strategies;
    ToolpathToolContext tool;
    HoleVisitOrderMode visitOrderMode = HoleVisitOrderMode::Optimize;
    HoleRapidLinkMode rapidLinkMode = HoleRapidLinkMode::DirectWhenSafe;
};

// 一次批执行请求:待处理批次列表、统一的基准工艺参数、可选的孔间
// 快速移动规划上下文,以及是否并行执行(受工作线程上限约束)。
struct HoleBatchExecutorRequest {
    std::vector<HoleBatchExecutorItem> batches;
    ToolpathParams baseParams;
    const HoleRapidPlanningContext* rapidContext = nullptr;
    bool parallel = false;
};

// 单个批次的执行结果:批次索引、逻辑 id 与该批生成的刀路批。
struct HoleBatchExecutorItemResult {
    std::size_t batchIndex = 0;
    std::string logicalId;
    HoleToolpathBatch generated;
};

// 批执行汇总结果:各批次结果、聚合加工摘要与诊断;失败/取消时
// batches/machining/diagnostics 一律清空,由 error/cancelled 表达状态。
struct HoleBatchExecutorResult {
    std::vector<HoleBatchExecutorItemResult> batches;
    HoleMachiningResult machining;
    std::vector<ToolpathDiagnostic> diagnostics;
    std::size_t completedBatchCount = 0;
    std::size_t cancelledBatchIndex = static_cast<std::size_t>(-1);
    bool cancelled = false;
    std::string error;

    // ok() 约定:未取消且无错误才算成功。
    bool ok() const noexcept
    {
        return !cancelled && error.empty();
    }
};

// 唯一逐批生成循环。Operation ID、来源索引和 Job 状态由调用方基于稳定
// batchIndex/logicalId 映射；取消时不返回部分 batches 或部分聚合结果。
class HoleBatchExecutor {
public:
    // 执行整个批请求。
    // 参数:request 批次与参数定义;control 可选取消/进度控制(取消回调
    //       会被加锁后从工作线程调用)。
    // 返回:汇总结果;失败/取消时 batches 为空且 ok() 为假。
    HoleBatchExecutorResult execute(
        const HoleBatchExecutorRequest& request,
        const ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::toolpath
