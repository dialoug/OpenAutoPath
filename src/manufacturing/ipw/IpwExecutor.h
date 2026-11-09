// MachiningJob 的统一 IPW 顺序执行器。
//
// 孔、槽腔和外轮廓共用父检查点链，检查点材料真值为分块多区间 Dexel。
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "manufacturing/ipw/IpwCheckpointStore.h"
#include "manufacturing/ipw/IpwSession.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::manufacturing {

class MachiningJob;

namespace ipw {

// 混合 IPW 引擎的稳定标识:孔原生减料 + 槽腔/Profile 语义图元减料。
inline constexpr const char* kHybridIpwEngineId =
    "oap-multi-dexel-ipw-v8-surface-finishing";

// 一次 IPW 执行的选项:孔/槽腔各自的仿真参数,以及可选的工序子集。
struct IpwExecutionOptions {
    toolpath::StockSimulationOptions hole;
    toolpath::StockSimulationOptions pocket;
    // 槽腔不同刀轴仍需要派生 BRep 过渡；R6 不改变该行为。
    bool safeMixedTopologyFallback = true;
    // 为空时执行 Job 中全部已启用工序；非空时只把这些稳定工序 ID
    // 映射到本次 IPW 计划。用于桌面端按单个 Setup 验证固定轴闭环，
    // 不修改或复制 Job 的持久化工序定义。
    std::vector<std::string> includedOperationIds;
};

// 执行结果状态:完整完成 / 失败 / 取消。
enum class IpwExecutionStatus {
    Complete,
    Failed,
    Cancelled,
};

// 执行结果:状态、计划更新统计、复用/执行节点数、执行的工序 ID、最终检查点
// 与错误消息;complete() 表示全部节点成功执行完毕。
struct IpwExecutionResult {
    IpwExecutionStatus status = IpwExecutionStatus::Failed;
    IpwPlanUpdate planUpdate;
    std::size_t reusedCheckpointCount = 0;
    std::size_t executedNodeCount = 0;
    std::size_t nativeHoleNodeCount = 0;
    std::vector<std::string> executedOperationIds;
    std::shared_ptr<const IpwStockCheckpoint> finalCheckpoint;
    std::string message;

    bool complete() const noexcept
    {
        return status == IpwExecutionStatus::Complete;
    }
};

// 统一 IPW 顺序执行器:按会话计划逐节点执行,复用检查点存储中的就绪前缀,
// 为每个节点发布检查点并推进会话状态。execute 为 const,本身无内部状态。
class IpwExecutor {
public:
    // 对 job 执行一次完整 IPW 顺序计划。步骤:计算执行帧(首个工序的刀轴与
    // 参考刀具直径)并配置会话 → 校验并清理就绪前缀 → 复用/重建根检查点 →
    // 逐节点仿真并发布检查点。initialStock 为原始毛坯形状;指纹用于内容寻址;
    // 引擎指纹默认 kHybridIpwEngineId。返回结果含最终检查点;失败时会话停在
    // 首个失败节点。control 允许取消与进度回调。
    IpwExecutionResult execute(
        MachiningJob& job,
        const TopoDS_Shape& initialStock,
        std::string initialStockFingerprint,
        std::string engineFingerprint =
            kHybridIpwEngineId,
        const IpwExecutionOptions& options = {},
        const toolpath::ToolpathGenerationControl* control =
            nullptr) const;
};

const char* ipwExecutionStatusName(
    IpwExecutionStatus status) noexcept;

} // namespace ipw
} // namespace oap::manufacturing
