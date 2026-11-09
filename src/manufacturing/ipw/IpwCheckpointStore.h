// IPW 几何检查点的进程内不可变存储。
//
// Job JSON 只保存可重建定义；TopoDS 形状和单工序仿真明细属于运行时缓存。
// 同一 stateKey 一旦发布便不可覆盖，防止变化后的工序错误复用旧几何。
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::manufacturing::ipw {

// IPW 几何检查点：某工序状态下的不可变材料真值与派生快照。
struct IpwStockCheckpoint {
    std::string stateKey;
    std::string parentStateKey;
    std::string operationId;
    std::string backendId;
    std::string contentFingerprint;
    // IPW 材料真值。内容摘要只基于该多区间状态，不基于派生 BRep。
    std::shared_ptr<const toolpath::MultiDexelStock>
        dexelStock;
    // 显示、旧内核适配或审计使用的派生快照；允许为空。
    TopoDS_Shape stock;
    double volume = 0.0;
    // 根检查点为空；工序检查点保存该 Job 节点的完整局部仿真结果。
    std::shared_ptr<const toolpath::StockSimulationResult>
        operationResult;
};

// 检查点存储:以 stateKey 为键保存不可变检查点。不变量:每个 stateKey 至多
// 发布一次;再次发布相同内容幂等成功,发布不同内容则报错,防止旧几何被
// 错误复用。由 MachiningJob::runtime().ipwCheckpoints 持有,随 Job 生命周期
// 存在;执行器串行访问,单线程使用。
class IpwCheckpointStore {
public:
    // 发布检查点。前置条件:stateKey/backendId/contentFingerprint 非空且
    // dexelStock 有效。同 key 已存在时:内容完全一致视为幂等成功并返回 true;
    // 否则返回 false 并写入错误。失败时保留既有检查点不变。
    bool publish(
        IpwStockCheckpoint checkpoint,
        std::string* error = nullptr);

    // 按 stateKey 查找检查点;不存在返回 nullptr。返回的是共享只读指针。
    std::shared_ptr<const IpwStockCheckpoint> find(
        const std::string& stateKey) const noexcept;
    bool contains(const std::string& stateKey) const noexcept;

    // 只保留当前会话仍可到达的根和就绪前缀。
    void retain(const std::vector<std::string>& stateKeys);
    void clear() noexcept;
    std::size_t size() const noexcept { return m_checkpoints.size(); }

private:
    std::unordered_map<
        std::string,
        std::shared_ptr<const IpwStockCheckpoint>>
        m_checkpoints;
};

} // namespace oap::manufacturing::ipw
