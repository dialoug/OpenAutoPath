// 统一 IPW 工序链的运行期状态。
//
// 该层不依赖具体几何表达。多区间 Dexel、BRep 审计和显示网格都通过
// IpwArtifactDescriptor 发布不可变产物；输入变化时仅复用相同的连续前缀，
// 并从首个变化工序开始向后失效。
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace oap::manufacturing::ipw {

// 工序种类:孔、槽腔、外轮廓或未知;决定执行时选择哪个后端。
enum class IpwOperationKind {
    Hole,
    Pocket,
    Profile,
    SurfaceFinishing,
    Other,
};

// 节点的生命周期状态:Dirty(待执行) -> Computing(执行中) ->
// Ready(完成) | Failed(失败) | Cancelled(取消)。
enum class IpwNodeStatus {
    Dirty,
    Computing,
    Ready,
    Failed,
    Cancelled,
};

// IPW 计划中的工序定义(运行期视图):由 Job 定义按顺序映射而来,
// 指纹与序列用于计算稳定的节点 stateKey。
struct IpwOperationDefinition {
    std::string id;
    std::string setupId;
    std::string toolControllerId;
    std::string inputFingerprint;
    int sequence = 0;
    IpwOperationKind kind = IpwOperationKind::Other;

    bool operator==(const IpwOperationDefinition&) const = default;
};

// 已发布产物的不可变描述:由后端在执行后填写,挂接到 Ready 节点。
struct IpwArtifactDescriptor {
    // 必须与所属节点的 stateKey 相同，防止错误地挂接旧缓存。
    std::string stateKey;
    std::string backendId;
    std::string contentFingerprint;
    // 可为空；为空表示产物只存在于当前进程内存中。
    std::string storagePath;
    bool checkpoint = false;
};

// 计划中的单个节点:工序定义 + 父节点 stateKey + 本节点 stateKey + 状态。
// 不变量:节点按全局序列有序,parentStateKey 总指向前一节点或根键。
struct IpwNode {
    IpwOperationDefinition operation;
    std::string parentStateKey;
    std::string stateKey;
    IpwNodeStatus status = IpwNodeStatus::Dirty;
    std::optional<IpwArtifactDescriptor> artifact;
    std::string message;
};

// configure() 的结果:统计被复用/置脏/丢弃的节点数,以及初始毛坯或引擎
// 指纹是否变化;ok() 表示配置有效,changed() 表示是否产生实际变化。
struct IpwPlanUpdate {
    std::size_t reusedNodeCount = 0;
    std::size_t dirtyNodeCount = 0;
    std::size_t discardedNodeCount = 0;
    bool initialStockChanged = false;
    bool engineChanged = false;
    std::string error;

    bool ok() const noexcept { return error.empty(); }
    bool changed() const noexcept
    {
        return initialStockChanged || engineChanged ||
               dirtyNodeCount > 0 || discardedNodeCount > 0;
    }
};

// IPW 会话:维护工序链的运行期状态与内容寻址的 stateKey 链。职责:
// 1) configure() 按新指纹与工序集合重建节点链,仅复用相同的连续前缀;
// 2) 按状态机推进 Dirty->Computing->Ready(Failed/Cancelled),并支持从
//    任意索引向后失效。几何真值不在本层,而由执行器经检查点存储发布。
// 由 MachiningJob::ipwSession() 持有;单线程使用,不提供同步。
class IpwSession {
public:
    // configure() 是事务式的：输入无效时保留原会话不变。
    // operations 可以无序传入，但 sequence 必须唯一且为非负数。
    IpwPlanUpdate configure(
        std::string initialStockFingerprint,
        std::string engineFingerprint,
        std::vector<IpwOperationDefinition> operations);

    bool configured() const noexcept
    {
        return !m_initialStockFingerprint.empty() &&
               !m_engineFingerprint.empty();
    }
    const std::string& initialStockFingerprint() const noexcept
    {
        return m_initialStockFingerprint;
    }
    const std::string& engineFingerprint() const noexcept
    {
        return m_engineFingerprint;
    }
    const std::string& initialStateKey() const noexcept
    {
        return m_initialStateKey;
    }
    const std::vector<IpwNode>& nodes() const noexcept
    {
        return m_nodes;
    }
    const IpwNode* node(std::size_t index) const noexcept;
    const IpwNode* nodeByOperationId(
        const std::string& operationId) const noexcept;

    // 仅当全部前置节点 Ready 时，Dirty 节点才可启动。
    std::optional<std::size_t> firstRunnableNode() const noexcept;
    // 把 Dirty 节点推进为 Computing(仅当前置节点全部 Ready 时允许);
    // 成功后递增修订号。
    bool beginNode(std::size_t index);
    // 校验产物描述(stateKey/backendId/内容指纹)后把 Computing 节点置为
    // Ready;不匹配或状态非法返回 false。
    bool completeNode(
        std::size_t index,
        IpwArtifactDescriptor artifact);
    // 把 Computing 节点置为 Failed/Cancelled 并记录消息;仅接受 Computing。
    bool failNode(std::size_t index, std::string message);
    bool cancelNode(std::size_t index, std::string message = {});

    // 从给定索引起把节点全部重置为 Dirty 并清空产物与消息。
    void invalidateFrom(std::size_t index);
    // 会话已配置且所有节点都已 Ready。
    bool complete() const noexcept;
    std::size_t revision() const noexcept { return m_revision; }
    // 清空会话(指纹、根键与节点),回到未配置状态。
    void clear();

private:
    IpwNode* mutableNode(std::size_t index) noexcept;
    bool precedingNodesReady(std::size_t index) const noexcept;

    std::string m_initialStockFingerprint;
    std::string m_engineFingerprint;
    std::string m_initialStateKey;
    std::vector<IpwNode> m_nodes;
    std::size_t m_revision = 0;
};

const char* ipwNodeStatusName(IpwNodeStatus status) noexcept;
const char* ipwOperationKindName(IpwOperationKind kind) noexcept;

} // namespace oap::manufacturing::ipw
