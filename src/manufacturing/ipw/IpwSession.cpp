// IpwSession 的实现:内容寻址的 stateKey 链、最长前缀复用与节点状态机。
// 属于 oap_manufacturing::ipw 模块,配合 IpwCheckpointStore 使用。
#include "manufacturing/ipw/IpwSession.h"

#include <algorithm>
#include <string_view>
#include <unordered_set>
#include <utility>

#include <QByteArray>
#include <QCryptographicHash>

namespace oap::manufacturing::ipw {
namespace {

void addFingerprintPart(
    QCryptographicHash& hash,
    std::string_view value)
{
    const QByteArray bytes(
        value.data(),
        static_cast<qsizetype>(value.size()));
    hash.addData(QByteArray::number(bytes.size()));
    hash.addData(QByteArrayLiteral(":"));
    hash.addData(bytes);
    hash.addData(QByteArrayLiteral(";"));
}

// 以长度前缀方式把多个部件拼入 SHA-256,得到确定性的内容寻址键
// (长度前缀避免部件边界歧义)。
std::string stableStateKey(
    const std::vector<std::string_view>& parts)
{
    QCryptographicHash hash(QCryptographicHash::Sha256);
    for (const std::string_view part : parts)
        addFingerprintPart(hash, part);
    return hash.result().toHex().toStdString();
}

// 根节点键:由初始毛坯指纹与引擎指纹决定;任一变化即生成新根键,使旧前缀失效。
std::string rootStateKey(
    const std::string& stockFingerprint,
    const std::string& engineFingerprint)
{
    return stableStateKey({
        "oap-ipw-root-v1",
        stockFingerprint,
        engineFingerprint,
    });
}

// 节点键:由父节点键 + 工序的 ID/设置/刀具/输入指纹/序列/种类决定;因此
// 输入变化只会让从该节点起的后缀键全部改变,而前缀键保持不变。
std::string nodeStateKey(
    const std::string& parentStateKey,
    const IpwOperationDefinition& operation)
{
    const std::string sequence =
        std::to_string(operation.sequence);
    const std::string kind =
        std::to_string(static_cast<int>(operation.kind));
    return stableStateKey({
        "oap-ipw-node-v1",
        parentStateKey,
        operation.id,
        operation.setupId,
        operation.toolControllerId,
        operation.inputFingerprint,
        sequence,
        kind,
    });
}

std::string validateOperations(
    const std::vector<IpwOperationDefinition>& operations)
{
    std::unordered_set<std::string> ids;
    std::unordered_set<int> sequences;
    ids.reserve(operations.size());
    sequences.reserve(operations.size());
    for (const IpwOperationDefinition& operation : operations) {
        if (operation.id.empty())
            return "IPW operation id is empty";
        if (operation.setupId.empty())
            return "IPW operation setup id is empty";
        if (operation.toolControllerId.empty())
            return "IPW operation tool controller id is empty";
        if (operation.inputFingerprint.empty())
            return "IPW operation input fingerprint is empty";
        if (operation.sequence < 0)
            return "IPW operation sequence is negative";
        const int kind = static_cast<int>(operation.kind);
        if (kind < static_cast<int>(IpwOperationKind::Hole) ||
            kind > static_cast<int>(IpwOperationKind::Other)) {
            return "IPW operation kind is invalid";
        }
        if (!ids.insert(operation.id).second)
            return "IPW operation id is duplicated";
        if (!sequences.insert(operation.sequence).second)
            return "IPW operation sequence is duplicated";
    }
    return {};
}

} // namespace

const char* ipwNodeStatusName(IpwNodeStatus status) noexcept
{
    switch (status) {
        case IpwNodeStatus::Dirty: return "dirty";
        case IpwNodeStatus::Computing: return "computing";
        case IpwNodeStatus::Ready: return "ready";
        case IpwNodeStatus::Failed: return "failed";
        case IpwNodeStatus::Cancelled: return "cancelled";
    }
    return "dirty";
}

const char* ipwOperationKindName(IpwOperationKind kind) noexcept
{
    switch (kind) {
        case IpwOperationKind::Hole: return "hole";
        case IpwOperationKind::Pocket: return "pocket";
        case IpwOperationKind::Profile: return "profile";
        case IpwOperationKind::SurfaceFinishing: return "surface_finishing";
        case IpwOperationKind::Other: return "other";
    }
    return "other";
}

IpwPlanUpdate IpwSession::configure(
    std::string initialStockFingerprint,
    std::string engineFingerprint,
    std::vector<IpwOperationDefinition> operations)
{
    IpwPlanUpdate update;
    if (initialStockFingerprint.empty()) {
        update.error = "IPW initial stock fingerprint is empty";
        return update;
    }
    if (engineFingerprint.empty()) {
        update.error = "IPW engine fingerprint is empty";
        return update;
    }
    update.error = validateOperations(operations);
    if (!update.error.empty())
        return update;

    // 先按序列排序,保证节点链顺序与全局加工顺序一致。
    std::sort(
        operations.begin(), operations.end(),
        [](const IpwOperationDefinition& left,
           const IpwOperationDefinition& right) {
            return left.sequence < right.sequence;
        });

    // 用新指纹重建节点链:每个节点持父键,stateKey 为父键与工序的确定性哈希。
    const std::string nextInitialStateKey =
        rootStateKey(initialStockFingerprint, engineFingerprint);
    std::vector<IpwNode> nextNodes;
    nextNodes.reserve(operations.size());
    std::string parentStateKey = nextInitialStateKey;
    for (IpwOperationDefinition& operation : operations) {
        IpwNode node;
        node.operation = std::move(operation);
        node.parentStateKey = parentStateKey;
        node.stateKey =
            nodeStateKey(node.parentStateKey, node.operation);
        parentStateKey = node.stateKey;
        nextNodes.push_back(std::move(node));
    }

    update.initialStockChanged =
        m_initialStockFingerprint != initialStockFingerprint;
    update.engineChanged =
        m_engineFingerprint != engineFingerprint;
    // 复用最长相同前缀:只要旧节点与新节点在工序内容与 stateKey 上完全
    // 一致,就延续其状态与产物;此后所有节点按 Dirty 重新排队。
    const std::size_t maximumReusable =
        std::min(m_nodes.size(), nextNodes.size());
    while (update.reusedNodeCount < maximumReusable) {
        const IpwNode& oldNode =
            m_nodes[update.reusedNodeCount];
        IpwNode& nextNode =
            nextNodes[update.reusedNodeCount];
        if (oldNode.operation != nextNode.operation ||
            oldNode.parentStateKey != nextNode.parentStateKey ||
            oldNode.stateKey != nextNode.stateKey) {
            break;
        }
        nextNode.status = oldNode.status;
        nextNode.artifact = oldNode.artifact;
        nextNode.message = oldNode.message;
        ++update.reusedNodeCount;
    }
    update.dirtyNodeCount =
        nextNodes.size() - update.reusedNodeCount;
    update.discardedNodeCount =
        m_nodes.size() - update.reusedNodeCount;

    // 只有实际变化(指纹/复用数/丢弃数)时才递增修订号。
    const bool changed = update.changed();
    m_initialStockFingerprint =
        std::move(initialStockFingerprint);
    m_engineFingerprint = std::move(engineFingerprint);
    m_initialStateKey = nextInitialStateKey;
    m_nodes = std::move(nextNodes);
    if (changed)
        ++m_revision;
    return update;
}

const IpwNode* IpwSession::node(std::size_t index) const noexcept
{
    return index < m_nodes.size() ? &m_nodes[index] : nullptr;
}

IpwNode* IpwSession::mutableNode(std::size_t index) noexcept
{
    return index < m_nodes.size() ? &m_nodes[index] : nullptr;
}

const IpwNode* IpwSession::nodeByOperationId(
    const std::string& operationId) const noexcept
{
    const auto found = std::find_if(
        m_nodes.begin(), m_nodes.end(),
        [&operationId](const IpwNode& node) {
            return node.operation.id == operationId;
        });
    return found == m_nodes.end() ? nullptr : &*found;
}

bool IpwSession::precedingNodesReady(
    std::size_t index) const noexcept
{
    return index <= m_nodes.size() &&
           std::all_of(
               m_nodes.begin(),
               m_nodes.begin() +
                   static_cast<std::ptrdiff_t>(index),
               [](const IpwNode& node) {
                   return node.status == IpwNodeStatus::Ready;
               });
}

std::optional<std::size_t>
IpwSession::firstRunnableNode() const noexcept
{
    for (std::size_t index = 0;
         index < m_nodes.size(); ++index) {
        if (m_nodes[index].status == IpwNodeStatus::Dirty &&
            precedingNodesReady(index)) {
            return index;
        }
        if (m_nodes[index].status != IpwNodeStatus::Ready)
            return std::nullopt;
    }
    return std::nullopt;
}

bool IpwSession::beginNode(std::size_t index)
{
    IpwNode* target = mutableNode(index);
    if (!target ||
        target->status != IpwNodeStatus::Dirty ||
        !precedingNodesReady(index)) {
        return false;
    }
    target->status = IpwNodeStatus::Computing;
    target->artifact.reset();
    target->message.clear();
    ++m_revision;
    return true;
}

// 产物必须与节点 stateKey 严格一致,防止把旧缓存错误挂接到新节点。
bool IpwSession::completeNode(
    std::size_t index,
    IpwArtifactDescriptor artifact)
{
    IpwNode* target = mutableNode(index);
    if (!target ||
        target->status != IpwNodeStatus::Computing ||
        artifact.stateKey != target->stateKey ||
        artifact.backendId.empty() ||
        artifact.contentFingerprint.empty()) {
        return false;
    }
    target->status = IpwNodeStatus::Ready;
    target->artifact = std::move(artifact);
    target->message.clear();
    ++m_revision;
    return true;
}

bool IpwSession::failNode(
    std::size_t index,
    std::string message)
{
    IpwNode* target = mutableNode(index);
    if (!target ||
        target->status != IpwNodeStatus::Computing) {
        return false;
    }
    target->status = IpwNodeStatus::Failed;
    target->artifact.reset();
    target->message = std::move(message);
    ++m_revision;
    return true;
}

bool IpwSession::cancelNode(
    std::size_t index,
    std::string message)
{
    IpwNode* target = mutableNode(index);
    if (!target ||
        target->status != IpwNodeStatus::Computing) {
        return false;
    }
    target->status = IpwNodeStatus::Cancelled;
    target->artifact.reset();
    target->message = std::move(message);
    ++m_revision;
    return true;
}

// 把 index 及其后的节点全部重置为 Dirty;若任何节点状态确有变化则递增修订号。
void IpwSession::invalidateFrom(std::size_t index)
{
    if (index >= m_nodes.size())
        return;
    bool changed = false;
    for (std::size_t current = index;
         current < m_nodes.size(); ++current) {
        IpwNode& target = m_nodes[current];
        changed =
            changed ||
            target.status != IpwNodeStatus::Dirty ||
            target.artifact.has_value() ||
            !target.message.empty();
        target.status = IpwNodeStatus::Dirty;
        target.artifact.reset();
        target.message.clear();
    }
    if (changed)
        ++m_revision;
}

bool IpwSession::complete() const noexcept
{
    return configured() &&
           std::all_of(
               m_nodes.begin(), m_nodes.end(),
               [](const IpwNode& node) {
                   return node.status == IpwNodeStatus::Ready;
               });
}

void IpwSession::clear()
{
    if (!configured() && m_nodes.empty())
        return;
    m_initialStockFingerprint.clear();
    m_engineFingerprint.clear();
    m_initialStateKey.clear();
    m_nodes.clear();
    ++m_revision;
}

} // namespace oap::manufacturing::ipw
