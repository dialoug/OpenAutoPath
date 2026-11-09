// IpwCheckpointStore 的实现:进程内不可变检查点存储,支持幂等发布、
// 键查询与按可达前缀清理。属于 oap_manufacturing::ipw 模块。
#include "manufacturing/ipw/IpwCheckpointStore.h"

#include <unordered_set>
#include <utility>

namespace oap::manufacturing::ipw {

bool IpwCheckpointStore::publish(
    IpwStockCheckpoint checkpoint,
    std::string* error)
{
    auto fail = [error](std::string message) {
        if (error)
            *error = std::move(message);
        return false;
    };
    if (checkpoint.stateKey.empty())
        return fail("IPW checkpoint state key is empty");
    if (checkpoint.backendId.empty())
        return fail("IPW checkpoint backend id is empty");
    if (checkpoint.contentFingerprint.empty())
        return fail("IPW checkpoint content fingerprint is empty");
    if (!checkpoint.dexelStock ||
        !checkpoint.dexelStock->valid()) {
        return fail(
            "IPW checkpoint multi-dexel stock is invalid");
    }

    // 同 key 已存在:若四要素(内容指纹/后端/父键/工序 ID)完全一致则视为
    // 幂等重复发布,直接成功;否则说明同 key 绑定了不同几何,拒绝发布。
    const auto existing =
        m_checkpoints.find(checkpoint.stateKey);
    if (existing != m_checkpoints.end()) {
        if (existing->second->contentFingerprint ==
                checkpoint.contentFingerprint &&
            existing->second->backendId ==
                checkpoint.backendId &&
            existing->second->parentStateKey ==
                checkpoint.parentStateKey &&
            existing->second->operationId ==
                checkpoint.operationId) {
            if (error)
                error->clear();
            return true;
        }
        return fail(
            "IPW checkpoint state key is already bound to different content");
    }

    const std::string key = checkpoint.stateKey;
    m_checkpoints.emplace(
        key,
        std::make_shared<const IpwStockCheckpoint>(
            std::move(checkpoint)));
    if (error)
        error->clear();
    return true;
}

std::shared_ptr<const IpwStockCheckpoint>
IpwCheckpointStore::find(
    const std::string& stateKey) const noexcept
{
    const auto found = m_checkpoints.find(stateKey);
    return found == m_checkpoints.end()
        ? nullptr
        : found->second;
}

bool IpwCheckpointStore::contains(
    const std::string& stateKey) const noexcept
{
    return m_checkpoints.contains(stateKey);
}

// 只保留传入的 stateKey(当前会话可达的根与就绪前缀),其余检查点移除,
// 用于在执行前清理失效分支。
void IpwCheckpointStore::retain(
    const std::vector<std::string>& stateKeys)
{
    const std::unordered_set<std::string> retained(
        stateKeys.begin(), stateKeys.end());
    std::erase_if(
        m_checkpoints,
        [&retained](const auto& item) {
            return !retained.contains(item.first);
        });
}

void IpwCheckpointStore::clear() noexcept
{
    m_checkpoints.clear();
}

} // namespace oap::manufacturing::ipw
