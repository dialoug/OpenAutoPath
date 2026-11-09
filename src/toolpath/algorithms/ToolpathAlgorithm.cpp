// 刀路算法注册表、稳定 ID 查找和默认内置算法集合的实现。
//
// 模块归属:oap_toolpath_algorithms(可替换算法实现与注册表)。
// 注册表以 id → 算法对象(shared_ptr<const>)的映射组织,注册时拒绝
// 空指针/空 id 与重复 id;应用层可先复制默认注册表再追加自定义算法。
// 只读共享给多个后台任务是安全的,本文件无全局可变状态。
#include "toolpath/algorithms/ToolpathAlgorithm.h"

#include <utility>

#include "toolpath/algorithms/hole/BuiltinHoleToolpathAlgorithm.h"
#include "toolpath/algorithms/pocket/BuiltinPocketToolpathAlgorithm.h"
#include "toolpath/algorithms/profile/BuiltinProfileToolpathAlgorithm.h"

namespace oap::toolpath {

// 注册槽腔算法:拒绝空指针/空 id;重复 id 注册返回 false(首次注册胜出)。
bool ToolpathAlgorithmRegistry::registerPocketAlgorithm(
    std::shared_ptr<const IPocketToolpathAlgorithm> algorithm)
{
    if (!algorithm || algorithm->id().empty())
        return false;
    const std::string id(algorithm->id());
    return m_pocketAlgorithms.emplace(id, std::move(algorithm)).second;
}

// 注册孔加工算法:规则与 registerPocketAlgorithm 相同。
bool ToolpathAlgorithmRegistry::registerHoleAlgorithm(
    std::shared_ptr<const IHoleToolpathAlgorithm> algorithm)
{
    if (!algorithm || algorithm->id().empty())
        return false;
    const std::string id(algorithm->id());
    return m_holeAlgorithms.emplace(id, std::move(algorithm)).second;
}

bool ToolpathAlgorithmRegistry::registerProfileAlgorithm(
    std::shared_ptr<const IProfileToolpathAlgorithm> algorithm)
{
    if (!algorithm || algorithm->id().empty())
        return false;
    const std::string id(algorithm->id());
    return m_profileAlgorithms.emplace(id, std::move(algorithm)).second;
}

// 按 id 查询槽腔算法;未注册返回 nullptr。返回裸指针的生命周期由注册表
// 的 shared_ptr 保证,调用方不得持有超过注册表的存续期。
const IPocketToolpathAlgorithm* ToolpathAlgorithmRegistry::pocketAlgorithm(
    std::string_view id) const noexcept
{
    const auto found = m_pocketAlgorithms.find(id);
    return found == m_pocketAlgorithms.end() ? nullptr : found->second.get();
}

// 按 id 查询孔加工算法;未注册返回 nullptr,生命周期语义同上。
const IHoleToolpathAlgorithm* ToolpathAlgorithmRegistry::holeAlgorithm(
    std::string_view id) const noexcept
{
    const auto found = m_holeAlgorithms.find(id);
    return found == m_holeAlgorithms.end() ? nullptr : found->second.get();
}

const IProfileToolpathAlgorithm* ToolpathAlgorithmRegistry::profileAlgorithm(
    std::string_view id) const noexcept
{
    const auto found = m_profileAlgorithms.find(id);
    return found == m_profileAlgorithms.end() ? nullptr : found->second.get();
}

std::vector<std::string> ToolpathAlgorithmRegistry::pocketAlgorithmIds() const
{
    std::vector<std::string> ids;
    ids.reserve(m_pocketAlgorithms.size());
    for (const auto& [id, algorithm] : m_pocketAlgorithms) {
        (void)algorithm;
        ids.push_back(id);
    }
    return ids;
}

std::vector<std::string> ToolpathAlgorithmRegistry::holeAlgorithmIds() const
{
    std::vector<std::string> ids;
    ids.reserve(m_holeAlgorithms.size());
    for (const auto& [id, algorithm] : m_holeAlgorithms) {
        (void)algorithm;
        ids.push_back(id);
    }
    return ids;
}

std::vector<std::string> ToolpathAlgorithmRegistry::profileAlgorithmIds() const
{
    std::vector<std::string> ids;
    ids.reserve(m_profileAlgorithms.size());
    for (const auto& [id, algorithm] : m_profileAlgorithms) {
        (void)algorithm;
        ids.push_back(id);
    }
    return ids;
}

// 构造并缓存默认注册表(首次调用时惰性初始化):注册内置槽腔与孔算法后
// 以 const 形式共享。返回的对象被所有调用方共享且只读,可安全跨线程使用。
std::shared_ptr<const ToolpathAlgorithmRegistry>
defaultToolpathAlgorithmRegistry()
{
    static const std::shared_ptr<const ToolpathAlgorithmRegistry> registry = [] {
        auto mutableRegistry = std::make_shared<ToolpathAlgorithmRegistry>();
        mutableRegistry->registerPocketAlgorithm(
            makeBuiltinPocketToolpathAlgorithm());
        mutableRegistry->registerHoleAlgorithm(
            makeBuiltinHoleToolpathAlgorithm());
        mutableRegistry->registerProfileAlgorithm(
            makeBuiltinProfileToolpathAlgorithm());
        return std::shared_ptr<const ToolpathAlgorithmRegistry>(
            std::move(mutableRegistry));
    }();
    return registry;
}

} // namespace oap::toolpath
