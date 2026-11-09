// SelectionCoordinator 实现（oap_app_workflow_base）。
// 停靠面板与 3D 视图共享的选择来源策略，见 SelectionCoordinator.h。
#include "SelectionCoordinator.h"

#include <utility>

namespace oap::app {
namespace {

// 从快照中取出指定来源的候选列表。
std::vector<std::string>& values(
    SelectionSnapshot& snapshot,
    SelectionSource source)
{
    switch (source) {
        case SelectionSource::Features: return snapshot.features;
        case SelectionSource::Groups: return snapshot.groups;
        case SelectionSource::Analysis: return snapshot.analysis;
    }
    return snapshot.analysis;
}

} // namespace

// 快照按值传入,因此返回值可以安全地 move 走候选列表;
// 回退时跳过首选来源避免重复,全部为空才返回空列表。
std::vector<std::string> SelectionCoordinator::current(
    SelectionSnapshot snapshot) const
{
    std::vector<std::string>& preferred = values(snapshot, m_source);
    if (!preferred.empty())
        return std::move(preferred);

    constexpr std::array<SelectionSource, 3> fallbackOrder{
        SelectionSource::Analysis,
        SelectionSource::Features,
        SelectionSource::Groups};
    for (const SelectionSource source : fallbackOrder) {
        if (source == m_source)
            continue;
        std::vector<std::string>& candidate = values(snapshot, source);
        if (!candidate.empty())
            return std::move(candidate);
    }
    return {};
}

} // namespace oap::app
