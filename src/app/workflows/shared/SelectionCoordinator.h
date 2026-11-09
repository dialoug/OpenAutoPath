// 停靠面板与 3D 视图共享的选择来源策略。
#pragma once

#include <array>
#include <string>
#include <vector>

namespace oap::app {

// 候选选择来源:分别对应停靠面板中的不同列表。
enum class SelectionSource {
    Analysis,
    Features,
    Groups,
};

// 一份各来源候选 id 的完整快照,由调用方(面板/视图)提供。
struct SelectionSnapshot {
    std::vector<std::string> analysis;
    std::vector<std::string> features;
    std::vector<std::string> groups;
};

// 选择来源协调器:记录最近交互来源,并在各来源间做确定性回退,
// 使面板与 3D 视图保持同一套选择语义。
class SelectionCoordinator {
public:
    void setSource(SelectionSource source) noexcept { m_source = source; }
    SelectionSource source() const noexcept { return m_source; }
    void reset() noexcept { m_source = SelectionSource::Analysis; }

    // 优先采用最近交互过的来源。若其为空,则按稳定的回退顺序
    // Analysis -> Features -> Groups。
    std::vector<std::string> current(
        SelectionSnapshot snapshot) const;

private:
    SelectionSource m_source = SelectionSource::Analysis;
};

} // namespace oap::app
