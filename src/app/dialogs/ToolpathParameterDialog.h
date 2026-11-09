// 刀路参数编辑对话框。组件只编辑传入的值快照，不访问 MainWindow、
// 加工 Job、场景或仿真状态；调用方在确认后自行提交结果。
#pragma once

#include <optional>

#include "app/workflows/project/ToolpathParameterEditorState.h"

class QWidget;

namespace oap::app {

enum class ToolpathParameterEditorPage {
    Hole = 0,
    Pocket,
    Profile,
    Engraving,
    Setup,
    Output,
};

enum class ToolpathParameterEditorScope {
    AllPages,
    InitialPageOnly,
};

// 模态编辑刀路参数;确认返回新状态,取消返回 nullopt。
// 组件只编辑传入的值快照,不访问 MainWindow、加工 Job、场景或仿真状态。
std::optional<ToolpathParameterEditorState> editToolpathParameters(
    QWidget* parent,
    const ToolpathParameterEditorState& initial,
    ToolpathParameterEditorPage initialPage =
        ToolpathParameterEditorPage::Hole,
    ToolpathParameterEditorScope scope =
        ToolpathParameterEditorScope::AllPages);

} // namespace oap::app
