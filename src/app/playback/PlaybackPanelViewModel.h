// 纯播放到面板的投影;不包含 QWidget 或 OCCT 呈现所有权。
// 职责:把播放会话的移动/刀具批次信息投影为面板可直接展示的文本
//   与导航使能状态,保持纯函数式(无 UI 依赖)。
// 模块归属:src/app(playback 模块);由播放面板 ViewModel 需求驱动,
//   输入取自播放会话,输出供 PlaybackPanel 绑定。
#pragma once

#include <vector>

#include <QString>

#include "app/workflows/playback/PlaybackSession.h"

namespace oap::app {

// 面板投影的输入:播放移动与刀具批次起点(可空)、当前步号/进度比例。
struct PlaybackPanelViewModelInput {
    const std::vector<PlaybackMove>* moves = nullptr;
    const std::vector<int>* toolBatchStarts = nullptr;
    int index = 0;
    double fraction = 1.0;
};

// 面板展示模型:信息文本、阶段/刀具文本、信息与导航步号、
// 上/下一刀具使能状态。
struct PlaybackPanelViewModel {
    QString infoText;
    QString phaseText = QStringLiteral("等待开始");
    QString toolText = QStringLiteral("—");
    int navigationMoveIndex = -1;
    int informationMoveIndex = -1;
    bool previousToolEnabled = false;
    bool nextToolEnabled = false;
};

// 面板模型构建器:把输入投影为面板展示模型。
class PlaybackPanelViewModelBuilder {
public:
    static PlaybackPanelViewModel build(
        const PlaybackPanelViewModelInput& input);
};

} // namespace oap::app
