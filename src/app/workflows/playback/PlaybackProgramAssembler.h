// 模块归属：app/workflows/playback（oap_app_workflow_playback）刀路播放工作流。
// 职责：把 toolpath::ToolpathProgram 展平为播放会话所需的纯值快照
//       （PlaybackMove 序列 + 换刀批次起始索引），可在无 Qt 环境测试。
// 协作：PlaybackSession 持有快照，本类是其唯一写入口；
//       ToolpathPlaybackController/Worker 在后台消费快照生成帧几何。
// ToolpathProgram 到播放会话的纯值适配；不依赖 QWidget 或 AIS。
#pragma once

#include <vector>

#include <Quantity_Color.hxx>

#include "app/workflows/playback/PlaybackSession.h"
#include "toolpath/core/ToolpathProgram.h"

namespace oap::app {

// 播放程序构建选项：通用口袋预览抽稀、换刀批次配色与批次语义。
struct PlaybackProgramBuildOptions {
    bool limitPocketCutMoves = false;
    int previewCutMoveBudget = 50000;
    Quantity_Color pocketRapidColor{
        0.15, 0.45, 1.0, Quantity_TOC_sRGB};
    Quantity_Color holeRapidColor{
        0.0, 0.85, 0.20, Quantity_TOC_sRGB};
    Quantity_Color verticalColor{
        1.0, 0.85, 0.15, Quantity_TOC_sRGB};
};

// 构建结果快照：moves 为展平后的播放移动序列；
// toolBatchStarts 记录每个换刀批次在 moves 中的起始下标。
struct PlaybackProgramSnapshot {
    std::vector<PlaybackMove> moves;
    std::vector<int> toolBatchStarts;
};

// 程序展平器：把 ToolpathProgram 转换为播放会话的扁平移动序列，
// 并按「装夹面/孔腔类型/刀具/工序阶段」切分换刀批次。
class PlaybackProgramAssembler {
public:
    // UI/presentation modules consume semantic mappings through this adapter
    // so toolpath-core knowledge remains inside the workflow boundary.
    static std::vector<toolpath::ToolpathMoveSemantic> mapMoveSemantics(
        const std::vector<toolpath::ToolpathMove>& moves,
        const std::vector<toolpath::ToolpathPrimitive>& primitives);

    // 构建播放快照（纯函数）。options 控制口袋刀路的预览抽稀：
    // 当 Cut 移动总数超出预算时按均匀步长丢弃部分移动，
    // 仅降低显示成本，不改变刀路本身。
    static PlaybackProgramSnapshot build(
        const toolpath::ToolpathProgram& program,
        const PlaybackProgramBuildOptions& options = {});

    // 用新程序整体替换会话内容：清空旧快照后原子写入 moves 与批次。
    static void replaceProgram(
        PlaybackSession& session,
        const toolpath::ToolpathProgram& program,
        const PlaybackProgramBuildOptions& options = {});
};

} // namespace oap::app
