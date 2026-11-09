// PlaybackPanelViewModel 实现（oap_app_playback）。
// 纯播放到面板的投影，不含 QWidget 或 OCCT 呈现所有权，见 PlaybackPanelViewModel.h。
#include "app/playback/PlaybackPanelViewModel.h"

#include <algorithm>

#include "tooling/library/ToolLibrary.h"

namespace oap::app {
namespace {

// 是否为孔类特征。
bool isHoleKind(FeatureKind kind)
{
    return kind == FeatureKind::ThroughHole ||
        kind == FeatureKind::BlindHole ||
        kind == FeatureKind::CountersinkHole;
}

// 特征类型 → 中文显示名。
QString featureKindText(FeatureKind kind)
{
    switch (kind) {
        case FeatureKind::ThroughHole: return QStringLiteral("通孔");
        case FeatureKind::BlindHole: return QStringLiteral("盲孔");
        case FeatureKind::CountersinkHole: return QStringLiteral("沉孔");
        case FeatureKind::Slot: return QStringLiteral("槽");
        case FeatureKind::Pocket: return QStringLiteral("腔");
    }
    return QStringLiteral("特征");
}

// 加工阶段 → 中文阶段名(槽/腔在标准阶段用通用名)。
QString phaseText(toolpath::HoleMachiningPhase phase, FeatureKind kind)
{
    switch (phase) {
        case toolpath::HoleMachiningPhase::CenterSpot:
            return QStringLiteral("定位阶段");
        case toolpath::HoleMachiningPhase::Drill:
            return QStringLiteral("钻孔阶段");
        case toolpath::HoleMachiningPhase::FlatEndMill:
            return QStringLiteral("平底刀阶段");
        case toolpath::HoleMachiningPhase::Standard:
            return isHoleKind(kind) ? QStringLiteral("单刀孔加工阶段")
                                    : QStringLiteral("槽/腔加工阶段");
    }
    return QStringLiteral("加工阶段");
}

// 移动类型 → 中文显示名。
QString moveTypeText(toolpath::ToolpathMoveType type)
{
    switch (type) {
        case toolpath::ToolpathMoveType::Rapid: return QStringLiteral("快速");
        case toolpath::ToolpathMoveType::Plunge: return QStringLiteral("进刀");
        case toolpath::ToolpathMoveType::Cut: return QStringLiteral("切削");
        case toolpath::ToolpathMoveType::Retract: return QStringLiteral("退刀");
    }
    return QStringLiteral("移动");
}

// 刀具显示文本:优先展示批次刀具(刀号·名称·直径·类型);
// 无有效快照时明确显示“未绑定刀具”，不推测刀具几何。
QString toolText(const PlaybackMove& move)
{
    if (move.tool.valid) {
        QString identity = QString::fromStdString(move.tool.toolName);
        if (identity.isEmpty())
            identity = QString::fromStdString(move.tool.toolId);
        QString text = QStringLiteral("T%1").arg(move.tool.toolNumber);
        if (!identity.isEmpty())
            text += QStringLiteral(" · %1").arg(identity);
        text += QStringLiteral(" · Ø%1 mm").arg(
            move.tool.diameter, 0, 'f', 3);
        const tooling::ToolType type = tooling::toolTypeFromId(
            QString::fromStdString(move.tool.toolType));
        return text + QStringLiteral(" · %1").arg(
            tooling::toolTypeDisplayName(type));
    }

    return QStringLiteral("未绑定刀具");
}

} // namespace

// 投影逻辑:先由步号/比例确定“信息步号”(进行中显示当前步,否则
// 显示上一步;刀具批次边界精确落在当前步时信息步号取该步),
// 再计算导航步号与上/下一刀具使能,最后填充阶段/刀具/步进等文本。
PlaybackPanelViewModel PlaybackPanelViewModelBuilder::build(
    const PlaybackPanelViewModelInput& input)
{
    PlaybackPanelViewModel model;
    const std::vector<PlaybackMove> emptyMoves;
    const std::vector<int> emptyStarts;
    const auto& moves = input.moves ? *input.moves : emptyMoves;
    const auto& starts = input.toolBatchStarts
        ? *input.toolBatchStarts : emptyStarts;
    const int total = static_cast<int>(moves.size());
    const int completed = std::clamp(input.index, 0, total);
    const bool inProgress = input.fraction < 1.0 &&
        input.index >= 0 && input.index < total;
    const int displayStep = inProgress ? completed + 1 : completed;

    if (inProgress)
        model.informationMoveIndex = input.index;
    else if (completed > 0)
        model.informationMoveIndex = completed - 1;

    // 精确的刀具批次边界描述的是即将运行的刀具。
    if (!inProgress && completed < total) {
        const int batch = moves[completed].toolBatchIndex;
        if (batch >= 0 && batch < static_cast<int>(starts.size()) &&
            starts[static_cast<std::size_t>(batch)] == completed) {
            model.informationMoveIndex = completed;
        }
    }

    model.navigationMoveIndex = completed < total
        ? completed : (total > 0 ? total - 1 : -1);
    if (model.navigationMoveIndex >= 0 && !starts.empty()) {
        const int batch = moves[model.navigationMoveIndex].toolBatchIndex;
        if (batch >= 0 && batch < static_cast<int>(starts.size())) {
            model.previousToolEnabled = batch > 0;
            model.nextToolEnabled = batch + 1 < static_cast<int>(starts.size());
        }
    }

    model.infoText = QStringLiteral("第 %1/%2 步")
        .arg(displayStep).arg(total);
    if (model.informationMoveIndex < 0)
        return model;

    const PlaybackMove& move = moves[model.informationMoveIndex];
    model.phaseText = phaseText(move.phase, move.kind);
    if (move.setupFace != MoldSetupFace::Unresolved) {
        model.phaseText += QStringLiteral(" · 装夹面 %1").arg(
            QString::fromLatin1(moldSetupFaceName(move.setupFace)));
    }
    model.toolText = toolText(move);

    model.infoText += QStringLiteral(" · %1").arg(
        moveTypeText(move.move.type));
    if (move.move.type == toolpath::ToolpathMoveType::Cut) {
        const QString label = isHoleKind(move.kind)
            ? QString::fromUtf8(
                  toolpath::holeMachiningStrategyZh(move.holeStrategy))
            : featureKindText(move.kind);
        model.infoText += QStringLiteral("(%1)").arg(label);
    }
    if (move.setupIndex >= 0 && move.setupCount > 0) {
        model.infoText += QStringLiteral(" · 装夹 %1/%2")
            .arg(move.setupIndex + 1).arg(move.setupCount);
    }
    model.infoText += QStringLiteral(" · 深度层 %1").arg(
        move.move.depthIndex);
    return model;
}

} // namespace oap::app
