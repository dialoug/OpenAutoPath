// 刀路播放期间各静态覆盖层的统一可见性策略。
#pragma once

namespace oap::app::presentation {

// 静态覆盖层的可见性开关集,由协调器据此调 OverlayController::setVisible。
struct PlaybackStaticOverlayVisibility {
    bool fullToolpath = false;      // 完整刀路预览组
    bool currentToolpath = false;   // 当前刀路段预览组
    bool holeStageHighlight = false; // 孔阶段壁面/路径高亮组
};

// 计算播放各阶段静态覆盖层的可见性。
// hasMoves:是否存在可播放刀路;completedMoveCount:已完成的移动数;
// currentMoveFraction:当前移动的进度(0~1,起点处为 1.0);
// currentToolpathOnly:仅显示当前刀路段模式。
// 返回:仅在“尚未开始播放”时(无移动,或还在第一个移动上)显示静态层,
// 播放开始后这些层被隐藏以免遮挡动态剩余路径。
inline PlaybackStaticOverlayVisibility playbackStaticOverlayVisibility(
    bool hasMoves,
    int completedMoveCount,
    double currentMoveFraction,
    bool currentToolpathOnly) noexcept
{
    const bool atUnplayedStart = !hasMoves ||
        (completedMoveCount <= 0 && currentMoveFraction >= 1.0);
    return {
        atUnplayedStart && !currentToolpathOnly,
        atUnplayedStart && currentToolpathOnly,
        atUnplayedStart};
}

} // namespace oap::app::presentation
