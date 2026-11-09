// PlaybackCameraFollower 实现（oap_app_playback）。
// 刀路播放的相机跟随策略适配器，见 PlaybackCameraFollower.h。
#include "app/playback/PlaybackCameraFollower.h"

#include <gp_Vec.hxx>

namespace oap::app {

// 重定中心:保持观察方向不变,把 eye 与 at 整体平移 to 使新目标点
// 落到视图中心。
PlaybackCameraPose PlaybackCameraFollower::recenter(
    const PlaybackCameraPose& pose,
    const gp_Pnt& target) noexcept
{
    const gp_Vec translation(pose.at, target);
    return {pose.eye.Translated(translation), target};
}

// 跟随:未启用或视图无效时不动作;否则读取当前相机位姿,
// 以 target 重定中心后写回视图。
bool PlaybackCameraFollower::follow(
    const Handle(V3d_View)& view,
    const gp_Pnt& target,
    bool enabled)
{
    if (!enabled || view.IsNull())
        return false;
    Standard_Real eyeX, eyeY, eyeZ, atX, atY, atZ;
    view->Eye(eyeX, eyeY, eyeZ);
    view->At(atX, atY, atZ);
    const PlaybackCameraPose centered = recenter(
        {gp_Pnt(eyeX, eyeY, eyeZ), gp_Pnt(atX, atY, atZ)}, target);
    view->SetEye(
        centered.eye.X(), centered.eye.Y(), centered.eye.Z());
    view->SetAt(centered.at.X(), centered.at.Y(), centered.at.Z());
    return true;
}

} // namespace oap::app
