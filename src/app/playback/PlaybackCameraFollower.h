// 刀路播放的相机跟随策略适配器。
// 职责:提供纯函数的重定中心位姿计算与对 V3d_View 的相机跟随动作,
//   供播放协调器在播放时把视线保持在刀具尖端附近。
// 模块归属:src/app(playback 模块),无状态、无所有权,仅静态方法。
#pragma once

#include <V3d_View.hxx>
#include <gp_Pnt.hxx>

namespace oap::app {

// 相机位姿:观察点 eye 与目标点 at。
struct PlaybackCameraPose {
    gp_Pnt eye;
    gp_Pnt at;
};

// 相机跟随器:纯静态工具,不做状态存储。
class PlaybackCameraFollower {
public:
    // 以 target 为新目标点重定中心:整个位姿按 at→target 平移。
    static PlaybackCameraPose recenter(
        const PlaybackCameraPose& pose,
        const gp_Pnt& target) noexcept;
    // 若启用且视图有效,把视图相机重定中心到 target,返回是否执行。
    static bool follow(
        const Handle(V3d_View)& view,
        const gp_Pnt& target,
        bool enabled);
};

} // namespace oap::app
