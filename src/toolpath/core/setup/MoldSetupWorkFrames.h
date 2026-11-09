// 六个模具装夹面的独立工件坐标系及人工确认状态。
#pragma once

#include <array>
#include <string>

#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include "analysis/Model.h"
#include "analysis/MoldCoordinateFrame.h"
#include "toolpath/core/MachiningSetup.h"

namespace oap::toolpath {

// 单个模具装夹面的工件坐标系、世界→工件变换与人工确认状态。
// 由 MoldSetupWorkFrames::fromMold 构建；confirmed 表示操作者已核对
// 机床坐标原点与轴映射，导出前必须为 true。
struct MoldFaceSetupWorkFrame {
    MoldSetupFace face = MoldSetupFace::Unresolved;
    gp_Ax3 setupWorkFrame;
    gp_Trsf worldToSetupTransform;
    WorkCoordinateSystem workOffset = WorkCoordinateSystem::None;
    bool valid = false;
    bool confirmed = false;
    bool requiresPhysicalReclamp = false;
    std::string axisMapping;
    std::string operatorInstruction;
    // 导出当前面为程序级 MachiningSetup（含人工确认标记）。
    MachiningSetup machiningSetup() const;
};

// 六个装夹面工件坐标系的容器。由 fromMold 一次性构建；forFace 提供
// 读写访问，setConfirmed 记录操作者确认，isValid 表示六面全部建立成功。
class MoldSetupWorkFrames {
public:
    static MoldSetupWorkFrames fromMold(
        const Bbox& worldBounds,
        const MoldCoordinateFrame& moldFrame);

    // 按面取工件坐标系；face 非法时返回 nullptr。
    const MoldFaceSetupWorkFrame* forFace(MoldSetupFace face) const;
    // 按面取可修改的工件坐标系；face 非法时返回 nullptr。
    MoldFaceSetupWorkFrame* forFace(MoldSetupFace face);
    // 设置某面的人工确认状态；面无效或该面构建失败时返回 false。
    bool setConfirmed(MoldSetupFace face, bool confirmed);
    bool isValid() const noexcept { return m_valid; }

private:
    std::array<MoldFaceSetupWorkFrame, 6> m_faces;
    bool m_valid = false;
};

} // namespace oap::toolpath
