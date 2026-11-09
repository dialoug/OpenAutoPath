// 加工装夹与程序级设置。刀路几何仍保留世界坐标，后处理阶段才转换到工件
// 坐标，避免显示、验证和导出各自维护不同版本的点。
#pragma once

#include <optional>
#include <string>

#include <gp_Ax3.hxx>
#include <gp_Trsf.hxx>

#include "analysis/FeatureTypes.h"

namespace oap {
struct MoldCoordinateFrame;
}

namespace oap::toolpath {

// 工件坐标系偏置：None 表示未指定，G54～G59 对应该面装夹的 G 代码偏置号。
enum class WorkCoordinateSystem {
    None,
    G54,
    G55,
    G56,
    G57,
    G58,
    G59,
};

// 一次装夹的完整程序级设置：工件坐标系、偏置号与操作者确认状态。
// 由 makeMoldFaceMachiningSetup 或 MoldFaceSetupWorkFrame::machiningSetup
// 构建，值语义复制，供生成、验证与后处理共享同一份世界坐标。
struct MachiningSetup {
    gp_Ax3 workFrame; // 原点和 XYZ 方向定义世界坐标到工件坐标的变换。
    WorkCoordinateSystem workOffset = WorkCoordinateSystem::G54;
    // 孔程序导出时按 setupFace 从模具六面自动建立右手坐标系；每个面仍
    // 单独输出，操作者必须在翻面后把对应外表面左下角设为该坐标原点。
    bool useMoldFaceWorkFrame = true;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    bool requiresManualConfirmation = false;
    bool workFrameConfirmed = true;
    bool requiresPhysicalReclamp = false;
    bool hasWorldToWorkTransform = false;
    gp_Trsf worldToWorkTransform;
    std::string axisMapping;
    std::string operatorInstruction;
};

// path-NC 分支只生成仓库样例格式的参考 NC。机床档案、G53
// 换刀位置、G43/H 快照和控制器限位等机床专用参数不属于本分支。
struct GCodeProgramOptions {
    std::string programName = "OpenAutoPath pocket clearing";
    // 清空高度、R 平面和 G98/G99 直接采用碰撞规划后的循环原语；
    // 日期、精度、G21、M07/M09 和尾注采用固定模板规则。
    std::string toolId;        // 来自刀库的稳定 ID；为空表示手动参数。
    std::string toolName;      // 用于 G-code 头注释和追溯。
    bool coolant = false;      // true 时输出参考模板的 M07/M09。
};

const char* workCoordinateCode(WorkCoordinateSystem system) noexcept;

// 从模具坐标系与指定加工面构建确定性工件坐标系：以该面在装夹外侧的
// 二维左下角为原点，保证面内坐标非负。模具或面无效时返回 nullopt。
std::optional<MachiningSetup> makeMoldFaceMachiningSetup(
    const oap::MoldCoordinateFrame& moldFrame,
    MoldSetupFace face,
    WorkCoordinateSystem workOffset = WorkCoordinateSystem::G54);

} // namespace oap::toolpath
