// 仿 FreeCAD CAM 模拟器风格的场景准备工作。
//
// 当前 FreeCAD CAM 模拟器会消费生成的刀路中的每个运动,构建旋转刀具轮廓,
// 并用 OpenGL 深度/模板 CSG 显示毛坯减去刀具扫掠体积后的结果。本文件把与
// 几何无关的准备工作留在 toolpath 层,使其无需创建 OpenGL 上下文即可测试。
//
// 属于 oap_toolpath_pocket 的毛坯仿真层：本场景只驱动「显示用」的 OpenGL
// CSG 模拟，不参与验证与 NC 导出；场景中的显示路径可被简化，但源刀路不变。
#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <TopoDS_Shape.hxx>
#include <gp_Vec.hxx>

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"

namespace oap::toolpath {

// 槽腔 CAM 仿真选项：显示用 CSG 仿真的配置。
struct PocketCamSimulationOptions {
    // Mixed setups replay authoritative IPW prefixes instead of fixed-axis GPU cuts.
    bool authoritativeIpwPlayback = false;
    // FreeCAD 暴露 1..10 的 quality 值。其运动分辨率为
    // maxStockDimension * 0.05 / quality,钳制在 0.5..4.0 mm。
    double quality = 10.0;
    // 可选、仅用于显示的简化容差。默认保留每一个生成的运动,与 FreeCAD
    // 一致。正值启用显式简化;构建器也仅在显示运动的安全预算将要超限时才
    // 进行简化。经过验证和导出的刀路永不被改动。
    double pathSimplificationTolerance = 0.0;
    // true 时，超过显示预算可逐步增大简化容差；审计型播放应设为 false，
    // 使超预算直接报错，而不是生成偏离权威 moves 的显示刀路。
    bool allowAutomaticPathSimplification = true;
    // CSG 开销随每次完成的扫掠而增长。保持仅用于显示的历史记录有界;
    // 简化从不改变验证或 NC 数据。
    std::size_t maxDisplayMotionCount = 100000;
};

// 固定轴平底刀可直接用于 GPU 解析距离场的二维扫掠类型。None 表示该段
// 仍按普通线性 Multi-Dexel 更新处理，例如斜坡、螺旋或语义映射不完整。
enum class PocketCamAnalyticSweepKind {
    None,
    Linear,
    CircularArc,
};

// 槽腔 CAM 解析扫掠：单段运动的解析扫掠面。
struct PocketCamAnalyticSweep {
    PocketCamAnalyticSweepKind kind =
        PocketCamAnalyticSweepKind::None;
    // CircularArc 使用；from/to 继续取所属 PocketCamMotion 的 move。
    gp_Pnt center;
    gp_Vec axis;
    ToolpathArcDirection direction =
        ToolpathArcDirection::Counterclockwise;
    // 当前显示 motion 对应的精确圆弧子段正扫角，而不是整条原语扫角。
    double sweepRadians = 0.0;
    std::size_t sourcePrimitiveIndex = 0u;
    // true 表示规划语义流中只有折线，本字段由显示层在严格残差内从源 Cut moves
    // 恢复得到。它仍然只服务 GPU 距离场，不修改 NC 或权威材料计算。
    bool recoveredFromLinearMoves = false;

    bool valid() const noexcept
    {
        return kind != PocketCamAnalyticSweepKind::None;
    }
};

// 槽腔 CAM 运动：仿真场景中的单段刀具运动。
struct PocketCamMotion {
    gp_Vec outwardToolAxis;
    ToolpathMove move;
    int operationIndex = -1;
    int toolIndex = -1;
    // 本显示 motion 完成时，对应 operation.clearing.moves 的已完成前缀。
    // 即使显示折线经过简化，也能据此请求完全一致的 Multi-Dexel 临时帧。
    std::size_t sourceMoveEndCount = 0u;
    bool startsChain = false;
    std::size_t firstSimulationStep = 0;
    int simulationStepCount = 1;
    // 与源 moves 前缀严格对应的解析扫掠元数据。它只影响 GPU 显示距离场，
    // 权威材料、验证和 NC 仍由原始 moves/primitives 决定。
    PocketCamAnalyticSweep analyticSweep;
    // 仅真实语义原语且逐段几何与权威 fallback moves 一致时为 true。
    // 折线猜测恢复或校验失败的圆弧必须保持线性播放。
    bool analyticPlaybackTrusted = false;

    // 播放、刀具显示和增量去料共同使用此插值；可信语义圆弧按解析几何
    // 插值，其余情况严格回退到 controller-independent move 线段。
    gp_Pnt playbackPointAt(double fraction) const;
};

// 槽腔 CAM 刀具：仿真用旋转刀具轮廓。
struct PocketCamTool {
    int toolNumber = 0;
    std::string toolId;
    std::string toolName;
    // 侧面轮廓按从顶部外边到刀尖中心排列。每对为
    // (半径, 距刀尖的轴向距离)，单位为毫米。
    std::vector<std::pair<double, double>> profile;
    double diameter = 0.0;
    double length = 0.0;
    bool flatEnd = false;
};

// 槽腔 CAM 仿真场景：驱动显示用 OpenGL CSG 模拟。
struct PocketCamSimulationScene {
    bool authoritativeIpwPlayback = false;
    gp_Vec outwardToolAxis;
    std::vector<PocketCamTool> tools;
    std::vector<PocketCamMotion> motions;
    double quality = 10.0;
    double motionResolution = 0.5;
    double maximumStockDimension = 0.0;
    std::size_t sourceMotionCount = 0;
    std::size_t totalSimulationSteps = 0;
    std::size_t analyticLinearMotionCount = 0;
    std::size_t analyticArcMotionCount = 0;
    std::size_t trustedAnalyticArcMotionCount = 0;
    std::size_t recoveredAnalyticArcMotionCount = 0;
    std::size_t analyticFallbackMotionCount = 0;
    bool pathSimplified = false;
};

// 槽腔 CAM 仿真场景结果：场景构建产物与统计。
struct PocketCamSimulationSceneResult {
    PocketCamSimulationScene scene;
    std::vector<std::string> warnings;
    std::string error;

    bool ok() const noexcept { return error.empty(); }
};

// 一次构建输出：场景 + 警告/错误。ok() 为 false 时 scene 可能仍部分填充，
// 调用方必须优先检查 error 而非 scene 内容。
class PocketCamSimulationSceneBuilder {
public:
    // 输入 stock 毛坯、一系列槽腔工序与批次参数，输出可直接驱动 OpenGL
    // 逐运动模拟的 scene。前置条件：operations 非空、所有工序刀轴一致、
    // 刀具直径有效；违反时返回带 error 的结果。
    PocketCamSimulationSceneResult build(
        const TopoDS_Shape& stock,
        const std::vector<PocketToolpathOperation>& operations,
        const ToolpathParams& params,
        const PocketCamSimulationOptions& options = {}) const;
};

} // namespace oap::toolpath
