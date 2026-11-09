// 机床方言无关的有序刀路原语。moves 承担预览和线性安全校验；primitives
// 保留圆弧、螺旋与固定循环语义，并为曲线连续扫掠提供原始几何。后处理
// 不支持原生原语时使用 fallbackMoves 展开。
#pragma once

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "ToolpathMoves.h"

namespace oap::toolpath {

enum class ToolpathPrimitiveType {
    Linear,
    CircularArc,
    DrillCycle,
};

enum class ToolpathArcDirection {
    Clockwise,
    Counterclockwise,
};

enum class ToolpathDrillCycleType {
    Drilling,   // 普通钻削，通常映射为 G81。
    Dwell,      // 孔底停留，通常映射为 G82。
    Peck,       // 深孔啄钻，通常映射为 G83。
    ChipBreak,  // 断屑钻削，通常映射为 G73。
    Tapping,    // 攻丝，通常映射为 G84。
};

enum class ToolpathDrillRetractMode {
    InitialPlane, // G98：返回循环开始前的清空面。
    RetractPlane, // G99：返回 R 平面。
};

enum class ToolpathDrillCycleSource {
    None,
    AutomaticDepthDiameterRatio,
    ExplicitSetting,
};

// 圆弧移动原语：移动类型、方向、起止点与圆心。
struct ToolpathCircularArc {
    ToolpathMoveType moveType = ToolpathMoveType::Cut;
    ToolpathArcDirection direction =
        ToolpathArcDirection::Counterclockwise;
    gp_Pnt from;
    gp_Pnt to;
    gp_Pnt center;
    gp_Vec axis;               // 圆弧正法向；方向语义相对此轴定义。
    double sweepRadians = 0.0; // 正扫角，允许完整 2*pi 圆和带 Z 变化的螺旋圆弧。
    double feed = 0.0;
};

// 钻削循环原语：类型、回退模式与清空/回退/进入点。
struct ToolpathDrillCycle {
    ToolpathDrillCycleType type = ToolpathDrillCycleType::Drilling;
    ToolpathDrillRetractMode retractMode =
        ToolpathDrillRetractMode::InitialPlane;
    gp_Pnt clearance; // 循环开始前和 G98 完成后的刀尖位置。
    gp_Pnt retract;   // R 平面位置。
    gp_Pnt entry;     // 工件表面的名义孔入口。
    gp_Pnt bottom;    // 含可选钻尖补偿的实际加工终点。
    gp_Vec axis;      // 从入口指向孔底的进刀轴。
    ToolpathDrillCycleSource source = ToolpathDrillCycleSource::None;
    double nominalDepth = 0.0;
    double depthDiameterRatio = 0.0;
    double peckDepth = 0.0;
    double chipBreakRetract = 0.0;
    double reentryClearance = 0.0;
    double dwellSeconds = 0.0;
    double feed = 0.0;
};

// 原语上下文：环序号、深度层、环号与移动意图。
struct ToolpathPrimitiveContext {
    int loopSequence = -1;
    int depthIndex = 0;
    int ring = 0;
    ToolpathMoveIntent intent = ToolpathMoveIntent::General;
};

// 统一刀路原语：线性/圆弧/钻削循环及回退 move。
struct ToolpathPrimitive {
    ToolpathPrimitiveType type = ToolpathPrimitiveType::Linear;
    ToolpathPrimitiveContext context;
    ToolpathMove linear;
    ToolpathCircularArc arc;
    ToolpathDrillCycle drillCycle;
    // 原生圆弧/固定循环不可用时的控制器无关展开结果。
    // 非空时后处理按其展开；空则按 type 对应的原生字段直接输出，
    // 线性原语无 fallback 时即对应 linear 本身。
    std::vector<ToolpathMove> fallbackMoves;
};

// 便捷构造函数：把 move 的元数据复制进 context，并保存可选 fallback。
inline ToolpathPrimitive makeLinearPrimitive(
    const ToolpathMove& move,
    std::vector<ToolpathMove> fallbackMoves = {})
{
    ToolpathPrimitive primitive;
    primitive.type = ToolpathPrimitiveType::Linear;
    primitive.context.loopSequence = move.loopSequence;
    primitive.context.depthIndex = move.depthIndex;
    primitive.context.ring = move.ring;
    primitive.context.intent = move.intent;
    primitive.linear = move;
    primitive.fallbackMoves = std::move(fallbackMoves);
    return primitive;
}

// 便捷构造函数：把弧线几何与展开结果包装为 CircularArc 原语。
inline ToolpathPrimitive makeArcPrimitive(
    const ToolpathCircularArc& arc,
    std::vector<ToolpathMove> fallbackMoves,
    ToolpathPrimitiveContext context = {})
{
    ToolpathPrimitive primitive;
    primitive.type = ToolpathPrimitiveType::CircularArc;
    primitive.context = context;
    primitive.arc = arc;
    primitive.fallbackMoves = std::move(fallbackMoves);
    return primitive;
}

// 便捷构造函数：把固定循环与展开结果包装为 DrillCycle 原语。
inline ToolpathPrimitive makeDrillCyclePrimitive(
    const ToolpathDrillCycle& cycle,
    std::vector<ToolpathMove> fallbackMoves,
    ToolpathPrimitiveContext context = {})
{
    ToolpathPrimitive primitive;
    primitive.type = ToolpathPrimitiveType::DrillCycle;
    primitive.context = context;
    primitive.drillCycle = cycle;
    primitive.fallbackMoves = std::move(fallbackMoves);
    return primitive;
}

// 把 moves 的 [begin, end) 区间逐条追加为线性原语；
// 区间越界时自动夹紧到合法范围，begin>end 则追加为空。
inline void appendLinearPrimitives(
    std::vector<ToolpathPrimitive>& destination,
    const std::vector<ToolpathMove>& moves,
    std::size_t begin,
    std::size_t end)
{
    end = std::min(end, moves.size());
    for (std::size_t index = std::min(begin, end); index < end; ++index)
        destination.push_back(makeLinearPrimitive(moves[index]));
}

} // namespace oap::toolpath
