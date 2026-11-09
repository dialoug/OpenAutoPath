// 孔策略共享的坐标、采样和安全连接工具。具体钻削策略不得在此实现。
// 属于 oap_toolpath_hole 模块 detail/strategies 私有部分；只被三种孔策略
// 生成器共享，不构成稳定对外接口。
#pragma once

#include <string>
#include <vector>

#include <gp_Pnt.hxx>

#include "toolpath/hole/detail/HoleStrategyGenerators.h"

namespace oap::toolpath::detail::strategy {

inline constexpr double kTwoPi = 6.28318530717958647692;

// 组装一个 ToolpathMove 并追加到 moves；零长度段直接忽略。
void appendMove(std::vector<ToolpathMove>& moves,
                ToolpathMoveType type,
                const gp_Pnt& from,
                const gp_Pnt& to,
                int operationIndex,
                int depthIndex,
                double feed,
                ToolpathMoveIntent intent = ToolpathMoveIntent::General);

// 在孔局部坐标系中求点：轴向深度 depth、径向 radius、转角 theta（弧度）。
gp_Pnt pointAt(const HoleFrame& frame,
               double axialDepth,
               double radius,
               double theta);

double safeOffset(const ToolpathParams& params);          // 安全高度（非负）。
double effectivePitch(const HoleFrame& frame,            // 螺旋策略有效螺距。
                      const ToolpathParams& params);
double effectiveLayerDepth(const HoleFrame& frame,       // 分层策略有效层深。
                           const ToolpathParams& params);
double effectiveRadialStep(const ToolpathParams& params);// 分层径向步距。
int pitchDepthIndex(double depth, double pitch);          // 深度对应的循环深度序号。

// 在点数预算内拟合角度步长；放不下时返回 false 并给出中文警告。
bool fitSampleStep(double thetaSpan,
                   const ToolpathParams& params,
                   double& dTheta,
                   std::string& warning);

// 工序安全高度/清空高度的有效值（计入避碰规划器最小余量）。
double effectiveClearanceOffset(const ToolpathParams& params,
                                const HoleRapidPlanner* planner);
double effectiveCenteringOffset(const ToolpathParams& params,
                                const HoleRapidPlanner* planner);

// 追加孔间安全连接：有规划器时走碰撞检查，无规划器时退化为直连快移。
bool addClearanceLink(HoleMachiningOperation& operation,
                      const gp_Pnt& from,
                      const gp_Pnt& to,
                      int operationIndex,
                      const ToolpathParams& params,
                      HoleRapidPlanner* planner);

} // namespace oap::toolpath::detail::strategy
