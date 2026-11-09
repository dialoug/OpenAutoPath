// HoleMachining —— 将识别出的轴向孔转换为可执行移动列表。
// 属于 oap_toolpath_hole 模块核心入口；依赖 detail/ 下的快速移动规划、
// 访问顺序优化与三种孔策略生成器，输出孔工序集合及统计/诊断。
// 策略选择和路径采样属于纯几何操作；提供障碍环境后，孔间连接在加入刀路前
// 会针对完整模型执行检查。
#pragma once

#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "analysis/MoldCoordinateFrame.h"
#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/core/collision/CollisionScene.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathMoves.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathPrimitives.h"
#include "toolpath/core/ToolpathToolContext.h"
#include "toolpath/hole/HoleMachiningInput.h"

namespace oap::toolpath {

enum class HoleMachiningStrategy {
    Auto,                // 根据刀具与孔的直径比自动选择。
    SmallDrill,          // 钻头/U 钻沿轴向中心线执行固定循环；名称保留 JSON 兼容。
    MediumHelical,       // 螺旋下刀后在底部执行一圈精加工。
    LargeLayeredSpiral,  // 在每个深度层重复平面径向螺旋。
};

// 一个可独立执行的孔加工操作。entry/bottom 保留识别特征几何；moves 是
// 世界坐标下有序的机床移动，sourceHoleIndex 用于映射回输入向量。
struct HoleMachiningOperation {
    std::string operationId;
    std::string algorithmId;
    int sourceHoleIndex = -1;
    // 一条合并刀路可能实际覆盖多个同轴同径识别孔段。为空时为兼容
    // 手动/旧调用方，覆盖审计仍只使用 sourceHoleIndex。
    std::vector<int> coveredSourceHoleIndices;
    FeatureKind kind = FeatureKind::BlindHole;
    HoleMachiningStrategy strategy = HoleMachiningStrategy::SmallDrill;
    HoleMachiningPhase phase = HoleMachiningPhase::Standard;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    ToolpathToolContext tool;
    int setupIndex = 0;
    gp_Pnt entry;
    gp_Pnt bottom;              // CAD 名义底点：完整刀径应到达的圆柱终止平面。
    gp_Pnt commandedBottom;     // 实际刀尖目标；补偿后可深于 bottom。
    bool hasCommandedBottom = false;
    double drillTipLength = 0.0;
    double breakthroughDepth = 0.0;
    // 由钻削总深度或孔铣单层轴向吃刀量确定的最小有效刃长。
    // 该值是工序需求，不会反写或伪造刀库中的实际刀具几何。
    double minimumCuttingEdgeLength = 0.0;
    // 刀尖到工件入口面的最小轴向可达长度；独立于分层切削所需刃长。
    double minimumToolReach = 0.0;
    double radius = 0.0;
    std::vector<ToolpathMove> moves;
    std::vector<ToolpathPrimitive> primitives;
    // true 表示该 Operation 属于独立局部批次，不参与后续全量批次归并。
};

// 孔快速规划上下文：障碍形状、碰撞场景、模架帧与容差。
struct HoleRapidPlanningContext {
    TopoDS_Shape obstacleShape;       // 作为保守障碍物使用的未加工模型。
    // 新接口优先于 obstacleShape；旧调用方仍会把 obstacleShape 视为 workpiece。
    CollisionScene collisionScene;
    const MoldCoordinateFrame* moldFrame = nullptr;
    double collisionTolerance = 1.0e-4; // 加到刀具半径阈值上的碰撞公差，单位 mm。
    double axisToleranceDeg = 1.0;    // 同一装夹中允许的最大轴向夹角。
};

// 孔加工结果：工序列表、诊断、警告与孔大小统计。
struct HoleMachiningResult {
    std::vector<HoleMachiningOperation> operations;
    // diagnostics 是稳定的机器可读结果；warnings 为兼容旧 UI/日志保留。
    std::vector<ToolpathDiagnostic> diagnostics;
    std::vector<std::string> warnings;
    int smallCount = 0;
    int mediumCount = 0;
    int largeCount = 0;
    int skippedCount = 0;
    int setupCount = 0;
    int directRapidLinkCount = 0;
    int liftedRapidLinkCount = 0;
    int unsafeRapidLinkCount = 0;
    int broadPhaseSafeSegmentCount = 0;
    int exactCollisionCheckCount = 0;
    int solidSweepCheckCount = 0;
    int solidSweepConstructedPartCount = 0;
    int nonCuttingMoveCollisionCount = 0;
    int cuttingMoveCollisionCount = 0;
    int arcSweepPrimitiveCount = 0;
    int twoOptImprovementCount = 0;
    int relocationImprovementCount = 0;
    int threeOptImprovementCount = 0;
    int rapidRepairRelocationCount = 0;
    int rapidRepairRecoveredLinkCount = 0;
    double nearestNeighborRouteLength = 0.0;
    double twoOptRouteLength = 0.0;
    double relocationRouteLength = 0.0;
    double optimizedRouteLength = 0.0;
    bool cancelled = false;
};

void addHoleDiagnostic(HoleMachiningResult& result,
                       ToolpathDiagnosticCode code,
                       ToolpathDiagnosticSeverity severity,
                       std::string message,
                       int sourceIndex = -1,
                       std::string operationId = {});

// 返回工艺策略所需的最小有效刃长：钻削按孔深，螺旋/分层铣按轴向吃刀量。
double holeMinimumCuttingEdgeLength(
    const HoleMachiningInput& hole,
    HoleMachiningStrategy strategy,
    const ToolpathParams& params) noexcept;

// 返回刀尖到孔工序最深指令点所需的最小总可达长度；钻削包含钻尖与通孔突破。
double holeMinimumToolReach(
    const HoleMachiningInput& hole,
    HoleMachiningStrategy strategy,
    const ToolpathParams& params) noexcept;

// 孔加工内核：把孔输入生成孔加工工序与刀路。
class HoleMachining {
public:
    // 无效孔和不安全的孔间连接会被分别跳过并写入 warnings；
    // 单个异常特征不会使此前输出失效。
    // strategyOverrides 与 holes 使用相同索引，其长度可以小于 holes。
    HoleMachiningResult generate(const std::vector<HoleMachiningInput>& holes,
                                 const ToolpathParams& params,
                                 const HoleRapidPlanningContext* rapidContext = nullptr,
                                 const std::vector<HoleMachiningStrategy>* strategyOverrides = nullptr,
                                 const ToolpathGenerationControl* control = nullptr) const;
};

const char* holeMachiningStrategyZh(HoleMachiningStrategy strategy);

} // namespace oap::toolpath
