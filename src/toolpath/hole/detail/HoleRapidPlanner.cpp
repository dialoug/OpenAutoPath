// 孔间快速移动规划：先做包围盒粗筛，再做 OCCT 精确距离检查并尝试包络外抬高。
// 属于 oap_toolpath_hole 模块 detail 私有实现，由 HoleMachining 创建使用。
#include "toolpath/hole/detail/HoleRapidPlanner.h"
#include "toolpath/hole/HoleToolpathValidator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <utility>

#include <Bnd_Box.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>

namespace oap::toolpath::detail {

namespace {
constexpr double kHoleTolerance = 1.0e-7;

std::uint64_t doubleBits(double value)
{
    if (value == 0.0)
        value = 0.0; // 规范化负零。
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

bool pointLess(const gp_Pnt& a, const gp_Pnt& b)
{
    if (a.X() != b.X()) return a.X() < b.X();
    if (a.Y() != b.Y()) return a.Y() < b.Y();
    return a.Z() < b.Z();
}

bool positiveFinite(double value)
{
    return std::isfinite(value) && value > 0.0;
}

bool completeToolAssemblyGeometry(const ToolpathParams& params)
{
    return positiveFinite(params.tool.cuttingEdgeLength) &&
           positiveFinite(params.tool.overallLength) &&
           params.tool.overallLength >= params.tool.cuttingEdgeLength &&
           positiveFinite(params.tool.shankDiameter) &&
           positiveFinite(params.tool.holderDiameter) &&
           positiveFinite(params.tool.holderLength) &&
           positiveFinite(params.tool.spindleEnvelopeDiameter) &&
           positiveFinite(params.tool.spindleEnvelopeLength);
}

const char* moveTypeName(ToolpathMoveType type)
{
    switch (type) {
        case ToolpathMoveType::Rapid: return "rapid";
        case ToolpathMoveType::Plunge: return "plunge";
        case ToolpathMoveType::Cut: return "cut";
        case ToolpathMoveType::Retract: return "retract";
    }
    return "unknown";
}

const char* moveIntentName(ToolpathMoveIntent intent)
{
    switch (intent) {
        case ToolpathMoveIntent::General: return "general";
        case ToolpathMoveIntent::DrillCycleReentry:
            return "drill-cycle-reentry";
        case ToolpathMoveIntent::PocketLinearRampEntry:
            return "pocket-linear-ramp-entry";
        case ToolpathMoveIntent::PocketHelicalRampEntry:
            return "pocket-helical-ramp-entry";
    }
    return "unknown";
}

std::string pointText(const gp_Pnt& point)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << '(' << point.X() << ',' << point.Y() << ',' << point.Z() << ')';
    return stream.str();
}
}

size_t HoleRapidPlanner::RapidLinkKeyHash::operator()(
    const RapidLinkKey& key) const noexcept
{
    size_t hash = 1469598103934665603ull;
    for (std::uint64_t value : key.values) {
        hash ^= static_cast<size_t>(value);
        hash *= 1099511628211ull;
    }
    return hash;
}

HoleRapidPlanner::HoleRapidPlanner(const HoleRapidPlanningContext* context,
                                   const ToolpathParams& params,
                                   HoleMachiningResult& result,
                                   HoleRapidLinkMode linkMode,
                                   const ToolpathGenerationControl* control)
    : m_context(context),
      m_params(params),
      m_result(result),
      m_linkMode(linkMode),
      m_control(control)
{
    const double cutterRadius = m_params.tool.radius();
    m_maxEnvelopeRadius = std::isfinite(cutterRadius) && cutterRadius > 0.0
        ? cutterRadius : 0.0;
    if (!m_context)
        return;

    // 规划器启动时冻结容差和障碍物包围盒，单段评估不再重复遍历实体。
    const double numericTolerance =
        std::isfinite(m_context->collisionTolerance)
        ? std::max(m_context->collisionTolerance, Precision::Confusion())
        : std::max(1.0e-4, Precision::Confusion());
    const double operationClearance =
        std::isfinite(m_params.hole.collisionClearance)
        ? std::max(0.0, m_params.hole.collisionClearance)
        : 0.0;
    m_tolerance = numericTolerance + operationClearance;
    m_axisToleranceDeg =
        std::isfinite(m_context->axisToleranceDeg) &&
        m_context->axisToleranceDeg > 0.0
            ? std::clamp(m_context->axisToleranceDeg, 0.01, 45.0)
            : 1.0;
    m_toolEnvelopeValid = buildToolEnvelope();
    if (!m_toolEnvelopeValid) {
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::InvalidToolAssembly,
            ToolpathDiagnosticSeverity::Error,
            "collision-envelope checks require a positive tool diameter");
        return;
    }
    if (m_params.hole.collisionEnvelopeMode !=
            HoleCollisionEnvelopeMode::CutterOnly &&
        !completeToolAssemblyGeometry(m_params)) {
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::InvalidToolAssembly,
            ToolpathDiagnosticSeverity::Warning,
            "tool-assembly geometry is incomplete; collision checks continue with available components and omit unknown cutter, shank, holder, or spindle sections");
    }
    if (!m_context->collisionScene.empty()) {
        m_collisionBodies = m_context->collisionScene.bodies();
    } else if (!m_context->obstacleShape.IsNull()) {
        m_collisionBodies.push_back(
            {"legacy_workpiece", CollisionBodyRole::Workpiece,
             m_context->obstacleShape});
    }
    if (m_collisionBodies.empty())
        return;

    try {
        TopoDS_Compound combined;
        BRep_Builder builder;
        builder.MakeCompound(combined);
        m_collisionBodyBounds.reserve(m_collisionBodies.size());
        for (const CollisionBody& body : m_collisionBodies) {
            builder.Add(combined, body.shape);
            CollisionBodyBounds cached;
            try {
                Bnd_Box bodyBounds;
                BRepBndLib::Add(body.shape, bodyBounds);
                if (!bodyBounds.IsVoid() && !bodyBounds.IsOpen()) {
                    bodyBounds.Get(cached.xMin, cached.yMin, cached.zMin,
                                   cached.xMax, cached.yMax, cached.zMax);
                    cached.valid =
                        std::isfinite(cached.xMin) &&
                        std::isfinite(cached.yMin) &&
                        std::isfinite(cached.zMin) &&
                        std::isfinite(cached.xMax) &&
                        std::isfinite(cached.yMax) &&
                        std::isfinite(cached.zMax);
                }
            } catch (const Standard_Failure&) {
                // 单体包围盒不可用时退回精确检查，不能因此把碰撞判为安全。
            } catch (...) {
                // 同上：无粗筛证据时保守地保留该障碍物。
            }
            m_collisionBodyBounds.push_back(cached);
        }
        m_combinedObstacle = combined;
        Bnd_Box bounds;
        BRepBndLib::Add(m_combinedObstacle, bounds);
        if (bounds.IsVoid() || bounds.IsOpen())
            return;
        bounds.Get(m_xMin, m_yMin, m_zMin, m_xMax, m_yMax, m_zMax);
        m_ready = std::isfinite(m_xMin) && std::isfinite(m_yMin) &&
                  std::isfinite(m_zMin) && std::isfinite(m_xMax) &&
                  std::isfinite(m_yMax) && std::isfinite(m_zMax);
    } catch (const Standard_Failure&) {
        m_ready = false;
    }
}

bool HoleRapidPlanner::acceptAxis(const gp_Vec& inwardAxis,
                                  const std::string& holeLabel)
{
    if (!m_context)
        return true;
    if (!m_toolEnvelopeValid) {
        ++m_result.unsafeRapidLinkCount;
        return false;
    }

    gp_Vec axis = inwardAxis;
    if (axis.Magnitude() <= kHoleTolerance) {
        ++m_result.unsafeRapidLinkCount;
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::InvalidSetupAxis,
            ToolpathDiagnosticSeverity::Error,
            holeLabel + " skipped because its machining axis is invalid");
        return false;
    }
    axis.Normalize();

    if (!m_hasSetupAxis) {
        m_inwardAxis = axis;
        m_outwardAxis = axis.Reversed();
        m_hasSetupAxis = true;
        if (m_ready)
            computeGlobalClearanceProjection();
        return true;
    }

    constexpr double kDegreesToRadians =
        3.14159265358979323846 / 180.0;
    const double minimumDot =
        std::cos(m_axisToleranceDeg * kDegreesToRadians);
    if (m_inwardAxis.Dot(axis) < minimumDot) {
        ++m_result.unsafeRapidLinkCount;
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::SetupAxisMismatch,
            ToolpathDiagnosticSeverity::Error,
            holeLabel +
                " skipped because its axis differs from the active setup axis");
        return false;
    }
    return true;
}

double HoleRapidPlanner::minimumClearanceOffset() const
{
    return m_maxEnvelopeRadius + 2.0 * m_tolerance;
}

bool HoleRapidPlanner::canPlanLinks() const
{
    return m_context && m_ready && m_toolEnvelopeValid && m_hasSetupAxis;
}

RapidLinkPlan HoleRapidPlanner::evaluateLink(const gp_Pnt& from,
                                             const gp_Pnt& to)
{
    if (cancellationRequested()) {
        RapidLinkPlan plan;
        plan.unavailable = true;
        return plan;
    }
    if (!m_context) {
        RapidLinkPlan plan;
        plan.safe = true;
        plan.direct = {SegmentSafety::Safe,
                       std::numeric_limits<double>::infinity(), false};
        plan.length = from.Distance(to);
        return plan;
    }
    if (!m_ready || !m_hasSetupAxis) {
        RapidLinkPlan plan;
        plan.unavailable = true;
        return plan;
    }

    // 快速连接是无向安全问题，端点规范排序后可让正反方向共享缓存。
    const gp_Pnt& first = pointLess(to, from) ? to : from;
    const gp_Pnt& second = pointLess(to, from) ? from : to;
    const RapidLinkKey key{{doubleBits(first.X()), doubleBits(first.Y()),
                            doubleBits(first.Z()), doubleBits(second.X()),
                            doubleBits(second.Y()), doubleBits(second.Z())}};
    auto found = m_linkCache.find(key);
    if (found == m_linkCache.end())
        found = m_linkCache.emplace(key, evaluateCanonicalLink(first, second)).first;

    RapidLinkPlan plan = found->second;
    if (pointLess(to, from)) {
        std::swap(plan.fromLifted, plan.toLifted);
        std::swap(plan.sourceLift, plan.targetDescent);
    }
    return plan;
}

bool HoleRapidPlanner::appendLink(std::vector<ToolpathMove>& moves,
                                  const gp_Pnt& from,
                                  const gp_Pnt& to,
                                  int operationIndex)
{
    if (cancellationRequested())
        return false;
    if (!m_context) {
        appendRapidMove(moves, from, to, operationIndex,
                        m_params.tool.horizRapid);
        return true;
    }
    if (!m_ready || !m_hasSetupAxis) {
        ++m_result.unsafeRapidLinkCount;
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::CollisionPlanningUnavailable,
            ToolpathDiagnosticSeverity::Error,
            "hole rapid link skipped because obstacle planning is unavailable");
        return false;
    }

    // 优先采用直接连线；仅在检测到碰撞时才尝试“抬升—横移—下降”。
    const RapidLinkPlan plan = evaluateLink(from, to);
    if (cancellationRequested())
        return false;
    if (plan.safe && !plan.lifted) {
        appendRapidMove(moves, from, to, operationIndex,
                        m_params.tool.horizRapid);
        ++m_result.directRapidLinkCount;
        return true;
    }
    if (!plan.safe && plan.direct.safety == SegmentSafety::Failed) {
        ++m_result.unsafeRapidLinkCount;
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::CollisionCheckFailed,
            ToolpathDiagnosticSeverity::Error,
            "hole rapid link skipped for operation " +
                std::to_string(operationIndex + 1) +
                " because collision detection failed: " +
                segmentCheckText("direct", plan.direct),
            -1, "DRILL" + std::to_string(operationIndex + 1));
        return false;
    }

    if (!plan.safe) {
        ++m_result.unsafeRapidLinkCount;
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::RapidLinkCollision,
            ToolpathDiagnosticSeverity::Error,
            "hole rapid link skipped for operation " +
                std::to_string(operationIndex + 1) +
                " because the lifted route is not collision-free: " +
                segmentCheckText("direct", plan.direct) + "; " +
                segmentCheckText("source_lift", plan.sourceLift) + "; " +
                segmentCheckText("top_traverse", plan.topTraverse) + "; " +
                segmentCheckText("target_descent", plan.targetDescent),
            -1, "DRILL" + std::to_string(operationIndex + 1));
        return false;
    }

    appendRapidMove(moves, from, plan.fromLifted, operationIndex,
                    m_params.tool.vertRapid);
    appendRapidMove(moves, plan.fromLifted, plan.toLifted, operationIndex,
                    m_params.tool.horizRapid);
    appendRapidMove(moves, plan.toLifted, to, operationIndex,
                    m_params.tool.vertRapid);
    ++m_result.liftedRapidLinkCount;
    return true;
}

bool HoleRapidPlanner::validateNonCuttingMoves(
    const std::vector<ToolpathMove>& moves,
    const std::string& operationId)
{
    if (m_params.hole.collisionEnvelopeMode !=
        HoleCollisionEnvelopeMode::ToolAssemblySolidSweep)
        return true;
    if (!m_ready || !m_hasSetupAxis || !m_toolEnvelopeValid) {
        ++m_result.unsafeRapidLinkCount;
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::CollisionPlanningUnavailable,
            ToolpathDiagnosticSeverity::Error,
            operationId +
                " rejected because solid-sweep collision planning is unavailable",
            -1, operationId);
        return false;
    }

    ToolpathValidationReport reentryReport;
    validateHoleDrillReentries(moves, m_inwardAxis, operationId, reentryReport);
    if (!reentryReport.ok()) {
        ++m_result.unsafeRapidLinkCount;
        addHoleDiagnostic(m_result, ToolpathDiagnosticCode::CollisionCheckFailed,
                          ToolpathDiagnosticSeverity::Error,
                          operationId + ": " + reentryReport.issues.front().message,
                          -1, operationId);
        return false;
    }

    for (std::size_t index = 0; index < moves.size(); ++index) {
        if (cancellationRequested())
            return false;
        const ToolpathMove& move = moves[index];
        if (move.type == ToolpathMoveType::Cut)
            continue;
        // 进刀、退刀以及沿当前装夹轴向孔内运动的 G83 重入，允许切削部位于
        // 已识别孔的工件空间内；刀杆/刀柄以及任意组件对夹具或机床结构的
        // 接触始终禁止。普通 Rapid 即使沿轴向移动也不获得此豁免。
        bool drillCycleReentry =
            move.type == ToolpathMoveType::Rapid &&
            move.intent == ToolpathMoveIntent::DrillCycleReentry &&
            move.depthIndex > 0;
        if (drillCycleReentry) {
            gp_Vec travel(move.from, move.to);
            if (travel.Magnitude() <= kHoleTolerance) {
                drillCycleReentry = false;
            } else {
                travel.Normalize();
                constexpr double kDegreesToRadians =
                    3.14159265358979323846 / 180.0;
                const double minimumDot =
                    std::cos(m_axisToleranceDeg * kDegreesToRadians);
                drillCycleReentry = travel.Dot(m_inwardAxis) >= minimumDot;
            }
        }
        const bool allowCutterWorkpiece =
            move.type == ToolpathMoveType::Plunge ||
            move.type == ToolpathMoveType::Retract ||
            drillCycleReentry;
        const SegmentCheck check = solidSweepSafety(
            move.from, move.to, allowCutterWorkpiece);
        if (cancellationRequested())
            return false;
        if (check.safety == SegmentSafety::Safe)
            continue;

        ++m_result.unsafeRapidLinkCount;
        if (check.safety == SegmentSafety::Collision) {
            ++m_result.nonCuttingMoveCollisionCount;
            addHoleDiagnostic(
                m_result, ToolpathDiagnosticCode::NonCuttingMoveCollision,
                ToolpathDiagnosticSeverity::Error,
                operationId + " rejected because non-cutting move " +
                    std::to_string(index) + " [type=" +
                    moveTypeName(move.type) + ",intent=" +
                    moveIntentName(move.intent) + ",depth=" +
                    std::to_string(move.depthIndex) + ",from=" +
                    pointText(move.from) + ",to=" + pointText(move.to) +
                    "] collides: " +
                    segmentCheckText("solid_sweep", check),
                -1, operationId);
        } else {
            addHoleDiagnostic(
                m_result, ToolpathDiagnosticCode::CollisionCheckFailed,
                ToolpathDiagnosticSeverity::Error,
                operationId + " rejected because solid sweep failed for move " +
                    std::to_string(index) + " [type=" +
                    moveTypeName(move.type) + ",intent=" +
                    moveIntentName(move.intent) + ",depth=" +
                    std::to_string(move.depthIndex) + ",from=" +
                    pointText(move.from) + ",to=" + pointText(move.to) +
                    "]: " +
                    segmentCheckText("solid_sweep", check),
                -1, operationId);
        }
        return false;
    }
    return true;
}

bool HoleRapidPlanner::validateCuttingGeometry(
    const std::vector<ToolpathMove>& moves,
    const std::vector<ToolpathPrimitive>& primitives,
    const std::string& operationId)
{
    if (m_params.hole.collisionEnvelopeMode !=
        HoleCollisionEnvelopeMode::ToolAssemblySolidSweep)
        return true;
    if (!m_ready || !m_hasSetupAxis || !m_toolEnvelopeValid) {
        addHoleDiagnostic(
            m_result, ToolpathDiagnosticCode::CollisionPlanningUnavailable,
            ToolpathDiagnosticSeverity::Error,
            operationId +
                " rejected because cutting-sweep collision planning is unavailable",
            -1, operationId);
        return false;
    }

    auto acceptCheck = [&](const SegmentCheck& check,
                           const std::string& geometryLabel) {
        if (cancellationRequested())
            return false;
        if (check.safety == SegmentSafety::Safe)
            return true;
        if (check.safety == SegmentSafety::Collision) {
            ++m_result.cuttingMoveCollisionCount;
            addHoleDiagnostic(
                m_result, ToolpathDiagnosticCode::CuttingMoveCollision,
                ToolpathDiagnosticSeverity::Error,
                operationId + " rejected because " + geometryLabel +
                    " collides: " + segmentCheckText("cutting_sweep", check),
                -1, operationId);
        } else {
            addHoleDiagnostic(
                m_result, ToolpathDiagnosticCode::CollisionCheckFailed,
                ToolpathDiagnosticSeverity::Error,
                operationId + " rejected because " + geometryLabel +
                    " sweep construction failed",
                -1, operationId);
        }
        return false;
    };

    bool hasPrimitiveCuttingGeometry = false;
    for (std::size_t index = 0; index < primitives.size(); ++index) {
        if (cancellationRequested())
            return false;
        const ToolpathPrimitive& primitive = primitives[index];
        if (primitive.type == ToolpathPrimitiveType::Linear &&
            primitive.linear.type == ToolpathMoveType::Cut) {
            hasPrimitiveCuttingGeometry = true;
            if (!acceptCheck(
                    solidSweepSafety(primitive.linear.from,
                                     primitive.linear.to, true),
                    "linear cutting primitive " + std::to_string(index)))
                return false;
            continue;
        }
        if (primitive.type == ToolpathPrimitiveType::CircularArc &&
            primitive.arc.moveType == ToolpathMoveType::Cut) {
            hasPrimitiveCuttingGeometry = true;
            ++m_result.arcSweepPrimitiveCount;
            if (!acceptCheck(
                    arcSweepSafety(primitive.arc),
                    "circular/helical primitive " + std::to_string(index)))
                return false;
            continue;
        }
        if (primitive.type != ToolpathPrimitiveType::DrillCycle)
            continue;

        bool checkedFallback = false;
        for (std::size_t moveIndex = 0;
             moveIndex < primitive.fallbackMoves.size(); ++moveIndex) {
            if (cancellationRequested())
                return false;
            const ToolpathMove& move = primitive.fallbackMoves[moveIndex];
            if (move.type != ToolpathMoveType::Cut)
                continue;
            checkedFallback = true;
            hasPrimitiveCuttingGeometry = true;
            if (!acceptCheck(
                    solidSweepSafety(move.from, move.to, true),
                    "drill-cycle cutting move " +
                        std::to_string(moveIndex)))
                return false;
        }
        if (!checkedFallback) {
            hasPrimitiveCuttingGeometry = true;
            const ToolpathDrillCycle& cycle = primitive.drillCycle;
            if (!acceptCheck(
                    solidSweepSafety(cycle.entry, cycle.bottom, true),
                    "drill-cycle cutting stroke"))
                return false;
        }
    }

    if (hasPrimitiveCuttingGeometry)
        return true;
    for (std::size_t index = 0; index < moves.size(); ++index) {
        if (cancellationRequested())
            return false;
        const ToolpathMove& move = moves[index];
        if (move.type != ToolpathMoveType::Cut)
            continue;
        if (!acceptCheck(
                solidSweepSafety(move.from, move.to, true),
                "linear cutting move " + std::to_string(index)))
            return false;
    }
    return true;
}

RapidLinkPlan HoleRapidPlanner::evaluateCanonicalLink(const gp_Pnt& from,
                                                       const gp_Pnt& to)
{
    RapidLinkPlan plan;
    if (cancellationRequested()) {
        plan.unavailable = true;
        return plan;
    }
    plan.direct = segmentSafety(from, to);
    if (cancellationRequested()) {
        plan.unavailable = true;
        return plan;
    }
    const bool sameClearancePlane =
        std::abs(projection(from) - projection(to)) <=
        kHoleTolerance;
    if (plan.direct.safety == SegmentSafety::Safe &&
        (m_linkMode == HoleRapidLinkMode::DirectWhenSafe ||
         sameClearancePlane)) {
        plan.safe = true;
        plan.length = from.Distance(to);
        return plan;
    }
    if (plan.direct.safety == SegmentSafety::Failed)
        return plan;

    // 抬升平面位于整个障碍包络之外，因此三个分段可独立做精确安全验证。
    plan.fromLifted = liftToGlobalPlane(from);
    plan.toLifted = liftToGlobalPlane(to);
    plan.sourceLift = segmentSafety(from, plan.fromLifted);
    if (cancellationRequested())
        return plan;
    plan.topTraverse = segmentSafety(plan.fromLifted, plan.toLifted);
    if (cancellationRequested())
        return plan;
    plan.targetDescent = segmentSafety(plan.toLifted, to);
    if (plan.sourceLift.safety == SegmentSafety::Safe &&
        plan.topTraverse.safety == SegmentSafety::Safe &&
        plan.targetDescent.safety == SegmentSafety::Safe) {
        plan.safe = true;
        plan.lifted = true;
        plan.length = from.Distance(plan.fromLifted) +
                      plan.fromLifted.Distance(plan.toLifted) +
                      plan.toLifted.Distance(to);
    }
    return plan;
}

std::string HoleRapidPlanner::segmentCheckText(
    const char* label, const SegmentCheck& check) const
{
    const char* safety = "failed";
    if (check.safety == SegmentSafety::Safe)
        safety = "safe";
    else if (check.safety == SegmentSafety::Collision)
        safety = "collision";

    std::ostringstream stream;
    stream << label << '=' << safety << "(distance=";
    if (std::isfinite(check.distance))
        stream << std::fixed << std::setprecision(6) << check.distance;
    else
        stream << "n/a";
    stream << ",threshold=" << std::fixed << std::setprecision(6)
           << (std::isfinite(check.threshold)
                   ? check.threshold
                   : m_maxEnvelopeRadius + m_tolerance)
           << ",component="
           << (check.component.empty() ? "cutter" : check.component)
           << ",body="
           << (check.bodyId.empty() ? "obstacle" : check.bodyId)
           << ",offset=" << check.axialOffset
           << ",inner=" << (check.innerSolution ? 1 : 0) << ')';
    return stream.str();
}

double HoleRapidPlanner::projection(const gp_Pnt& point) const
{
    return gp_Vec(gp_Pnt(0.0, 0.0, 0.0), point).Dot(m_outwardAxis);
}

void HoleRapidPlanner::computeGlobalClearanceProjection()
{
    // 沿装夹外向轴投影包围盒八角点，取最外侧值再叠加刀具/配置安全余量。
    const std::array<double, 2> xs{m_xMin, m_xMax};
    const std::array<double, 2> ys{m_yMin, m_yMax};
    const std::array<double, 2> zs{m_zMin, m_zMax};
    double modelMaximum = -std::numeric_limits<double>::max();
    for (double x : xs)
        for (double y : ys)
            for (double z : zs)
                modelMaximum = std::max(
                    modelMaximum, projection(gp_Pnt(x, y, z)));

    const double configuredClearance =
        std::isfinite(m_params.depth.clearanceHeight)
            ? std::max(0.0, m_params.depth.clearanceHeight)
            : 0.0;
    const double envelope = std::max(
        configuredClearance, m_maxEnvelopeRadius + m_tolerance);
    m_globalClearanceProjection = modelMaximum + envelope;
}

gp_Pnt HoleRapidPlanner::liftToGlobalPlane(const gp_Pnt& point) const
{
    const double lift = m_globalClearanceProjection - projection(point);
    return lift > kHoleTolerance
        ? point.Translated(m_outwardAxis.Multiplied(lift))
        : point;
}

bool HoleRapidPlanner::cancellationRequested() const
{
    return m_control && m_control->cancelled();
}

SegmentCheck HoleRapidPlanner::segmentSafety(
    const gp_Pnt& from, const gp_Pnt& to) const
{
    if (cancellationRequested())
        return {SegmentSafety::Failed};
    if (from.Distance(to) <= kHoleTolerance) {
        return {SegmentSafety::Safe,
                std::numeric_limits<double>::infinity(), false};
    }
    if (m_params.hole.collisionEnvelopeMode ==
        HoleCollisionEnvelopeMode::ToolAssemblySolidSweep) {
        return solidSweepSafety(from, to, false);
    }

    // 每个采样站代表刀轴方向上一段连续旋转体，并以略扩大的胶囊包络
    // 保守覆盖该段。刀柄模式因此能发现刀尖轨迹安全但上方夹头擦碰的路径。
    bool usedExactCheck = false;
    SegmentCheck closest{SegmentSafety::Safe,
                         std::numeric_limits<double>::infinity(), false};
    for (const EnvelopeStation& station : m_envelopeStations) {
        if (cancellationRequested())
            return {SegmentSafety::Failed};
        const gp_Vec axial = m_outwardAxis.Multiplied(station.axialOffset);
        const gp_Pnt stationFrom = from.Translated(axial);
        const gp_Pnt stationTo = to.Translated(axial);
        const double threshold = station.radius + m_tolerance;
        const bool overlapsExpandedBounds =
            std::max(stationFrom.X(), stationTo.X()) >= m_xMin - threshold &&
            std::min(stationFrom.X(), stationTo.X()) <= m_xMax + threshold &&
            std::max(stationFrom.Y(), stationTo.Y()) >= m_yMin - threshold &&
            std::min(stationFrom.Y(), stationTo.Y()) <= m_yMax + threshold &&
            std::max(stationFrom.Z(), stationTo.Z()) >= m_zMin - threshold &&
            std::min(stationFrom.Z(), stationTo.Z()) <= m_zMax + threshold;
        if (!overlapsExpandedBounds)
            continue;

        try {
            usedExactCheck = true;
            ++m_result.exactCollisionCheckCount;
            BRepBuilderAPI_MakeEdge edgeBuilder(stationFrom, stationTo);
            if (!edgeBuilder.IsDone()) {
                return {SegmentSafety::Failed,
                        std::numeric_limits<double>::quiet_NaN(), false,
                        threshold, station.axialOffset, station.component};
            }

            BRepExtrema_DistShapeShape distance(
                edgeBuilder.Edge(), m_combinedObstacle, m_tolerance);
            if (cancellationRequested())
                return {SegmentSafety::Failed};
            if (!distance.IsDone() || !std::isfinite(distance.Value())) {
                return {SegmentSafety::Failed,
                        std::numeric_limits<double>::quiet_NaN(), false,
                        threshold, station.axialOffset, station.component};
            }
            const bool inner = distance.InnerSolution();
            const double value = distance.Value();
            if (inner || value <= threshold) {
                return {SegmentSafety::Collision, value, inner, threshold,
                        station.axialOffset, station.component};
            }
            if (value < closest.distance) {
                closest = {SegmentSafety::Safe, value, inner, threshold,
                           station.axialOffset, station.component};
            }
        } catch (const Standard_Failure&) {
            return {SegmentSafety::Failed,
                    std::numeric_limits<double>::quiet_NaN(), false,
                    threshold, station.axialOffset, station.component};
        } catch (...) {
            return {SegmentSafety::Failed,
                    std::numeric_limits<double>::quiet_NaN(), false,
                    threshold, station.axialOffset, station.component};
        }
    }
    if (!usedExactCheck)
        ++m_result.broadPhaseSafeSegmentCount;
    return closest;
}

SegmentCheck HoleRapidPlanner::solidSweepSafety(
    const gp_Pnt& from,
    const gp_Pnt& to,
    bool allowCutterWorkpiece) const
{
    return solidSweepSafety({from, to}, {0.0}, allowCutterWorkpiece);
}

SegmentCheck HoleRapidPlanner::solidSweepSafety(
    const std::vector<gp_Pnt>& path,
    const std::vector<double>& segmentInflations,
    bool allowCutterWorkpiece) const
{
    try {
        if (cancellationRequested())
            return {SegmentSafety::Failed};
        if (path.empty() ||
            (path.size() > 1 &&
             segmentInflations.size() != path.size() - 1) ||
            m_collisionBodyBounds.size() != m_collisionBodies.size()) {
            return {SegmentSafety::Failed};
        }
        SegmentCheck closest{SegmentSafety::Safe,
                             std::numeric_limits<double>::infinity(), false};
        bool usedExactCheck = false;
        bool usedBroadPhase = false;
        for (const EnvelopeStation& station : m_envelopeStations) {
            if (cancellationRequested())
                return {SegmentSafety::Failed};
            std::vector<std::size_t> candidateBodies;
            candidateBodies.reserve(m_collisionBodies.size());
            for (std::size_t bodyIndex = 0;
                 bodyIndex < m_collisionBodies.size(); ++bodyIndex) {
                if (cancellationRequested())
                    return {SegmentSafety::Failed};
                const CollisionBody& body = m_collisionBodies[bodyIndex];
                // 切削、进刀和退刀允许切削部接触工件。必须在构造胶囊体
                // 之前跳过，否则长刃刀仍会白白创建数十个 OCCT 复合体。
                if (allowCutterWorkpiece && station.component == "cutter" &&
                    body.role == CollisionBodyRole::Workpiece) {
                    continue;
                }
                ++m_result.solidSweepCheckCount;
                if (!stationMayOverlapBody(
                        station, path, segmentInflations,
                        m_collisionBodyBounds[bodyIndex])) {
                    usedBroadPhase = true;
                    continue;
                }
                candidateBodies.push_back(bodyIndex);
            }
            if (candidateBodies.empty())
                continue;

            const ToolSolidPart part = buildSweptToolSolid(
                station, path, segmentInflations);
            if (part.shape.IsNull()) {
                return {SegmentSafety::Failed,
                        std::numeric_limits<double>::quiet_NaN(), false,
                        m_tolerance, station.axialOffset,
                        station.component};
            }
            ++m_result.solidSweepConstructedPartCount;
            for (const std::size_t bodyIndex : candidateBodies) {
                if (cancellationRequested())
                    return {SegmentSafety::Failed};
                const CollisionBody& body = m_collisionBodies[bodyIndex];
                usedExactCheck = true;
                ++m_result.exactCollisionCheckCount;
                BRepExtrema_DistShapeShape distance(
                    part.shape, body.shape, m_tolerance);
                if (cancellationRequested())
                    return {SegmentSafety::Failed};
                if (!distance.IsDone() || !std::isfinite(distance.Value())) {
                    return {SegmentSafety::Failed,
                            std::numeric_limits<double>::quiet_NaN(), false,
                            m_tolerance, station.axialOffset,
                            part.component, body.id};
                }
                const bool inner = distance.InnerSolution();
                const double value = distance.Value();
                if (inner || value <= m_tolerance) {
                    return {SegmentSafety::Collision, value, inner,
                            m_tolerance, station.axialOffset,
                            part.component, body.id};
                }
                if (value < closest.distance) {
                    closest = {SegmentSafety::Safe, value, inner,
                               m_tolerance, station.axialOffset,
                               part.component, body.id};
                }
            }
        }
        if (!usedExactCheck && usedBroadPhase)
            ++m_result.broadPhaseSafeSegmentCount;
        return closest;
    } catch (const Standard_Failure&) {
        return {SegmentSafety::Failed};
    } catch (...) {
        return {SegmentSafety::Failed};
    }
}

SegmentCheck HoleRapidPlanner::arcSweepSafety(
    const ToolpathCircularArc& arc) const
{
    if (!std::isfinite(arc.sweepRadians) || arc.sweepRadians <= 0.0 ||
        arc.axis.SquareMagnitude() <= kHoleTolerance * kHoleTolerance)
        return {SegmentSafety::Failed};

    gp_Vec axis = arc.axis;
    axis.Normalize();
    const gp_Vec centerToStart(arc.center, arc.from);
    const double startAxial = centerToStart.Dot(axis);
    const gp_Vec startRadial =
        centerToStart - axis.Multiplied(startAxial);
    const double radius = startRadial.Magnitude();
    if (!std::isfinite(radius) || radius <= kHoleTolerance)
        return {SegmentSafety::Failed};

    const gp_Vec fromTo(arc.from, arc.to);
    const double totalAxial = fromTo.Dot(axis);
    const double signedSweep =
        arc.direction == ToolpathArcDirection::Counterclockwise
            ? arc.sweepRadians : -arc.sweepRadians;
    const double requestedAngleDeg =
        std::isfinite(m_params.hole.sampleAngleDeg)
            ? m_params.hole.sampleAngleDeg : 5.0;
    constexpr double kDegreesToRadians =
        3.14159265358979323846 / 180.0;
    const double maximumSegmentAngle =
        std::clamp(requestedAngleDeg, 1.0, 10.0) * kDegreesToRadians;
    const double requestedSegmentsValue =
        std::ceil(arc.sweepRadians / maximumSegmentAngle);
    if (!std::isfinite(requestedSegmentsValue))
        return {SegmentSafety::Failed};
    const int maximumSegments = std::clamp(
        m_params.hole.maxSamplePoints, 16, 10000);
    const int segmentCount = std::max(
        1, static_cast<int>(std::min(
               requestedSegmentsValue,
               static_cast<double>(maximumSegments))));
    const double segmentAngle = arc.sweepRadians / segmentCount;
    const double sagitta =
        radius * (1.0 - std::cos(0.5 * segmentAngle));
    if (!std::isfinite(sagitta) || sagitta < 0.0)
        return {SegmentSafety::Failed};

    std::vector<gp_Pnt> path;
    std::vector<double> segmentInflations;
    path.reserve(static_cast<std::size_t>(segmentCount) + 1);
    segmentInflations.assign(
        static_cast<std::size_t>(segmentCount), sagitta);
    const gp_Vec quarterTurn = axis.Crossed(startRadial);
    for (int index = 0; index <= segmentCount; ++index) {
        if (cancellationRequested())
            return {SegmentSafety::Failed};
        if (index == segmentCount) {
            path.push_back(arc.to);
            continue;
        }
        const double ratio =
            static_cast<double>(index) / segmentCount;
        const double angle = signedSweep * ratio;
        const gp_Vec radial =
            startRadial.Multiplied(std::cos(angle)) +
            quarterTurn.Multiplied(std::sin(angle));
        path.push_back(arc.center.Translated(
            radial + axis.Multiplied(
                         startAxial + totalAxial * ratio)));
    }

    // 拒绝端点、半径或扫角互相矛盾的原语，避免以一条人工补线掩盖坏几何。
    const double endAngle = signedSweep;
    const gp_Vec expectedEndRadial =
        startRadial.Multiplied(std::cos(endAngle)) +
        quarterTurn.Multiplied(std::sin(endAngle));
    const gp_Pnt expectedEnd = arc.center.Translated(
        expectedEndRadial +
        axis.Multiplied(startAxial + totalAxial));
    const double endpointTolerance =
        std::max(10.0 * m_tolerance, 1.0e-6 * radius);
    if (expectedEnd.Distance(arc.to) > endpointTolerance)
        return {SegmentSafety::Failed};
    return solidSweepSafety(path, segmentInflations, true);
}

bool HoleRapidPlanner::stationMayOverlapBody(
    const EnvelopeStation& station,
    const std::vector<gp_Pnt>& path,
    const std::vector<double>& segmentInflations,
    const CollisionBodyBounds& bodyBounds) const
{
    if (!bodyBounds.valid)
        return true;
    if (path.empty())
        return false;

    double maximumInflation = 0.0;
    for (const double inflation : segmentInflations) {
        if (!std::isfinite(inflation) || inflation < 0.0)
            return true;
        maximumInflation = std::max(maximumInflation, inflation);
    }
    const double radius = station.radius + maximumInflation + m_tolerance;
    if (!std::isfinite(radius) || radius <= 0.0)
        return true;
    if (!std::isfinite(station.axialOffset))
        return true;

    const gp_Vec axial = m_outwardAxis.Multiplied(station.axialOffset);
    double xMin = std::numeric_limits<double>::infinity();
    double yMin = std::numeric_limits<double>::infinity();
    double zMin = std::numeric_limits<double>::infinity();
    double xMax = -std::numeric_limits<double>::infinity();
    double yMax = -std::numeric_limits<double>::infinity();
    double zMax = -std::numeric_limits<double>::infinity();
    for (const gp_Pnt& point : path) {
        const gp_Pnt shifted = point.Translated(axial);
        if (!std::isfinite(shifted.X()) || !std::isfinite(shifted.Y()) ||
            !std::isfinite(shifted.Z())) {
            return true;
        }
        xMin = std::min(xMin, shifted.X());
        yMin = std::min(yMin, shifted.Y());
        zMin = std::min(zMin, shifted.Z());
        xMax = std::max(xMax, shifted.X());
        yMax = std::max(yMax, shifted.Y());
        zMax = std::max(zMax, shifted.Z());
    }
    return xMax + radius >= bodyBounds.xMin &&
           xMin - radius <= bodyBounds.xMax &&
           yMax + radius >= bodyBounds.yMin &&
           yMin - radius <= bodyBounds.yMax &&
           zMax + radius >= bodyBounds.zMin &&
           zMin - radius <= bodyBounds.zMax;
}

HoleRapidPlanner::ToolSolidPart HoleRapidPlanner::buildSweptToolSolid(
    const EnvelopeStation& station,
    const std::vector<gp_Pnt>& path,
    const std::vector<double>& segmentInflations) const
{
    if (path.empty() ||
        (path.size() > 1 && segmentInflations.size() != path.size() - 1)) {
        return {};
    }

    const gp_Vec axial = m_outwardAxis.Multiplied(station.axialOffset);
    TopoDS_Compound swept;
    BRep_Builder builder;
    builder.MakeCompound(swept);
    if (path.size() == 1) {
        BRepPrimAPI_MakeSphere sphere(
            path.front().Translated(axial), station.radius);
        const TopoDS_Shape shape = sphere.Shape();
        if (shape.IsNull())
            return {};
        builder.Add(swept, shape);
    }
    for (std::size_t index = 1; index < path.size(); ++index) {
        if (cancellationRequested())
            return {};
        const double inflation = segmentInflations[index - 1];
        if (!std::isfinite(inflation) || inflation < 0.0)
            return {};
        const double radius = station.radius + inflation;
        const gp_Pnt start = path[index - 1].Translated(axial);
        const gp_Pnt end = path[index].Translated(axial);
        const gp_Vec travel(start, end);
        const double travelLength = travel.Magnitude();
        BRepPrimAPI_MakeSphere startSphere(start, radius);
        const TopoDS_Shape startShape = startSphere.Shape();
        if (startShape.IsNull())
            return {};
        builder.Add(swept, startShape);
        if (travelLength <= kHoleTolerance)
            continue;
        BRepPrimAPI_MakeCylinder cylinder(
            gp_Ax2(start, gp_Dir(travel)), radius, travelLength);
        BRepPrimAPI_MakeSphere endSphere(end, radius);
        const TopoDS_Shape cylinderShape = cylinder.Shape();
        const TopoDS_Shape endShape = endSphere.Shape();
        if (cylinderShape.IsNull() || endShape.IsNull())
            return {};
        builder.Add(swept, cylinderShape);
        builder.Add(swept, endShape);
    }
    return {swept, station.component};
}

bool HoleRapidPlanner::buildToolEnvelope()
{
    m_envelopeStations.clear();
    const double cutterRadius = m_params.tool.radius();
    if (!std::isfinite(cutterRadius) || cutterRadius <= 0.0)
        return false;

    if (m_params.hole.collisionEnvelopeMode ==
        HoleCollisionEnvelopeMode::CutterOnly) {
        m_envelopeStations.push_back({0.0, cutterRadius, "cutter"});
        m_maxEnvelopeRadius = cutterRadius;
        return true;
    }

    const double cuttingLength = m_params.tool.cuttingEdgeLength;
    const double overall = m_params.tool.overallLength;
    const bool hasCuttingLength = positiveFinite(cuttingLength);
    const bool hasConsistentOverall =
        positiveFinite(overall) && hasCuttingLength &&
        overall >= cuttingLength;

    if (hasCuttingLength) {
        appendEnvelopeSection(0.0, cuttingLength, cutterRadius, "cutter");
    } else {
        // 刃长未知时，以刀具半径为轴向站位保留最低限度的刀具实体检查。
        // 未知的其余轴向段不做推断，由诊断明确标出。
        m_envelopeStations.push_back(
            {cutterRadius, cutterRadius, "cutter"});
    }

    if (hasConsistentOverall && overall - cuttingLength > kHoleTolerance) {
        const double shankDiameter =
            positiveFinite(m_params.tool.shankDiameter)
                ? m_params.tool.shankDiameter
                : m_params.tool.diameter;
        appendEnvelopeSection(cuttingLength, overall,
                              0.5 * shankDiameter, "shank");
    }

    const double holderDiameter = m_params.tool.holderDiameter;
    const double holderLength = m_params.tool.holderLength;
    const bool hasHolder = hasConsistentOverall &&
        positiveFinite(holderDiameter) && positiveFinite(holderLength);
    if (hasHolder) {
        appendEnvelopeSection(overall, overall + holderLength,
                              0.5 * holderDiameter, "holder");
    }

    const double spindleDiameter =
        m_params.tool.spindleEnvelopeDiameter;
    const double spindleLength =
        m_params.tool.spindleEnvelopeLength;
    if (hasHolder && positiveFinite(spindleDiameter) &&
        positiveFinite(spindleLength)) {
        appendEnvelopeSection(
            overall + holderLength,
            overall + holderLength + spindleLength,
            0.5 * spindleDiameter, "spindle-head");
    }
    for (const EnvelopeStation& station : m_envelopeStations)
        m_maxEnvelopeRadius = std::max(m_maxEnvelopeRadius, station.radius);
    return !m_envelopeStations.empty();
}

void HoleRapidPlanner::appendEnvelopeSection(double startOffset,
                                              double endOffset,
                                              double radius,
                                              const char* component)
{
    const double length = endOffset - startOffset;
    if (!std::isfinite(length) || length <= 0.0 ||
        !std::isfinite(radius) || radius <= 0.0)
        return;
    const double requestedStep =
        std::isfinite(m_params.hole.collisionEnvelopeSampleStep)
        ? m_params.hole.collisionEnvelopeSampleStep : 2.0;
    const double sampleStep = std::clamp(requestedStep, 0.1, 50.0);
    constexpr double kMaximumEnvelopeStationsPerSection = 10000.0;
    const double requestedCount = std::ceil(length / sampleStep);
    const int sampleCount = std::max(
        1, static_cast<int>(std::min(
               requestedCount, kMaximumEnvelopeStationsPerSection)));
    const double coveredLength = length / sampleCount;
    // 半段轴向误差并入半径，使离散胶囊的并集完整覆盖连续圆柱段。
    const double coveredRadius = std::hypot(radius, 0.5 * coveredLength);
    for (int index = 0; index < sampleCount; ++index) {
        m_envelopeStations.push_back({
            startOffset + (index + 0.5) * coveredLength,
            coveredRadius,
            component});
    }
}

void HoleRapidPlanner::appendRapidMove(std::vector<ToolpathMove>& moves,
                                       const gp_Pnt& from,
                                       const gp_Pnt& to,
                                       int operationIndex,
                                       double feed) const
{
    if (from.Distance(to) <= kHoleTolerance)
        return;
    ToolpathMove move;
    move.type = ToolpathMoveType::Rapid;
    move.from = from;
    move.to = to;
    move.loopSequence = operationIndex;
    move.depthIndex = 0;
    move.ring = 0;
    move.feed = feed;
    moves.push_back(std::move(move));
}

} // namespace oap::toolpath::detail
