// 验证孔轴、刀具径向余量、中心钻切削段和最终退刀位置。
// 属于 oap_toolpath_hole 模块内部实现：在通用参数/移动/原语/刀具验证之上
// 叠加孔工序专属结构检查，供刀路生成后与预览前使用。
#include "toolpath/hole/HoleToolpathValidator.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <gp_Vec.hxx>

#include "toolpath/core/validation/ToolpathMoveValidator.h"
#include "toolpath/core/validation/ToolpathParameterValidator.h"
#include "toolpath/core/validation/ToolpathPrimitiveValidator.h"
#include "toolpath/core/validation/ToolpathToolValidator.h"

namespace oap::toolpath {

void validateHoleDrillReentries(
    const std::vector<ToolpathMove>& moves,
    const gp_Vec& inwardAxis,
    const std::string& operationId,
    ToolpathValidationReport& report)
{
    constexpr double tolerance = 1.0e-7;
    const double axisLength = inwardAxis.Magnitude();
    const bool validAxis = std::isfinite(axisLength) && axisLength > tolerance;
    const gp_Vec axis = validAxis ? inwardAxis / axisLength : gp_Vec(0, 0, 1);
    bool covered = false;
    gp_Pnt origin;
    double low = 0.0, high = 0.0;
    const auto onLine = [&](const gp_Pnt& point) {
        const gp_Vec offset(origin, point);
        return offset.Crossed(axis).Magnitude() <= tolerance;
    };
    for (std::size_t index = 0; index < moves.size(); ++index) {
        const auto& move = moves[index];
        const bool finite = finiteToolpathPoint(move.from) && finiteToolpathPoint(move.to);
        if (move.type == ToolpathMoveType::Cut) {
            const gp_Vec travel(move.from, move.to);
            if (!validAxis || !finite || travel.Dot(axis) <= tolerance ||
                travel.Crossed(axis).Magnitude() > tolerance) {
                covered = false;
                continue;
            }
            const double from = covered ? gp_Vec(origin, move.from).Dot(axis) : 0.0;
            const double to = covered ? gp_Vec(origin, move.to).Dot(axis) : 0.0;
            if (covered && onLine(move.from) && onLine(move.to) &&
                from <= high + tolerance && to >= low - tolerance) {
                low = std::min(low, from);
                high = std::max(high, to);
            } else {
                // Retain only the most recent continuous interval. Disjoint
                // cuts must not turn the gap between them into clearance.
                origin = move.from;
                low = 0.0;
                high = travel.Dot(axis);
                covered = true;
            }
            continue;
        }
        if (move.intent != ToolpathMoveIntent::DrillCycleReentry)
            continue;
        bool valid = validAxis && finite && covered &&
                     move.type == ToolpathMoveType::Rapid && move.depthIndex > 0;
        if (valid) {
            const double from = gp_Vec(origin, move.from).Dot(axis);
            const double to = gp_Vec(origin, move.to).Dot(axis);
            valid = onLine(move.from) && onLine(move.to) && to > from + tolerance &&
                    from >= low - tolerance && to <= high + tolerance;
        }
        if (!valid) {
            addToolpathIssue(report, ToolpathIssueSeverity::Error,
                            "unproven_drill_reentry",
                            "drill reentry must retrace a previously cut continuous centerline interval",
                            operationId, static_cast<int>(index));
            return;
        }
    }
}

ToolpathValidationReport HoleToolpathValidator::validate(
    const std::vector<HoleMachiningOperation>& operations,
    const ToolpathParams& params) const
{
    ToolpathValidationReport report;
    validateToolpathParameters(params, report);

    if (operations.empty()) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error,
            "empty_operation_set",
            "no hole drilling operations were generated");
        return report;
    }

    std::unordered_set<std::string> operationIds;
    for (const HoleMachiningOperation& operation : operations) {
        const gp_Pnt& commandedBottom = operation.hasCommandedBottom
            ? operation.commandedBottom
            : operation.bottom;
        if (operation.operationId.empty()) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Warning,
                "missing_operation_id",
                "hole operation has no operation id");
        } else if (!operationIds.insert(
                       operation.operationId).second) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Warning,
                "duplicate_operation_id",
                "duplicate hole operation id",
                operation.operationId);
        }

        const bool finiteHole =
            finiteToolpathPoint(operation.entry) &&
            finiteToolpathPoint(operation.bottom) &&
            finiteToolpathPoint(commandedBottom);
        if (!finiteHole) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "non_finite_hole",
                "hole operation has non-finite entry, nominal bottom, or commanded bottom point",
                operation.operationId);
        }
        const double axisLength = finiteHole
            ? operation.entry.Distance(operation.bottom)
            : 0.0;
        if (finiteHole &&
            (!std::isfinite(axisLength) ||
             axisLength <= kMinimumToolpathMoveLength)) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "degenerate_hole_axis",
                "hole entry and bottom are identical or near identical",
                operation.operationId);
        }
        if (!std::isfinite(operation.radius) ||
            operation.radius <= 0.0) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Warning,
                "invalid_hole_radius",
                "hole radius is not finite and positive",
                operation.operationId);
        }
        if (operation.tool.valid) {
            if (operation.tool.toolNumber <= 0 ||
                !std::isfinite(operation.tool.diameter) ||
                operation.tool.diameter <= 0.0) {
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "invalid_operation_tool",
                    "operation tool must have a positive T number and diameter",
                    operation.operationId);
            } else if (
                operation.strategy !=
                    HoleMachiningStrategy::SmallDrill &&
                0.5 * operation.tool.diameter +
                        std::max(0.0, params.hole.allowance) >=
                    operation.radius - 1.0e-7) {
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "operation_tool_has_no_radial_clearance",
                    "operation tool diameter leaves no radial clearance for the milling strategy",
                    operation.operationId);
            }
        }
        // 中心定位只按浅点钻深度触碰工件；其余阶段分别验证策略吃刀刃长
        // 与刀具总可达长度，避免把深腔总深度误作分层铣削刃长。
        if (operation.phase != HoleMachiningPhase::CenterSpot) {
            validateCuttingEdgeRequirement(
                operation.tool.valid ? &operation.tool : nullptr,
                params.tool.cuttingEdgeLength,
                operation.minimumCuttingEdgeLength,
                operation.operationId, report);
            validateToolReachRequirement(
                operation.tool.valid ? &operation.tool : nullptr,
                params.tool.overallLength,
                operation.minimumToolReach,
                operation.operationId, report);
        }

        const ToolpathMoveStats stats = validateToolpathMoves(
            operation.moves, operation.operationId, report);
        validateHoleDrillReentries(operation.moves,
                                  gp_Vec(operation.entry, operation.bottom),
                                  operation.operationId, report);
        validateToolpathPrimitives(
            operation.primitives, operation.operationId, report);

        if (finiteHole &&
            axisLength > kMinimumToolpathMoveLength &&
            !operation.moves.empty() &&
            finiteToolpathPoint(operation.moves.back().to)) {
            gp_Vec axis(operation.entry, operation.bottom);
            axis.Normalize();
            const double finalAxialDepth =
                gp_Vec(operation.entry, operation.moves.back().to)
                    .Dot(axis);
            if (finalAxialDepth > 1.0e-6) {
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "unsafe_hole_end",
                    "hole operation ends below the entry plane",
                    operation.operationId,
                    static_cast<int>(operation.moves.size() - 1));
            }
        }

        if (operation.strategy !=
            HoleMachiningStrategy::SmallDrill) {
            continue;
        }
        // 小孔钻削专属检查：固定循环必须恰好（或至少）有一条沿孔轴的中心
        // 切削段，刀尖目标必须落在孔轴上且深于全径深度；直接钻削的第一刀
        // 还必须从入口直达刀尖目标。
        const ToolpathDrillCycle* drillCycle = nullptr;
        for (const ToolpathPrimitive& primitive : operation.primitives) {
            if (primitive.type == ToolpathPrimitiveType::DrillCycle) {
                drillCycle = &primitive.drillCycle;
                break;
            }
        }
        const bool peckCycle = drillCycle &&
            (drillCycle->type == ToolpathDrillCycleType::Peck ||
             drillCycle->type == ToolpathDrillCycleType::ChipBreak);
        if ((!peckCycle && stats.cut != 1) ||
            (peckCycle && stats.cut < 1)) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "invalid_small_drill_cut_count",
                peckCycle
                    ? "peck drilling must contain at least one centerline cut"
                    : "direct small-hole drilling must contain exactly one centerline cut",
                operation.operationId);
            continue;
        }

        if (finiteHole && axisLength > kMinimumToolpathMoveLength) {
            gp_Vec axis(operation.entry, operation.bottom);
            axis.Normalize();
            const gp_Vec targetVector(operation.entry, commandedBottom);
            const double commandedDepth = targetVector.Dot(axis);
            const gp_Vec radial =
                targetVector - axis.Multiplied(commandedDepth);
            if (commandedDepth + 1.0e-6 < axisLength ||
                radial.Magnitude() > 1.0e-6) {
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "invalid_drill_depth_target",
                    "drill tip target must remain on the hole axis at or below the full-diameter depth",
                    operation.operationId);
            }
        }

        const ToolpathMove* firstCut = nullptr;
        const ToolpathMove* targetCut = nullptr;
        for (const ToolpathMove& move : operation.moves) {
            if (move.type != ToolpathMoveType::Cut)
                continue;
            if (!firstCut)
                firstCut = &move;
            if (move.to.Distance(commandedBottom) <= 1.0e-6)
                targetCut = &move;
        }
        if (!peckCycle && firstCut && finiteHole &&
            (!finiteToolpathPoint(firstCut->from) ||
             !finiteToolpathPoint(firstCut->to) ||
             firstCut->from.Distance(operation.entry) > 1.0e-6 ||
             firstCut->to.Distance(commandedBottom) > 1.0e-6 ||
             firstCut->depthIndex != 1)) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "invalid_small_drill_centerline",
                "direct small-hole cut must run from entry to the commanded drill-tip target",
                operation.operationId);
        } else if (peckCycle && finiteHole && !targetCut) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "invalid_peck_drill_target",
                "peck drilling must finish a centerline cut at the commanded drill-tip target",
                operation.operationId);
        }
    }
    return report;
}

} // namespace oap::toolpath
