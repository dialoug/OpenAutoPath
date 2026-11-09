// 孔刀路总调度器：准备输入、按装夹分组、优化当前刀具批次的独立任务顺序。
#include "toolpath/hole/HoleMachining.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <gp_Vec.hxx>

#include "toolpath/hole/DrillTipParameterBinding.h"
#include "toolpath/hole/detail/HoleRapidPlanner.h"
#include "toolpath/hole/detail/HoleStrategyGenerators.h"
#include "toolpath/hole/detail/HoleVisitOptimizer.h"

namespace oap::toolpath {

using detail::HoleFrame;
using detail::HoleRapidPlanner;
using detail::HoleVisitInput;
using detail::buildLargeSpiral;
using detail::buildMediumHelix;
using detail::buildSmallDrill;
using detail::kHoleTolerance;
using detail::makeHoleFrame;
using detail::optimizeHoleVisitOrder;
using detail::repairUnsafeHoleVisitOrder;

namespace {

double routeClearanceOffset(const ToolpathParams& params,
                            const HoleRapidPlanner& planner)
{
    const double safe = std::isfinite(params.depth.safeHeight)
        ? std::max(0.0, params.depth.safeHeight)
        : 0.0;
    const double configured = std::isfinite(params.depth.clearanceHeight)
        ? std::max(safe, params.depth.clearanceHeight)
        : safe;
    return std::max(configured, planner.minimumClearanceOffset());
}

gp_Pnt clearancePoint(const HoleFrame& frame, double offset)
{
    return frame.origin.Translated(frame.axis.Multiplied(-offset));
}

ToolpathDrillCycle* drillCycleOf(HoleMachiningOperation& operation) noexcept
{
    for (ToolpathPrimitive& primitive : operation.primitives) {
        if (primitive.type == ToolpathPrimitiveType::DrillCycle)
            return &primitive.drillCycle;
    }
    return nullptr;
}

const ToolpathDrillCycle* drillCycleOf(
    const HoleMachiningOperation& operation) noexcept
{
    for (const ToolpathPrimitive& primitive : operation.primitives) {
        if (primitive.type == ToolpathPrimitiveType::DrillCycle)
            return &primitive.drillCycle;
    }
    return nullptr;
}

int setupFaceRank(const gp_Vec& inwardAxis,
                  const MoldCoordinateFrame& moldFrame,
                  double minimumAxisDot)
{
    if (inwardAxis.Magnitude() <= kHoleTolerance)
        return 6;
    gp_Vec outwardAxis = inwardAxis;
    outwardAxis.Normalize();
    outwardAxis.Reverse();

    // 排名按刀具所在的模具外表面定义；例如 +Z 面的实际进刀轴是 -Z。
    if (outwardAxis.Dot(gp_Vec(moldFrame.zAxis)) >= minimumAxisDot) return 0;
    if (outwardAxis.Dot(gp_Vec(moldFrame.xAxis)) >= minimumAxisDot) return 1;
    if (outwardAxis.Dot(gp_Vec(moldFrame.yAxis)) >= minimumAxisDot) return 2;
    if (outwardAxis.Dot(gp_Vec(moldFrame.xAxis)) <= -minimumAxisDot) return 3;
    if (outwardAxis.Dot(gp_Vec(moldFrame.yAxis)) <= -minimumAxisDot) return 4;
    if (outwardAxis.Dot(gp_Vec(moldFrame.zAxis)) <= -minimumAxisDot) return 5;
    return 6; // 非六面轴向保持原识别顺序，并排在标准六面之后。
}

} // namespace

double holeMinimumCuttingEdgeLength(
    const HoleMachiningInput& hole,
    HoleMachiningStrategy strategy,
    const ToolpathParams& params) noexcept
{
    const double depth = hole.entry.Distance(hole.bottom);
    if (!std::isfinite(depth) || depth <= 0.0)
        return 0.0;
    switch (strategy) {
        case HoleMachiningStrategy::SmallDrill: {
            const bool through =
                hole.kind == FeatureKind::ThroughHole ||
                (hole.kind == FeatureKind::CountersinkHole &&
                 hole.bottomProfile == HoleBottomProfile::Open);
            const double breakthrough =
                through &&
                std::isfinite(params.hole.throughHoleBreakthrough)
                    ? std::max(0.0, params.hole.throughHoleBreakthrough)
                    : 0.0;
            return depth + breakthrough;
        }
        case HoleMachiningStrategy::MediumHelical: {
            const double pitch =
                std::isfinite(params.hole.helixPitch) &&
                        params.hole.helixPitch > kHoleTolerance
                    ? params.hole.helixPitch
                    : params.depth.stepDown;
            return std::isfinite(pitch) && pitch > kHoleTolerance
                ? std::min(depth, pitch)
                : depth;
        }
        case HoleMachiningStrategy::LargeLayeredSpiral:
            return std::isfinite(params.depth.stepDown) &&
                    params.depth.stepDown > kHoleTolerance
                ? std::min(depth, params.depth.stepDown)
                : depth;
        case HoleMachiningStrategy::Auto:
            return depth;
    }
    return depth;
}

double holeMinimumToolReach(
    const HoleMachiningInput& hole,
    HoleMachiningStrategy strategy,
    const ToolpathParams& params) noexcept
{
    const double depth = hole.entry.Distance(hole.bottom);
    if (!std::isfinite(depth) || depth <= 0.0)
        return 0.0;
    if (strategy != HoleMachiningStrategy::SmallDrill)
        return depth;

    const bool through =
        hole.kind == FeatureKind::ThroughHole ||
        (hole.kind == FeatureKind::CountersinkHole &&
         hole.bottomProfile == HoleBottomProfile::Open);
    const bool fullDiameterDepth = through ||
        hole.bottomProfile == HoleBottomProfile::Open ||
        hole.bottomProfile == HoleBottomProfile::Planar ||
        hole.bottomProfile == HoleBottomProfile::Conical;
    double extension = 0.0;
    if (fullDiameterDepth) {
        if (std::isfinite(params.tool.tipCompensation) &&
            params.tool.tipCompensation > 0.0) {
            extension += params.tool.tipCompensation;
        } else {
            extension += oap::toolpath::drillTipLength(
                params.tool.diameter, params.hole.drillTipAngleDeg);
        }
    }
    if (through &&
        std::isfinite(params.hole.throughHoleBreakthrough)) {
        extension += std::max(0.0, params.hole.throughHoleBreakthrough);
    }
    return depth + extension;
}

void addHoleDiagnostic(HoleMachiningResult& result,
                       ToolpathDiagnosticCode code,
                       ToolpathDiagnosticSeverity severity,
                       std::string message,
                       int sourceIndex,
                       std::string operationId)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = std::move(message);
    diagnostic.operationId = std::move(operationId);
    diagnostic.sourceIndex = sourceIndex;
    if (!diagnostic.message.empty())
        result.warnings.push_back(diagnostic.message);
    result.diagnostics.push_back(std::move(diagnostic));
}

const char* holeMachiningStrategyZh(HoleMachiningStrategy strategy)
{
    switch (strategy) {
        case HoleMachiningStrategy::Auto:
            return "自动孔策略";
        case HoleMachiningStrategy::SmallDrill:
            return "小孔钻削";
        case HoleMachiningStrategy::MediumHelical:
            return "中孔螺旋铣";
        case HoleMachiningStrategy::LargeLayeredSpiral:
            return "大孔分层螺旋清料";
    }
    return "孔加工";
}

HoleMachiningResult HoleMachining::generate(const std::vector<HoleMachiningInput>& holes,
                                             const ToolpathParams& params,
                                             const HoleRapidPlanningContext* rapidContext,
                                             const std::vector<HoleMachiningStrategy>* strategyOverrides,
                                             const ToolpathGenerationControl* control) const
{
    HoleMachiningResult result;
    result.operations.reserve(holes.size());
    std::vector<bool> completed(holes.size(), false);
    std::size_t completedCount = 0;
    auto markCompleted = [&](std::size_t index) {
        if (index >= completed.size() || completed[index])
            return;
        completed[index] = true;
        ++completedCount;
        if (control)
            control->progress(completedCount, holes.size());
    };
    auto cancellationRequested = [&] {
        if (!control || !control->cancelled())
            return false;
        result.cancelled = true;
        if (result.diagnostics.empty() ||
            result.diagnostics.back().code !=
                ToolpathDiagnosticCode::GenerationCancelled) {
            addHoleDiagnostic(
                result, ToolpathDiagnosticCode::GenerationCancelled,
                ToolpathDiagnosticSeverity::Info,
                "hole generation was cancelled");
        }
        return true;
    };
    if (control)
        control->progress(0, holes.size());

    if (!std::isfinite(params.tool.diameter) || params.tool.diameter <= kHoleTolerance) {
        result.skippedCount = static_cast<int>(holes.size());
        addHoleDiagnostic(result, ToolpathDiagnosticCode::InvalidToolDiameter,
                          ToolpathDiagnosticSeverity::Error,
                          "invalid tool diameter, all hole toolpaths skipped");
        return result;
    }

    const double toolRadius = params.tool.radius();
    const double allowance = std::isfinite(params.hole.allowance)
        ? std::max(0.0, params.hole.allowance)
        : 0.0;
    const double smallHoleDiameterTolerance =
        std::isfinite(params.hole.smallHoleDiameterTolerance) &&
                params.hole.smallHoleDiameterTolerance >= 0.0
            ? params.hole.smallHoleDiameterTolerance
            : 0.05;
    const double smallHoleRadialTolerance =
        0.5 * smallHoleDiameterTolerance;
    const double mediumRadiusToolDiameterRatio =
        (std::isfinite(params.hole.mediumHoleRadiusToolDiameterRatio) &&
         params.hole.mediumHoleRadiusToolDiameterRatio > 0.0)
            ? std::min(params.hole.mediumHoleRadiusToolDiameterRatio, 1.0)
            : 0.95;

    if (!std::isfinite(params.hole.sampleAngleDeg)) {
        addHoleDiagnostic(result,
                          ToolpathDiagnosticCode::SamplingParameterAdjusted,
                          ToolpathDiagnosticSeverity::Warning,
                          "invalid hole sampling angle, using the 10 degree default");
    }

    struct PreparedHole {
        size_t originalIndex = 0;
        const HoleMachiningInput* hole = nullptr;
        HoleFrame frame;
        double cutterCenterRadius = 0.0;
        HoleMachiningStrategy strategyOverride = HoleMachiningStrategy::Auto;
    };
    struct AxisGroup {
        gp_Vec inwardAxis;
        std::vector<PreparedHole> holes;
    };

    double axisToleranceDeg = 1.0;
    if (rapidContext && std::isfinite(rapidContext->axisToleranceDeg) &&
        rapidContext->axisToleranceDeg > 0.0) {
        axisToleranceDeg = std::clamp(rapidContext->axisToleranceDeg, 0.01, 45.0);
    }
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double minimumAxisDot = std::cos(axisToleranceDeg * kDegToRad);

    const MoldCoordinateFrame* moldFrame =
        rapidContext && rapidContext->moldFrame && rapidContext->moldFrame->isValid
            ? rapidContext->moldFrame
            : nullptr;
    const bool preserveInputOrder =
        control &&
        control->holeVisitOrderMode ==
            HoleVisitOrderMode::PreserveInput;
    const HoleRapidLinkMode rapidLinkMode = control
        ? control->holeRapidLinkMode
        : HoleRapidLinkMode::DirectWhenSafe;

    // STEP 圆柱 V 方向并不一致。尚未归类的普通厚度轴通孔统一从局部 +Z
    // 加工；已由六面归类或外包络恢复确定入口的通孔必须保留真实装夹侧。
    std::vector<HoleMachiningInput> normalizedHoles = holes;
    if (moldFrame) {
        const gp_Vec moldZ(moldFrame->zAxis);
        for (HoleMachiningInput& hole : normalizedHoles) {
            if (hole.kind != FeatureKind::ThroughHole || hole.preserveEntrySide)
                continue;
            gp_Vec direction(hole.entry, hole.bottom);
            if (direction.Magnitude() <= kHoleTolerance)
                continue;
            direction.Normalize();
            const double zDot = direction.Dot(moldZ);
            if (std::abs(zDot) >= minimumAxisDot && zDot > 0.0)
                std::swap(hole.entry, hole.bottom);
        }
    }

    std::vector<AxisGroup> groups;
    groups.reserve(normalizedHoles.size());

    // 预处理阶段：逐个校验孔并做入口/半径/几何有效性过滤；有效孔按轴向
    // 分组（同轴孔归同一装夹组），为后续按装夹优化访问顺序做准备。
    for (size_t holeIndex = 0; holeIndex < normalizedHoles.size(); ++holeIndex) {
        if (cancellationRequested())
            return result;
        const HoleMachiningInput& hole = normalizedHoles[holeIndex];
        const std::string holeLabel = "hole " + std::to_string(holeIndex + 1);

        if (hole.entryResolution == HoleEntryResolution::Unresolved) {
            ++result.skippedCount;
            addHoleDiagnostic(
                result, ToolpathDiagnosticCode::HoleOpeningUnresolved,
                ToolpathDiagnosticSeverity::Error,
                holeLabel +
                    " skipped because no reliable exterior opening was resolved",
                static_cast<int>(holeIndex));
            markCompleted(holeIndex);
            continue;
        }
        if (hole.entryResolution == HoleEntryResolution::MoldBoundaryFallback) {
            addHoleDiagnostic(
                result, ToolpathDiagnosticCode::HoleOpeningFallback,
                ToolpathDiagnosticSeverity::Warning,
                holeLabel +
                    " uses mold-boundary fallback for the exterior opening",
                static_cast<int>(holeIndex));
        }

        if (!std::isfinite(hole.radius) || hole.radius <= kHoleTolerance) {
            ++result.skippedCount;
            addHoleDiagnostic(result, ToolpathDiagnosticCode::InvalidHoleRadius,
                              ToolpathDiagnosticSeverity::Error,
                              holeLabel + " skipped because radius is invalid",
                              static_cast<int>(holeIndex));
            markCompleted(holeIndex);
            continue;
        }

        HoleFrame frame;
        if (!makeHoleFrame(hole, frame)) {
            ++result.skippedCount;
            addHoleDiagnostic(
                result, ToolpathDiagnosticCode::InvalidHoleGeometry,
                ToolpathDiagnosticSeverity::Error,
                holeLabel + " skipped because depth, endpoint, or axis is invalid",
                static_cast<int>(holeIndex));
            markCompleted(holeIndex);
            continue;
        }

        const double rc = hole.radius - toolRadius - allowance;
        if (hole.kind == FeatureKind::BlindHole) {
            if (toolRadius > hole.radius + smallHoleRadialTolerance) {
                std::ostringstream warning;
                warning << std::fixed << std::setprecision(3)
                        << holeLabel << " skipped because tool radius "
                        << toolRadius << " mm exceeds blind-hole radius "
                        << hole.radius << " mm plus tolerance "
                        << smallHoleRadialTolerance << " mm";
                ++result.skippedCount;
                addHoleDiagnostic(result, ToolpathDiagnosticCode::ToolExceedsHole,
                                  ToolpathDiagnosticSeverity::Error,
                                  warning.str(), static_cast<int>(holeIndex));
                markCompleted(holeIndex);
                continue;
            }
        } else if (!std::isfinite(rc) || rc < -kHoleTolerance) {
            ++result.skippedCount;
            addHoleDiagnostic(
                result, ToolpathDiagnosticCode::ToolExceedsHole,
                ToolpathDiagnosticSeverity::Error,
                holeLabel +
                    " skipped because the tool and allowance exceed the hole radius",
                static_cast<int>(holeIndex));
            markCompleted(holeIndex);
            continue;
        }

        size_t groupIndex = groups.size();
        if (preserveInputOrder) {
            // 只延续相邻同轴孔，避免 A/B/A 轴输入被聚合成 A/A/B。
            if (!groups.empty() &&
                groups.back().inwardAxis.Dot(frame.axis) >= minimumAxisDot) {
                groupIndex = groups.size() - 1;
            }
        } else {
            for (size_t candidate = 0; candidate < groups.size(); ++candidate) {
                if (groups[candidate].inwardAxis.Dot(frame.axis) >=
                    minimumAxisDot) {
                    groupIndex = candidate;
                    break;
                }
            }
        }
        if (groupIndex == groups.size())
            groups.push_back(AxisGroup{frame.axis, {}});
        HoleMachiningStrategy strategyOverride = HoleMachiningStrategy::Auto;
        if (strategyOverrides && holeIndex < strategyOverrides->size())
            strategyOverride = (*strategyOverrides)[holeIndex];
        groups[groupIndex].holes.push_back(
            PreparedHole{holeIndex, &hole, frame, rc, strategyOverride});
    }

    if (moldFrame && !preserveInputOrder) {
        // 按刀具所在模具面固定装夹顺序；inwardAxis 与表面方向相反。
        std::stable_sort(groups.begin(), groups.end(),
                         [&](const AxisGroup& a, const AxisGroup& b) {
            return setupFaceRank(a.inwardAxis, *moldFrame, minimumAxisDot) <
                   setupFaceRank(b.inwardAxis, *moldFrame, minimumAxisDot);
        });

        for (AxisGroup& group : groups) {
            if (group.holes.size() < 2)
                continue;
            HoleMachiningResult scratchResult;
            HoleRapidPlanner clearancePlanner(
                rapidContext, params, scratchResult, rapidLinkMode, control);
            const double offset = routeClearanceOffset(params, clearancePlanner);
            std::vector<HoleVisitInput> inputs;
            inputs.reserve(group.holes.size());
            for (const PreparedHole& prepared : group.holes) {
                inputs.push_back(HoleVisitInput{
                    prepared.originalIndex,
                    prepared.hole->entry,
                    clearancePoint(prepared.frame, offset)});
            }
            const std::vector<std::size_t> order = optimizeHoleVisitOrder(
                inputs, *moldFrame, params.hole.startCorner, result, control);
            if (cancellationRequested())
                return result;
            std::vector<PreparedHole> optimized;
            optimized.reserve(group.holes.size());
            for (std::size_t index : order)
                optimized.push_back(std::move(group.holes[index]));
            group.holes = std::move(optimized);
        }
    }

    for (AxisGroup& group : groups) {
        const size_t operationsBeforeSetup = result.operations.size();
        const int setupIndex = result.setupCount;
        bool hasCurrentClearance = false;
        gp_Pnt currentClearance;
        HoleRapidPlanner rapidPlanner(
            rapidContext, params, result, rapidLinkMode, control);

        if (!preserveInputOrder &&
            group.holes.size() > 2 && rapidContext &&
            rapidPlanner.acceptAxis(group.inwardAxis, "setup") &&
            rapidPlanner.canPlanLinks()) {
            const double offset = routeClearanceOffset(params, rapidPlanner);
            std::vector<HoleVisitInput> inputs;
            inputs.reserve(group.holes.size());
            for (const PreparedHole& prepared : group.holes) {
                inputs.push_back(HoleVisitInput{
                    prepared.originalIndex,
                    prepared.hole->entry,
                    clearancePoint(prepared.frame, offset)});
            }
            const std::vector<std::size_t> repaired =
                repairUnsafeHoleVisitOrder(
                    inputs, rapidPlanner, result, control);
            if (cancellationRequested())
                return result;
            std::vector<PreparedHole> reordered;
            reordered.reserve(group.holes.size());
            for (std::size_t index : repaired)
                reordered.push_back(std::move(group.holes[index]));
            group.holes = std::move(reordered);
        }

        for (const PreparedHole& prepared : group.holes) {
            if (cancellationRequested())
                return result;
            const HoleMachiningInput& hole = *prepared.hole;
            const HoleFrame& frame = prepared.frame;
            const double rc = prepared.cutterCenterRadius;
            const std::string holeLabel =
                "hole " + std::to_string(prepared.originalIndex + 1);

            const bool hasAcceptedOperation = hasCurrentClearance;
            if (hasAcceptedOperation &&
                !rapidPlanner.acceptAxis(frame.axis, holeLabel)) {
                ++result.skippedCount;
                markCompleted(prepared.originalIndex);
                continue;
            }

            const int operationIndex = static_cast<int>(result.operations.size());
            const gp_Pnt* previousClearance =
                hasCurrentClearance ? &currentClearance : nullptr;
            const int unsafeLinksBeforeBuild = result.unsafeRapidLinkCount;

            HoleMachiningOperation op;
            HoleMachiningStrategy strategy = prepared.strategyOverride;
            // 自动策略选择：盲孔一律钻削；铣削策略必须保留径向余量——铣刀
            // 中心与孔壁间距不足时退回钻削，孔半径相对刀具不大时用螺旋铣，
            // 其余大孔用分层螺旋清料。
            if (strategy == HoleMachiningStrategy::Auto) {
                if (hole.kind == FeatureKind::BlindHole)
                    strategy = HoleMachiningStrategy::SmallDrill;
                else if (rc <= smallHoleRadialTolerance + kHoleTolerance)
                    strategy = HoleMachiningStrategy::SmallDrill;
                else if (hole.radius <=
                         mediumRadiusToolDiameterRatio * params.tool.diameter)
                    strategy = HoleMachiningStrategy::MediumHelical;
                else
                    strategy = HoleMachiningStrategy::LargeLayeredSpiral;
            }
            if (strategy != HoleMachiningStrategy::SmallDrill && rc <= kHoleTolerance) {
                ++result.skippedCount;
                addHoleDiagnostic(
                    result, ToolpathDiagnosticCode::ForcedStrategyNoClearance,
                    ToolpathDiagnosticSeverity::Error,
                    holeLabel +
                        " skipped because the forced milling strategy has no radial clearance",
                    static_cast<int>(prepared.originalIndex));
                markCompleted(prepared.originalIndex);
                continue;
            }

            if (strategy == HoleMachiningStrategy::SmallDrill) {
                op = buildSmallDrill(hole, frame, operationIndex, params,
                                     previousClearance, &rapidPlanner);
                if (op.moves.empty()) {
                    if (cancellationRequested())
                        return result;
                    ++result.skippedCount;
                    if (result.unsafeRapidLinkCount == unsafeLinksBeforeBuild) {
                        addHoleDiagnostic(
                            result, ToolpathDiagnosticCode::EmptyHoleOperation,
                            ToolpathDiagnosticSeverity::Error,
                            holeLabel + " skipped because small-hole moves are empty",
                            static_cast<int>(prepared.originalIndex));
                    }
                    markCompleted(prepared.originalIndex);
                    continue;
                }
                ++result.smallCount;
            } else if (strategy == HoleMachiningStrategy::MediumHelical) {
                op = buildMediumHelix(hole, frame, toolRadius, operationIndex, params,
                                      result, previousClearance, &rapidPlanner);
                if (op.moves.empty()) {
                    if (cancellationRequested())
                        return result;
                    ++result.skippedCount;
                    markCompleted(prepared.originalIndex);
                    continue;
                }
                ++result.mediumCount;
            } else {
                op = buildLargeSpiral(hole, frame, toolRadius, operationIndex, params,
                                      result, previousClearance, &rapidPlanner);
                if (op.moves.empty()) {
                    if (cancellationRequested())
                        return result;
                    ++result.skippedCount;
                    markCompleted(prepared.originalIndex);
                    continue;
                }
                ++result.largeCount;
            }

            if (!op.moves.empty()) {
                if (!hasAcceptedOperation &&
                    !rapidPlanner.acceptAxis(frame.axis, holeLabel)) {
                    ++result.skippedCount;
                    markCompleted(prepared.originalIndex);
                    continue;
                }
                if (!rapidPlanner.validateNonCuttingMoves(
                        op.moves, op.operationId)) {
                    if (cancellationRequested())
                        return result;
                    ++result.skippedCount;
                    markCompleted(prepared.originalIndex);
                    continue;
                }
                if (!rapidPlanner.validateCuttingGeometry(
                        op.moves, op.primitives, op.operationId)) {
                    if (cancellationRequested())
                        return result;
                    ++result.skippedCount;
                    markCompleted(prepared.originalIndex);
                    continue;
                }
                op.setupIndex = setupIndex;
                op.setupFace = hole.setupFace;
                op.sourceHoleIndex = static_cast<int>(prepared.originalIndex);
                op.minimumCuttingEdgeLength =
                    holeMinimumCuttingEdgeLength(
                        hole, op.strategy, params);
                op.minimumToolReach = op.entry.Distance(
                    op.hasCommandedBottom
                        ? op.commandedBottom
                        : op.bottom);
                currentClearance = op.moves.back().to;
                hasCurrentClearance = true;
                result.operations.push_back(std::move(op));
            }
            markCompleted(prepared.originalIndex);
        }

        // 仅当相邻孔使用同一循环、R 平面共面且完整刀具包络的直连已经通过
        // 碰撞检查时，才把前一孔改为 G99。最后一孔和循环切换点保持 G98。
        if (params.hole.allowG99RetractOptimization &&
            rapidPlanner.canPlanLinks()) {
            const std::size_t operationsAfterSetup = result.operations.size();
            for (std::size_t index = operationsBeforeSetup;
                 index + 1 < operationsAfterSetup; ++index) {
                ToolpathDrillCycle* current =
                    drillCycleOf(result.operations[index]);
                const ToolpathDrillCycle* next =
                    drillCycleOf(result.operations[index + 1]);
                if (!current || !next || current->type != next->type)
                    continue;
                const gp_Vec rLink(current->retract, next->retract);
                if (std::abs(rLink.Dot(group.inwardAxis)) > 1.0e-6)
                    continue;
                const detail::RapidLinkPlan link = rapidPlanner.evaluateLink(
                    current->retract, next->retract);
                if (link.safe && !link.lifted) {
                    current->retractMode =
                        ToolpathDrillRetractMode::RetractPlane;
                }
            }
        }

        if (result.operations.size() > operationsBeforeSetup)
            ++result.setupCount;
    }

    return result;
}

} // namespace oap::toolpath
