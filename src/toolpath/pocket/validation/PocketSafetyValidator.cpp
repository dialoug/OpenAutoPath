// 对切削和进刀段采样刀具中心，确认其始终落在考虑刀具半径后的安全可达区域内。
#include "toolpath/pocket/validation/PocketSafetyValidator.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <sstream>
#include <vector>

#include "toolpath/core/ToolpathParallel.h"

#include "toolpath/pocket/planning/detail/PocketCurveSemanticBuilder.h"
#include "toolpath/pocket/planning/detail/PocketSafetyGeometry.h"

namespace oap::toolpath {

namespace {

constexpr std::size_t kMaximumSamplesPerOperation = 200000;
constexpr std::size_t kParallelSampleThreshold = 8192;

enum class SafetyViolationKind : std::uint8_t {
    None,
    RootBoundary,
    ProtectedMaterial,
    Cancelled
};

struct PreparedSafetyMove {
    std::size_t moveIndex = 0;
    std::size_t segmentSamples = 0;
    bool skipFirstSample = false;
    bool checkRootBoundary = false;
    bool checkProtectedMaterial = false;
    gp_Pnt2d from;
    gp_Pnt2d to;
};

struct SafetyViolation {
    SafetyViolationKind kind = SafetyViolationKind::None;
    std::size_t sample = 0;
    gp_Pnt2d point;
};

// 对单个预处理的移动段做逐点采样校验：跳过首样本（避免与上一段重复
// 校验同一点），依次检查根边界与受保护材料；每 256 个样本检查一次取消。
SafetyViolation validatePreparedMove(
    const PreparedSafetyMove& prepared,
    const ToolpathMove& move,
    const detail::PocketSafetyArea& safetyArea,
    const std::function<bool()>& isCancellationRequested)
{
    const std::size_t firstSample =
        prepared.skipFirstSample ? 1u : 0u;
    const double dx = prepared.to.X() - prepared.from.X();
    const double dy = prepared.to.Y() - prepared.from.Y();
    for (std::size_t sample = firstSample;
         sample <= prepared.segmentSamples; ++sample) {
        if ((sample & 0xffu) == 0u &&
            isCancellationRequested &&
            isCancellationRequested()) {
            return {SafetyViolationKind::Cancelled};
        }
        const double t = static_cast<double>(sample) /
                         static_cast<double>(prepared.segmentSamples);
        const gp_Pnt2d point(
            prepared.from.X() + t * dx,
            prepared.from.Y() + t * dy);
        if (prepared.checkRootBoundary &&
            !safetyArea.rootClear(point)) {
            return {
                SafetyViolationKind::RootBoundary,
                sample, point};
        }
        if (prepared.checkProtectedMaterial &&
            !safetyArea.protectedMaterialClear(
                point, move.depthIndex)) {
            return {
                SafetyViolationKind::ProtectedMaterial,
                sample, point};
        }
    }
    return {};
}

void addSamplingLimitError(
    PocketSafetyValidationResult& result,
    const PocketToolpathOperation& operation)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code =
        ToolpathDiagnosticCode::PocketSafetySamplingLimited;
    diagnostic.severity = ToolpathDiagnosticSeverity::Error;
    diagnostic.message =
        "pocket safety validation is incomplete: sampling exceeds the operation budget";
    diagnostic.operationId = operation.operationId;
    result.diagnostics.push_back(std::move(diagnostic));

    ToolpathIssue issue;
    issue.severity = ToolpathIssueSeverity::Error;
    issue.code = toolpathDiagnosticCodeName(
        ToolpathDiagnosticCode::PocketSafetySamplingLimited);
    issue.message = "pocket safety validation is incomplete: sampling exceeds the operation budget";
    issue.operationId = operation.operationId;
    result.report.issues.push_back(std::move(issue));
    ++result.report.errorCount;
}

void addViolation(PocketSafetyValidationResult& result,
                  const PocketToolpathOperation& operation,
                  const ToolpathMove& move,
                  int moveIndex,
                  const std::string& message)
{
    ToolpathIssue issue;
    issue.severity = ToolpathIssueSeverity::Error;
    issue.code = toolpathDiagnosticCodeName(
        ToolpathDiagnosticCode::PocketBoundaryViolation);
    issue.message = message;
    issue.operationId = operation.operationId;
    issue.moveIndex = moveIndex;
    result.report.issues.push_back(std::move(issue));
    ++result.report.errorCount;

    ToolpathDiagnostic diagnostic;
    diagnostic.code = ToolpathDiagnosticCode::PocketBoundaryViolation;
    diagnostic.severity = ToolpathDiagnosticSeverity::Error;
    diagnostic.message = message;
    diagnostic.operationId = operation.operationId;
    diagnostic.depthIndex = move.depthIndex;
    diagnostic.moveIndex = moveIndex;
    result.diagnostics.push_back(std::move(diagnostic));
}

} // namespace

PocketSafetyValidationResult PocketSafetyValidator::validate(
    const PocketRegion& region,
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    const std::function<bool()>& isCancellationRequested) const
{
    // 主流程：① 推导安全余量与采样间距，构造 PocketSafetyArea →
    // ② 把 Cut/Plunge 移动预处理成 PreparedSafetyMove（含采样数预算） →
    // ③ 并行/串行采样校验 → ④ 圆弧图元解析安全判定 → ⑤ 汇总报告。
    // 采样超预算时保留已检查部分的结果，但以错误阻止未完成验证的工序提交。
    PocketSafetyValidationResult result;
    if (isCancellationRequested && isCancellationRequested()) {
        result.cancelled = true;
        return result;
    }
    if (region.boundary.points.size() < 3 || operation.clearing.moves.empty())
        return result;

    int deepestCutIndex = -1;
    for (const ToolpathMove& move : operation.clearing.moves) {
        if (move.type == ToolpathMoveType::Cut)
            deepestCutIndex = std::max(deepestCutIndex, move.depthIndex);
    }
    if (deepestCutIndex < 0)
        return result;

    const double requiredMargin =
        params.tool.radius() + std::max(0.0, params.clearing.extraOffset);
    const double quantization =
        std::pow(10.0, -std::clamp(params.clearing.precision, 0, 8));
    const double tolerance = std::max(0.02, 4.0 * quantization);
    const double sampleSpacing = std::clamp(
        requiredMargin > 0.0 ? requiredMargin / 3.0 : 0.25,
        0.05, 0.5);
    const detail::PocketSafetyArea safetyArea(
        region, params,
        operation.clearing.protectedMaterialLayers);

    std::vector<PreparedSafetyMove> preparedMoves;
    preparedMoves.reserve(std::min<std::size_t>(
        operation.clearing.moves.size(),
        kMaximumSamplesPerOperation / 2u));
    std::size_t sampled = 0;
    std::size_t sampleChecks = 0;
    bool samplingLimited = false;
    bool previousEndpointValidated = false;
    gp_Pnt previousEndpoint;
    int previousDepthIndex = -1;
    bool previousCheckedRootBoundary = false;
    for (std::size_t moveIndex = 0;
         moveIndex < operation.clearing.moves.size(); ++moveIndex) {
        if ((moveIndex & 0xffu) == 0u &&
            isCancellationRequested &&
            isCancellationRequested()) {
            result.cancelled = true;
            return result;
        }
        const ToolpathMove& move = operation.clearing.moves[moveIndex];
        if (move.type != ToolpathMoveType::Cut &&
            move.type != ToolpathMoveType::Plunge) {
            continue;
        }
        // 入口根轮廓只约束最终深度；每一层的真实材料障碍则始终校验。
        const bool checkRootBoundary =
            !operation.clearing.usedPerLayerSections ||
            move.depthIndex == deepestCutIndex;
        const bool checkProtectedMaterial =
            operation.clearing.usedProtectedMaterialSections;
        if (!checkRootBoundary &&
            !checkProtectedMaterial) {
            continue;
        }

        const double length = move.from.Distance(move.to);
        const double requestedSamples = std::max(
            1.0, std::ceil(length / sampleSpacing));
        // 转为整数前检查有限性和剩余预算，避免超范围转换及加法溢出。
        const std::size_t remaining = kMaximumSamplesPerOperation - sampled;
        if (!std::isfinite(length) || !std::isfinite(requestedSamples) ||
            requestedSamples >= static_cast<double>(remaining)) {
            samplingLimited = true;
            break;
        }
        const auto segmentSamples = static_cast<std::size_t>(requestedSamples);

        const bool skipRepeatedFirstSample =
            previousEndpointValidated &&
            previousDepthIndex == move.depthIndex &&
            previousCheckedRootBoundary ==
                checkRootBoundary &&
            previousEndpoint.SquareDistance(
                move.from) <= 1.0e-24;
        preparedMoves.push_back({
            moveIndex,
            segmentSamples,
            skipRepeatedFirstSample,
            checkRootBoundary,
            checkProtectedMaterial,
            detail::projectToPocketPlane(move.from, region.frame),
            detail::projectToPocketPlane(move.to, region.frame)});
        sampled += segmentSamples + 1;
        sampleChecks += segmentSamples + 1 -
                        (skipRepeatedFirstSample ? 1u : 0u);
        previousEndpointValidated = true;
        previousEndpoint = move.to;
        previousDepthIndex = move.depthIndex;
        previousCheckedRootBoundary =
            checkRootBoundary;
    }

    std::vector<SafetyViolation> violations(preparedMoves.size());
    const auto validateMove = [&](std::size_t preparedIndex) {
        const PreparedSafetyMove& prepared =
            preparedMoves[preparedIndex];
        violations[preparedIndex] = validatePreparedMove(
            prepared,
            operation.clearing.moves[prepared.moveIndex],
            safetyArea, isCancellationRequested);
    };
    if (sampleChecks >= kParallelSampleThreshold &&
        preparedMoves.size() > 1u) {
        toolpathParallelForBudgeted(preparedMoves.size(), validateMove);
    } else {
        for (std::size_t preparedIndex = 0;
             preparedIndex < preparedMoves.size(); ++preparedIndex) {
            validateMove(preparedIndex);
        }
    }

    const auto violation = std::find_if(
        violations.begin(), violations.end(),
        [](const SafetyViolation& candidate) {
            return candidate.kind != SafetyViolationKind::None;
        });
    if ((violation != violations.end() &&
         violation->kind == SafetyViolationKind::Cancelled) ||
        (isCancellationRequested && isCancellationRequested())) {
        result.cancelled = true;
        return result;
    }
    if (violation != violations.end()) {
        const std::size_t preparedIndex =
            static_cast<std::size_t>(
                std::distance(violations.begin(), violation));
        const PreparedSafetyMove& prepared =
            preparedMoves[preparedIndex];
        const ToolpathMove& move =
            operation.clearing.moves[prepared.moveIndex];
        std::ostringstream message;
        if (violation->kind ==
            SafetyViolationKind::RootBoundary) {
            message << "cut/entry move leaves the cutter-center safe area at sample "
                    << violation->sample << '/'
                    << prepared.segmentSamples;
        } else {
            message << "cut/entry move intersects the protected "
                       "workpiece-material envelope at sample "
                    << violation->sample << '/'
                    << prepared.segmentSamples
                    << " (move=" << prepared.moveIndex
                    << ", depth=" << move.depthIndex
                    << ", x=" << violation->point.X()
                    << ", y=" << violation->point.Y()
                    << ", required=" << requiredMargin
                    << ", tolerance=" << tolerance
                    << ", ring=" << move.ring << ")";
        }
        addViolation(
            result, operation, move,
            static_cast<int>(prepared.moveIndex),
            message.str());
    }
    if (!result.report.ok())
        return result;

    if (samplingLimited) {
        addSamplingLimitError(result, operation);
        return result;
    }

    std::vector<std::size_t> arcPrimitiveIndices;
    arcPrimitiveIndices.reserve(
        operation.clearing.primitives.size());
    for (std::size_t primitiveIndex = 0;
         primitiveIndex < operation.clearing.primitives.size();
         ++primitiveIndex) {
        const ToolpathPrimitive& primitive =
            operation.clearing.primitives[primitiveIndex];
        if (primitive.type != ToolpathPrimitiveType::CircularArc ||
            (primitive.arc.moveType != ToolpathMoveType::Cut &&
             primitive.arc.moveType != ToolpathMoveType::Plunge)) {
            continue;
        }
        arcPrimitiveIndices.push_back(primitiveIndex);
    }

    std::vector<std::uint8_t> unsafeArcs(
        arcPrimitiveIndices.size(), 0u);
    const auto validateArc = [&](std::size_t arcIndex) {
        if (isCancellationRequested && isCancellationRequested())
            return;
        const ToolpathPrimitive& primitive =
            operation.clearing.primitives[
                arcPrimitiveIndices[arcIndex]];
        unsafeArcs[arcIndex] =
            detail::isPocketArcCutterCenterSafe(
                primitive.arc, primitive.context,
                region, params, safetyArea,
                isCancellationRequested)
            ? 0u
            : 1u;
    };
    toolpathParallelForBudgeted(arcPrimitiveIndices.size(), validateArc);

    if (isCancellationRequested && isCancellationRequested()) {
        result.cancelled = true;
        return result;
    }

    const auto unsafeArc = std::find(
        unsafeArcs.begin(), unsafeArcs.end(), 1u);
    if (unsafeArc != unsafeArcs.end()) {
        const std::size_t arcIndex =
            static_cast<std::size_t>(
                std::distance(unsafeArcs.begin(), unsafeArc));
        const ToolpathPrimitive& primitive =
            operation.clearing.primitives[
                arcPrimitiveIndices[arcIndex]];
        ToolpathMove reference;
        if (!primitive.fallbackMoves.empty()) {
            reference = primitive.fallbackMoves.front();
        } else {
            reference.type = primitive.arc.moveType;
            reference.from = primitive.arc.from;
            reference.to = primitive.arc.to;
            reference.loopSequence = primitive.context.loopSequence;
            reference.depthIndex = primitive.context.depthIndex;
            reference.ring = primitive.context.ring;
            reference.intent = primitive.context.intent;
            reference.feed = primitive.arc.feed;
        }
        addViolation(
            result, operation, reference, -1,
            "circular primitive leaves the cutter-center safe area");
    }
    return result;
}

} // namespace oap::toolpath
