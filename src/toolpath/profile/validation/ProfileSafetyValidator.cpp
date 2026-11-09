// ProfileSafetyValidator 实现（oap_toolpath_profile）。
// 沿刀轴对直壁 2.5D 成品包络做分段刀具/刀杆/刀柄碰撞采样，见
// ProfileSafetyValidator.h。
#include "toolpath/profile/validation/ProfileSafetyValidator.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "toolpath/pocket/geometry/OcctAreaAdapter.h"

namespace oap::toolpath {
namespace {

constexpr double kAxisTolerance = 1.0e-6;

struct ToolSection {
    const char* name = "tool";
    double axialStart = 0.0;
    double axialEnd = 0.0;
    double radius = 0.0;
    bool cutter = false;
};

double pointSegmentDistance(
    const gp_Pnt2d& point,
    const gp_Pnt2d& a,
    const gp_Pnt2d& b)
{
    const double dx = b.X() - a.X();
    const double dy = b.Y() - a.Y();
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 1.0e-24)
        return point.Distance(a);
    const double t = std::clamp(
        ((point.X() - a.X()) * dx +
         (point.Y() - a.Y()) * dy) / lengthSquared,
        0.0, 1.0);
    return std::hypot(
        point.X() - (a.X() + t * dx),
        point.Y() - (a.Y() + t * dy));
}

bool pointInsideBoundary(
    const gp_Pnt2d& point,
    const std::vector<gp_Pnt2d>& polygon)
{
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1;
         i < polygon.size(); j = i++) {
        const gp_Pnt2d& a = polygon[i];
        const gp_Pnt2d& b = polygon[j];
        const bool crosses =
            ((a.Y() > point.Y()) != (b.Y() > point.Y())) &&
            point.X() <
                (b.X() - a.X()) * (point.Y() - a.Y()) /
                    (b.Y() - a.Y()) + a.X();
        if (crosses)
            inside = !inside;
    }
    return inside;
}

double cross2d(
    const gp_Pnt2d& a,
    const gp_Pnt2d& b,
    const gp_Pnt2d& c)
{
    return (b.X() - a.X()) * (c.Y() - a.Y()) -
        (b.Y() - a.Y()) * (c.X() - a.X());
}

bool onSegment(
    const gp_Pnt2d& point,
    const gp_Pnt2d& a,
    const gp_Pnt2d& b,
    double tolerance)
{
    return std::abs(cross2d(a, b, point)) <= tolerance &&
        point.X() >= std::min(a.X(), b.X()) - tolerance &&
        point.X() <= std::max(a.X(), b.X()) + tolerance &&
        point.Y() >= std::min(a.Y(), b.Y()) - tolerance &&
        point.Y() <= std::max(a.Y(), b.Y()) + tolerance;
}

bool segmentsIntersect(
    const gp_Pnt2d& a,
    const gp_Pnt2d& b,
    const gp_Pnt2d& c,
    const gp_Pnt2d& d,
    double tolerance)
{
    const double abC = cross2d(a, b, c);
    const double abD = cross2d(a, b, d);
    const double cdA = cross2d(c, d, a);
    const double cdB = cross2d(c, d, b);
    if (((abC > tolerance && abD < -tolerance) ||
         (abC < -tolerance && abD > tolerance)) &&
        ((cdA > tolerance && cdB < -tolerance) ||
         (cdA < -tolerance && cdB > tolerance))) {
        return true;
    }
    return (std::abs(abC) <= tolerance && onSegment(c, a, b, tolerance)) ||
        (std::abs(abD) <= tolerance && onSegment(d, a, b, tolerance)) ||
        (std::abs(cdA) <= tolerance && onSegment(a, c, d, tolerance)) ||
        (std::abs(cdB) <= tolerance && onSegment(b, c, d, tolerance));
}

bool segmentEncroachesBoundary(
    const gp_Pnt2d& from,
    const gp_Pnt2d& to,
    const std::vector<gp_Pnt2d>& polygon,
    double requiredDistance,
    double tolerance,
    double& collisionDistance)
{
    if (pointInsideBoundary(from, polygon) ||
        pointInsideBoundary(to, polygon)) {
        collisionDistance = 0.0;
        return true;
    }

    // A distance is only unsafe when distance + tolerance < requiredDistance.
    // Use that threshold to discard almost every distant boundary edge before
    // doing the more expensive exact segment-distance calculation.  This keeps
    // validation proportional for clearing profiles containing hundreds of
    // thousands of short moves without imposing an arbitrary move-count cap.
    const double unsafeDistance = requiredDistance - tolerance;
    if (unsafeDistance <= 0.0)
        return false;
    const double unsafeDistanceSquared = unsafeDistance * unsafeDistance;
    const double segmentMinX = std::min(from.X(), to.X());
    const double segmentMaxX = std::max(from.X(), to.X());
    const double segmentMinY = std::min(from.Y(), to.Y());
    const double segmentMaxY = std::max(from.Y(), to.Y());

    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const gp_Pnt2d& edgeFrom = polygon[index];
        const gp_Pnt2d& edgeTo = polygon[(index + 1) % polygon.size()];
        const double edgeMinX = std::min(edgeFrom.X(), edgeTo.X());
        const double edgeMaxX = std::max(edgeFrom.X(), edgeTo.X());
        const double edgeMinY = std::min(edgeFrom.Y(), edgeTo.Y());
        const double edgeMaxY = std::max(edgeFrom.Y(), edgeTo.Y());
        const double gapX = std::max({
            edgeMinX - segmentMaxX,
            segmentMinX - edgeMaxX, 0.0});
        const double gapY = std::max({
            edgeMinY - segmentMaxY,
            segmentMinY - edgeMaxY, 0.0});
        if (gapX * gapX + gapY * gapY >= unsafeDistanceSquared)
            continue;

        if (segmentsIntersect(
                from, to, edgeFrom, edgeTo, tolerance)) {
            collisionDistance = 0.0;
            return true;
        }
        const double distance = std::min({
            pointSegmentDistance(from, edgeFrom, edgeTo),
            pointSegmentDistance(to, edgeFrom, edgeTo),
            pointSegmentDistance(edgeFrom, from, to),
            pointSegmentDistance(edgeTo, from, to)});
        if (distance < unsafeDistance) {
            collisionDistance = distance;
            return true;
        }
    }
    return false;
}

void addIssue(
    ProfileSafetyValidationResult& result,
    ToolpathDiagnosticCode code,
    ToolpathDiagnosticSeverity severity,
    const std::string& message,
    const ProfileToolpathOperation& operation,
    int moveIndex = -1,
    int depthIndex = -1)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = message;
    diagnostic.operationId = operation.operationId;
    diagnostic.moveIndex = moveIndex;
    diagnostic.depthIndex = depthIndex;
    result.diagnostics.push_back(std::move(diagnostic));

    addToolpathIssue(
        result.report,
        severity == ToolpathDiagnosticSeverity::Error
            ? ToolpathIssueSeverity::Error
            : ToolpathIssueSeverity::Warning,
        toolpathDiagnosticCodeName(code), message,
        operation.operationId, moveIndex);
}

bool axialIntervalsOverlap(
    double leftStart,
    double leftEnd,
    double rightStart,
    double rightEnd,
    double tolerance)
{
    return std::max(leftStart, rightStart) <=
           std::min(leftEnd, rightEnd) + tolerance;
}

std::vector<ToolSection> toolSections(
    const ToolDef& tool,
    double requiredCuttingLength,
    ProfileSafetyValidationResult& result,
    const ProfileToolpathOperation& operation)
{
    std::vector<ToolSection> sections;
    const double cutterLength = tool.cuttingEdgeLength > 0.0
        ? tool.cuttingEdgeLength : requiredCuttingLength;
    sections.push_back(
        {"cutting edge", 0.0, cutterLength,
         tool.radius(), true});

    if (tool.cuttingEdgeLength <= 0.0) {
        addIssue(
            result, ToolpathDiagnosticCode::InvalidToolAssembly,
            ToolpathDiagnosticSeverity::Warning,
            "profile cutting-edge length is unknown; it was inferred from the maximum axial engagement per depth layer",
            operation);
    }

    if (tool.overallLength > cutterLength) {
        sections.push_back(
            {"shank", cutterLength, tool.overallLength,
             0.5 * (tool.shankDiameter > 0.0
                        ? tool.shankDiameter : tool.diameter),
             false});
    } else if (tool.overallLength > 0.0 &&
               tool.overallLength + 1.0e-9 < cutterLength) {
        addIssue(
            result, ToolpathDiagnosticCode::InvalidToolAssembly,
            ToolpathDiagnosticSeverity::Error,
            "profile tool overall length is shorter than its cutting-edge length",
            operation);
    }

    if ((tool.holderDiameter > 0.0) != (tool.holderLength > 0.0)) {
        addIssue(
            result, ToolpathDiagnosticCode::InvalidToolAssembly,
            ToolpathDiagnosticSeverity::Warning,
            "profile holder envelope requires both diameter and length",
            operation);
    } else if (tool.holderDiameter > 0.0 &&
               tool.holderLength > 0.0) {
        const double start = std::max(tool.overallLength, cutterLength);
        sections.push_back(
            {"holder", start, start + tool.holderLength,
             0.5 * tool.holderDiameter, false});
    }

    if ((tool.spindleEnvelopeDiameter > 0.0) !=
        (tool.spindleEnvelopeLength > 0.0)) {
        addIssue(
            result, ToolpathDiagnosticCode::InvalidToolAssembly,
            ToolpathDiagnosticSeverity::Warning,
            "profile spindle envelope requires both diameter and length",
            operation);
    } else if (tool.spindleEnvelopeDiameter > 0.0 &&
               tool.spindleEnvelopeLength > 0.0) {
        const double holderEnd =
            std::max(tool.overallLength, cutterLength) +
            std::max(0.0, tool.holderLength);
        sections.push_back(
            {"spindle", holderEnd,
             holderEnd + tool.spindleEnvelopeLength,
             0.5 * tool.spindleEnvelopeDiameter, false});
    }
    return sections;
}

double maximumAxialEngagement(
    const ProfileRegion& region,
    const ProfileToolpathOperation& operation,
    const DepthParams& depth)
{
    // Profile 是逐层从开口面向最终深度加工。已经由上一深度层去除的材料不会
    // 再要求当前层的刀刃覆盖，因此刃长约束作用于相邻深度层差值；总深度由
    // 刀具伸出和装配包络独立约束。
    double previousOffset = region.depth;
    double maximum = 0.0;
    for (const ProfileLoop& loop : operation.contour.loops) {
        const double offset = std::clamp(
            loop.depthOffset, 0.0, region.depth);
        if (offset + 1.0e-9 < previousOffset) {
            maximum = std::max(maximum, previousOffset - offset);
            previousOffset = offset;
        }
    }
    // 动态截面会主动省略被相反装夹面材料阻挡的中间层。省略后的相邻
    // 可执行轮廓并不表示刀具一次轴向切入了整个缺口；实际分层仍由请求的
    // stepDown 控制，后续装夹负责缺失结构。因此刃长约束按计划层深封顶。
    if (region.useMaterialSectionBoundaryAtEveryDepth &&
        std::isfinite(depth.stepDown) && depth.stepDown > kAxisTolerance) {
        maximum = std::min(maximum, depth.stepDown);
    }
    return maximum > 1.0e-9 ? maximum : region.depth;
}

} // namespace

ProfileSafetyValidationResult ProfileSafetyValidator::validate(
    const ProfileRegion& region,
    const ProfileToolpathOperation& operation,
    const ToolpathParams& params,
    const std::function<bool()>& isCancellationRequested) const
{
    ProfileSafetyValidationResult result;
    if (isCancellationRequested && isCancellationRequested()) {
        result.cancelled = true;
        return result;
    }
    if (region.boundary.points.size() < 3 ||
        operation.contour.moves.empty()) {
        return result;
    }

    if (operation.outwardToolAxis.SquareMagnitude() <= 1.0e-24) {
        addIssue(
            result, ToolpathDiagnosticCode::InvalidSetupAxis,
            ToolpathDiagnosticSeverity::Error,
            "profile operation has no valid fixed tool axis", operation);
        return result;
    }
    gp_Vec operationAxis = operation.outwardToolAxis;
    operationAxis.Normalize();
    if (operationAxis.Dot(gp_Vec(region.openDir)) <
        1.0 - kAxisTolerance) {
        addIssue(
            result, ToolpathDiagnosticCode::SetupAxisMismatch,
            ToolpathDiagnosticSeverity::Error,
            "profile operation axis does not match the recognized exterior opening direction",
            operation);
        return result;
    }

    const double quantization = std::pow(
        10.0, -std::clamp(params.profile.precision, 0, 8));
    const double tolerance = std::max(0.02, 4.0 * quantization);
    const double requiredCuttingLength =
        maximumAxialEngagement(region, operation, params.depth);
    if (params.tool.cuttingEdgeLength > 0.0 &&
        params.tool.cuttingEdgeLength + tolerance < requiredCuttingLength) {
        std::ostringstream message;
        message << "profile maximum axial engagement "
                << requiredCuttingLength
                << " mm exceeds cutting-edge length "
                << params.tool.cuttingEdgeLength << " mm";
        addIssue(
            result,
            ToolpathDiagnosticCode::ProfileCuttingEdgeReachInsufficient,
            ToolpathDiagnosticSeverity::Error,
            message.str(), operation);
    }

    if (params.tool.overallLength > 0.0 &&
        params.tool.overallLength + tolerance < region.depth) {
        std::ostringstream message;
        message << "profile depth " << region.depth
                << " mm exceeds known tool reach "
                << params.tool.overallLength << " mm";
        addIssue(
            result,
            ToolpathDiagnosticCode::ProfileToolReachInsufficient,
            ToolpathDiagnosticSeverity::Error,
            message.str(), operation);
    } else if (params.tool.overallLength <= 0.0 &&
               region.depth > requiredCuttingLength + tolerance) {
        std::ostringstream message;
        message << "profile total depth is " << region.depth
                << " mm, but tool reach is unknown; verify stick-out before machining";
        addIssue(
            result, ToolpathDiagnosticCode::InvalidToolAssembly,
            ToolpathDiagnosticSeverity::Warning,
            message.str(), operation);
    }

    const std::vector<ToolSection> sections = toolSections(
        params.tool, requiredCuttingLength,
        result, operation);
    if (!result.report.ok())
        return result;

    const double sideAllowance =
        std::max(0.0, params.profile.sideAllowance);
    std::vector<const ProfileBoundaries*> materialBoundariesByLoop(
        operation.contour.loops.size(), nullptr);
    for (const ProfileLoop& loop : operation.contour.loops) {
        if (loop.sequence >= 0 &&
            loop.sequence < static_cast<int>(materialBoundariesByLoop.size()) &&
            loop.materialBoundaries &&
            !loop.materialBoundaries->empty()) {
            materialBoundariesByLoop[static_cast<std::size_t>(loop.sequence)] =
                loop.materialBoundaries.get();
        }
    }

    const ProfileBoundaries fallback{region.boundary.points};
    for (std::size_t moveIndex = 0;
         moveIndex < operation.contour.moves.size(); ++moveIndex) {
        if ((moveIndex & 0xffu) == 0u &&
            isCancellationRequested && isCancellationRequested()) {
            result.cancelled = true;
            return result;
        }
        const ToolpathMove& move = operation.contour.moves[moveIndex];
        const gp_Pnt2d planarFrom =
            OcctAreaAdapter::projectPoint(move.from, region.frame);
        const gp_Pnt2d planarTo =
            OcctAreaAdapter::projectPoint(move.to, region.frame);
        const double tipAxialFrom = gp_Vec(
            region.frame.Location(), move.from).Dot(gp_Vec(region.openDir));
        const double tipAxialTo = gp_Vec(
            region.frame.Location(), move.to).Dot(gp_Vec(region.openDir));
        const ProfileBoundaries* materialBoundaries =
            move.loopSequence >= 0 &&
                    move.loopSequence < static_cast<int>(
                        materialBoundariesByLoop.size())
                ? materialBoundariesByLoop[
                      static_cast<std::size_t>(move.loopSequence)]
                : nullptr;
        const ProfileBoundaries& finishedBoundaries =
            materialBoundaries ? *materialBoundaries : fallback;
        for (const ToolSection& section : sections) {
            const double axialStart = std::min(
                tipAxialFrom, tipAxialTo) + section.axialStart;
            const double axialEnd = std::max(
                tipAxialFrom, tipAxialTo) + section.axialEnd;
            if (!axialIntervalsOverlap(
                    axialStart, axialEnd,
                    0.0, region.depth, tolerance)) {
                continue;
            }
            const bool cuttingMove =
                move.type == ToolpathMoveType::Cut ||
                move.type == ToolpathMoveType::Plunge;
            const double required = section.radius +
                (section.cutter && cuttingMove
                     ? sideAllowance : 0.0);
            double distance = 0.0;
            if (std::none_of(finishedBoundaries.begin(), finishedBoundaries.end(),
                    [&](const ProfileBoundary& boundary) {
                        return segmentEncroachesBoundary(planarFrom, planarTo,
                            boundary, required, tolerance, distance);
                    })) {
                continue;
            }

            std::ostringstream message;
            message << section.name
                    << " envelope intersects the finished-part profile"
                    << " on linear move " << moveIndex
                    << " (minimum planar distance=" << distance
                    << " mm, required=" << required << " mm)";
            const ToolpathDiagnosticCode code =
                section.cutter && cuttingMove
                ? ToolpathDiagnosticCode::ProfileFinishedPartIntrusion
                : cuttingMove
                    ? ToolpathDiagnosticCode::CuttingMoveCollision
                    : ToolpathDiagnosticCode::NonCuttingMoveCollision;
            addIssue(
                result, code, ToolpathDiagnosticSeverity::Error,
                message.str(), operation,
                static_cast<int>(moveIndex), move.depthIndex);
            return result;
        }
    }
    return result;
}

} // namespace oap::toolpath
