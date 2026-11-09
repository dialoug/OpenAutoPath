// FixedAxisBRepPreflight::inspect 实现（oap_toolpath_pocket 的 fixedaxis
// 私有目录）。只解析整次审计共享的刀具几何与移动类别，不构造 OCCT 减料体；
// 工序级错误仍由构造器生成对应快照。参见 FixedAxisBRepPreflight.h。
#include "toolpath/pocket/simulation/fixedaxis/FixedAxisBRepPreflight.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Precision.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath {
namespace {

bool finitePoint(const gp_Pnt& point)
{
    return std::isfinite(point.X()) &&
           std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

bool normalizedAxis(
    const gp_Vec& source,
    double tolerance,
    gp_Vec& result)
{
    if (!std::isfinite(source.X()) ||
        !std::isfinite(source.Y()) ||
        !std::isfinite(source.Z()) ||
        source.Magnitude() <= tolerance) {
        return false;
    }
    result = source;
    result.Normalize();
    return true;
}

double projection(
    const gp_Pnt& point,
    const gp_Vec& axis)
{
    return point.X() * axis.X() +
           point.Y() * axis.Y() +
           point.Z() * axis.Z();
}

bool stockProjectionSpan(
    const TopoDS_Shape& stock,
    const gp_Vec& axis,
    double& span)
{
    if (stock.IsNull())
        return false;
    Bnd_Box bounds;
    BRepBndLib::AddOptimal(
        stock, bounds, Standard_False, Standard_False);
    if (bounds.IsVoid())
        return false;

    double xMin = 0.0;
    double yMin = 0.0;
    double zMin = 0.0;
    double xMax = 0.0;
    double yMax = 0.0;
    double zMax = 0.0;
    bounds.Get(
        xMin, yMin, zMin,
        xMax, yMax, zMax);
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (int corner = 0; corner < 8; ++corner) {
        const gp_Pnt point(
            (corner & 1) != 0 ? xMax : xMin,
            (corner & 2) != 0 ? yMax : yMin,
            (corner & 4) != 0 ? zMax : zMin);
        const double value = projection(point, axis);
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
    }
    span = maximum - minimum;
    return std::isfinite(span) && span > 0.0;
}

void appendInferredLengthDiagnostic(
    FixedAxisBRepPreflightResult& result,
    double length)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code =
        ToolpathDiagnosticCode::StockCuttingLengthInferred;
    diagnostic.severity =
        ToolpathDiagnosticSeverity::Warning;
    diagnostic.message =
        "刀具刃长未设置；本次固定轴 BRep 审计按毛坯轴向跨度推导为 " +
        std::to_string(length) +
        " mm。该值只用于减料体审计，不能替代真实刀具伸出与碰撞校验。";
    result.diagnostics.push_back(std::move(diagnostic));
}

} // namespace

FixedAxisBRepPreflightResult
FixedAxisBRepPreflight::inspect(
    const TopoDS_Shape& stock,
    const std::vector<PocketToolpathOperation>& operations,
    const ToolpathParams& params,
    const StockSimulationOptions& options) const
{
    FixedAxisBRepPreflightResult result;
    result.effectiveParams = params;
    const double tolerance = std::max(
        options.geometricTolerance,
        Precision::Confusion());

    gp_Vec referenceAxis;
    bool hasReferenceAxis = false;
    // 第一遍：按「轴向分量 / 平面分量」把每个切削移动分类到
    // 恒深切削、轴向进刀、直线/螺旋斜坡，并记录参考刀轴。
    for (const PocketToolpathOperation& operation :
         operations) {
        gp_Vec axis;
        if (!normalizedAxis(
                operation.outwardToolAxis,
                tolerance, axis)) {
            continue;
        }
        if (!hasReferenceAxis) {
            referenceAxis = axis;
            hasReferenceAxis = true;
        }
        if (operation.clearing.helicalRampEntryCount > 0)
            ++result.helicalRampOperationCount;

        for (const ToolpathMove& move :
             operation.clearing.moves) {
            if ((move.type != ToolpathMoveType::Cut &&
                 move.type != ToolpathMoveType::Plunge) ||
                !finitePoint(move.from) ||
                !finitePoint(move.to)) {
                continue;
            }
            const gp_Vec delta(move.from, move.to);
            const double axial =
                std::abs(delta.Dot(axis));
            const gp_Vec planar =
                delta - axis.Multiplied(delta.Dot(axis));
            if (move.type == ToolpathMoveType::Cut &&
                axial <= tolerance) {
                ++result.constantDepthCutMoveCount;
            } else if (
                move.type == ToolpathMoveType::Plunge &&
                planar.Magnitude() <= tolerance) {
                ++result.axialPlungeMoveCount;
            } else if (
                move.type == ToolpathMoveType::Plunge &&
                axial > tolerance &&
                planar.Magnitude() > tolerance) {
                if (move.intent ==
                    ToolpathMoveIntent::
                        PocketHelicalRampEntry) {
                    ++result.helicalRampMoveCount;
                } else {
                    ++result.linearRampMoveCount;
                }
            }
        }
    }

    const double configuredLength =
        params.tool.cuttingEdgeLength;
    if (std::isfinite(configuredLength) &&
        configuredLength > tolerance) {
        result.effectiveCuttingEdgeLength =
            configuredLength;
        return result;
    }

    double stockSpan = 0.0;
    // 第二遍：未显式配置刃长时，按毛坯沿刀轴的投影跨度推导刃长，
    // 并记录「推断」诊断。该值只用于减料体审计。
    if (hasReferenceAxis &&
        stockProjectionSpan(
            stock, referenceAxis, stockSpan) &&
        std::isfinite(params.tool.diameter) &&
        params.tool.diameter > tolerance) {
        const double inferredLength =
            std::max(
                params.tool.diameter,
                stockSpan + params.tool.diameter);
        result.effectiveParams.tool.cuttingEdgeLength =
            inferredLength;
        result.effectiveCuttingEdgeLength =
            inferredLength;
        result.inferredCuttingEdgeLength = true;
        appendInferredLengthDiagnostic(
            result, inferredLength);
    }
    return result;
}

} // namespace oap::toolpath
