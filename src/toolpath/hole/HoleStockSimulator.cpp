// 将孔刀路语义转换为固定刀轴切削体，并按操作顺序更新 InProcessStock。
// 属于 oap_toolpath_hole 模块内部实现：为每个孔工序构造去料实体并逐个做
// 布尔减料，产出工序级毛坯快照、体积记录与耗时统计。
#include "toolpath/hole/HoleStockSimulator.h"
#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>

namespace oap::toolpath {
namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedMilliseconds(const SteadyClock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               SteadyClock::now() - start)
        .count();
}

class ScopedElapsedMilliseconds {
public:
    explicit ScopedElapsedMilliseconds(double& output)
        : m_output(output), m_start(SteadyClock::now())
    {
    }

    ~ScopedElapsedMilliseconds()
    {
        m_output = elapsedMilliseconds(m_start);
    }

private:
    double& m_output;
    SteadyClock::time_point m_start;
};

constexpr double kPi = 3.14159265358979323846;

struct CutterGeometry {
    double radius = 0.0;
    double cuttingLength = 0.0;
    double tipAngleDeg = 0.0;
    bool drill = false;
    bool inferredCuttingLength = false;
};

struct RemovalBuildResult {
    TopoDS_Shape shape;
    std::string error;
    double maximumChordError = 0.0;
    int segmentCount = 0;
    bool inferredCuttingLength = false;
    double effectiveCuttingLength = 0.0;
};

bool finitePoint(const gp_Pnt& point)
{
    return std::isfinite(point.X()) && std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

CutterGeometry cutterGeometry(const HoleMachiningOperation& operation,
                              const ToolpathParams& params)
{
    CutterGeometry geometry;
    const bool operationTool = operation.tool.valid;
    const double diameter = operationTool && operation.tool.diameter > 0.0
        ? operation.tool.diameter : params.tool.diameter;
    geometry.radius = 0.5 * diameter;
    // 多刀工序已经绑定刀具时，未知刃长不能回退到另一把程序级刀具。
    // 材料去除仿真只需要一个覆盖当前孔深的虚拟切削体，因此未知时使用
    // 工序几何需求；该推导不会写回刀库，也不能代替真实伸出/碰撞确认。
    const double recordedCuttingLength = operationTool
        ? operation.tool.cuttingEdgeLength
        : params.tool.cuttingEdgeLength;
    if (std::isfinite(recordedCuttingLength) &&
        recordedCuttingLength > 0.0) {
        geometry.cuttingLength = recordedCuttingLength;
    } else {
        const double nominalDepth = operation.entry.Distance(operation.bottom);
        const double derivedLength =
            std::isfinite(operation.minimumCuttingEdgeLength) &&
                    operation.minimumCuttingEdgeLength > 0.0
                ? operation.minimumCuttingEdgeLength
                : nominalDepth;
        if (std::isfinite(derivedLength) && derivedLength > 0.0) {
            geometry.cuttingLength = derivedLength;
            // 点钻的材料去除范围本来就由该工序的定位深度定义；未知物理刃长
            // 已由刀具验证器单独告警，不应在每个点钻工序中重复产生仿真告警。
            geometry.inferredCuttingLength =
                operation.phase != HoleMachiningPhase::CenterSpot;
        }
    }
    geometry.tipAngleDeg =
        operationTool && operation.tool.tipAngle > 0.0
            ? operation.tool.tipAngle : params.hole.drillTipAngleDeg;
    geometry.drill =
        operation.strategy == HoleMachiningStrategy::SmallDrill ||
        operation.phase == HoleMachiningPhase::CenterSpot;
    return geometry;
}

TopoDS_Shape makeCylinder(const gp_Pnt& tip,
                          const gp_Vec& outwardAxis,
                          double radius,
                          double length)
{
    if (length <= Precision::Confusion())
        return {};
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(tip, gp_Dir(outwardAxis)), radius, length).Shape();
}

TopoDS_Shape makeFlatCutterLinearSweep(const gp_Pnt& from,
                                       const gp_Pnt& to,
                                       const gp_Vec& outwardAxis,
                                       double radius,
                                       double cuttingLength,
                                       double tolerance)
{
    const gp_Vec travel(from, to);
    const double axialTravel = travel.Dot(outwardAxis);
    const gp_Vec radialTravel =
        travel - outwardAxis.Multiplied(axialTravel);
    if (radialTravel.Magnitude() <= tolerance) {
        const gp_Pnt base = from.Translated(
            outwardAxis.Multiplied(std::min(0.0, axialTravel)));
        return makeCylinder(base, outwardAxis, radius,
                            cuttingLength + std::abs(axialTravel));
    }

    gp_Vec side = outwardAxis.Crossed(radialTravel);
    if (side.Magnitude() <= tolerance)
        return {};
    side.Normalize();
    const gp_Vec halfWidth = side.Multiplied(radius);

    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(from.Translated(halfWidth));
    polygon.Add(to.Translated(halfWidth));
    polygon.Add(to.Translated(halfWidth.Reversed()));
    polygon.Add(from.Translated(halfWidth.Reversed()));
    polygon.Close();
    if (!polygon.IsDone())
        return {};
    BRepBuilderAPI_MakeFace face(polygon.Wire());
    const TopoDS_Shape faceShape = face.Shape();
    if (faceShape.IsNull())
        return {};
    BRepPrimAPI_MakePrism central(
        faceShape, outwardAxis.Multiplied(cuttingLength));
    const TopoDS_Shape centralShape = central.Shape();
    const TopoDS_Shape fromCylinder = makeCylinder(
        from, outwardAxis, radius, cuttingLength);
    const TopoDS_Shape toCylinder = makeCylinder(
        to, outwardAxis, radius, cuttingLength);
    if (centralShape.IsNull() || fromCylinder.IsNull() ||
        toCylinder.IsNull())
        return {};

    TopoDS_Compound sweep;
    BRep_Builder builder;
    builder.MakeCompound(sweep);
    builder.Add(sweep, centralShape);
    builder.Add(sweep, fromCylinder);
    builder.Add(sweep, toCylinder);
    return sweep;
}

TopoDS_Shape makeDrillLinearSweep(const gp_Pnt& from,
                                  const gp_Pnt& to,
                                  const gp_Vec& outwardAxis,
                                  const CutterGeometry& cutter,
                                  double tolerance)
{
    const gp_Vec travel(from, to);
    const double axialTravel = travel.Dot(outwardAxis);
    const gp_Vec radialTravel =
        travel - outwardAxis.Multiplied(axialTravel);
    if (radialTravel.Magnitude() > tolerance)
        return {};
    const gp_Pnt deepestTip = from.Translated(
        outwardAxis.Multiplied(std::min(0.0, axialTravel)));
    const double stroke = std::abs(axialTravel);
    if (!(std::isfinite(cutter.tipAngleDeg) &&
          cutter.tipAngleDeg > 0.0 && cutter.tipAngleDeg < 180.0)) {
        return makeCylinder(deepestTip, outwardAxis, cutter.radius,
                            cutter.cuttingLength + stroke);
    }

    const double tangent =
        std::tan(0.5 * cutter.tipAngleDeg * kPi / 180.0);
    if (!std::isfinite(tangent) || tangent <= tolerance)
        return {};
    const double tipLength = cutter.radius / tangent;
    if (!std::isfinite(tipLength) ||
        tipLength >= cutter.cuttingLength - tolerance)
        return {};

    // 由轴截面一次旋转生成钻尖与圆柱刃部的单一实体。相比“圆锥 + 圆柱”
    // 复合体，它没有共面子实体，连续布尔减料时更不容易积累退化拓扑。
    const double totalLength = cutter.cuttingLength + stroke;
    gp_Vec reference = std::abs(outwardAxis.X()) < 0.9
        ? gp_Vec(1.0, 0.0, 0.0)
        : gp_Vec(0.0, 1.0, 0.0);
    gp_Vec radial = outwardAxis.Crossed(reference);
    if (radial.Magnitude() <= tolerance)
        return {};
    radial.Normalize();
    const gp_Vec radialExtent = radial.Multiplied(cutter.radius);
    const gp_Pnt shoulder = deepestTip.Translated(
        outwardAxis.Multiplied(tipLength) + radialExtent);
    const gp_Pnt outerTop = deepestTip.Translated(
        outwardAxis.Multiplied(totalLength) + radialExtent);
    const gp_Pnt axisTop = deepestTip.Translated(
        outwardAxis.Multiplied(totalLength));

    BRepBuilderAPI_MakePolygon profile;
    profile.Add(deepestTip);
    profile.Add(shoulder);
    profile.Add(outerTop);
    profile.Add(axisTop);
    profile.Close();
    if (!profile.IsDone())
        return {};
    BRepBuilderAPI_MakeFace face(profile.Wire());
    if (!face.IsDone())
        return {};
    BRepPrimAPI_MakeRevol revolved(
        face.Shape(), gp_Ax1(deepestTip, gp_Dir(outwardAxis)), true);
    if (!revolved.IsDone())
        return {};
    return revolved.Shape();
}

TopoDS_Shape makeLinearSweep(const gp_Pnt& from,
                             const gp_Pnt& to,
                             const gp_Vec& outwardAxis,
                             const CutterGeometry& cutter,
                             double tolerance)
{
    if (!finitePoint(from) || !finitePoint(to))
        return {};
    if (cutter.drill) {
        return makeDrillLinearSweep(
            from, to, outwardAxis, cutter, tolerance);
    }
    return makeFlatCutterLinearSweep(
        from, to, outwardAxis, cutter.radius,
        cutter.cuttingLength, tolerance);
}

bool sampleArc(const ToolpathCircularArc& arc,
               const HoleStockSimulationOptions& options,
               int remainingSegments,
               std::vector<gp_Pnt>& points,
               double& chordError,
               std::string& error)
{
    if (!std::isfinite(arc.sweepRadians) || arc.sweepRadians <= 0.0 ||
        arc.axis.SquareMagnitude() <= 1.0e-16 ||
        !finitePoint(arc.from) || !finitePoint(arc.to) ||
        !finitePoint(arc.center)) {
        error = "circular primitive has invalid geometry";
        return false;
    }
    gp_Vec axis = arc.axis;
    axis.Normalize();
    const gp_Vec centerToStart(arc.center, arc.from);
    const double startAxial = centerToStart.Dot(axis);
    const gp_Vec startRadial =
        centerToStart - axis.Multiplied(startAxial);
    const double radius = startRadial.Magnitude();
    if (!std::isfinite(radius) || radius <= Precision::Confusion()) {
        error = "circular primitive has zero radius";
        return false;
    }
    const double tolerance = std::clamp(
        options.arcChordTolerance, 1.0e-4, std::max(1.0e-4, radius));
    const double cosine = std::clamp(1.0 - tolerance / radius, -1.0, 1.0);
    double maximumAngle = 2.0 * std::acos(cosine);
    maximumAngle = std::clamp(maximumAngle, kPi / 180.0, kPi / 18.0);
    const double segmentValue = std::ceil(arc.sweepRadians / maximumAngle);
    if (!std::isfinite(segmentValue) || segmentValue > remainingSegments) {
        error = "circular primitive exceeds the stock sweep segment budget";
        return false;
    }
    const int segmentCount = std::max(1, static_cast<int>(segmentValue));
    const double segmentAngle = arc.sweepRadians / segmentCount;
    chordError = radius * (1.0 - std::cos(0.5 * segmentAngle));

    const double direction =
        arc.direction == ToolpathArcDirection::Counterclockwise ? 1.0 : -1.0;
    const double totalAxial = gp_Vec(arc.from, arc.to).Dot(axis);
    const gp_Vec quarterTurn = axis.Crossed(startRadial);
    points.clear();
    points.reserve(static_cast<std::size_t>(segmentCount) + 1);
    for (int index = 0; index <= segmentCount; ++index) {
        if (index == segmentCount) {
            points.push_back(arc.to);
            continue;
        }
        const double ratio = static_cast<double>(index) / segmentCount;
        const double angle = direction * arc.sweepRadians * ratio;
        const gp_Vec radial =
            startRadial.Multiplied(std::cos(angle)) +
            quarterTurn.Multiplied(std::sin(angle));
        points.push_back(arc.center.Translated(
            radial + axis.Multiplied(startAxial + totalAxial * ratio)));
    }
    const double endAngle = direction * arc.sweepRadians;
    const gp_Vec expectedRadial =
        startRadial.Multiplied(std::cos(endAngle)) +
        quarterTurn.Multiplied(std::sin(endAngle));
    const gp_Pnt expectedEnd = arc.center.Translated(
        expectedRadial + axis.Multiplied(startAxial + totalAxial));
    if (expectedEnd.Distance(arc.to) >
        std::max(10.0 * options.geometricTolerance, 1.0e-6 * radius)) {
        error = "circular primitive endpoint disagrees with its sweep";
        return false;
    }
    return true;
}

RemovalBuildResult buildRemovalVolume(
    const HoleMachiningOperation& operation,
    const ToolpathParams& fallbackParams,
    const HoleStockSimulationOptions& options)
{
    RemovalBuildResult result;
    CutterGeometry cutter = cutterGeometry(operation, fallbackParams);
    result.inferredCuttingLength = cutter.inferredCuttingLength;
    result.effectiveCuttingLength = cutter.cuttingLength;
    if (!std::isfinite(cutter.radius) || cutter.radius <= 0.0 ||
        !std::isfinite(cutter.cuttingLength) ||
        cutter.cuttingLength <= 0.0) {
        result.error =
            "positive cutter diameter and a valid operation depth are required";
        return result;
    }
    gp_Vec inwardAxis(operation.entry, operation.bottom);
    if (inwardAxis.Magnitude() <= options.geometricTolerance) {
        result.error = "hole operation has an invalid fixed tool axis";
        return result;
    }
    inwardAxis.Normalize();
    const gp_Vec outwardAxis = inwardAxis.Reversed();
    const int segmentBudget = std::clamp(
        options.maxSweepSegments, 1, 200000);

    // 孔工序级快照只关心该工序完成后的毛坯。对能够证明已清满截面的铣孔
    // 策略，解析圆柱与全部切削原语的最终并集等价，且避免 OCCT 对数百个
    // 相互重叠弦段做全体求交。不能证明覆盖时仍走下方原语扫掠回退。
    const double depth = operation.entry.Distance(operation.bottom);
    const double removedRadius =
        operation.radius - std::max(0.0, fallbackParams.hole.allowance);
    // 螺旋和分层铣孔的刀具沿轴向覆盖完整孔深；刃长只需覆盖相邻螺距/
    // 分层间距，而不需要等于总孔深。HoleMachining 已把这一策略相关要求
    // 写入 minimumCuttingEdgeLength，验证器也使用同一字段。继续要求整孔
    // 深刃长会让已经验证通过的短刃长工序退回到数千弦段的复合扫掠。
    const double requiredCuttingLength =
        std::isfinite(operation.minimumCuttingEdgeLength) &&
                operation.minimumCuttingEdgeLength >
                    options.geometricTolerance
            ? operation.minimumCuttingEdgeLength
            : depth;
    const bool cuttingPatternCovered =
        cutter.cuttingLength + options.geometricTolerance >=
        requiredCuttingLength;
    const bool mediumCoversCenter =
        removedRadius <= 2.0 * cutter.radius + options.geometricTolerance;
    const bool analyticMilledBore =
        !cutter.drill && cuttingPatternCovered &&
        ((operation.strategy == HoleMachiningStrategy::MediumHelical &&
          mediumCoversCenter) ||
         operation.strategy == HoleMachiningStrategy::LargeLayeredSpiral);
    if (analyticMilledBore && std::isfinite(removedRadius) &&
        removedRadius > options.geometricTolerance) {
        result.shape = makeCylinder(
            operation.bottom, outwardAxis, removedRadius, depth);
        if (result.shape.IsNull()) {
            result.error = "failed to construct the analytic milled-bore volume";
            return result;
        }
        result.segmentCount = 1;
        return result;
    }

    TopoDS_Compound removal;
    BRep_Builder builder;
    builder.MakeCompound(removal);
    bool hasCuttingGeometry = false;
    auto addLine = [&](const gp_Pnt& from, const gp_Pnt& to) {
        if (result.segmentCount >= segmentBudget) {
            result.error = "stock sweep segment budget exceeded";
            return false;
        }
        const TopoDS_Shape sweep = makeLinearSweep(
            from, to, outwardAxis, cutter,
            std::max(options.geometricTolerance, Precision::Confusion()));
        if (sweep.IsNull()) {
            result.error = "failed to construct a linear cutter sweep";
            return false;
        }
        builder.Add(removal, sweep);
        ++result.segmentCount;
        hasCuttingGeometry = true;
        return true;
    };
    auto addArc = [&](const ToolpathCircularArc& arc) {
        if (cutter.drill) {
            result.error = "drill tools cannot follow circular cutting primitives";
            return false;
        }
        std::vector<gp_Pnt> points;
        double chordError = 0.0;
        if (!sampleArc(arc, options,
                       segmentBudget - result.segmentCount,
                       points, chordError, result.error))
            return false;
        result.maximumChordError =
            std::max(result.maximumChordError, chordError);
        for (std::size_t index = 1; index < points.size(); ++index) {
            if (!addLine(points[index - 1], points[index]))
                return false;
        }
        return true;
    };
    auto addDrillCycle = [&](const ToolpathPrimitive& primitive) {
        if (!cutter.drill) {
            result.error =
                "drill cycles require an axial drill cutter";
            return false;
        }
        const double tolerance =
            std::max(options.geometricTolerance, Precision::Confusion());
        double shallowestProjection =
            -std::numeric_limits<double>::infinity();
        double deepestProjection =
            std::numeric_limits<double>::infinity();
        gp_Pnt shallowest;
        gp_Pnt deepest;
        bool hasCutMove = false;
        auto includePoint = [&](const gp_Pnt& point) {
            if (!finitePoint(point)) {
                result.error = "drill cycle contains a non-finite point";
                return false;
            }
            const gp_Vec offset(operation.entry, point);
            const double projection = offset.Dot(outwardAxis);
            const gp_Vec radialOffset =
                offset - outwardAxis.Multiplied(projection);
            if (radialOffset.Magnitude() > tolerance) {
                result.error =
                    "drill cycle leaves the fixed tool axis";
                return false;
            }
            if (projection > shallowestProjection) {
                shallowestProjection = projection;
                shallowest = point;
            }
            if (projection < deepestProjection) {
                deepestProjection = projection;
                deepest = point;
            }
            return true;
        };
        for (const ToolpathMove& move : primitive.fallbackMoves) {
            if (move.type != ToolpathMoveType::Cut)
                continue;
            hasCutMove = true;
            if (!includePoint(move.from) || !includePoint(move.to))
                return false;
        }
        if (!hasCutMove &&
            (!includePoint(primitive.drillCycle.entry) ||
             !includePoint(primitive.drillCycle.bottom))) {
            return false;
        }
        if (shallowestProjection - deepestProjection <= tolerance) {
            result.error = "drill cycle has no positive axial cutting span";
            return false;
        }

        // G73/G83 的各次进刀在同一轴线上并覆盖连续深度区间，其切削体并集
        // 等价于从最浅切削点到最深切削点的一次扫掠。只构造一个包络，避免
        // 将大量相互重叠的啄钻扫掠交给 OCCT 布尔运算。
        return addLine(shallowest, deepest);
    };

    bool hasPrimitiveCuttingGeometry = false;
    for (const ToolpathPrimitive& primitive : operation.primitives) {
        if (primitive.type == ToolpathPrimitiveType::Linear &&
            primitive.linear.type == ToolpathMoveType::Cut) {
            hasPrimitiveCuttingGeometry = true;
            if (!addLine(primitive.linear.from, primitive.linear.to))
                return result;
        } else if (primitive.type == ToolpathPrimitiveType::CircularArc &&
                   primitive.arc.moveType == ToolpathMoveType::Cut) {
            hasPrimitiveCuttingGeometry = true;
            if (!addArc(primitive.arc))
                return result;
        } else if (primitive.type == ToolpathPrimitiveType::DrillCycle) {
            hasPrimitiveCuttingGeometry = true;
            if (!addDrillCycle(primitive))
                return result;
        }
    }
    if (!hasPrimitiveCuttingGeometry) {
        for (const ToolpathMove& move : operation.moves) {
            if (move.type == ToolpathMoveType::Cut &&
                !addLine(move.from, move.to))
                return result;
        }
    }
    if (!hasCuttingGeometry) {
        result.error = "operation contains no cutting geometry";
        return result;
    }
    result.shape = removal;
    return result;
}

void addSimulationDiagnostic(HoleStockSimulationResult& result,
                             ToolpathDiagnosticCode code,
                             ToolpathDiagnosticSeverity severity,
                             const std::string& message,
                             const std::string& operationId = {})
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = message;
    diagnostic.operationId = operationId;
    result.diagnostics.push_back(std::move(diagnostic));
}

} // namespace

HoleStockSimulationResult HoleStockSimulator::simulate(
    const TopoDS_Shape& initialStock,
    const std::vector<HoleMachiningOperation>& operations,
    const ToolpathParams& fallbackParams,
    const HoleStockSimulationOptions& options,
    const ToolpathGenerationControl* control) const
{
    ToolpathParallelScope parallelScope(control);
    HoleStockSimulationResult result;
    ScopedElapsedMilliseconds totalTimer(
        result.totalMilliseconds);
    if (control)
        control->progress(0, operations.size());
    result.initialStock = initialStock;
    result.parallelProcessingEnabled = options.runParallel;
    result.parallelWorkerCount = options.runParallel
        ? static_cast<int>(toolpathHardwareParallelism())
        : 1;
    std::function<bool()> cancellationCheck;
    if (control) {
        cancellationCheck = [control] {
            return control->cancelled();
        };
    }
    InProcessStock stock(
        initialStock, options.geometricTolerance,
        options.runParallel, std::move(cancellationCheck));
    result.initialVolume = stock.initialVolume();
    result.finalVolume = stock.currentVolume();
    result.finalStock = stock.currentShape();
    if (!stock.valid()) {
        addSimulationDiagnostic(
            result, ToolpathDiagnosticCode::InvalidInitialStock,
            ToolpathDiagnosticSeverity::Error,
            "hole stock simulation requires a valid positive-volume initial stock");
        return result;
    }

    bool fatalFailure = false;
    auto cancellationRequested = [&]() {
        if (!control || !control->cancelled())
            return false;
        result.cancelled = true;
        addSimulationDiagnostic(
            result, ToolpathDiagnosticCode::GenerationCancelled,
            ToolpathDiagnosticSeverity::Warning,
            "hole stock simulation was cancelled; completed stock snapshots were retained");
        return true;
    };
    // 主循环：逐工序构造切削体 → 应用布尔减料 → 记录状态快照；单个工序
    // 失败默认不中断后续工序，仅当 stopOnFailure 时才提前退出。
    for (std::size_t index = 0; index < operations.size(); ++index) {
        if (cancellationRequested())
            break;
        const HoleMachiningOperation& operation = operations[index];
        RemovalBuildResult removal;
        const SteadyClock::time_point sweepBuildStart =
            SteadyClock::now();
        try {
            removal = buildRemovalVolume(operation, fallbackParams, options);
        } catch (const Standard_Failure&) {
            removal.error = "OCCT raised an exception while building the cutting sweep";
        } catch (...) {
            removal.error = "unknown exception while building the cutting sweep";
        }
        result.sweepBuildMilliseconds +=
            elapsedMilliseconds(sweepBuildStart);
        result.maximumChordError =
            std::max(result.maximumChordError, removal.maximumChordError);
        result.sweepSegmentCount += removal.segmentCount;
        if (removal.inferredCuttingLength) {
            addSimulationDiagnostic(
                result, ToolpathDiagnosticCode::StockCuttingLengthInferred,
                ToolpathDiagnosticSeverity::Info,
                operation.operationId +
                    ": cutting-edge length is unknown; stock simulation used the " +
                    std::to_string(removal.effectiveCuttingLength) +
                    " mm length derived from the hole operation",
                operation.operationId);
        }
        if (removal.shape.IsNull()) {
            StockRemovalRecord record;
            record.operationId = operation.operationId;
            record.sequenceIndex = static_cast<int>(index);
            record.status = StockRemovalStatus::InvalidRemovalVolume;
            record.volumeBefore = stock.currentVolume();
            record.volumeAfter = stock.currentVolume();
            record.stockAfter = stock.currentShape();
            record.message = removal.error;
            result.steps.push_back(std::move(record));
            addSimulationDiagnostic(
                result, ToolpathDiagnosticCode::StockSweepConstructionFailed,
                ToolpathDiagnosticSeverity::Error,
                operation.operationId + ": " + removal.error,
                operation.operationId);
            fatalFailure = true;
            if (control)
                control->progress(index + 1, operations.size());
            if (options.stopOnFailure)
                break;
            continue;
        }

        // Fallback sweeps are compounds of overlapping segment solids, not a
        // pre-fused manifold tool. Submit their solids as separate Boolean tools
        // so OCCT subtracts their union; passing the compound as one argument can
        // leave intersecting shells in a helical bore. The stock commits only
        // after every batch and the final topology/volume checks succeed.
        TopExp_Explorer solids(removal.shape, TopAbs_SOLID);
        if (solids.More()) solids.Next();
        StockRemovalRecord record = solids.More()
            ? stock.applyRemovalWithIndependentSolids(
                  removal.shape, operation.operationId, static_cast<int>(index))
            : stock.applyRemoval(
                  removal.shape, operation.operationId, static_cast<int>(index));
        result.inputValidationMilliseconds +=
            record.inputValidationMilliseconds;
        result.booleanMilliseconds +=
            record.booleanMilliseconds;
        result.resultValidationMilliseconds +=
            record.resultValidationMilliseconds;
        result.booleanBatchCount +=
            record.booleanBatchCount;
        result.steps.push_back(record);
        if (record.status == StockRemovalStatus::Cancelled) {
            cancellationRequested();
            if (control)
                control->progress(index + 1, operations.size());
            break;
        }
        ++result.processedOperationCount;
        if (record.status == StockRemovalStatus::Applied) {
            ++result.appliedOperationCount;
        } else if (record.status == StockRemovalStatus::NoMaterialRemoved) {
            ++result.noMaterialOperationCount;
            addSimulationDiagnostic(
                result, ToolpathDiagnosticCode::StockNoMaterialRemoved,
                ToolpathDiagnosticSeverity::Warning,
                operation.operationId + ": " + record.message,
                operation.operationId);
        } else {
            addSimulationDiagnostic(
                result, ToolpathDiagnosticCode::StockBooleanFailed,
                ToolpathDiagnosticSeverity::Error,
                operation.operationId + ": " + record.message,
                operation.operationId);
            fatalFailure = true;
            if (options.stopOnFailure) {
                if (control)
                    control->progress(index + 1, operations.size());
                break;
            }
        }
        if (control)
            control->progress(index + 1, operations.size());
    }
    result.finalStock = stock.currentShape();
    result.finalVolume = stock.currentVolume();
    result.removedVolume = std::max(
        0.0, result.initialVolume - result.finalVolume);
    result.complete = !fatalFailure && !result.cancelled &&
        result.processedOperationCount == static_cast<int>(operations.size());
    return result;
}

} // namespace oap::toolpath
