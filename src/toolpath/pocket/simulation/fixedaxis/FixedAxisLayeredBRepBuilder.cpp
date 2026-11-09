// FixedAxisLayeredBRepBuilder 实现（oap_toolpath_pocket 的 fixedaxis 私有
// 目录）。把刀具中心路径按二维 Clipper 笔画求并、分解为不重叠轴向片体并
// 提升为闭合 OCCT 棱柱；解析快路径失败时整工序回退到此 Clipper 分层路径。
// 参见 FixedAxisLayeredBRepBuilder.h。
#include "toolpath/pocket/simulation/fixedaxis/FixedAxisLayeredBRepBuilder.h"
#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <Precision.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "toolpath/pocket/planning/detail/PocketClipperUtils.h"
#include "toolpath/core/simulation/DexelToolProfile.h"
#include "toolpath/pocket/simulation/fixedaxis/AnalyticFixedAxisLayeredBRepBuilder.h"

namespace oap::toolpath {
namespace {

using Clock = std::chrono::steady_clock;
using detail::PathD;
using detail::PathInfo;
using detail::PathsD;
using detail::PointD;
constexpr std::size_t kMinimumParallelTaskCount = 4u;

// 以毛坯/工序共用的刀轴建立的局部坐标系：u/v 为平面方向、w 为轴向；
// 全部几何在进入二维 Clipper 前投影到该平面，轴向高度由 axial() 单独计算。
struct FixedAxisFrame {
    gp_Pnt origin;
    gp_Dir u;
    gp_Dir v;
    gp_Dir w;

    PointD project(const gp_Pnt& point) const
    {
        const gp_Vec delta(origin, point);
        return {
            delta.Dot(gp_Vec(u)),
            delta.Dot(gp_Vec(v))};
    }

    double axial(const gp_Pnt& point) const
    {
        return gp_Vec(origin, point).Dot(gp_Vec(w));
    }

    gp_Pnt point(const PointD& planar, double axialValue) const
    {
        gp_Pnt result = origin;
        result.Translate(
            gp_Vec(u).Multiplied(planar.x) +
            gp_Vec(v).Multiplied(planar.y) +
            gp_Vec(w).Multiplied(axialValue));
        return result;
    }
};

// 一组「同一轴向区间 [bottom, top] × 同一半径」的刀心路径集合；
// area 是这些路径按刀具半径偏置并求并后的二维轮廓（懒计算）。
struct RawSweepGroup {
    double bottom = 0.0;
    double top = 0.0;
    double radius = 0.0;
    PathsD centrePaths;
    PathsD area;
};

// 刀具轮廓的自适应轴向分段：每段是半径恒定的环形带
// （区间 [bottomOffset, topOffset] 相对刀尖），牛鼻/球头由多段逼近。
struct ToolProfileBand {
    double bottomOffset = 0.0;
    double topOffset = 0.0;
    double radius = 0.0;
};

// 三维进刀段（直线/螺旋斜坡）的采样线段；fromW/toW 为刀尖轴向高度。
struct RampSegment {
    PointD from;
    PointD to;
    double fromW = 0.0;
    double toW = 0.0;
};

enum class RampSweepKind {
    Linear,
    Helical,
};

// 按 loop/depth/ring 聚合的同种斜坡进刀段集合，整组一起做轴向切片。
struct RawRampSweepGroup {
    RampSweepKind kind = RampSweepKind::Linear;
    int loopSequence = -1;
    int depthIndex = 0;
    int ring = 0;
    double minimumTip = std::numeric_limits<double>::infinity();
    double maximumTip = -std::numeric_limits<double>::infinity();
    int moveCount = 0;
    std::vector<RampSegment> segments;
};

// 最终去重后的一个轴向薄片：底部/顶部高度 + 该区间的二维并集轮廓，
// 将被拉伸成闭合棱柱。
struct AxialSlab {
    double bottom = 0.0;
    double top = 0.0;
    PathsD area;
};

struct SlabSolidBuild {
    std::vector<TopoDS_Shape> solids;
    int outerCount = 0;
    int holeCount = 0;
    int skippedInvalidCount = 0;
    bool succeeded = false;
};

template <typename Function>
void forEachIndex(
    std::size_t count,
    bool runParallel,
    Function&& function)
{
    if (count < kMinimumParallelTaskCount ||
        !runParallel) {
        for (std::size_t index = 0;
             index < count; ++index) {
            function(index);
        }
        return;
    }
    toolpathParallelForBudgeted(count, std::forward<Function>(function));
}

bool finitePoint(const gp_Pnt& point)
{
    return std::isfinite(point.X()) &&
           std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

bool finiteVector(const gp_Vec& vector)
{
    return std::isfinite(vector.X()) &&
           std::isfinite(vector.Y()) &&
           std::isfinite(vector.Z());
}

bool cancelled(const std::function<bool()>& cancellationRequested)
{
    return cancellationRequested && cancellationRequested();
}

double elapsedMilliseconds(const Clock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - start)
        .count();
}

struct SemanticCurveErrors {
    double maximumFitResidual = 0.0;
    double maximumChordSagitta = 0.0;
};

SemanticCurveErrors measureSemanticCurveErrors(
    const PocketToolpathOperation& operation,
    const FixedAxisFrame& frame,
    double tolerance)
{
    SemanticCurveErrors errors;
    for (const ToolpathPrimitive& primitive :
         operation.clearing.primitives) {
        if (primitive.type !=
                ToolpathPrimitiveType::CircularArc ||
            primitive.fallbackMoves.empty()) {
            continue;
        }
        const PointD center =
            frame.project(primitive.arc.center);
        const PointD start =
            frame.project(primitive.arc.from);
        const double radius = std::hypot(
            start.x - center.x,
            start.y - center.y);
        if (!std::isfinite(radius) || radius <= tolerance)
            continue;

        const auto measurePoint = [&](const gp_Pnt& point) {
            const PointD projected = frame.project(point);
            const double pointRadius = std::hypot(
                projected.x - center.x,
                projected.y - center.y);
            errors.maximumFitResidual = std::max(
                errors.maximumFitResidual,
                std::abs(pointRadius - radius));
        };
        measurePoint(primitive.arc.from);
        for (const ToolpathMove& move :
             primitive.fallbackMoves) {
            measurePoint(move.from);
            measurePoint(move.to);
            const PointD from = frame.project(move.from);
            const PointD to = frame.project(move.to);
            const double halfChord = 0.5 * std::hypot(
                to.x - from.x, to.y - from.y);
            const double boundedHalfChord = std::min(
                radius, halfChord);
            const double sagitta = radius - std::sqrt(
                std::max(
                    0.0,
                    radius * radius -
                        boundedHalfChord *
                            boundedHalfChord));
            errors.maximumChordSagitta = std::max(
                errors.maximumChordSagitta,
                sagitta);
        }
    }
    return errors;
}

class ScopedElapsedAccumulator {
public:
    explicit ScopedElapsedAccumulator(double& output)
        : m_output(output), m_start(Clock::now())
    {
    }

    ~ScopedElapsedAccumulator()
    {
        m_output += elapsedMilliseconds(m_start);
    }

private:
    double& m_output;
    Clock::time_point m_start;
};

bool makeFrame(
    const gp_Vec& sourceAxis,
    double tolerance,
    FixedAxisFrame& frame)
{
    if (!finiteVector(sourceAxis) ||
        sourceAxis.Magnitude() <= tolerance) {
        return false;
    }
    const gp_Dir axis(sourceAxis);
    const gp_Ax3 coordinateSystem(
        gp_Pnt(0.0, 0.0, 0.0), axis);
    frame.origin = coordinateSystem.Location();
    frame.u = coordinateSystem.XDirection();
    frame.v = coordinateSystem.YDirection();
    frame.w = coordinateSystem.Direction();
    return true;
}

bool stockAxialBounds(
    const TopoDS_Shape& stock,
    const FixedAxisFrame& frame,
    double& minimum,
    double& maximum)
{
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
    minimum = std::numeric_limits<double>::infinity();
    maximum = -std::numeric_limits<double>::infinity();
    for (int corner = 0; corner < 8; ++corner) {
        const gp_Pnt point(
            (corner & 1) != 0 ? xMax : xMin,
            (corner & 2) != 0 ? yMax : yMin,
            (corner & 4) != 0 ? zMax : zMin);
        const double value = frame.axial(point);
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
    }
    return std::isfinite(minimum) &&
           std::isfinite(maximum) &&
           maximum > minimum;
}

RawSweepGroup& findOrCreateGroup(
    std::vector<RawSweepGroup>& groups,
    double bottom,
    double top,
    double radius,
    double tolerance)
{
    const auto existing = std::find_if(
        groups.begin(), groups.end(),
        [&](const RawSweepGroup& group) {
            return std::abs(group.bottom - bottom) <= tolerance &&
                   std::abs(group.top - top) <= tolerance &&
                   std::abs(group.radius - radius) <= tolerance;
        });
    if (existing != groups.end())
        return *existing;
    groups.push_back(
        {bottom, top, radius, {}, {}});
    return groups.back();
}

RawRampSweepGroup& findOrCreateRampGroup(
    std::vector<RawRampSweepGroup>& groups,
    const ToolpathMove& move,
    RampSweepKind kind)
{
    const auto existing = std::find_if(
        groups.begin(), groups.end(),
        [&](const RawRampSweepGroup& group) {
            return group.kind == kind &&
                   group.loopSequence == move.loopSequence &&
                   group.depthIndex == move.depthIndex &&
                   group.ring == move.ring;
        });
    if (existing != groups.end())
        return *existing;
    RawRampSweepGroup group;
    group.kind = kind;
    group.loopSequence = move.loopSequence;
    group.depthIndex = move.depthIndex;
    group.ring = move.ring;
    groups.push_back(std::move(group));
    return groups.back();
}

RampSweepKind rampSweepKind(
    const ToolpathMove& move,
    const PocketToolpathOperation& operation)
{
    if (move.intent ==
        ToolpathMoveIntent::
            PocketHelicalRampEntry) {
        return RampSweepKind::Helical;
    }
    if (move.intent ==
        ToolpathMoveIntent::
            PocketLinearRampEntry) {
        return RampSweepKind::Linear;
    }
    // 兼容没有移动语义标记的旧刀路缓存。新生成刀路始终使用显式 intent，
    // 因此混合了螺旋成功与线性回退的工序也能准确分类。
    if (operation.clearing.helicalRampEntryCount > 0 &&
        operation.clearing.rampOnShapeEntryCount == 0 &&
        operation.clearing.linearRampEntryCount == 0 &&
        operation.clearing.zigzagRampEntryCount == 0) {
        return RampSweepKind::Helical;
    }
    return RampSweepKind::Linear;
}

// 把线段追加到组内的刀心路径：若上一段与当前段共线（叉积≈0、点积≥0）
// 则直接改写末端点合并成一条折线段，否则新开一段。
void appendLinePath(
    RawSweepGroup& group,
    const PointD& from,
    const PointD& to)
{
    constexpr double mergeTolerance = 1.0e-9;
    auto samePoint = [mergeTolerance](
                         const PointD& first,
                         const PointD& second) {
        return std::abs(first.x - second.x) <=
                   mergeTolerance &&
               std::abs(first.y - second.y) <=
                   mergeTolerance;
    };
    if (group.centrePaths.empty() ||
        group.centrePaths.back().empty() ||
        !samePoint(
            group.centrePaths.back().back(), from)) {
        group.centrePaths.push_back(
            PathD{from, to});
        return;
    }

    PathD& path = group.centrePaths.back();
    if (path.size() >= 2) {
        const PointD& previous =
            path[path.size() - 2u];
        const double firstX = from.x - previous.x;
        const double firstY = from.y - previous.y;
        const double secondX = to.x - from.x;
        const double secondY = to.y - from.y;
        const double firstLength =
            std::hypot(firstX, firstY);
        const double secondLength =
            std::hypot(secondX, secondY);
        const double cross =
            firstX * secondY - firstY * secondX;
        const double dot =
            firstX * secondX + firstY * secondY;
        if (firstLength > mergeTolerance &&
            secondLength > mergeTolerance &&
            std::abs(cross) <=
                mergeTolerance *
                    firstLength * secondLength &&
            dot >= 0.0) {
            path.back() = to;
            return;
        }
    }
    path.push_back(to);
}

void appendPointPath(
    RawSweepGroup& group,
    const PointD& point)
{
    group.centrePaths.push_back(
        PathD{point});
}

// 对一组刀心路径按 radius 做圆角偏置（Clipper2 InflatePaths），再求并
// 清理自交，得到刀具扫掠的二维轮廓。这是「恒深切削 → 环形带」的核心映射。
PathsD inflatedCentrePaths(
    const PathsD& centrePaths,
    double radius,
    int precision,
    double arcTolerance)
{
    if (centrePaths.empty())
        return {};
    PathsD area = Clipper2Lib::InflatePaths(
        centrePaths,
        radius,
        Clipper2Lib::JoinType::Round,
        Clipper2Lib::EndType::Round,
        2.0,
        precision,
        arcTolerance);
    return detail::cleanPaths(
        Clipper2Lib::Union(
            area,
            Clipper2Lib::FillRule::NonZero,
            precision),
        precision);
}

bool inflateMissingSweepGroups(
    std::vector<RawSweepGroup>& groups,
    int precision,
    double arcTolerance,
    bool runParallel)
{
    std::atomic_bool failed{false};
    forEachIndex(
        groups.size(), runParallel,
        [&](std::size_t index) {
            RawSweepGroup& group = groups[index];
            if (!group.area.empty() ||
                failed.load(std::memory_order_relaxed)) {
                return;
            }
            try {
                group.area = inflatedCentrePaths(
                    group.centrePaths,
                    group.radius,
                    precision, arcTolerance);
                if (group.area.empty()) {
                    failed.store(
                        true,
                        std::memory_order_relaxed);
                }
            } catch (...) {
                failed.store(
                    true,
                    std::memory_order_relaxed);
            }
        });
    return !failed.load(std::memory_order_relaxed);
}

// 判断斜坡扫掠是否已被平面恒深切削组的二维轮廓完全覆盖：
// 对每个刀具轮廓带，把斜坡路径同样偏置后逐轴区间与覆盖面积求差，
// 差集为空才认为可省略斜坡切片（避免重复去除体积）。
bool rampCoveredByPlanarSweeps(
    const RawRampSweepGroup& ramp,
    const std::vector<RawSweepGroup>& groups,
    const std::vector<ToolProfileBand>& profileBands,
    double stockMinimum,
    double stockMaximum,
    int precision,
    double arcTolerance,
    double tolerance)
{
    RawSweepGroup rampPathGroup;
    for (const RampSegment& segment : ramp.segments) {
        appendLinePath(
            rampPathGroup,
            segment.from,
            segment.to);
    }
    for (const ToolProfileBand& band :
         profileBands) {
        const double requiredBottom =
            std::max(
                ramp.minimumTip +
                    band.bottomOffset,
                stockMinimum);
        const double requiredTop =
            std::min(
                ramp.maximumTip +
                    band.topOffset,
                stockMaximum);
        if (requiredTop -
                requiredBottom <=
            tolerance) {
            continue;
        }
        const PathsD rampArea =
            inflatedCentrePaths(
                rampPathGroup.centrePaths,
                band.radius,
                precision, arcTolerance);
        if (rampArea.empty())
            return false;

        std::vector<double> boundaries{
            requiredBottom,
            requiredTop};
        for (const RawSweepGroup& group :
             groups) {
            if (group.top <=
                    requiredBottom + tolerance ||
                group.bottom >=
                    requiredTop - tolerance) {
                continue;
            }
            boundaries.push_back(
                std::clamp(
                    group.bottom,
                    requiredBottom,
                    requiredTop));
            boundaries.push_back(
                std::clamp(
                    group.top,
                    requiredBottom,
                    requiredTop));
        }
        std::sort(
            boundaries.begin(),
            boundaries.end());
        boundaries.erase(
            std::unique(
                boundaries.begin(),
                boundaries.end(),
                [&](double first,
                    double second) {
                    return std::abs(
                               first -
                               second) <=
                           tolerance;
                }),
            boundaries.end());
        if (boundaries.size() < 2)
            return false;
        for (std::size_t index = 1;
             index < boundaries.size();
             ++index) {
            const double slabBottom =
                boundaries[index - 1u];
            const double slabTop =
                boundaries[index];
            if (slabTop - slabBottom <=
                tolerance) {
                continue;
            }
            PathsD coveringArea;
            for (const RawSweepGroup& group :
                 groups) {
                if (group.bottom >
                        slabBottom + tolerance ||
                    group.top <
                        slabTop - tolerance) {
                    continue;
                }
                coveringArea.insert(
                    coveringArea.end(),
                    group.area.begin(),
                    group.area.end());
            }
            if (coveringArea.empty())
                return false;
            coveringArea =
                detail::cleanPaths(
                    Clipper2Lib::Union(
                        coveringArea,
                        Clipper2Lib::FillRule::
                            NonZero,
                        precision),
                    precision);
            const PathsD uncovered =
                detail::cleanPaths(
                    Clipper2Lib::Difference(
                        rampArea,
                        coveringArea,
                        Clipper2Lib::FillRule::
                            NonZero,
                        precision),
                    precision);
            if (!uncovered.empty())
                return false;
        }
    }
    return true;
}

PointD interpolate(
    const RampSegment& segment,
    double axialValue)
{
    const double denominator =
        segment.toW - segment.fromW;
    const double fraction =
        std::abs(denominator) <=
                std::numeric_limits<double>::epsilon()
            ? 0.0
            : std::clamp(
                  (axialValue - segment.fromW) /
                      denominator,
                  0.0, 1.0);
    return {
        segment.from.x +
            fraction *
                (segment.to.x - segment.from.x),
        segment.from.y +
            fraction *
                (segment.to.y - segment.from.y)};
}

// 把斜坡段按轴向步长切成多个切片，每个切片内用共有的内接轴向区间
// 叠加刀具轮廓带生成刀心路径。返回 false 表示切片数量超过预算。
bool appendRampSlices(
    const RawRampSweepGroup& ramp,
    std::vector<RawSweepGroup>& groups,
    const std::vector<ToolProfileBand>& profileBands,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double axialStep,
    double tolerance,
    int maximumSlices,
    FixedAxisLayeredBRepBuildResult& result)
{
    const double rampLow = std::max(
        ramp.minimumTip,
        stockMinimum - cuttingLength);
    const double rampHigh = std::min(
        ramp.maximumTip,
        stockMaximum);
    for (const RampSegment& segment : ramp.segments) {
        const double sourceLow =
            std::min(segment.fromW, segment.toW);
        const double sourceHigh =
            std::max(segment.fromW, segment.toW);
        const double low = std::max(
            sourceLow, stockMinimum - cuttingLength);
        const double high = std::min(
            sourceHigh, stockMaximum);
        if (high - low <= tolerance)
            continue;

        const long long firstBin =
            static_cast<long long>(std::floor(
                (low - stockMinimum) / axialStep));
        const long long lastBin =
            static_cast<long long>(std::ceil(
                (high - stockMinimum) / axialStep)) -
            1LL;
        for (long long bin = firstBin;
             bin <= lastBin; ++bin) {
            const double binBottom =
                stockMinimum +
                static_cast<double>(bin) * axialStep;
            const double binTop =
                binBottom + axialStep;
            const double pieceLow =
                std::max(low, binBottom);
            const double pieceHigh =
                std::min(high, binTop);
            if (pieceHigh - pieceLow <= tolerance)
                continue;

            const PointD first =
                interpolate(segment, pieceLow);
            const PointD second =
                interpolate(segment, pieceHigh);
            // 同一刀尖轴向带内的所有采样段统一使用共有内接区间；再叠加
            // 刀具轮廓内接带。层数只随切深和刀具轮廓精度增长，不随螺旋
            // 采样点数放大，且每个棱柱都严格位于真实扫掠内部。
            const double layerLow =
                std::max(rampLow, binBottom);
            const double layerHigh =
                std::min(rampHigh, binTop);
            for (const ToolProfileBand& band :
                 profileBands) {
                const double bottom =
                    std::max(
                        layerHigh +
                            band.bottomOffset,
                        stockMinimum);
                const double top =
                    std::min(
                        layerLow +
                            band.topOffset,
                        stockMaximum);
                if (top - bottom <= tolerance)
                    continue;
                if (result.rampSweepSliceCount >=
                    maximumSlices) {
                    return false;
                }
                RawSweepGroup& group =
                    findOrCreateGroup(
                        groups, bottom, top,
                        band.radius,
                        tolerance);
                appendLinePath(
                    group, first, second);
                ++result.rampSweepSliceCount;
            }
            result.maximumRampAxialError =
                std::max(
                    result.maximumRampAxialError,
                    axialStep);
        }
    }
    return true;
}

std::vector<double> uniqueBoundaries(
    const std::vector<RawSweepGroup>& groups,
    double tolerance)
{
    std::vector<double> boundaries;
    boundaries.reserve(groups.size() * 2u);
    for (const RawSweepGroup& group : groups) {
        boundaries.push_back(group.bottom);
        boundaries.push_back(group.top);
    }
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(
        std::unique(
            boundaries.begin(), boundaries.end(),
            [&](double first, double second) {
                return std::abs(first - second) <= tolerance;
            }),
        boundaries.end());
    return boundaries;
}

bool samePaths(
    const PathsD& first,
    const PathsD& second,
    double tolerance)
{
    if (first.size() != second.size())
        return false;
    for (std::size_t pathIndex = 0;
         pathIndex < first.size(); ++pathIndex) {
        const PathD& firstPath = first[pathIndex];
        const PathD& secondPath = second[pathIndex];
        if (firstPath.size() != secondPath.size())
            return false;
        for (std::size_t pointIndex = 0;
             pointIndex < firstPath.size(); ++pointIndex) {
            if (std::abs(
                    firstPath[pointIndex].x -
                    secondPath[pointIndex].x) > tolerance ||
                std::abs(
                    firstPath[pointIndex].y -
                    secondPath[pointIndex].y) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

bool shareCommonTop(
    const std::vector<RawSweepGroup>& groups,
    double tolerance)
{
    if (groups.empty())
        return false;
    const double commonTop = groups.front().top;
    return std::all_of(
        groups.begin(), groups.end(),
        [&](const RawSweepGroup& group) {
            return std::abs(
                       group.top - commonTop) <=
                   tolerance;
        });
}

PathsD unionAreas(
    const PathsD& first,
    const PathsD& second,
    int precision)
{
    if (first.empty())
        return second;
    if (second.empty())
        return first;
    PathsD combined = first;
    combined.insert(
        combined.end(),
        second.begin(), second.end());
    return detail::cleanPaths(
        Clipper2Lib::Union(
            combined,
            Clipper2Lib::FillRule::NonZero,
            precision),
        precision);
}

bool inclusivePrefixUnions(
    const std::vector<PathsD>& leaves,
    int precision,
    bool runParallel,
    const std::function<bool()>& cancellationRequested,
    bool& wasCancelled,
    std::vector<PathsD>& prefixes)
{
    prefixes.clear();
    if (leaves.empty())
        return true;

    std::size_t treeSize = 1u;
    while (treeSize < leaves.size())
        treeSize *= 2u;
    std::vector<PathsD> tree(treeSize);
    std::copy(
        leaves.begin(), leaves.end(),
        tree.begin());
    std::atomic_bool failed{false};

    // Blelloch 扫描以 O(n) 次二维并集计算所有累计层，且每一树层可并行。
    // 相比第 i 层从头合并 0..i 的 O(n²) 输入量，深腔层数增加时不会
    // 反复处理已经完成并集的浅层轮廓。
    for (std::size_t width = 2u;
         width <= treeSize;
         width *= 2u) {
        const std::size_t taskCount =
            treeSize / width;
        forEachIndex(
            taskCount, runParallel,
            [&](std::size_t taskIndex) {
                if (failed.load(
                        std::memory_order_relaxed)) {
                    return;
                }
                const std::size_t right =
                    (taskIndex + 1u) * width - 1u;
                const std::size_t left =
                    right - width / 2u;
                try {
                    tree[right] = unionAreas(
                        tree[left], tree[right],
                        precision);
                } catch (...) {
                    failed.store(
                        true,
                        std::memory_order_relaxed);
                }
            });
        if (failed.load(
                std::memory_order_relaxed)) {
            return false;
        }
        if (cancelled(cancellationRequested)) {
            wasCancelled = true;
            return false;
        }
    }

    tree.back().clear();
    for (std::size_t width = treeSize;
         width >= 2u;
         width /= 2u) {
        const std::size_t taskCount =
            treeSize / width;
        forEachIndex(
            taskCount, runParallel,
            [&](std::size_t taskIndex) {
                if (failed.load(
                        std::memory_order_relaxed)) {
                    return;
                }
                const std::size_t right =
                    (taskIndex + 1u) * width - 1u;
                const std::size_t left =
                    right - width / 2u;
                try {
                    PathsD leftAggregate =
                        std::move(tree[left]);
                    PathsD parentPrefix =
                        tree[right];
                    PathsD rightPrefix =
                        unionAreas(
                            parentPrefix,
                            leftAggregate,
                            precision);
                    tree[left] =
                        std::move(parentPrefix);
                    tree[right] =
                        std::move(rightPrefix);
                } catch (...) {
                    failed.store(
                        true,
                        std::memory_order_relaxed);
                }
            });
        if (failed.load(
                std::memory_order_relaxed)) {
            return false;
        }
        if (cancelled(cancellationRequested)) {
            wasCancelled = true;
            return false;
        }
    }

    prefixes.resize(leaves.size());
    forEachIndex(
        leaves.size(), runParallel,
        [&](std::size_t index) {
            if (failed.load(
                    std::memory_order_relaxed)) {
                return;
            }
            try {
                prefixes[index] =
                    unionAreas(
                        tree[index],
                        leaves[index],
                        precision);
            } catch (...) {
                failed.store(
                    true,
                    std::memory_order_relaxed);
            }
        });
    return !failed.load(
        std::memory_order_relaxed);
}

std::vector<AxialSlab> makeCommonTopSlabs(
    const std::vector<RawSweepGroup>& groups,
    const std::vector<double>& boundaries,
    int precision,
    double tolerance,
    bool runParallel,
    const std::function<bool()>& cancellationRequested,
    bool& wasCancelled)
{
    std::vector<AxialSlab> slabs;
    if (boundaries.size() < 2u)
        return slabs;

    // 刀刃覆盖至同一毛坯顶面的固定轴工序是最常见情形。每个轴向
    // 区间只记录刚进入的轮廓，再由并行前缀扫描生成完整累计面积。
    const std::size_t candidateCount =
        boundaries.size() - 1u;
    std::vector<PathsD> additions(
        candidateCount);
    std::vector<unsigned char> activated(
        groups.size(), 0u);
    std::vector<int> additionGroupCounts(
        candidateCount, 0);
    for (std::size_t index = 0;
         index < candidateCount;
         ++index) {
        if (cancelled(cancellationRequested)) {
            wasCancelled = true;
            return {};
        }
        const double bottom = boundaries[index];
        const double top = boundaries[index + 1u];
        if (top - bottom <= tolerance)
            continue;

        for (std::size_t groupIndex = 0;
             groupIndex < groups.size();
             ++groupIndex) {
            if (activated[groupIndex] != 0u)
                continue;
            const RawSweepGroup& group =
                groups[groupIndex];
            if (group.bottom >= top - tolerance ||
                group.top <= bottom + tolerance) {
                continue;
            }
            additions[index].insert(
                additions[index].end(),
                group.area.begin(),
                group.area.end());
            activated[groupIndex] = 1u;
            ++additionGroupCounts[index];
        }
    }

    std::atomic_bool normalizationFailed{false};
    forEachIndex(
        additions.size(), runParallel,
        [&](std::size_t index) {
            if (additionGroupCounts[index] <= 1 ||
                normalizationFailed.load(
                    std::memory_order_relaxed)) {
                return;
            }
            try {
                additions[index] =
                    detail::cleanPaths(
                        Clipper2Lib::Union(
                            additions[index],
                            Clipper2Lib::FillRule::NonZero,
                            precision),
                        precision);
            } catch (...) {
                normalizationFailed.store(
                    true,
                    std::memory_order_relaxed);
            }
        });
    if (normalizationFailed.load(
            std::memory_order_relaxed)) {
        return {};
    }

    std::vector<PathsD> cumulativeAreas;
    if (!inclusivePrefixUnions(
            additions, precision,
            runParallel,
            cancellationRequested,
            wasCancelled,
            cumulativeAreas)) {
        return {};
    }

    slabs.reserve(candidateCount);
    for (std::size_t index = 0;
         index < candidateCount;
         ++index) {
        const double bottom = boundaries[index];
        const double top = boundaries[index + 1u];
        PathsD& area = cumulativeAreas[index];
        if (top - bottom <= tolerance ||
            area.empty()) {
            continue;
        }

        if (!slabs.empty() &&
            std::abs(
                slabs.back().top - bottom) <=
                tolerance &&
            samePaths(
                slabs.back().area,
                area,
                tolerance)) {
            slabs.back().top = top;
        } else {
            slabs.push_back(
                {bottom, top,
                 std::move(area)});
        }
    }
    return slabs;
}

std::vector<AxialSlab> makeSlabs(
    const std::vector<RawSweepGroup>& groups,
    int precision,
    double tolerance,
    bool runParallel,
    const std::function<bool()>& cancellationRequested,
    bool& wasCancelled)
{
    std::vector<AxialSlab> slabs;
    const std::vector<double> boundaries =
        uniqueBoundaries(groups, tolerance);
    if (boundaries.size() < 2u)
        return slabs;
    if (cancelled(cancellationRequested)) {
        wasCancelled = true;
        return {};
    }
    if (shareCommonTop(groups, tolerance)) {
        return makeCommonTopSlabs(
            groups, boundaries,
            precision, tolerance,
            runParallel,
            cancellationRequested,
            wasCancelled);
    }
    const std::size_t candidateCount =
        boundaries.size() - 1u;
    std::vector<AxialSlab> candidates(candidateCount);
    std::vector<unsigned char> candidateValid(
        candidateCount, 0u);
    std::atomic_bool failed{false};
    forEachIndex(
        candidateCount, runParallel,
        [&](std::size_t index) {
            if (failed.load(std::memory_order_relaxed))
                return;
            try {
                const double bottom = boundaries[index];
                const double top = boundaries[index + 1u];
                if (top - bottom <= tolerance)
                    return;

                PathsD combined;
                for (const RawSweepGroup& group : groups) {
                    if (group.bottom >= top - tolerance ||
                        group.top <= bottom + tolerance) {
                        continue;
                    }
                    combined.insert(
                        combined.end(),
                        group.area.begin(), group.area.end());
                }
                if (combined.empty())
                    return;
                combined = detail::cleanPaths(
                    Clipper2Lib::Union(
                        combined,
                        Clipper2Lib::FillRule::NonZero,
                        precision),
                    precision);
                if (combined.empty())
                    return;
                candidates[index] = {
                    bottom, top, std::move(combined)};
                candidateValid[index] = 1u;
            } catch (...) {
                failed.store(
                    true,
                    std::memory_order_relaxed);
            }
        });
    if (failed.load(std::memory_order_relaxed))
        return {};
    if (cancelled(cancellationRequested)) {
        wasCancelled = true;
        return {};
    }

    for (std::size_t index = 0;
         index < candidateCount; ++index) {
        if (candidateValid[index] == 0u)
            continue;
        AxialSlab& candidate = candidates[index];
        if (!slabs.empty() &&
            std::abs(
                slabs.back().top -
                candidate.bottom) <= tolerance &&
            samePaths(
                slabs.back().area,
                candidate.area,
                tolerance)) {
            slabs.back().top = candidate.top;
        } else {
            slabs.push_back(std::move(candidate));
        }
    }
    return slabs;
}

TopoDS_Wire makeWire(
    PathD path,
    const FixedAxisFrame& frame,
    double axialValue,
    bool positive)
{
    const double area = Clipper2Lib::Area(path);
    if ((positive && area < 0.0) ||
        (!positive && area > 0.0)) {
        std::reverse(path.begin(), path.end());
    }

    BRepBuilderAPI_MakePolygon polygon;
    for (const PointD& point : path)
        polygon.Add(frame.point(point, axialValue));
    polygon.Close();
    return polygon.IsDone()
        ? polygon.Wire()
        : TopoDS_Wire{};
}

bool pathInside(
    const PathD& candidate,
    const PathD& container)
{
    if (candidate.empty() || container.empty())
        return false;
    return Clipper2Lib::PointInPolygon(
               candidate.front(), container) ==
           Clipper2Lib::PointInPolygonResult::IsInside;
}

// 把单个轴向薄片的二维轮廓提升为闭合棱柱：按嵌套层级区分外环/内孔，
// 外环建面、内孔以反向线框加入孔洞，校验通过后沿 w 轴拉伸。
// 退化/无效面会被跳过并计数，不使整层失败。
bool buildSlabSolids(
    const AxialSlab& slab,
    const FixedAxisFrame& frame,
    SlabSolidBuild& result)
{
    const std::vector<PathInfo> infos =
        detail::sortedPathInfos(slab.area);
    const gp_Vec extrusion =
        gp_Vec(frame.w).Multiplied(
            slab.top - slab.bottom);
    bool builtAny = false;
    bool skippedAny = false;

    for (std::size_t outerIndex = 0;
         outerIndex < infos.size(); ++outerIndex) {
        const PathInfo& outer = infos[outerIndex];
        if ((outer.nesting % 2) != 0 ||
            outer.path.size() < 3) {
            continue;
        }
        const TopoDS_Wire outerWire =
            makeWire(
                outer.path, frame,
                slab.bottom, true);
        if (outerWire.IsNull())
            return false;

        BRepBuilderAPI_MakeFace faceBuilder(outerWire);
        ++result.outerCount;
        for (std::size_t holeIndex = 0;
             holeIndex < infos.size(); ++holeIndex) {
            const PathInfo& hole = infos[holeIndex];
            if (hole.nesting != outer.nesting + 1 ||
                hole.path.size() < 3 ||
                !pathInside(hole.path, outer.path)) {
                continue;
            }
            const TopoDS_Wire holeWire =
                makeWire(
                    hole.path, frame,
                    slab.bottom, false);
            if (holeWire.IsNull())
                return false;
            faceBuilder.Add(holeWire);
            ++result.holeCount;
        }
        faceBuilder.Build();
        if (!faceBuilder.IsDone())
            return false;

        TopoDS_Face face = faceBuilder.Face();
        if (!BRepCheck_Analyzer(
                 face,
                 Standard_True,
                 Standard_False)
                 .IsValid()) {
            ++result.skippedInvalidCount;
            skippedAny = true;
            continue;
        }
        BRepPrimAPI_MakePrism prism(
            face, extrusion,
            Standard_True, Standard_True);
        if (!prism.IsDone() ||
            prism.Shape().IsNull()) {
            return false;
        }
        TopoDS_Shape prismShape =
            prism.Shape();
        result.solids.push_back(
            std::move(prismShape));
        builtAny = true;
    }
    result.succeeded = builtAny || skippedAny;
    return result.succeeded;
}

} // namespace

FixedAxisLayeredBRepBuildResult
FixedAxisLayeredBRepBuilder::build(
    const TopoDS_Shape& stock,
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    // 主流程（Clipper 分层路径）：① 校验输入并建立刀轴局部坐标系 →
    // ② 构建刀具轮廓自适应环形带 → ③ 收集恒深切削组与斜坡组 →
    // ④ 刀心路径半径偏置 → ⑤ 斜坡切片/包含判定 → ⑥ 轴向薄片求并 →
    // ⑦ 逐层拉伸为闭合棱柱并打包 Compound。解析快路径失败或不合资格时
    // 才走到这里（见头文件注释）。
    FixedAxisLayeredBRepBuildResult result;
    const Clock::time_point buildStart = Clock::now();
    const auto finish = [&] {
        result.buildMilliseconds =
            elapsedMilliseconds(buildStart);
        return result;
    };

    const double tolerance = std::max(
        options.geometricTolerance,
        Precision::Confusion());
    if (stock.IsNull()) {
        result.error = "固定轴 BRep 要求毛坯非空";
        return finish();
    }
    if (!std::isfinite(params.tool.diameter) ||
        params.tool.diameter <= tolerance) {
        result.error =
            "固定轴 BRep 要求平底刀直径为有限正数";
        return finish();
    }
    if (!std::isfinite(params.tool.cornerRadius) ||
        params.tool.cornerRadius < -tolerance) {
        result.error =
            "固定轴 BRep 要求刀角半径为有限非负数";
        return finish();
    }
    if (!std::isfinite(params.tool.cuttingEdgeLength) ||
        params.tool.cuttingEdgeLength <= tolerance) {
        result.unsupported = true;
        result.error =
            "固定轴 BRep 无法解析有效刀刃长度";
        return finish();
    }
    FixedAxisFrame frame;
    if (!makeFrame(
            operation.outwardToolAxis,
            tolerance, frame)) {
        result.error =
            "固定轴 BRep 要求工序具有有效刀轴";
        return finish();
    }
    double stockMinimum = 0.0;
    double stockMaximum = 0.0;
    if (!stockAxialBounds(
            stock, frame,
            stockMinimum, stockMaximum)) {
        result.error =
            "固定轴 BRep 无法确定毛坯沿刀轴方向的边界";
        return finish();
    }
    const SemanticCurveErrors semanticCurveErrors =
        measureSemanticCurveErrors(
            operation, frame, tolerance);
    result.maximumCurveFitError =
        semanticCurveErrors.maximumFitResidual;

    const double radius =
        0.5 * params.tool.diameter;
    const double cornerRadius =
        std::max(0.0, params.tool.cornerRadius);
    const double cuttingLength =
        params.tool.cuttingEdgeLength;
    if (cornerRadius > radius + tolerance) {
        result.error =
            "固定轴 BRep 要求刀角半径不大于刀具半径";
        return finish();
    }
    // 让解析端点优先选择。特别是,精确的整球偏置不需要回退的半径带;
    // 若要求带构建器在奇异的球尖处满足详细的径向预算,可能拒绝一个
    // 解析轮廓误差为零的表达。
    const AnalyticFixedAxisLayeredBRepBuildResult analytic =
        AnalyticFixedAxisLayeredBRepBuilder().build(
            stock, operation, params, options,
            cancellationRequested);
    result.analyticPlanarAttempted = analytic.attempted;
    result.analyticPlanarMilliseconds =
        analytic.buildMilliseconds;
    result.analyticPlanarPrimitiveCount =
        analytic.primitiveCount;
    result.analyticPlanarArcCount =
        analytic.arcCount;
    result.analyticPlanarBooleanGroupCount =
        analytic.booleanGroupCount;
    if (analytic.cancelled) {
        result.cancelled = true;
        return finish();
    }
    if (analytic.succeeded) {
        result.analyticPlanarUsed = true;
        result.linearRampMoveCount =
            analytic.linearRampCount;
        result.helicalRampMoveCount =
            analytic.helicalRampMoveCount;
        result.rampSweepGroupCount =
            analytic.linearRampGroupCount +
            analytic.helicalRampGroupCount;
        result.containedRampSweepGroupCount =
            analytic.containedRampGroupCount;
        result.containedRampMoveCount =
            analytic.containedRampCount;
        result.rampSweepSliceCount =
            analytic.rampSweepSliceCount;
        result.maximumRampAxialError =
            analytic.maximumRampAxialError;
        result.rampProcessingMilliseconds =
            analytic.rampProcessingMilliseconds;
        result.cuttingMoveCount =
            analytic.cuttingMoveCount;
        result.planarLayerCount =
            analytic.planarLayerCount;
        result.removalSolidCount =
            analytic.removalSolidCount;
        result.footprintOuterCount =
            analytic.footprintOuterCount;
        result.footprintHoleCount =
            analytic.footprintHoleCount;
        result.footprintArea = analytic.footprintArea;
        result.maximumFootprintArea =
            analytic.maximumFootprintArea;
        result.pathCollectionMilliseconds =
            analytic.pathCollectionMilliseconds;
        result.slabUnionMilliseconds =
            analytic.planarBooleanMilliseconds;
        result.solidBuildMilliseconds =
            analytic.solidBuildMilliseconds;
        if (analytic.closedOffsetFillUsed) {
            result.maximumCurveFitError =
                analytic.maximumBoundaryFitError;
            // 填充轮廓要么使用已接受的语义圆弧本身,要么逐字使用其原始
            // 回退线段。任何输出曲线都不由弦表示,因此弦的弓高不是该
            // 路径的端点误差。
            result.maximumChordError = 0.0;
            result.maximumToolProfileAxialError =
                analytic.maximumToolProfileAxialError;
            result.maximumToolProfileRadialError =
                analytic.maximumToolProfileRadialError;
            result.toolProfileBandCount =
                analytic.toolProfileBandCount;
        }
        result.noMaterial = analytic.noMaterial;
        result.removalVolume = analytic.removalVolume;
        return finish();
    }
    if (analytic.attempted) {
        result.analyticPlanarFallbackOperationCount = 1;
        result.analyticPlanarFallbackReason = analytic.reason;
    }

    const double profileAxialTolerance =
        std::isfinite(
            options.toolProfileAxialTolerance) &&
                options.toolProfileAxialTolerance >
                    tolerance
            ? options.toolProfileAxialTolerance
            : std::max(
                  std::clamp(
                      0.05 * cornerRadius,
                      0.05, 2.5),
                  std::isfinite(
                          options.arcChordTolerance)
                      ? options.arcChordTolerance
                      : 0.0);
    const double curvedProfileHeight =
        std::min(cornerRadius, cuttingLength);
    const double defaultRadialTolerance =
        curvedProfileHeight > tolerance
        ? std::max(
              profileAxialTolerance,
              std::sqrt(std::max(
                  0.0,
                  2.0 * cornerRadius *
                          std::min(
                              profileAxialTolerance,
                              curvedProfileHeight) -
                      std::pow(
                          std::min(
                              profileAxialTolerance,
                              curvedProfileHeight),
                          2.0))) +
                  tolerance)
        : profileAxialTolerance;
    const double profileRadialTolerance =
        std::isfinite(
            options.toolProfileRadialTolerance) &&
                options.toolProfileRadialTolerance > tolerance
            ? options.toolProfileRadialTolerance
            : defaultRadialTolerance;
    DexelToolProfile toolProfile;
    std::string toolProfileError;
    std::vector<ToolProfileBand> profileBands;
    if (!buildDexelToolProfile(
            {DexelToolProfileFamily::RoundedEnd,
             2.0 * radius, cuttingLength,
             cornerRadius, 0.0},
            {profileAxialTolerance,
             profileRadialTolerance,
             tolerance,
             std::max(1, options.maxToolProfileBands)},
            toolProfile, &toolProfileError)) {
        result.error = toolProfileError.find("budget") !=
                std::string::npos
            ? "固定轴 BRep 刀具轮廓分层数量超过预算上限；请增大刀具轮廓轴向或径向容差"
            : "固定轴 BRep 刀具轮廓自适应分层失败：" +
                  toolProfileError;
        return finish();
    }
    profileBands.reserve(toolProfile.bands.size());
    for (const DexelToolProfileBand& band : toolProfile.bands) {
        profileBands.push_back(
            {band.lower, band.upper, band.radius});
    }
    result.maximumToolProfileAxialError =
        toolProfile.maximumAxialError;
    result.maximumToolProfileRadialError =
        toolProfile.maximumRadialError;
    result.toolProfileBandCount =
        static_cast<int>(profileBands.size());

    result.maximumChordError =
        semanticCurveErrors.maximumChordSagitta;

    std::vector<RawSweepGroup> groups;
    std::vector<RawRampSweepGroup> rampGroups;
    {
        ScopedElapsedAccumulator pathCollectionTimer(
            result.pathCollectionMilliseconds);
        for (const ToolpathMove& move :
             operation.clearing.moves) {
            if (move.type != ToolpathMoveType::Cut &&
                move.type != ToolpathMoveType::Plunge) {
                continue;
            }
            if (cancelled(cancellationRequested)) {
                result.cancelled = true;
                return finish();
            }
            if (!finitePoint(move.from) ||
                !finitePoint(move.to)) {
                result.error =
                    "固定轴 BRep 收到包含非有限坐标的切削移动";
                return finish();
            }
            ++result.cuttingMoveCount;
            if (result.cuttingMoveCount >
                options.maxSweepSegments) {
                result.error =
                    "固定轴 BRep 切削移动数量超过预算上限";
                return finish();
            }

            const PointD from = frame.project(move.from);
            const PointD to = frame.project(move.to);
            const double fromW = frame.axial(move.from);
            const double toW = frame.axial(move.to);
            const double planarDistance =
                std::hypot(to.x - from.x, to.y - from.y);
            const double axialDistance =
                std::abs(toW - fromW);

            bool pointSweep = false;
            double tipLow = 0.0;
            double tipHigh = 0.0;
            if (move.type == ToolpathMoveType::Cut) {
                if (axialDistance > tolerance) {
                    result.unsupported = true;
                    result.error =
                        "固定轴 BRep 仅在下刀段支持轴向变化；普通切削段必须保持恒深";
                    return finish();
                }
                if (planarDistance <= tolerance)
                    continue;
                tipLow = 0.5 * (fromW + toW);
                tipHigh = tipLow;
            } else {
                if (planarDistance > tolerance) {
                    if (axialDistance <= tolerance)
                        continue;
                    const RampSweepKind kind =
                        rampSweepKind(move, operation);
                    RawRampSweepGroup& ramp =
                        findOrCreateRampGroup(
                            rampGroups, move, kind);
                    ramp.minimumTip = std::min(
                        ramp.minimumTip,
                        std::min(fromW, toW));
                    ramp.maximumTip = std::max(
                        ramp.maximumTip,
                        std::max(fromW, toW));
                    ramp.segments.push_back(
                        {from, to, fromW, toW});
                    ++ramp.moveCount;
                    if (kind ==
                        RampSweepKind::Helical) {
                        ++result.helicalRampMoveCount;
                    } else {
                        ++result.linearRampMoveCount;
                    }
                    continue;
                }
                if (axialDistance <= tolerance)
                    continue;
                tipLow = std::min(fromW, toW);
                tipHigh = std::max(fromW, toW);
                pointSweep = true;
            }

            for (const ToolProfileBand& band :
                 profileBands) {
                const double bottom =
                    std::max(
                        tipLow + band.bottomOffset,
                        stockMinimum);
                const double top =
                    std::min(
                        tipHigh + band.topOffset,
                        stockMaximum);
                if (top - bottom <= tolerance)
                    continue;
                RawSweepGroup& group =
                    findOrCreateGroup(
                        groups, bottom, top,
                        band.radius, tolerance);
                if (pointSweep)
                    appendPointPath(group, from);
                else
                    appendLinePath(
                        group, from, to);
            }
        }
    }

    const int precision = std::clamp(
        params.clearing.precision, 0, 8);
    const double arcTolerance =
        std::isfinite(options.arcChordTolerance) &&
                options.arcChordTolerance > 0.0
            ? options.arcChordTolerance
            : 0.0;
    if (cancelled(cancellationRequested)) {
        result.cancelled = true;
        return finish();
    }
    const Clock::time_point initialInflationStart =
        Clock::now();
    const bool initialInflationSucceeded =
        inflateMissingSweepGroups(
            groups, precision, arcTolerance,
            options.runParallel);
    result.sweepInflationMilliseconds +=
        elapsedMilliseconds(
            initialInflationStart);
    if (!initialInflationSucceeded) {
        result.error =
            "固定轴 BRep 无法对刀心路径执行刀具半径偏置";
        return finish();
    }

    const double requestedRampAxialTolerance =
        std::isfinite(options.rampAxialTolerance) &&
                options.rampAxialTolerance > tolerance
            ? options.rampAxialTolerance
            : std::max(0.05, arcTolerance);
    double rampAxialTolerance =
        requestedRampAxialTolerance;
    if (cornerRadius > tolerance) {
        // 三维入刀的刀尖高度变化必须小于曲面轮廓带高度，否则该带在
        // 公共内接轴向区间中会完全消失。使用半带高可保证每个正半径
        // 球面/圆角带都能生成非退化棱柱，同时保持规则轴向网格。
        for (const ToolProfileBand& band :
             profileBands) {
            if (band.bottomOffset >=
                cornerRadius - tolerance) {
                continue;
            }
            const double bandHeight =
                band.topOffset -
                band.bottomOffset;
            if (bandHeight > tolerance) {
                rampAxialTolerance =
                    std::min(
                        rampAxialTolerance,
                        0.5 * bandHeight);
            }
        }
    }
    const int maximumRampSlices =
        std::max(1, options.maxRampSweepSlices);
    result.rampSweepGroupCount =
        static_cast<int>(rampGroups.size());
    const long long containmentCheckCount =
        static_cast<long long>(
            rampGroups.size()) *
        static_cast<long long>(
            profileBands.size());
    const bool checkRampContainment =
        !groups.empty() &&
        options.maxRampContainmentChecks > 0 &&
        containmentCheckCount <=
            static_cast<long long>(
                options.maxRampContainmentChecks);
    if (!checkRampContainment) {
        result.skippedRampContainmentGroupCount =
            static_cast<int>(rampGroups.size());
    }
    const Clock::time_point rampProcessingStart =
        Clock::now();
    for (const RawRampSweepGroup& ramp :
         rampGroups) {
        if (cancelled(cancellationRequested)) {
            result.cancelled = true;
            return finish();
        }
        if (checkRampContainment &&
            rampCoveredByPlanarSweeps(
                ramp, groups,
                profileBands,
                stockMinimum, stockMaximum,
                precision,
                arcTolerance, tolerance)) {
            ++result.containedRampSweepGroupCount;
            result.containedRampMoveCount +=
                ramp.moveCount;
            continue;
        }
        if (!appendRampSlices(
                ramp, groups,
                profileBands,
                cuttingLength,
                stockMinimum, stockMaximum,
                rampAxialTolerance,
                tolerance,
                maximumRampSlices,
                result)) {
            result.error =
                "固定轴 BRep 三维入刀分层数量超过预算上限；请增大斜坡轴向容差或拆分工序";
            return finish();
        }
    }
    result.rampProcessingMilliseconds +=
        elapsedMilliseconds(
            rampProcessingStart);

    if (cancelled(cancellationRequested)) {
        result.cancelled = true;
        return finish();
    }
    const Clock::time_point rampInflationStart =
        Clock::now();
    const bool rampInflationSucceeded =
        inflateMissingSweepGroups(
            groups, precision, arcTolerance,
            options.runParallel);
    result.sweepInflationMilliseconds +=
        elapsedMilliseconds(
            rampInflationStart);
    if (!rampInflationSucceeded) {
        result.error =
            "固定轴 BRep 无法构造三维入刀的二维扫掠轮廓";
        return finish();
    }

    if (groups.empty()) {
        result.noMaterial = true;
        return finish();
    }

    bool slabCancellation = false;
    const Clock::time_point slabUnionStart =
        Clock::now();
    const std::vector<AxialSlab> slabs =
        makeSlabs(
            groups, precision, tolerance,
            options.runParallel,
            cancellationRequested,
            slabCancellation);
    result.slabUnionMilliseconds =
        elapsedMilliseconds(slabUnionStart);
    if (slabCancellation) {
        result.cancelled = true;
        return finish();
    }
    if (slabs.empty()) {
        result.error =
            "固定轴 BRep 未生成任何轴向减料层";
        return finish();
    }

    TopoDS_Compound removal;
    BRep_Builder compoundBuilder;
    compoundBuilder.MakeCompound(removal);
    const Clock::time_point solidBuildStart =
        Clock::now();
    if (cancelled(cancellationRequested)) {
        result.cancelled = true;
        return finish();
    }
    std::vector<SlabSolidBuild> slabBuilds(
        slabs.size());
    std::atomic_bool slabBuildFailed{false};
    forEachIndex(
        slabs.size(), options.runParallel,
        [&](std::size_t index) {
            if (slabBuildFailed.load(
                    std::memory_order_relaxed)) {
                return;
            }
            try {
                if (!buildSlabSolids(
                        slabs[index], frame,
                        slabBuilds[index])) {
                    slabBuildFailed.store(
                        true,
                        std::memory_order_relaxed);
                }
            } catch (...) {
                slabBuildFailed.store(
                    true,
                    std::memory_order_relaxed);
            }
        });
    if (cancelled(cancellationRequested)) {
        result.cancelled = true;
        return finish();
    }
    if (slabBuildFailed.load(
            std::memory_order_relaxed)) {
        result.error =
            "固定轴 BRep 无法将平面减料层拉伸为三维实体";
        return finish();
    }

    for (std::size_t index = 0;
         index < slabs.size(); ++index) {
        const AxialSlab& slab = slabs[index];
        SlabSolidBuild& slabBuild =
            slabBuilds[index];
        const double area = std::abs(
            Clipper2Lib::Area(slab.area));
        result.footprintArea += area;
        result.maximumFootprintArea =
            std::max(
                result.maximumFootprintArea,
                area);
        result.footprintOuterCount +=
            slabBuild.outerCount;
        result.footprintHoleCount +=
            slabBuild.holeCount;
        result.skippedInvalidRemovalSolidCount +=
            slabBuild.skippedInvalidCount;
        for (const TopoDS_Shape& solid :
             slabBuild.solids) {
            compoundBuilder.Add(removal, solid);
            ++result.removalSolidCount;
        }
        ++result.planarLayerCount;
    }
    if (result.removalSolidCount <= 0) {
        result.error =
            "固定轴 BRep 未生成任何闭合减料实体";
        return finish();
    }
    result.removalVolume = removal;
    result.solidBuildMilliseconds =
        elapsedMilliseconds(solidBuildStart);
    return finish();
}

} // namespace oap::toolpath
