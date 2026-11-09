// 在指定加工深度截取实体轮廓，把离散交线连接为槽腔边界和岛屿环。
#include "toolpath/pocket/planning/PocketLayerSlicer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <iterator>
#include <limits>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <utility>

#include <BRepAlgoAPI_Section.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GProp_GProps.hxx>
#include <Message_ProgressIndicator.hxx>
#include <OSD_Parallel.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Vec.hxx>

#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/ToolpathOcctParallel.h"

namespace oap::toolpath {

namespace {

constexpr int kInternalParallelFaceThreshold = 64;

template <typename Function>
std::size_t parallelSectionFor(
    std::size_t count,
    const ToolpathGenerationControl* control,
    Function&& function)
{
    ToolpathParallelScope parallelScope(control);
    if (count == 0u)
        return 0u;
    const std::size_t maximumParallelism =
        toolpathParallelismFor(count);
    ToolpathParallelLease lease(
        control, maximumParallelism - 1u,
        toolpathHardwareParallelism() - 1u);
    const std::size_t workerCount = std::min(
        maximumParallelism, lease.parallelism());
    toolpathParallelForWorkers(
        count, workerCount,
        std::forward<Function>(function));
    return workerCount;
}

class SectionCancellationProgressIndicator final
    : public Message_ProgressIndicator {
public:
    explicit SectionCancellationProgressIndicator(
        std::function<bool()> cancellationRequested)
        : m_cancellationRequested(
              std::move(cancellationRequested))
    {
    }

protected:
    Standard_Boolean UserBreak() override
    {
        try {
            return m_cancellationRequested &&
                   m_cancellationRequested();
        } catch (...) {
            return Standard_True;
        }
    }

    void Show(const Message_ProgressScope&,
              const Standard_Boolean) override
    {
    }

private:
    std::function<bool()> m_cancellationRequested;
};

bool hasAtLeastFaces(const TopoDS_Shape& shape, int threshold)
{
    int count = 0;
    for (TopExp_Explorer explorer(shape, TopAbs_FACE);
         explorer.More(); explorer.Next()) {
        if (++count >= threshold)
            return true;
    }
    return false;
}

bool buildSectionShape(
    const TopoDS_Shape& shape,
    const gp_Pln& plane,
    bool approximation,
    const ToolpathGenerationControl* control,
    PocketLayerSlice& audit,
    TopoDS_Shape& sectionShape)
{
    const auto perform = [&](bool runParallel) {
        BRepAlgoAPI_Section section(
            shape, plane, Standard_False);
        section.SetNonDestructive(Standard_True);
        section.SetRunParallel(runParallel);
        section.Approximation(approximation);
        if (control && control->isCancellationRequested) {
            const Handle(Message_ProgressIndicator) progress =
                new SectionCancellationProgressIndicator(
                    control->isCancellationRequested);
            section.Build(progress->Start());
        } else {
            section.Build();
        }
        if (!section.IsDone())
            return false;
        sectionShape = section.Shape();
        return !sectionShape.IsNull();
    };

    // Boolean-only OCCT parallelism requires its actual backend/pool demand,
    // a complete lease and the shared nonblocking internal-operation gate.
    {
        ToolpathOcctParallelLease lease(
            hasAtLeastFaces(shape, kInternalParallelFaceThreshold), control);
        if (lease.enabled()) {
            ++audit.occtInternalParallelCount;
            try {
                if (perform(true))
                    return true;
            } catch (...) {
                if (control && control->cancelled())
                    throw;
            }
            if (control && control->cancelled())
                return false;
            ++audit.occtSerialFallbackCount;
            sectionShape.Nullify();
        }
    }
    return perform(false);
}

bool finiteLoop(const Loop2d& loop)
{
    if (loop.points.size() < 3 || !std::isfinite(loop.signedArea))
        return false;
    for (const gp_Pnt2d& point : loop.points)
        if (!std::isfinite(point.X()) || !std::isfinite(point.Y()))
            return false;
    return true;
}

void appendSliceParallelAudit(PocketLayerSlice& target,
                              const PocketLayerSlice& source)
{
    target.wireParallelBatchCount +=
        source.wireParallelBatchCount;
    target.wireParallelTaskCount +=
        source.wireParallelTaskCount;
    target.wirePeakParallelism = std::max(
        target.wirePeakParallelism,
        source.wirePeakParallelism);
    target.occtInternalParallelCount +=
        source.occtInternalParallelCount;
    target.occtSerialFallbackCount +=
        source.occtSerialFallbackCount;
}

void resetSliceParallelAudit(PocketLayerSlice& slice)
{
    slice.wireParallelBatchCount = 0u;
    slice.wireParallelTaskCount = 0u;
    slice.wirePeakParallelism = 1u;
    slice.occtInternalParallelCount = 0u;
    slice.occtSerialFallbackCount = 0u;
}

bool pointInPolygon(const gp_Pnt2d& point, const std::vector<gp_Pnt2d>& polygon)
{
    bool inside = false;
    const size_t count = polygon.size();
    for (size_t i = 0, j = count - 1; i < count; j = i++) {
        const gp_Pnt2d& a = polygon[i];
        const gp_Pnt2d& b = polygon[j];
        const bool crosses = ((a.Y() > point.Y()) != (b.Y() > point.Y()));
        if (!crosses)
            continue;
        const double x = (b.X() - a.X()) * (point.Y() - a.Y()) /
                             (b.Y() - a.Y()) +
                         a.X();
        if (point.X() < x)
            inside = !inside;
    }
    return inside;
}

// 按嵌套深度统一环绕向：偶深度为实体轮廓（逆时针），奇深度为孔洞/禁入岛（顺时针）。
void normalizeByNesting(std::vector<Loop2d>& loops)
{
    // 按面积降序排序后，对每个环统计被多少个更大的环包含，得到嵌套深度。
    // 偶深度 = 实体轮廓（应为正面积/逆时针），奇深度 = 孔洞或禁入岛
    // （应为负面积/顺时针），据此统一每个环的绕向。
    std::sort(loops.begin(), loops.end(), [](const Loop2d& a,
                                             const Loop2d& b) {
        const double leftArea = std::abs(a.signedArea);
        const double rightArea = std::abs(b.signedArea);
        if (std::abs(leftArea - rightArea) > 1.0e-12)
            return leftArea > rightArea;
        const auto minimumPoint = [](const Loop2d& loop) {
            gp_Pnt2d minimum(
                std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity());
            for (const gp_Pnt2d& point : loop.points) {
                if (point.X() < minimum.X() ||
                    (point.X() == minimum.X() &&
                     point.Y() < minimum.Y())) {
                    minimum = point;
                }
            }
            return minimum;
        };
        const gp_Pnt2d leftMinimum = minimumPoint(a);
        const gp_Pnt2d rightMinimum = minimumPoint(b);
        if (leftMinimum.X() != rightMinimum.X())
            return leftMinimum.X() < rightMinimum.X();
        if (leftMinimum.Y() != rightMinimum.Y())
            return leftMinimum.Y() < rightMinimum.Y();
        return a.points.size() < b.points.size();
    });

    for (size_t i = 0; i < loops.size(); ++i) {
        int nesting = 0;
        const gp_Pnt2d probe = loops[i].points.front();
        for (size_t j = 0; j < i; ++j)
            if (pointInPolygon(probe, loops[j].points))
                ++nesting;

        const bool outer = (nesting % 2) == 0;
        if ((outer && loops[i].signedArea < 0.0) ||
            (!outer && loops[i].signedArea > 0.0))
            OcctAreaAdapter::reverseLoop(loops[i]);
    }
}

double wireLength(const TopoDS_Wire& wire)
{
    GProp_GProps props;
    BRepGProp::LinearProperties(wire, props);
    return props.Mass();
}

double wireEndGap(const TopoDS_Wire& wire)
{
    if (wire.Closed() || BRep_Tool::IsClosed(wire))
        return 0.0;

    TopoDS_Vertex first;
    TopoDS_Vertex last;
    for (BRepTools_WireExplorer explorer(wire); explorer.More(); explorer.Next()) {
        TopoDS_Vertex edgeFirst;
        TopoDS_Vertex edgeLast;
        TopExp::Vertices(explorer.Current(), edgeFirst, edgeLast, Standard_True);
        if (first.IsNull())
            first = edgeFirst;
        last = edgeLast;
    }
    if (first.IsNull() || last.IsNull())
        return std::numeric_limits<double>::infinity();
    return BRep_Tool::Pnt(first).Distance(BRep_Tool::Pnt(last));
}

bool wireIsClosed(const TopoDS_Wire& wire, double tolerance)
{
    return wireEndGap(wire) <= tolerance;
}

bool conservativeLineArtifact(
    const TopoDS_Wire& wire,
    const gp_Ax3& frame,
    double deflection,
    double tolerance,
    double wireLengthValue,
    double endGap,
    Loop2d& artifact)
{
    if (!std::isfinite(wireLengthValue) ||
        !std::isfinite(endGap) ||
        endGap <= tolerance ||
        wireLengthValue >
            endGap * 1.10 +
                std::max(deflection, tolerance) * 2.0) {
        return false;
    }

    const Loop2d openLine =
        OcctAreaAdapter::wireToLoop2d(
            wire, frame, deflection, tolerance);
    if (openLine.points.size() < 2)
        return false;

    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    for (const gp_Pnt2d& point : openLine.points) {
        minX = std::min(minX, point.X());
        minY = std::min(minY, point.Y());
        maxX = std::max(maxX, point.X());
        maxY = std::max(maxY, point.Y());
    }
    if (!std::isfinite(minX) || !std::isfinite(minY) ||
        !std::isfinite(maxX) || !std::isfinite(maxY)) {
        return false;
    }

    // 开放线状交集通常来自圆角/圆柱与采样平面的相切。不能直接丢弃，
    // 否则它落在腔内时会形成漏检碰撞；把它包成一个很薄的正面积材料
    // 障碍，后续刀具半径偏置会按普通凸台一样保持安全距离。
    const double padding =
        std::max(deflection, tolerance * 8.0);
    artifact.points = {
        gp_Pnt2d(minX - padding, minY - padding),
        gp_Pnt2d(maxX + padding, minY - padding),
        gp_Pnt2d(maxX + padding, maxY + padding),
        gp_Pnt2d(minX - padding, maxY + padding)};
    artifact.closed = true;
    artifact.signedArea =
        OcctAreaAdapter::signedArea(artifact.points);
    return std::isfinite(artifact.signedArea) &&
           artifact.signedArea > 0.0;
}

constexpr double kMaximumProtectedMaterialConnectionTolerance = 0.05;
constexpr double kMaximumProtectedMaterialClosureGap = 0.10;
constexpr double kMaximumSolidSectionRepairGap = 10.0;
constexpr double kMinimumSolidSectionRepairLength = 100.0;
constexpr double kMaximumSolidSectionRepairGapRatio = 0.01;

bool repairDominantSolidSectionContour(
    const TopoDS_Wire& wire,
    const gp_Ax3& frame,
    double deflection,
    double tolerance,
    double minArea,
    double wireLengthValue,
    double endGap,
    Loop2d& repaired,
    Loop2d& closureGuard)
{
    if (!std::isfinite(wireLengthValue) ||
        !std::isfinite(endGap) ||
        wireLengthValue < kMinimumSolidSectionRepairLength ||
        endGap <= tolerance ||
        endGap > kMaximumSolidSectionRepairGap ||
        endGap >
            wireLengthValue *
                kMaximumSolidSectionRepairGapRatio) {
        return false;
    }

    repaired = OcctAreaAdapter::wireToLoop2d(
        wire, frame, deflection, tolerance);
    if (!finiteLoop(repaired) ||
        std::abs(repaired.signedArea) <= minArea) {
        return false;
    }
    repaired.closed = true;

    // 有效实体应具有闭合的平面截面。某些大型 STEP 实体会让 OCCT 在
    // 相切/裁剪接缝附近漏掉一小段链,留下一条较长的开口外环。只闭合
    // 间隙相对其周长极小、占主导地位的环,然后保守地保护整个缺链邻域,
    // 使该修复不会产生新的贯通捷径。
    const gp_Pnt2d& first = repaired.points.front();
    const gp_Pnt2d& last = repaired.points.back();
    const double padding = std::max({
        deflection, tolerance * 8.0, endGap});
    const double minX =
        std::min(first.X(), last.X()) - padding;
    const double minY =
        std::min(first.Y(), last.Y()) - padding;
    const double maxX =
        std::max(first.X(), last.X()) + padding;
    const double maxY =
        std::max(first.Y(), last.Y()) + padding;
    closureGuard.points = {
        gp_Pnt2d(minX, minY),
        gp_Pnt2d(maxX, minY),
        gp_Pnt2d(maxX, maxY),
        gp_Pnt2d(minX, maxY)};
    closureGuard.closed = true;
    closureGuard.signedArea =
        OcctAreaAdapter::signedArea(
            closureGuard.points);
    return std::isfinite(closureGuard.signedArea) &&
           closureGuard.signedArea > 0.0;
}

double protectedMaterialSampleStep(double tolerance,
                                   double deflection)
{
    return std::max(
        tolerance * 8.0,
        std::min(
            0.01,
            std::max(deflection, tolerance)));
}

double protectedMaterialAdvance(
    const PocketRegion& region,
    double depthOffset,
    double tolerance,
    double deflection)
{
    const double preferredAdvance =
        protectedMaterialSampleStep(
            tolerance, deflection);
    const double openingRoom =
        std::max(0.0, region.depth - depthOffset);
    double advance = std::min(
        preferredAdvance, openingRoom * 0.25);
    if (advance <= tolerance &&
        depthOffset > preferredAdvance) {
        advance = -preferredAdvance;
    }
    return advance;
}

gp_Pln protectedMaterialPlane(const PocketRegion& region,
                              double depthOffset,
                              double tolerance,
                              double deflection,
                              double planeRetreat = 0.0)
{
    const double advance =
        protectedMaterialAdvance(
            region, depthOffset,
            tolerance, deflection);

    gp_Pln plane = region.floorPlane;
    double planeOffset = depthOffset + advance;
    plane.Translate(
        gp_Vec(region.openDir) *
        (planeOffset - planeRetreat));
    // projectedDepth 来自离散曲线极值，同一物理开口可能得到相差若干
    // 浮点尾数的 depth。OCCT 在圆角相切附近会因这些尾数走入完全不同的
    // 截交分支，因此把平面沿法向的位置稳定到 1e-5 mm 网格。该量级远小于
    // 切片离散公差，不改变加工几何，只消除不可重复的拓扑碎边。
    constexpr double planePositionGrid = 1.0e-5;
    const gp_Vec normal(region.openDir);
    const double planePosition =
        gp_Vec(plane.Location().XYZ()).Dot(normal);
    const double stablePlanePosition =
        std::round(
            planePosition / planePositionGrid) *
        planePositionGrid;
    // 同时把平面原点规范到世界原点在该平面上的法向投影，避免两个等价
    // gp_Pln 仅因切向原点不同而让布尔截交产生不同的边拆分。
    return gp_Pln(
        gp_Pnt(
            region.openDir.X() *
                stablePlanePosition,
            region.openDir.Y() *
                stablePlanePosition,
            region.openDir.Z() *
                stablePlanePosition),
        region.openDir);
}

bool sliceProtectedMaterialAtDepth(const PocketRegion& region,
                                   double depthOffset,
                                   double tolerance,
                                   double deflection,
                                   double minArea,
                                   PocketLayerSlice& result,
                                   std::string& error,
                                   const ToolpathGenerationControl* control,
                                   double planeRetreat = 0.0)
{
    result.protectedMaterialContours.clear();
    result.failureCode =
        PocketLayerSliceFailureCode::None;
    if (region.protectedMaterialShape.IsNull())
        return true;

    try {
        // 刀具从当前刀尖层沿 openDir 指向主轴。材料截面稍向开口侧取样，
        // 既允许刀尖接触最终底面，也能保留从底面升起的凸台。
        const gp_Pln sampledPlane =
            protectedMaterialPlane(
                region, depthOffset,
                tolerance, deflection,
                planeRetreat);
        // 材料保护需要稳定的拓扑闭环。启用曲线近似会让大型 STEP 的同一
        // 截面在重复执行时产生不同数量的碎边，因此这里保留精确交线，
        // 后续再按 sectionDeflection 离散为二维轮廓。
        TopoDS_Shape sectionShape;
        if (!buildSectionShape(
                region.protectedMaterialShape,
                sampledPlane, false, control,
                result, sectionShape)) {
            result.failureCode =
                PocketLayerSliceFailureCode::SectionOperationFailed;
            error = "protected-material section operation did not complete";
            return false;
        }

        std::vector<TopoDS_Edge> sectionEdges;
        for (TopExp_Explorer explorer(
                 sectionShape, TopAbs_EDGE);
             explorer.More(); explorer.Next()) {
            const TopoDS_Edge edge =
                TopoDS::Edge(explorer.Current());
            if (!edge.IsNull())
                sectionEdges.push_back(edge);
        }
        if (sectionEdges.empty()) {
            result.failureCode =
                PocketLayerSliceFailureCode::NoEdges;
            error = "protected-material section produced no edges";
            return false;
        }

        // 实体截面比局部特征面组复杂得多。某些 STEP 在完整外轮廓上会留下
        // 大于离散 deflection 的亚毫米首尾缝隙。边与边的连接仍严格限制在
        // deflection 内，避免相邻细节被误接；仅在最后一次尝试中，允许已经
        // 组装成单条长线框的两个端点在 0.10 mm 内直线封口。
        const double maxConnectionTolerance = std::max(
            tolerance,
            std::min(
                kMaximumProtectedMaterialConnectionTolerance,
                std::max(tolerance, deflection)));
        // 即使 STEP 面本身携带了较大的拓扑公差，也从 OCCT 的基础混淆
        // 公差开始逐级连接。FreeBounds 会在每轮修复端点关系；直接从
        // 1e-3 mm 跳到 1e-2 mm 会绕过中间修复阶段，在复杂外轮廓上反而
        // 得到更多开放支路。
        double connectionTolerance =
            std::min(
                tolerance,
                Precision::Confusion());
        int connectionAttemptCount = 0;
        int lastWireCount = 0;
        for (;;) {
            ++connectionAttemptCount;
            Handle(TopTools_HSequenceOfShape) edges =
                new TopTools_HSequenceOfShape;
            for (const TopoDS_Edge& edge : sectionEdges)
                edges->Append(edge);

            Handle(TopTools_HSequenceOfShape) wires =
                new TopTools_HSequenceOfShape;
            ShapeAnalysis_FreeBounds::ConnectEdgesToWires(
                edges, connectionTolerance,
                Standard_False, wires);
            lastWireCount = wires->Length();

            const bool finalConnectionAttempt =
                connectionTolerance >=
                maxConnectionTolerance -
                    Precision::Confusion();
            const double closureTolerance =
                finalConnectionAttempt
                    ? kMaximumProtectedMaterialClosureGap
                    : connectionTolerance * 4.0;
            const double attemptMinArea = std::max(
                minArea,
                connectionTolerance *
                    connectionTolerance * 4.0);
            std::vector<TopoDS_Wire> wireShapes;
            wireShapes.reserve(
                static_cast<std::size_t>(wires->Length()));
            for (int index = 1; index <= wires->Length(); ++index) {
                wireShapes.push_back(
                    TopoDS::Wire(wires->Value(index)));
            }
            struct ProtectedWireAnalysis {
                bool usable = false;
                bool open = false;
                bool hasArtifact = false;
                bool hasLoop = false;
                double length = 0.0;
                double endGap = 0.0;
                Loop2d artifact;
                Loop2d loop;
            };
            std::vector<ProtectedWireAnalysis> analyses(
                wireShapes.size());
            const std::size_t wireWorkers = parallelSectionFor(
                wireShapes.size(), control,
                [&](std::size_t wireIndex) {
                    const TopoDS_Wire& wire =
                        wireShapes[wireIndex];
                    ProtectedWireAnalysis& analysis =
                        analyses[wireIndex];
                    if (wire.IsNull())
                        return;
                    analysis.length = wireLength(wire);
                    if (!std::isfinite(analysis.length) ||
                        analysis.length <=
                            connectionTolerance * 10.0) {
                        return;
                    }
                    analysis.usable = true;
                    analysis.endGap = wireEndGap(wire);
                    analysis.open =
                        analysis.endGap > closureTolerance;
                    if (analysis.open) {
                        analysis.hasArtifact =
                            conservativeLineArtifact(
                                wire, region.frame,
                                deflection,
                                connectionTolerance,
                                analysis.length,
                                analysis.endGap,
                                analysis.artifact);
                        return;
                    }
                    analysis.loop =
                        OcctAreaAdapter::wireToLoop2d(
                            wire, region.frame,
                            deflection,
                            connectionTolerance);
                    if (!finiteLoop(analysis.loop) ||
                        std::abs(analysis.loop.signedArea) <=
                            attemptMinArea) {
                        return;
                    }
                    analysis.loop.closed = true;
                    analysis.hasLoop = true;
                });
            if (wireWorkers > 1u) {
                ++result.wireParallelBatchCount;
                result.wireParallelTaskCount +=
                    wireShapes.size();
                result.wirePeakParallelism = std::max(
                    result.wirePeakParallelism,
                    wireWorkers);
            }

            std::vector<Loop2d> contours;
            std::vector<Loop2d>
                conservativeOpenArtifacts;
            bool hasSignificantOpenWire = false;
            bool repairedSolidSectionContour = false;
            int significantOpenWireCount = 0;
            double smallestOpenGap =
                std::numeric_limits<double>::infinity();
            double largestOpenGap = 0.0;
            for (std::size_t wireIndex = 0u;
                 wireIndex < analyses.size(); ++wireIndex) {
                ProtectedWireAnalysis& analysis =
                    analyses[wireIndex];
                if (!analysis.usable)
                    continue;
                if (analysis.open) {
                    if (analysis.hasArtifact) {
                        conservativeOpenArtifacts.push_back(
                            std::move(analysis.artifact));
                        continue;
                    }
                    Loop2d repaired;
                    Loop2d closureGuard;
                    if (finalConnectionAttempt &&
                        !repairedSolidSectionContour &&
                        region.protectedMaterialShape
                                .ShapeType() ==
                            TopAbs_SOLID &&
                        repairDominantSolidSectionContour(
                            wireShapes[wireIndex], region.frame,
                            deflection,
                            connectionTolerance,
                            attemptMinArea,
                            analysis.length,
                            analysis.endGap,
                            repaired,
                            closureGuard)) {
                        contours.push_back(
                            std::move(repaired));
                        conservativeOpenArtifacts
                            .push_back(
                                std::move(closureGuard));
                        repairedSolidSectionContour =
                            true;
                        continue;
                    }
                    hasSignificantOpenWire = true;
                    ++significantOpenWireCount;
                    smallestOpenGap =
                        std::min(
                            smallestOpenGap,
                            analysis.endGap);
                    largestOpenGap =
                        std::max(
                            largestOpenGap,
                            analysis.endGap);
                    continue;
                }
                if (analysis.hasLoop)
                    contours.push_back(
                        std::move(analysis.loop));
            }

            if (!hasSignificantOpenWire &&
                !contours.empty()) {
                normalizeByNesting(contours);
                contours.insert(
                    contours.end(),
                    std::make_move_iterator(
                        conservativeOpenArtifacts.begin()),
                    std::make_move_iterator(
                        conservativeOpenArtifacts.end()));
                result.protectedMaterialContours =
                    std::move(contours);
                return true;
            }
            if (connectionTolerance >=
                maxConnectionTolerance -
                    Precision::Confusion()) {
                result.failureCode =
                    hasSignificantOpenWire
                        ? PocketLayerSliceFailureCode::OpenContour
                        : PocketLayerSliceFailureCode::NoValidContour;
                std::ostringstream diagnostic;
                diagnostic
                    << (hasSignificantOpenWire
                            ? "protected-material section has an open contour"
                            : "protected-material section has no valid closed contours")
                    << " (edges=" << sectionEdges.size()
                    << ", wires=" << lastWireCount
                    << ", attempts=" << connectionAttemptCount
                    << ", tolerance=" << connectionTolerance
                    << " mm, maximum closure gap="
                    << kMaximumProtectedMaterialClosureGap
                    << " mm";
                if (hasSignificantOpenWire) {
                    diagnostic
                        << ", open wires="
                        << significantOpenWireCount
                        << ", endpoint gaps="
                        << smallestOpenGap << ".."
                        << largestOpenGap << " mm";
                }
                diagnostic << ')';
                const std::string primaryError =
                    diagnostic.str();
                if (hasSignificantOpenWire &&
                    planeRetreat <= tolerance) {
                    const double retryStep =
                        protectedMaterialSampleStep(
                            tolerance, deflection);
                    const double advance =
                        protectedMaterialAdvance(
                            region, depthOffset,
                            tolerance, deflection);
                    const double maximumSafeRetreat =
                        std::max(
                            0.0,
                            depthOffset + advance);
                    std::string retryDiagnostics;
                    for (const double retryRetreat :
                         {retryStep,
                          retryStep * 4.0}) {
                        if (retryRetreat >
                            maximumSafeRetreat +
                                tolerance) {
                            continue;
                        }
                        std::string retryError;
                        if (sliceProtectedMaterialAtDepth(
                                region, depthOffset,
                                tolerance, deflection,
                                minArea, result,
                                retryError, control,
                                retryRetreat)) {
                            return true;
                        }
                        if (!retryDiagnostics.empty()) {
                            retryDiagnostics += "; ";
                        }
                        retryDiagnostics +=
                            "retreat " +
                            std::to_string(
                                retryRetreat) +
                            " mm: " +
                            retryError;
                    }
                    error =
                        primaryError +
                        "; protected-material nearby-plane retries failed: " +
                        retryDiagnostics;
                    return false;
                }
                error = primaryError;
                return false;
            }
            connectionTolerance = std::min(
                maxConnectionTolerance,
                connectionTolerance * 10.0);
        }
    } catch (const Standard_Failure& failure) {
        result.failureCode =
            PocketLayerSliceFailureCode::SectionOperationFailed;
        error = "protected-material section failed: ";
        error += failure.GetMessageString();
    } catch (const std::exception& failure) {
        result.failureCode =
            PocketLayerSliceFailureCode::SectionOperationFailed;
        error = "protected-material section failed: ";
        error += failure.what();
    }
    return false;
}

std::int64_t quantized(double value, double step)
{
    if (!std::isfinite(value))
        return 0;
    return static_cast<std::int64_t>(
        std::llround(value / step));
}

enum class SectionCacheKind : std::uint8_t {
    ProtectedMaterial,
    Region,
};

struct SectionCacheKey {
    SectionCacheKind kind = SectionCacheKind::ProtectedMaterial;
    TopoDS_Shape sectionShape;
    TopoDS_Shape protectedMaterialShape;
    std::array<std::int64_t, 17> geometry{};
    std::vector<std::int64_t> boundaryGeometry;

    bool operator==(const SectionCacheKey& other) const noexcept
    {
        const auto sameShape = [](const TopoDS_Shape& left,
                                  const TopoDS_Shape& right) {
            return left.IsNull()
                ? right.IsNull()
                : !right.IsNull() && left.IsSame(right);
        };
        return kind == other.kind &&
               sameShape(sectionShape, other.sectionShape) &&
               sameShape(protectedMaterialShape,
                         other.protectedMaterialShape) &&
               geometry == other.geometry &&
               boundaryGeometry == other.boundaryGeometry;
    }
};

void mixHash(std::size_t& hash, std::size_t part) noexcept
{
    hash ^= part + 0x9e3779b97f4a7c15ULL +
            (hash << 6) + (hash >> 2);
}

std::size_t shapeHash(const TopoDS_Shape& shape) noexcept
{
    return shape.IsNull()
        ? 0u
        : TopTools_ShapeMapHasher{}(shape);
}

void appendLoopGeometry(std::vector<std::int64_t>& geometry,
                        const Loop2d& loop,
                        double coordinateGrid)
{
    geometry.push_back(static_cast<std::int64_t>(loop.points.size()));
    geometry.push_back(loop.closed ? 1 : 0);
    geometry.push_back(loop.virtualOpenBoundary ? 1 : 0);
    for (const gp_Pnt2d& point : loop.points) {
        geometry.push_back(quantized(point.X(), coordinateGrid));
        geometry.push_back(quantized(point.Y(), coordinateGrid));
    }
}

struct SectionCacheKeyHash {
    std::size_t operator()(const SectionCacheKey& key) const noexcept
    {
        std::size_t hash = static_cast<std::size_t>(key.kind);
        mixHash(hash, shapeHash(key.sectionShape));
        mixHash(hash, shapeHash(key.protectedMaterialShape));
        for (const std::int64_t value : key.geometry) {
            mixHash(hash, std::hash<std::int64_t>{}(value));
        }
        for (const std::int64_t value : key.boundaryGeometry)
            mixHash(hash, std::hash<std::int64_t>{}(value));
        return hash;
    }
};

SectionCacheKey materialSectionCacheKey(
    const PocketRegion& region,
    double depthOffset)
{
    const double tolerance = std::max(
        region.sectionTolerance, Precision::Confusion());
    const double deflection = std::max(
        region.sectionDeflection, tolerance);
    const gp_Pln plane = protectedMaterialPlane(
        region, depthOffset, tolerance, deflection);
    const gp_Dir normal = plane.Axis().Direction();
    const gp_Dir xDirection = region.frame.XDirection();
    const gp_Dir yDirection = region.frame.YDirection();
    const gp_Pnt& frameOrigin = region.frame.Location();
    const double coordinateGrid =
        std::max(1.0e-7, tolerance);
    constexpr double directionGrid = 1.0e-10;

    SectionCacheKey key;
    key.kind = SectionCacheKind::ProtectedMaterial;
    key.protectedMaterialShape = region.protectedMaterialShape;
    key.geometry = {
        quantized(normal.X(), directionGrid),
        quantized(normal.Y(), directionGrid),
        quantized(normal.Z(), directionGrid),
        quantized(
            gp_Vec(plane.Location().XYZ()).Dot(gp_Vec(normal)),
            coordinateGrid),
        quantized(xDirection.X(), directionGrid),
        quantized(xDirection.Y(), directionGrid),
        quantized(xDirection.Z(), directionGrid),
        quantized(yDirection.X(), directionGrid),
        quantized(yDirection.Y(), directionGrid),
        quantized(yDirection.Z(), directionGrid),
        quantized(
            gp_Vec(frameOrigin.XYZ()).Dot(gp_Vec(xDirection)),
            coordinateGrid),
        quantized(
            gp_Vec(frameOrigin.XYZ()).Dot(gp_Vec(yDirection)),
            coordinateGrid),
        quantized(tolerance, 1.0e-12),
        quantized(deflection, 1.0e-9),
        0,
        0,
        0,
    };
    return key;
}

SectionCacheKey regionSectionCacheKey(
    const PocketRegion& region,
    double depthOffset)
{
    const double tolerance = std::max(
        region.sectionTolerance, Precision::Confusion());
    const double deflection = std::max(
        region.sectionDeflection, Precision::Confusion());
    gp_Pln plane = region.floorPlane;
    plane.Translate(gp_Vec(region.openDir) * depthOffset);
    const gp_Dir normal = plane.Axis().Direction();
    const gp_Dir xDirection = region.frame.XDirection();
    const gp_Dir yDirection = region.frame.YDirection();
    const gp_Pnt& frameOrigin = region.frame.Location();
    const double coordinateGrid = std::max(1.0e-7, tolerance);
    constexpr double directionGrid = 1.0e-10;

    SectionCacheKey key;
    key.kind = SectionCacheKind::Region;
    key.sectionShape = region.sectionShape;
    key.protectedMaterialShape = region.protectedMaterialShape;
    key.geometry = {
        quantized(normal.X(), directionGrid),
        quantized(normal.Y(), directionGrid),
        quantized(normal.Z(), directionGrid),
        quantized(
            gp_Vec(plane.Location().XYZ()).Dot(gp_Vec(normal)),
            coordinateGrid),
        quantized(xDirection.X(), directionGrid),
        quantized(xDirection.Y(), directionGrid),
        quantized(xDirection.Z(), directionGrid),
        quantized(yDirection.X(), directionGrid),
        quantized(yDirection.Y(), directionGrid),
        quantized(yDirection.Z(), directionGrid),
        quantized(
            gp_Vec(frameOrigin.XYZ()).Dot(gp_Vec(xDirection)),
            coordinateGrid),
        quantized(
            gp_Vec(frameOrigin.XYZ()).Dot(gp_Vec(yDirection)),
            coordinateGrid),
        quantized(tolerance, 1.0e-12),
        quantized(deflection, 1.0e-9),
        quantized(region.depth, coordinateGrid),
        region.allowOpenSectionContours ? 1 : 0,
        region.useSectionBoundaryAtEveryDepth ? 1 : 0,
    };
    key.boundaryGeometry.reserve(
        3u + 2u * region.boundary.points.size());
    appendLoopGeometry(
        key.boundaryGeometry, region.boundary, coordinateGrid);
    key.boundaryGeometry.push_back(
        static_cast<std::int64_t>(region.islands.size()));
    for (const Loop2d& island : region.islands) {
        appendLoopGeometry(
            key.boundaryGeometry, island, coordinateGrid);
    }
    return key;
}

} // namespace

struct PocketMaterialSectionCache::Impl {
    struct Entry {
        PocketLayerSlice result;
        std::string error;
        bool ok = false;
        bool ready = false;
        std::condition_variable readyCondition;
    };

    mutable std::mutex mutex;
    std::unordered_map<
        SectionCacheKey,
        std::shared_ptr<Entry>,
        SectionCacheKeyHash>
        entries;
    std::size_t hits = 0;
    std::size_t misses = 0;

    template <typename Compute>
    bool getOrCompute(const SectionCacheKey& key,
                      PocketLayerSlice& result,
                      std::string& error,
                      Compute&& compute)
    {
        std::shared_ptr<Entry> entry;
        {
            std::unique_lock<std::mutex> lock(mutex);
            const auto found = entries.find(key);
            if (found != entries.end()) {
                ++hits;
                entry = found->second;
                entry->readyCondition.wait(
                    lock, [&entry]() { return entry->ready; });
                result = entry->result;
                resetSliceParallelAudit(result);
                error = entry->error;
                return entry->ok;
            }
            ++misses;
            entry = std::make_shared<Entry>();
            entries.emplace(key, entry);
        }

        PocketLayerSlice computed;
        std::string computedError;
        bool ok = false;
        try {
            ok = compute(computed, computedError);
        } catch (const Standard_Failure& failure) {
            computed.failureCode =
                PocketLayerSliceFailureCode::SectionOperationFailed;
            const char* message = failure.GetMessageString();
            computedError = message
                ? std::string("cached section failed: ") + message
                : "cached section failed";
        } catch (const std::exception& failure) {
            computed.failureCode =
                PocketLayerSliceFailureCode::SectionOperationFailed;
            computedError =
                std::string("cached section failed: ") + failure.what();
        } catch (...) {
            computed.failureCode =
                PocketLayerSliceFailureCode::SectionOperationFailed;
            computedError = "cached section failed with an unknown exception";
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            entry->result = std::move(computed);
            entry->error = std::move(computedError);
            entry->ok = ok;
            entry->ready = true;
            result = entry->result;
            error = entry->error;
        }
        entry->readyCondition.notify_all();
        return ok;
    }
};

PocketMaterialSectionCache::PocketMaterialSectionCache()
    : m_impl(std::make_unique<Impl>())
{
}

PocketMaterialSectionCache::~PocketMaterialSectionCache() =
    default;

bool PocketMaterialSectionCache::slice(
    const PocketRegion& region,
    double depthOffset,
    PocketLayerSlice& result,
    std::string& error,
    const ToolpathGenerationControl* control)
{
    // 查询/更新缓存：命中直接返回上次结果，未命中则调用切片器计算后
    // 写入；并发安全由 m_impl->mutex 保证，计算部分在锁外执行。
    if (region.protectedMaterialShape.IsNull()) {
        result = {};
        error.clear();
        return true;
    }

    return m_impl->getOrCompute(
        materialSectionCacheKey(region, depthOffset),
        result, error,
        [&region, depthOffset, control](PocketLayerSlice& computed,
                                        std::string& computedError) {
            return PocketLayerSlicer().sliceProtectedMaterial(
                region, depthOffset, computed, computedError,
                control);
        });
}

bool PocketMaterialSectionCache::sliceRegion(
    const PocketRegion& region,
    double depthOffset,
    PocketLayerSlice& result,
    std::string& error,
    const ToolpathGenerationControl* control)
{
    if (!std::isfinite(depthOffset) || !std::isfinite(region.depth)) {
        return PocketLayerSlicer().slice(
            region, depthOffset, result, error,
            nullptr, control);
    }
    return m_impl->getOrCompute(
        regionSectionCacheKey(region, depthOffset),
        result, error,
        [this, &region, depthOffset, control](
            PocketLayerSlice& computed,
            std::string& computedError) {
            return PocketLayerSlicer().slice(
                region, depthOffset, computed, computedError,
                this, control);
        });
}

std::size_t PocketMaterialSectionCache::hitCount() const
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->hits;
}

std::size_t PocketMaterialSectionCache::missCount() const
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->misses;
}

std::size_t PocketMaterialSectionCache::entryCount() const
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->entries.size();
}

bool PocketLayerSlicer::sliceProtectedMaterial(
    const PocketRegion& region,
    double depthOffset,
    PocketLayerSlice& result,
    std::string& error,
    const ToolpathGenerationControl* control) const
{
    ToolpathParallelScope parallelScope(control);
    result = {};
    error.clear();
    if (!std::isfinite(depthOffset) || depthOffset < 0.0 ||
        depthOffset > region.depth +
            std::max(region.sectionTolerance,
                     Precision::Confusion())) {
        result.failureCode =
            PocketLayerSliceFailureCode::InvalidDepth;
        error = "protected-material depth is outside the pocket range";
        return false;
    }

    const double tolerance = std::max(
        region.sectionTolerance, Precision::Confusion());
    const double deflection = std::max(
        region.sectionDeflection, tolerance);
    const double minArea = std::max(
        tolerance * tolerance * 16.0,
        deflection * deflection * 0.01);
    return sliceProtectedMaterialAtDepth(
        region, depthOffset, tolerance,
        deflection, minArea, result, error,
        control);
}

bool PocketLayerSlicer::slice(const PocketRegion& region,
                              double depthOffset,
                              PocketLayerSlice& result,
                              std::string& error,
                              PocketMaterialSectionCache*
                                  materialCache,
                              const ToolpathGenerationControl*
                                  control) const
{
    ToolpathParallelScope parallelScope(control);
    result = PocketLayerSlice{};
    error.clear();

    const double tolerance =
        std::max(region.sectionTolerance, Precision::Confusion());
    const double deflection =
        std::max(region.sectionDeflection, Precision::Confusion());
    const double minArea = std::max(1.0e-10, tolerance * tolerance * 100.0);

    if (!std::isfinite(depthOffset) || !std::isfinite(region.depth) ||
        depthOffset < -tolerance || depthOffset > region.depth + tolerance) {
        result.failureCode = PocketLayerSliceFailureCode::InvalidDepth;
        error = "depth offset is outside the recognized pocket range";
        return false;
    }

    // 普通槽腔的已识别边界位于 offset=0 的底层；主外表面内环区域
    // 的已识别根内环则位于 offset=depth 的开口层。只在各自真实边界
    // 所在层复用线框，其他深度均截取结构面。
    const double recognizedBoundaryOffset =
        region.useSectionBoundaryAtEveryDepth
            ? region.depth
            : 0.0;
    if (std::abs(
            depthOffset - recognizedBoundaryOffset) <=
        tolerance) {
        if (region.boundary.points.size() < 3) {
            result.failureCode = PocketLayerSliceFailureCode::NoValidContour;
            error = "floor boundary is not usable";
            return false;
        }
        result.contours.push_back(region.boundary);
        result.contours.insert(result.contours.end(),
                               region.islands.begin(), region.islands.end());
        normalizeByNesting(result.contours);
        result.connectionTolerance = tolerance;
        if (materialCache) {
            PocketLayerSlice materialSlice;
            if (!materialCache->slice(
                    region, depthOffset,
                    materialSlice, error,
                    control)) {
                appendSliceParallelAudit(
                    result, materialSlice);
                result.failureCode =
                    materialSlice.failureCode;
                return false;
            }
            result.protectedMaterialContours =
                std::move(
                    materialSlice
                        .protectedMaterialContours);
            appendSliceParallelAudit(result, materialSlice);
            return true;
        }
        return sliceProtectedMaterialAtDepth(
            region, depthOffset, tolerance,
            deflection, minArea, result, error,
            control);
    }

    if (region.sectionShape.IsNull()) {
        result.failureCode = PocketLayerSliceFailureCode::MissingSectionShape;
        error = "recognized pocket has no section face group";
        return false;
    }

    try {
        // depthOffset 沿开口方向从底面向上量取，和深度层规划器保持同一约定。
        gp_Pln layerPlane = region.floorPlane;
        layerPlane.Translate(gp_Vec(region.openDir) * depthOffset);

        TopoDS_Shape sectionShape;
        if (!buildSectionShape(
                region.sectionShape, layerPlane, true,
                control, result, sectionShape)) {
            result.failureCode = PocketLayerSliceFailureCode::SectionOperationFailed;
            error = "OCCT section operation did not complete";
            return false;
        }

        std::vector<TopoDS_Edge> sectionEdges;
        for (TopExp_Explorer ex(sectionShape, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
            if (!edge.IsNull())
                sectionEdges.push_back(edge);
        }
        result.sectionEdgeCount = static_cast<int>(sectionEdges.size());
        if (sectionEdges.empty()) {
            result.failureCode = PocketLayerSliceFailureCode::NoEdges;
            error = "section produced no edges";
            return false;
        }

        // STEP 截面端点缝隙常大于拓扑 tolerance，但通常仍落在离散公差内。
        // 从拓扑公差逐级放大到 deflection，绝不上探超过 0.05 mm。
        const double maxConnectionTolerance = std::max(
            tolerance, std::min(0.05, std::max(tolerance, deflection)));
        double connectionTolerance = tolerance;
        bool sawWires = false;
        bool sawOpenContour = false;
        int lastWireCount = 0;
        const bool hasSolidMaterialGuard =
            !region.protectedMaterialShape.IsNull() &&
            region.protectedMaterialShape.ShapeType() ==
                TopAbs_SOLID;

        for (;;) {
            ++result.connectionAttemptCount;
            Handle(TopTools_HSequenceOfShape) edges = new TopTools_HSequenceOfShape;
            for (const TopoDS_Edge& edge : sectionEdges)
                edges->Append(edge);

            Handle(TopTools_HSequenceOfShape) wires = new TopTools_HSequenceOfShape;
            ShapeAnalysis_FreeBounds::ConnectEdgesToWires(
                edges, connectionTolerance, Standard_False, wires);
            lastWireCount = wires->Length();
            sawWires = sawWires || lastWireCount > 0;

            // 先把 OCCT 序列复制成稳定索引槽，再并行执行每条 wire 的长度、
            // 闭合性和二维离散。ConnectEdgesToWires 与最终嵌套归一化仍保持
            // 串行，因此不会改变公差重试和轮廓选择顺序。
            std::vector<TopoDS_Wire> wireShapes;
            wireShapes.reserve(
                static_cast<std::size_t>(wires->Length()));
            for (int index = 1; index <= wires->Length(); ++index) {
                wireShapes.push_back(
                    TopoDS::Wire(wires->Value(index)));
            }
            struct WireAnalysis {
                bool open = false;
                bool significantOpen = false;
                bool valid = false;
                Loop2d loop;
            };
            std::vector<WireAnalysis> analyses(
                wireShapes.size());
            const double attemptMinArea = std::max(
                minArea,
                connectionTolerance * connectionTolerance * 4.0);
            const std::size_t wireWorkers = parallelSectionFor(
                wireShapes.size(), control,
                [&](std::size_t wireIndex) {
                    const TopoDS_Wire& wire =
                        wireShapes[wireIndex];
                    WireAnalysis& analysis =
                        analyses[wireIndex];
                    if (wire.IsNull())
                        return;
                    const double length = wireLength(wire);
                    if (!std::isfinite(length) ||
                        length <= connectionTolerance * 10.0) {
                        return;
                    }
                    analysis.open = !wireIsClosed(
                        wire, connectionTolerance * 4.0);
                    if (analysis.open &&
                        !region.allowOpenSectionContours &&
                        !hasSolidMaterialGuard) {
                        analysis.significantOpen = true;
                        return;
                    }
                    analysis.loop =
                        OcctAreaAdapter::wireToLoop2d(
                            wire, region.frame, deflection,
                            connectionTolerance);
                    if (!finiteLoop(analysis.loop) ||
                        std::abs(analysis.loop.signedArea) <=
                            attemptMinArea) {
                        return;
                    }
                    analysis.loop.closed = true;
                    analysis.loop.virtualOpenBoundary =
                        analysis.open;
                    analysis.valid = true;
                });
            if (wireWorkers > 1u) {
                ++result.wireParallelBatchCount;
                result.wireParallelTaskCount +=
                    wireShapes.size();
                result.wirePeakParallelism = std::max(
                    result.wirePeakParallelism,
                    wireWorkers);
            }

            std::vector<Loop2d> contours;
            bool hasSignificantOpenWire = false;
            int virtualOpenBoundaryCount = 0;
            for (WireAnalysis& analysis : analyses) {
                sawOpenContour = sawOpenContour || analysis.open;
                hasSignificantOpenWire =
                    hasSignificantOpenWire ||
                    analysis.significantOpen;
                if (!analysis.valid)
                    continue;
                if (analysis.open)
                    ++virtualOpenBoundaryCount;
                contours.push_back(std::move(analysis.loop));
            }

            sawOpenContour = sawOpenContour || hasSignificantOpenWire;
            if (!hasSignificantOpenWire && !contours.empty()) {
                result.contours = std::move(contours);
                result.connectionTolerance = connectionTolerance;
                result.wireCount = lastWireCount;
                normalizeByNesting(result.contours);
                result.virtualOpenBoundaryCount =
                    virtualOpenBoundaryCount;
                if (materialCache) {
                    PocketLayerSlice materialSlice;
                    if (!materialCache->slice(
                            region, depthOffset,
                            materialSlice, error,
                            control)) {
                        appendSliceParallelAudit(
                            result, materialSlice);
                        result.failureCode =
                            materialSlice.failureCode;
                        return false;
                    }
                    result.protectedMaterialContours =
                        std::move(
                            materialSlice
                                .protectedMaterialContours);
                    appendSliceParallelAudit(
                        result, materialSlice);
                } else if (
                    !sliceProtectedMaterialAtDepth(
                        region, depthOffset, tolerance,
                        deflection, minArea,
                        result, error, control)) {
                    return false;
                }
                return true;
            }

            if (connectionTolerance >= maxConnectionTolerance - Precision::Confusion())
                break;
            connectionTolerance = std::min(maxConnectionTolerance,
                                           connectionTolerance * 10.0);
        }

        result.connectionTolerance = connectionTolerance;
        result.wireCount = lastWireCount;
        std::ostringstream diagnostic;
        if (sawOpenContour) {
            result.failureCode = PocketLayerSliceFailureCode::OpenContour;
            diagnostic << "section contour cannot be closed after tolerance retries"
                       << " (edges=" << result.sectionEdgeCount
                       << ", wires=" << result.wireCount
                       << ", attempts=" << result.connectionAttemptCount
                       << ", tolerance=" << connectionTolerance << " mm)";
        } else if (!sawWires) {
            result.failureCode = PocketLayerSliceFailureCode::NoWires;
            diagnostic << "section edges could not be connected into wires"
                       << " (edges=" << result.sectionEdgeCount
                       << ", attempts=" << result.connectionAttemptCount
                       << ", tolerance=" << connectionTolerance << " mm)";
        } else {
            result.failureCode = PocketLayerSliceFailureCode::NoValidContour;
            diagnostic << "section produced no valid closed contours"
                       << " (edges=" << result.sectionEdgeCount
                       << ", wires=" << result.wireCount
                       << ", attempts=" << result.connectionAttemptCount
                       << ", tolerance=" << connectionTolerance << " mm)";
        }
        error = diagnostic.str();
        return false;
    } catch (const Standard_Failure& failure) {
        result.failureCode = PocketLayerSliceFailureCode::SectionOperationFailed;
        error = "OCCT section failed: ";
        error += failure.GetMessageString();
    } catch (const std::exception& failure) {
        result.failureCode = PocketLayerSliceFailureCode::SectionOperationFailed;
        error = "section failed: ";
        error += failure.what();
    } catch (...) {
        result.failureCode = PocketLayerSliceFailureCode::SectionOperationFailed;
        error = "section failed with an unknown exception";
    }
    return false;
}

} // namespace oap::toolpath
