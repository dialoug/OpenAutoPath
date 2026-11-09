// Analytic fixed-axis material build orchestration; geometry and sweep stages compile independently.
#include "toolpath/pocket/simulation/fixedaxis/AnalyticFixedAxisLayeredBRepBuilder.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include "toolpath/pocket/simulation/fixedaxis/AnalyticPlanarBoolean.h"
#include "toolpath/pocket/simulation/fixedaxis/AnalyticPlanarSweep.h"
#include "toolpath/core/validation/ToolpathPrimitiveValidator.h"

#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisTypes.h"
#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisGeometry.h"
#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisSweeps.h"
#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisBooleans.h"

namespace oap::toolpath {
using namespace analytic_fixed_axis_detail;

AnalyticFixedAxisLayeredBRepBuildResult
AnalyticFixedAxisLayeredBRepBuilder::build(
    const TopoDS_Shape& stock,
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    AnalyticFixedAxisLayeredBRepBuildResult result;
    const Clock::time_point buildStart = Clock::now();
    const auto finish = [&] {
        result.buildMilliseconds =
            elapsedMilliseconds(buildStart);
        return result;
    };
    if (operation.clearing.primitives.empty()) {
        result.reason =
            "operation has no semantic primitive stream";
        return finish();
    }
    result.attempted = true;
    const double tolerance = std::max(
        options.geometricTolerance,
        Precision::Confusion());
    if (stock.IsNull() ||
        !std::isfinite(params.tool.diameter) ||
        params.tool.diameter <= tolerance ||
        !std::isfinite(params.tool.cornerRadius) ||
        params.tool.cornerRadius < -tolerance ||
        !std::isfinite(params.tool.cuttingEdgeLength) ||
        params.tool.cuttingEdgeLength <= tolerance) {
        result.reason =
            "analytic layered BRep received invalid stock or tool geometry";
        return finish();
    }
    const double cutterRadius = 0.5 * params.tool.diameter;
    const double cornerRadius =
        std::max(0.0, params.tool.cornerRadius);
    const bool flatEnd = cornerRadius <= tolerance;
    const bool ballEnd =
        std::abs(cornerRadius - cutterRadius) <= tolerance;
    if (!flatEnd && !ballEnd) {
        result.reason =
            "analytic layered BRep currently supports flat-end and full ball-end tools only";
        return finish();
    }

    ToolpathValidationReport primitiveReport;
    validateToolpathPrimitives(
        operation.clearing.primitives,
        operation.operationId,
        primitiveReport);
    validateToolpathPrimitiveExpansion(
        operation.clearing.primitives,
        operation.clearing.moves,
        operation.operationId,
        primitiveReport, true);
    if (!primitiveReport.ok()) {
        result.reason =
            "semantic primitive stream does not exactly expand to moves";
        return finish();
    }

    FixedAxisFrame frame;
    if (!makeFrame(
            operation.outwardToolAxis,
            tolerance, frame)) {
        result.reason =
            "analytic layered BRep requires a valid fixed axis";
        return finish();
    }
    double stockMinimum = 0.0;
    double stockMaximum = 0.0;
    if (!stockAxialBounds(
            stock, frame,
            stockMinimum, stockMaximum)) {
        result.reason =
            "analytic layered BRep could not determine stock bounds";
        return finish();
    }
    result.eligible = true;

    for (const ToolpathMove& move : operation.clearing.moves) {
        if (move.type == ToolpathMoveType::Cut ||
            move.type == ToolpathMoveType::Plunge) {
            ++result.cuttingMoveCount;
        }
    }
    if (result.cuttingMoveCount > options.maxSweepSegments) {
        result.eligible = false;
        result.reason =
            "analytic layered BRep exceeds the sweep segment budget";
        return finish();
    }

    const Clock::time_point pathStart = Clock::now();
    const double cuttingLength = params.tool.cuttingEdgeLength;
    const double requestedRampAxialTolerance =
        std::isfinite(options.rampAxialTolerance) &&
                options.rampAxialTolerance > tolerance
            ? options.rampAxialTolerance
            : std::max(
                  0.05,
                  std::isfinite(options.arcChordTolerance)
                      ? options.arcChordTolerance
                      : 0.0);
    const double rampAxialStep = std::min(
        requestedRampAxialTolerance,
        0.5 * cuttingLength);
    const int maximumRampSlices =
        std::max(1, options.maxRampSweepSlices);
    const AnalyticPlanarSweep planarSweep;
    const gp_Ax3 referencePlane = frame.referencePlane();
    std::vector<AnalyticSweepRecord> records;
    records.reserve(operation.clearing.primitives.size());
    std::vector<AnalyticLinearRampGroup> rampGroups;
    std::vector<AnalyticHelicalRampGroup> helicalRampGroups;
    std::vector<AnalyticPlanarCut> planarCuts;
    std::vector<AnalyticPlanarCoverage> planarCoverage;
    std::unordered_set<int> retainedLoopSequences;
    std::vector<AnalyticSweepRecord> fillRecords;
    std::string closedOffsetFillRejection;
    const double closedOffsetNestingTolerance = std::max(
        tolerance,
        std::isfinite(options.arcChordTolerance)
            ? 3.0 * options.arcChordTolerance
            : tolerance);
    const bool useClosedOffsetFill =
        buildClosedOffsetFillRecords(
            operation, params, frame, cuttingLength,
            stockMinimum, stockMaximum, tolerance,
            closedOffsetNestingTolerance,
            retainedLoopSequences, fillRecords,
            result.maximumBoundaryFitError,
            &closedOffsetFillRejection);
    result.closedOffsetFillUsed = useClosedOffsetFill;
    result.closedOffsetFillRecordCount =
        static_cast<int>(fillRecords.size());
    if (ballEnd && !useClosedOffsetFill) {
        result.eligible = false;
        result.reason =
            "analytic ball-end BRep requires an eligible closed offset fill";
        if (!closedOffsetFillRejection.empty()) {
            result.reason += ": " +
                closedOffsetFillRejection;
        }
        return finish();
    }
    if (useClosedOffsetFill) {
        records = std::move(fillRecords);
        records.reserve(
            records.size() +
            operation.clearing.primitives.size());
    }
    for (const ToolpathPrimitive& primitive :
         operation.clearing.primitives) {
        if (cancelled(cancellationRequested)) {
            result.cancelled = true;
            return finish();
        }
        if (useClosedOffsetFill)
            continue;

        ToolpathMoveType moveType = ToolpathMoveType::Rapid;
        gp_Pnt from;
        gp_Pnt to;
        if (primitive.type == ToolpathPrimitiveType::Linear) {
            moveType = primitive.linear.type;
            from = primitive.linear.from;
            to = primitive.linear.to;
        } else if (primitive.type ==
                   ToolpathPrimitiveType::CircularArc) {
            moveType = primitive.arc.moveType;
            from = primitive.arc.from;
            to = primitive.arc.to;
        } else {
            result.eligible = false;
            result.reason =
                "analytic layered BRep does not accept drill-cycle primitives";
            return finish();
        }
        if (moveType != ToolpathMoveType::Cut &&
            moveType != ToolpathMoveType::Plunge) {
            continue;
        }
        if (useClosedOffsetFill &&
            !retainedLoopSequences.contains(
                primitive.context.loopSequence)) {
            continue;
        }

        const double fromW = frame.axial(from);
        const double toW = frame.axial(to);
        const double axialDistance = std::abs(toW - fromW);
        const double planarDistance =
            frame.planarDistance(from, to);
        double tipLow = 0.0;
        double tipHigh = 0.0;
        bool linearRamp = false;
        bool helicalRamp = false;
        if (moveType == ToolpathMoveType::Cut) {
            if (axialDistance > tolerance) {
                result.eligible = false;
                result.reason =
                    "analytic layered BRep requires constant-depth cuts";
                return finish();
            }
            if (primitive.type == ToolpathPrimitiveType::Linear &&
                planarDistance <= tolerance) {
                continue;
            }
            tipLow = 0.5 * (fromW + toW);
            tipHigh = tipLow;
        } else {
            if (primitive.type == ToolpathPrimitiveType::CircularArc) {
                if (primitive.context.intent !=
                        ToolpathMoveIntent::PocketHelicalRampEntry ||
                    axialDistance <= tolerance ||
                    primitive.arc.sweepRadians >
                        2.0 * std::acos(-1.0) + 1.0e-10) {
                    result.eligible = false;
                    result.reason =
                        "analytic layered BRep requires a one-turn semantic helical entry";
                    return finish();
                }
                helicalRamp = true;
            } else if (primitive.type !=
                       ToolpathPrimitiveType::Linear) {
                result.eligible = false;
                result.reason =
                    "analytic layered BRep defers helical entries";
                return finish();
            }
            if (!helicalRamp && planarDistance > tolerance) {
                if (axialDistance <= tolerance)
                    continue;
                linearRamp = true;
            }
            if (axialDistance <= tolerance)
                continue;
            tipLow = std::min(fromW, toW);
            tipHigh = std::max(fromW, toW);
        }

        ++result.primitiveCount;
        if (primitive.type ==
            ToolpathPrimitiveType::CircularArc) {
            ++result.arcCount;
        }
        if (linearRamp) {
            ++result.linearRampCount;
            AnalyticLinearRampGroup& group =
                findOrCreateLinearRampGroup(
                    rampGroups, primitive.linear);
            group.minimumTip = std::min(
                group.minimumTip, tipLow);
            group.maximumTip = std::max(
                group.maximumTip, tipHigh);
            group.segments.push_back(primitive);
            continue;
        }
        if (helicalRamp) {
            AnalyticHelicalRampGroup& group =
                findOrCreateHelicalRampGroup(
                    helicalRampGroups, primitive.context);
            group.minimumTip = std::min(
                group.minimumTip, tipLow);
            group.maximumTip = std::max(
                group.maximumTip, tipHigh);
            group.moveCount += static_cast<int>(
                primitive.fallbackMoves.size());
            group.arcs.push_back(primitive);
            result.helicalRampMoveCount += static_cast<int>(
                primitive.fallbackMoves.size());
            continue;
        }
        const double bottom = std::max(
            tipLow, stockMinimum);
        const double top = std::min(
            tipHigh + cuttingLength,
            stockMaximum);
        if (top - bottom <= tolerance)
            continue;
        if (moveType == ToolpathMoveType::Cut &&
            primitive.type == ToolpathPrimitiveType::Linear) {
            planarCuts.push_back(
                {frame.project(from), frame.project(to),
                 bottom, top});
        }
        if (moveType == ToolpathMoveType::Cut) {
            planarCoverage.push_back(
                {projectedCoverageKey(primitive, frame),
                 bottom, top});
        }
        AnalyticPlanarSweepResult sweep =
            planarSweep.build(
                referencePlane, primitive,
                cutterRadius, tolerance,
                // 语义圆弧拟合器将其残差单独报告为 maximumCurveFitError。
                // 详细端点规划将建模预算的三分之一分配给每个曲线分量;
                // 此处接受完整的曲线份额,使无害的拟合端点残差不会
                // 迫使退化为多边形。
                std::isfinite(
                        options.arcChordTolerance)
                    ? 3.0 *
                          options.arcChordTolerance
                    : tolerance);
        if (!sweep.ok()) {
            result.reason = sweep.error.empty()
                ? "analytic planar sweep construction failed"
                : sweep.error;
            if (!closedOffsetFillRejection.empty()) {
                result.reason +=
                    "; closed offset fill rejected: " +
                    closedOffsetFillRejection;
            }
            return finish();
        }
        records.push_back(
            {bottom, top, std::move(sweep.face),
             primitiveGeometryKey(primitive, frame)});
    }
    result.linearRampGroupCount =
        static_cast<int>(rampGroups.size());
    result.helicalRampGroupCount =
        static_cast<int>(helicalRampGroups.size());
    const Clock::time_point rampStart = Clock::now();
    for (const AnalyticLinearRampGroup& group : rampGroups) {
        // 生成的直线进刀沿同一组中心线段行进,而该环随后在恒定深度切削。
        // 仅当精确共线的直线扫掠覆盖斜坡每一段完整的毛坯裁剪轴向范围时,
        // 才跳过该进刀。圆形图元不作为依据:将其回退弦替换为精确圆弧
        // 绝不能把这个包含捷径变成过切。
        if (linearRampCoveredByPlanarCuts(
                group, planarCuts, frame, cuttingLength,
                stockMinimum, stockMaximum, tolerance)) {
            ++result.containedRampGroupCount;
            result.containedRampCount +=
                static_cast<int>(group.segments.size());
            continue;
        }
        if (!appendLinearRampRecords(
                group, frame, referencePlane,
                planarSweep, cutterRadius, cuttingLength,
                stockMinimum, stockMaximum,
                rampAxialStep, tolerance,
                maximumRampSlices,
                cancellationRequested,
                records, result)) {
            result.rampProcessingMilliseconds +=
                elapsedMilliseconds(rampStart);
            return finish();
        }
    }
    for (const AnalyticHelicalRampGroup& group :
         helicalRampGroups) {
        // 当精确的恒定深度图元覆盖螺旋线整个毛坯裁剪轴向扫掠中的相同投影
        // 路径时,语义螺旋就是冗余的。这个保守证明避免了为普通腔进刀创建
        // 数千个 quality-10 轴向切片。
        if (helicalRampCoveredByPlanarCuts(
                group, planarCoverage, frame, cuttingLength,
                stockMinimum, stockMaximum, tolerance)) {
            ++result.containedRampGroupCount;
            result.containedRampCount += group.moveCount;
            continue;
        }
        if (!appendHelicalRampRecords(

                group, frame, referencePlane,
                planarSweep, cutterRadius, cuttingLength,
                stockMinimum, stockMaximum,
                rampAxialStep, tolerance,
                maximumRampSlices,
                cancellationRequested,
                records, result)) {
            result.rampProcessingMilliseconds +=
                elapsedMilliseconds(rampStart);
            return finish();
        }
    }
    result.rampProcessingMilliseconds +=
        elapsedMilliseconds(rampStart);
    result.pathCollectionMilliseconds =
        elapsedMilliseconds(pathStart);

    if (records.empty()) {
        result.noMaterial = true;
        result.succeeded = true;
        return finish();
    }

    if (useClosedOffsetFill) {
        if (ballEnd) {
            // 在填充的平面中心区域上扫掠完整球体,正是该区域按球半径
            // 偏置后的精确底部圆角。这完全避免了密集的轮廓放样截面堆叠:
            // 圆角的法向截面是解析 R 圆,因此在高度 h 处其径向距离为
            // sqrt(2Rh-h^2)。
            if (records.size() == 1u) {
                try {
                    const Clock::time_point solidStart = Clock::now();
                    const AnalyticSweepRecord& record = records.front();
                    BRepBuilderAPI_MakeFace centerFaceBuilder(
                        record.centerWire, Standard_True);
                    centerFaceBuilder.Build();
                    TopoDS_Face centerFace;
                    const double centerBottom =
                        record.sourceTip + cutterRadius;
                    const double centerTop =
                        stockMaximum + cutterRadius +
                        10.0 * tolerance;
                    if (centerFaceBuilder.IsDone() &&
                        !centerFaceBuilder.Face().IsNull() &&
                        translatedFace(
                            centerFaceBuilder.Face(), frame,
                            centerBottom, centerFace) &&
                        centerTop > centerBottom + tolerance) {
                        BRepPrimAPI_MakePrism centerPrism(
                            centerFace,
                            gp_Vec(frame.w).Multiplied(
                                centerTop - centerBottom),
                            Standard_False, Standard_True);
                        centerPrism.Build();
                        if (centerPrism.IsDone() &&
                            !centerPrism.Shape().IsNull()) {
                            BRepOffsetAPI_MakeOffsetShape offset;
                            offset.PerformByJoin(
                                centerPrism.Shape(),
                                cutterRadius, tolerance,
                                BRepOffset_Skin,
                                Standard_False,
                                Standard_False,
                                GeomAbs_Arc,
                                Standard_False);
                            if (cancelled(cancellationRequested)) {
                                result.cancelled = true;
                                result.solidBuildMilliseconds =
                                    elapsedMilliseconds(solidStart);
                                return finish();
                            }
                            const TopoDS_Shape rounded =
                                offset.Shape();
                            int solidCount = 0;
                            for (TopExp_Explorer explorer(
                                     rounded, TopAbs_SOLID);
                                 explorer.More(); explorer.Next()) {
                                ++solidCount;
                            }
                            GProp_GProps volumeProperties;
                            if (offset.IsDone() &&
                                !rounded.IsNull() &&
                                solidCount == 1 &&
                                BRepCheck_Analyzer(
                                    rounded, Standard_True,
                                    Standard_False)
                                    .IsValid()) {
                                BRepGProp::VolumeProperties(
                                    rounded, volumeProperties);
                            }
                            if (offset.IsDone() &&
                                !rounded.IsNull() &&
                                solidCount == 1 &&
                                std::isfinite(
                                    volumeProperties.Mass()) &&
                                volumeProperties.Mass() >
                                    tolerance * tolerance * tolerance) {
                                GProp_GProps areaProperties;
                                BRepGProp::SurfaceProperties(
                                    record.face, areaProperties);
                                const double area = std::abs(
                                    areaProperties.Mass());
                                result.removalVolume = rounded;
                                result.removalSolidCount = 1;
                                result.planarLayerCount = 1;
                                result.footprintOuterCount = 1;
                                result.footprintArea = area;
                                result.maximumFootprintArea = area;
                                result.toolProfileBandCount = 0;
                                result.maximumToolProfileAxialError = 0.0;
                                result.maximumToolProfileRadialError = 0.0;
                                result.solidBuildMilliseconds =
                                    elapsedMilliseconds(solidStart);
                                result.succeeded = true;
                                return finish();
                            }
                        }
                    }
                } catch (const Standard_Failure&) {
                    result.reason =
                        "OCCT failed while constructing exact analytic ball-end offset fill";
                    return finish();
                } catch (...) {
                    result.reason =
                        "failed while constructing exact analytic ball-end offset fill";
                    return finish();
                }
                result.reason =
                    "exact analytic ball-end offset-fill construction failed";
                return finish();
            }
            result.reason =
                "exact analytic ball-end BRep requires one compact closed-fill record";
            return finish();
        }
        try {
            std::stable_sort(
                records.begin(), records.end(),
                [](const AnalyticSweepRecord& first,
                   const AnalyticSweepRecord& second) {
                    return first.bottom > second.bottom;
                });
            const double solidTop =
                stockMaximum + 10.0 * tolerance;
            const Clock::time_point solidStart = Clock::now();

            // 嵌套的平底填充是每个轴向带各一个足迹的并集。直接把这些带
            // 构建为棱柱,而不是让放样去推断不相关闭合 B-spline 接缝之间的
            // 点对应关系。棱柱仅在水平边界上接触,因此 InProcessStock
            // 可将它们作为独立布尔刀具提交,不会出现 ThruSections 产生的
            // 扭曲面。
            TopoDS_Compound removal;
            BRep_Builder removalBuilder;
            removalBuilder.MakeCompound(removal);
            double upper = solidTop;
            int solidCount = 0;
            for (const AnalyticSweepRecord& record : records) {
                if (cancelled(cancellationRequested)) {
                    result.cancelled = true;
                    return finish();
                }
                const double lower = std::max(
                    stockMinimum, record.bottom);
                if (upper - lower <= tolerance) {
                    result.reason =
                        "nested offset-fill prism order is invalid";
                    return finish();
                }

                GProp_GProps properties;
                BRepGProp::SurfaceProperties(
                    record.face, properties);
                const double area = std::abs(properties.Mass());
                if (!std::isfinite(area) ||
                    area <= tolerance * tolerance) {
                    result.reason =
                        "nested offset-fill face has invalid area";
                    return finish();
                }

                TopoDS_Face lowerFace;
                if (!translatedFace(
                        record.face, frame, lower,
                        lowerFace)) {
                    result.reason =
                        "failed to position nested offset-fill prism face";
                    return finish();
                }
                BRepPrimAPI_MakePrism prism(
                    lowerFace,
                    gp_Vec(frame.w).Multiplied(upper - lower),
                    Standard_False, Standard_True);
                prism.Build();
                if (!prism.IsDone() || prism.Shape().IsNull()) {
                    result.reason =
                        "failed to build nested offset-fill prism";
                    return finish();
                }

                const TopoDS_Shape& prismShape = prism.Shape();
                if (prismShape.IsNull() ||
                    !BRepCheck_Analyzer(
                         prismShape, Standard_True,
                         Standard_False)
                         .IsValid()) {
                    result.reason =
                        "nested offset-fill prism is invalid";
                    return finish();
                }
                removalBuilder.Add(removal, prismShape);
                ++solidCount;
                result.footprintArea += area;
                result.maximumFootprintArea = std::max(
                    result.maximumFootprintArea, area);
                upper = lower;
            }
            result.solidBuildMilliseconds =
                elapsedMilliseconds(solidStart);
            if (solidCount <= 0) {
                result.reason =
                    "nested offset-fill generated no prism solid";
                return finish();
            }
            result.removalVolume = removal;
            result.removalSolidCount = solidCount;
        } catch (const Standard_Failure&) {
            result.reason =
                "OCCT failed while building nested offset-fill solids";
            return finish();
        } catch (...) {
            result.reason =
                "failed while building nested offset-fill solids";
            return finish();
        }
        result.planarLayerCount =
            static_cast<int>(records.size());
        result.footprintOuterCount =
            static_cast<int>(records.size());
        result.succeeded = true;
        return finish();
    }

    const std::vector<AnalyticSlab> slabs =
        makeSlabs(records, tolerance);
    if (slabs.empty()) {
        result.reason =
            "analytic layered BRep generated no active axial slab";
        return finish();
    }

    TopoDS_Compound removal;
    BRep_Builder compoundBuilder;
    compoundBuilder.MakeCompound(removal);
    const AnalyticPlanarBoolean planarBoolean;
    std::vector<std::string> previousSignature;
    std::vector<TopoDS_Face> previousAreaFaces;
    try {
        for (std::size_t slabIndex = 0;
             slabIndex < slabs.size(); ++slabIndex) {
            const AnalyticSlab& slab = slabs[slabIndex];
            if (cancelled(cancellationRequested)) {
                result.cancelled = true;
                return finish();
            }

            // 腔片体通常只在从底面移向毛坯顶部时增加足迹。复用已合并的
            // 前序区域,只融合新激活的语义足迹。若任一记录过期,则从其
            // 原始激活集合事务性地重建该片体。这避免了为每一细斜坡层反复
            // 融合每一条恒定深度路径。
            const bool incremental =
                !previousSignature.empty() &&
                std::includes(
                    slab.geometrySignature.begin(),
                    slab.geometrySignature.end(),
                    previousSignature.begin(),
                    previousSignature.end());
            std::vector<TopoDS_Face> faces;
            if (incremental) {
                faces = previousAreaFaces;
                faces.reserve(
                    faces.size() + slab.activeRecords.size());
            } else {
                faces.reserve(slab.activeRecords.size());
            }
            std::unordered_set<std::string> inserted;
            for (const std::size_t record : slab.activeRecords) {
                const std::string& key =
                    records[record].geometryKey;
                if (incremental &&
                    std::binary_search(
                        previousSignature.begin(),
                        previousSignature.end(), key)) {
                    continue;
                }
                if (inserted.insert(key).second)
                    faces.push_back(records[record].face);
            }

            const Clock::time_point booleanStart = Clock::now();
            AnalyticPlanarBooleanResult area =
                planarBoolean.unite(
                    faces, tolerance,
                    options.runParallel,
                    cancellationRequested);
            result.planarBooleanMilliseconds +=
                elapsedMilliseconds(booleanStart);
            if (area.cancelled) {
                result.cancelled = true;
                return finish();
            }
            if (!area.ok()) {
                result.reason = area.error.empty()
                    ? "analytic planar boolean failed"
                    : area.error;
                return finish();
            }
            result.booleanGroupCount +=
                area.spatialGroupCount;
            previousSignature = slab.geometrySignature;
            previousAreaFaces = area.faces;

            result.footprintArea += area.unionArea;
            result.maximumFootprintArea = std::max(
                result.maximumFootprintArea,
                area.unionArea);
            const Clock::time_point solidStart = Clock::now();
            // 否则相邻解析片体将共享一条完全共面的边界。多工具 Cut 可能
            // 拒绝该复合体,顺序 Cut 也可能在其中间毛坯中留下同样的重合。
            // 让相邻片体重叠一个建模容差;这既保持经认证的边界预算,又
            // 使它们的并集在布尔运算上更稳健。
            const bool touchesPrevious =
                slabIndex > 0u &&
                std::abs(
                    slabs[slabIndex - 1u].top -
                    slab.bottom) <= tolerance;
            const bool touchesNext =
                slabIndex + 1u < slabs.size() &&
                std::abs(
                    slab.top -
                    slabs[slabIndex + 1u].bottom) <= tolerance;
            const double solidBottom = std::max(
                stockMinimum,
                slab.bottom -
                    (touchesPrevious ? tolerance : 0.0));
            const double solidTop = std::min(
                stockMaximum,
                slab.top +
                    (touchesNext ? tolerance : 0.0));
            for (const TopoDS_Face& referenceFace : area.faces) {
                TopoDS_Face face;
                if (!translatedFace(
                        referenceFace, frame,
                        solidBottom, face)) {
                    result.reason =
                        "failed to position analytic slab face";
                    return finish();
                }
                const int wires = wireCount(face);
                if (wires <= 0) {
                    result.reason =
                        "analytic slab face has no closed wire";
                    return finish();
                }
                ++result.footprintOuterCount;
                result.footprintHoleCount += wires - 1;
                BRepPrimAPI_MakePrism prism(
                    face,
                    gp_Vec(frame.w).Multiplied(
                        solidTop - solidBottom),
                    Standard_True, Standard_True);
                if (!prism.IsDone() ||
                    prism.Shape().IsNull() ||
                    !BRepCheck_Analyzer(
                         prism.Shape(),
                         Standard_True,
                         Standard_False)
                         .IsValid()) {
                    result.reason =
                        "analytic slab prism is invalid";
                    return finish();
                }
                compoundBuilder.Add(
                    removal, prism.Shape());
                ++result.removalSolidCount;
            }
            result.solidBuildMilliseconds +=
                elapsedMilliseconds(solidStart);
            ++result.planarLayerCount;
        }
    } catch (const Standard_Failure&) {
        result.reason =
            "OCCT failed while building analytic slab solids";
        return finish();
    } catch (...) {
        result.reason =
            "failed while building analytic slab solids";
        return finish();
    }
    if (result.removalSolidCount <= 0) {
        result.reason =
            "analytic layered BRep generated no removal solid";
        return finish();
    }
    result.removalVolume = removal;
    result.succeeded = true;
    return finish();
}

} // namespace oap::toolpath
