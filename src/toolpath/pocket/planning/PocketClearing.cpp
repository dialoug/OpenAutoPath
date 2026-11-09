// 槽腔刀路总生成器：规划深度层、构造可达区域、生成清料图案并组装安全移动。
#include "toolpath/pocket/planning/PocketClearing.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <mutex>
#include <string>
#include <utility>

#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>

#include "PocketLayerSlicer.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/pocket/planning/detail/DepthLayerPlanner.h"
#include "toolpath/pocket/planning/detail/PocketClipperUtils.h"
#include "toolpath/pocket/planning/detail/PocketGeometryUtils.h"
#include "toolpath/pocket/planning/detail/PocketMoveAssembler.h"
#include "toolpath/pocket/planning/detail/PocketPatterns.h"

namespace oap::toolpath {

namespace {

constexpr int kMinClipperPrecision = 0;
constexpr int kMaxClipperPrecision = 8;
using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(const Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - start)
        .count();
}

template <typename Function>
void parallelFor(
    std::size_t count,
    const ToolpathGenerationControl* control,
    Function&& function)
{
    toolpathParallelForBudgeted(count, std::forward<Function>(function), control);
}

PocketClearingFailureCode clearingFailureFromSlice(
    PocketLayerSliceFailureCode failureCode)
{
    switch (failureCode) {
        case PocketLayerSliceFailureCode::NoEdges:
            return PocketClearingFailureCode::LayerNoEdges;
        case PocketLayerSliceFailureCode::OpenContour:
            return PocketClearingFailureCode::LayerOpenContour;
        case PocketLayerSliceFailureCode::NoValidContour:
            return PocketClearingFailureCode::LayerNoValidArea;
        case PocketLayerSliceFailureCode::None:
        case PocketLayerSliceFailureCode::InvalidDepth:
        case PocketLayerSliceFailureCode::MissingSectionShape:
        case PocketLayerSliceFailureCode::SectionOperationFailed:
        case PocketLayerSliceFailureCode::NoWires:
            return PocketClearingFailureCode::LayerSectionFailed;
    }
    return PocketClearingFailureCode::LayerSectionFailed;
}

} // namespace

PocketClearingResult PocketClearing::generate(const PocketRegion& region,
                                              const ToolpathParams& params,
                                              const ToolpathGenerationControl* control) const
{
    ToolpathParallelScope parallelScope(control);
    using detail::DepthLayer;
    using detail::PathsD;

    PocketClearingResult result;
    const Clock::time_point totalStart = Clock::now();
    const auto finish = [&] {
        result.usedParallelPatternProcessing =
            result.patternParallelBatchCount > 0u;
        result.totalMilliseconds =
            elapsedMilliseconds(totalStart);
        return result;
    };
    std::mutex cancellationMutex;
    const auto cancellationRequested = [&] {
        if (!control)
            return false;
        std::lock_guard<std::mutex> lock(cancellationMutex);
        return control->cancelled();
    };
    ToolpathGenerationControl serializedControl;
    const ToolpathGenerationControl* patternControl = nullptr;
    if (control) {
        serializedControl = *control;
        serializedControl.isCancellationRequested =
            cancellationRequested;
        patternControl = &serializedControl;
    }
    const auto markCancelled = [&] {
        result.loops.clear();
        result.moves.clear();
        result.primitives.clear();
        result.ok = false;
        result.cancelled = true;
        result.failureCode = PocketClearingFailureCode::None;
        result.error = "pocket generation was cancelled";
    };
    if (cancellationRequested()) {
        markCancelled();
        return finish();
    }

    if (region.boundary.points.size() < 3) {
        result.error = "pocket region has no usable boundary";
        result.failureCode = PocketClearingFailureCode::InvalidRegion;
        return finish();
    }

    // 在生成器边界再次约束 Clipper 精度，避免异常配置造成坐标放大溢出。
    ClearingParams clearing = params.clearing;
    const int requestedPrecision = clearing.precision;
    clearing.precision = std::clamp(
        clearing.precision, kMinClipperPrecision, kMaxClipperPrecision);
    if (clearing.precision != requestedPrecision) {
        detail::appendWarning(result.warning,
                              "Clipper precision was clamped to [0, 8]");
    }

    const double radius = params.tool.radius();
    result.toolRadius = radius;
    if (!std::isfinite(radius) || radius <= 0.0) {
        result.error = "tool radius must be positive";
        result.failureCode = PocketClearingFailureCode::InvalidRegion;
        return finish();
    }

    // 步距以刀具直径百分比表达；未配置时取半径作为保守默认值。
    double step = clearing.stepOverPercent > 0.0
        ? params.tool.diameter * clearing.stepOverPercent / 100.0
        : radius;
    if (step <= 0.0)
        step = radius;
    result.stepOver = step;

    const double firstOffset = radius + std::max(0.0, clearing.extraOffset);
    result.firstOffset = firstOffset;
    const double previousToolRadius =
        0.5 * clearing.restMachiningPreviousToolDiameter;
    result.restMachining =
        std::isfinite(previousToolRadius) &&
        previousToolRadius > radius +
            std::max(1.0e-7, std::pow(10.0, -clearing.precision));
    const bool reverse = clearing.cutMode == CutMode::Conventional;
    const std::vector<DepthLayer> depthLayers =
        detail::buildDepthLayers(region, params.depth, result);

    // 图案选择只消费归一化二维区域；螺旋不适用时自动回退到偏置清料。
    auto generatePattern = [&](const PathsD& area,
                               bool cutterCenterArea,
                               int& ringCount,
                               std::string& warning,
                               detail::PocketPatternParallelAudit&
                                   parallelAudit,
                               detail::AdaptiveFreePathDiagnostics*
                                   adaptiveDiagnostics) {
        ringCount = 0;
        const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(area);
        const double span =
            std::max(bounds.right - bounds.left, bounds.bottom - bounds.top);
        const int maximumRings =
            std::max(1, static_cast<int>(std::ceil(span / step)) + 8);

        if (clearing.pattern == ClearingPattern::ZigZag) {
            const double patternRadius =
                cutterCenterArea ? 0.0 : radius;
            const double patternExtraOffset =
                cutterCenterArea
                    ? 0.0
                    : clearing.extraOffset;
            std::vector<ClearingLoop> loops = detail::generateZigZagLoops(
                area, patternRadius, patternExtraOffset,
                step, clearing.zigzagAngle,
                clearing.perimeterPass, reverse, clearing.precision,
                clearing.arcTolerance, patternControl,
                &parallelAudit);
            ringCount = static_cast<int>(loops.size());
            return loops;
        }

        const double patternFirstOffset =
            cutterCenterArea ? 0.0 : firstOffset;
        if (clearing.pattern == ClearingPattern::FreePath) {
            return detail::generateFreePathLoops(
                area, patternFirstOffset, step,
                params.tool.diameter, clearing.adaptive, reverse,
                clearing.precision, clearing.arcTolerance,
                ringCount, patternControl, adaptiveDiagnostics);
        }
        if (clearing.pattern == ClearingPattern::Spiral) {
            std::vector<ClearingLoop> loops = detail::generateSpiralLoops(
                area, patternFirstOffset, step,
                maximumRings, reverse,
                clearing.expansionDirection, clearing.precision,
                clearing.arcTolerance, ringCount);
            if (!loops.empty())
                return loops;

            detail::appendWarning(
                warning,
                "Spiral is only supported for simply-connected pockets; "
                "falling back to Offset");
        }

        return detail::generateOffsetLoops(
            area, patternFirstOffset, step,
            maximumRings, reverse,
            clearing.expansionDirection, clearing.precision,
            clearing.arcTolerance, ringCount, patternControl,
            &parallelAudit);
    };

    // 二维图案在此按层映射回世界坐标，保持图案算法与 OCCT 坐标系解耦。
    auto appendLayerLoops = [&](const std::vector<ClearingLoop>& planarLoops,
                                const DepthLayer& layer) {
        for (const ClearingLoop& planar : planarLoops) {
            ClearingLoop loop = planar;
            loop.depthIndex = layer.index;
            loop.depthOffset = layer.offset;
            loop.world.clear();
            loop.world.reserve(loop.points.size());
            for (const gp_Pnt2d& point : loop.points) {
                loop.world.push_back(detail::pointFrom2dAtDepth(
                    point, region.frame, layer.offset));
            }
            loop.preferredWorldLinkFromPrevious.clear();
            loop.preferredWorldLinkFromPrevious.reserve(
                loop.preferredLinkFromPrevious.size());
            for (const gp_Pnt2d& point :
                 loop.preferredLinkFromPrevious) {
                loop.preferredWorldLinkFromPrevious.push_back(
                    detail::pointFrom2dAtDepth(
                        point, region.frame, layer.offset));
            }
            loop.sequence = static_cast<int>(result.loops.size());
            result.loops.push_back(std::move(loop));
        }
    };

    // 常规腔的 Offset/ZigZag 逐层截面。主外表面内环追踪结构没有传统
    // 底面区域，所有图案（包括 Slot/Spiral）都必须使用每层真实截面。
    const bool usePerLayerSections =
        !region.sectionShape.IsNull() &&
        (!region.protectedMaterialShape.IsNull() ||
         region.useSectionBoundaryAtEveryDepth ||
         (region.kind == FeatureKind::Pocket &&
           (clearing.pattern == ClearingPattern::Offset ||
           clearing.pattern == ClearingPattern::ZigZag ||
           clearing.pattern == ClearingPattern::FreePath)));

    if (usePerLayerSections) {
        result.usedPerLayerSections = true;
        result.usedProtectedMaterialSections =
            !region.protectedMaterialShape.IsNull();
        result.usedParallelLayerProcessing =
            depthLayers.size() > 1;

        // 逐层处理的状态机：切片 -> 材料阴影 -> 图案 -> 就绪/失败，
        // 每层独立进入 generateLayerPattern，支持 OSD 并行。
        enum class LayerState {
            Pending,
            SectionReady,
            SectionFailed,
            AreaEmpty,
            ToolDoesNotFit,
            PatternFailed,
            NoRestMaterial,
            Ready,
            Cancelled,
        };
        // 每个深度层的工作单元：保存切片结果、归一化区域、生成图案
        // 与诊断信息，由并行流水线填充。
        struct LayerWork {
            PocketLayerSlice slice;
            PathsD area;
            PathsD protectedMaterialArea;
            std::vector<ClearingLoop> loops;
            std::string error;
            std::string warning;
            LayerState state = LayerState::Pending;
            int ringCount = 0;
            int virtualOpenBoundaryCount = 0;
            detail::PocketPatternParallelAudit patternParallelAudit;
            detail::AdaptiveFreePathDiagnostics adaptiveDiagnostics;
        };
        std::vector<LayerWork> layerWork(depthLayers.size());
        std::atomic_bool cancellationObserved{false};
        PocketMaterialSectionCache* materialCache =
            control &&
                    control->pocketMaterialSectionCache
                ? control->pocketMaterialSectionCache.get()
                : nullptr;

        const auto generateLayerPattern =
            [&](std::size_t layerIndex) {
            LayerWork& work = layerWork[layerIndex];
            if (cancellationObserved.load(std::memory_order_relaxed) ||
                cancellationRequested()) {
                cancellationObserved.store(true, std::memory_order_relaxed);
                work.state = LayerState::Cancelled;
                return;
            }
            try {
                work.area = detail::normalizedClearArea(
                    work.slice.contours, clearing.precision);
                work.virtualOpenBoundaryCount =
                    work.slice.virtualOpenBoundaryCount;
                if (!work.area.empty() &&
                    work.virtualOpenBoundaryCount > 0) {
                    work.area =
                        detail::extendThroughVirtualOpenBoundaries(
                            work.area, work.slice.contours,
                            firstOffset + std::max(radius, step),
                            clearing.precision);
                }
                const bool hasProtectedMaterial =
                    !region.protectedMaterialShape.IsNull();
                if (hasProtectedMaterial &&
                    work.protectedMaterialArea.empty()) {
                    work.error =
                        "protected-material axial shadow has no usable area";
                    work.state = LayerState::PatternFailed;
                    return;
                }
                // 余料加工：用“上一大刀已覆盖范围”差出本刀仍需清除的区域；
                // 普通清腔则按实体材料阴影扣除刀具中心不可达区。
                if (result.restMachining) {
                    work.area = detail::restMachiningCenterArea(
                        work.area,
                        work.protectedMaterialArea,
                        radius, previousToolRadius,
                        std::max(0.0, clearing.extraOffset),
                        clearing.precision,
                        clearing.arcTolerance);
                    if (work.area.empty()) {
                        work.state = LayerState::NoRestMaterial;
                        return;
                    }
                } else if (hasProtectedMaterial) {
                    work.area =
                        detail::cutterCenterReachableArea(
                            work.area,
                            work.protectedMaterialArea,
                            firstOffset,
                            clearing.precision,
                            clearing.arcTolerance);
                    if (work.area.empty()) {
                        work.state =
                            LayerState::ToolDoesNotFit;
                        return;
                    }
                }
                if (work.area.empty()) {
                    work.state = LayerState::AreaEmpty;
                    return;
                }
                work.loops = generatePattern(
                    work.area,
                    hasProtectedMaterial || result.restMachining,
                    work.ringCount, work.warning,
                    work.patternParallelAudit,
                    &work.adaptiveDiagnostics);
                work.area.clear();
                work.slice.contours.clear();
                if (work.loops.empty()) {
                    work.state = LayerState::ToolDoesNotFit;
                    return;
                }
                work.state = LayerState::Ready;
            } catch (const Standard_Failure& failure) {
                work.state = LayerState::PatternFailed;
                const char* message = failure.GetMessageString();
                work.error = message
                    ? std::string("OCCT layer pattern failed: ") + message
                    : "OCCT layer pattern failed";
            } catch (const std::exception& failure) {
                work.state = LayerState::PatternFailed;
                work.error =
                    std::string("layer pattern failed: ") + failure.what();
            } catch (...) {
                work.state = LayerState::PatternFailed;
                work.error =
                    "layer pattern failed with an unknown exception";
            }
        };

        const auto sliceLayer =
            [&](std::size_t layerIndex) {
            LayerWork& work = layerWork[layerIndex];
            if (cancellationObserved.load(std::memory_order_relaxed) ||
                cancellationRequested()) {
                cancellationObserved.store(true, std::memory_order_relaxed);
                work.state = LayerState::Cancelled;
                return;
            }
            try {
                OCC_CATCH_SIGNALS
                thread_local bool signalsInitialized = false;
                if (!signalsInitialized) {
                    OSD::SetThreadLocalSignal(
                        OSD_SignalMode_Set, Standard_False);
                    signalsInitialized = true;
                }
                PocketLayerSlicer slicer;
                const bool sliceOk = materialCache
                    ? materialCache->sliceRegion(
                          region,
                          depthLayers[layerIndex].offset,
                          work.slice, work.error,
                          patternControl)
                    : slicer.slice(
                          region,
                          depthLayers[layerIndex].offset,
                          work.slice, work.error,
                          nullptr, patternControl);
                if (!sliceOk) {
                    work.state = LayerState::SectionFailed;
                    return;
                }
                work.state = LayerState::SectionReady;
            } catch (const Standard_Failure& failure) {
                work.state = LayerState::SectionFailed;
                const char* message = failure.GetMessageString();
                work.error = message
                    ? std::string("OCCT layer section failed: ") +
                          message
                    : "OCCT layer section failed";
            } catch (const std::exception& failure) {
                work.state = LayerState::SectionFailed;
                work.error =
                    std::string("layer section failed: ") +
                    failure.what();
            } catch (...) {
                work.state = LayerState::SectionFailed;
                work.error =
                    "layer section failed with an unknown exception";
            }
        };

        const auto accumulateSectionAudit =
            [&](const PocketLayerSlice& slice) {
            result.sectionWireParallelBatchCount +=
                slice.wireParallelBatchCount;
            result.sectionWireParallelTaskCount +=
                slice.wireParallelTaskCount;
            result.sectionWirePeakParallelism = std::max(
                result.sectionWirePeakParallelism,
                slice.wirePeakParallelism);
            result.occtInternalParallelCount +=
                slice.occtInternalParallelCount;
            result.occtSerialFallbackCount +=
                slice.occtSerialFallbackCount;
        };

        const Clock::time_point sectionStart = Clock::now();
        sliceLayer(0);
        accumulateSectionAudit(layerWork.front().slice);
        if (cancellationObserved.load(std::memory_order_relaxed) ||
            cancellationRequested()) {
            result.sectionMilliseconds =
                elapsedMilliseconds(sectionStart);
            markCancelled();
            return finish();
        }
        if (layerWork.front().state !=
            LayerState::SectionReady) {
            const LayerWork& work = layerWork.front();
            const DepthLayer& layer = depthLayers.front();
            result.failureCode = clearingFailureFromSlice(
                work.slice.failureCode);
            result.failedDepthIndex = layer.index;
            result.failedDepthOffset = layer.offset;
            result.error = "layer section failed: " + work.error;
            result.sectionMilliseconds =
                elapsedMilliseconds(sectionStart);
            return finish();
        }

        PathsD axialMaterialShadow;
        if (!region.protectedMaterialShape.IsNull()) {
            // 固定三轴刀具从当前刀尖层一直延伸到开口侧。只扣除当前 Z
            // 截面会漏掉位于刀尖上方的悬空凸台/倒扣，因此从开口层向下
            // 累积实体截面，形成每个加工层各自的轴向材料阴影。
            PocketLayerSlice openingMaterialSlice;
            std::string openingMaterialError;
            PocketLayerSlicer materialSlicer;
            const bool openingMaterialOk =
                materialCache
                    ? materialCache->slice(
                          region, region.depth,
                          openingMaterialSlice,
                          openingMaterialError,
                          patternControl)
                    : materialSlicer.sliceProtectedMaterial(
                          region, region.depth,
                          openingMaterialSlice,
                          openingMaterialError,
                          patternControl);
            accumulateSectionAudit(openingMaterialSlice);
            if (!openingMaterialOk) {
                result.failureCode = clearingFailureFromSlice(
                    openingMaterialSlice.failureCode);
                result.failedDepthIndex =
                    depthLayers.empty()
                        ? -1
                        : depthLayers.front().index;
                result.failedDepthOffset = region.depth;
                result.error =
                    "opening protected-material section failed: " +
                    openingMaterialError;
                result.sectionMilliseconds =
                    elapsedMilliseconds(sectionStart);
                return finish();
            }

            axialMaterialShadow =
                detail::normalizedNestedArea(
                    openingMaterialSlice
                        .protectedMaterialContours,
                    clearing.precision);
            if (axialMaterialShadow.empty()) {
                result.failureCode =
                    PocketClearingFailureCode::LayerNoValidArea;
                result.failedDepthIndex =
                    depthLayers.empty()
                        ? -1
                        : depthLayers.front().index;
                result.failedDepthOffset = region.depth;
                result.error =
                    "opening protected-material section has no usable area";
                result.sectionMilliseconds =
                    elapsedMilliseconds(sectionStart);
                return finish();
            }

            for (std::size_t layerIndex = 0;
                 layerIndex < 1;
                 ++layerIndex) {
                LayerWork& work = layerWork[layerIndex];
                PathsD layerMaterial =
                    detail::normalizedNestedArea(
                        work.slice
                            .protectedMaterialContours,
                        clearing.precision);
                if (layerMaterial.empty()) {
                    result.failureCode =
                        PocketClearingFailureCode::LayerNoValidArea;
                    result.failedDepthIndex =
                        depthLayers[layerIndex].index;
                    result.failedDepthOffset =
                        depthLayers[layerIndex].offset;
                    result.error =
                        "protected-material section has no usable area";
                    result.sectionMilliseconds =
                        elapsedMilliseconds(sectionStart);
                    return finish();
                }
                axialMaterialShadow = detail::cleanPaths(
                    Clipper2Lib::Union(
                        axialMaterialShadow,
                        layerMaterial,
                        Clipper2Lib::FillRule::NonZero,
                        clearing.precision),
                    clearing.precision);
                if (axialMaterialShadow.empty()) {
                    result.failureCode =
                        PocketClearingFailureCode::LayerNoValidArea;
                    result.failedDepthIndex =
                        depthLayers[layerIndex].index;
                    result.failedDepthOffset =
                        depthLayers[layerIndex].offset;
                    result.error =
                        "protected-material axial shadow is empty";
                    result.sectionMilliseconds =
                        elapsedMilliseconds(sectionStart);
                    return finish();
                }
                work.protectedMaterialArea =
                    axialMaterialShadow;
                work.slice.protectedMaterialContours =
                    detail::nestedAreaContours(
                        axialMaterialShadow);
            }
        }
        result.sectionMilliseconds =
            elapsedMilliseconds(sectionStart);

        const Clock::time_point patternStart = Clock::now();
        generateLayerPattern(0);

        if (cancellationObserved.load(std::memory_order_relaxed) ||
            cancellationRequested()) {
            result.patternMilliseconds =
                elapsedMilliseconds(patternStart);
            markCancelled();
            return finish();
        }

        double totalPatternMilliseconds =
            elapsedMilliseconds(patternStart);
        if (layerWork.front().state != LayerState::Ready &&
            layerWork.front().state != LayerState::NoRestMaterial) {
            LayerWork& work = layerWork.front();
            const DepthLayer& layer = depthLayers.front();
            result.firstLayerPreflightRejected = true;
            result.failedDepthIndex = layer.index;
            result.failedDepthOffset = layer.offset;
            if (work.state == LayerState::AreaEmpty) {
                result.failureCode =
                    PocketClearingFailureCode::LayerNoValidArea;
                result.error =
                    "layer section has no valid clearable area";
            } else if (
                work.state == LayerState::ToolDoesNotFit) {
                result.failureCode =
                    PocketClearingFailureCode::ToolDoesNotFit;
                result.error =
                    "layer section cannot contain a clearing pass";
            } else {
                result.failureCode =
                    PocketClearingFailureCode::AlgorithmExecutionFailed;
                result.error = work.error.empty()
                    ? "first-layer preflight did not complete"
                    : work.error;
            }
            result.patternMilliseconds =
                totalPatternMilliseconds;
            return finish();
        }

        if (layerWork.size() > 1) {
            const Clock::time_point remainingSectionStart =
                Clock::now();
            parallelFor(
                layerWork.size() - 1,
                patternControl,
                [&](std::size_t index) {
                    sliceLayer(index + 1);
                });
            for (std::size_t layerIndex = 1u;
                 layerIndex < layerWork.size(); ++layerIndex) {
                accumulateSectionAudit(
                    layerWork[layerIndex].slice);
            }
            result.sectionMilliseconds +=
                elapsedMilliseconds(
                    remainingSectionStart);
            if (cancellationObserved.load(
                    std::memory_order_relaxed) ||
                cancellationRequested()) {
                result.patternMilliseconds =
                    totalPatternMilliseconds;
                markCancelled();
                return finish();
            }
            for (std::size_t layerIndex = 1;
                 layerIndex < layerWork.size();
                 ++layerIndex) {
                const LayerWork& work =
                    layerWork[layerIndex];
                if (work.state ==
                    LayerState::SectionReady) {
                    continue;
                }
                const DepthLayer& layer =
                    depthLayers[layerIndex];
                result.failureCode =
                    clearingFailureFromSlice(
                        work.slice.failureCode);
                result.failedDepthIndex =
                    layer.index;
                result.failedDepthOffset =
                    layer.offset;
                result.error =
                    "layer section failed: " +
                    work.error;
                result.patternMilliseconds =
                    totalPatternMilliseconds;
                return finish();
            }

            if (!region.protectedMaterialShape.IsNull()) {
                const Clock::time_point
                    remainingMaterialStart =
                        Clock::now();
                for (std::size_t layerIndex = 1;
                     layerIndex < layerWork.size();
                     ++layerIndex) {
                    LayerWork& work =
                        layerWork[layerIndex];
                    PathsD layerMaterial =
                        detail::normalizedNestedArea(
                            work.slice
                                .protectedMaterialContours,
                            clearing.precision);
                    if (layerMaterial.empty()) {
                        result.failureCode =
                            PocketClearingFailureCode::
                                LayerNoValidArea;
                        result.failedDepthIndex =
                            depthLayers[layerIndex].index;
                        result.failedDepthOffset =
                            depthLayers[layerIndex].offset;
                        result.error =
                            "protected-material section has no usable area";
                        result.sectionMilliseconds +=
                            elapsedMilliseconds(
                                remainingMaterialStart);
                        result.patternMilliseconds =
                            totalPatternMilliseconds;
                        return finish();
                    }
                    axialMaterialShadow =
                        detail::cleanPaths(
                            Clipper2Lib::Union(
                                axialMaterialShadow,
                                layerMaterial,
                                Clipper2Lib::FillRule::
                                    NonZero,
                                clearing.precision),
                            clearing.precision);
                    if (axialMaterialShadow.empty()) {
                        result.failureCode =
                            PocketClearingFailureCode::
                                LayerNoValidArea;
                        result.failedDepthIndex =
                            depthLayers[layerIndex].index;
                        result.failedDepthOffset =
                            depthLayers[layerIndex].offset;
                        result.error =
                            "protected-material axial shadow is empty";
                        result.sectionMilliseconds +=
                            elapsedMilliseconds(
                                remainingMaterialStart);
                        result.patternMilliseconds =
                            totalPatternMilliseconds;
                        return finish();
                    }
                    work.protectedMaterialArea =
                        axialMaterialShadow;
                    work.slice
                        .protectedMaterialContours =
                        detail::nestedAreaContours(
                            axialMaterialShadow);
                }
                result.sectionMilliseconds +=
                    elapsedMilliseconds(
                        remainingMaterialStart);
            }

            const Clock::time_point remainingPatternStart =
                Clock::now();
            parallelFor(
                layerWork.size() - 1,
                patternControl,
                [&](std::size_t index) {
                    generateLayerPattern(index + 1);
                });
            totalPatternMilliseconds +=
                elapsedMilliseconds(
                    remainingPatternStart);
            if (cancellationObserved.load(
                    std::memory_order_relaxed) ||
                cancellationRequested()) {
                result.patternMilliseconds =
                    totalPatternMilliseconds;
                markCancelled();
                return finish();
            }
        }

        bool warnedAboutVirtualOutlet = false;
        for (std::size_t layerIndex = 0;
             layerIndex < layerWork.size(); ++layerIndex) {
            LayerWork& work = layerWork[layerIndex];
            const DepthLayer& layer = depthLayers[layerIndex];
            if (work.state == LayerState::NoRestMaterial) {
                ++result.sectionLayerCount;
                continue;
            }
            if (work.state == LayerState::AreaEmpty) {
                result.loops.clear();
                result.moves.clear();
                result.primitives.clear();
                result.failureCode =
                    PocketClearingFailureCode::LayerNoValidArea;
                result.failedDepthIndex = layer.index;
                result.failedDepthOffset = layer.offset;
                result.error =
                    "layer section has no valid clearable area";
                result.patternMilliseconds =
                    totalPatternMilliseconds;
                return finish();
            }
            if (work.state == LayerState::ToolDoesNotFit) {
                result.loops.clear();
                result.moves.clear();
                result.primitives.clear();
                result.failureCode =
                    PocketClearingFailureCode::ToolDoesNotFit;
                result.failedDepthIndex = layer.index;
                result.failedDepthOffset = layer.offset;
                result.error =
                    "layer section cannot contain a clearing pass";
                result.patternMilliseconds =
                    totalPatternMilliseconds;
                return finish();
            }
            if (work.state != LayerState::Ready) {
                result.loops.clear();
                result.moves.clear();
                result.primitives.clear();
                result.failureCode =
                    PocketClearingFailureCode::AlgorithmExecutionFailed;
                result.failedDepthIndex = layer.index;
                result.failedDepthOffset = layer.offset;
                result.error = work.error.empty()
                    ? "layer pattern did not complete"
                    : work.error;
                result.patternMilliseconds =
                    totalPatternMilliseconds;
                return finish();
            }

            if (work.virtualOpenBoundaryCount > 0) {
                ++result.openSectionLayerCount;
                result.virtualOpenBoundaryCount +=
                    work.virtualOpenBoundaryCount;
                if (!warnedAboutVirtualOutlet) {
                    detail::appendWarning(
                        result.warning,
                        "Connected through-slot sections use virtual "
                        "outlet boundaries");
                    warnedAboutVirtualOutlet = true;
                }
            }
            if (!work.warning.empty())
                detail::appendWarning(result.warning, work.warning);
            result.patternParallelBatchCount +=
                work.patternParallelAudit.batchCount;
            result.patternParallelTaskCount +=
                work.patternParallelAudit.taskCount;
            result.patternPeakParallelism = std::max(
                result.patternPeakParallelism,
                work.patternParallelAudit.peakParallelism);
            result.ringCount =
                std::max(result.ringCount, work.ringCount);
            result.adaptivePassCount +=
                work.adaptiveDiagnostics.adaptivePasses;
            result.adaptiveRemainingComponentCount +=
                work.adaptiveDiagnostics.remainingComponents;
            result.adaptiveOverloadSegmentCount +=
                work.adaptiveDiagnostics.overloadSegments;
            result.adaptiveWavefrontLayerCount +=
                work.adaptiveDiagnostics.wavefrontLayers;
            result.adaptiveCentralSpiralCount +=
                work.adaptiveDiagnostics.centralSpiralRuns;
            result.adaptiveRadialComponentCount +=
                work.adaptiveDiagnostics.radialComponents;
            result.adaptiveRadialPassCount +=
                work.adaptiveDiagnostics.radialPasses;
            result.adaptiveRemainingMaterialFraction = std::max(
                result.adaptiveRemainingMaterialFraction,
                work.adaptiveDiagnostics.remainingMaterialFraction);
            result.adaptiveMaximumEngagementAngleDeg = std::max(
                result.adaptiveMaximumEngagementAngleDeg,
                work.adaptiveDiagnostics.maximumEngagementAngleDeg);
            appendLayerLoops(work.loops, layer);
            if (result.usedProtectedMaterialSections) {
                ProtectedMaterialLayer materialLayer;
                materialLayer.depthIndex = layer.index;
                materialLayer.depthOffset = layer.offset;
                materialLayer.contours = std::move(
                    work.slice.protectedMaterialContours);
                result.protectedMaterialLayers.push_back(
                    std::move(materialLayer));
            }
            ++result.sectionLayerCount;
        }
        result.patternMilliseconds =
            totalPatternMilliseconds;
    } else {
        // 简单区域分支：底面/边界不随深度变化，直接基于二维区域生成
        // 一套图案，再复制到每个深度层（非 Offset/ZigZag 的常规腔、
        // 无保护材料、非逐层截面的情况）。
        const Clock::time_point patternStart = Clock::now();
        PathsD area = detail::normalizedClearArea(region, clearing.precision);
        if (area.empty()) {
            result.error =
                "pocket region has no valid clearable area after boundary/island normalization";
            result.failureCode = PocketClearingFailureCode::NoClearableArea;
            result.patternMilliseconds =
                elapsedMilliseconds(patternStart);
            return finish();
        }

        bool cutterCenterArea = false;
        if (result.restMachining) {
            area = detail::restMachiningCenterArea(
                area, {}, radius, previousToolRadius,
                std::max(0.0, clearing.extraOffset),
                clearing.precision,
                clearing.arcTolerance);
            cutterCenterArea = true;
            if (area.empty()) {
                result.depthLayerCount =
                    static_cast<int>(depthLayers.size());
                result.noRestMaterial = true;
                result.ok = true;
                result.failureCode =
                    PocketClearingFailureCode::None;
                result.patternMilliseconds =
                    elapsedMilliseconds(patternStart);
                return finish();
            }
        }

        int ringCount = 0;
        std::string patternWarning;
        detail::PocketPatternParallelAudit patternParallelAudit;
        detail::AdaptiveFreePathDiagnostics adaptiveDiagnostics;
        const std::vector<ClearingLoop> loops =
            generatePattern(
                area, cutterCenterArea,
                ringCount, patternWarning,
                patternParallelAudit,
                &adaptiveDiagnostics);
        result.adaptivePassCount = adaptiveDiagnostics.adaptivePasses;
        result.adaptiveRemainingComponentCount =
            adaptiveDiagnostics.remainingComponents;
        result.adaptiveOverloadSegmentCount =
            adaptiveDiagnostics.overloadSegments;
        result.adaptiveWavefrontLayerCount =
            adaptiveDiagnostics.wavefrontLayers;
        result.adaptiveCentralSpiralCount =
            adaptiveDiagnostics.centralSpiralRuns;
        result.adaptiveRadialComponentCount =
            adaptiveDiagnostics.radialComponents;
        result.adaptiveRadialPassCount =
            adaptiveDiagnostics.radialPasses;
        result.adaptiveRemainingMaterialFraction =
            adaptiveDiagnostics.remainingMaterialFraction;
        result.adaptiveMaximumEngagementAngleDeg =
            adaptiveDiagnostics.maximumEngagementAngleDeg;
        result.patternParallelBatchCount +=
            patternParallelAudit.batchCount;
        result.patternParallelTaskCount +=
            patternParallelAudit.taskCount;
        result.patternPeakParallelism = std::max(
            result.patternPeakParallelism,
            patternParallelAudit.peakParallelism);
        if (cancellationRequested()) {
            result.patternMilliseconds =
                elapsedMilliseconds(patternStart);
            markCancelled();
            return finish();
        }
        if (!patternWarning.empty())
            detail::appendWarning(result.warning, patternWarning);
        result.ringCount = ringCount;
        if (loops.empty()) {
            result.error = "tool is too large for the region (no clearing pass fits)";
            result.failureCode = PocketClearingFailureCode::NoClearableArea;
            result.patternMilliseconds =
                elapsedMilliseconds(patternStart);
            return finish();
        }

        if (region.kind == FeatureKind::Pocket && !region.sectionShape.IsNull() &&
            clearing.pattern == ClearingPattern::Spiral) {
            detail::appendWarning(
                result.warning,
                "Spiral uses the floor profile at every depth; "
                "per-layer sections are not applied");
        }
        for (const DepthLayer& layer : depthLayers)
            appendLayerLoops(loops, layer);
        result.patternMilliseconds =
            elapsedMilliseconds(patternStart);
    }

    // 平面折线全部生成成功后再统一组装进退刀，避免返回半成品移动序列。
    result.depthLayerCount = static_cast<int>(depthLayers.size());
    if (result.loops.empty() && result.restMachining) {
        result.noRestMaterial = true;
        result.ok = true;
        result.failureCode = PocketClearingFailureCode::None;
        return finish();
    }
    if (cancellationRequested()) {
        markCancelled();
        return finish();
    }
    const Clock::time_point moveStart = Clock::now();
    detail::buildPocketMoves(
        region, params, result, patternControl);
    result.moveAssemblyMilliseconds =
        elapsedMilliseconds(moveStart);
    result.ok = true;
    result.failureCode = PocketClearingFailureCode::None;
    return finish();
}

} // namespace oap::toolpath
