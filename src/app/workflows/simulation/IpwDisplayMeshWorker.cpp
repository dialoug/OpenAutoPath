// IpwDisplayMeshWorker 实现：表面网格/BRep/图元三类表示的构建与抽稀。
#include "app/workflows/simulation/IpwDisplayMeshWorker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>
#include "toolpath/core/simulation/FixedAxisToolpathView.h"

namespace oap::app {

toolpath::ToolpathParams ipwReplayParameters(const IpwTransientFrameSource& source, std::size_t index)
{
    if (!source.operations || index >= source.operations->size() || !source.parentStock) return source.params;
    auto params = source.operationParameters.size() == source.operations->size()
        ? source.operationParameters[index] : source.params;
    if (std::isfinite(params.tool.cuttingEdgeLength) && params.tool.cuttingEdgeLength > 0.0) return params;
    const bool explicitProfile = source.toolProfiles.size() == source.operations->size();
    const double diameter = explicitProfile && std::isfinite(source.toolProfiles[index].diameter) && source.toolProfiles[index].diameter > 0.0
        ? source.toolProfiles[index].diameter : params.tool.diameter;
    double minimum = std::numeric_limits<double>::infinity(), maximum = -minimum;
    for (const auto& move : (*source.operations)[index].clearing.moves) {
        if (move.type != toolpath::ToolpathMoveType::Cut && move.type != toolpath::ToolpathMoveType::Plunge) continue;
        const double from = gp_Vec(source.parentStock->origin(), move.from).Dot(source.parentStock->wAxis());
        const double to = gp_Vec(source.parentStock->origin(), move.to).Dot(source.parentStock->wAxis());
        if (!std::isfinite(from) || !std::isfinite(to)) continue;
        minimum = std::min(minimum, std::min(from, to));
        maximum = std::max(maximum, std::max(from, to));
    }
    params.tool.cuttingEdgeLength = diameter + (std::isfinite(minimum) ? std::max(0.0, maximum - minimum) : 0.0);
    return params;
}

IpwDisplayArtifact displayArtifactFromResult(
    const IpwDisplayMeshResult& result)
{
    IpwDisplayArtifact artifact;
    artifact.mesh = result.mesh;
    artifact.metadata.dexelMetrics = result.metrics;
    artifact.metadata.meshContentFingerprint =
        result.meshContentFingerprint;
    return artifact;
}

namespace {

// 取消检查：无取消句柄或未被置位时返回 false。
bool cancellationRequested(
    const std::shared_ptr<std::atomic_bool>& cancellation) noexcept
{
    return cancellation &&
        cancellation->load(std::memory_order_relaxed);
}

// 由 (u,v,w) 计算世界坐标：沿库存三个轴向平移原点。
gp_Pnt stockPoint(
    const toolpath::MultiDexelStock& stock,
    double u,
    double v,
    double w)
{
    gp_Vec offset = stock.uAxis();
    offset *= u;
    gp_Vec vOffset = stock.vAxis();
    vOffset *= v;
    gp_Vec wOffset = stock.wAxis();
    wOffset *= w;
    offset += vOffset;
    offset += wOffset;
    return stock.origin().Translated(offset);
}

// 计算抽稀步长：源数量超出预算时取 ceil(source/budget)，否则为 1。
int samplingStride(
    std::size_t sourceCount,
    std::size_t budget)
{
    if (budget == 0u || sourceCount <= budget)
        return 1;
    const std::size_t stride =
        (sourceCount + budget - 1u) / budget;
    return static_cast<int>(
        std::min<std::size_t>(
            stride,
            static_cast<std::size_t>(
                std::numeric_limits<int>::max())));
}

// 构建非表面显示表示：MidSection 取 W 向剖面四边形；其余按模式生成
// Dexel 柱段/端点/棱柱，超过预算时按 samplingStride 均匀抽稀。
// 取消回调返回 true 时中途放弃。
bool buildPrimitiveGeometry(
    const toolpath::MultiDexelStock& stock,
    IpwOcctDisplayMode mode,
    double sectionFraction,
    IpwDisplayPrimitiveGeometryPtr& output,
    const std::function<bool()>& cancelled)
{
    auto geometry =
        std::make_shared<IpwDisplayPrimitiveGeometry>();
    const double resolution = stock.resolution();
    const auto cellBounds =
        [&stock, resolution](int i, int j) {
            const double u0 =
                stock.uMin() +
                static_cast<double>(i) * resolution;
            const double v0 =
                stock.vMin() +
                static_cast<double>(j) * resolution;
            return std::array<double, 4>{
                u0,
                std::min(stock.uMax(), u0 + resolution),
                v0,
                std::min(stock.vMax(), v0 + resolution)};
        };

    if (mode == IpwOcctDisplayMode::MidSection) {
        geometry->kind = IpwDisplayPrimitiveKind::Quads;
        // 先扫一遍全网格得到 W 范围，再取比例位置作为剖切高度。
        double wMin = std::numeric_limits<double>::infinity();
        double wMax = -std::numeric_limits<double>::infinity();
        for (int j = 0; j < stock.ny(); ++j) {
            if (cancelled && cancelled())
                return false;
            for (int i = 0; i < stock.nx(); ++i) {
                for (const auto& interval :
                     stock.cellIntervalsView(i, j)) {
                    wMin = std::min(wMin, interval.lower);
                    wMax = std::max(wMax, interval.upper);
                }
            }
        }
        if (std::isfinite(wMin) && std::isfinite(wMax)) {
            const double fraction =
                std::clamp(sectionFraction, 0.0, 1.0);
            const double sectionW =
                wMin + (wMax - wMin) * fraction;
            geometry->sectionCoordinate = sectionW;
            for (int j = 0; j < stock.ny(); ++j) {
                if (cancelled && cancelled())
                    return false;
                for (int i = 0; i < stock.nx(); ++i) {
                    for (const auto& interval :
                         stock.cellIntervalsView(i, j)) {
                        if (interval.lower <= sectionW &&
                            interval.upper >= sectionW) {
                            ++geometry->sourcePrimitiveCount;
                            break;
                        }
                    }
                }
            }
            geometry->samplingStride = samplingStride(
                geometry->sourcePrimitiveCount, 150'000u);
            geometry->quads.reserve(
                std::min<std::size_t>(
                    geometry->sourcePrimitiveCount, 150'000u));
            std::size_t occupiedIndex = 0u;
            for (int j = 0; j < stock.ny(); ++j) {
                if (cancelled && cancelled())
                    return false;
                for (int i = 0; i < stock.nx(); ++i) {
                    bool occupied = false;
                    for (const auto& interval :
                         stock.cellIntervalsView(i, j)) {
                        if (interval.lower <= sectionW &&
                            interval.upper >= sectionW) {
                            occupied = true;
                            break;
                        }
                    }
                    if (!occupied)
                        continue;
                    if ((occupiedIndex++ % static_cast<std::size_t>(
                             geometry->samplingStride)) != 0u) {
                        continue;
                    }
                    const auto bounds = cellBounds(i, j);
                    geometry->quads.push_back({
                        stockPoint(stock, bounds[0], bounds[2], sectionW),
                        stockPoint(stock, bounds[1], bounds[2], sectionW),
                        stockPoint(stock, bounds[1], bounds[3], sectionW),
                        stockPoint(stock, bounds[0], bounds[3], sectionW)});
                }
            }
        }
        geometry->displayedPrimitiveCount = geometry->quads.size();
        output = std::move(geometry);
        return true;
    }

    const std::size_t sourceCount =
        static_cast<std::size_t>(
            std::max(0, stock.intervalCount()));
    geometry->sourcePrimitiveCount = sourceCount;
    std::size_t budget = 150'000u;
    if (mode == IpwOcctDisplayMode::IntervalEndpoints)
        budget = 200'000u;
    else if (mode == IpwOcctDisplayMode::IntervalPrisms)
        budget = 25'000u;
    geometry->samplingStride = samplingStride(sourceCount, budget);
    if (mode == IpwOcctDisplayMode::DexelColumns)
        geometry->kind = IpwDisplayPrimitiveKind::Segments;
    else if (mode == IpwOcctDisplayMode::IntervalEndpoints)
        geometry->kind = IpwDisplayPrimitiveKind::Points;
    else
        geometry->kind = IpwDisplayPrimitiveKind::Prisms;

    const std::size_t reserveCount =
        std::min(sourceCount, budget);
    if (geometry->kind == IpwDisplayPrimitiveKind::Prisms)
        geometry->prisms.reserve(reserveCount);
    else
        geometry->vertices.reserve(reserveCount * 2u);

    std::size_t intervalIndex = 0u;
    for (int j = 0; j < stock.ny(); ++j) {
        if (cancelled && cancelled())
            return false;
        for (int i = 0; i < stock.nx(); ++i) {
            const auto bounds = cellBounds(i, j);
            const double uCenter = (bounds[0] + bounds[1]) * 0.5;
            const double vCenter = (bounds[2] + bounds[3]) * 0.5;
            for (const auto& interval :
                 stock.cellIntervalsView(i, j)) {
                const bool selected =
                    (intervalIndex++ % static_cast<std::size_t>(
                         geometry->samplingStride)) == 0u;
                if (!selected)
                    continue;
                if (geometry->kind ==
                        IpwDisplayPrimitiveKind::Segments ||
                    geometry->kind ==
                        IpwDisplayPrimitiveKind::Points) {
                    geometry->vertices.push_back(
                        stockPoint(
                            stock, uCenter, vCenter,
                            interval.lower));
                    geometry->vertices.push_back(
                        stockPoint(
                            stock, uCenter, vCenter,
                            interval.upper));
                } else {
                    geometry->prisms.push_back({
                        stockPoint(stock, bounds[0], bounds[2], interval.lower),
                        stockPoint(stock, bounds[1], bounds[2], interval.lower),
                        stockPoint(stock, bounds[0], bounds[3], interval.lower),
                        stockPoint(stock, bounds[1], bounds[3], interval.lower),
                        stockPoint(stock, bounds[0], bounds[2], interval.upper),
                        stockPoint(stock, bounds[1], bounds[2], interval.upper),
                        stockPoint(stock, bounds[0], bounds[3], interval.upper),
                        stockPoint(stock, bounds[1], bounds[3], interval.upper)});
                }
            }
        }
    }
    geometry->displayedPrimitiveCount =
        geometry->kind == IpwDisplayPrimitiveKind::Prisms
        ? geometry->prisms.size()
        : geometry->vertices.size() / 2u;
    output = std::move(geometry);
    return true;
}

// 按显示模式分派：表面→三角网格，BRep→快照实体，Off→空，其余→图元。
bool buildStockRepresentation(
    const toolpath::MultiDexelStock& stock,
    const IpwDisplayMeshRequest& request,
    IpwDisplayMeshResult& result,
    const std::function<bool()>& cancelled)
{
    if (isIpwSurfaceDisplayMode(request.displayMode)) {
        return stock.buildDisplayMesh(
            result.mesh,
            &result.metrics,
            &result.error,
            cancelled,
            request.meshOptions);
    }
    if (request.displayMode == IpwOcctDisplayMode::BRepSnapshot) {
        return stock.buildSnapshot(
            result.shape, nullptr, &result.error, cancelled);
    }
    if (request.displayMode == IpwOcctDisplayMode::Off)
        return true;
    return buildPrimitiveGeometry(
        stock, request.displayMode, request.sectionFraction,
        result.primitiveGeometry, cancelled);
}

} // namespace

// 构建入口（后台线程）。逐移动临时帧先从父检查点重放移动前缀得到
// 当前材料状态，再构建显示表示；全程支持取消。
void IpwDisplayMeshWorker::build(
    IpwDisplayMeshRequest request)
{
    IpwDisplayMeshResult result;
    result.generation = request.generation;
    result.requestId = request.requestId;
    result.key = std::move(request.key);
    result.displayMode = request.displayMode;

    const auto cancelled = [&request] {
        return cancellationRequested(request.cancellation);
    };
    if (cancelled()) {
        result.cancelled = true;
        emit resultReady(result);
        return;
    }

    try {
        bool built = false;
        if (request.transientSource) {
            const IpwTransientFrameSource& source =
                *request.transientSource;
            result.transientFrame = true;
            if (source.operations) {
                for (const auto& operation :
                     *source.operations) {
                    result.totalMoveCount +=
                        operation.clearing.moves.size();
                }
            }
            result.completedMoveCount =
                std::min(
                    source.completedMoveCount,
                    result.totalMoveCount);
            if (!source.parentStock ||
                !source.parentStock->valid()) {
                result.error =
                    "IPW transient frame requires a valid parent checkpoint";
            } else if (!source.operations ||
                       source.operations->empty()) {
                result.error =
                    "IPW transient frame has no pocket operations";
            } else if (source.firstMoveCount > result.completedMoveCount ||
                       !std::isfinite(source.nextMoveFraction) || source.nextMoveFraction < 0.0 ||
                       source.nextMoveFraction >= 1.0 ||
                       (source.nextMoveFraction > 0.0 && source.completedMoveCount >= result.totalMoveCount)) {
                result.error = "IPW replay requires an ordered prefix and a valid partial move";
            } else {
                std::shared_ptr<const toolpath::MultiDexelStock>
                    current = source.parentStock;
                std::size_t remaining =
                    result.completedMoveCount;
                std::size_t skip = source.firstMoveCount;
                for (std::size_t operationIndex = 0;
                     operationIndex < source.operations->size() &&
                     remaining > 0u;
                     ++operationIndex) {
                    if (cancelled())
                        break;
                    const auto& operation =
                        (*source.operations)[operationIndex];
                    const std::size_t prefixCount =
                        std::min(
                            remaining,
                            operation.clearing.moves.size());
                    const std::size_t first = std::min(skip, prefixCount);
                    skip -= first;
                    if (first == prefixCount) {
                        remaining -= prefixCount;
                        continue;
                    }
                    const bool explicitProfile =
                        source.toolProfiles.size() ==
                        source.operations->size();
                    const auto params = ipwReplayParameters(source, operationIndex);
                    const toolpath::FixedAxisToolpathView range{
                        operation.operationId, operation.outwardToolAxis,
                        std::span<const toolpath::ToolpathMove>(operation.clearing.moves).subspan(first, prefixCount - first)};
                    toolpath::MultiDexelApplyResult applied = explicitProfile
                        ? current->applyToolpathPrefix(
                              range, params,
                              source.toolProfiles[operationIndex],
                              source.sequenceIndex +
                                  static_cast<int>(operationIndex),
                              prefixCount - first, source.options, cancelled)
                        : current->applyPocketPrefix(
                              range, params,
                              source.sequenceIndex +
                                  static_cast<int>(operationIndex),
                              prefixCount - first, source.options, cancelled);
                    if (applied.record.status ==
                        toolpath::StockRemovalStatus::Cancelled) {
                        break;
                    }
                    if (!applied.stock ||
                        (applied.record.status !=
                             toolpath::StockRemovalStatus::Applied &&
                         applied.record.status !=
                             toolpath::StockRemovalStatus::
                                 NoMaterialRemoved)) {
                        result.error =
                            applied.record.message.empty()
                            ? "IPW transient material update failed"
                            : applied.record.message;
                        break;
                    }
                    result.cuttingMoveCount +=
                        applied.cuttingMoveCount;
                    result.touchedTileCount +=
                        applied.touchedTileCount;
                    result.changedCellCount +=
                        applied.changedCellCount;
                    result.materialUpdateMilliseconds +=
                        applied.updateMilliseconds;
                    current = std::move(applied.stock);
                    remaining -= prefixCount;
                }
                // Keep the fully completed prefix separate: caching a partially cut
                // stock as a full prefix would retain future material removal on rewind.
                result.completedPrefixStock = current;
                if (!cancelled() && result.error.empty() && source.nextMoveFraction > 0.0) {
                    std::size_t next = result.completedMoveCount;
                    for (std::size_t i = 0; i < source.operations->size(); ++i) {
                        const auto& operation = (*source.operations)[i];
                        if (next >= operation.clearing.moves.size()) {
                            next -= operation.clearing.moves.size();
                            continue;
                        }
                        auto move = operation.clearing.moves[next];
                        move.to = move.from.Translated(gp_Vec(move.from, move.to) * source.nextMoveFraction);
                        const toolpath::FixedAxisToolpathView partial{
                            operation.operationId, operation.outwardToolAxis,
                            std::span<const toolpath::ToolpathMove>(&move, 1)};
                        const auto params = ipwReplayParameters(source, i);
                        const auto applied = source.toolProfiles.size() == source.operations->size()
                            ? current->applyToolpathPrefix(partial, params, source.toolProfiles[i],
                                source.sequenceIndex + static_cast<int>(i), 1, source.options, cancelled)
                            : current->applyPocketPrefix(partial, params,
                                source.sequenceIndex + static_cast<int>(i), 1, source.options, cancelled);
                        if (!applied.stock ||
                            (applied.record.status != toolpath::StockRemovalStatus::Applied &&
                             applied.record.status != toolpath::StockRemovalStatus::NoMaterialRemoved)) {
                            result.error = applied.record.message.empty()
                                ? "IPW partial move failed" : applied.record.message;
                        } else {
                            current = applied.stock;
                            result.cuttingMoveCount += applied.cuttingMoveCount;
                            result.touchedTileCount += applied.touchedTileCount;
                            result.changedCellCount += applied.changedCellCount;
                            result.materialUpdateMilliseconds += applied.updateMilliseconds;
                        }
                        break;
                    }
                }
                if (!cancelled() && result.error.empty()) {
                    result.materialStock = current;
                    result.materialVolume = current->volume();
                    result.removedVolume = std::max(
                        0.0,
                        source.parentStock->volume() -
                            result.materialVolume);
                    if (request.displayMode != IpwOcctDisplayMode::Off)
                        result.materialContentFingerprint = current->contentFingerprint();
                    const auto displayStarted =
                        std::chrono::steady_clock::now();
                    built = buildStockRepresentation(
                        *current, request, result, cancelled);
                    if (!isIpwSurfaceDisplayMode(
                            request.displayMode)) {
                        result.metrics.milliseconds =
                            std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() -
                                displayStarted).count();
                    }
                }
            }
        } else if (request.stockSource) {
            result.materialStock = request.stockSource;
            const auto displayStarted =
                std::chrono::steady_clock::now();
            built = buildStockRepresentation(
                *request.stockSource, request, result, cancelled);
            if (!isIpwSurfaceDisplayMode(request.displayMode)) {
                result.metrics.milliseconds =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() -
                        displayStarted).count();
            }
        } else if (request.differenceSource) {
            if (!isIpwSurfaceDisplayMode(request.displayMode)) {
                result.error =
                    "non-surface display modes require a Multi-Dexel stock source";
            } else {
                built = request.differenceSource->buildDisplayMesh(
                    request.differenceView,
                    result.mesh,
                    &result.metrics,
                    request.operationId,
                    &result.error,
                    cancelled,
                    request.meshOptions);
            }
        } else {
            result.error =
                "IPW display mesh request has no Multi-Dexel source";
        }

        result.cancelled = cancelled();
        if (result.cancelled) {
            // 取消优先：清空几何，丢弃错误文本，只保留取消标记。
            result.mesh.Nullify();
            result.primitiveGeometry.reset();
            result.shape.Nullify();
            result.error.clear();
        } else if (!built || !result.ok()) {
            result.mesh.Nullify();
            result.primitiveGeometry.reset();
            result.shape.Nullify();
            if (result.error.empty()) {
                result.error =
                    "IPW display representation produced no geometry";
            }
        } else if (isIpwSurfaceDisplayMode(
                       result.displayMode)) {
            result.meshContentFingerprint =
                toolpath::displayMeshContentFingerprint(
                    result.mesh);
            if (result.meshContentFingerprint.empty()) {
                result.mesh.Nullify();
                result.error =
                    "IPW display mesh content fingerprint is empty";
            }
        }
    } catch (const std::exception& exception) {
        result.mesh.Nullify();
        result.primitiveGeometry.reset();
        result.shape.Nullify();
        result.error = exception.what();
    } catch (...) {
        result.mesh.Nullify();
        result.primitiveGeometry.reset();
        result.shape.Nullify();
        result.error =
            "unknown IPW display mesh generation failure";
    }
    emit resultReady(result);
}

} // namespace oap::app
