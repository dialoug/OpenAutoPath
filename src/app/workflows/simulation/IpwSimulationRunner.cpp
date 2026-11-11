#include "toolpath/core/HeavyJobScheduler.h"
#include <QCoreApplication>
// 统一 IPW 桌面运行器实现：执行 Multi-Dexel IPW，并从同一检查点链派生
// 时间线（StockSimulationResult）、余量/缺料/过切比较与逐工序过切归因。
#include "app/workflows/simulation/detail/IpwSimulationRunner.h"

#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <iterator>
#include <limits>
#include <memory>
#include <string_view>
#include <unordered_set>
#include <utility>

#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Bnd_Box.hxx>
#include <OSD.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>

#include "toolpath/pocket/simulation/PocketZMapComparison.h"
#include "toolpath/pocket/simulation/StockTargetCache.h"
#include "toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h"

namespace oap::app {
namespace {

// 特征种类 → 稳定字符串标识（用于特征余量归因与显示）。
const char* featureKindName(FeatureKind kind) noexcept
{
    switch (kind) {
        case FeatureKind::ThroughHole: return "through_hole";
        case FeatureKind::BlindHole: return "blind_hole";
        case FeatureKind::CountersinkHole: return "countersink_hole";
        case FeatureKind::Slot: return "slot";
        case FeatureKind::Pocket: return "pocket";
    }
    return "feature";
}

TopoDS_Shape holeAllowanceRegion(
    const toolpath::HoleMachiningInput& hole)
{
    return toolpath::buildHoleFeatureAllowanceRegion(
        hole, Precision::Confusion());
}

// 与 IPW 执行器保持相同的定义/运行期 ID 匹配规则：自动拆分的运行期工序
// 可以使用“定义 ID-数字”或 Surface 的“定义 ID-r数字”后缀。
bool numericRuntimeSuffix(std::string_view suffix) noexcept
{
    if (suffix.size() <= 1u || suffix.front() != '-')
        return false;
    std::size_t firstDigit = 1u;
    if (suffix.size() > 2u && suffix[1] == 'r')
        firstDigit = 2u;
    return firstDigit < suffix.size() &&
        std::all_of(
            suffix.begin() + static_cast<std::ptrdiff_t>(firstDigit),
            suffix.end(),
            [](char value) { return value >= '0' && value <= '9'; });
}

bool runtimeIdMatches(
    const std::string& definitionId,
    const std::string& runtimeId) noexcept
{
    if (runtimeId == definitionId)
        return true;
    if (runtimeId.size() <= definitionId.size() ||
        !runtimeId.starts_with(definitionId)) {
        return false;
    }
    return numericRuntimeSuffix(
        std::string_view(runtimeId).substr(definitionId.size()));
}

bool includedDefinition(
    const std::unordered_set<std::string>& included,
    const std::string& definitionId)
{
    return std::any_of(
        included.begin(), included.end(),
        [&definitionId](const std::string& operationId) {
            return runtimeIdMatches(definitionId, operationId);
        });
}

// 为槽腔/外轮廓工序构造包围盒余量区域：优先取特征边界形状的包围盒；
// 否则聚合全部匹配的 Cut/Plunge 移动，并扩展到刀具或编程轮廓偏置边界。
TopoDS_Shape pocketAllowanceRegion(
    const std::vector<toolpath::PocketToolpathOperation>& operations,
    double resolution)
{
    Bnd_Box bounds;
    double toolGuard = 0.0;
    try {
        for (const toolpath::PocketToolpathOperation& operation :
             operations) {
            if (!operation.featureBoundaryShape.IsNull()) {
                BRepBndLib::AddOptimal(
                    operation.featureBoundaryShape, bounds,
                    Standard_False, Standard_False);
            } else {
                toolGuard = std::max(
                    toolGuard, std::max(
                        operation.clearing.toolRadius,
                        std::abs(operation.clearing.firstOffset)));
                for (const toolpath::ToolpathMove& move :
                     operation.clearing.moves) {
                    if (move.type == toolpath::ToolpathMoveType::Cut ||
                        move.type == toolpath::ToolpathMoveType::Plunge) {
                        bounds.Add(move.from);
                        bounds.Add(move.to);
                    }
                }
            }
        }
        if (bounds.IsVoid())
            return {};
        Standard_Real xMin = 0.0;
        Standard_Real yMin = 0.0;
        Standard_Real zMin = 0.0;
        Standard_Real xMax = 0.0;
        Standard_Real yMax = 0.0;
        Standard_Real zMax = 0.0;
        bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        const double guard = std::max(
            Precision::Confusion(),
            toolGuard + std::max(0.0, resolution));
        return BRepPrimAPI_MakeBox(
            gp_Pnt(xMin - guard, yMin - guard, zMin - guard),
            gp_Pnt(xMax + guard, yMax + guard, zMax + guard)).Shape();
    } catch (...) {
        return {};
    }
}

// 从 Job 定义与检查点链构建逐特征余量审计范围：
// 孔特征取几何余量区域（锥形底近似边界），腔特征取包围盒区域。
std::vector<toolpath::StockFeatureAllowanceScope>
buildFeatureAllowanceScopes(
    const manufacturing::MachiningJob& job,
    double resolution)
{
    std::vector<toolpath::StockFeatureAllowanceScope> scopes;
    std::unordered_set<std::string> included;
    for (const manufacturing::ipw::IpwNode& node :
         job.ipwSession().nodes()) {
        included.insert(node.operation.id);
    }

    std::unordered_set<std::string> featureIds;
    for (const manufacturing::HoleOperationDefinition& definition :
         job.definition().holeOperations) {
        if (!definition.enabled ||
            !includedDefinition(included, definition.id))
            continue;
        for (const manufacturing::HoleTargetReference& target :
             definition.targets) {
            std::string featureId = target.featureId;
            if (featureId.empty()) {
                featureId = definition.id + ":" +
                    std::to_string(target.sourceHoleIndex);
            }
            if (!featureIds.insert(featureId).second)
                continue;
            toolpath::StockFeatureAllowanceScope scope;
            scope.featureId = std::move(featureId);
            scope.operationId = definition.id;
            scope.featureKind = featureKindName(target.hole.kind);
            scope.sourceIndex = target.sourceHoleIndex;
            scope.region = holeAllowanceRegion(target.hole);
            scope.approximateBoundary =
                target.hole.bottomProfile ==
                HoleBottomProfile::Conical;
            if (!scope.region.IsNull())
                scopes.push_back(std::move(scope));
        }
    }

    if (const toolpath::PocketToolpathBatch* batch = job.pocketResult()) {
        for (const manufacturing::PocketOperationDefinition& definition :
             job.definition().pocketOperations) {
            if (!definition.enabled ||
                !includedDefinition(included, definition.id))
                continue;
            std::vector<toolpath::PocketToolpathOperation> runtime;
            std::copy_if(
                batch->operations.begin(), batch->operations.end(),
                std::back_inserter(runtime),
                [&definition](const toolpath::PocketToolpathOperation& item) {
                    return runtimeIdMatches(
                        definition.id, item.operationId);
                });
            if (runtime.empty())
                continue;
            toolpath::StockFeatureAllowanceScope scope;
            scope.featureId = definition.target.geometryFingerprint.empty()
                ? definition.id
                : definition.target.geometryFingerprint;
            scope.operationId = definition.id;
            scope.featureKind = featureKindName(definition.target.kind);
            scope.region = pocketAllowanceRegion(runtime, resolution);
            scope.approximateBoundary = true;
            if (!scope.region.IsNull())
                scopes.push_back(std::move(scope));
        }
    }

    if (const toolpath::ProfileToolpathBatch* batch = job.profileResult()) {
        for (const manufacturing::ProfileOperationDefinition& definition :
             job.definition().profileOperations) {
            if (!definition.enabled ||
                !includedDefinition(included, definition.id))
                continue;
            std::vector<toolpath::PocketToolpathOperation>
                simulationOperations;
            int sourceIndex = -1;
            for (const toolpath::ProfileToolpathOperation& item :
                 batch->operations) {
                if (!runtimeIdMatches(
                        definition.id, item.operationId)) {
                    continue;
                }
                if (sourceIndex < 0)
                    sourceIndex = item.sourceRegionIndex;
                simulationOperations.push_back(
                    toolpath::legacyProfileSimulationOperation(item));
            }
            if (simulationOperations.empty())
                continue;
            toolpath::StockFeatureAllowanceScope scope;
            scope.featureId = definition.target.geometryFingerprint.empty()
                ? definition.id
                : definition.target.geometryFingerprint;
            scope.operationId = definition.id;
            scope.featureKind = "profile";
            scope.sourceIndex = sourceIndex;
            scope.region = pocketAllowanceRegion(
                simulationOperations, resolution);
            scope.approximateBoundary = true;
            if (!scope.region.IsNull())
                scopes.push_back(std::move(scope));
        }
    }
    if (const auto* batch = job.currentFinishingResult()) {
        for (const auto& definition : job.definition().finishingOperations) {
            if (!definition.enabled || !includedDefinition(included, definition.id))
                continue;
            Bnd_Box bounds;
            double radius = 0;
            for (const auto& operation : batch->operations) {
                if (!runtimeIdMatches(definition.id, operation.operationId))
                    continue;
                const auto& params = operation.result.effectiveParams;
                radius = std::max(radius, params.diameter * 0.5);
                const gp_Vec length = gp_Vec(operation.workFrame.Direction()) * params.cuttingLength;
                for (const auto& move : operation.result.moves) {
                    if (move.type != toolpath::ToolpathMoveType::Cut &&
                        move.type != toolpath::ToolpathMoveType::Plunge)
                        continue;
                    bounds.Add(move.from);
                    bounds.Add(move.to);
                    bounds.Add(move.from.Translated(length));
                    bounds.Add(move.to.Translated(length));
                }
            }
            if (bounds.IsVoid())
                continue;
            try {
                Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                const double guard = std::max(Precision::Confusion(), radius + resolution);
                toolpath::StockFeatureAllowanceScope scope;
                scope.featureId = definition.target.geometryFingerprint.empty()
                    ? definition.id : definition.target.geometryFingerprint;
                scope.operationId = definition.id;
                scope.featureKind = "surface_finishing";
                scope.approximateBoundary = true;
                scope.region = BRepPrimAPI_MakeBox(
                    gp_Pnt(xmin - guard, ymin - guard, zmin - guard),
                    gp_Pnt(xmax + guard, ymax + guard, zmax + guard)).Shape();
                scopes.push_back(std::move(scope));
            } catch (const Standard_Failure&) {
                // Comparison retains its overall model scope when this optional
                // approximate per-operation audit box cannot be constructed.
            }
        }
    }
    return scopes;
}

// 从诊断列表倒序取最后一个 Error 消息作为失败原因。
std::string diagnosticFailure(
    const std::vector<toolpath::ToolpathDiagnostic>& diagnostics,
    const std::string& fallback)
{
    for (auto item = diagnostics.rbegin();
         item != diagnostics.rend(); ++item) {
        if (item->severity ==
            toolpath::ToolpathDiagnosticSeverity::Error) {
            return item->message;
        }
    }
    return fallback;
}

// 把当前工序结果追加到聚合结果：重排后续步骤的 sequenceIndex、
// 用检查点实体补齐末步 stockAfter，并逐项累加全部指标。
void appendSimulation(
    toolpath::StockSimulationResult& aggregate,
    const toolpath::StockSimulationResult& current,
    const TopoDS_Shape& checkpointShape)
{
    const std::size_t firstStep = aggregate.steps.size();
    aggregate.steps.insert(
        aggregate.steps.end(),
        current.steps.begin(), current.steps.end());
    for (std::size_t index = firstStep;
         index < aggregate.steps.size(); ++index) {
        aggregate.steps[index].sequenceIndex =
            static_cast<int>(index);
    }
    if (aggregate.steps.size() > firstStep &&
        aggregate.steps.back().stockAfter.IsNull() &&
        !checkpointShape.IsNull()) {
        aggregate.steps.back().stockAfter =
            checkpointShape;
    }
    aggregate.diagnostics.insert(
        aggregate.diagnostics.end(),
        current.diagnostics.begin(),
        current.diagnostics.end());

    aggregate.maximumCurveFitError =
        std::max(aggregate.maximumCurveFitError,
                 current.maximumCurveFitError);
    aggregate.maximumChordError =
        std::max(aggregate.maximumChordError,
                 current.maximumChordError);
    aggregate.sweepBuildMilliseconds +=
        current.sweepBuildMilliseconds;
    aggregate.inputValidationMilliseconds +=
        current.inputValidationMilliseconds;
    aggregate.booleanMilliseconds +=
        current.booleanMilliseconds;
    aggregate.resultValidationMilliseconds +=
        current.resultValidationMilliseconds;
    aggregate.totalMilliseconds +=
        current.totalMilliseconds;
    aggregate.sweepSegmentCount +=
        current.sweepSegmentCount;
    aggregate.removalSolidCount +=
        current.removalSolidCount;
    aggregate.planarSweepLayerCount +=
        current.planarSweepLayerCount;
    aggregate.skippedInvalidRemovalSolidCount +=
        current.skippedInvalidRemovalSolidCount;
    aggregate.planarFootprintArea +=
        current.planarFootprintArea;
    aggregate.maximumPlanarFootprintArea =
        std::max(aggregate.maximumPlanarFootprintArea,
                 current.maximumPlanarFootprintArea);
    aggregate.entrySweepGroupCount +=
        current.entrySweepGroupCount;
    aggregate.containedEntrySweepGroupCount +=
        current.containedEntrySweepGroupCount;
    aggregate.containedEntrySweepSegmentCount +=
        current.containedEntrySweepSegmentCount;
    aggregate.fallbackSweepSegmentCount +=
        current.fallbackSweepSegmentCount;
    aggregate.linearRampMoveCount +=
        current.linearRampMoveCount;
    aggregate.helicalRampMoveCount +=
        current.helicalRampMoveCount;
    aggregate.rampSweepSliceCount +=
        current.rampSweepSliceCount;
    aggregate.maximumRampAxialError =
        std::max(aggregate.maximumRampAxialError,
                 current.maximumRampAxialError);
    aggregate.toolProfileBandCount +=
        current.toolProfileBandCount;
    aggregate.maximumToolProfileAxialError =
        std::max(aggregate.maximumToolProfileAxialError,
                 current.maximumToolProfileAxialError);
    aggregate.maximumToolProfileRadialError =
        std::max(aggregate.maximumToolProfileRadialError,
                 current.maximumToolProfileRadialError);
    aggregate.sweepInflationMilliseconds +=
        current.sweepInflationMilliseconds;
    aggregate.analyticPlanarFallbackOperationCount +=
        current.analyticPlanarFallbackOperationCount;
    aggregate.analyticPlanarPrimitiveCount +=
        current.analyticPlanarPrimitiveCount;
    aggregate.analyticPlanarArcCount +=
        current.analyticPlanarArcCount;
    aggregate.analyticPlanarBooleanGroupCount +=
        current.analyticPlanarBooleanGroupCount;
    aggregate.analyticPlanarMilliseconds +=
        current.analyticPlanarMilliseconds;
    aggregate.inferredCuttingEdgeLength =
        aggregate.inferredCuttingEdgeLength ||
        current.inferredCuttingEdgeLength;
    aggregate.effectiveCuttingEdgeLength =
        std::max(aggregate.effectiveCuttingEdgeLength,
                 current.effectiveCuttingEdgeLength);
    aggregate.booleanBatchCount +=
        current.booleanBatchCount;
    aggregate.parallelProcessingEnabled =
        aggregate.parallelProcessingEnabled ||
        current.parallelProcessingEnabled;
    aggregate.parallelWorkerCount =
        std::max(aggregate.parallelWorkerCount,
                 current.parallelWorkerCount);
    aggregate.heightFieldSimulation =
        aggregate.heightFieldSimulation ||
        current.heightFieldSimulation;
    aggregate.zMapResolution =
        current.zMapResolution > 0.0
        ? current.zMapResolution
        : aggregate.zMapResolution;
    aggregate.zMapCellCount =
        std::max(aggregate.zMapCellCount,
                 current.zMapCellCount);
    aggregate.zMapActiveCellCount =
        current.zMapActiveCellCount;
    aggregate.zMapTileCount +=
        current.zMapTileCount;
    aggregate.zMapUpdateMilliseconds +=
        current.zMapUpdateMilliseconds;
    aggregate.processedOperationCount +=
        current.processedOperationCount;
    aggregate.appliedOperationCount +=
        current.appliedOperationCount;
    aggregate.noMaterialOperationCount +=
        current.noMaterialOperationCount;
    aggregate.cancelled =
        aggregate.cancelled || current.cancelled;
}

// 从检查点链重建时间线：根检查点给出初始毛坯/体积，逐节点读取工序
// 结果追加步骤；任一步骤缺少检查点即判定时间线不完整。
bool buildCheckpointTimeline(
    const manufacturing::MachiningJob& job,
    const TopoDS_Shape& initialStock,
    toolpath::StockSimulationResult& simulation,
    std::string& error)
{
    const manufacturing::ipw::IpwSession& session =
        job.ipwSession();
    const manufacturing::ipw::IpwCheckpointStore& store =
        job.runtime().ipwCheckpoints;
    const auto root =
        store.find(session.initialStateKey());
    if (!root || !root->dexelStock) {
        error =
            QCoreApplication::translate("OpenAutoPath", "统一 IPW 根检查点缺失或没有 Multi-Dexel 材料状态").toStdString();
        return false;
    }

    simulation = {};
    simulation.initialStock = initialStock;
    simulation.initialVolume = root->volume;
    simulation.finalStock = root->stock;
    simulation.finalVolume = root->volume;
    simulation.heightFieldSimulation = true;
    simulation.zMapResolution =
        root->dexelStock->resolution();
    simulation.zMapCellCount =
        root->dexelStock->nx() *
        root->dexelStock->ny();
    simulation.zMapActiveCellCount =
        root->dexelStock->activeCellCount();

    for (const manufacturing::ipw::IpwNode& node :
         session.nodes()) {
        const auto checkpoint = store.find(node.stateKey);
        if (!checkpoint || !checkpoint->dexelStock) {
            error =
                QCoreApplication::translate("OpenAutoPath", "统一 IPW 工序检查点缺失：").toStdString() +
                node.operation.id;
            return false;
        }
        if (checkpoint->operationResult) {
            appendSimulation(
                simulation,
                *checkpoint->operationResult,
                checkpoint->stock);
        }
        simulation.finalStock = checkpoint->stock;
        simulation.finalVolume = checkpoint->volume;
        simulation.zMapResolution =
            checkpoint->dexelStock->resolution();
        simulation.zMapCellCount =
            checkpoint->dexelStock->nx() *
            checkpoint->dexelStock->ny();
        simulation.zMapActiveCellCount =
            checkpoint->dexelStock->activeCellCount();
    }
    simulation.removedVolume = std::max(
        0.0,
        simulation.initialVolume -
            simulation.finalVolume);
    simulation.complete = session.complete();
    if (!simulation.complete) {
        error =
            QCoreApplication::translate("OpenAutoPath", "统一 IPW 检查点链未完整执行").toStdString();
        return false;
    }
    return true;
}

// 比较两个 Multi-Dexel 网格的栅格定义（尺寸/分辨率/原点/轴向）
// 是否一致，是后续逐工序 before/after 对比的前提。
bool sameGrid(
    const toolpath::MultiDexelStock& left,
    const toolpath::MultiDexelStock& right,
    double tolerance)
{
    return left.valid() && right.valid() &&
           left.nx() == right.nx() &&
           left.ny() == right.ny() &&
           std::abs(left.resolution() -
                    right.resolution()) <= tolerance &&
           std::abs(left.uMin() - right.uMin()) <= tolerance &&
           std::abs(left.uMax() - right.uMax()) <= tolerance &&
           std::abs(left.vMin() - right.vMin()) <= tolerance &&
           std::abs(left.vMax() - right.vMax()) <= tolerance &&
           gp_Vec(left.origin(), right.origin()).Magnitude() <=
               tolerance &&
           left.uAxis().IsEqual(
               right.uAxis(), tolerance, tolerance) &&
           left.vAxis().IsEqual(
               right.vAxis(), tolerance, tolerance) &&
           left.wAxis().IsEqual(
               right.wAxis(), tolerance, tolerance);
}

// 把检查点链汇总为原生 Multi-Dexel 比较数据：
// 所有检查点必须与初始网格同栅格，否则无法建立逐工序前后状态。
std::shared_ptr<const toolpath::MultiDexelComparisonData>
buildMultiDexelComparisonData(
    const manufacturing::MachiningJob& job,
    const toolpath::ToolpathGenerationControl* control)
{
    const manufacturing::ipw::IpwSession& session =
        job.ipwSession();
    const manufacturing::ipw::IpwCheckpointStore& store =
        job.runtime().ipwCheckpoints;
    const auto root =
        store.find(session.initialStateKey());
    if (!root || !root->dexelStock ||
        !root->dexelStock->valid()) {
        return {};
    }
    const double tolerance = std::max(
        1.0e-8,
        1.0e-9 * root->dexelStock->resolution());
    auto data = std::make_shared<
        toolpath::MultiDexelComparisonData>();
    data->initial = root->dexelStock;
    auto previous = root;
    data->operations.reserve(session.nodes().size());
    for (const manufacturing::ipw::IpwNode& node :
         session.nodes()) {
        if (control && control->cancelled())
            return {};
        const auto current =
            store.find(node.stateKey);
        if (!current || !current->dexelStock ||
            !sameGrid(
                *data->initial,
                *current->dexelStock,
                tolerance)) {
            return {};
        }
        toolpath::MultiDexelOperationState operation;
        operation.operationId = node.operation.id;
        operation.sequenceIndex =
            node.operation.sequence;
        operation.before = previous->dexelStock;
        operation.after = current->dexelStock;
        data->operations.push_back(
            std::move(operation));
        previous = current;
    }
    data->final = previous->dexelStock;
    data->featureAllowanceScopes =
        buildFeatureAllowanceScopes(
            job, data->initial->resolution());
    return data->valid() ? data : nullptr;
}

// PocketZMapComparison 表示每列一个固定底面和一个可下降顶面。统一
// Multi-Dexel 是更一般的多区间模型，因此这里只在整条检查点链能够无损
// 降级为单区间高度场时建立视图。减料后完全清空的柱以负无穷顶面表示；
// 真正的多区间柱（例如悬空材料）由 MultiDexelComparison 原生审计。
std::shared_ptr<const toolpath::PocketZMapSimulationData>
buildSingleIntervalZMapView(
    const manufacturing::MachiningJob& job,
    const toolpath::ToolpathGenerationControl* control)
{
    const manufacturing::ipw::IpwSession& session =
        job.ipwSession();
    const manufacturing::ipw::IpwCheckpointStore& store =
        job.runtime().ipwCheckpoints;
    const auto root = store.find(session.initialStateKey());
    if (!root || !root->dexelStock ||
        !root->dexelStock->valid()) {
        return {};
    }

    const toolpath::MultiDexelStock& initial =
        *root->dexelStock;
    const double tolerance = std::max(
        1.0e-8, 1.0e-9 * initial.resolution());
    auto data =
        std::make_shared<toolpath::PocketZMapSimulationData>();
    data->gridSource = root->dexelStock;
    data->origin = initial.origin();
    data->uAxis = initial.uAxis();
    data->vAxis = initial.vAxis();
    data->wAxis = initial.wAxis();
    data->uMin = initial.uMin();
    data->uMax = initial.uMax();
    data->vMin = initial.vMin();
    data->vMax = initial.vMax();
    data->resolution = initial.resolution();
    data->nx = initial.nx();
    data->ny = initial.ny();
    data->initialVolume = initial.volume();

    const std::size_t cellCount =
        static_cast<std::size_t>(data->nx) *
        static_cast<std::size_t>(data->ny);
    const double noBottom =
        std::numeric_limits<double>::infinity();
    const double noTop =
        -std::numeric_limits<double>::infinity();
    data->bottoms.assign(cellCount, noBottom);
    data->initialTops.assign(cellCount, noTop);
    data->finalTops.assign(cellCount, noTop);

    for (int j = 0; j < data->ny; ++j) {
        if (control && control->cancelled())
            return {};
        for (int i = 0; i < data->nx; ++i) {
            const auto intervals =
                initial.cellIntervalsView(i, j);
            if (intervals.size() > 1u)
                return {};
            if (intervals.empty())
                continue;
            const std::size_t cell =
                static_cast<std::size_t>(j) *
                    static_cast<std::size_t>(data->nx) +
                static_cast<std::size_t>(i);
            data->bottoms[cell] = intervals.front().lower;
            data->initialTops[cell] =
                intervals.front().upper;
            data->finalTops[cell] =
                intervals.front().upper;
        }
    }

    std::shared_ptr<const manufacturing::ipw::IpwStockCheckpoint>
        previous = root;
    data->operationDeltas.reserve(session.nodes().size());
    for (const manufacturing::ipw::IpwNode& node :
         session.nodes()) {
        if (control && control->cancelled())
            return {};
        const auto current = store.find(node.stateKey);
        if (!current || !current->dexelStock ||
            !sameGrid(
                initial, *current->dexelStock,
                tolerance)) {
            return {};
        }

        toolpath::PocketZMapOperationDelta delta;
        delta.operationId = node.operation.id;
        delta.sequenceIndex = node.operation.sequence;
        for (int j = 0; j < data->ny; ++j) {
            if (control && control->cancelled())
                return {};
            for (int i = 0; i < data->nx; ++i) {
                const auto before =
                    previous->dexelStock->cellIntervalsView(
                        i, j);
                const auto after =
                    current->dexelStock->cellIntervalsView(
                        i, j);
                if (before.size() > 1u ||
                    after.size() > 1u) {
                    return {};
                }
                if (before.empty()) {
                    if (!after.empty())
                        return {};
                    continue;
                }
                const double beforeTop =
                    before.front().upper;
                double afterTop = noTop;
                if (!after.empty()) {
                    if (std::abs(
                            before.front().lower -
                            after.front().lower) > tolerance) {
                        return {};
                    }
                    afterTop = after.front().upper;
                    if (afterTop - beforeTop > tolerance)
                        return {};
                    if (beforeTop - afterTop <= tolerance)
                        continue;
                }
                const std::size_t cell =
                    static_cast<std::size_t>(j) *
                        static_cast<std::size_t>(
                            data->nx) +
                    static_cast<std::size_t>(i);
                data->finalTops[cell] = afterTop;
                toolpath::PocketZMapCellDelta change;
                change.cellIndex =
                    static_cast<int>(cell);
                change.beforeTop = beforeTop;
                change.afterTop = afterTop;
                delta.cells.push_back(change);
            }
        }
        data->operationDeltas.push_back(
            std::move(delta));
        previous = current;
    }
    data->finalVolume = previous->dexelStock->volume();
    data->featureAllowanceScopes =
        buildFeatureAllowanceScopes(job, initial.resolution());
    return data->valid() ? data : nullptr;
}

} // namespace

// 无 UI 同步入口：执行 IPW → 重建时间线 → Z-map/原生比较 → 过切归因。
// control 提供取消回调；request.targetShape 为空时跳过比较与归因。
IpwSimulationRunResult runIpwSimulation(
    manufacturing::MachiningJob& job,
    IpwSimulationRequest request,
    const toolpath::ToolpathGenerationControl* control)
{
    toolpath::ToolpathParallelScope parallelScope(control);
    IpwSimulationRunResult result;
    if (request.initialStock.IsNull()) {
        result.errorMessage =
            QCoreApplication::translate("OpenAutoPath", "统一 IPW 需要非空初始毛坯").toStdString();
        return result;
    }
    if (request.initialStockFingerprint.empty()) {
        result.errorMessage =
            QCoreApplication::translate("OpenAutoPath", "统一 IPW 需要稳定的初始毛坯指纹").toStdString();
        return result;
    }

    result.execution =
        manufacturing::ipw::IpwExecutor().execute(
            job, request.initialStock,
            std::move(request.initialStockFingerprint),
            manufacturing::ipw::kHybridIpwEngineId,
            request.options, control);
    result.cancelled =
        result.execution.status ==
        manufacturing::ipw::IpwExecutionStatus::Cancelled;
    if (!result.execution.complete()) {
        result.errorMessage = result.execution.message;
        result.failed = !result.cancelled;
        return result;
    }

    // 派生视图顺序：先时间线，再 Z-map 视图；Z-map 降级失败时
    // 回退到原生 Multi-Dexel 比较数据。
    if (!buildCheckpointTimeline(
            job, request.initialStock,
            result.simulation,
            result.errorMessage)) {
        result.failed = true;
        return result;
    }
    result.simulation.pocketZMapData =
        buildSingleIntervalZMapView(job, control);
    std::shared_ptr<const toolpath::MultiDexelComparisonData>
        multiDexelData;
    if (!result.simulation.pocketZMapData) {
        multiDexelData =
            buildMultiDexelComparisonData(job, control);
    }
    if (control && control->cancelled()) {
        result.cancelled = true;
        return result;
    }
    if (request.targetShape.IsNull())
        return result;

    auto& targetCache = job.runtime().stockTargetCache;
    const auto& part = job.definition().part;
    const std::string targetRevision = part.sourceFingerprint + "|" + part.units;
    if (!targetCache || !targetCache->matches(request.targetShape, targetRevision)) {
        targetCache = std::make_shared<toolpath::StockTargetCache>(
            request.targetShape, targetRevision, request.exactTargetBounds);
    }
    request.comparisonOptions.targetCache = targetCache;

    if (result.simulation.pocketZMapData) {
        result.comparison =
            toolpath::PocketZMapComparison().compare(
                result.simulation.pocketZMapData,
                result.simulation.initialStock,
                request.targetShape,
                request.comparisonOptions,
                control);
    } else if (multiDexelData) {
        result.comparison =
            toolpath::MultiDexelComparison().compare(
                multiDexelData,
                result.simulation.initialStock,
                request.targetShape,
                request.comparisonOptions,
                control);
    } else {
        result.failed = true;
        result.errorMessage =
            QCoreApplication::translate("OpenAutoPath", "统一 IPW 无法建立原生 Multi-Dexel 审计时间线").toStdString();
        return result;
    }
    result.cancelled = result.comparison.cancelled;
    if (result.cancelled)
        return result;
    if (!result.comparison.complete) {
        result.failed = true;
        result.errorMessage = diagnosticFailure(
            result.comparison.diagnostics,
            QCoreApplication::translate("OpenAutoPath", "统一 IPW 余量、缺料和过切检查失败").toStdString());
        return result;
    }

    const double attributionTolerance = std::max(
        1.0e-8,
        1.0e-9 * result.comparison.targetVolume);
    // 过切体积低于数值容差时跳过逐工序归因，直接标记完成。
    if (result.comparison.overcutVolume <=
        attributionTolerance) {
        result.overcutAttribution.complete = true;
        return result;
    }
    if (result.simulation.pocketZMapData) {
        result.overcutAttribution =
            toolpath::PocketZMapComparison()
                .attributeOvercut(
                    result.simulation.pocketZMapData,
                    request.targetShape,
                    request.comparisonOptions,
                    control,
                    result.comparison
                        .heightFieldDisplaySource);
    } else if (multiDexelData) {
        result.overcutAttribution =
            toolpath::MultiDexelComparison()
                .attributeOvercut(
                    multiDexelData,
                    request.targetShape,
                    request.comparisonOptions,
                    control,
                    result.comparison
                        .heightFieldDisplaySource);
    }
    result.cancelled =
        result.overcutAttribution.cancelled;
    if (!result.cancelled &&
        !result.overcutAttribution.complete) {
        result.failed = true;
        result.errorMessage = diagnosticFailure(
            result.overcutAttribution.diagnostics,
            QCoreApplication::translate("OpenAutoPath", "统一 IPW 逐工序过切归因失败").toStdString());
    }
    return result;
}

// 交互式入口：后台线程执行 runIpwSimulation，通过 WorkflowProgressPresenter
// 显示不确定进度并支持取消；任务异常统一转换为 failed 结果。
IpwSimulationRunResult simulateIpwWithProgress(
    const WorkflowProgressPresenter& progress,
    manufacturing::MachiningJob& job,
    IpwSimulationRequest request,
    std::shared_ptr<std::atomic_bool> cancellation)
{
    if (!cancellation) {
        cancellation =
            std::make_shared<std::atomic_bool>(false);
    }
    auto future = toolpath::submitHeavyJob(
        cancellation,
        [&job, request = std::move(request),
         cancellation]() mutable {
          try {
            OCC_CATCH_SIGNALS
            OSD::SetThreadLocalSignal(
                OSD_SignalMode_Set, Standard_False);
            toolpath::ToolpathGenerationControl control;
            toolpath::attachGlobalToolpathParallelBudget(control);
            control.isCancellationRequested =
                [cancellation] {
                    return cancellation->load(
                        std::memory_order_relaxed);
                };
            return runIpwSimulation(
                job, std::move(request), &control);
          } catch (const Standard_Failure& failure) {
            IpwSimulationRunResult result;
            result.failed = true;
            const char* detail =
                failure.GetMessageString();
            result.errorMessage =
                QCoreApplication::translate("OpenAutoPath", "OCCT 执行统一 IPW 时发生异常").toStdString();
            if (detail && *detail) {
                result.errorMessage +=
                    std::string("：") + detail;
            }
            return result;
          } catch (const std::exception& failure) {
            IpwSimulationRunResult result;
            result.failed = true;
            result.errorMessage =
                std::string(QCoreApplication::translate("OpenAutoPath", "统一 IPW 执行失败：").toStdString()) +
                failure.what();
            return result;
          } catch (...) {
            IpwSimulationRunResult result;
            result.failed = true;
            result.errorMessage =
                QCoreApplication::translate("OpenAutoPath", "统一 IPW 后台任务发生未知异常").toStdString();
            return result;
          }
        }, [](toolpath::HeavyJobRejection reason) {
            IpwSimulationRunResult result;
            result.cancelled = reason == toolpath::HeavyJobRejection::Cancelled;
            result.failed = !result.cancelled;
            if (result.failed)
                result.errorMessage = toolpath::heavyJobRejectionMessage(reason);
            return result;
        });
    // The synchronous API exclusively borrows job until this guard has joined.
    // Pending cancellation never touches the job or commits an IPW checkpoint.
    toolpath::HeavyJobWaitGuard waitGuard(future, cancellation);

    WorkflowProgressRequest progressRequest;
    progressRequest.title =
        QCoreApplication::translate("OpenAutoPath", "统一 IPW 腔加工仿真");
    progressRequest.label = QCoreApplication::translate("OpenAutoPath", "正在执行统一 Multi-Dexel IPW，并从检查点派生余量结果…");
    progressRequest.cancellingLabel = QCoreApplication::translate("OpenAutoPath", "正在取消，等待当前 Dexel/OCCT 批次安全结束…");
    progressRequest.indeterminate = true;
    progressRequest.queued = [cancellation] { return toolpath::heavyJobScheduler().isPending(cancellation); };
    progressRequest.ready = [&future] {
        return future.wait_for(std::chrono::seconds(0)) ==
               std::future_status::ready;
    };
    progressRequest.cancel = [cancellation] {
        toolpath::cancelHeavyJob(cancellation);
    };
    progressRequest.cancellationRequested = [cancellation] {
        return cancellation->load(std::memory_order_relaxed);
    };
    waitForWorkflowTask(progress, progressRequest);

    try {
        return future.get();
    } catch (...) {
        IpwSimulationRunResult result;
        result.failed = true;
        result.errorMessage =
            QCoreApplication::translate("OpenAutoPath", "统一 IPW 后台任务无法返回结果").toStdString();
        return result;
    }
}

} // namespace oap::app
