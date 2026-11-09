// IpwExecutor 的实现:统一 IPW 顺序执行器。流程见 execute():含孔/槽腔的
// 运行时分派、检查点复用、根检查点重建与节点状态推进。属于 oap_manufacturing::ipw。
#include "manufacturing/ipw/IpwExecutor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string_view>
#include "toolpath/core/ToolpathParallel.h"
#include <unordered_set>
#include <utility>

#include <QByteArray>
#include <QCryptographicHash>
#include <Standard_Failure.hxx>

#include "manufacturing/job/MachiningJob.h"
#include "manufacturing/ipw/HoleMultiDexelUpdater.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/core/PocketToolBinding.h"
#include "toolpath/pocket/simulation/fixedaxis/FixedAxisBRepStockSimulator.h"
#include "toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h"
#include "toolpath/profile/simulation/ProfileSimulationAdapter.h"

namespace oap::manufacturing::ipw {
namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedMilliseconds(
    const SteadyClock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               SteadyClock::now() - start)
        .count();
}

// 各后端标识:根检查点、孔原生 Dexel、槽腔 Dexel、槽腔 BRep 换轴过渡。
constexpr std::string_view kInitialBackend =
    "multi-dexel-initial-v2";
constexpr std::string_view kHoleDexelBackend =
    "tiled-multi-dexel-hole-v2-oriented-profile";
constexpr std::string_view kPocketDexelBackend =
    "tiled-multi-dexel-pocket-v4-semantic-primitives";
constexpr std::string_view kPocketTransitionBackend =
    "occt-pocket-transition+multi-dexel-v4-adaptive-helical";
constexpr std::string_view kProfileDexelBackend =
    "tiled-multi-dexel-profile-v1-semantic-primitives";
constexpr std::string_view kProfileTransitionBackend =
    "occt-profile-transition+multi-dexel-v1";
constexpr std::string_view kFinishingDexelBackend =
    "tiled-multi-dexel-finishing-v1-rounded-oriented";

// 执行帧:描述首个工序的刀具轴与参考刀具直径,用于初始毛坯光栅化与引擎指纹。
struct ExecutionFrame {
    gp_Vec outwardAxis{0.0, 0.0, 1.0};
    double referenceCutterDiameter = 1.0;
};

// 每个已启用工序的候选执行帧(最终取序列最小者作为全局执行帧)。
struct ExecutionFrameCandidate {
    std::string operationId;
    int sequence = 0;
    ExecutionFrame frame;
};

// 单节点仿真结果:仿真明细、更新后的 Dexel 材料真值、后端标识与错误。
struct NodeSimulation {
    std::shared_ptr<const toolpath::StockSimulationResult>
        result;
    std::shared_ptr<const toolpath::MultiDexelStock>
        dexelStock;
    std::string backend;
    std::string error;
};

std::string hashText(const std::string& value)
{
    return QCryptographicHash::hash(
               QByteArray::fromStdString(value),
               QCryptographicHash::Sha256)
        .toHex()
        .toStdString();
}

void appendSimulationOptions(
    std::ostringstream& output,
    const toolpath::StockSimulationOptions& options)
{
    output << static_cast<int>(options.method) << '|'
           << options.geometricTolerance << '|'
           << options.arcChordTolerance << '|'
           << options.rampAxialTolerance << '|'
           << options.toolProfileAxialTolerance << '|'
           << options.toolProfileRadialTolerance << '|'
           << options.maxSweepSegments << '|'
           << options.maxRampSweepSlices << '|'
           << options.maxRampContainmentChecks << '|'
           << options.maxToolProfileBands << '|'
           << options.maxOrientedToolFacets << '|'
           << options.stopOnFailure << '|'
           << options.runParallel << '|'
           << options.zMapResolution << '|'
           << options.zMapMaxCellCount << '|'
           << options.zMapTileSize;
}

// 计算"有效引擎指纹":引擎 ID、执行帧与各工序族仿真参数
// 全部纳入,任何参数变化都会使缓存前缀失效。
std::string effectiveEngineFingerprint(
    const std::string& engineId,
    const IpwExecutionOptions& options,
    const ExecutionFrame& frame)
{
    std::ostringstream input;
    input.precision(17);
    input << "oap-ipw-engine-options-v12-surface-finishing|"
          << engineId << '|'
          << frame.outwardAxis.X() << '|'
          << frame.outwardAxis.Y() << '|'
          << frame.outwardAxis.Z() << '|'
          << frame.referenceCutterDiameter << '|'
          << options.safeMixedTopologyFallback << '|';
    appendSimulationOptions(input, options.hole);
    input << '|';
    appendSimulationOptions(input, options.pocket);
    input << '|';
    return hashText(input.str());
}

// 检查点内容指纹:stateKey + 毛坯自身内容指纹的哈希,用于检测同 key 内容变化。
std::string checkpointContentFingerprint(
    const std::string& stateKey,
    const toolpath::MultiDexelStock& stock)
{
    std::string input;
    input.reserve(
        stateKey.size() +
        stock.contentFingerprint().size() + 32u);
    input =
        "oap-ipw-checkpoint-content-v2|";
    input += stateKey;
    input += '|';
    input += stock.contentFingerprint();
    return hashText(input);
}

std::vector<toolpath::HoleMachiningOperation>
holeOperationsForNode(
    const MachiningJob& job,
    const IpwNode& node)
{
    std::vector<toolpath::HoleMachiningOperation> operations;
    const toolpath::HoleToolpathBatch* batch =
        job.holeResult();
    if (!batch)
        return operations;
    for (const toolpath::HoleMachiningOperation& operation :
         batch->machining.operations) {
        if (runtimeOperationIdMatches(
                node.operation.id,
                operation.operationId)) {
            operations.push_back(operation);
        }
    }
    return operations;
}

// 取出 Job 运行期结果中属于该节点的全部槽腔工序(按定义 ID 前缀匹配)。
std::vector<toolpath::PocketToolpathOperation>
pocketOperationsForNode(
    const MachiningJob& job,
    const IpwNode& node)
{
    std::vector<toolpath::PocketToolpathOperation> operations;
    const toolpath::PocketToolpathBatch* batch =
        job.pocketResult();
    if (!batch)
        return operations;
    for (const toolpath::PocketToolpathOperation& operation :
         batch->operations) {
        if (runtimeOperationIdMatches(
                node.operation.id,
                operation.operationId)) {
            operations.push_back(operation);
        }
    }
    return operations;
}

std::vector<toolpath::ProfileToolpathOperation>
profileOperationsForNode(
    const MachiningJob& job,
    const IpwNode& node)
{
    std::vector<toolpath::ProfileToolpathOperation> operations;
    const toolpath::ProfileToolpathBatch* batch =
        job.profileResult();
    if (!batch)
        return operations;
    for (const toolpath::ProfileToolpathOperation& operation :
         batch->operations) {
        if (runtimeOperationIdMatches(
                node.operation.id,
                operation.operationId)) {
            operations.push_back(operation);
        }
    }
    return operations;
}

std::vector<toolpath::SurfaceFinishingOperation>
finishingOperationsForNode(const MachiningJob& job, const IpwNode& node)
{
    std::vector<toolpath::SurfaceFinishingOperation> operations;
    const auto* batch = job.currentFinishingResult();
    if (!batch)
        return operations;
    for (const auto& operation : batch->operations)
        if (runtimeOperationIdMatches(node.operation.id, operation.operationId))
            operations.push_back(operation);
    return operations;
}

gp_Vec normalizedOrDefault(gp_Vec axis)
{
    if (!std::isfinite(axis.X()) ||
        !std::isfinite(axis.Y()) ||
        !std::isfinite(axis.Z()) ||
        axis.Magnitude() <= 1.0e-9) {
        return {0.0, 0.0, 1.0};
    }
    axis.Normalize();
    return axis;
}

gp_Vec pocketAxis(
    const toolpath::PocketToolpathOperation& operation)
{
    gp_Vec axis = operation.outwardToolAxis;
    if (axis.Magnitude() > 1.0e-9)
        return normalizedOrDefault(axis);
    for (const toolpath::ToolpathMove& move :
         operation.clearing.moves) {
        if (move.type !=
            toolpath::ToolpathMoveType::Plunge) {
            continue;
        }
        axis = gp_Vec(move.to, move.from);
        if (axis.Magnitude() > 1.0e-9)
            return normalizedOrDefault(axis);
    }
    return {0.0, 0.0, 1.0};
}

gp_Vec holeAxis(
    const toolpath::HoleMachiningOperation& operation)
{
    return normalizedOrDefault(
        gp_Vec(operation.bottom, operation.entry));
}

double positiveDiameter(
    double operationDiameter,
    double fallbackDiameter)
{
    if (std::isfinite(operationDiameter) &&
        operationDiameter > 0.0) {
        return operationDiameter;
    }
    if (std::isfinite(fallbackDiameter) &&
        fallbackDiameter > 0.0) {
        return fallbackDiameter;
    }
    return 1.0;
}

// 收集全部已启用(且已生成运行期结果)工序的执行帧候选;includedOperationIds
// 非空时仅收集子集内的工序。
std::vector<ExecutionFrameCandidate>
executionFrameCandidates(
    const MachiningJob& job,
    const std::vector<std::string>&
        includedOperationIds)
{
    const std::unordered_set<std::string> included(
        includedOperationIds.begin(),
        includedOperationIds.end());
    const auto selected = [&included](
                              const std::string& id) {
        return included.empty() ||
               included.contains(id);
    };
    std::vector<ExecutionFrameCandidate> candidates;
    for (const PocketOperationDefinition& definition :
         job.definition().pocketOperations) {
        if (!definition.enabled ||
            !selected(definition.id)) {
            continue;
        }
        IpwOperationDefinition operation;
        operation.id = definition.id;
        IpwNode node;
        node.operation = std::move(operation);
        const auto runtime =
            pocketOperationsForNode(job, node);
        if (runtime.empty())
            continue;
        const toolpath::PocketToolpathBatch* batch =
            job.pocketResult();
        candidates.push_back({
            definition.id,
            definition.sequence,
            {pocketAxis(runtime.front()),
             positiveDiameter(
                 runtime.front().tool.valid
                     ? runtime.front().tool.diameter
                     : 0.0,
                 batch
                     ? batch->effectiveParams.tool.diameter
                     : 0.0)}});
    }
    for (const HoleOperationDefinition& definition :
         job.definition().holeOperations) {
        if (!definition.enabled ||
            !selected(definition.id)) {
            continue;
        }
        IpwOperationDefinition operation;
        operation.id = definition.id;
        IpwNode node;
        node.operation = std::move(operation);
        const auto runtime =
            holeOperationsForNode(job, node);
        if (runtime.empty())
            continue;
        const toolpath::HoleToolpathBatch* batch =
            job.holeResult();
        const double operationDiameter =
            runtime.front().tool.valid
            ? runtime.front().tool.diameter
            : 0.0;
        candidates.push_back({
            definition.id,
            definition.sequence,
            {holeAxis(runtime.front()),
             positiveDiameter(
                 operationDiameter,
                 batch
                     ? batch->effectiveParams.tool.diameter
                     : 0.0)}});
    }
    for (const ProfileOperationDefinition& definition :
         job.definition().profileOperations) {
        if (!definition.enabled ||
            !selected(definition.id)) {
            continue;
        }
        IpwOperationDefinition operation;
        operation.id = definition.id;
        IpwNode node;
        node.operation = std::move(operation);
        const auto runtime =
            profileOperationsForNode(job, node);
        if (runtime.empty())
            continue;
        const toolpath::ProfileToolpathBatch* batch =
            job.profileResult();
        candidates.push_back({
            definition.id,
            definition.sequence,
            {normalizedOrDefault(
                 runtime.front().outwardToolAxis),
             positiveDiameter(
                 runtime.front().tool.valid
                     ? runtime.front().tool.diameter
                     : 0.0,
                 batch
                     ? batch->effectiveParams.tool.diameter
                     : 0.0)}});
    }
    for (const auto& definition : job.definition().finishingOperations) {
        if (!definition.enabled || !selected(definition.id))
            continue;
        IpwNode node;
        node.operation.id = definition.id;
        const auto runtime = finishingOperationsForNode(job, node);
        if (runtime.empty())
            continue;
        candidates.push_back({definition.id, definition.sequence,
            {gp_Vec(runtime.front().workFrame.Direction()),
             positiveDiameter(runtime.front().tool.diameter,
                 runtime.front().result.effectiveParams.diameter)}});
    }
    return candidates;
}

const ExecutionFrameCandidate* firstExecutionFrame(
    const std::vector<ExecutionFrameCandidate>&
        candidates)
{
    if (candidates.empty())
        return nullptr;
    return &*std::min_element(
        candidates.begin(), candidates.end(),
        [](const ExecutionFrameCandidate& left,
           const ExecutionFrameCandidate& right) {
            return left.sequence < right.sequence;
        });
}

std::string axisText(const gp_Vec& axis)
{
    std::ostringstream output;
    output.precision(8);
    output << '('
           << axis.X() << ','
           << axis.Y() << ','
           << axis.Z() << ')';
    return output.str();
}

std::string simulationFailureMessage(
    const toolpath::StockSimulationResult& result)
{
    for (auto diagnostic = result.diagnostics.rbegin();
         diagnostic != result.diagnostics.rend();
         ++diagnostic) {
        if (diagnostic->severity ==
            toolpath::ToolpathDiagnosticSeverity::Error) {
            return diagnostic->message;
        }
    }
    for (auto record = result.steps.rbegin();
         record != result.steps.rend();
         ++record) {
        if (!record->message.empty())
            return record->message;
    }
    return "IPW operation simulation did not complete";
}

toolpath::ToolpathGenerationControl localControl(
    const toolpath::ToolpathGenerationControl* control)
{
    toolpath::ToolpathGenerationControl result;
    if (!control)
        return result;
    result.isCancellationRequested =
        control->isCancellationRequested;
    result.reportStatus = control->reportStatus;
    result.pocketMaterialSectionCache =
        control->pocketMaterialSectionCache;
    result.tryAcquireParallelWorkers = control->tryAcquireParallelWorkers;
    result.releaseParallelWorkers = control->releaseParallelWorkers;
    return result;
}

bool derivedSnapshot(
    const IpwStockCheckpoint& checkpoint,
    TopoDS_Shape& shape,
    std::string& error,
    const toolpath::ToolpathGenerationControl* control)
{
    if (!checkpoint.stock.IsNull()) {
        shape = checkpoint.stock;
        return true;
    }
    if (!checkpoint.dexelStock) {
        error =
            "IPW checkpoint has neither Dexel state nor a derived shape";
        return false;
    }
    std::function<bool()> cancelled;
    if (control)
        cancelled = control->isCancellationRequested;
    return checkpoint.dexelStock->buildSnapshot(
        shape, nullptr, &error, std::move(cancelled));
}

// 把 BRep 过渡结果重新光栅化为 Multi-Dexel:轴与父网格对齐时沿用父网格
// 分辨率,否则按刀具直径新建网格。
std::shared_ptr<const toolpath::MultiDexelStock>
rasterizeTransitionResult(
    const TopoDS_Shape& shape,
    const gp_Vec& axis,
    double cutterDiameter,
    const std::shared_ptr<const toolpath::MultiDexelStock>&
        parentDexel,
    const IpwExecutionOptions& options,
    const toolpath::ToolpathGenerationControl* control,
    std::string& error)
{
    std::function<bool()> cancelled;
    if (control)
        cancelled = control->isCancellationRequested;
    if (parentDexel && parentDexel->axisAligned(axis)) {
        return toolpath::MultiDexelStock::fromShapeOnGrid(
            shape, *parentDexel, options.pocket,
            nullptr, &error, std::move(cancelled));
    }
    return toolpath::MultiDexelStock::fromShape(
        shape, axis, cutterDiameter,
        options.pocket, nullptr, &error,
        std::move(cancelled));
}

// 槽腔 Dexel 直算:沿当前网格刀轴顺序应用槽腔工序,不经过 BRep;
// 同轴后续节点只消费 Multi-Dexel 真值。
toolpath::FixedAxisToolpathView executionView(
    const toolpath::PocketToolpathOperation& operation)
{
    return {operation.operationId, operation.outwardToolAxis,
            operation.clearing.moves};
}

toolpath::FixedAxisToolpathView executionView(
    const toolpath::ProfileToolpathOperation& operation)
{
    return toolpath::profileSimulationView(operation);
}

template <typename Operation>
toolpath::MultiDexelApplyResult applyExecutionOperation(
    const toolpath::MultiDexelStock& stock, const Operation& operation,
    const toolpath::ToolpathParams& params, int sequence,
    const toolpath::StockSimulationOptions& options,
    std::function<bool()> cancelled)
{
    return stock.applyFixedAxis(executionView(operation),
        toolpath::bindPocketToolParameters(params, operation.tool),
        sequence, options, std::move(cancelled));
}

toolpath::MultiDexelApplyResult applyExecutionOperation(
    const toolpath::MultiDexelStock& stock,
    const toolpath::SurfaceFinishingOperation& operation,
    const toolpath::ToolpathParams&, int sequence,
    const toolpath::StockSimulationOptions& options,
    std::function<bool()> cancelled)
{
    const auto params = toolpath::bindPocketToolParameters(
        operation.result.toolpathParams(), operation.tool);
    const gp_Vec axis(operation.workFrame.Direction());
    const toolpath::FixedAxisToolpathView view{
        operation.operationId, axis, operation.result.moves};
    toolpath::DexelToolProfileDefinition profile;
    profile.diameter = params.tool.diameter;
    profile.cornerRadius = params.tool.cornerRadius;
    profile.cuttingLength = params.tool.cuttingEdgeLength;
    // Retain the actual rounded cutting edge. A non-Z setup sweeps against the
    // existing parent grid; it does not rebuild stock or impersonate a pocket.
    if (stock.axisAligned(axis))
        return stock.applyAxialToolpath(view, params, profile, sequence,
            options, std::move(cancelled));
    return stock.applyOrientedToolpath(view, params, profile, sequence,
        options, std::move(cancelled));
}

template <typename Operation>
NodeSimulation simulateFixedAxisDexel(
    const IpwStockCheckpoint& parent,
    const std::vector<Operation>& operations,
    const toolpath::ToolpathParams& params,
    const IpwExecutionOptions& options,
    const toolpath::ToolpathGenerationControl* control)
{
    NodeSimulation output;
    output.backend =
        std::string(kPocketDexelBackend);
    if (!parent.dexelStock ||
        !parent.dexelStock->valid()) {
        output.error =
            "IPW fixed-axis node requires a valid parent multi-dexel state";
        return output;
    }
    const SteadyClock::time_point simulationStart =
        SteadyClock::now();
    auto simulation =
        std::make_shared<toolpath::StockSimulationResult>();
    simulation->initialStock = parent.stock;
    simulation->initialVolume =
        parent.dexelStock->volume();
    simulation->finalVolume =
        simulation->initialVolume;
    simulation->zMapResolution =
        parent.dexelStock->resolution();
    simulation->zMapCellCount =
        parent.dexelStock->nx() *
        parent.dexelStock->ny();
    simulation->zMapActiveCellCount =
        parent.dexelStock->activeCellCount();
    simulation->parallelProcessingEnabled =
        options.pocket.runParallel;
    simulation->parallelWorkerCount =
        options.pocket.runParallel
        ? static_cast<int>(toolpath::toolpathHardwareParallelism())
        : 1;

    std::shared_ptr<const toolpath::MultiDexelStock>
        current = parent.dexelStock;
    bool fatalFailure = false;
    for (std::size_t index = 0;
         index < operations.size(); ++index) {
        if (control && control->cancelled()) {
            simulation->cancelled = true;
            break;
        }
        toolpath::MultiDexelApplyResult applied =
            applyExecutionOperation(
                *current, operations[index], params,
                static_cast<int>(index),
                options.pocket,
                control
                    ? control->isCancellationRequested
                    : std::function<bool()>{});
        simulation->inputValidationMilliseconds +=
            applied.record.inputValidationMilliseconds;
        simulation->booleanMilliseconds +=
            applied.record.booleanMilliseconds;
        simulation->zMapUpdateMilliseconds +=
            applied.updateMilliseconds;
        simulation->booleanBatchCount +=
            applied.record.booleanBatchCount;
        simulation->zMapTileCount +=
            applied.touchedTileCount;
        simulation->sweepSegmentCount +=
            applied.cuttingMoveCount;
        simulation->removalSolidCount +=
            applied.record.removalSolidCount;
        simulation->planarSweepLayerCount +=
            applied.planarLayerCount;
        simulation->fallbackSweepSegmentCount +=
            applied.nonPlanarMoveCount;
        simulation->toolProfileBandCount +=
            applied.toolProfileBandCount;
        simulation->maximumToolProfileAxialError =
            std::max(
                simulation->maximumToolProfileAxialError,
                applied.maximumToolProfileAxialError);
        simulation->maximumToolProfileRadialError =
            std::max(
                simulation->maximumToolProfileRadialError,
                applied.maximumToolProfileRadialError);
        simulation->steps.push_back(applied.record);
        if (applied.record.status ==
            toolpath::StockRemovalStatus::Cancelled) {
            simulation->cancelled = true;
            break;
        }
        ++simulation->processedOperationCount;
        if (applied.record.status ==
            toolpath::StockRemovalStatus::Applied) {
            ++simulation->appliedOperationCount;
            current = std::move(applied.stock);
        } else if (applied.record.status ==
                   toolpath::StockRemovalStatus::
                       NoMaterialRemoved) {
            ++simulation->noMaterialOperationCount;
            current = std::move(applied.stock);
        } else {
            fatalFailure = true;
            if (options.pocket.stopOnFailure)
                break;
        }
    }

    simulation->finalVolume = current->volume();
    simulation->removedVolume = std::max(
        0.0,
        simulation->initialVolume -
            simulation->finalVolume);
    simulation->complete =
        !fatalFailure &&
        !simulation->cancelled &&
        simulation->processedOperationCount ==
            static_cast<int>(operations.size());
    simulation->totalMilliseconds =
        elapsedMilliseconds(simulationStart);
    // 同轴后续节点只消费 Multi-Dexel。BRep/显示网格是派生产物，只有
    // 换轴、旧内核适配、用户查看或显式审计时才按需构造。
    output.dexelStock = std::move(current);
    output.result = std::move(simulation);
    return output;
}

// 按节点种类分派到后端：孔使用原生 Dexel 更新，槽腔/轮廓保留 BRep 换轴过渡。
NodeSimulation simulateNode(
    const MachiningJob& job,
    const IpwSession& session,
    std::size_t index,
    const IpwStockCheckpoint& parent,
    const IpwExecutionOptions& options,
    const toolpath::ToolpathGenerationControl* control)
{
    NodeSimulation output;
    const IpwNode* node = session.node(index);
    if (!node) {
        output.error =
            "IPW runnable node no longer exists";
        return output;
    }
    toolpath::ToolpathGenerationControl nested =
        localControl(control);
    const toolpath::ToolpathGenerationControl* nestedControl =
        control ? &nested : nullptr;

    if (node->operation.kind ==
        IpwOperationKind::Pocket) {
        const toolpath::PocketToolpathBatch* batch =
            job.pocketResult();
        const auto operations =
            pocketOperationsForNode(job, *node);
        if (!batch || operations.empty()) {
            output.error =
                "IPW pocket node has no matching generated toolpath operations";
            return output;
        }
        const gp_Vec axis =
            pocketAxis(operations.front());
        const bool directDexel =
            options.pocket.method ==
                toolpath::StockSimulationMethod::HeightField &&
            parent.dexelStock &&
            parent.dexelStock->axisAligned(axis);
        if (directDexel) {
            return simulateFixedAxisDexel(
                parent, operations,
                batch->effectiveParams,
                options, control);
        }
        if (!options.safeMixedTopologyFallback &&
            options.pocket.method ==
                toolpath::StockSimulationMethod::HeightField) {
            const std::string parentAxis = parent.dexelStock
                ? axisText(parent.dexelStock->wAxis())
                : std::string("unavailable");
            output.error =
                "IPW pocket operation cannot use the parent Dexel grid"
                " (grid axis=" +
                parentAxis +
                ", operation axis=" +
                axisText(axis) + ')';
            return output;
        }

        TopoDS_Shape parentShape;
        if (!derivedSnapshot(
                parent, parentShape,
                output.error, control)) {
            return output;
        }
        toolpath::StockSimulationOptions brepOptions =
            options.pocket;
        brepOptions.method =
            toolpath::StockSimulationMethod::
                FixedAxisLayeredBRep;
        auto simulation =
            std::make_shared<const
                toolpath::StockSimulationResult>(
                toolpath::FixedAxisBRepStockSimulator().simulate(
                    parentShape, operations,
                    batch->effectiveParams,
                    brepOptions, nestedControl));
        output.backend =
            std::string(kPocketTransitionBackend);
        output.result = simulation;
        if (!simulation->complete ||
            simulation->finalStock.IsNull()) {
            output.error =
                simulationFailureMessage(*simulation);
            return output;
        }
        output.dexelStock =
            rasterizeTransitionResult(
                simulation->finalStock,
                axis,
                positiveDiameter(
                    operations.front().tool.valid
                        ? operations.front().tool.diameter
                        : 0.0,
                    batch->effectiveParams.tool.diameter),
                parent.dexelStock,
                options, control, output.error);
        return output;
    }

    if (node->operation.kind ==
        IpwOperationKind::Profile) {
        const toolpath::ProfileToolpathBatch* batch =
            job.profileResult();
        const auto profileOperations =
            profileOperationsForNode(job, *node);
        if (!batch || profileOperations.empty()) {
            output.error =
                "IPW profile node has no matching generated toolpath operations";
            return output;
        }
        const gp_Vec axis = normalizedOrDefault(
            profileOperations.front().outwardToolAxis);
        const bool directDexel =
            options.pocket.method ==
                toolpath::StockSimulationMethod::HeightField &&
            parent.dexelStock &&
            parent.dexelStock->axisAligned(axis);
        if (directDexel) {
            NodeSimulation result = simulateFixedAxisDexel(
                parent, profileOperations,
                batch->effectiveParams,
                options, control);
            result.backend = std::string(kProfileDexelBackend);
            return result;
        }
        if (!options.safeMixedTopologyFallback &&
            options.pocket.method ==
                toolpath::StockSimulationMethod::HeightField) {
            const std::string parentAxis = parent.dexelStock
                ? axisText(parent.dexelStock->wAxis())
                : std::string("unavailable");
            output.error =
                "IPW profile operation cannot use the parent Dexel grid"
                " (grid axis=" + parentAxis +
                ", operation axis=" + axisText(axis) + ')';
            return output;
        }

        TopoDS_Shape parentShape;
        if (!derivedSnapshot(
                parent, parentShape,
                output.error, control)) {
            return output;
        }
        toolpath::StockSimulationOptions brepOptions =
            options.pocket;
        brepOptions.method =
            toolpath::StockSimulationMethod::FixedAxisLayeredBRep;
        // Only the legacy BRep fallback needs a Pocket-shaped carrier.
        const auto operations = toolpath::legacyProfileSimulationOperations(
            profileOperations);
        auto simulation = std::make_shared<const
            toolpath::StockSimulationResult>(
                toolpath::FixedAxisBRepStockSimulator().simulate(
                    parentShape, operations,
                    batch->effectiveParams,
                    brepOptions, nestedControl));
        output.backend = std::string(kProfileTransitionBackend);
        output.result = simulation;
        if (!simulation->complete ||
            simulation->finalStock.IsNull()) {
            output.error = simulationFailureMessage(*simulation);
            return output;
        }
        output.dexelStock = rasterizeTransitionResult(
            simulation->finalStock, axis,
            positiveDiameter(
                profileOperations.front().tool.valid
                    ? profileOperations.front().tool.diameter
                    : 0.0,
                batch->effectiveParams.tool.diameter),
            parent.dexelStock,
            options, control, output.error);
        return output;
    }

    if (node->operation.kind == IpwOperationKind::SurfaceFinishing) {
        const auto operations = finishingOperationsForNode(job, *node);
        if (operations.empty()) {
            output.error = "IPW engraving node has no current generated toolpath";
            return output;
        }
        if (options.pocket.method != toolpath::StockSimulationMethod::HeightField) {
            output.error = "Engraving requires the native Multi-Dexel IPW method";
            return output;
        }
        auto result = simulateFixedAxisDexel(parent, operations,
            operations.front().result.toolpathParams(), options, control);
        result.backend = std::string(kFinishingDexelBackend);
        return result;
    }

    if (node->operation.kind ==
        IpwOperationKind::Hole) {
        const toolpath::HoleToolpathBatch* batch =
            job.holeResult();
        const auto operations =
            holeOperationsForNode(job, *node);
        if (!batch || operations.empty()) {
            output.error =
                "IPW hole node has no matching generated toolpath operations";
            return output;
        }
        const gp_Vec axis = holeAxis(operations.front());
        const bool nativeDexel =
            options.hole.method ==
                toolpath::StockSimulationMethod::HeightField &&
            parent.dexelStock &&
            parent.dexelStock->valid();
        if (!nativeDexel) {
            const std::string parentAxis = parent.dexelStock
                ? axisText(parent.dexelStock->wAxis())
                : std::string("unavailable");
            output.error =
                "IPW hole operation cannot publish a native Dexel state"
                " (grid axis=" +
                parentAxis +
                ", operation axis=" + axisText(axis) + ')';
            return output;
        }
        HoleMultiDexelUpdateResult updated =
            applyHoleOperationsToMultiDexel(
                parent.dexelStock, operations,
                batch->effectiveParams,
                options.hole, control);
        output.backend = std::string(kHoleDexelBackend);
        output.result = std::move(updated.simulation);
        output.dexelStock = std::move(updated.stock);
        output.error = updated.cancelled
            ? "hole multi-dexel update was cancelled"
            : std::move(updated.error);
        return output;
    }

    output.error =
        "IPW operation kind has no registered execution backend";
    return output;
}

// 校验会话的 Ready 前缀与检查点存储逐字段一致(状态、产物与检查点);
// 出现断链则从该节点起全部失效,避免错误复用旧几何。
void invalidateBrokenReadyPrefix(
    IpwSession& session,
    const IpwCheckpointStore& store)
{
    for (std::size_t index = 0;
         index < session.nodes().size();
         ++index) {
        const IpwNode& node =
            session.nodes()[index];
        if (node.status != IpwNodeStatus::Ready ||
            !node.artifact) {
            session.invalidateFrom(index);
            return;
        }
        const auto checkpoint =
            store.find(node.stateKey);
        if (!checkpoint ||
            !checkpoint->dexelStock ||
            checkpoint->parentStateKey !=
                node.parentStateKey ||
            checkpoint->operationId !=
                node.operation.id ||
            checkpoint->contentFingerprint !=
                node.artifact->contentFingerprint ||
            checkpoint->backendId !=
                node.artifact->backendId) {
            session.invalidateFrom(index);
            return;
        }
    }
}

// 收集当前会话仍可达的 stateKey(根 + Ready 前缀),供 retain() 清理检查点。
std::vector<std::string> retainedStateKeys(
    const IpwSession& session)
{
    std::vector<std::string> keys;
    keys.reserve(session.nodes().size() + 1u);
    keys.push_back(session.initialStateKey());
    for (const IpwNode& node : session.nodes()) {
        if (node.status != IpwNodeStatus::Ready ||
            !node.artifact) {
            break;
        }
        keys.push_back(node.stateKey);
    }
    return keys;
}

} // namespace

const char* ipwExecutionStatusName(
    IpwExecutionStatus status) noexcept
{
    switch (status) {
        case IpwExecutionStatus::Complete:
            return "complete";
        case IpwExecutionStatus::Failed:
            return "failed";
        case IpwExecutionStatus::Cancelled:
            return "cancelled";
    }
    return "failed";
}

IpwExecutionResult IpwExecutor::execute(
    MachiningJob& job,
    const TopoDS_Shape& initialStock,
    std::string initialStockFingerprint,
    std::string engineFingerprint,
    const IpwExecutionOptions& options,
    const toolpath::ToolpathGenerationControl* control) const
{
    toolpath::ToolpathParallelScope parallelScope(control);
    IpwExecutionResult result;
    if (initialStock.IsNull()) {
        result.message =
            "IPW execution requires a non-null initial stock";
        return result;
    }

    // 先确定执行帧(序列最小工序的刀轴与参考直径),并据此配置会话计划;
    // 引擎指纹把执行帧与全部仿真参数纳入,参数变化会使缓存前缀失效。
    const auto frameCandidates =
        executionFrameCandidates(
            job, options.includedOperationIds);
    const ExecutionFrameCandidate* firstFrame =
        firstExecutionFrame(frameCandidates);
    const ExecutionFrame frame =
        firstFrame
        ? firstFrame->frame
        : ExecutionFrame{};
    result.planUpdate = job.configureIpwPlan(
        std::move(initialStockFingerprint),
        effectiveEngineFingerprint(
            engineFingerprint, options, frame),
        options.includedOperationIds);
    if (!result.planUpdate.ok()) {
        result.message =
            result.planUpdate.error;
        return result;
    }

    IpwSession& session = job.ipwSession();
    IpwCheckpointStore& store =
        job.runtime().ipwCheckpoints;
    // A Ready material checkpoint must not bypass the finishing generation
    // fingerprint gate after tool/setup/target/parameter edits.
    for (std::size_t index = 0; index < session.nodes().size(); ++index) {
        if (session.nodes()[index].operation.kind != IpwOperationKind::SurfaceFinishing)
            continue;
        std::string error;
        if (!job.currentFinishingResult(&error)) {
            session.invalidateFrom(index);
            store.retain(retainedStateKeys(session));
            result.message = std::move(error);
            return result;
        }
        break;
    }
    // 执行前清理:使 Ready 前缀与会话/检查点存储对齐,并回收不可达检查点。
    invalidateBrokenReadyPrefix(session, store);
    store.retain(retainedStateKeys(session));

    // 根检查点:缓存存在且后端一致时直接复用其 Dexel 状态;否则按执行帧的
    // 刀轴与参考直径重新光栅化初始毛坯并发布根检查点。
    std::shared_ptr<const toolpath::MultiDexelStock>
        rootDexel;
    const auto cachedRoot =
        store.find(session.initialStateKey());
    const bool reuseCachedRoot =
        cachedRoot &&
        cachedRoot->backendId == kInitialBackend &&
        cachedRoot->dexelStock;
    if (reuseCachedRoot) {
        rootDexel = cachedRoot->dexelStock;
    } else {
        if (cachedRoot) {
            session.invalidateFrom(0);
            store.clear();
        }
        std::string buildError;
        rootDexel =
            toolpath::MultiDexelStock::fromShape(
                initialStock,
                frame.outwardAxis,
                frame.referenceCutterDiameter,
                options.pocket,
                nullptr,
                &buildError,
                control
                    ? control->isCancellationRequested
                    : std::function<bool()>{});
        if (!rootDexel) {
            result.message = std::move(buildError);
            if (control && control->cancelled()) {
                result.status =
                    IpwExecutionStatus::Cancelled;
            }
            return result;
        }
    }

    IpwStockCheckpoint root;
    root.stateKey = session.initialStateKey();
    root.backendId =
        std::string(kInitialBackend);
    root.dexelStock = rootDexel;
    root.stock = initialStock;
    root.volume = rootDexel->volume();
    root.contentFingerprint = reuseCachedRoot
        ? cachedRoot->contentFingerprint
        : checkpointContentFingerprint(
              root.stateKey, *rootDexel);
    std::string publishError;
    if (!store.publish(
            std::move(root), &publishError)) {
        result.message = std::move(publishError);
        return result;
    }

    // 统计本次执行可复用的就绪节点数(会话中仍保持 Ready 的前缀节点),
    // 供进度回调与结果报告使用。
    for (const IpwNode& node : session.nodes()) {
        if (node.status != IpwNodeStatus::Ready)
            break;
        ++result.reusedCheckpointCount;
    }
    if (control) {
        control->progress(
            result.reusedCheckpointCount,
            session.nodes().size());
    }

    // 主执行循环:每次取出最早可运行的 Dirty 节点,以其父检查点仿真,发布
    // 新检查点并推进节点状态;取消/失败/缺少父检查点立即中止。
    while (const std::optional<std::size_t> runnable =
               session.firstRunnableNode()) {
        const std::size_t index = *runnable;
        const IpwNode* node = session.node(index);
        if (!node) {
            result.message =
                "IPW runnable node disappeared";
            return result;
        }
        const std::string operationId =
            node->operation.id;
        const std::string stateKey =
            node->stateKey;
        const std::string parentStateKey =
            node->parentStateKey;
        if (control && control->cancelled()) {
            result.status =
                IpwExecutionStatus::Cancelled;
            result.message =
                "IPW execution was cancelled";
            return result;
        }
        const auto parent =
            store.find(parentStateKey);
        if (!parent) {
            result.message =
                "IPW parent checkpoint is missing: " +
                parentStateKey;
            return result;
        }
        if (!session.beginNode(index)) {
            result.message =
                "IPW node could not enter computing state";
            return result;
        }

        NodeSimulation simulation;
        try {
            simulation = simulateNode(
                job, session, index,
                *parent, options, control);
        } catch (const Standard_Failure& failure) {
            const char* message =
                failure.GetMessageString();
            simulation.error =
                std::string(
                    "OCCT failed during IPW execution: ") +
                (message
                     ? message
                     : "unknown OCCT failure");
        } catch (const std::exception& failure) {
            simulation.error =
                std::string(
                    "IPW execution failed: ") +
                failure.what();
        } catch (...) {
            simulation.error =
                "unknown failure during IPW execution";
        }
        if (!simulation.result ||
            !simulation.dexelStock) {
            const std::string message =
                simulation.error.empty()
                ? "IPW node did not publish a multi-dexel state"
                : simulation.error;
            session.failNode(index, message);
            result.message =
                operationId + ": " + message;
            return result;
        }
        if (simulation.result->cancelled ||
            (control && control->cancelled())) {
            session.cancelNode(
                index,
                "IPW operation simulation was cancelled");
            result.status =
                IpwExecutionStatus::Cancelled;
            result.message =
                "IPW execution was cancelled during " +
                operationId;
            return result;
        }
        if (!simulation.result->complete) {
            const std::string message =
                simulation.error.empty()
                ? simulationFailureMessage(
                      *simulation.result)
                : simulation.error;
            session.failNode(index, message);
            result.message =
                operationId + ": " + message;
            return result;
        }

        // 组装并发布节点检查点:携带状态键、父键、工序 ID、后端标识、Dexel
        // 真值与单工序仿真明细;发布成功后再把产物描述提交给会话。
        IpwStockCheckpoint checkpoint;
        checkpoint.stateKey = stateKey;
        checkpoint.parentStateKey =
            parentStateKey;
        checkpoint.operationId = operationId;
        checkpoint.backendId =
            simulation.backend;
        checkpoint.dexelStock =
            simulation.dexelStock;
        checkpoint.stock =
            simulation.result->finalStock;
        checkpoint.volume =
            simulation.dexelStock->volume();
        checkpoint.contentFingerprint =
            checkpointContentFingerprint(
                checkpoint.stateKey,
                *checkpoint.dexelStock);
        checkpoint.operationResult =
            std::move(simulation.result);
        const std::string contentFingerprint =
            checkpoint.contentFingerprint;
        const std::string backend =
            checkpoint.backendId;
        if (!store.publish(
                std::move(checkpoint),
                &publishError)) {
            session.failNode(
                index, publishError);
            result.message =
                std::move(publishError);
            return result;
        }

        IpwArtifactDescriptor artifact;
        artifact.stateKey = stateKey;
        artifact.backendId = backend;
        artifact.contentFingerprint =
            contentFingerprint;
        artifact.checkpoint = true;
        if (!session.completeNode(
                index, std::move(artifact))) {
            result.message =
                "IPW session rejected a matching completed checkpoint";
            return result;
        }
        ++result.executedNodeCount;
        if (backend == kHoleDexelBackend) {
            ++result.nativeHoleNodeCount;

        }
        result.executedOperationIds.push_back(
            operationId);
        if (control) {
            control->progress(
                index + 1,
                session.nodes().size());
        }
    }

    if (!session.complete()) {
        result.message =
            "IPW execution stopped before all nodes became ready";
        return result;
    }
    // 全部节点就绪后,取末尾节点(无节点时取根)的检查点作为最终结果。
    const std::string finalKey =
        session.nodes().empty()
        ? session.initialStateKey()
        : session.nodes().back().stateKey;
    result.finalCheckpoint =
        store.find(finalKey);
    if (!result.finalCheckpoint) {
        result.message =
            "IPW final checkpoint is missing after execution";
        return result;
    }
    result.status =
        IpwExecutionStatus::Complete;
    return result;
}

} // namespace oap::manufacturing::ipw
