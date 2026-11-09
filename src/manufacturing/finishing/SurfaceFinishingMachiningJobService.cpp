#include "SurfaceFinishingMachiningJobService.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_set>
#include <QCryptographicHash>
#include <QDateTime>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Compound.hxx>

#include "manufacturing/job/MachiningJobApplicationService.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "manufacturing/job/detail/SurfaceFinishingJson.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"
#include "toolpath/core/simulation/StockBoxBuilder.h"
#include "toolpath/core/simulation/StockModelLoader.h"

namespace oap::manufacturing {
namespace {
std::string targetFingerprint(const PartDefinition& part, const std::vector<std::string>& units)
{
    QByteArray input = QByteArray::fromStdString(part.sourceFingerprint);
    for (const auto& id : units) {
        input += '\0';
        input += QByteArray::fromStdString(id);
    }
    return QCryptographicHash::hash(input, QCryptographicHash::Sha256).toHex().toStdString();
}
void fail(toolpath::SurfaceFinishingBatch& batch, const std::string& message)
{
    batch.operations.clear();
    toolpath::addToolpathIssue(batch.validation, toolpath::ToolpathIssueSeverity::Error,
        "surface_finishing_job", message);
}
toolpath::SurfaceFinishingParams boundParameters(toolpath::SurfaceFinishingParams params,
                                                const toolpath::ToolpathToolContext& tool)
{
    params.diameter = tool.diameter;
    params.cuttingLength = tool.cuttingEdgeLength;
    params.cornerRadius = tool.cornerRadius;
    params.feed = tool.horizontalFeed;
    params.plungeFeed = tool.verticalFeed;
    return params;
}
bool sameBoundGeometry(const toolpath::SurfaceFinishingParams& p, const toolpath::ToolpathToolContext& tool)
{
    const auto same = [](double a, double b) { return std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <= 1e-8; };
    const double expectedCorner = p.cutterKind == toolpath::FinishingCutterKind::Ball ? p.diameter / 2 :
        p.cutterKind == toolpath::FinishingCutterKind::Bull ? p.cornerRadius : 0;
    return tool.valid && tool.toolNumber > 0 && detail::finishingCutterMatchesTool(p.cutterKind, tool) && same(p.diameter, tool.diameter) &&
        same(p.cuttingLength, tool.cuttingEdgeLength) && same(expectedCorner, tool.cornerRadius) &&
        same(p.feed, tool.horizontalFeed) && same(p.plungeFeed, tool.verticalFeed);
}
toolpath::SurfaceFinishingOperation runtimeOperation(
    const SurfaceFinishingOperationDefinition& definition, const SetupDefinition& setup,
    const toolpath::ToolpathToolContext& tool, toolpath::SurfaceFinishingResult result)
{
    toolpath::SurfaceFinishingOperation operation;
    operation.operationId = definition.id;
    operation.setupFace = setup.face;
    operation.setupIndex = setup.setupIndex;
    operation.workFrame = gp_Ax3(toPoint(setup.origin),
        gp_Dir(setup.zAxis.x, setup.zAxis.y, setup.zAxis.z),
        gp_Dir(setup.xAxis.x, setup.xAxis.y, setup.xAxis.z));
    operation.result = std::move(result);
    operation.tool = tool;
    return operation;
}
void validateBatch(toolpath::SurfaceFinishingBatch& batch)
{
    for (const auto& operation : batch.operations) {
        batch.validation.append(toolpath::ToolpathProgramValidator().validate(
            operation.toProgram(), operation.result.toolpathParams()));
    }
    if (!batch.operations.empty())
        batch.effectiveParams = batch.operations.front().result.toolpathParams();
}
bool refreshActivePlan(MachiningJob& next, const MachiningJob& previous)
{
    const auto& session = previous.ipwSession();
    if (!session.configured()) return true;
    std::unordered_set<std::string> enabled;
    for (const auto& op : MachiningOperationService::operations(previous.definition()))
        if (op.enabled) enabled.insert(op.id);
    bool full = enabled.size() == session.nodes().size();
    std::vector<std::string> included;
    for (const auto& node : session.nodes()) {
        full = full && enabled.contains(node.operation.id);
        included.push_back(node.operation.id);
    }
    if (full) {
        included.clear();
    } else {
        std::unordered_set<std::string> retained;
        for (const auto& op : MachiningOperationService::operations(next.definition()))
            if (op.enabled) retained.insert(op.id);
        std::erase_if(included, [&](const auto& id) { return !retained.contains(id); });
        // An emptied explicit selection must not become "all operations".
        if (included.empty()) {
            next.runtime().ipwSession.clear();
            return true;
        }
    }
    return next.configureIpwPlan(session.initialStockFingerprint(), session.engineFingerprint(), included).ok();
}
} // namespace

TopoDS_Shape SurfaceFinishingMachiningJobService::buildPlanningStock(
    StockDefinition& stock, const std::vector<TopoDS_Shape>& completeTargetUnits,
    std::string* error, const toolpath::ToolpathGenerationControl* control)
{
    if (error) error->clear();
    const auto reject = [error](const std::string& message) {
        if (error) *error = message;
        return TopoDS_Shape{};
    };
    if (control && control->cancelled()) return reject("Planning stock construction was cancelled.");
    if (stock.sourceKind == StockSourceKind::StepModel) {
        QString failure;
        const auto before = MachiningJobStorage::sourceFileFingerprint(
            QString::fromStdString(stock.sourcePath), &failure);
        if (before.isEmpty()) return reject(failure.toStdString());
        if (!stock.sourceFingerprint.empty() && stock.sourceFingerprint != before.toStdString())
            return reject("Planning stock STEP content changed; explicitly reselect the stock and regenerate the job.");
        const auto loaded = toolpath::StockModelLoader().loadStep(stock.sourcePath, control);
        if (!loaded.ok()) return reject(loaded.message);
        const auto after = MachiningJobStorage::sourceFileFingerprint(
            QString::fromStdString(stock.sourcePath), &failure);
        if (after.isEmpty() || after != before)
            return reject("Planning stock STEP changed while it was being read.");
        stock.sourceFingerprint = before.toStdString();
        return loaded.shape;
    }
    if (stock.sourceKind != StockSourceKind::PerUnitBoxes && stock.sourceKind != StockSourceKind::ModelBox)
        return reject("Planning stock source is invalid.");
    if (completeTargetUnits.empty()) return reject("Planning stock requires complete target units.");
    BRep_Builder builder;
    TopoDS_Compound boxes;
    builder.MakeCompound(boxes);
    Bnd_Box modelBounds;
    const auto makeBox = [&](const Bnd_Box& bounds) {
        if (bounds.IsVoid() || bounds.IsWhole())
            return reject("A complete target has no finite planning-stock bounds.");
        Bbox coordinates;
        bounds.Get(coordinates.xmin, coordinates.ymin, coordinates.zmin,
                   coordinates.xmax, coordinates.ymax, coordinates.zmax);
        const auto result = toolpath::buildStockBox(coordinates, nullptr,
            stock.marginX, stock.marginY, stock.marginZ);
        if (!result.ok()) return reject(result.message);
        return result.shape;
    };
    for (const auto& target : completeTargetUnits) {
        if (control && control->cancelled()) return reject("Planning stock construction was cancelled.");
        if (target.IsNull()) return reject("A complete target unit is missing.");
        Bnd_Box bounds;
        // A conservative planning-stock envelope may be wider than the target;
        // avoid costly spline extrema searches and never depend on cached mesh.
        BRepBndLib::Add(target, bounds, Standard_False);
        if (stock.sourceKind == StockSourceKind::ModelBox) {
            if (bounds.IsVoid() || bounds.IsWhole()) return reject("A complete target has invalid bounds.");
            modelBounds.Add(bounds);
        } else {
            const auto box = makeBox(bounds);
            if (box.IsNull()) return {};
            builder.Add(boxes, box);
        }
    }
    stock.sourceFingerprint.clear();
    return stock.sourceKind == StockSourceKind::ModelBox ? makeBox(modelBounds) : boxes;
}

std::string SurfaceFinishingMachiningJobService::operationFingerprint(
    const MachiningJobDefinition& definition, const SurfaceFinishingOperationDefinition& operation)
{
    MachiningJobDefinition one = definition;
    one.finishingOperations = {operation};
    auto& normalized = one.finishingOperations.front();
    normalized.inputFingerprint.clear();
    normalized.sequence = 0;
    normalized.enabled = true;
    return MachiningJobStorage::finishingExecutionFingerprint(one).toStdString();
}

bool SurfaceFinishingMachiningJobService::capture(
    MachiningJob& job, const SurfaceFinishingJobCaptureRequest& request, std::string* error) const
{
    if (error) error->clear();
    const auto reject = [error](const std::string& message) { if (error) *error = message; return false; };
    if (!request.result || !request.result->ok || request.result->cancelled || request.result->moves.empty() ||
        !request.tool || !request.setupFrames || request.unitIds.empty())
        return reject("Engraving capture requires a successful result, tool, complete target and setup.");
    if (!sameBoundGeometry(request.result->effectiveParams, *request.tool))
        return reject("The captured engraving tool differs from the tool used to generate the path.");
    const auto* frame = request.setupFrames->forFace(request.setupFace);
    if (!frame || !frame->valid) return reject("The engraving setup is invalid.");
    MachiningJob next = job;
    auto& def = next.definition();
    toolpath::SurfaceFinishingBatch batch;
    if (const auto* old = job.currentFinishingResult()) batch = *old;
    auto setup = std::find_if(def.setups.begin(), def.setups.end(), [&](const auto& s) { return s.face == request.setupFace; });
    if (setup == def.setups.end()) {
        SetupDefinition created;
        std::unordered_set<int> used;
        for (const auto& s : def.setups) used.insert(s.setupIndex);
        while (used.contains(created.setupIndex)) ++created.setupIndex;
        if (created.setupIndex >= 6) return reject("No setup index is available for engraving.");
        created.id = "setup-finishing-" + std::to_string(created.setupIndex);
        if (next.setupById(created.id)) return reject("Engraving setup ID conflicts with an existing setup.");
        created.face = request.setupFace;
        def.setups.push_back(created);
    }
    MachiningJobApplicationService::captureSetups(def, *request.setupFrames,
        QDateTime::currentDateTimeUtc().toString(Qt::ISODate).toStdString());
    for (const auto& original : job.definition().setups) {
        const auto* updated = next.setupById(original.id);
        if (!updated || MachiningJobStorage::setupInputFingerprint(def.part, *updated) !=
            MachiningJobStorage::setupInputFingerprint(job.definition().part, original))
            return reject("Setup geometry changed; regenerate existing operations before installing engraving.");
    }
    setup = std::find_if(def.setups.begin(), def.setups.end(), [&](const auto& s) { return s.face == request.setupFace; });
    SurfaceFinishingOperationDefinition operation;
    if (!request.replaceOperationId.empty()) {
        const auto old = std::find_if(def.finishingOperations.begin(), def.finishingOperations.end(),
            [&](const auto& item) { return item.id == request.replaceOperationId; });
        if (old == def.finishingOperations.end()) return reject("The engraving operation to edit no longer exists.");
        operation = *old;
    } else {
        std::unordered_set<std::string> ids;
        int last = -1;
        for (const auto& existing : MachiningOperationService::operations(def)) {
            ids.insert(existing.id);
            last = std::max(last, existing.sequence);
        }
        if (last == std::numeric_limits<int>::max()) return reject("Operation sequence limit exceeded.");
        operation.sequence = last + 1;
        std::size_t suffix = 1;
        do { operation.id = "surface-finish-" + std::to_string(suffix++); } while (ids.contains(operation.id));
    }
    operation.setupId = setup->id;
    operation.toolControllerId = operation.id + "-tool";
    for (const auto& other : MachiningOperationService::operations(def))
        if (other.id != operation.id && other.toolControllerId == operation.toolControllerId)
            return reject("Engraving tool controller ID is shared by another operation.");
    operation.parameters = request.result->effectiveParams;
    operation.algorithmId = operation.parameters.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing
        ? toolpath::kOpenCamLibStockEngravingAlgorithmId : toolpath::kOpenCamLibSurfaceFinishingAlgorithmId;
    operation.target.unitIds = request.unitIds;
    std::sort(operation.target.unitIds.begin(), operation.target.unitIds.end());
    operation.target.unitIds.erase(std::unique(operation.target.unitIds.begin(), operation.target.unitIds.end()), operation.target.unitIds.end());
    operation.target.geometryFingerprint = targetFingerprint(def.part, operation.target.unitIds);
    operation.state = setup->confirmation.confirmed ? JobOperationState::Warning : JobOperationState::NeedsReview;
    JobToolController controller;
    controller.id = operation.toolControllerId;
    controller.libraryId = request.libraryId;
    controller.tool = *request.tool;
    const auto existingController = std::find_if(def.toolControllers.begin(), def.toolControllers.end(),
        [&](const auto& item) { return item.id == controller.id; });
    if (existingController == def.toolControllers.end()) def.toolControllers.push_back(controller);
    else *existingController = controller;
    operation.inputFingerprint = operationFingerprint(def, operation);
    const auto existing = std::find_if(def.finishingOperations.begin(), def.finishingOperations.end(),
        [&](const auto& item) { return item.id == operation.id; });
    if (existing == def.finishingOperations.end()) def.finishingOperations.push_back(operation);
    else *existing = operation;
    std::erase_if(batch.operations, [&](const auto& item) { return item.operationId == operation.id; });
    if (operation.enabled)
        batch.operations.push_back(runtimeOperation(operation, *setup, *request.tool, *request.result));
    validateBatch(batch);
    if (!batch.validation.ok()) return reject(batch.validation.summary());
    const auto errors = next.validationErrors();
    if (!errors.empty()) return reject(errors.front());
    next.setFinishingResult(std::move(batch), MachiningJobStorage::finishingExecutionFingerprint(def).toStdString());
    if (!refreshActivePlan(next, job)) return reject("Updated engraving cannot restore the active IPW plan.");
    job = std::move(next);
    return true;
}

bool SurfaceFinishingMachiningJobService::replaceFamily(
    MachiningJob& job, const std::vector<SurfaceFinishingJobCaptureRequest>& requests,
    std::string* error) const
{
    if (error) error->clear();
    const auto reject = [error](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (requests.empty()) return reject("Engraving requires at least one generated setup.");
    MachiningJob next = job;
    auto& def = next.definition();
    const auto previousOperations = def.finishingOperations;
    std::unordered_set<int> faces;
    for (const auto& request : requests) {
        if (!request.result || !request.result->ok || request.result->cancelled || request.result->moves.empty() ||
            !request.tool || !request.setupFrames || request.unitIds.empty() ||
            !sameBoundGeometry(request.result->effectiveParams, *request.tool))
            return reject("Engraving batch contains an incomplete result or mismatched tool.");
        const auto* frame = request.setupFrames->forFace(request.setupFace);
        if (!frame || !frame->valid || !faces.insert(static_cast<int>(request.setupFace)).second)
            return reject("Engraving batch contains an invalid or duplicate setup face.");
        auto setup = std::find_if(def.setups.begin(), def.setups.end(),
            [&](const auto& item) { return item.face == request.setupFace; });
        if (setup != def.setups.end()) continue;
        SetupDefinition created;
        std::unordered_set<int> used;
        for (const auto& item : def.setups) used.insert(item.setupIndex);
        while (used.contains(created.setupIndex)) ++created.setupIndex;
        if (created.setupIndex >= 6) return reject("No setup index is available for engraving.");
        created.id = "setup-engraving-" + std::to_string(created.setupIndex);
        if (next.setupById(created.id)) return reject("Engraving setup ID conflicts with an existing setup.");
        created.face = request.setupFace;
        def.setups.push_back(std::move(created));
    }
    MachiningJobApplicationService::captureSetups(def, *requests.front().setupFrames,
        QDateTime::currentDateTimeUtc().toString(Qt::ISODate).toStdString());
    for (const auto& previous : job.definition().setups) {
        const auto* updated = next.setupById(previous.id);
        if (!updated || MachiningJobStorage::setupInputFingerprint(def.part, *updated) !=
            MachiningJobStorage::setupInputFingerprint(job.definition().part, previous))
            return reject("Setup geometry changed; regenerate preceding operations before engraving.");
    }
    std::unordered_set<std::string> usedIds;
    int lastSequence = -1;
    for (const auto& item : MachiningOperationService::operations(def)) {
        usedIds.insert(item.id);
        lastSequence = std::max(lastSequence, item.sequence);
    }
    def.finishingOperations.clear();
    toolpath::SurfaceFinishingBatch batch;
    for (const auto& request : requests) {
        const auto setup = std::find_if(def.setups.begin(), def.setups.end(),
            [&](const auto& item) { return item.face == request.setupFace; });
        SurfaceFinishingOperationDefinition operation;
        const auto previous = std::find_if(previousOperations.begin(), previousOperations.end(),
            [&](const auto& item) { return item.setupId == setup->id; });
        if (previous != previousOperations.end()) {
            operation = *previous;
        } else {
            if (lastSequence == std::numeric_limits<int>::max())
                return reject("Operation sequence limit exceeded.");
            operation.sequence = ++lastSequence;
            std::size_t suffix = 1;
            do { operation.id = "engraving-" + std::to_string(suffix++); } while (usedIds.contains(operation.id));
            usedIds.insert(operation.id);
        }
        operation.setupId = setup->id;
        operation.toolControllerId = operation.id + "-tool";
        for (const auto& other : MachiningOperationService::operations(def)) {
            if (other.id != operation.id && other.toolControllerId == operation.toolControllerId)
                return reject("Engraving controller ID is shared by another operation.");
        }
        operation.parameters = request.result->effectiveParams;
        operation.algorithmId = operation.parameters.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing
            ? toolpath::kOpenCamLibStockEngravingAlgorithmId : toolpath::kOpenCamLibSurfaceFinishingAlgorithmId;
        operation.target.unitIds = request.unitIds;
        std::sort(operation.target.unitIds.begin(), operation.target.unitIds.end());
        operation.target.unitIds.erase(std::unique(operation.target.unitIds.begin(), operation.target.unitIds.end()), operation.target.unitIds.end());
        operation.target.geometryFingerprint = targetFingerprint(def.part, operation.target.unitIds);
        operation.state = setup->confirmation.confirmed ? JobOperationState::Warning : JobOperationState::NeedsReview;
        JobToolController controller{operation.toolControllerId, request.libraryId, *request.tool};
        const auto existing = std::find_if(def.toolControllers.begin(), def.toolControllers.end(),
            [&](const auto& item) { return item.id == controller.id; });
        if (existing == def.toolControllers.end()) def.toolControllers.push_back(std::move(controller));
        else *existing = std::move(controller);
        operation.inputFingerprint = operationFingerprint(def, operation);
        if (operation.enabled)
            batch.operations.push_back(runtimeOperation(operation, *setup, *request.tool, *request.result));
        def.finishingOperations.push_back(std::move(operation));
    }
    std::unordered_set<std::string> referencedControllers;
    for (const auto& item : MachiningOperationService::operations(def))
        referencedControllers.insert(item.toolControllerId);
    for (const auto& previous : previousOperations) {
        if (!referencedControllers.contains(previous.toolControllerId))
            std::erase_if(def.toolControllers, [&](const auto& item) { return item.id == previous.toolControllerId; });
    }
    std::sort(def.finishingOperations.begin(), def.finishingOperations.end(),
        [](const auto& a, const auto& b) { return a.sequence < b.sequence; });
    const auto sequenceFor = [&](const std::string& id) {
        const auto operation = std::find_if(def.finishingOperations.begin(), def.finishingOperations.end(),
            [&](const auto& value) { return value.id == id; });
        return operation->sequence;
    };
    std::sort(batch.operations.begin(), batch.operations.end(), [&](const auto& a, const auto& b) {
        return sequenceFor(a.operationId) < sequenceFor(b.operationId);
    });
    validateBatch(batch);
    if (!batch.validation.ok()) return reject(batch.validation.summary());
    const auto errors = next.validationErrors();
    if (!errors.empty()) return reject(errors.front());
    next.setFinishingResult(std::move(batch), MachiningJobStorage::finishingExecutionFingerprint(def).toStdString());
    if (!refreshActivePlan(next, job)) return reject("Updated engraving cannot restore the active IPW plan.");
    job = std::move(next);
    return true;
}

toolpath::SurfaceFinishingBatch SurfaceFinishingMachiningJobService::regenerate(
    MachiningJob& job, const StepIndexer& indexer, const toolpath::ToolpathGenerationControl* control) const
{
    toolpath::SurfaceFinishingBatch batch;
    const auto errors = job.validationErrors();
    if (!errors.empty()) { fail(batch, errors.front()); return batch; }
    MachiningJob next = job;
    auto& def = next.definition();
    std::vector<SurfaceFinishingOperationDefinition*> ordered;
    std::map<std::vector<std::string>, TopoDS_Shape> planningStocks;
    for (auto& op : def.finishingOperations) if (op.enabled) ordered.push_back(&op);
    // One shared budget for the enabled layered family. Historic definitions
    // may carry different per-operation settings; honor the strictest budget.
    std::size_t remainingMoves = std::numeric_limits<std::size_t>::max();
    for (const auto* op : ordered)
        if (op->parameters.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing)
            remainingMoves = std::min(remainingMoves, op->parameters.maxMoves);
    std::sort(ordered.begin(), ordered.end(), [](const auto* a, const auto* b) { return a->sequence < b->sequence; });
    for (auto* op : ordered) {
        if (control && control->cancelled()) { batch.cancelled = true; batch.operations.clear(); return batch; }
        const auto* setup = next.setupById(op->setupId);
        const auto* controller = next.toolControllerById(op->toolControllerId);
        if (!setup || !setup->valid || !controller || !controller->tool.valid) {
            fail(batch, "Engraving operation references an invalid setup/tool: " + op->id); return batch;
        }
        if (targetFingerprint(def.part, op->target.unitIds) != op->target.geometryFingerprint) {
            fail(batch, "Engraving target identity changed; reselect the complete target: " + op->id); return batch;
        }
        BRep_Builder builder;
        TopoDS_Compound target;
        builder.MakeCompound(target);
        std::vector<TopoDS_Shape> targetUnits;
        for (const auto& unit : op->target.unitIds) {
            const auto* shape = indexer.shapeForUnit(unit);
            if (!shape || shape->IsNull()) { fail(batch, "Engraving target unit is missing: " + unit); return batch; }
            builder.Add(target, *shape);
            targetUnits.push_back(*shape);
        }
        op->parameters = boundParameters(op->parameters, controller->tool);
        if (!detail::validFinishingParameters(op->parameters) || !sameBoundGeometry(op->parameters, controller->tool)) {
            fail(batch, "Bound engraving tool/parameters are invalid: " + op->id); return batch;
        }
        const gp_Ax3 frame(toPoint(setup->origin), gp_Dir(setup->zAxis.x, setup->zAxis.y, setup->zAxis.z),
            gp_Dir(setup->xAxis.x, setup->xAxis.y, setup->xAxis.z));
        auto& stock = planningStocks[op->target.unitIds];
        std::string stockError;
        if (stock.IsNull()) stock = buildPlanningStock(def.stock, targetUnits, &stockError, control);
        if (control && control->cancelled()) { batch.cancelled = true; batch.operations.clear(); return batch; }
        if (stock.IsNull()) { fail(batch, stockError); return batch; }
        const bool layered = op->parameters.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing;
        auto generationParams = op->parameters;
        if (layered) {
            if (remainingMoves < 4) {
                fail(batch, "The cumulative layered engraving move budget was exhausted across setups.");
                return batch;
            }
            generationParams.maxMoves = remainingMoves;
        }
        auto result = toolpath::SurfaceFinishing().generate(target, frame, generationParams, control, &stock);
        if (result.cancelled) { batch.cancelled = true; batch.operations.clear(); return batch; }
        if (!result.ok) { fail(batch, result.error); return batch; }
        if (layered) {
            if (result.moves.size() > remainingMoves) {
                fail(batch, "The cumulative layered engraving move budget was exceeded across setups.");
                return batch;
            }
            remainingMoves -= result.moves.size();
            result.effectiveParams.maxMoves = op->parameters.maxMoves;
        }
        op->inputFingerprint = operationFingerprint(def, *op);
        op->state = setup->confirmation.confirmed ? JobOperationState::Warning : JobOperationState::NeedsReview;
        batch.operations.push_back(runtimeOperation(*op, *setup, controller->tool, std::move(result)));
    }
    validateBatch(batch);
    if (!batch.validation.ok()) { batch.operations.clear(); return batch; }
    if (control && control->cancelled()) { batch.cancelled = true; batch.operations.clear(); return batch; }
    next.setFinishingResult(batch, MachiningJobStorage::finishingExecutionFingerprint(def).toStdString());
    if (!refreshActivePlan(next, job)) { fail(batch, "Regenerated engraving cannot restore the active IPW plan."); return batch; }
    job = std::move(next);
    return batch;
}
} // namespace oap::manufacturing
