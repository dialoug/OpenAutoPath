#include "Project.h"
#include "Planning.h"
#include "Simulation.h"
#include "toolpath/pocket/simulation/IpwQuality.h"
#include "recognition/FeatureRecognizer.h"
#include "tooling/adapter/hole/HoleMachiningInputMapper.h"
#include "manufacturing/program/MachiningProgramPartitioner.h"
#include "NcReview.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"
#include "toolpath/core/PocketToolBinding.h"
#include "toolpath/pocket/program/PocketToolpathExport.h"
#include <map>
#include <sstream>
#include "manufacturing/finishing/SurfaceFinishingMachiningJobService.h"
#include "manufacturing/hole/HoleMachiningJobService.h"
#include "manufacturing/ipw/IpwExecutor.h"
#include "manufacturing/job/MachiningJobApplicationService.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "manufacturing/pocket/PocketMachiningJobService.h"
#include "manufacturing/profile/ProfileMachiningJobService.h"
#include "toolpath/program/ToolpathProgramExport.h"
#include "tooling/adapter/AssignedToolContext.h"
#include <TopoDS_Iterator.hxx>
#include <set>

#include <QDateTime>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QUuid>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace oap::worker
{
namespace
{
using namespace manufacturing;
QString modelHash(const QJsonObject &request)
{
    QString error;
    auto hash = MachiningJobStorage::sourceFileFingerprint(request["model"].toString(), &error);
    if (hash.isEmpty())
        throw std::runtime_error(error.toStdString());
    return hash;
}
void saveDefinition(const MachiningJobDefinition &def)
{
    QString error;
    if (!MachiningJobStorage::save("project.oapjob", def, &error))
        throw std::runtime_error(error.toStdString());
}

// Generated Web operations have one recognized hole and one physical hole.
// Remap the batch-local indices before installing it into the project's global order.
void appendGeneratedHole(MachiningJobDefinition &def, const QString &file)
{
    MachiningJob source;
    auto &add = source.definition();
    QString error;
    if (!MachiningJobStorage::load(file, add, &error))
        throw std::runtime_error(error.toStdString());
    if (add.part.sourceFingerprint != def.part.sourceFingerprint || add.recognizedHoles.size() != 1 ||
        add.recognizedPhysicalHoleCount != 1 || add.holeOperations.empty())
        throw std::runtime_error("工序与项目的零件或识别结果不匹配");
    const std::string prefix = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString() + "-";
    const auto operations = MachiningOperationService::operations(def);
    int sequence = operations.empty() ? 0 : operations.back().sequence + 1;
    int holeIndex = -1, physicalIndex = -1;
    const auto &incoming = add.holeOperations.front().targets.at(0);
    for (const auto &op : def.holeOperations)
        for (const auto &target : op.targets)
            if (target.geometryFingerprint == incoming.geometryFingerprint &&
                target.featureId == incoming.featureId)
            {
                holeIndex = target.sourceHoleIndex;
                physicalIndex = target.physicalHoleIndex;
            }
    if (holeIndex < 0)
    {
        holeIndex = static_cast<int>(def.recognizedHoles.size());
        physicalIndex = def.recognizedPhysicalHoleCount++;
        def.recognizedHoles.push_back(add.recognizedHoles.front());
    }
    for (auto controller : add.toolControllers)
    {
        controller.id = prefix + controller.id;
        def.toolControllers.push_back(std::move(controller));
    }
    for (auto op : add.holeOperations)
    {
        const auto *setup = source.setupById(op.setupId);
        if (!setup)
            throw std::runtime_error("工序缺少装夹定义");
        const auto existing = std::find_if(def.setups.begin(), def.setups.end(),
                                           [&](const auto &s) { return s.face == setup->face; });
        if (existing == def.setups.end())
        {
            auto copy = *setup;
            copy.id = prefix + copy.id;
            def.setups.push_back(copy);
            op.setupId = copy.id;
        }
        else
            op.setupId = existing->id;
        op.id = prefix + op.id;
        op.toolControllerId = prefix + op.toolControllerId;
        op.sequence = sequence++;
        op.state = JobOperationState::Dirty;
        for (auto &target : op.targets)
        {
            target.sourceHoleIndex = holeIndex;
            target.physicalHoleIndex = physicalIndex;
            for (auto &covered : target.coveredSourceHoleIndices)
                covered = holeIndex;
        }
        def.holeOperations.push_back(std::move(op));
    }
}
void changePocketTool(MachiningJob& job, const QJsonObject& request, const tooling::ToolLibrary& library)
{
    auto& def = job.definition();
    const auto id = request["operation"].toString().toStdString();
    auto op = std::find_if(def.pocketOperations.begin(), def.pocketOperations.end(),
                           [&](const auto& item) { return item.id == id && item.enabled; });
    if (op == def.pocketOperations.end())
        throw std::runtime_error("请选择已启用的槽腔工序");
    const auto toolId = request["tool"].toString();
    const auto* bit = library.toolById(toolId);
    const auto* slot = library.slotForTool(toolId);
    if (!bit || !slot || !tooling::hasSupportedMillingGeometry(*bit))
        throw std::runtime_error("请选择已分配刀位的铣刀");
    const auto replacement = tooling::makeToolpathToolContext({*bit, *slot});
    const auto& params = op->parameters ? *op->parameters : def.pocketParameters;
    if (params.depth.stepDown > replacement.cuttingEdgeLength)
        throw std::runtime_error("工序层深超过所选刀具刃长");
    const auto oldId = op->toolControllerId;
    op->toolControllerId = "tool-controller-pocket-" + QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    def.toolControllers.push_back({op->toolControllerId, library.id().toStdString(), replacement});
    const auto all = MachiningOperationService::operations(def);
    if (std::none_of(all.begin(), all.end(), [&](const auto& item){return item.toolControllerId == oldId;}))
        std::erase_if(def.toolControllers, [&](const auto& item){return item.id == oldId;});
    op->state = JobOperationState::Dirty;
}

template <class T> void selectOperation(std::vector<T> &operations, const std::string &id)
{
    for (auto &op : operations)
        op.enabled = op.enabled && op.id == id;
}
struct RebuiltOperation
{
    toolpath::ToolpathProgram program;
    toolpath::ToolpathParams params;
    toolpath::ToolpathValidationReport report;
};

RebuiltOperation rebuildFamily(MachiningJob &job, JobOperationKind kind, const StepIndexer &index,
                               const toolpath::HoleRapidPlanningContext &context,
                               const std::string &stockPath)
{
    toolpath::ToolpathProgram program;
    toolpath::ToolpathParams params;
    toolpath::ToolpathValidationReport report;
    switch (kind)
    {
    case JobOperationKind::Hole: {
        const auto batch = HoleMachiningJobService().regenerate(job, &context);
        report = batch.validation;
        program = batch.toProgram();
        params = batch.effectiveParams;
        break;
    }
    case JobOperationKind::Pocket: {
        const auto batch = PocketMachiningJobService().regenerate(job, index);
        if (!job.currentPocketResult())
            throw std::runtime_error("槽腔工序重建或验证失败");
        report = batch.validation;
        program = batch.toProgram();
        params = batch.effectiveParams;
        break;
    }
    case JobOperationKind::Profile: {
        const auto batch = ProfileMachiningJobService().regenerate(job, index);
        if (batch.cancelled || !job.hasProfileResult() || !batch.safetyValidation.ok())
            throw std::runtime_error("外轮廓工序重建或验证失败");
        report = batch.safetyValidation;
        program = batch.toProgram();
        params = batch.effectiveParams;
        break;
    }
    case JobOperationKind::SurfaceFinishing: {
        const auto batch = SurfaceFinishingMachiningJobService().regenerate(job, index, nullptr, stockPath);
        if (!job.currentFinishingResult())
            throw std::runtime_error("雕刻工序重建或验证失败");
        report = batch.validation;
        program = batch.toProgram();
        params = batch.effectiveParams;
        break;
    }
    }
    if (program.empty() || !report.ok())
        throw std::runtime_error("工序无有效刀路: " + report.summary());
    return {std::move(program), params, std::move(report)};
}

QJsonArray point(const gp_Pnt &p)
{
    return {p.X(), p.Y(), p.Z()};
}

ipw::IpwExecutionResult executeProject(MachiningJob &job, const StepIndexer &index,
                                       const toolpath::HoleRapidPlanningContext &context,
                                       std::map<JobOperationKind, toolpath::ToolpathParams> &parameters,
                                       const std::string &stockPath, const ipw::IpwExecutionOptions &options)
{
    std::set<JobOperationKind> kinds;
    for (const auto &operation : MachiningOperationService::operations(job.definition()))
        if (operation.enabled)
            kinds.insert(operation.kind);
    if (kinds.empty())
        throw std::runtime_error("没有启用的加工工序");
    for (auto kind : kinds)
        parameters.emplace(kind, rebuildFamily(job, kind, index, context, stockPath).params);
    std::vector<TopoDS_Shape> units;
    for (TopoDS_Iterator it(context.obstacleShape); it.More(); it.Next())
        units.push_back(it.Value());
    std::string error;
    auto stock = SurfaceFinishingMachiningJobService::buildPlanningStock(job.definition().stock, units,
                                                                         &error, nullptr, stockPath);
    if (stock.IsNull())
        throw std::runtime_error(error);
    const auto simulation = ipw::IpwExecutor().execute(
        job, stock, MachiningJobStorage::definitionFingerprint(job.definition()).toStdString(),
        ipw::kHybridIpwEngineId, options);
    if (!simulation.complete() || !simulation.finalCheckpoint || !simulation.finalCheckpoint->dexelStock)
        throw std::runtime_error("IPW: " + simulation.message);
    return simulation;
}

void selectPrefix(MachiningJobDefinition &def, const std::string &last)
{
    const auto ordered = MachiningOperationService::operations(def);
    const auto end = std::find_if(ordered.begin(), ordered.end(),
                                  [&](const auto &op) { return op.id == last && op.enabled; });
    if (end == ordered.end())
        throw std::runtime_error("请选择已启用的仿真终点工序");
    std::set<std::string> included;
    for (auto it = ordered.begin(); it != end + 1; ++it)
        if (it->enabled)
            included.insert(it->id);
    const auto trim = [&](auto &ops) {
        for (auto &op : ops)
            op.enabled = included.contains(op.id);
    };
    trim(def.holeOperations);
    trim(def.pocketOperations);
    trim(def.profileOperations);
    trim(def.finishingOperations);
}

void confirmSetups(MachiningJob &job, toolpath::MoldSetupWorkFrames frames, const QJsonArray &ids)
{
    auto &def = job.definition();
    for (const auto &setup : def.setups)
        frames.setConfirmed(setup.face, false);
    for (const auto &id : ids)
    {
        const auto *setup = job.setupById(id.toString().toStdString());
        if (!setup || !setup->valid || !frames.setConfirmed(setup->face, true))
            throw std::runtime_error("无法确认无效装夹");
    }
    MachiningJobApplicationService::captureSetups(
        def, frames, QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs).toStdString(),
        "browser-session");
}

QJsonArray ncMachiningWarnings(const MachiningJob &job, JobOperationKind kind)
{
    QJsonArray warnings;
    if (kind == JobOperationKind::Hole && job.holeResult())
    {
        const auto &batch = *job.holeResult();
        if (!batch.coverage.complete())
            warnings.append(
                QJsonObject{{"line", 0}, {"message", QString::fromStdString(batch.coverage.summary())}});
        if (!batch.productionSafety.complete())
            warnings.append(QJsonObject{{"line", 0}, {"message", "刀具装配实体扫掠审计未完成"}});
    }
    if (kind == JobOperationKind::SurfaceFinishing)
        warnings.append(QJsonObject{{"line", 0},
                                    {"message", "雕刻刀刃接触采用采样计算，连续刀柄与夹具碰撞仍需复核"}});
    return warnings;
}

QJsonObject exportPrograms(const MachiningJob &job, toolpath::MoldSetupWorkFrames frames,
                           const std::map<JobOperationKind, toolpath::ToolpathParams> &parameters)
{
    const auto result = MachiningProgramPartitioner().partition(job);
    if (!result.assembly.ok() && !result.assembly.issues.empty())
        throw std::runtime_error(result.assembly.issues.front().message);
    if (!result.ok())
        throw std::runtime_error(result.issues.empty() ? "没有可导出的程序" : result.issues.front().message);
    const auto operations = MachiningOperationService::operations(job.definition());
    QJsonArray programs;
    int ncErrors = 0;
    for (const auto &part : result.partitions)
    {
        const auto op = std::find_if(operations.begin(), operations.end(),
                                     [&](const auto &o) { return o.id == part.formalOperationIds.front(); });
        const auto *controller = job.toolControllerById(part.toolControllerId);
        const auto *setup = job.setupById(part.setupId);
        if (op == operations.end() || !controller || !setup || !setup->confirmation.confirmed)
            throw std::runtime_error("程序缺少工序、刀具或装夹");
        frames.setConfirmed(setup->face, true);
        const auto frame = frames.forFace(setup->face)->machiningSetup();
        auto options = job.definition().post.options;
        const QString filename = QString("%1-%2-T%3.nc")
                                     .arg(part.index + 1, 3, 10, QChar('0'))
                                     .arg(toolpath::workCoordinateCode(part.workOffset))
                                     .arg(part.toolNumber);
        options.programName += " " + filename.toStdString();
        options.toolId = part.toolId;
        options.toolName = controller->tool.toolName;
        std::string error;
        auto params = parameters.at(op->kind);
        if (op->kind == JobOperationKind::Pocket)
        {
            const auto &pockets = job.definition().pocketOperations;
            const auto pocket = std::find_if(pockets.begin(), pockets.end(),
                                             [&](const auto &item) { return item.id == op->id; });
            params = toolpath::bindPocketToolParameters(
                pocket->parameters.value_or(job.definition().pocketParameters), controller->tool);
        }
        const auto validation = toolpath::ToolpathProgramValidator().validate(part.program, params);
        if (!validation.ok())
            throw std::runtime_error(validation.summary());
        std::ostringstream buffer;
        if (!toolpath::ReferenceNcGCodePostProcessor().write(buffer, part.program, params, frame, options,
                                                             error))
            throw std::runtime_error(error);
        const auto nc = buffer.str();
        QJsonArray ids;
        for (const auto &id : part.formalOperationIds)
            ids.append(QString::fromStdString(id));
        const auto program = reviewNc(nc, filename, QString::fromStdString(setup->id),
                                      QString::fromStdString(options.toolName), ids, ncMachiningWarnings(job, op->kind));
        ncErrors += static_cast<int>(program["errors"].toArray().size());
        programs.append(program);
    }
    writeNcReport(programs);
    return {{"programCount", programs.size()}, {"ncErrors", ncErrors}};
}

QJsonObject planHoles(MachiningJob &job, const Model &model, const StepIndexer &index,
                      const tooling::ToolLibrary &library, const toolpath::MoldSetupWorkFrames &frames,
                      const toolpath::HoleRapidPlanningContext &context)
{
    Bbox bounds;
    for (const auto &unit : model.units)
        bounds.merge(unit.bbox);
    const auto holes = FeatureRecognizer().recognizeHoleGeometries(model, index);
    const auto inputs =
        tooling::HoleMachiningInputMapper().resolve(holes.geometries, bounds, *context.moldFrame);
    if (inputs.empty())
        throw std::runtime_error("没有可规划的孔");
    const auto generated =
        tooling::HoleAutomaticToolpath().generate(library, inputs, job.definition().holeParameters, &context);
    if (generated.batch.toProgram().empty() || !generated.batch.validation.ok())
        throw std::runtime_error(generated.batch.diagnostics.empty()
                                     ? generated.batch.validation.summary()
                                     : generated.batch.diagnostics.back().message);
    HoleJobCaptureRequest capture;
    capture.definition = job.definition();
    capture.inputs = &inputs;
    capture.automaticSelection = &generated.selection;
    capture.generated = &generated.batch;
    capture.setupFrames = &frames;
    capture.toolLibraryId = library.id().toStdString();
    std::string error;
    if (!HoleMachiningJobService().capture(job, capture, &error))
        throw std::runtime_error(error);
    const auto &coverage = generated.batch.coverage;
    QJsonArray issues;
    for (const auto &issue : coverage.blockingIssues)
        issues.append(QString::fromStdString(issue.message));
    for (const auto &issue : coverage.warningIssues)
        issues.append(QString::fromStdString(issue.message));
    return {{"input", coverage.inputHoleSegmentCount},
            {"covered", coverage.coveredHoleSegmentCount},
            {"eligible", coverage.sixFaceHoleSegmentCount},
            {"issues", issues}};
}
} // namespace

void captureProject(const QJsonObject &request, const tooling::HoleAutomaticToolpathResult &generated,
                    const std::vector<toolpath::HoleMachiningInput> &inputs,
                    const toolpath::MoldSetupWorkFrames &frames, const tooling::ToolLibrary &library)
{
    HoleJobCaptureRequest capture;
    auto &def = capture.definition;
    def.id = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    const QString name = QFileInfo(request["sourceName"].toString("model.step")).fileName();
    def.name = QFileInfo(name).completeBaseName().toStdString();
    def.part.sourcePath = name.toStdString();
    def.part.sourceFingerprint = modelHash(request).toStdString();
    def.activeToolLibraryId = library.id().toStdString();
    def.createdAt = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs).toStdString();
    def.modifiedAt = def.createdAt;
    capture.inputs = &inputs;
    capture.automaticSelection = &generated.selection;
    capture.generated = &generated.batch;
    capture.setupFrames = &frames;
    capture.toolLibraryId = def.activeToolLibraryId;
    manufacturing::MachiningJob job;
    std::string error;
    if (!HoleMachiningJobService().capture(job, capture, &error))
        throw std::runtime_error(error);
    saveDefinition(job.definition());
}

QJsonObject processProject(const QJsonObject &request, const StepIndexer &index, const Model &model,
                           const tooling::ToolLibrary &library,
                           const toolpath::MoldSetupWorkFrames &modelFrames,
                           const toolpath::HoleRapidPlanningContext &context)
{
    auto frames = modelFrames;
    manufacturing::MachiningJob job;
    auto &def = job.definition();
    QString error;
    if (!request.contains("document") &&
        (request["action"].toString() == "project" || request["action"].toString() == "plan" ||
         request["action"].toString() == "holes"))
    {
        def.id = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
        def.part.sourcePath =
            QFileInfo(request["sourceName"].toString("model.step")).fileName().toStdString();
        def.name = QFileInfo(QString::fromStdString(def.part.sourcePath)).completeBaseName().toStdString();
        def.part.sourceFingerprint = modelHash(request).toStdString();
        def.createdAt = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs).toStdString();
        MachiningJobApplicationService::captureSetups(def, frames, {});
    }
    else if (!MachiningJobStorage::deserializeDefinition(
                 QJsonDocument(request["document"].toObject()).toJson(), def, &error))
        throw std::runtime_error(error.toStdString());
    if (def.part.sourceFingerprint != modelHash(request).toStdString())
        throw std::runtime_error("项目引用的 STEP 指纹与当前模型不一致");
    // The desktop reload path clears legacy collision references. Keep them in portable project versions.
    auto collisionScene = std::move(def.collisionScene);
    MachiningJobApplicationService::rebindLoadedSetups(def, frames);
    def.collisionScene = std::move(collisionScene);
    for (const auto &setup : def.setups)
        frames.setConfirmed(setup.face, setup.confirmation.confirmed);
    const auto action = request["action"].toString();
    const auto stockPath = request["stockFile"].toString().toStdString();
    if (action == "stock")
    {
        const auto expected = request["stockHash"].toString();
        if (stockPath.empty() || expected.isEmpty() ||
            expected != MachiningJobStorage::sourceFileFingerprint(request["stockFile"].toString(), &error))
            throw std::runtime_error("毛坯 STEP 指纹校验失败");
        def.stock.sourceKind = StockSourceKind::StepModel;
        def.stock.sourcePath = QFileInfo(request["stockName"].toString()).fileName().toStdString();
        def.stock.sourceFingerprint = expected.toStdString();
    }
    const bool computing = action == "plan" || action == "rebuild" || action == "simulate" ||
                           action == "export" || action == "holes" || action == "replay";
    if (computing && def.stock.sourceKind == StockSourceKind::StepModel && stockPath.empty())
        throw std::runtime_error("请在加工项目中绑定与毛坯指纹匹配的 STEP");
    if (computing && !def.collisionScene.models.empty())
        throw std::runtime_error("该项目含外部碰撞模型，请在桌面端复核");
    if (request.contains("appendFile"))
        appendGeneratedHole(def, request["appendFile"].toString());
    if (action == "setups")
        confirmSetups(job, frames, request["confirmedSetups"].toArray());
    QJsonObject result;
    if (action == "holes")
        result.insert("holeCoverage", planHoles(job, model, index, library, frames, context));
    if (action == "plan")
    {
        planFeature(job, request, model, index, library, frames, context.obstacleShape);
    }
    if (action == "rebuild" && !request["tool"].toString().isEmpty())
        changePocketTool(job, request, library);
    def.modifiedAt = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs).toStdString();
    saveDefinition(def);
    result.insert("name", QString::fromStdString(def.name));
    if (action == "project" || action == "plan" || action == "setups" || action == "stock" ||
        action == "holes")
        return result;
    if (action == "simulate" || action == "replay" || action == "export")
    {
        const int quality = request["quality"].toInt(5);
        ipw::IpwExecutionOptions options;
        if (action != "export") {
            if (quality < 1 || quality > 10)
                throw std::runtime_error("仿真质量超出范围");
            // Use the complete enabled project before trimming a prefix so that
            // inspecting a move reproduces the same grid as its project replay.
            double diameter = std::numeric_limits<double>::infinity();
            for (const auto &op : MachiningOperationService::operations(def)) {
                const auto *controller = job.toolControllerById(op.toolControllerId);
                if (op.enabled && controller && controller->tool.diameter > 0)
                    diameter = std::min(diameter, controller->tool.diameter);
            }
            const auto profile = toolpath::ipwSimulationQualityProfile(quality, std::isfinite(diameter) ? diameter : 1.0);
            options.pocket.zMapResolution = profile.requestedResolution;
            options.pocket.zMapMaxCellCount = profile.maximumCellCount;
            options.hole = options.pocket;
        }
        const auto through = request["operation"].toString();
        if ((action == "simulate" || action == "replay") && !through.isEmpty())
            selectPrefix(def, through.toStdString());
        if (action == "export")
        {
            for (const auto &op : MachiningOperationService::operations(def))
            {
                const auto *setup = job.setupById(op.setupId);
                if (op.enabled && (!setup || !setup->confirmation.confirmed))
                    throw std::runtime_error("请先核对并确认工序的装夹 " + op.setupId);
            }
        }
        std::map<JobOperationKind, toolpath::ToolpathParams> parameters;
        const auto simulation = executeProject(job, index, context, parameters, stockPath, options);
        if (action == "replay")
            return replayResult(job, !through.isEmpty(), quality);
        if (action == "export")
            return exportPrograms(job, frames, parameters);
        result = stockResult(job, simulation, request, context.obstacleShape);
        result.insert("throughOperation", through);
        return result;
    }

    const auto id = request["operation"].toString().toStdString();
    const auto operations = MachiningOperationService::operations(def);
    const auto it = std::find_if(operations.begin(), operations.end(),
                                 [&](const auto &op) { return op.id == id && op.enabled; });
    if (it == operations.end())
        throw std::runtime_error("请选择已启用的工序");
    // Regenerate an isolated copy; publish its updated pocket fingerprint in the complete project.
    auto selected = job;
    auto& subset = selected.definition();
    selectOperation(subset.holeOperations, id);
    selectOperation(subset.pocketOperations, id);
    selectOperation(subset.profileOperations, id);
    selectOperation(subset.finishingOperations, id);
    auto [program, params, report] = rebuildFamily(selected, it->kind, index, context, stockPath);
    if (it->kind == JobOperationKind::Pocket) {
        const auto actual = std::find_if(subset.pocketOperations.begin(), subset.pocketOperations.end(),
                                         [&](const auto& op){ return op.id == id; });
        const auto original = std::find_if(def.pocketOperations.begin(), def.pocketOperations.end(),
                                           [&](const auto& op){ return op.id == id; });
        std::string exportError;
        if (!toolpath::PocketToolpathExport().writeJsonl(selected.currentPocketResult()->operations, params,
                                                         "toolpath.jsonl", exportError))
            throw std::runtime_error(exportError);
        result.insert("pocketJsonl", true);
        *original = *actual;
        saveDefinition(def);
    }
    const auto *setup = frames.forFace(program.operations.front().setupFace);
    if (!setup || !setup->valid)
        throw std::runtime_error("装夹坐标无效");
    auto options = def.post.options;
    if (it->kind == JobOperationKind::Pocket) {
        const auto* controller = job.toolControllerById(it->toolControllerId);
        options.toolId = controller->tool.toolId;
        options.toolName = controller->tool.toolName;
    }
    options.programName += " " + id;
    std::string why;
    if (!toolpath::ToolpathProgramExport().writeGCode(program, params, setup->machiningSetup(), options,
                                                      "reference.nc", why))
        throw std::runtime_error(why);
    QJsonArray path;
    for (const auto &op : program.operations)
        for (const auto &move : op.moves)
            path.append(QJsonArray{point(move.from), point(move.to)});
    const auto data = QJsonDocument(path).toJson(QJsonDocument::Compact);
    QSaveFile file("path.json");
    if (!file.open(QIODevice::WriteOnly) || file.write(data) != data.size() || !file.commit())
        throw std::runtime_error("无法写入刀路预览");
    result.insert("operation", QString::fromStdString(id));
    const auto *boundTool = job.toolControllerById(it->toolControllerId);
    const auto toolName = QString::fromStdString(boundTool ? boundTool->tool.toolName : options.toolName);
    result.insert("tool", toolName);
    result.insert("singleNcReview", true);
    result.insert("ncErrors", reviewSingleNc(toolpath::workCoordinateCode(setup->workOffset), toolName,
                                              {QString::fromStdString(id)}, ncMachiningWarnings(selected, it->kind)));
    result.insert("moves", path.size());
    result.insert("validation", QString::fromStdString(report.summary()));
    return result;
}
} // namespace oap::worker
