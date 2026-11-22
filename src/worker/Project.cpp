#include "Project.h"
#include "Planning.h"
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
                               const toolpath::HoleRapidPlanningContext &context)
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
        const auto batch = SurfaceFinishingMachiningJobService().regenerate(job, index);
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

QJsonObject simulateProject(MachiningJob &job, const StepIndexer &index,
                            const toolpath::HoleRapidPlanningContext &context)
{
    std::set<JobOperationKind> kinds;
    for (const auto &operation : MachiningOperationService::operations(job.definition()))
        if (operation.enabled)
            kinds.insert(operation.kind);
    if (kinds.empty())
        throw std::runtime_error("没有启用的加工工序");
    for (auto kind : kinds)
        rebuildFamily(job, kind, index, context);
    std::vector<TopoDS_Shape> units;
    for (TopoDS_Iterator it(context.obstacleShape); it.More(); it.Next())
        units.push_back(it.Value());
    std::string error;
    auto stock =
        SurfaceFinishingMachiningJobService::buildPlanningStock(job.definition().stock, units, &error);
    if (stock.IsNull())
        throw std::runtime_error(error);
    const auto simulation = ipw::IpwExecutor().execute(
        job, stock, MachiningJobStorage::definitionFingerprint(job.definition()).toStdString());
    if (!simulation.complete() || !simulation.finalCheckpoint || !simulation.finalCheckpoint->dexelStock)
        throw std::runtime_error("IPW: " + simulation.message);
    Handle(Poly_Triangulation) mesh;
    toolpath::DexelDisplayMeshOptions opts;
    opts.maximumPlanarCellCount = 180000;
    if (!simulation.finalCheckpoint->dexelStock->buildDisplayMesh(mesh, nullptr, &error, {}, opts) ||
        mesh.IsNull())
        throw std::runtime_error("IPW mesh: " + error);
    QJsonArray positions, indices;
    for (int i = 1; i <= mesh->NbNodes(); ++i)
    {
        const auto p = mesh->Node(i);
        positions.append(p.X());
        positions.append(p.Y());
        positions.append(p.Z());
    }
    for (int i = 1; i <= mesh->NbTriangles(); ++i)
    {
        const auto &tri = mesh->Triangle(i);
        for (int k = 1; k <= 3; ++k)
            indices.append(tri.Value(k) - 1);
    }
    QSaveFile file("stock.json");
    const auto bytes =
        QJsonDocument(QJsonArray{QJsonObject{{"id", "IPW"}, {"positions", positions}, {"indices", indices}}})
            .toJson(QJsonDocument::Compact);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
        throw std::runtime_error("无法保存余料网格");
    return {{"volume", simulation.finalCheckpoint->volume},
            {"operations", static_cast<qint64>(simulation.executedNodeCount)}};
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
                           const tooling::ToolLibrary &library, const toolpath::MoldSetupWorkFrames &frames,
                           const toolpath::HoleRapidPlanningContext &context)
{
    manufacturing::MachiningJob job;
    auto &def = job.definition();
    QString error;
    if (!request.contains("document") && (request["action"].toString() == "project" || request["action"].toString() == "plan"))
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
    const auto action = request["action"].toString();
    if (action != "project" && (def.stock.sourceKind == StockSourceKind::StepModel || !def.collisionScene.models.empty()))
        throw std::runtime_error("该项目引用外部毛坯或碰撞模型，请在桌面端计算该版本");
    if (request.contains("appendFile"))
        appendGeneratedHole(def, request["appendFile"].toString());
    if (action == "plan")
    {
        planFeature(job, request, model, index, library, frames, context.obstacleShape);
    }
    if (action == "rebuild" && !request["tool"].toString().isEmpty())
        changePocketTool(job, request, library);
    def.modifiedAt = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs).toStdString();
    saveDefinition(def);
    QJsonObject result{{"name", QString::fromStdString(def.name)}};
    if (request["action"].toString() == "project" || request["action"].toString() == "plan")
        return result;
    if (request["action"].toString() == "simulate")
        return simulateProject(job, index, context);

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
    auto [program, params, report] = rebuildFamily(selected, it->kind, index, context);
    if (it->kind == JobOperationKind::Pocket) {
        const auto actual = std::find_if(subset.pocketOperations.begin(), subset.pocketOperations.end(),
                                         [&](const auto& op){ return op.id == id; });
        const auto original = std::find_if(def.pocketOperations.begin(), def.pocketOperations.end(),
                                           [&](const auto& op){ return op.id == id; });
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
    options.programName = def.name + " " + id;
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
    result.insert("moves", path.size());
    result.insert("validation", QString::fromStdString(report.summary()));
    return result;
}
} // namespace oap::worker
