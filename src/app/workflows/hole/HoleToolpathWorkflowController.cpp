// HoleToolpathWorkflowController 实现（oap_app_workflow_cam）。
// 生成孔程序前的与 UI 无关的准备与原子提交，见 HoleToolpathWorkflowController.h。
#include "app/workflows/hole/HoleToolpathWorkflowController.h"
#include <QCoreApplication>

#include "app/workflows/shared/detail/ToolpathRequestBuilder.h"
#include "manufacturing/hole/HoleMachiningJobService.h"
#include "tooling/adapter/hole/HoleStageIdentity.h"

#include <QDateTime>
#include <QFileInfo>

namespace oap::app {

HoleToolpathGeometryPreparation
HoleToolpathWorkflowController::prepareGeometry(
    const std::vector<toolpath::HoleMachiningInput>& recognized,
    const Model& model,
    const StepIndexer& indexer,
    const Bbox& moldBounds,
    const MoldCoordinateFrame& moldFrame)
{
    HoleToolpathGeometryPreparation result;
    // 步骤 1：解析外包络入口——把识别孔组恢复到模具外包络面并确定装夹方向。
    result.envelope = toolpath::HoleMoldEnvelopeEntryResolver().resolve(
        recognized, moldBounds, moldFrame);
    result.sources.reserve(result.envelope.holes.size());
    // 步骤 2：为每个孔生成稳定的阶段键（stageKey），
    // 作为 Job 捕获与识别源匹配的关联依据。
    for (const toolpath::HoleMachiningInput& hole : result.envelope.holes) {
        RecognizedHoleSource source;
        source.hole = hole;
        source.sourceFaceIds = hole.sourceFaceIds;
        source.key = tooling::holeStageKey(hole);
        result.sources.push_back(std::move(source));
    }

    // 步骤 3：收集模型所有非空单元实体作为快速避障的障碍体。
    std::vector<HoleObstacleSource> obstacles;
    obstacles.reserve(model.units.size());
    for (const Unit& unit : model.units) {
        const TopoDS_Shape* shape = indexer.shapeForUnit(unit.unit_id);
        if (!shape || shape->IsNull())
            continue;
        obstacles.push_back({unit.unit_id, *shape});
    }
    // 步骤 4：用障碍体构建快速规划上下文，供孔间连接与 Rapid 避障使用。
    HoleRapidPlanningContextBuildResult rapid =
        buildHoleRapidPlanningContext(
            obstacles, moldFrame.isValid ? &moldFrame : nullptr);
    result.rapidContext = std::move(rapid.context);
    result.obstacleShapeCount = rapid.obstacleBodyCount;
    return result;
}

HoleToolpathCommitResult HoleToolpathWorkflowController::commit(
    const HoleToolpathCommitRequest& request)
{
    HoleToolpathCommitResult result;
    // 前置校验：任何必填输入缺失时直接失败，不触碰现有 Job。
    if (!request.currentJob || !request.inputs ||
        !request.generated || !request.setupFrames || !request.automaticSelection) {
        result.error = QCoreApplication::translate("OpenAutoPath", "孔加工提交请求不完整。");
        return result;
    }
    result.job = *request.currentJob;
    // 在 Job 副本上补齐加工项目定义（零件路径、名称、创建时间、识别版本）。
    if (!MachiningProjectController::ensureDefinition(
            result.job, request.definition, &result.error)) {
        return result;
    }
    const QFileInfo part(request.definition.sourcePath);
    const std::string partName =
        part.completeBaseName().toUtf8().toStdString();
    result.job.definition().id = "job-" + partName;
    result.job.definition().name = partName;
    result.job.definition().createdAt =
        QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)
            .toStdString();
    if (!request.recognitionProfileVersion.isEmpty())
        result.job.definition().part.recognitionProfileVersion =
            request.recognitionProfileVersion.toStdString();

    // 通过 HoleMachiningJobService 捕获生成结果（定义、输入、策略、选刀、
    // 刀路、装夹坐标系）；失败时只返回错误，原 Job 保持基线不变。
    manufacturing::HoleJobCaptureRequest capture;
    capture.definition = result.job.definition();
    capture.inputs = request.inputs;
    capture.automaticSelection = request.automaticSelection;
    capture.generated = request.generated;
    capture.setupFrames = request.setupFrames;
    capture.toolLibraryId = request.definition.operationToolLibraryId;
    std::string captureError;
    if (!manufacturing::HoleMachiningJobService().capture(
            result.job, capture, &captureError)) {
        result.error = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(captureError).toUtf8().constData());
        return result;
    }

    result.ok = true;
    return result;
}

} // namespace oap::app
