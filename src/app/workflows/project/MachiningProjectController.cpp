// MachiningProjectController 实现（oap_app_workflow_cam）。
// 不依赖 MainWindow 状态的 .oapjob 初始化、捕获与加载，见 MachiningProjectController.h。
#include "MachiningProjectController.h"

#include <algorithm>
#include <utility>

#include <BRep_Builder.hxx>
#include <QDateTime>
#include <QFileInfo>
#include <TopoDS_Compound.hxx>

#include "manufacturing/hole/HoleMachiningJobService.h"
#include "manufacturing/job/MachiningJobApplicationService.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/pocket/PocketMachiningJobService.h"
#include "manufacturing/profile/ProfileMachiningJobService.h"
#include "manufacturing/finishing/SurfaceFinishingMachiningJobService.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"

namespace oap::app {
namespace {

// 槽腔实例键:对面列表排序后以 '|' 拼接,使同一组面无论顺序
// 如何都得到稳定标识,用于跨会话/跨请求匹配既有槽腔目标。
std::string pocketInstanceKey(std::vector<std::string> faceIds)
{
    std::sort(faceIds.begin(), faceIds.end());
    std::string key;
    for (const std::string& faceId : faceIds) {
        if (!key.empty())
            key += '|';
        key += faceId;
    }
    return key;
}

// 把校验报告格式化为用户可读文本:先逐条追加 Error、再追加 Warning,
// 超过 maximumIssues 条后仅提示剩余数量,避免刷屏。
QString validationMessage(
    const toolpath::ToolpathValidationReport& report,
    int maximumIssues = 5)
{
    QString message = QString::fromStdString(report.summary());
    const int issueCount = static_cast<int>(report.issues.size());
    const int displayedCount = std::min(maximumIssues, issueCount);
    int appended = 0;
    const auto appendSeverity =
        [&](toolpath::ToolpathIssueSeverity severity) {
            for (const toolpath::ToolpathIssue& issue : report.issues) {
                if (appended >= displayedCount ||
                    issue.severity != severity) {
                    continue;
                }
                message += issue.severity ==
                        toolpath::ToolpathIssueSeverity::Error
                    ? QStringLiteral("\nERROR ")
                    : QStringLiteral("\nWARN ");
                message += QString::fromStdString(issue.code);
                if (!issue.operationId.empty()) {
                    message += QStringLiteral(" [") +
                        QString::fromStdString(issue.operationId) +
                        QLatin1Char(']');
                }
                if (issue.moveIndex >= 0) {
                    message += QStringLiteral(" move %1")
                        .arg(issue.moveIndex);
                }
                message += QStringLiteral(": ") +
                    QString::fromStdString(issue.message);
                ++appended;
            }
        };
    appendSeverity(toolpath::ToolpathIssueSeverity::Error);
    appendSeverity(toolpath::ToolpathIssueSeverity::Warning);
    if (issueCount > appended) {
        message += QStringLiteral("\n... %1 more issue(s)")
            .arg(issueCount - appended);
    }
    return message;
}

void assignError(QString* error, QString value)
{
    if (error)
        *error = std::move(value);
}

} // namespace

// 初始化/校验 Job 定义:零件路径与源指纹必须有效;
// 若旧 Job 引用的是另一份零件(指纹不同),视为换了零件,先整体清空。
bool MachiningProjectController::ensureDefinition(
    manufacturing::MachiningJob& job,
    const MachiningProjectDefinitionContext& context,
    QString* error)
{
    if (error)
        error->clear();
    if (context.sourcePath.isEmpty()) {
        assignError(error, QStringLiteral("当前零件路径为空。"));
        return false;
    }
    QString fingerprintError;
    const QString fingerprint =
        manufacturing::MachiningJobStorage::sourceFileFingerprint(
            context.sourcePath, &fingerprintError);
    if (!fingerprintError.isEmpty() || fingerprint.isEmpty()) {
        assignError(error, std::move(fingerprintError));
        return false;
    }

    if (!job.definition().part.sourceFingerprint.empty() &&
        job.definition().part.sourceFingerprint !=
            fingerprint.toStdString()) {
        job.clear();
    }
    manufacturing::MachiningJobDefinition& definition = job.definition();
    const QFileInfo source(context.sourcePath);
    const std::string partName =
        source.completeBaseName().toUtf8().toStdString();
    definition.schemaVersion =
        manufacturing::kMachiningJobSchemaVersion;
    // 仅填充缺失字段(id/名称/创建时间沿用首次写入值,保留历史语义)。
    if (definition.id.empty())
        definition.id = "job-" + partName;
    if (definition.name.empty())
        definition.name = partName;
    if (definition.createdAt.empty()) {
        definition.createdAt = QDateTime::currentDateTimeUtc()
            .toString(Qt::ISODateWithMs).toStdString();
    }
    definition.modifiedAt = QDateTime::currentDateTimeUtc()
        .toString(Qt::ISODateWithMs).toStdString();
    definition.part.sourcePath =
        context.sourcePath.toUtf8().toStdString();
    definition.part.sourceFingerprint = fingerprint.toStdString();
    // Existing families share this part identity. Adding another family must
    // not silently migrate it and stale otherwise unchanged runtime results.
    if (definition.part.recognitionProfileVersion.empty())
        definition.part.recognitionProfileVersion = "phase-3-unified-ipw";
    definition.activeToolLibraryId =
        context.activeToolLibraryId.toStdString();
    definition.post.id = "reference.nc";
    definition.post.options = context.gcodeOptions;
    auto stock = context.stock;
    if (stock.sourceKind == manufacturing::StockSourceKind::StepModel &&
        stock.sourceKind == definition.stock.sourceKind &&
        stock.sourcePath == definition.stock.sourcePath && stock.sourceFingerprint.empty()) {
        stock.sourceFingerprint = definition.stock.sourceFingerprint;
    }
    definition.stock = std::move(stock);
    return true;
}

// 槽腔捕获:输入为完整加工工序,Job 里的工序定义需要的是
// 稳定面/根内环引用,因此先做一次"目标引用"转换再交给捕获服务。
bool MachiningProjectController::capturePocketOperations(
    manufacturing::MachiningJob& job,
    const MachiningProjectPocketCaptureRequest& request,
    QString* error)
{
    if (error)
        error->clear();
    if (!request.operations || request.operations->empty() ||
        !request.effectiveParams || !request.setupFrames) {
        assignError(error, QStringLiteral("当前没有可捕获的槽腔预览。"));
        return false;
    }
    auto candidate = job;
    if (!ensureDefinition(candidate, request.definition, error))
        return false;

    std::vector<manufacturing::PocketTargetReference> targets;
    targets.reserve(request.operations->size());
    for (const auto& generated : *request.operations) {
        manufacturing::PocketTargetReference target;
        target.kind = generated.kind;
        target.sourceFaceIds = generated.sourceFaceIds;
        target.floorFaceId = generated.floorFaceId;
        // 优先沿用 Job 中已捕获的根内环目标(保证稳定面引用跨次捕获不变);
        // 没有则从 rootSources 里按同一实例键匹配新根内环,最后才退化为普通目标。
        const std::string key =
            pocketInstanceKey(generated.sourceFaceIds);
        const auto retained = std::find_if(
            candidate.definition().pocketOperations.begin(),
            candidate.definition().pocketOperations.end(),
            [&](const manufacturing::PocketOperationDefinition& operation) {
                return pocketInstanceKey(
                           operation.target.sourceFaceIds) == key &&
                       operation.target.floorFaceId ==
                           generated.floorFaceId;
            });
        if (retained != candidate.definition().pocketOperations.end() &&
            retained->target.sourceKind ==
                manufacturing::PocketRegionSourceKind::RootInnerWire) {
            targets.push_back(retained->target);
            continue;
        }
        if (request.rootSources) {
            const auto root = std::find_if(
                request.rootSources->begin(), request.rootSources->end(),
                [&](const toolpath::RootInnerWirePocketSource& source) {
                    return pocketInstanceKey(source.sourceFaceIds) == key &&
                           source.rootFaceId ==
                               generated.floorFaceId;
                });
            if (root != request.rootSources->end()) {
                target.sourceKind =
                    manufacturing::PocketRegionSourceKind::RootInnerWire;
                target.rootFaceId = root->rootFaceId;
                target.rootWireId = root->rootWireId;
                target.allowOpenSectionContours =
                    root->allowOpenSectionContours;
                target.openBoundaryFaceCount =
                    root->openBoundaryFaceCount;
            }
        }
        targets.push_back(std::move(target));
    }

    manufacturing::PocketJobCaptureRequest capture;
    capture.operations = request.operations;
    capture.targets = &targets;
    capture.parameters = request.effectiveParams;
    capture.setupFrames = request.setupFrames;
    capture.toolLibraryId = request.definition.operationToolLibraryId;
    std::string captureError;
    if (!manufacturing::PocketMachiningJobService().capture(
            candidate, capture, &captureError)) {
        assignError(error, QString::fromStdString(captureError));
        return false;
    }
    job = std::move(candidate);
    return true;
}

bool MachiningProjectController::captureProfilePreview(
    manufacturing::MachiningJob& job,
    const MachiningProjectProfileCaptureRequest& request,
    QString* error)
{
    if (error) error->clear();
    if (!request.operations || request.operations->empty() ||
        !request.effectiveParams ||
        !request.setupFrames) {
        assignError(error, QStringLiteral("当前没有可捕获的外轮廓预览。"));
        return false;
    }
    if (!ensureDefinition(job, request.definition, error))
        return false;

    std::vector<manufacturing::ProfileTargetReference> targets;
    targets.reserve(request.operations->size());
    for (const toolpath::ProfileToolpathOperation& operation :
         *request.operations) {
        const toolpath::RootOuterWireProfileSource* source = nullptr;
        if (request.rootSources) {
            const auto found = std::find_if(
                request.rootSources->begin(), request.rootSources->end(),
                [&](const toolpath::RootOuterWireProfileSource& item) {
                    return item.rootFaceId == operation.rootFaceId &&
                           item.rootWireId == operation.rootWireId;
                });
            if (found != request.rootSources->end())
                source = &*found;
        }
        if (!source) {
            const auto retained = std::find_if(
                job.definition().profileOperations.begin(),
                job.definition().profileOperations.end(),
                [&](const manufacturing::ProfileOperationDefinition& item) {
                    return item.target.rootFaceId == operation.rootFaceId &&
                           item.target.rootWireId == operation.rootWireId;
                });
            if (retained != job.definition().profileOperations.end()) {
                targets.push_back(retained->target);
                continue;
            }
            assignError(
                error,
                QStringLiteral("外轮廓工序缺少可持久化的根 outer wire 来源。"));
            return false;
        }
        manufacturing::ProfileTargetReference target;
        target.axisLabel = source->axisLabel;
        target.normalDirection = source->normalDirection;
        target.sourceFaceIds = source->sourceFaceIds;
        target.machiningFaceIds = source->machiningFaceIds;
        target.terminalFaceIds = source->terminalFaceIds;
        target.mainOuterFaceIds = source->mainOuterFaceIds;
        target.rootFaceId = source->rootFaceId;
        target.rootWireId = source->rootWireId;
        targets.push_back(std::move(target));
    }

    manufacturing::ProfileJobCaptureRequest capture;
    capture.operations = request.operations;
    capture.targets = &targets;
    capture.parameters = request.effectiveParams;
    capture.setupFrames = request.setupFrames;
    capture.toolLibraryId = request.definition.operationToolLibraryId;
    std::string captureError;
    if (!manufacturing::ProfileMachiningJobService().capture(
            job, capture, &captureError)) {
        assignError(error, QString::fromStdString(captureError));
        return false;
    }
    return true;
}

// 保存前同步:以"当前上下文"为准刷新 Job 的零件/后处理/毛坯字段,
// 并写入工位坐标系,确保落盘的 .oapjob 与界面状态一致。
bool MachiningProjectController::synchronizeForSave(
    manufacturing::MachiningJob& job,
    const MachiningProjectSaveRequest& request,
    QString* error)
{
    if (!request.setupFrames) {
        assignError(error, QStringLiteral("装夹工作坐标为空。"));
        return false;
    }
    if (!ensureDefinition(job, request.definition, error))
        return false;

    manufacturing::MachiningJobSaveSynchronizationRequest synchronization;
    synchronization.modifiedAt = QDateTime::currentDateTimeUtc()
        .toString(Qt::ISODateWithMs).toStdString();
    synchronization.confirmationTime = synchronization.modifiedAt;
    synchronization.post = job.definition().post;
    synchronization.post.id = "reference.nc";
    synchronization.post.options = request.definition.gcodeOptions;
    synchronization.stock = request.definition.stock;
    synchronization.activeToolLibraryId =
        request.definition.activeToolLibraryId.toStdString();
    synchronization.setupFrames = request.setupFrames;
    std::string synchronizationError;
    if (!manufacturing::MachiningJobApplicationService::synchronizeForSave(
            job, synchronization, &synchronizationError)) {
        assignError(
            error, QString::fromStdString(synchronizationError));
        return false;
    }
    return true;
}

// 完整加载流程先完成读盘、零件身份校验和 Setup 重绑定，再把内存候选
// 交给共享重建入口。这样文件加载与正式工序编辑使用同一套生成门禁。
MachiningProjectLoadResult
MachiningProjectController::loadAndRegenerate(
    const MachiningProjectLoadRequest& request)
{
    MachiningProjectLoadResult result;
    manufacturing::MachiningJobDefinition definition;
    if (!manufacturing::MachiningJobStorage::load(
            request.filePath, definition, &result.error)) {
        result.failure = MachiningProjectLoadFailure::File;
        return result;
    }
    if (!request.model || request.model->faces.empty() ||
        !request.indexer || !request.setupFrames ||
        request.currentSourcePath.isEmpty()) {
        result.failure = MachiningProjectLoadFailure::PartRequired;
        result.error = QStringLiteral("请先打开加工项目引用的零件：\n%1")
            .arg(QString::fromUtf8(definition.part.sourcePath));
        return result;
    }

    QString fingerprintError;
    const QString currentFingerprint =
        manufacturing::MachiningJobStorage::sourceFileFingerprint(
            request.currentSourcePath, &fingerprintError);
    if (!fingerprintError.isEmpty()) {
        result.failure = MachiningProjectLoadFailure::PartFingerprint;
        result.error = QStringLiteral("无法校验当前零件：\n%1")
            .arg(fingerprintError);
        return result;
    }
    if (!definition.part.sourceFingerprint.empty() &&
        currentFingerprint.toStdString() !=
            definition.part.sourceFingerprint) {
        result.failure = MachiningProjectLoadFailure::PartFingerprint;
        result.error = QStringLiteral(
            "当前 STEP 与加工项目中的零件指纹不一致。\n"
            "项目引用：%1\n当前零件：%2")
            .arg(QString::fromUtf8(definition.part.sourcePath),
                 request.currentSourcePath);
        return result;
    }

    // 重新绑定工位坐标系后,把定义转交给内存候选重建入口。
    manufacturing::MachiningJobApplicationService::rebindLoadedSetups(
        definition, *request.setupFrames);
    manufacturing::MachiningJob job;
    job.definition() = std::move(definition);
    return regenerateJob(std::move(job), request);
}

// 内存候选 Job 的全领域事务式重建。所有生成都写入 result.job；任一领域
// 失败只返回候选与诊断，活动工作区由调用方保持不变。
MachiningProjectLoadResult MachiningProjectController::regenerateJob(
    manufacturing::MachiningJob job,
    const MachiningProjectLoadRequest& request)
{
    MachiningProjectLoadResult result;
    if (!request.model || request.model->faces.empty() ||
        !request.indexer || !request.setupFrames) {
        result.failure = MachiningProjectLoadFailure::PartRequired;
        result.error = QStringLiteral("重建加工项目需要当前零件、拓扑索引和装夹坐标。");
        return result;
    }
    if (!job.validationErrors().empty()) {
        result.failure = MachiningProjectLoadFailure::File;
        result.error = QString::fromStdString(job.validationErrors().front());
        return result;
    }
    result.job = std::move(job);
    result.hasHoleDefinitions =
        !result.job.definition().holeOperations.empty();
    result.hasPocketDefinitions =
        !result.job.definition().pocketOperations.empty();
    result.hasProfileDefinitions =
        !result.job.definition().profileOperations.empty();
    result.hasFinishingDefinitions =
        !result.job.definition().finishingOperations.empty();
    if (!result.hasHoleDefinitions && !result.hasPocketDefinitions &&
        !result.hasProfileDefinitions && !result.hasFinishingDefinitions) {
        result.failure = MachiningProjectLoadFailure::EmptyProject;
        result.error =
            QStringLiteral("加工项目中没有孔、槽腔、外轮廓或雕刻加工工序。");
        return result;
    }

    // 为孔刀路快速规划装配碰撞场景:把所有单位形状收集为障碍物,
    // 供再生时做避让计算。
    toolpath::HoleRapidPlanningContext rapidContext;
    if (request.moldFrame && request.moldFrame->isValid)
        rapidContext.moldFrame = request.moldFrame;
    TopoDS_Compound obstacles;
    BRep_Builder builder;
    builder.MakeCompound(obstacles);
    int obstacleCount = 0;
    for (const Unit& unit : request.model->units) {
        const TopoDS_Shape* shape =
            request.indexer->shapeForUnit(unit.unit_id);
        if (!shape || shape->IsNull())
            continue;
        builder.Add(obstacles, *shape);
        rapidContext.collisionScene.addBody(
            unit.unit_id, toolpath::CollisionBodyRole::Workpiece,
            *shape);
        ++obstacleCount;
    }
    if (obstacleCount > 0)
        rapidContext.obstacleShape = obstacles;

    if (!result.hasHoleDefinitions)
        result.job.clearHoleResult();
    if (result.hasHoleDefinitions) {
        result.holes = manufacturing::HoleMachiningJobService()
            .regenerate(result.job, &rapidContext);
        if (result.holes.cancelled || !result.job.hasHoleResult() ||
            !result.holes.validation.ok()) {
            result.failure = MachiningProjectLoadFailure::HoleRegeneration;
            const QString detail = result.holes.validation.issues.empty()
                ? QStringLiteral("项目中没有可重建的有效孔工序。")
                : validationMessage(result.holes.validation);
            result.error = QStringLiteral("孔加工 Job 重建失败：\n%1")
                .arg(detail);
            return result;
        }
    }
    if (!result.hasPocketDefinitions)
        result.job.clearPocketResult();
    if (result.hasPocketDefinitions) {
        result.pockets = manufacturing::PocketMachiningJobService()
            .regenerate(result.job, *request.indexer);
        if (result.pockets.cancelled ||
            result.pockets.operations.empty() ||
            !result.pockets.validation.ok()) {
            result.failure = MachiningProjectLoadFailure::PocketRegeneration;
            const QString detail = result.pockets.validation.issues.empty()
                ? QStringLiteral(
                      "稳定面或根内环引用无法重建有效槽腔工序。")
                : validationMessage(result.pockets.validation);
            result.error = QStringLiteral("槽腔 Job 重建失败：\n%1")
                .arg(detail);
            return result;
        }
    }
    if (!result.hasProfileDefinitions)
        result.job.clearProfileResult();
    if (result.hasProfileDefinitions) {
        result.profiles = manufacturing::ProfileMachiningJobService()
            .regenerate(result.job, *request.indexer);
        const toolpath::ToolpathValidationReport validation =
            toolpath::ToolpathProgramValidator().validate(
                result.profiles.toProgram(),
                result.profiles.effectiveParams);
        if (result.profiles.cancelled ||
            result.profiles.operations.empty() || !validation.ok()) {
            result.failure =
                MachiningProjectLoadFailure::ProfileRegeneration;
            result.error = QStringLiteral(
                "外轮廓 Job 重建失败：稳定根面或 outer wire 引用无法生成有效刀路。");
            return result;
        }
    }
    if (!result.hasFinishingDefinitions)
        result.job.clearFinishingResult();
    if (result.hasFinishingDefinitions) {
        result.finishing = manufacturing::SurfaceFinishingMachiningJobService()
            .regenerate(result.job, *request.indexer);
        const bool hasEnabledFinishing = std::any_of(
            result.job.definition().finishingOperations.begin(),
            result.job.definition().finishingOperations.end(),
            [](const auto& operation) { return operation.enabled; });
        if (result.finishing.cancelled || !result.finishing.validation.ok() ||
            (hasEnabledFinishing && !result.job.currentFinishingResult())) {
            result.failure = MachiningProjectLoadFailure::SurfaceFinishingRegeneration;
            result.error = QStringLiteral("雕刻加工重建失败：%1")
                .arg(validationMessage(result.finishing.validation));
            return result;
        }
    }
    return result;
}

MachiningProjectLoadResult MachiningProjectController::restoreSnapshot(
    manufacturing::MachiningJob job)
{
    MachiningProjectLoadResult result;
    const std::vector<std::string> errors = job.validationErrors();
    if (!errors.empty()) {
        result.failure = MachiningProjectLoadFailure::File;
        result.error = QString::fromStdString(errors.front());
        return result;
    }
    result.hasHoleDefinitions =
        !job.definition().holeOperations.empty();
    result.hasPocketDefinitions =
        !job.definition().pocketOperations.empty();
    result.hasProfileDefinitions =
        !job.definition().profileOperations.empty();
    result.hasFinishingDefinitions = !job.definition().finishingOperations.empty();
    if (const auto* batch = job.finishingResult())
        result.finishing = *batch;
    if (const toolpath::HoleToolpathBatch* batch = job.holeResult())
        result.holes = *batch;
    if (const toolpath::PocketToolpathBatch* batch = job.pocketResult())
        result.pockets = *batch;
    if (const toolpath::ProfileToolpathBatch* batch = job.profileResult())
        result.profiles = *batch;
    result.job = std::move(job);
    return result;
}

} // namespace oap::app
