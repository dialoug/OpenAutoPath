// Automatic hole operation definition capture.
#include "manufacturing/hole/HoleMachiningJobService.h"
#include <algorithm>
#include <map>
#include <tuple>
#include <utility>

#include "manufacturing/hole/detail/HoleJobTypes.h"
#include "manufacturing/hole/detail/HoleJobFoundation.h"
#include "manufacturing/hole/detail/HoleJobEnablement.h"

namespace oap::manufacturing::hole_job_detail {

// 从生成的刀路批次推导 Job 工序状态:校验失败/无结果/存在错误诊断 -> Error;
// 存在警告 -> Warning;否则 Generated。
JobOperationState generatedState(
    const toolpath::HoleToolpathBatch& batch)
{
    if (!batch.validation.ok() ||
        (batch.machining.operations.empty() && !batch.coverage.complete()))
        return JobOperationState::Error;
    const bool diagnosticError = std::any_of(
        batch.diagnostics.begin(), batch.diagnostics.end(),
        [](const toolpath::ToolpathDiagnostic& item) {
            return item.severity == toolpath::ToolpathDiagnosticSeverity::Error;
        });
    if (diagnosticError)
        return JobOperationState::Error;
    const bool diagnosticWarning = std::any_of(
        batch.diagnostics.begin(), batch.diagnostics.end(),
        [](const toolpath::ToolpathDiagnostic& item) {
            return item.severity ==
                toolpath::ToolpathDiagnosticSeverity::Warning;
        });
    if (batch.validation.warningCount > 0 ||
        !batch.machining.warnings.empty() ||
        diagnosticWarning) {
        return JobOperationState::Warning;
    }
    return JobOperationState::Generated;
}

// 自动模式捕获:按选择计划批次创建工序。装夹面(索引+面)与刀具(指纹)已在
// 定义中存在的直接复用,否则新建 Setup/ToolController;批次按顺序编号。
void addAutomaticDefinitions(
    MachiningJobDefinition& definition,
    const HoleJobCaptureRequest& request,
    JobOperationState state)
{
    const tooling::HoleToolSelectionPlan& plan = *request.automaticSelection;
    std::map<std::tuple<int, MoldSetupFace>, std::string> setups;
    std::map<std::string, std::string> controllers;
    for (const SetupDefinition& setup : definition.setups) {
        setups.emplace(
            std::make_tuple(setup.setupIndex, setup.face),
            setup.id);
    }
    for (const JobToolController& controller :
         definition.toolControllers) {
        if (controller.libraryId != request.toolLibraryId.value_or(definition.activeToolLibraryId)) continue;
        controllers.emplace(
            controllerKey(controller.tool), controller.id);
    }
    int sequence = 0;
    // 每个批次生成一条 HoleOperationDefinition:复用/新建 Setup 与刀具
    // 控制器后,把任务列表转为目标引用并计算输入指纹。
    for (const tooling::HoleToolBatch& batch : plan.batches) {
        const auto setupKey = std::make_tuple(batch.setupIndex, batch.setupFace);
        auto setupIt = setups.find(setupKey);
        if (setupIt == setups.end()) {
            const std::string id = ensureSetup(
                definition, batch.setupIndex, batch.setupFace,
                request.setupFrames);
            setupIt = setups.emplace(setupKey, id).first;
        }

        const std::string currentControllerKey =
            controllerKey(batch.tool);
        auto controllerIt =
            controllers.find(currentControllerKey);
        if (controllerIt == controllers.end()) {
            JobToolController controller;
            controller.id = controllerId(
                batch.tool, definition.toolControllers.size(),
                currentControllerKey);
            controller.libraryId = request.toolLibraryId.value_or(definition.activeToolLibraryId);
            controller.tool = batch.tool;
            controllerIt = controllers.emplace(
                currentControllerKey, controller.id).first;
            definition.toolControllers.push_back(std::move(controller));
        }

        HoleOperationDefinition operation;
        operation.id = "hole-operation-" + std::to_string(sequence + 1);
        operation.setupId = setupIt->second;
        operation.toolControllerId = controllerIt->second;
        operation.phase = batch.phase;
        operation.sequence = sequence++;
        operation.state = state;
        operation.targets.reserve(batch.tasks.size());
        for (const tooling::HoleToolTask& task : batch.tasks) {
            operation.targets.push_back(makeTarget(
                task.hole, static_cast<int>(task.sourceHoleIndex),
                static_cast<int>(task.physicalHoleIndex), task.strategy,
                task.coveredSourceHoleIndices));
        }
        operation.inputFingerprint = operationFingerprint(
            definition, operation);
        definition.holeOperations.push_back(std::move(operation));
    }
}

} // namespace oap::manufacturing::hole_job_detail
