// 孔刀路应用服务实现：归一化参数、选择算法、处理异常/取消并执行专用验证。
//
// 模块归属:oap_toolpath_workflow 的孔服务子目录。编排链路为:输入过滤 →
// 参数归一化 → 注册表解析算法 → 算法生成(异常/取消兜底)→ 孔专用验证
// → 覆盖率审计 → 生产安全证据汇总。所有失败都以批内诊断形式返回,
// 不在服务边界外抛出领域异常。
#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <exception>
#include <utility>

#include "toolpath/hole/HoleProgramAdapter.h"
#include "toolpath/hole/HoleToolpathValidator.h"
#include "toolpath/workflow/detail/ToolpathServiceSupport.h"

namespace oap::toolpath {
// 汇总生产安全证据:实体扫掠模式与碰撞计数取自工艺参数/生成结果,
// 碰撞场景中的实体按角色(工件/夹具/机床)分别计数;没有显式场景时,
// 退化为把单一障碍物视为工件。
HoleProductionSafetyEvidence buildHoleProductionSafetyEvidence(
    const ToolpathParams& params,
    const HoleRapidPlanningContext* context,
    const HoleMachiningResult& machining)
{
    HoleProductionSafetyEvidence evidence;
    evidence.solidSweepMode = params.hole.collisionEnvelopeMode ==
        HoleCollisionEnvelopeMode::ToolAssemblySolidSweep;
    evidence.solidSweepCheckCount = machining.solidSweepCheckCount;
    evidence.detectedCollisionCount =
        machining.cuttingMoveCollisionCount +
        machining.nonCuttingMoveCollisionCount +
        machining.unsafeRapidLinkCount;
    if (!context)
        return evidence;
    if (!context->collisionScene.empty()) {
        for (const CollisionBody& body : context->collisionScene.bodies()) {
            switch (body.role) {
                case CollisionBodyRole::Workpiece:
                    ++evidence.workpieceBodyCount;
                    break;
                case CollisionBodyRole::Fixture:
                    ++evidence.fixtureBodyCount;
                    break;
                case CollisionBodyRole::Machine:
                    ++evidence.machineBodyCount;
                    break;
            }
        }
    } else if (!context->obstacleShape.IsNull()) {
        evidence.workpieceBodyCount = 1;
    }
    return evidence;
}

HoleToolpathService::HoleToolpathService()
    : HoleToolpathService(defaultToolpathAlgorithmRegistry())
{
}

HoleToolpathService::HoleToolpathService(
    std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
    std::string algorithmId)
    : m_algorithms(algorithms ? std::move(algorithms)
                              : defaultToolpathAlgorithmRegistry()),
      m_algorithmId(std::move(algorithmId))
{
}

ToolpathProgram HoleToolpathBatch::toProgram() const
{
    return makeToolpathProgram(machining.operations);
}

HoleToolpathBatch HoleToolpathService::generate(
    const std::vector<HoleMachiningInput>& inputs,
    const ToolpathParams& params,
    const HoleRapidPlanningContext* rapidContext,
    const std::vector<HoleMachiningStrategy>* strategyOverrides,
    const ToolpathGenerationControl* control) const
{
    // 输入过滤:未就绪的孔(入口未解析/装夹未解析)从生成中剔除,并为
    // 每个被剔除项生成诊断;策略覆盖列表按输入位置同步收缩。
    std::vector<HoleMachiningInput> readyHoles;
    std::vector<HoleMachiningStrategy> readyOverrides;
    std::vector<std::size_t> sourceIndices;
    std::vector<ToolpathDiagnostic> mappingDiagnostics;
    readyHoles.reserve(inputs.size());
    sourceIndices.reserve(inputs.size());
    if (strategyOverrides)
        readyOverrides.reserve(inputs.size());

    for (std::size_t index = 0; index < inputs.size(); ++index) {
        if (inputs[index].ready()) {
            readyHoles.push_back(inputs[index]);
            sourceIndices.push_back(index);
            if (strategyOverrides) {
                readyOverrides.push_back(
                    index < strategyOverrides->size()
                        ? (*strategyOverrides)[index]
                        : HoleMachiningStrategy::Auto);
            }
            continue;
        }

        ToolpathDiagnostic diagnostic;
        diagnostic.sourceIndex = static_cast<int>(index);
        if (inputs[index].mappingState ==
            HoleMachiningMappingState::UnresolvedEntry) {
            diagnostic.code = ToolpathDiagnosticCode::HoleOpeningUnresolved;
            diagnostic.severity = ToolpathDiagnosticSeverity::Error;
            diagnostic.message =
                "hole entry is unresolved and cannot enter toolpath generation";
        } else {
            diagnostic.code = ToolpathDiagnosticCode::HoleOutsideSixFace;
            diagnostic.severity = ToolpathDiagnosticSeverity::Warning;
            diagnostic.message =
                "hole setup is unresolved and was excluded from toolpath generation";
        }
        mappingDiagnostics.push_back(std::move(diagnostic));
    }

    const std::vector<HoleMachiningStrategy>* selectedOverrides =
        strategyOverrides ? &readyOverrides : nullptr;
    HoleToolpathBatch batch = generateReadyInputs(
        readyHoles, params, rapidContext, selectedOverrides, control);

    // 生成内部使用「已就绪列表」的本地索引;这里把操作/覆盖的来源孔索引
    // 映射回原始输入索引,使诊断与 UI 对应关系保持一致。
    const auto remapSource = [&sourceIndices](int localIndex) {
        return localIndex >= 0 &&
                localIndex < static_cast<int>(sourceIndices.size())
            ? static_cast<int>(
                  sourceIndices[static_cast<std::size_t>(localIndex)])
            : -1;
    };
    for (HoleMachiningOperation& operation : batch.machining.operations) {
        operation.sourceHoleIndex = remapSource(operation.sourceHoleIndex);
        for (int& coveredSource : operation.coveredSourceHoleIndices)
            coveredSource = remapSource(coveredSource);
        operation.coveredSourceHoleIndices.erase(
            std::remove(
                operation.coveredSourceHoleIndices.begin(),
                operation.coveredSourceHoleIndices.end(), -1),
            operation.coveredSourceHoleIndices.end());
    }
    batch.machining.skippedCount +=
        static_cast<int>(mappingDiagnostics.size());
    batch.diagnostics.insert(
        batch.diagnostics.end(),
        mappingDiagnostics.begin(), mappingDiagnostics.end());
    // 用「原始输入全集」做覆盖率审计,保证被剔除的孔也计入 skipped 口径。
    batch.coverage = auditHoleCoverage(
        inputs, batch.machining.operations,
        batch.machining.setupCount > 0
            ? static_cast<int>(inputs.size()) : 0,
        batch.machining.skippedCount,
        batch.diagnostics, batch.machining.diagnostics);
    return batch;
}

HoleToolpathBatch HoleToolpathService::generateReadyInputs(
    const std::vector<HoleMachiningInput>& holes,
    const ToolpathParams& params,
    const HoleRapidPlanningContext* rapidContext,
    const std::vector<HoleMachiningStrategy>* strategyOverrides,
    const ToolpathGenerationControl* control) const
{
    ToolpathParallelScope parallelScope(control);
    // 内核对已就绪孔的主生成流程;先预留覆盖率审计 lambda 供各提前
    // 返回路径统一复用。
    HoleToolpathBatch batch;
    batch.algorithmId = m_algorithmId;
    const auto auditCoverage = [&] {
        batch.coverage = auditHoleCoverage(
            holes, batch.machining.operations,
            batch.machining.setupCount > 0
                ? static_cast<int>(holes.size()) : 0,
            batch.machining.skippedCount,
            batch.diagnostics, batch.machining.diagnostics);
    };
    // 参数归一化:未通过时整批标记为跳过,不再调用算法。
    const bool parametersValid =
        workflow_detail::normalizeBatchParameters(batch, params);

    if (control)
        control->progress(0, holes.size());
    if (control && control->cancelled()) {
        batch.cancelled = true;
        batch.diagnostics.push_back(
            {ToolpathDiagnosticCode::GenerationCancelled,
             ToolpathDiagnosticSeverity::Info,
             "hole generation was cancelled"});
        auditCoverage();
        return batch;
    }
    if (!parametersValid) {
        batch.machining.skippedCount = static_cast<int>(holes.size());
        batch.validation = HoleToolpathValidator().validate(
            batch.machining.operations, batch.effectiveParams);
        auditCoverage();
        return batch;
    }

    // 从注册表解析所选算法;未注册按 Error 诊断处理并整批跳过。
    const IHoleToolpathAlgorithm* algorithm =
        m_algorithms->holeAlgorithm(batch.algorithmId);
    if (!algorithm) {
        batch.machining.skippedCount = static_cast<int>(holes.size());
        batch.diagnostics.push_back(workflow_detail::unavailableAlgorithmDiagnostic(
            "hole", batch.algorithmId));
        batch.validation = HoleToolpathValidator().validate(
            batch.machining.operations, batch.effectiveParams);
        auditCoverage();
        return batch;
    }

    // 算法调用包裹在 try/catch 中:领域异常一律转为失败诊断,保证
    // 服务接口不向外抛异常。
    try {
        batch.machining = algorithm->generate(
            holes, batch.effectiveParams, rapidContext, strategyOverrides,
            control);
    } catch (const std::exception& error) {
        batch.machining.skippedCount = static_cast<int>(holes.size());
        batch.diagnostics.push_back(workflow_detail::failedAlgorithmDiagnostic(
            "hole", error.what()));
    } catch (...) {
        batch.machining.skippedCount = static_cast<int>(holes.size());
        batch.diagnostics.push_back(workflow_detail::failedAlgorithmDiagnostic(
            "hole", nullptr));
    }

    for (HoleMachiningOperation& operation : batch.machining.operations)
        operation.algorithmId = batch.algorithmId;
    // 取消处理:丢弃已完成的部分操作,避免向调用方泄漏不完整结果。
    if (batch.machining.cancelled || (control && control->cancelled())) {
        batch.cancelled = true;
        batch.machining.operations.clear();
        batch.diagnostics.push_back(
            {ToolpathDiagnosticCode::GenerationCancelled,
             ToolpathDiagnosticSeverity::Info,
             "hole generation was cancelled; completed operations were discarded"});
    }
    batch.validation = HoleToolpathValidator().validate(
        batch.machining.operations, batch.effectiveParams);
    auditCoverage();
    batch.productionSafety = buildHoleProductionSafetyEvidence(
        batch.effectiveParams, rapidContext, batch.machining);
    if (control)
        control->progress(holes.size(), holes.size());
    return batch;
}

} // namespace oap::toolpath
