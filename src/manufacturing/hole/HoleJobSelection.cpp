// Reconstruct a hole tool-selection plan from persisted Job definitions.
#include "manufacturing/hole/HoleMachiningJobService.h"
#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <utility>

#include "manufacturing/hole/detail/HoleJobTypes.h"
#include "manufacturing/hole/detail/HoleJobEnablement.h"

namespace oap::manufacturing {
using namespace hole_job_detail;

//
// 从持久化孔工序定义恢复按全局序列排序的自动选刀计划。
tooling::HoleToolSelectionPlan
HoleMachiningJobService::selectionPlan(const MachiningJob& job) const
{
    tooling::HoleToolSelectionPlan result;
    MachiningJobDefinition definition = job.definition();
    result.physicalHoleCount = definition.recognizedPhysicalHoleCount;
    std::set<std::string> setupIds;
    std::vector<const HoleOperationDefinition*> operations;
    // 只取已启用孔工序,并按 sequence 稳定排序,保证批次顺序与加工顺序一致。
    for (const HoleOperationDefinition& operation :
         definition.holeOperations) {
        if (operation.enabled)
            operations.push_back(&operation);
    }
    std::stable_sort(
        operations.begin(), operations.end(),
        [](const HoleOperationDefinition* left,
           const HoleOperationDefinition* right) {
            return left->sequence < right->sequence;
        });
    // 每个已启用孔工序转换为一个 HoleToolBatch:绑定装夹面/相位/刀具,
    // 把各目标映射为 HoleToolTask，保留来源及覆盖范围。
    for (const HoleOperationDefinition* operation : operations) {
        const SetupDefinition* setup = job.setupById(operation->setupId);
        const JobToolController* controller =
            job.toolControllerById(operation->toolControllerId);
        if (!setup || !controller)
            continue;
        tooling::HoleToolBatch batch;
        batch.setupFace = setup->face;
        batch.setupIndex = setup->setupIndex;
        batch.phase = operation->phase;
        batch.tool = controller->tool;
        for (const HoleTargetReference& target : operation->targets) {
            if (target.sourceHoleIndex < 0)
                continue;
            tooling::HoleToolTask task;
            const toolpath::HoleMachiningInput& input = target.hole;
            // 目标孔未就绪(开口映射失败等)时,生成错误诊断并从计划中排除。
            if (!input.ready()) {
                toolpath::ToolpathDiagnostic diagnostic;
                diagnostic.code = input.mappingState ==
                        toolpath::HoleMachiningMappingState::UnresolvedEntry
                    ? toolpath::ToolpathDiagnosticCode::HoleOpeningUnresolved
                    : toolpath::ToolpathDiagnosticCode::HoleOutsideSixFace;
                diagnostic.severity =
                    toolpath::ToolpathDiagnosticSeverity::Error;
                diagnostic.operationId = operation->id;
                diagnostic.sourceIndex = target.sourceHoleIndex;
                diagnostic.message =
                    "unresolved Job target was excluded from the editable selection plan";
                result.diagnostics.push_back(std::move(diagnostic));
                continue;
            }
            task.hole = input;
            task.sourceHoleIndex = static_cast<std::size_t>(
                target.sourceHoleIndex);
            task.physicalHoleIndex = static_cast<std::size_t>(
                std::max(0, target.physicalHoleIndex));
            task.strategy = target.strategy;
            for (const int sourceIndex :
                 target.coveredSourceHoleIndices) {
                if (sourceIndex >= 0) {
                    task.coveredSourceHoleIndices.push_back(
                        static_cast<std::size_t>(sourceIndex));
                }
            }
            batch.tasks.push_back(std::move(task));
        }
        if (!batch.tasks.empty()) {
            setupIds.insert(operation->setupId);
            result.batches.push_back(std::move(batch));
        }
    }
    result.setupCount = static_cast<int>(setupIds.size());
    return result;
}

} // namespace oap::manufacturing
