// MachiningWorkspaceSession 实现（oap_app_workflow_cam）。
// 每个模型的孔/槽加工工作区，见 MachiningWorkspaceSession.h。
#include "MachiningWorkspaceSession.h"

#include <algorithm>
#include <utility>

namespace oap::app {

// 全量复位:预览条目、参数与 Job一并清空,
// 使会话回到"未打开零件"的初始状态。
void MachiningWorkspaceSession::resetForModel()
{
    m_pocketPreviewEntries.clear();
    m_pocketPreviewParams = toolpath::ToolpathParams{};
    m_pocketPreviewEffectiveParams = toolpath::ToolpathParams{};
    m_hasPocketPreviewParams = false;
    m_pocketPreviewActive = false;
    m_profilePreviewOperations.clear();
    m_profilePreviewParams = toolpath::ToolpathParams{};
    m_profilePreviewEffectiveParams = toolpath::ToolpathParams{};
    m_hasProfilePreviewParams = false;
    m_activePreview = ActiveToolpathPreview::None;

    m_recognizedHoleSources.clear();
    m_holePreviewEntries.clear();
    m_highlightedHoleStageSources.clear();
    m_job.clear();
    m_holePreviewParams = toolpath::ToolpathParams{};
    m_hasHolePreviewParams = false;
    m_holeToolpathNeedsRegeneration = false;
    m_pocketToolpathNeedsRegeneration = false;
    m_profileToolpathNeedsRegeneration = false;
    m_engravingToolpathNeedsRegeneration = false;
}

bool MachiningWorkspaceSession::toolpathNeedsRegeneration(ActiveToolpathPreview family) const noexcept
{
    switch (family) {
    case ActiveToolpathPreview::Hole: return m_holeToolpathNeedsRegeneration;
    case ActiveToolpathPreview::Pocket: return m_pocketToolpathNeedsRegeneration;
    case ActiveToolpathPreview::Profile: return m_profileToolpathNeedsRegeneration;
    case ActiveToolpathPreview::SurfaceFinishing: return m_engravingToolpathNeedsRegeneration;
    case ActiveToolpathPreview::None: return false;
    }
    return false;
}

void MachiningWorkspaceSession::setToolpathNeedsRegeneration(ActiveToolpathPreview family, bool value) noexcept
{
    switch (family) {
    case ActiveToolpathPreview::Hole: m_holeToolpathNeedsRegeneration = value; break;
    case ActiveToolpathPreview::Pocket: m_pocketToolpathNeedsRegeneration = value; break;
    case ActiveToolpathPreview::Profile: m_profileToolpathNeedsRegeneration = value; break;
    case ActiveToolpathPreview::SurfaceFinishing: m_engravingToolpathNeedsRegeneration = value; break;
    case ActiveToolpathPreview::None: break;
    }
}

bool MachiningWorkspaceSession::hasPendingRegeneration(bool enabledOnly) const noexcept
{
    const auto relevant = [enabledOnly](const auto& operations) {
        return enabledOnly ? std::any_of(operations.begin(), operations.end(), [](const auto& op) { return op.enabled; })
                           : !operations.empty();
    };
    const auto& definition = m_job.definition();
    return (m_holeToolpathNeedsRegeneration && relevant(definition.holeOperations)) ||
        (m_pocketToolpathNeedsRegeneration && relevant(definition.pocketOperations)) ||
        (m_profileToolpathNeedsRegeneration && relevant(definition.profileOperations)) ||
        (m_engravingToolpathNeedsRegeneration && relevant(definition.finishingOperations));
}

void MachiningWorkspaceSession::synchronizeHolePreviewBatch()
{
    // 即使没有生成任何工序也保留运行期覆盖诊断；
    // holeResult() 有意隐藏空批次。
    toolpath::HoleToolpathBatch batch = m_job.runtime().holeResult;
    batch.machining.operations.clear();
    batch.machining.operations.reserve(m_holePreviewEntries.size());
    for (const HolePreviewEntry& entry : m_holePreviewEntries)
        batch.machining.operations.push_back(entry.operation);
    batch.effectiveParams = m_holePreviewParams;
    m_job.setHoleResult(
        std::move(batch),
        m_job.runtime().holeExecutionFingerprint);
}

// 从 Job 运行期结果重建预览;同时把有效参数同步为预览参数,
// 保证 UI 与已保存的生成结果一致。
void MachiningWorkspaceSession::rebuildHolePreviewEntriesFromJob()
{
    m_holePreviewEntries.clear();
    const toolpath::HoleToolpathBatch& batch = m_job.runtime().holeResult;
    m_holePreviewEntries.reserve(batch.machining.operations.size());
    for (const toolpath::HoleMachiningOperation& operation :
         batch.machining.operations) {
        HolePreviewEntry entry;
        entry.sourceIndex = operation.sourceHoleIndex;
        entry.operation = operation;
        if (entry.sourceIndex >= 0 &&
            entry.sourceIndex <
                static_cast<int>(m_recognizedHoleSources.size())) {
            entry.key = m_recognizedHoleSources[
                static_cast<std::size_t>(entry.sourceIndex)].key;
        }
        m_holePreviewEntries.push_back(std::move(entry));
    }
    m_holePreviewParams = batch.effectiveParams;
    m_hasHolePreviewParams = true;
}

} // namespace oap::app
