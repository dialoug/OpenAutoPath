// 每个模型的孔/槽加工工作区。
//
// 它是可编辑预览 DTO 与持久化 MachiningJob 的所有者。
// 它刻意不包含任何 QWidget、AIS 对象或查看器状态。
#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleMachiningInput.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/profile/program/ProfileToolpathOperation.h"
#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/workflow/ToolpathWorkflow.h"

namespace oap::app {

// 当前激活的刀路预览类型:决定主预览区显示孔还是槽腔。
enum class ActiveToolpathPreview {
    None,
    Pocket,
    Profile,
    Hole,
    SurfaceFinishing,
};

// 槽腔预览条目:key 为面集合实例标识,operation 为待展示的刀路工序。
struct PocketPreviewEntry {
    std::string key;
    toolpath::PocketToolpathOperation operation;
};

// 识别出的孔源:孔几何 + 来源面 + 用于跨批次匹配的阶段键。
struct RecognizedHoleSource {
    toolpath::HoleMachiningInput hole;
    std::vector<std::string> sourceFaceIds;
    std::string key;
};

// 孔预览条目:key 指向其所属 RecognizedHoleSource,
// sourceIndex 为源索引(可能为 -1,表示无对应源)。
struct HolePreviewEntry {
    std::string key;
    int sourceIndex = -1;
    toolpath::HoleMachiningOperation operation;
};

// 单个模型的工作区会话:由工程会话拥有,随模型切换重置;
// 不包含任何 Qt 控件或 OCCT 展示对象,仅供工作流控制器读写。
class MachiningWorkspaceSession {
public:
    // 切换到新模型时清空全部预览数据与 Job。
    void resetForModel();
    // 用当前预览条目重建孔刀路批次并写回 Job 运行期,
    // 使后续保存/导出看到的是用户编辑后的状态。
    void synchronizeHolePreviewBatch();
    // 从 Job 运行期的孔结果反向重建预览条目与预览参数。
    void rebuildHolePreviewEntriesFromJob();
    std::vector<PocketPreviewEntry>& pocketPreviewEntries() noexcept
    {
        return m_pocketPreviewEntries;
    }
    const std::vector<PocketPreviewEntry>&
    pocketPreviewEntries() const noexcept
    {
        return m_pocketPreviewEntries;
    }
    toolpath::ToolpathParams& pocketPreviewParams() noexcept
    {
        return m_pocketPreviewParams;
    }
    const toolpath::ToolpathParams& pocketPreviewParams() const noexcept
    {
        return m_pocketPreviewParams;
    }
    toolpath::ToolpathParams& pocketPreviewEffectiveParams() noexcept
    {
        return m_pocketPreviewEffectiveParams;
    }
    const toolpath::ToolpathParams&
    pocketPreviewEffectiveParams() const noexcept
    {
        return m_pocketPreviewEffectiveParams;
    }
    bool& hasPocketPreviewParams() noexcept
    {
        return m_hasPocketPreviewParams;
    }
    bool hasPocketPreviewParams() const noexcept
    {
        return m_hasPocketPreviewParams;
    }
    bool& pocketPreviewActive() noexcept { return m_pocketPreviewActive; }
    bool pocketPreviewActive() const noexcept { return m_pocketPreviewActive; }

    std::vector<toolpath::ProfileToolpathOperation>&
    profilePreviewOperations() noexcept
    {
        return m_profilePreviewOperations;
    }
    const std::vector<toolpath::ProfileToolpathOperation>&
    profilePreviewOperations() const noexcept
    {
        return m_profilePreviewOperations;
    }
    toolpath::ToolpathParams& profilePreviewParams() noexcept
    {
        return m_profilePreviewParams;
    }
    const toolpath::ToolpathParams& profilePreviewParams() const noexcept
    {
        return m_profilePreviewParams;
    }
    toolpath::ToolpathParams& profilePreviewEffectiveParams() noexcept
    {
        return m_profilePreviewEffectiveParams;
    }
    const toolpath::ToolpathParams&
    profilePreviewEffectiveParams() const noexcept
    {
        return m_profilePreviewEffectiveParams;
    }
    bool& hasProfilePreviewParams() noexcept
    {
        return m_hasProfilePreviewParams;
    }
    bool hasProfilePreviewParams() const noexcept
    {
        return m_hasProfilePreviewParams;
    }

    ActiveToolpathPreview& activePreview() noexcept { return m_activePreview; }
    ActiveToolpathPreview activePreview() const noexcept
    {
        return m_activePreview;
    }

    std::vector<RecognizedHoleSource>& recognizedHoleSources() noexcept
    {
        return m_recognizedHoleSources;
    }
    const std::vector<RecognizedHoleSource>&
    recognizedHoleSources() const noexcept
    {
        return m_recognizedHoleSources;
    }
    std::vector<HolePreviewEntry>& holePreviewEntries() noexcept
    {
        return m_holePreviewEntries;
    }
    const std::vector<HolePreviewEntry>& holePreviewEntries() const noexcept
    {
        return m_holePreviewEntries;
    }
    std::unordered_set<int>& highlightedHoleStageSources() noexcept
    {
        return m_highlightedHoleStageSources;
    }
    const std::unordered_set<int>&
    highlightedHoleStageSources() const noexcept
    {
        return m_highlightedHoleStageSources;
    }

    manufacturing::MachiningJob& job() noexcept { return m_job; }
    const manufacturing::MachiningJob& job() const noexcept { return m_job; }
    toolpath::ToolpathParams& holePreviewParams() noexcept
    {
        return m_holePreviewParams;
    }
    const toolpath::ToolpathParams& holePreviewParams() const noexcept
    {
        return m_holePreviewParams;
    }
    bool& hasHolePreviewParams() noexcept { return m_hasHolePreviewParams; }
    bool hasHolePreviewParams() const noexcept { return m_hasHolePreviewParams; }
    bool& holeToolpathNeedsRegeneration() noexcept
    {
        return m_holeToolpathNeedsRegeneration;
    }
    bool holeToolpathNeedsRegeneration() const noexcept
    {
        return m_holeToolpathNeedsRegeneration;
    }
    bool toolpathNeedsRegeneration(ActiveToolpathPreview family) const noexcept;
    void setToolpathNeedsRegeneration(ActiveToolpathPreview family, bool value) noexcept;
    // Disabled definitions still retain pending edits for saving; simulation
    // may ask only about families with an enabled operation.
    bool hasPendingRegeneration(bool enabledOnly = false) const noexcept;

private:
    std::vector<PocketPreviewEntry> m_pocketPreviewEntries;
    toolpath::ToolpathParams m_pocketPreviewParams;
    toolpath::ToolpathParams m_pocketPreviewEffectiveParams;
    bool m_hasPocketPreviewParams = false;
    bool m_pocketPreviewActive = false;
    std::vector<toolpath::ProfileToolpathOperation> m_profilePreviewOperations;
    toolpath::ToolpathParams m_profilePreviewParams;
    toolpath::ToolpathParams m_profilePreviewEffectiveParams;
    bool m_hasProfilePreviewParams = false;
    ActiveToolpathPreview m_activePreview = ActiveToolpathPreview::None;

    std::vector<RecognizedHoleSource> m_recognizedHoleSources;
    std::vector<HolePreviewEntry> m_holePreviewEntries;
    std::unordered_set<int> m_highlightedHoleStageSources;
    manufacturing::MachiningJob m_job;
    toolpath::ToolpathParams m_holePreviewParams;
    bool m_hasHolePreviewParams = false;
    bool m_holeToolpathNeedsRegeneration = false;
    bool m_pocketToolpathNeedsRegeneration = false;
    bool m_profileToolpathNeedsRegeneration = false;
    bool m_engravingToolpathNeedsRegeneration = false;
};

} // namespace oap::app
