// 每文档的识别状态与派生的加工输入。
#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "analysis/Model.h"
#include "recognition/FeatureRecognizer.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"
#include "toolpath/hole/HoleMachiningInput.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"
#include "toolpath/workflow/ToolpathWorkflow.h"

namespace oap::app {

// 每文档的识别会话:保存特征识别的原始结果、派生的孔/槽腔
// 加工输入与线框追踪子集;由文档会话持有,
// 换文档时必须 reset()。访问器全部暴露内部可变引用。
class RecognitionSession {
public:
    // 清空全部识别状态,回到"未识别"的初始状态。
    void reset();

    RecognitionResult& result() noexcept { return m_result; }
    const RecognitionResult& result() const noexcept { return m_result; }
    bool& hasResult() noexcept { return m_hasResult; }
    bool hasResult() const noexcept { return m_hasResult; }

    std::vector<toolpath::HoleMachiningInput>& holeToolpathInputs() noexcept
    {
        return m_holeToolpathInputs;
    }
    const std::vector<toolpath::HoleMachiningInput>&
    holeToolpathInputs() const noexcept
    {
        return m_holeToolpathInputs;
    }
    bool& hasHoleToolpathInputs() noexcept
    {
        return m_hasHoleToolpathInputs;
    }
    bool hasHoleToolpathInputs() const noexcept
    {
        return m_hasHoleToolpathInputs;
    }
    HoleGeometryRecognitionResult& holeGeometryProfile() noexcept
    {
        return m_holeGeometryProfile;
    }
    const HoleGeometryRecognitionResult& holeGeometryProfile() const noexcept
    {
        return m_holeGeometryProfile;
    }

    std::unordered_set<std::string>& baseFaces() noexcept
    {
        return m_baseFaces;
    }
    const std::unordered_set<std::string>& baseFaces() const noexcept
    {
        return m_baseFaces;
    }
    std::unordered_set<std::string>& throughStructureFaces() noexcept
    {
        return m_throughStructureFaces;
    }
    const std::unordered_set<std::string>&
    throughStructureFaces() const noexcept
    {
        return m_throughStructureFaces;
    }
    std::unordered_set<std::string>& axisParallelPlaneFaces() noexcept
    {
        return m_axisParallelPlaneFaces;
    }
    const std::unordered_set<std::string>&
    axisParallelPlaneFaces() const noexcept
    {
        return m_axisParallelPlaneFaces;
    }
    std::unordered_set<std::string>& combinedWireTraceFaces() noexcept
    {
        return m_combinedWireTraceFaces;
    }
    const std::unordered_set<std::string>&
    combinedWireTraceFaces() const noexcept
    {
        return m_combinedWireTraceFaces;
    }
    std::vector<ThreeAxisFeatureSubset>& wireTraceFeatureSubsets() noexcept
    {
        return m_wireTraceFeatureSubsets;
    }
    const std::vector<ThreeAxisFeatureSubset>&
    wireTraceFeatureSubsets() const noexcept
    {
        return m_wireTraceFeatureSubsets;
    }
    std::vector<toolpath::RootInnerWirePocketSource>&
    pocketSources() noexcept
    {
        return m_pocketSources;
    }
    const std::vector<toolpath::RootInnerWirePocketSource>&
    pocketSources() const noexcept
    {
        return m_pocketSources;
    }
    std::vector<toolpath::RootOuterWireProfileSource>&
    profileSources() noexcept
    {
        return m_profileSources;
    }
    const std::vector<toolpath::RootOuterWireProfileSource>&
    profileSources() const noexcept
    {
        return m_profileSources;
    }
    std::vector<toolpath::ProfileRegion>& profileRegionCache() noexcept
    {
        return m_profileRegionCache;
    }
    const std::vector<toolpath::ProfileRegion>&
    profileRegionCache() const noexcept
    {
        return m_profileRegionCache;
    }
    bool& hasProfileRegionCache() noexcept
    {
        return m_hasProfileRegionCache;
    }
    bool hasProfileRegionCache() const noexcept
    {
        return m_hasProfileRegionCache;
    }
    toolpath::PocketRegionWorkflowResult& pocketRegionCache() noexcept
    {
        return m_pocketRegionCache;
    }
    const toolpath::PocketRegionWorkflowResult&
    pocketRegionCache() const noexcept
    {
        return m_pocketRegionCache;
    }
    bool& hasPocketRegionCache() noexcept
    {
        return m_hasPocketRegionCache;
    }
    bool hasPocketRegionCache() const noexcept
    {
        return m_hasPocketRegionCache;
    }
private:
    RecognitionResult m_result;
    bool m_hasResult = false;
    std::vector<toolpath::HoleMachiningInput> m_holeToolpathInputs;
    bool m_hasHoleToolpathInputs = false;
    HoleGeometryRecognitionResult m_holeGeometryProfile;
    std::unordered_set<std::string> m_baseFaces;
    std::unordered_set<std::string> m_throughStructureFaces;
    std::unordered_set<std::string> m_axisParallelPlaneFaces;
    std::unordered_set<std::string> m_combinedWireTraceFaces;
    std::vector<ThreeAxisFeatureSubset> m_wireTraceFeatureSubsets;
    std::vector<toolpath::RootInnerWirePocketSource> m_pocketSources;
    std::vector<toolpath::RootOuterWireProfileSource> m_profileSources;
    std::vector<toolpath::ProfileRegion> m_profileRegionCache;
    bool m_hasProfileRegionCache = false;
    toolpath::PocketRegionWorkflowResult m_pocketRegionCache;
    bool m_hasPocketRegionCache = false;
};

} // namespace oap::app
