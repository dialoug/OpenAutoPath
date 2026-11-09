#include "RecognitionSession.h"

namespace oap::app {

// 全量复位:原始结果、派生输入、标签与缓存全部还原为初值,
// 对应的"是否已生成"标志位一并置假。
void RecognitionSession::reset()
{
    m_result = RecognitionResult{};
    m_hasResult = false;
    m_holeToolpathInputs.clear();
    m_hasHoleToolpathInputs = false;
    m_holeGeometryProfile = HoleGeometryRecognitionResult{};
    m_baseFaces.clear();
    m_throughStructureFaces.clear();
    m_axisParallelPlaneFaces.clear();
    m_combinedWireTraceFaces.clear();
    m_wireTraceFeatureSubsets.clear();
    m_pocketSources.clear();
    m_profileSources.clear();
    m_profileRegionCache.clear();
    m_hasProfileRegionCache = false;
    m_pocketRegionCache = toolpath::PocketRegionWorkflowResult{};
    m_hasPocketRegionCache = false;
}

} // namespace oap::app
