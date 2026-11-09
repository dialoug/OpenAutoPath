// RecognitionPresentationCoordinator 实现（oap_app_presentation）。
// 持有推导出的识别颜色/边界并应用到场景，见 RecognitionPresentationCoordinator.h。
#include "app/presentation/RecognitionPresentationCoordinator.h"

#include "app/presentation/SceneController.h"

#include <array>
#include <algorithm>

#include <Quantity_NameOfColor.hxx>

namespace oap::app::presentation {
namespace {

const std::array<Quantity_Color, 3> kAxisPlaneColors{{
    Quantity_Color(Quantity_NOC_BLUE),
    Quantity_Color(Quantity_NOC_GREEN),
    Quantity_Color(Quantity_NOC_RED),
}};

} // namespace

void RecognitionPresentationCoordinator::reset()
{
    m_colors.clear();
    m_boundaries.clear();
}

RecognitionPresentationStats
RecognitionPresentationCoordinator::rebuild(
    const std::array<ThreeAxisTraceStats, 3>& axisStats,
    const std::vector<ThreeAxisFeatureSubset>& subsets,
    const SceneController& scene)
{
    reset();
    RecognitionPresentationStats result;
    // 三轴平面面按 X/Y/Z 顺序染成蓝/绿/红,并统计其中未被隐藏的面数。
    for (std::size_t axisIndex = 0;
         axisIndex < axisStats.size() && axisIndex < kAxisPlaneColors.size();
         ++axisIndex) {
        for (const std::string& faceId : axisStats[axisIndex].planeFaceIds) {
            m_colors.emplace(faceId, kAxisPlaneColors[axisIndex]);
            if (!scene.isHidden(faceId))
                ++result.visiblePlaneFaces;
        }
    }
    m_boundaries.reserve(subsets.size());
    for (const ThreeAxisFeatureSubset& subset : subsets) {
        m_boundaries.push_back(FeatureBoundary{subset.faceIds});
        const Quantity_Color color = subset.featureKind ==
                AxisWireTraceFeatureKind::BlindSlot
            ? featureColor(FeatureKind::Slot)
            : featureColor(FeatureKind::Pocket);
        for (const std::string& faceId : subset.faceIds)
            m_colors.insert_or_assign(faceId, color);
    }
    for (const auto& [faceId, color] : m_colors) {
        (void)color;
        if (!scene.isHidden(faceId))
            ++result.visibleTraceFaces;
    }
    result.visibleTraceFaces = std::max(
        0, result.visibleTraceFaces - result.visiblePlaneFaces);
    return result;
}

void RecognitionPresentationCoordinator::apply(
    SceneController& scene,
    const RecognitionResult& recognition,
    const std::unordered_set<std::string>& throughStructureFaces,
    const Quantity_Color& throughStructureColor) const
{
    for (const auto& [faceId, color] : m_colors) {
        if (!scene.isHidden(faceId))
            scene.setFaceColor(faceId, color);
    }
    for (const std::string& faceId : throughStructureFaces) {
        if (!scene.isHidden(faceId))
            scene.setFaceColor(faceId, throughStructureColor);
    }
    for (const auto& [faceId, kind] : recognition.faceKind) {
        if (!scene.isHidden(faceId))
            scene.setFaceColor(faceId, featureColor(kind));
    }
    scene.setFeatureBoundaries(m_boundaries);
}

} // namespace oap::app::presentation
