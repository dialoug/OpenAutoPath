// HoleJobSourceSynchronizer 实现（oap_app_workflow_cam）。
// 将已加载 Job 的权威孔几何同步回当前识别会话，见 HoleJobSourceSynchronizer.h。
#include "app/workflows/hole/HoleJobSourceSynchronizer.h"

#include <algorithm>
#include <iterator>
#include <map>
#include <utility>

namespace oap::app {
namespace {

bool isHoleKind(FeatureKind kind) noexcept
{
    return kind == FeatureKind::ThroughHole ||
        kind == FeatureKind::BlindHole ||
        kind == FeatureKind::CountersinkHole;
}

FeatureKind mergedHoleKind(FeatureKind current,
                           FeatureKind candidate) noexcept
{
    if (current == FeatureKind::CountersinkHole ||
        candidate == FeatureKind::CountersinkHole) {
        return FeatureKind::CountersinkHole;
    }
    if (current == FeatureKind::ThroughHole ||
        candidate == FeatureKind::ThroughHole) {
        return FeatureKind::ThroughHole;
    }
    return FeatureKind::BlindHole;
}

std::vector<std::string> instanceFaceIds(const oap::toolpath::HoleMachiningInput& hole)
{
    std::vector<std::string> result = hole.sourceFaceIds.empty()
        ? hole.stageWallFaceIds : hole.sourceFaceIds;
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

} // namespace

void synchronizeLoadedJobHoleSources(
    const std::vector<oap::toolpath::HoleMachiningInput>& jobHoles,
    RecognitionResult& recognition,
    std::vector<oap::toolpath::HoleMachiningInput>& holeCache,
    bool& hasHoleCache)
{
    // 用 Job 孔数据整体替换当前识别孔几何，并同步孔缓存。
    recognition.holeGeometries.clear();
    recognition.holeGeometries.reserve(jobHoles.size());
    for (const oap::toolpath::HoleMachiningInput& hole : jobHoles) {
        recognition.holeGeometries.push_back(
            static_cast<const RecognizedHoleGeometry&>(hole));
    }
    holeCache = jobHoles;
    hasHoleCache = !holeCache.empty();

    std::vector<RecognitionResult::Instance> nonHoleInstances;
    nonHoleInstances.reserve(recognition.instances.size());
    for (RecognitionResult::Instance& instance : recognition.instances) {
        if (!isHoleKind(instance.kind))
            nonHoleInstances.push_back(std::move(instance));
    }

    // 按去重后的面 ID 聚合孔实例；同一组面只保留一条实例，
    // 多个阶段类型冲突时合并为更宽泛的类型（沉孔 > 通孔 > 盲孔）。
    std::map<std::vector<std::string>, RecognitionResult::Instance>
        holeInstances;
    for (const oap::toolpath::HoleMachiningInput& hole : jobHoles) {
        std::vector<std::string> faceIds = instanceFaceIds(hole);
        if (faceIds.empty())
            continue;
        const auto [found, inserted] = holeInstances.try_emplace(faceIds);
        RecognitionResult::Instance& instance = found->second;
        if (inserted) {
            instance.kind = hole.kind;
            instance.faceIds = std::move(faceIds);
        } else {
            instance.kind = mergedHoleKind(instance.kind, hole.kind);
        }
        instance.manual = instance.manual || hole.manuallyConfirmed;
    }

    // 重建识别实例表：孔实例在前、原非孔实例在后，再统一去重与派生重建。
    recognition.instances.clear();
    recognition.instances.reserve(
        holeInstances.size() + nonHoleInstances.size());
    for (auto& [faceIds, instance] : holeInstances) {
        (void)faceIds;
        recognition.instances.push_back(std::move(instance));
    }
    recognition.instances.insert(
        recognition.instances.end(),
        std::make_move_iterator(nonHoleInstances.begin()),
        std::make_move_iterator(nonHoleInstances.end()));
    recognition.dedupInstances();
    recognition.rebuildDerived();
}

} // namespace oap::app
