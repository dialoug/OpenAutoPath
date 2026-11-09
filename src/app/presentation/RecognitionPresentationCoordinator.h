// 持有推导出的识别颜色/边界并将其应用到场景。
// 模块:oap_app_presentation。识别流程产出每个 feature 的颜色与 FeatureBoundary
// 集合;本协调器暂存这些结果,并在 apply() 时把颜色/边界落到 SceneController
// (面级染色 + 特征边界白线)。
#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Quantity_Color.hxx>

#include "analysis/Model.h"
#include "app/presentation/SceneController.h"
#include "recognition/FeatureRecognizer.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"

namespace oap::app::presentation {

// 识别呈现统计:可见的平面追踪面数 / 可见的特征追踪面数。
struct RecognitionPresentationStats {
    int visiblePlaneFaces = 0;
    int visibleTraceFaces = 0;
};

// 识别呈现协调器:rebuild() 根据三轴追踪结果推导颜色映射并统计可见面,
// apply() 把暂存的颜色与特征边界应用到 SceneController。全部运行在 UI 线程。
class RecognitionPresentationCoordinator {
public:
    // 清空已推导的颜色与边界缓存。
    void reset();
    // 重新推导:把三轴平面面染成蓝/绿/红,把特征子集面染成槽/腔特征色,
    // 并缓存 FeatureBoundary。返回可见面统计(已扣除平面面与特征面的重叠)。
    RecognitionPresentationStats rebuild(
        const std::array<ThreeAxisTraceStats, 3>& axisStats,
        const std::vector<ThreeAxisFeatureSubset>& subsets,
        const SceneController& scene);
    // 把缓存的颜色映射 + 特征边界写入 scene;另叠加贯通结构面颜色与
    // recognition.faceKind 的强制颜色(优先级最高,覆盖前两者)。
    void apply(
        SceneController& scene,
        const RecognitionResult& recognition,
        const std::unordered_set<std::string>& throughStructureFaces,
        const Quantity_Color& throughStructureColor) const;

    bool empty() const noexcept { return m_colors.empty(); }

private:
    std::unordered_map<std::string, Quantity_Color> m_colors;
    std::vector<FeatureBoundary> m_boundaries;
};

} // namespace oap::app::presentation
