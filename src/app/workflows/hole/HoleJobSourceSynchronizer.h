// 将已加载 Job 的权威孔几何同步回当前识别会话。
// 调用场景：打开旧项目后，识别结果要以 Job 里保存的孔数据为准，
// 而不是重新运行识别；非孔实例则原样保留。
#pragma once

#include <vector>

#include "recognition/FeatureRecognizer.h"
#include "toolpath/hole/HoleMachiningInput.h"

namespace oap::app {

// 用 Job 中保存的孔输入刷新识别会话：清空并重建孔几何、
// 按面 ID 合并生成孔实例（保留非孔实例），并更新孔缓存。
void synchronizeLoadedJobHoleSources(
    const std::vector<oap::toolpath::HoleMachiningInput>& jobHoles,
    RecognitionResult& recognition,
    std::vector<oap::toolpath::HoleMachiningInput>& holeCache,
    bool& hasHoleCache);

} // namespace oap::app
