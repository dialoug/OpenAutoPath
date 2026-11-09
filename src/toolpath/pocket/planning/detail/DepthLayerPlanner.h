// 腔和槽清料使用的内部深度层计划。
#pragma once

#include <vector>

#include "toolpath/pocket/planning/PocketClearing.h"

namespace oap::toolpath::detail {

// 单个加工深度层：index 为层序号（0=第一刀层，向最终深度递增），
// offset 为该层相对腔底的局部偏移（0=底面，正值朝开口方向）。
struct DepthLayer {
    int index = 0;
    double offset = 0.0;
};

// 根据总深度、步下量、起始/最终深度和精修留量生成单调递减的深度序列；
// 结果写入 result（startDepthOffset/finalDepthOffset/stepDown/finishDepth）。
std::vector<DepthLayer> buildDepthLayers(const PocketRegion& region,
                                         const DepthParams& depth,
                                         PocketClearingResult& result);

} // namespace oap::toolpath::detail
