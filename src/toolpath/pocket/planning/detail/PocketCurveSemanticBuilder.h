// 将清料折线的离散移动段压缩为控制器无关的直线/圆弧语义原语。
// 属于 oap_toolpath_pocket 的内部实现（detail/）：仅供 PocketMoveAssembler
// 调用，压缩后的每个原语都保留原始移动段作为 fallback，展开执行流不变。
#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/pocket/planning/detail/PocketSafetyGeometry.h"

namespace oap::toolpath::detail {

// 将现有切削折线压缩为控制器无关的直线/圆弧语义。每个压缩原语都保留
// 原始 moves 作为 fallback，因而展开后的执行流保持完全一致。
std::vector<ToolpathPrimitive> buildPocketCurvePrimitives(
    const PocketRegion& region,
    const ToolpathParams& params,
    const PocketSafetyArea& safetyArea,
    const ClearingLoop& loop,
    const std::vector<ToolpathMove>& moves,
    std::size_t begin,
    std::size_t end);

bool isPocketArcCutterCenterSafe(
    const ToolpathCircularArc& arc,
    const ToolpathPrimitiveContext& context,
    const PocketRegion& region,
    const ToolpathParams& params,
    const PocketSafetyArea& safetyArea,
    const std::function<bool()>& isCancellationRequested = {});

} // namespace oap::toolpath::detail
