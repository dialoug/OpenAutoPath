// 为单条槽腔清料路径规划垂直、沿轮廓、直线、往复或螺旋斜坡入刀。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

#include "toolpath/pocket/planning/PocketClearing.h"

namespace oap::toolpath::detail {

class PocketSafetyArea;

// 螺旋入刀中的一圈圆弧语义：把 feedPoints 的连续一段移动段压缩为一个
// 整圈圆弧原语，供移动组装器替换为控制器无关的圆弧指令。
struct PocketEntryArc {
    ToolpathCircularArc arc;
    // feedPoints 中相邻点形成的移动段编号，0 表示 feedPoints[0] -> [1]。
    std::size_t firstFeedSegment = 0;
    std::size_t feedSegmentCount = 0;
};

struct PocketEntryPath {
    PocketEntryMode usedMode = PocketEntryMode::VerticalPlunge;
    gp_Pnt2d startPoint;
    // 第一项位于安全面。斜坡模式会先沿开口轴进给到所选高度基准，
    // 再执行斜坡；最后一项必须等于清料路径的切削起点。
    std::vector<gp_Pnt> feedPoints;
    std::vector<PocketEntryArc> arcs;
    bool fallback = false;
    std::string warning;
};

// 按配置的 entryMode 规划清料环入刀；各模式找不到刀心安全路径时按其
// 回退链降级。safeOffset 为安全面，rampStartOffset 为配置基准换算后的深度。
PocketEntryPath planPocketEntry(const PocketRegion& region,
                                const ToolpathParams& params,
                                const ClearingLoop& loop,
                                double safeOffset,
                                double rampStartOffset,
                                const PocketSafetyArea* safetyArea = nullptr);

} // namespace oap::toolpath::detail
