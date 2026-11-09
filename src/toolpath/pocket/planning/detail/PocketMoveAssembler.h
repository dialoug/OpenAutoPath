// 将几何清料环转换为有序的机床移动段。
#pragma once

#include "toolpath/pocket/planning/PocketClearing.h"

namespace oap::toolpath::detail {

// 将 result.loops 中的二维清料环提升为世界坐标移动段（Cut/Plunge/
// Rapid/Retract），按深度层和环序插入进刀、层间连接与抬刀退刀，
// 并把切削段压缩为直线/圆弧语义原语。结果写回 result.moves 与
// result.primitives，两者保持执行顺序一致。
void buildPocketMoves(const PocketRegion& region,
                      const ToolpathParams& params,
                      PocketClearingResult& result,
                      const ToolpathGenerationControl* control = nullptr);

} // namespace oap::toolpath::detail
