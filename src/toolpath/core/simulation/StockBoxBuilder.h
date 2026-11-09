// 轴对齐包围盒毛坯构造。优先使用分析阶段保存的包围盒；记录失效时可从
// 原始 OCCT 形状重新计算，以免旧缓存或局部包围盒计算失败阻断毛坯仿真。
#pragma once

#include <string>

#include <TopoDS_Shape.hxx>

#include "analysis/Model.h"

namespace oap::toolpath {

// 毛坯盒构建结果：形状、有效边界与状态消息。
struct StockBoxBuildResult {
    TopoDS_Shape shape;
    Bbox effectiveBounds;
    bool usedShapeBounds = false;
    std::string message;

    bool ok() const noexcept { return !shape.IsNull(); }
};

// marginX/Y/Z 均为对应方向的单边余量。recordedBounds 无法生成三维实体盒时，
// 若 sourceShape 有效，则从其真实几何重新求包围盒后重试。
StockBoxBuildResult buildStockBox(
    const Bbox& recordedBounds,
    const TopoDS_Shape* sourceShape,
    double marginX,
    double marginY,
    double marginZ);

} // namespace oap::toolpath
