// 孔仿真毛坯/目标体的无状态构建器。调用方负责把 Model/StepIndexer 解析成
// 形状输入；本类只处理包围盒恢复、复合体构造和实体去重。
#pragma once

#include "analysis/Model.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>

#include <string>
#include <vector>

namespace oap::app {

// 一个模型单元（如镶件）的毛坯输入：记录包围盒与源实体；
// recordedBounds 失效时可用 sourceShape 重新计算真实包围盒。
struct HoleStockUnitGeometry {
    std::string unitId;
    Bbox recordedBounds;
    TopoDS_Shape sourceShape;
};

// 构建结果：shape 为构造出的毛坯/目标复合体，
// ok() 要求无错误、形状非空且至少包含一个实体。
struct HoleStockGeometryBuildResult {
    TopoDS_Shape shape;
    int solidCount = 0;
    std::vector<std::string> warnings;
    std::string error;

    bool ok() const noexcept
    {
        return error.empty() && !shape.IsNull() && solidCount > 0;
    }
};

// 孔仿真几何的无状态构建器（静态工具类）：调用方负责把
// Model/StepIndexer 解析成形状输入，本类只负责包围盒恢复、
// 复合体构造与实体去重。
class HoleStockGeometryBuilder {
public:
    // 构建初始毛坯：perUnit=false 时基于全部单元的总包围盒构造单个毛坯；
    // perUnit=true 时为每个单元各自构造并合成复合体。
    static HoleStockGeometryBuildResult buildInitialStock(
        const std::vector<HoleStockUnitGeometry>& units,
        double marginX,
        double marginY,
        double marginZ,
        bool perUnit);

    // 构建目标体：优先 selectedSolids，为空时回退 fallbackSolids，
    // 并对实体去重后合并为复合体。
    static HoleStockGeometryBuildResult buildTargetShape(
        const std::vector<TopoDS_Solid>& selectedSolids,
        const std::vector<TopoDS_Solid>& fallbackSolids);
};

} // namespace oap::app
