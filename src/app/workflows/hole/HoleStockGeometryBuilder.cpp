// HoleStockGeometryBuilder 实现（oap_app_workflow_cam）。
// 孔仿真毛坯/目标体的无状态构建器，见 HoleStockGeometryBuilder.h。
#include "app/workflows/hole/HoleStockGeometryBuilder.h"

#include "toolpath/core/simulation/StockBoxBuilder.h"

#include <BRep_Builder.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopoDS_Compound.hxx>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace oap::app {

HoleStockGeometryBuildResult
HoleStockGeometryBuilder::buildInitialStock(
    const std::vector<HoleStockUnitGeometry>& units,
    double marginX,
    double marginY,
    double marginZ,
    bool perUnit)
{
    HoleStockGeometryBuildResult result;
    if (!std::isfinite(marginX) || !std::isfinite(marginY) ||
        !std::isfinite(marginZ) || marginX < 0.0 || marginY < 0.0 ||
        marginZ < 0.0) {
        result.error = "毛坯余量必须是有限的非负数。";
        return result;
    }
    if (units.empty()) {
        result.error = "当前模型没有可用于构造毛坯的单元。";
        return result;
    }

    // 整体毛坯模式：合并所有单元的包围盒与源形状，
    // 构造单个外包络毛坯（含用户指定的三轴余量）。
    if (!perUnit) {
        Bbox modelBounds;
        TopoDS_Compound sourceShapes;
        BRep_Builder sourceBuilder;
        sourceBuilder.MakeCompound(sourceShapes);
        int sourceShapeCount = 0;
        for (const HoleStockUnitGeometry& unit : units) {
            modelBounds.merge(unit.recordedBounds);
            if (!unit.sourceShape.IsNull()) {
                sourceBuilder.Add(sourceShapes, unit.sourceShape);
                ++sourceShapeCount;
            }
        }
        const TopoDS_Shape* fallbackShape =
            sourceShapeCount > 0 ? &sourceShapes : nullptr;
        toolpath::StockBoxBuildResult built = toolpath::buildStockBox(
            modelBounds, fallbackShape, marginX, marginY, marginZ);
        if (!built.ok()) {
            result.error =
                "无法从当前模型总包围盒构造有效实体毛坯：" +
                built.message + "。";
            return result;
        }
        if (built.usedShapeBounds) {
            result.warnings.push_back(
                "recorded model bounds were unusable; stock bounds were recomputed from source shapes");
        }
        result.shape = built.shape;
        result.solidCount = 1;
        return result;
    }

    // 逐单元模式：每个单元独立构造毛坯块，失败单元跳过并记录；
    // 全部失败则聚合错误信息返回失败。
    TopoDS_Compound blocks;
    BRep_Builder builder;
    builder.MakeCompound(blocks);
    std::vector<std::string> failures;
    for (const HoleStockUnitGeometry& unit : units) {
        const TopoDS_Shape* sourceShape =
            unit.sourceShape.IsNull() ? nullptr : &unit.sourceShape;
        toolpath::StockBoxBuildResult built = toolpath::buildStockBox(
            unit.recordedBounds,
            sourceShape,
            marginX,
            marginY,
            marginZ);
        if (!built.ok()) {
            if (failures.size() < 3) {
                failures.push_back(unit.unitId + ": " + built.message);
            }
            continue;
        }
        if (built.usedShapeBounds) {
            result.warnings.push_back(
                unit.unitId +
                ": recorded bounds were unusable; stock bounds were recomputed from source shape");
        }
        builder.Add(blocks, built.shape);
        ++result.solidCount;
    }
    if (result.solidCount == 0) {
        std::ostringstream message;
        message << "当前模型的 " << units.size()
                << " 个单元均无法构造实体毛坯。";
        if (!failures.empty()) {
            message << ' ';
            for (std::size_t index = 0; index < failures.size(); ++index) {
                if (index > 0)
                    message << "; ";
                message << failures[index];
            }
        }
        result.error = message.str();
        return result;
    }
    result.shape = blocks;
    return result;
}

HoleStockGeometryBuildResult
HoleStockGeometryBuilder::buildTargetShape(
    const std::vector<TopoDS_Solid>& selectedSolids,
    const std::vector<TopoDS_Solid>& fallbackSolids)
{
    HoleStockGeometryBuildResult result;
    TopoDS_Compound solids;
    BRep_Builder builder;
    builder.MakeCompound(solids);
    // 先去重再加入复合体：同一实体被多个面引用时只保留一次，
    // 避免布尔操作对重复目标产生不稳定结果。
    TopTools_MapOfShape added;
    const auto append = [&](const std::vector<TopoDS_Solid>& candidates) {
        for (const TopoDS_Solid& solid : candidates) {
            if (solid.IsNull() || !added.Add(solid))
                continue;
            builder.Add(solids, solid);
            ++result.solidCount;
        }
    };

    append(selectedSolids);
    if (result.solidCount == 0)
        append(fallbackSolids);
    if (result.solidCount == 0) {
        result.error =
            "无法找到孔操作所属的封闭目标实体，不能执行毛坯差异检查。";
        return result;
    }
    result.shape = solids;
    return result;
}

} // namespace oap::app
