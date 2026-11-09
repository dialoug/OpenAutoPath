// 基于轴对齐平面极值盒的公共边界面判定。
// 用轴对齐盒近似判定面的内/外归属，供拓扑索引、基面识别和 CAM 使用。
// 本文件为纯函数接口、不持有状态；OCC 形状与 Bnd_Box 由调用方传入并负责生命周期。
#pragma once

#include <Bnd_Box.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pln.hxx>

namespace oap {

// 只用有效平面（含平面回归）中近似轴对齐的面建立极值盒，避免非平面凸起
// 拉偏主外表面的参考边界。没有符合条件的面时返回空盒，由调用方回退整体 AABB。
Bnd_Box buildAxisAlignedPlanarBox(const TopoDS_Shape& shape);

// 累加已分类平面及其紧致面盒；不执行曲面回归或几何求盒。
// 非轴对齐平面和空面盒不改变 referenceBox。
void addAxisAlignedPlanarFaceBox(const gp_Pln& plane,
                                const Bnd_Box& faceBox,
                                Bnd_Box& referenceBox);

// 使用已有分类和紧致面盒的纯几何版本，与面接口使用相同容差。
bool isFaceOnAabbBoundary(const gp_Pln& plane,
                         const Bnd_Box& solidBox,
                         const Bnd_Box& faceBox);

// 判定面是否落在实体包围盒的边界平面上：当面的包围盒（faceBox）在某个轴向上
// 与 solidBox 的对应端面重合（或在容差内）时视为位于 AABB 边界。
// 传入 faceBox 可避免重复计算面的包围盒；返回 false 表示非边界或数据无效。
bool isFaceOnAabbBoundary(const TopoDS_Face& face,
                          const Bnd_Box& solidBox,
                          const Bnd_Box* faceBox = nullptr);

// 便捷封装：判定面是否位于整体 AABB 的外表面（主外表面参考）。
bool isOuterFace(const TopoDS_Face& face, const Bnd_Box& solidBox);

} // namespace oap
