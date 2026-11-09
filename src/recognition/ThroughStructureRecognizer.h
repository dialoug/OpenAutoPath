// 通结构面识别：筛选整面法向垂直 Z 轴的面，并检查其非 Z 向共享边的凸性。
#pragma once

#include <unordered_set>
#include <vector>

class AAG;
class TopoDS_Edge;
class TopoDS_Shape;

namespace oap {

class RecognitionGeometryCache;

namespace detail {

// 模块私有实现细节，不对外暴露；仅供识别器内部与几何回归测试使用。
// 解析直线按方向直接判断；样条等一般曲线则同时检查端点弦向与沿程切向。
// 该函数暴露在 detail 命名空间，供几何回归测试验证 STEP 中常见的
// “直线形状、B-spline 表示”边。
bool isEdgeGeometricallyZDirected(const TopoDS_Edge& edge);

} // namespace detail

// 通结构识别结果：命中面索引与各类筛选/排除统计。
struct ThroughStructureRecognitionResult {
    // AAG 中的 1-based 面索引。每个命中项代表一张通结构面。
    std::vector<int> faceIndices;
    int verticalSurfaceCount = 0;
    int excludedHoleFaceCount = 0;
    int excludedMainOuterFaceCount = 0;
    int rejectedWithoutNonZNeighborCount = 0;
    int rejectedNonConvexCount = 0;
};

// 规则：
// 1. 候选整张面上的有效法向都必须垂直 Z 轴；
//    平面、圆柱和线性拉伸面用解析条件，其余曲面执行覆盖裁剪域的法向验证；
// 2. 属于孔识别结果的任意面直接排除；
// 3. 属于逐体 AABB 主外表面识别结果的面直接排除；
// 4. 忽略几何上沿 Z 轴的共享边（包括直线形状的 B-spline），只检查其余共享边；
// 5. 至少存在一条此类共享边，且每一条都必须为凸边。
ThroughStructureRecognitionResult RecognizeThroughStructuresOCCT(
    const TopoDS_Shape& shape, AAG& graph);

// 联合识别入口使用的重载：孔识别已经完成时，直接把同一 AAG 中的
// 1-based 孔面索引作为筛选前排除集，避免再次执行孔识别。
ThroughStructureRecognitionResult RecognizeThroughStructuresOCCT(
    const TopoDS_Shape& shape,
    AAG& graph,
    const std::unordered_set<int>& recognizedHoleFaceIndices);

// 带缓存重载：geometryCache 必须由调用方先以相同 shape/graph 构建，
// 用于复用主外表面标记与边—面映射，避免重复几何计算。
ThroughStructureRecognitionResult RecognizeThroughStructuresOCCT(
    const TopoDS_Shape& shape,
    AAG& graph,
    const std::unordered_set<int>& recognizedHoleFaceIndices,
    const RecognitionGeometryCache& geometryCache);

} // namespace oap
