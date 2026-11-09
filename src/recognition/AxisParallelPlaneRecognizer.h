// 直接识别法向平行给定坐标轴的内部平面。
// 模块: oap_recognition。构建或复用 RecognitionGeometryCache 判定逐体主外表面，
//       与调用方传入的孔/通结构排除面一起过滤候选，再用 effectivePlane 回归
//       平面做轴对齐判定（|n·axis|≈1）。
// 消费方: ThreeAxisWireTraceUseCase 的三轴联合识别；RecognizeZParallelPlanesOCCT
//       为保留的 Z 轴兼容入口。
#pragma once

#include <unordered_set>
#include <vector>

class AAG;
class TopoDS_Shape;
class gp_Dir;

namespace oap {

class RecognitionGeometryCache;

// 识别统计结果：faceIndices 为命中面的 AAG 1-based 索引（升序加入），
// 其余计数字段供识别器向调用方汇报各筛选阶段的面数。
struct AxisParallelPlaneRecognitionResult {
    // AAG 中的 1-based 面索引。
    std::vector<int> faceIndices;
    int planeFaceCount = 0;
    int mainOuterFaceCount = 0;
    int excludedFaceCount = 0;
    int skippedInvalidFaceCount = 0;
};

// 在任何平面/法向筛选前，将逐体 AABB 主外表面与调用方提供的孔、通结构
// 排除集合并；剩余面若为平面且几何法向平行给定轴的正负方向，则加入结果。
AxisParallelPlaneRecognitionResult RecognizeAxisParallelPlanesOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices = {});

// 多方向联合识别使用：复用同一 Unit 已建立的主外表面几何缓存。
// 参数 geometryCache 必须与 graph 来自同一次 AAG 构建，否则面索引错位。
AxisParallelPlaneRecognitionResult RecognizeAxisParallelPlanesOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices,
    const RecognitionGeometryCache& geometryCache);

// 在任何平面/法向筛选前，将逐体 AABB 主外表面与调用方提供的孔、通结构
// 排除集合并；剩余面若为平面且几何法向平行 ±Z，则加入结果。
// 此接口保留为 Z 轴兼容入口。
using ZParallelPlaneRecognitionResult =
    AxisParallelPlaneRecognitionResult;

ZParallelPlaneRecognitionResult RecognizeZParallelPlanesOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const std::unordered_set<int>& excludedFaceIndices = {});

} // namespace oap
