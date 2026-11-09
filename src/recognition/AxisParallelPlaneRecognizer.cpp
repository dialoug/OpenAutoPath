// AxisParallelPlaneRecognizer 实现 — 参见 AxisParallelPlaneRecognizer.h。
// 模块: oap_recognition。核心流程：先收集主外表面与调用方排除面，再对剩余面
//       逐张取 effectivePlane 平面，用 |n·axis|≈1 判轴对齐；OCCT 异常的面
//       跳过并计数。
#include "AxisParallelPlaneRecognizer.h"

#include "AAG.h"
#include "RecognitionGeometryCache.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>

namespace oap {
namespace {

// 对单位法向而言，轴对齐判定等价于 abs(abs(n·axis)-1) < 0.01。
constexpr double kAxisAlignedComponentTolerance = 0.01;

} // namespace

AxisParallelPlaneRecognitionResult RecognizeAxisParallelPlanesOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices)
{
    RecognitionGeometryCache geometryCache;
    geometryCache.build(shape, graph);
    return RecognizeAxisParallelPlanesOCCT(
        shape, graph, axisDirection,
        excludedFaceIndices, geometryCache);
}

AxisParallelPlaneRecognitionResult RecognizeAxisParallelPlanesOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices,
    const RecognitionGeometryCache& geometryCache)
{
    AxisParallelPlaneRecognitionResult result;
    if (shape.IsNull() || graph.faceMap.Extent() == 0)
        return result;

    // 第一步：排除面集合 = 调用方排除面 ∪ 所有主外表面（主外表面只作
    // 入口过滤，不进入识别结果）。
    std::unordered_set<int> blockedFaces =
        excludedFaceIndices;
    for (int faceIndex = 1;
         faceIndex <= graph.faceMap.Extent(); ++faceIndex) {
        if (!geometryCache.isMainOuterFace(faceIndex))
            continue;
        blockedFaces.insert(faceIndex);
        ++result.mainOuterFaceCount;
    }

    result.excludedFaceCount =
        static_cast<int>(std::count_if(
            blockedFaces.begin(), blockedFaces.end(),
            [&](int faceIndex) {
                return faceIndex >= 1 &&
                       faceIndex <= graph.faceMap.Extent();
            }));

    for (int faceIndex = 1;
         faceIndex <= graph.faceMap.Extent(); ++faceIndex) {
        // 排除判断先于曲面适配、平面类型和法向计算。
        if (blockedFaces.count(faceIndex) > 0)
            continue;
        try {
            gp_Pln plane;
            if (!geometryCache.effectivePlane(
                    faceIndex, plane))
                continue;
            ++result.planeFaceCount;
            const double alignment = std::abs(
                plane.Axis().Direction().Dot(axisDirection));
            if (std::abs(alignment - 1.0) <
                kAxisAlignedComponentTolerance)
                result.faceIndices.push_back(faceIndex);
        } catch (const Standard_Failure&) {
            ++result.skippedInvalidFaceCount;
        }
    }
    return result;
}

ZParallelPlaneRecognitionResult RecognizeZParallelPlanesOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const std::unordered_set<int>& excludedFaceIndices)
{
    return RecognizeAxisParallelPlanesOCCT(
        shape, graph, gp::DZ(), excludedFaceIndices);
}

} // namespace oap
