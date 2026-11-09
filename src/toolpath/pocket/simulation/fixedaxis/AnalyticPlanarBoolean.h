// 同平面解析 Face 的空间分组并集原型。非相交区域直接保留，相交组只提交
// 一次多参数 OCCT Fuse，避免随原语数量增长的逐条布尔链。
//
// 位于 oap_toolpath_pocket 的 fixedaxis/ 模块私有实现目录（内部细节），
// 由 AnalyticFixedAxisLayeredBRepBuilder 的解析快路径使用；仅接受
// 由 Line/Circle 边围成的平面 Face，输出保留相同曲线语义。
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace oap::toolpath {

// 解析平面布尔结果：并集区域、面列表与统计。
struct AnalyticPlanarBooleanResult {
    TopoDS_Shape area;
    std::vector<TopoDS_Face> faces;
    std::string error;
    bool cancelled = false;
    int inputFaceCount = 0;
    int spatialGroupCount = 0;
    int fusedGroupCount = 0;
    int largestGroupSize = 0;
    int outputFaceCount = 0;
    int lineEdgeCount = 0;
    int circleEdgeCount = 0;
    int unsupportedEdgeCount = 0;
    double inputArea = 0.0;
    double unionArea = 0.0;
    double overlapArea = 0.0;
    double buildMilliseconds = 0.0;

    bool ok() const noexcept
    {
        return error.empty() && !area.IsNull() &&
               !faces.empty();
    }
};

// 解析平面布尔：同平面 Face 空间分组并集，避免逐条布尔链。
class AnalyticPlanarBoolean {
public:
    AnalyticPlanarBooleanResult unite(
        const std::vector<TopoDS_Face>& faces,
        double tolerance = 1.0e-7,
        bool runParallel = true,
        std::function<bool()> cancellationRequested = {}) const;
};

} // namespace oap::toolpath
