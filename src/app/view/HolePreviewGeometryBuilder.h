// 孔加工预览几何构造接口；输出形状的生命周期由场景控制器管理。
// 模块:oap_app_view。仅构造“仅供显示”的预览线段/Edge(采样、共线重叠裁剪、
// 并行 Edge 构建),不参与实际刀路计算;HolePreviewEdge 供 presentation 层聚合显示。
#pragma once

#include <cstddef>
#include <vector>

#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>

namespace oap::appdetail {

// 预览线段的语义分类,决定其在视图中的颜色与宽度。
enum class HolePreviewSegmentKind {
    Cut,       // 切削段
    Rapid,     // 快移段
    Vertical   // 垂直进退刀段
};

// 以端点表示的预览线段。
struct HolePreviewSegment {
    gp_Pnt from;
    gp_Pnt to;
    HolePreviewSegmentKind kind = HolePreviewSegmentKind::Cut;
    // 可选来源下标；颜色重叠裁剪产生碎片时原样传播。几何层本身不解释
    // 该值，呈现层可用它恢复工序/移动身份。
    std::size_t sourceIndex = static_cast<std::size_t>(-1);
};

// 以 OCCT Edge 表示的预览线段,可直接聚合进 AIS_Shape。
struct HolePreviewEdge {
    TopoDS_Edge edge;
    HolePreviewSegmentKind kind = HolePreviewSegmentKind::Cut;
};

// 均匀采样仅用于显示的线段,同时保留首段与末段。
// 加工操作与导出的程序保持原样不动。
std::vector<std::size_t> holePreviewSampleIndices(
    std::size_t sourceCount,
    std::size_t maximumSegments);

// 按切削、垂直移动、快速移动的顺序裁掉低优先级重合区间，
// 避免多个 AIS 对象在同一深度争抢像素而随视角变色。
std::vector<HolePreviewSegment> resolveHolePreviewColorOverlaps(
    const std::vector<HolePreviewSegment>& segments);

} // namespace oap::appdetail
