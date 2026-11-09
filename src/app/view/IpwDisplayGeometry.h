// Multi-Dexel 在 OCCT 主视口中的派生显示形式和中性图元。
//
// 后台线程只填充这些轻量数据；AIS/Graphic3d 对象仍只在 UI 线程创建。
// 模块:oap_app_view。本头文件是后台线程与呈现层的“数据契约”,只含纯数据。
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include <gp_Pnt.hxx>

namespace oap::app {

// IPW 在 OCCT 视图中的显示模式,由 UI 下拉框直接映射。
enum class IpwOcctDisplayMode : int {
    SurfaceWithEdges = 0,  // 表面 + 边线
    SurfaceShaded = 1,     // 纯着色表面
    SurfaceWireframe = 2,  // 线框表面
    DexelColumns = 3,      // Dexel 柱
    IntervalEndpoints = 4, // 区间端点
    IntervalPrisms = 5,    // 区间棱柱
    MidSection = 6,        // 中截面
    BRepSnapshot = 7,      // BRep 快照
    Off = 8                // 关闭 IPW 显示
};

// 是否为“表面型”显示模式(需要网格/面渲染)。
inline bool isIpwSurfaceDisplayMode(
    IpwOcctDisplayMode mode) noexcept
{
    return mode == IpwOcctDisplayMode::SurfaceWithEdges ||
           mode == IpwOcctDisplayMode::SurfaceShaded ||
           mode == IpwOcctDisplayMode::SurfaceWireframe;
}

// 中性图元的种类:后台线程按此填充对应的顶点容器。
enum class IpwDisplayPrimitiveKind {
    Segments,  // 线段(相邻点对)
    Points,    // 点
    Prisms,    // 棱柱(8 顶点六面体)
    Quads      // 四边形
};

// 显示图元数据(纯数据,可跨线程传递):segments/points 共用 vertices,
// prisms 与 quads 为定长顶点组。sourcePrimitiveCount 记录原始图元数,
// samplingStride 记录采样步长,用于统计与调试。
struct IpwDisplayPrimitiveGeometry {
    IpwDisplayPrimitiveKind kind =
        IpwDisplayPrimitiveKind::Segments;
    // Segments 按相邻点对存储；Points 每项为一个端点。
    std::vector<gp_Pnt> vertices;
    std::vector<std::array<gp_Pnt, 8>> prisms;
    std::vector<std::array<gp_Pnt, 4>> quads;
    std::size_t sourcePrimitiveCount = 0u;
    std::size_t displayedPrimitiveCount = 0u;
    int samplingStride = 1;
    double sectionCoordinate = 0.0;

    bool empty() const noexcept
    {
        return vertices.empty() && prisms.empty() &&
               quads.empty();
    }
};

using IpwDisplayPrimitiveGeometryPtr =
    std::shared_ptr<const IpwDisplayPrimitiveGeometry>;

} // namespace oap::app
