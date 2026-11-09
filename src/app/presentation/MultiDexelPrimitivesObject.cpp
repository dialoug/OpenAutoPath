// MultiDexelPrimitivesObject 实现（oap_app_presentation 的 detail 私有目录）。
// 把后台线程算好的 Multi-Dexel 中性图元一次性上屏（只做 mode-0、不拾取），
// 见 detail/MultiDexelPrimitivesObject.h。
#include "app/presentation/detail/MultiDexelPrimitivesObject.h"

#include <algorithm>
#include <array>
#include <limits>

#include <Aspect_TypeOfLine.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <Graphic3d_AlphaMode.hxx>
#include <Graphic3d_ArrayOfPoints.hxx>
#include <Graphic3d_ArrayOfSegments.hxx>
#include <Graphic3d_ArrayOfTriangles.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>
#include <Graphic3d_AspectLine3d.hxx>
#include <Graphic3d_AspectMarker3d.hxx>
#include <Graphic3d_Group.hxx>
#include <Graphic3d_TypeOfShadingModel.hxx>
#include <Prs3d_Presentation.hxx>
#include <Quantity_ColorRGBA.hxx>

namespace oap::app::presentation {

namespace {

Standard_Integer boundedVertexCount(std::size_t count)
{
    return static_cast<Standard_Integer>(
        std::min<std::size_t>(
            count,
            static_cast<std::size_t>(
                std::numeric_limits<Standard_Integer>::max())));
}

Handle(Graphic3d_AspectFillArea3d) makeFillAspect(
    const Quantity_Color& color,
    double transparency)
{
    Handle(Graphic3d_AspectFillArea3d) aspect =
        new Graphic3d_AspectFillArea3d();
    const float alpha = static_cast<float>(
        1.0 - std::clamp(transparency, 0.0, 1.0));
    aspect->SetInteriorColor(
        Quantity_ColorRGBA(color, alpha));
    aspect->SetAlphaMode(Graphic3d_AlphaMode_BlendAuto);
    aspect->SetShadingModel(Graphic3d_TOSM_UNLIT);
    aspect->SetEdgeOn();
    aspect->SetEdgeColor(
        Quantity_Color(0.03, 0.12, 0.10, Quantity_TOC_RGB));
    aspect->SetEdgeWidth(1.0);
    return aspect;
}

void appendTriangle(
    const Handle(Graphic3d_ArrayOfTriangles)& triangles,
    const gp_Pnt& a,
    const gp_Pnt& b,
    const gp_Pnt& c)
{
    triangles->AddVertex(a);
    triangles->AddVertex(b);
    triangles->AddVertex(c);
}

} // namespace

MultiDexelPrimitivesObject::MultiDexelPrimitivesObject(
    IpwDisplayPrimitiveGeometryPtr geometry,
    const Quantity_Color& color,
    double transparency)
    : m_geometry(std::move(geometry)),
      m_color(color),
      m_transparency(
          std::clamp(transparency, 0.0, 1.0))
{
    SetDisplayMode(0);
}

void MultiDexelPrimitivesObject::Compute(
    const Handle(PrsMgr_PresentationManager)&,
    const Handle(Prs3d_Presentation)& presentation,
    Standard_Integer)
{
    if (!m_geometry || m_geometry->empty())
        return;

    const Handle(Graphic3d_Group) group =
        presentation->CurrentGroup();
    switch (m_geometry->kind) {
        case IpwDisplayPrimitiveKind::Segments: {
            Handle(Graphic3d_ArrayOfSegments) segments =
                new Graphic3d_ArrayOfSegments(
                    boundedVertexCount(
                        m_geometry->vertices.size()));
            for (const gp_Pnt& point : m_geometry->vertices)
                segments->AddVertex(point);
            group->SetPrimitivesAspect(
                new Graphic3d_AspectLine3d(
                    m_color, Aspect_TOL_SOLID, 2.0));
            group->AddPrimitiveArray(segments);
            break;
        }
        case IpwDisplayPrimitiveKind::Points: {
            Handle(Graphic3d_ArrayOfPoints) points =
                new Graphic3d_ArrayOfPoints(
                    boundedVertexCount(
                        m_geometry->vertices.size()));
            for (const gp_Pnt& point : m_geometry->vertices)
                points->AddVertex(point);
            group->SetPrimitivesAspect(
                new Graphic3d_AspectMarker3d(
                    Aspect_TOM_BALL, m_color, 2.5));
            group->AddPrimitiveArray(points);
            break;
        }
        case IpwDisplayPrimitiveKind::Prisms: {
            // 棱柱图元以 8 顶点六面体表示,拆成 6 个面 × 2 个三角形 = 12 个
            // 三角面。静态面索引表定义各面的顶点组合,循环展开每个棱柱。
            constexpr std::array<std::array<int, 3>, 12> faces = {{
                {{0, 2, 1}}, {{1, 2, 3}},
                {{4, 5, 6}}, {{5, 7, 6}},
                {{0, 1, 4}}, {{1, 5, 4}},
                {{2, 6, 3}}, {{3, 6, 7}},
                {{0, 4, 2}}, {{2, 4, 6}},
                {{1, 3, 5}}, {{3, 7, 5}}
            }};
            Handle(Graphic3d_ArrayOfTriangles) triangles =
                new Graphic3d_ArrayOfTriangles(
                    boundedVertexCount(
                        m_geometry->prisms.size() * 36u));
            for (const auto& prism : m_geometry->prisms) {
                for (const auto& face : faces) {
                    appendTriangle(
                        triangles,
                        prism[static_cast<std::size_t>(face[0])],
                        prism[static_cast<std::size_t>(face[1])],
                        prism[static_cast<std::size_t>(face[2])]);
                }
            }
            group->SetPrimitivesAspect(
                makeFillAspect(m_color, m_transparency));
            group->AddPrimitiveArray(triangles);
            break;
        }
        case IpwDisplayPrimitiveKind::Quads: {
            // 四边形图元拆成两个三角形 (0,1,2) 与 (0,2,3),共用同一填充面属性。
            Handle(Graphic3d_ArrayOfTriangles) triangles =
                new Graphic3d_ArrayOfTriangles(
                    boundedVertexCount(
                        m_geometry->quads.size() * 6u));
            for (const auto& quad : m_geometry->quads) {
                appendTriangle(
                    triangles, quad[0], quad[1], quad[2]);
                appendTriangle(
                    triangles, quad[0], quad[2], quad[3]);
            }
            group->SetPrimitivesAspect(
                makeFillAspect(m_color, m_transparency));
            group->AddPrimitiveArray(triangles);
            break;
        }
    }
}

void MultiDexelPrimitivesObject::ComputeSelection(
    const Handle(SelectMgr_Selection)&,
    Standard_Integer)
{
}

} // namespace oap::app::presentation
