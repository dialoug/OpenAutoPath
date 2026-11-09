// Private mesh and coordinate conversions shared by the CAM dialog and renderers.
#pragma once

#include <cstdint>
#include <utility>
#include <vector>
#include <QMatrix4x4>
#include <QString>
#include <QVector3D>
#include <Poly_Triangulation.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include "app/ui/dialogs/PocketCamSimulatorDialog.h"

namespace oap::app::pocket_cam_detail {

struct RenderVertex {
    QVector3D position;
    QVector3D normal;
};

struct PathRenderVertex {
    QVector3D position;
    float simulationStep = 0.0f;
    float rapid = 0.0f;
};

struct RenderMesh {
    std::vector<RenderVertex> vertices;
    std::vector<std::uint32_t> indices;

    bool empty() const noexcept
    {
        return vertices.empty() || indices.empty();
    }
};

struct ToolFrame {
    gp_Vec u;
    gp_Vec v;
    gp_Vec w;
};

QString mouseButtonText(Qt::MouseButton button);
QString rotationModeText(PocketCamRotationMode mode);
QVector3D vector3(const gp_Pnt& point);
QVector3D vector3(const gp_Vec& vector);
RenderMesh meshShape(const TopoDS_Shape& shape, double deflection, QString& error);
RenderMesh meshTriangulation(const Handle(Poly_Triangulation)& triangulation, QString& error);
RenderMesh makeRevolvedTool(const std::vector<std::pair<double, double>>& sideProfile, int slices);
QMatrix4x4 transform(const gp_Vec& xAxis, const gp_Vec& yAxis, const gp_Vec& zAxis, const gp_Pnt& origin);
ToolFrame makeToolFrame(const gp_Vec& outwardAxis);
QMatrix4x4 toolTransform(const ToolFrame& frame, const gp_Pnt& tip);
gp_Pnt interpolate(const gp_Pnt& from, const gp_Pnt& to, double fraction);

} // namespace oap::app::pocket_cam_detail
