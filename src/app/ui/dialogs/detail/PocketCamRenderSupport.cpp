#include "app/ui/dialogs/detail/PocketCamRenderSupport.h"
#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <QVector4D>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>

namespace oap::app::pocket_cam_detail {

QString mouseButtonText(Qt::MouseButton button)
{
    switch (button) {
        case Qt::LeftButton:
            return QCoreApplication::translate("OpenAutoPath", "左键");
        case Qt::RightButton:
            return QCoreApplication::translate("OpenAutoPath", "右键");
        case Qt::MiddleButton:
            return QCoreApplication::translate("OpenAutoPath", "中键");
        case Qt::BackButton:
            return QCoreApplication::translate("OpenAutoPath", "后退键");
        case Qt::ForwardButton:
            return QCoreApplication::translate("OpenAutoPath", "前进键");
        case Qt::NoButton:
            return QCoreApplication::translate("OpenAutoPath", "未绑定");
        default:
            return QCoreApplication::translate("OpenAutoPath", "鼠标键 %1")
                .arg(static_cast<int>(button));
    }
}

QString rotationModeText(PocketCamRotationMode mode)
{
    return mode == PocketCamRotationMode::Model
        ? QCoreApplication::translate("OpenAutoPath", "固定相机，旋转模型")
        : QCoreApplication::translate("OpenAutoPath", "相机环绕");
}

static void addTriangle(
    RenderMesh& mesh,
    const QVector3D& first,
    const QVector3D& second,
    const QVector3D& third)
{
    QVector3D normal =
        QVector3D::crossProduct(second - first, third - first);
    if (normal.lengthSquared() <=
        std::numeric_limits<float>::epsilon()) {
        return;
    }
    normal.normalize();
    const std::uint32_t base =
        static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({first, normal});
    mesh.vertices.push_back({second, normal});
    mesh.vertices.push_back({third, normal});
    mesh.indices.push_back(base);
    mesh.indices.push_back(base + 1u);
    mesh.indices.push_back(base + 2u);
}

QVector3D vector3(const gp_Pnt& point)
{
    return {
        static_cast<float>(point.X()),
        static_cast<float>(point.Y()),
        static_cast<float>(point.Z())};
}

QVector3D vector3(const gp_Vec& vector)
{
    return {
        static_cast<float>(vector.X()),
        static_cast<float>(vector.Y()),
        static_cast<float>(vector.Z())};
}

RenderMesh meshShape(
    const TopoDS_Shape& shape,
    double deflection,
    QString& error)
{
    RenderMesh mesh;
    if (shape.IsNull()) {
        error = QCoreApplication::translate("OpenAutoPath", "OpenGL CAM 仿真的毛坯为空。");
        return mesh;
    }
    try {
        BRepMesh_IncrementalMesh triangulator(
            shape, std::max(1.0e-3, deflection),
            Standard_False, 0.35, Standard_True);
        triangulator.Perform();
        if (!triangulator.IsDone()) {
            error = QCoreApplication::translate("OpenAutoPath", "无法为 OpenGL CAM 仿真生成毛坯三角网格。");
            return mesh;
        }
        for (TopExp_Explorer explorer(shape, TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            const TopoDS_Face face =
                TopoDS::Face(explorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation =
                BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull())
                continue;
            const gp_Trsf transform = location.Transformation();
            const bool reversed =
                face.Orientation() == TopAbs_REVERSED;
            for (int triangleIndex = 1;
                 triangleIndex <= triangulation->NbTriangles();
                 ++triangleIndex) {
                int n1 = 0;
                int n2 = 0;
                int n3 = 0;
                triangulation->Triangle(triangleIndex).Get(
                    n1, n2, n3);
                if (reversed)
                    std::swap(n2, n3);
                addTriangle(
                    mesh,
                    vector3(triangulation->Node(n1).Transformed(transform)),
                    vector3(triangulation->Node(n2).Transformed(transform)),
                    vector3(triangulation->Node(n3).Transformed(transform)));
            }
        }
    } catch (...) {
        error = QCoreApplication::translate("OpenAutoPath", "生成 OpenGL CAM 毛坯网格时发生 OCCT 异常。");
        mesh = {};
        return mesh;
    }
    if (mesh.empty())
        error = QCoreApplication::translate("OpenAutoPath", "OpenGL CAM 毛坯网格不包含有效三角形。");
    return mesh;
}

RenderMesh meshTriangulation(
    const Handle(Poly_Triangulation)& triangulation,
    QString& error)
{
    RenderMesh mesh;
    if (triangulation.IsNull()) {
        error = QCoreApplication::translate("OpenAutoPath", "统一 IPW 最终显示网格为空。");
        return mesh;
    }
    try {
        mesh.vertices.reserve(
            static_cast<std::size_t>(
                triangulation->NbTriangles()) *
            3u);
        mesh.indices.reserve(
            static_cast<std::size_t>(
                triangulation->NbTriangles()) *
            3u);
        for (int triangleIndex = 1;
             triangleIndex <=
                 triangulation->NbTriangles();
             ++triangleIndex) {
            int n1 = 0;
            int n2 = 0;
            int n3 = 0;
            triangulation->Triangle(
                triangleIndex).Get(n1, n2, n3);
            addTriangle(
                mesh,
                vector3(triangulation->Node(n1)),
                vector3(triangulation->Node(n2)),
                vector3(triangulation->Node(n3)));
        }
    } catch (...) {
        error = QCoreApplication::translate("OpenAutoPath", "转换统一 IPW 最终显示网格时发生异常。");
        return {};
    }
    if (mesh.empty()) {
        error = QCoreApplication::translate("OpenAutoPath", "统一 IPW 最终显示网格不包含有效三角形。");
    }
    return mesh;
}

RenderMesh makeRevolvedTool(
    const std::vector<std::pair<double, double>>& sideProfile,
    int slices)
{
    std::vector<std::pair<double, double>> profile;
    if (sideProfile.empty())
        return {};
    profile.reserve(sideProfile.size() + 1u);
    profile.emplace_back(0.0, sideProfile.front().second);
    profile.insert(
        profile.end(), sideProfile.begin(), sideProfile.end());

    RenderMesh mesh;
    slices = std::max(4, slices);
    const int angularSegments = slices;
    auto point = [&](std::size_t profileIndex, int slice) {
        const double angle = 2.0 * std::numbers::pi *
            static_cast<double>(slice) /
            static_cast<double>(angularSegments);
        const double radius = profile[profileIndex].first;
        return QVector3D(
            static_cast<float>(
                radius * std::cos(angle)),
            static_cast<float>(
                radius * std::sin(angle)),
            static_cast<float>(
                profile[profileIndex].second));
    };
    for (std::size_t ring = 0; ring + 1u < profile.size(); ++ring) {
        for (int slice = 0;
             slice < angularSegments; ++slice) {
            const QVector3D a0 = point(ring, slice);
            const QVector3D a1 = point(ring, slice + 1);
            const QVector3D b0 = point(ring + 1u, slice);
            const QVector3D b1 = point(ring + 1u, slice + 1);
            addTriangle(mesh, a0, b0, b1);
            addTriangle(mesh, a0, b1, a1);
        }
    }
    return mesh;
}

QMatrix4x4 transform(
    const gp_Vec& xAxis,
    const gp_Vec& yAxis,
    const gp_Vec& zAxis,
    const gp_Pnt& origin)
{
    QMatrix4x4 matrix;
    matrix.setColumn(0, QVector4D(vector3(xAxis), 0.0f));
    matrix.setColumn(1, QVector4D(vector3(yAxis), 0.0f));
    matrix.setColumn(2, QVector4D(vector3(zAxis), 0.0f));
    matrix.setColumn(3, QVector4D(vector3(origin), 1.0f));
    return matrix;
}

ToolFrame makeToolFrame(const gp_Vec& outwardAxis)
{
    gp_Vec w = outwardAxis;
    w.Normalize();
    const gp_Ax3 frame(
        gp_Pnt(0.0, 0.0, 0.0), gp_Dir(w));
    return {
        gp_Vec(frame.XDirection()),
        gp_Vec(frame.YDirection()),
        w};
}

QMatrix4x4 toolTransform(
    const ToolFrame& frame,
    const gp_Pnt& tip)
{
    return transform(frame.u, frame.v, frame.w, tip);
}

gp_Pnt interpolate(
    const gp_Pnt& from,
    const gp_Pnt& to,
    double fraction)
{
    gp_Pnt result = from;
    result.Translate(gp_Vec(from, to).Multiplied(
        std::clamp(fraction, 0.0, 1.0)));
    return result;
}

} // namespace oap::app::pocket_cam_detail
