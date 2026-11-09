#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"
#include <algorithm>
#include <limits>

namespace oap::app::pocket_cam_detail {
void PocketCamOpenGLWidget::calculateBounds()
{
    QVector3D minimum(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());
    QVector3D maximum(
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max());
    bool hasPoint = false;
    const auto expand = [&](const QVector3D& point) {
        hasPoint = true;
        minimum.setX(std::min(
            minimum.x(), point.x()));
        minimum.setY(std::min(
            minimum.y(), point.y()));
        minimum.setZ(std::min(
            minimum.z(), point.z()));
        maximum.setX(std::max(
            maximum.x(), point.x()));
        maximum.setY(std::max(
            maximum.y(), point.y()));
        maximum.setZ(std::max(
            maximum.z(), point.z()));
    };
    const auto expandMesh = [&](const RenderMesh& mesh) {
        for (const RenderVertex& vertex : mesh.vertices)
            expand(vertex.position);
    };
    if (m_stockBoundsValid) {
        expand(m_stockBoundsMinimum);
        expand(m_stockBoundsMaximum);
    }
    if (!hasPoint)
        expandMesh(m_finalStockCpu);
    if (!hasPoint)
        expandMesh(m_targetCpu);
    if (!hasPoint) {
        for (const toolpath::PocketCamMotion& motion :
             m_scene.motions) {
            expand(vector3(motion.move.from));
            expand(vector3(motion.move.to));
        }
    }
    if (!hasPoint) {
        m_center = {};
        m_radius = 1.0f;
        resetCamera();
        return;
    }
    m_center = 0.5f * (minimum + maximum);
    m_radius = std::max(
        1.0e-3f, 0.5f * (maximum - minimum).length());
    resetCamera();
}

} // namespace oap::app::pocket_cam_detail
