#include "app/ui/dialogs/detail/PocketCamView.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <utility>
#include <QMouseEvent>
#include <QQuaternion>
#include <QWheelEvent>

namespace oap::app::pocket_cam_detail {

void PocketCamView::rotateTurntable(double degrees)
{
    if (!std::isfinite(degrees) ||
        std::abs(degrees) <=
            std::numeric_limits<double>::epsilon()) {
        return;
    }
    if (m_interaction.rotationMode ==
        PocketCamRotationMode::Model) {
        // 相机绕轨道转 +a 在视觉上等价于场景旋转 -a。模型模式下
        // 保持相机固定,同时保留屏幕上的既有转盘方向。
        rotateModelAroundAxis(
            -static_cast<float>(degrees),
            QVector3D(0.0f, 0.0f, 1.0f));
    } else {
        const QQuaternion rotation =
            QQuaternion::fromAxisAndAngle(
                QVector3D(0.0f, 0.0f, 1.0f),
                static_cast<float>(degrees));
        m_cameraDirection =
            rotation.rotatedVector(
                m_cameraDirection).normalized();
        m_cameraUp =
            rotation.rotatedVector(
                m_cameraUp).normalized();
    }
    widget()->update();
}

void PocketCamView::resetCamera()
{
    constexpr float yawDegrees = 45.0f;
    constexpr float pitchDegrees = 28.0f;
    const float yaw =
        yawDegrees *
        static_cast<float>(std::numbers::pi / 180.0);
    const float pitch =
        pitchDegrees *
        static_cast<float>(std::numbers::pi / 180.0);
    m_cameraDirection = QVector3D(
        std::cos(pitch) * std::cos(yaw),
        std::cos(pitch) * std::sin(yaw),
        std::sin(pitch)).normalized();
    const QVector3D worldUp(0.0f, 0.0f, 1.0f);
    QVector3D right = QVector3D::crossProduct(
        worldUp, m_cameraDirection);
    if (right.lengthSquared() <= 1.0e-12f)
        right = QVector3D(1.0f, 0.0f, 0.0f);
    right.normalize();
    m_cameraUp = QVector3D::crossProduct(
        m_cameraDirection, right).normalized();
    m_distance = std::max(1.0f, 2.8f * m_radius);
    m_pan = {};
    m_sceneTransform.setToIdentity();
    widget()->update();
}

void PocketCamView::setVisualAcceptanceView(
    const PocketCamVisualAcceptanceView& view)
{
    if (!view.valid ||
        !std::isfinite(view.halfHeight) ||
        view.halfHeight <= 0.0) {
        return;
    }
    m_center = vector3(view.center);
    m_cameraDirection =
        vector3(gp_Vec(view.eyeDirection))
            .normalized();
    m_cameraUp =
        vector3(gp_Vec(view.up)).normalized();
    constexpr float halfFieldRadians =
        19.0f * static_cast<float>(
            std::numbers::pi / 180.0);
    m_distance = std::max(
        1.0f,
        static_cast<float>(view.halfHeight) /
            std::tan(halfFieldRadians));
    m_pan = {};
    m_sceneTransform.setToIdentity();
    m_orthographicProjection = true;
    m_showPath = false;
    m_showCurrentTool = false;
    m_ambientOcclusionEnabled = false;
    m_modelMode = 0;
    widget()->update();
}

void PocketCamView::handleMousePress(QMouseEvent* event)
{
    m_lastMouse = event->position();
    event->accept();
}

void PocketCamView::handleMouseMove(QMouseEvent* event)
{
    const QPointF delta = event->position() - m_lastMouse;
    const bool rotating =
        m_interaction.rotateButton != Qt::NoButton &&
        event->buttons().testFlag(
            m_interaction.rotateButton);
    const bool panning =
        m_interaction.panButton != Qt::NoButton &&

        event->buttons().testFlag(
            m_interaction.panButton);
    if (rotating) {
        if (m_interaction.rotationMode ==
            PocketCamRotationMode::Model) {
            rotateModelByPixels(delta);
        } else {
            rotateCameraArcball(
                m_lastMouse, event->position());
        }
        widget()->update();
    } else if (panning) {
        const float scale =
            2.0f * m_distance /
            static_cast<float>(std::max(1, widget()->height()));
        const QVector3D direction =
            m_cameraDirection.normalized();
        const QVector3D up =
            m_cameraUp.normalized();
        const QVector3D right =
            QVector3D::crossProduct(
                up, direction).normalized();
        m_pan +=
            right *
                static_cast<float>(-delta.x()) * scale +
            up *
                static_cast<float>(delta.y()) * scale;
        widget()->update();
    }
    m_lastMouse = event->position();
    event->accept();
}

void PocketCamView::handleWheel(QWheelEvent* event)
{
    const double steps =
        static_cast<double>(event->angleDelta().y()) / 120.0;
    m_distance *= static_cast<float>(
        std::pow(0.85, steps));
    m_distance = std::clamp(
        m_distance,
        std::max(0.01f, 0.05f * m_radius),
        std::max(10.0f, 100.0f * m_radius));
    widget()->update();
    event->accept();
}

void PocketCamView::handleDoubleClick(QMouseEvent* event)
{
    resetCamera();
    event->accept();
}

QVector3D PocketCamView::arcballVector(const QPointF& position) const
{
    const float diameter = static_cast<float>(
        std::max(1, std::min(widget()->width(), widget()->height())));
    const float x = static_cast<float>(
        (2.0 * position.x() - widget()->width()) / diameter);
    const float y = static_cast<float>(
        (widget()->height() - 2.0 * position.y()) / diameter);
    const float radiusSquared = x * x + y * y;
    const float z = radiusSquared <= 0.5f
        ? std::sqrt(std::max(0.0f, 1.0f - radiusSquared))
        : 0.5f / std::sqrt(radiusSquared);
    return QVector3D(x, y, z).normalized();
}

void PocketCamView::rotateCameraArcball(
    const QPointF& from,
    const QPointF& to)
{
    const QVector3D fromVector = arcballVector(from);
    const QVector3D toVector = arcballVector(to);
    QVector3D axisInView =
        QVector3D::crossProduct(fromVector, toVector);
    if (axisInView.lengthSquared() <= 1.0e-12f)
        return;

    const float cosine = std::clamp(
        QVector3D::dotProduct(fromVector, toVector),
        -1.0f, 1.0f);
    const float angleDegrees =
        std::acos(cosine) *
        static_cast<float>(180.0 / std::numbers::pi);

    QVector3D direction =
        m_cameraDirection.normalized();
    QVector3D up = m_cameraUp.normalized();
    QVector3D right =
        QVector3D::crossProduct(up, direction);
    if (right.lengthSquared() <= 1.0e-12f)
        return;
    right.normalize();
    up = QVector3D::crossProduct(
        direction, right).normalized();

    axisInView.normalize();
    QVector3D axisInWorld =
        right * axisInView.x() +
        up * axisInView.y() +
        direction * axisInView.z();
    if (axisInWorld.lengthSquared() <= 1.0e-12f)
        return;
    axisInWorld.normalize();
    const QQuaternion rotation =
        QQuaternion::fromAxisAndAngle(
            // OCCT 的拖拽旋转使用与这里所用的 OpenGL 相机基
            // 相反的活动旋转符号。
            axisInWorld, -angleDegrees);
    direction =
        rotation.rotatedVector(direction).normalized();
    up = rotation.rotatedVector(up).normalized();

    // 移除累积的浮点偏移,同时保留 Arcball 的翻滚分量。
    // 与旧的固定世界向上相机不同,这个基可以越过任意极点
    // 并完全颠倒。
    right = QVector3D::crossProduct(up, direction);
    if (right.lengthSquared() <= 1.0e-12f)
        return;
    right.normalize();
    m_cameraDirection = direction;
    m_cameraUp = QVector3D::crossProduct(
        direction, right).normalized();
}

void PocketCamView::rotateModelAroundAxis(
    float degrees,
    const QVector3D& axis)
{
    if (!std::isfinite(degrees) ||
        std::abs(degrees) <=
            std::numeric_limits<float>::epsilon() ||
        axis.lengthSquared() <= 1.0e-12f) {
        return;
    }
    const QVector3D pivot = m_sceneTransform.map(m_center);
    QMatrix4x4 rotation;
    rotation.translate(pivot);
    rotation.rotate(degrees, axis.normalized());
    rotation.translate(-pivot);
    m_sceneTransform = rotation * m_sceneTransform;
}

void PocketCamView::rotateModelByPixels(const QPointF& delta)
{
    constexpr float radiansPerPixel = 0.008f;
    constexpr float degreesPerRadian =
        static_cast<float>(180.0 / std::numbers::pi);
    const float pixelScale =
        static_cast<float>(widget()->devicePixelRatioF());
    const float horizontalPixels =
        static_cast<float>(delta.x()) * pixelScale;
    const float verticalPixels =
        static_cast<float>(delta.y()) * pixelScale *
        (m_interaction.modelRotationVerticalInverted
             ? -1.0f
             : 1.0f);

    const QVector3D direction =
        m_cameraDirection.normalized();
    const QVector3D up = m_cameraUp.normalized();
    QVector3D right =
        QVector3D::crossProduct(up, direction);
    if (right.lengthSquared() <= 1.0e-12f)
        return;
    right.normalize();

    // 与 OcctView::rotateModelByPixels 一致:先绕相机上向量
    // 偏航,再绕相机右向量俯仰。
    rotateModelAroundAxis(
        -horizontalPixels * radiansPerPixel *
            degreesPerRadian,
        up);
    rotateModelAroundAxis(
        -verticalPixels * radiansPerPixel *
            degreesPerRadian,
        right);
}

float PocketCamView::cameraNearPlane() const
{
    return std::max(
        0.001f, 0.001f * m_radius);
}

float PocketCamView::cameraFarPlane() const
{
    return std::max(
        100.0f,
        m_distance + 20.0f * m_radius);
}

QMatrix4x4 PocketCamView::cameraMatrix() const
{
    const QVector3D direction =
        m_cameraDirection.normalized();
    const QVector3D target = m_center + m_pan;
    const QVector3D eye =
        target + direction * m_distance;
    QMatrix4x4 view;
    view.lookAt(eye, target, m_cameraUp);
    QMatrix4x4 projection;
    const float aspect =
        static_cast<float>(std::max(1, widget()->width())) /
        static_cast<float>(std::max(1, widget()->height()));
    if (m_orthographicProjection) {
        const float halfHeight =
            std::max(
                0.001f,
                m_distance *
                    std::tan(
                        19.0f *
                        static_cast<float>(
                            std::numbers::pi / 180.0)));
        const float halfWidth =
            halfHeight * aspect;
        projection.ortho(
            -halfWidth, halfWidth,
            -halfHeight, halfHeight,
            cameraNearPlane(),
            cameraFarPlane());
    } else {
        projection.perspective(
            38.0f, aspect,
            cameraNearPlane(),
            cameraFarPlane());
    }
    return projection * view;
}

void PocketCamView::setPathVisible(bool visible)
{
    if (m_showPath == visible)
        return;
    m_showPath = visible;
    widget()->update();
}

void PocketCamView::setModelMode(int mode)
{
    mode = std::clamp(mode, 0, 2);
    if (m_modelMode == mode)
        return;
    m_modelMode = mode;
    widget()->update();
}

void PocketCamView::setAmbientOcclusionEnabled(bool enabled)
{
    if (m_ambientOcclusionEnabled == enabled)
        return;
    m_ambientOcclusionEnabled = enabled;
    widget()->update();
}

void PocketCamView::setOrthographicProjection(bool enabled)
{
    if (m_orthographicProjection == enabled)
        return;
    m_orthographicProjection = enabled;
    widget()->update();
}

} // namespace oap::app::pocket_cam_detail
