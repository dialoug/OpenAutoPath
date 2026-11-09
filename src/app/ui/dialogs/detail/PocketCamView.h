// Rendering-independent controls and camera shared by the simulation backends.
#pragma once
#include "app/ui/dialogs/detail/PocketCamRenderSupport.h"
#include <QImage>
#include <QPointF>
#include <QWidget>

class QMouseEvent;
class QWheelEvent;
namespace oap::app::pocket_cam_detail {
enum class PocketCamBackend { OpenGL, Vulkan };

class PocketCamView {
public:
    explicit PocketCamView(PocketCamInteractionOptions interaction) : m_interaction(interaction) {}
    virtual ~PocketCamView() = default;
    virtual QWidget* widget() = 0;
    virtual const QWidget* widget() const = 0;
    virtual QImage grabFramebuffer() = 0;
    virtual std::size_t totalSteps() const noexcept = 0;
    virtual std::size_t currentStep() const noexcept = 0;
    virtual void setCurrentStep(std::size_t step) = 0;
    virtual bool playbackBusy() const noexcept = 0;
    virtual QString errorString() const = 0;
    virtual QString materialStatusString() const = 0;
    virtual bool supportsAmbientOcclusion() const { return true; }
    void setPathVisible(bool visible);
    void setModelMode(int mode);
    void setAmbientOcclusionEnabled(bool enabled);
    void setOrthographicProjection(bool enabled);
    void rotateTurntable(double degrees);
    void resetCamera();
    void setVisualAcceptanceView(const PocketCamVisualAcceptanceView& view);
protected:
    void handleMousePress(QMouseEvent* event);
    void handleMouseMove(QMouseEvent* event);
    void handleWheel(QWheelEvent* event);
    void handleDoubleClick(QMouseEvent* event);
    QVector3D arcballVector(const QPointF& position) const;
    void rotateCameraArcball(const QPointF& from, const QPointF& to);
    void rotateModelAroundAxis(float degrees, const QVector3D& axis);
    void rotateModelByPixels(const QPointF& delta);
    float cameraNearPlane() const;
    float cameraFarPlane() const;
    QMatrix4x4 cameraMatrix() const;
    PocketCamInteractionOptions m_interaction;
    QVector3D m_center, m_pan;
    QMatrix4x4 m_sceneTransform;
    QVector3D m_cameraDirection{0.624f, 0.624f, 0.469f};
    QVector3D m_cameraUp{-0.332f, -0.332f, 0.883f};
    float m_radius = 1.0f, m_distance = 3.0f;
    QPointF m_lastMouse;
    bool m_showPath = true, m_showCurrentTool = true;
    bool m_ambientOcclusionEnabled = false, m_orthographicProjection = false;
    int m_modelMode = 0;
};
} // namespace oap::app::pocket_cam_detail
