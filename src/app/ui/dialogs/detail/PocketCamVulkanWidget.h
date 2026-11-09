// Vulkan GPU material simulation with authoritative CPU IPW compatibility playback.
#pragma once
#include "app/ui/dialogs/detail/PocketCamView.h"
#include "app/workflows/simulation/IpwDisplayMeshController.h"
#include <QRhiWidget>
#include <memory>

namespace oap::app::pocket_cam_detail {
class PocketCamVulkanWidget final : public QRhiWidget, public PocketCamView {
public:
    PocketCamVulkanWidget(toolpath::PocketCamSimulationScene scene,
        RenderMesh target, PocketCamInteractionOptions interaction,
        std::shared_ptr<const toolpath::MultiDexelStock> initial,
        std::shared_ptr<const toolpath::MultiDexelStock> finalStock,
        std::vector<std::shared_ptr<const IpwTransientFrameSource>> sources,
        QVector3D minimum, QVector3D maximum, bool boundsValid, QWidget* parent = nullptr);
    ~PocketCamVulkanWidget() override;
    QWidget* widget() override { return this; }
    const QWidget* widget() const override { return this; }
    QImage grabFramebuffer() override { return QRhiWidget::grabFramebuffer(); }
    std::size_t totalSteps() const noexcept override { return m_scene.totalSimulationSteps; }
    std::size_t currentStep() const noexcept override { return m_step; }
    void setCurrentStep(std::size_t step) override;
    bool playbackBusy() const noexcept override { return m_pending || !m_materialReady || !m_renderReady; }
    QString errorString() const override { return m_error; }
    QString materialStatusString() const override;
    // Explicit acceptance diagnostics; GPU volume readback is never used during playback.
    bool usingVulkan() const;
    bool usingGpuMaterial() const;
    std::size_t gpuDispatchCount() const;
    QString gpuDeviceName() const;
    double displayedVolume() const;
protected:
    void initialize(QRhiCommandBuffer*) override;
    void render(QRhiCommandBuffer*) override;
    void releaseResources() override;
    void mousePressEvent(QMouseEvent* e) override { handleMousePress(e); }
    void mouseMoveEvent(QMouseEvent* e) override { handleMouseMove(e); }
    void wheelEvent(QWheelEvent* e) override { handleWheel(e); }
    void mouseDoubleClickEvent(QMouseEvent* e) override { handleDoubleClick(e); }
private:
    struct Renderer;
    struct RendererDeleter { void operator()(Renderer*) const; };
    std::unique_ptr<Renderer, RendererDeleter> m_renderer;
    void acceptFrame(const IpwDisplayMeshResult& result);
    void requestFrame(std::size_t step);
    void updateTool();
    struct SourcePath {
        const toolpath::PocketToolpathOperation* operation = nullptr;
        std::vector<double> distances;
    };
    toolpath::PocketCamSimulationScene m_scene;
    RenderMesh m_target, m_material, m_path;
    std::vector<RenderMesh> m_tools;
    std::shared_ptr<const toolpath::MultiDexelStock> m_initial, m_final, m_stock, m_completedStock;
    std::vector<std::shared_ptr<const IpwTransientFrameSource>> m_sources;
    std::unique_ptr<IpwDisplayMeshController> m_controller;
    std::vector<std::size_t> m_motionEnds;
    std::vector<SourcePath> m_sourcePaths;
    gp_Pnt m_tip, m_requestedTip;
    std::size_t m_step = 0, m_requestedStep = 0, m_materialMoves = 0, m_requestedMoves = 0;
    int m_operation = -1, m_requestedOperation = -1, m_tool = -1;
    QMatrix4x4 m_toolTransform;
    QString m_error;
    bool m_pending = false, m_materialReady = false, m_renderReady = false;
    bool m_materialDirty = true;
    bool m_gpuPlayback = false;
    bool m_gpuSurface = true;
    QString m_gpuFallbackReason;
};
} // namespace oap::app::pocket_cam_detail
