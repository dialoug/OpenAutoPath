// Private GL view. Owns its context-bound resources and operation baseline lifecycle.
// Method implementations are compiled by rendering, camera and playback responsibility.
#pragma once

#include "app/ui/dialogs/detail/PocketCamView.h"
#include <cstddef>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <QColor>
#include <QMatrix4x4>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPointF>
#include <QVector2D>
#include <QVector3D>
#include "app/ui/dialogs/detail/PocketCamRenderSupport.h"
#include "app/workflows/simulation/IpwDisplayMeshController.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"

namespace oap::app::pocket_cam_detail {

class PocketCamOpenGLWidget final
    : public QOpenGLWidget,
      protected QOpenGLFunctions_3_3_Core, public PocketCamView {
public:
    PocketCamOpenGLWidget(
        toolpath::PocketCamSimulationScene scene,
        RenderMesh finalStock,
        RenderMesh target,
        PocketCamInteractionOptions interaction,
        std::shared_ptr<const toolpath::MultiDexelStock>
            initialIpwStock,
        std::shared_ptr<const toolpath::MultiDexelStock>
            finalIpwStock,
        std::vector<std::shared_ptr<const IpwTransientFrameSource>>
            operationInitialIpwSources,
        QVector3D stockBoundsMinimum,
        QVector3D stockBoundsMaximum,
        bool stockBoundsValid,
        QWidget* parent = nullptr);

    ~PocketCamOpenGLWidget() override;
    QWidget* widget() override { return this; }
    const QWidget* widget() const override { return this; }
    QImage grabFramebuffer() override { return QOpenGLWidget::grabFramebuffer(); }

    std::size_t totalSteps() const noexcept;

    std::size_t currentStep() const noexcept;

    QString errorString() const;

    QString materialStatusString() const;
    bool usingGpuMaterial() const { return m_gpuDexelReady; }
    // Synchronous acceptance diagnostic, excluded from frame timing.
    double displayedGpuVolume();
    QString gpuDeviceName();

    void setCurrentStep(std::size_t step);
    bool playbackBusy() const noexcept
    { return m_scene.authoritativeIpwPlayback && (!m_gpuReady || m_waitingForOperationBaseline); }

protected:
    void initializeGL() override;

    void resizeGL(int width, int height) override;

    void paintGL() override;

    void renderScene(const QMatrix4x4& viewProjection);

    void clearSceneFramebuffer();

    void renderStockLayer(const QMatrix4x4& viewProjection);

    void renderSceneOverlays(
        const QMatrix4x4& viewProjection);

    void mousePressEvent(QMouseEvent* event) override { handleMousePress(event); }

    void mouseMoveEvent(QMouseEvent* event) override { handleMouseMove(event); }

    void wheelEvent(QWheelEvent* event) override { handleWheel(event); }

    void mouseDoubleClickEvent(QMouseEvent* event) override { handleDoubleClick(event); }

private:
    void setAuthoritativeStep(std::size_t step);
    void acceptAuthoritativeFrame(const IpwDisplayMeshResult& result);
    std::size_t m_requestedPlaybackStep = 0;
    std::size_t m_requestedMaterialMoveCount = 0;
    std::size_t m_materialMoveCount = 0;
    int m_requestedMaterialOperation = -1;
    int m_materialOperation = -1;
    std::shared_ptr<const toolpath::MultiDexelStock> m_materialStock;
    struct GpuMesh {
        GLuint vao = 0u;
        GLuint vertexBuffer = 0u;
        GLuint indexBuffer = 0u;
        GLsizei indexCount = 0;
    };

    struct AnalyticBoundarySnapshot {
        int operationIndex = -1;
        GLuint buffer = 0u;
    };

    void destroyAnalyticBoundarySnapshots();

    bool cacheAnalyticBoundarySnapshot(int operationIndex);

    bool restoreAnalyticBoundarySnapshot(int operationIndex);

    bool initializeGpuDexelPrograms();

    void destroyGpuDexelState();

    bool uploadGpuDexelStock(
        const toolpath::MultiDexelStock& stock,
        std::size_t appliedStep = 0u);

    // Authoritative playback can render a bounded mesh from the same CPU
    // checkpoint when its dense SSBO representation exceeds GPU limits.
    bool uploadPlaybackStock(const toolpath::MultiDexelStock& stock,
                             std::size_t appliedStep = 0u,
                             const Handle(Poly_Triangulation)& preparedMesh = {});

    QVector3D gpuDexelCoordinates(const gp_Pnt& point) const;

    QString takeOpenGlErrors();

    void beginOpenGlStage(const QString& stage);

    bool checkOpenGlStage(const QString& stage);

    bool dispatchGpuDexelMotion(
        const toolpath::PocketCamMotion& motion,
        double fromFraction,
        double toFraction);

    bool prepareGpuDexelSurface();

    void syncGpuDexelToCurrentStep();

    void renderGpuDexel(const QMatrix4x4& viewProjection);

    void destroyPostFramebuffer();

    bool ensurePostFramebuffer(
        int pixelWidth,
        int pixelHeight);

    void renderPostProcess(
        int pixelWidth,
        int pixelHeight);

    bool compileProgram(
        QOpenGLShaderProgram& program,
        const char* vertex,
        const char* fragment,
        const QString& label);

    void uploadMesh(const RenderMesh& source, GpuMesh& target);

    void destroyGpuMesh(GpuMesh& mesh);

    void calculateBounds();

    std::size_t operationFirstStep(int operationIndex) const;

    bool requestOperationBaseline(int operationIndex);

    void acceptOperationBaseline(
        const IpwDisplayMeshResult& result);

    void prepareInstanceTimeline();

    void rebuildVisibleInstances();

    void uploadPathLines();

    void bindPlain(
        const QMatrix4x4& viewProjection,
        const QColor& color,
        const QMatrix4x4& model = QMatrix4x4());

    void drawPlain(const GpuMesh& mesh);

    void renderAuthoritativeFinalStock(
        const QMatrix4x4& viewProjection);

    void renderTarget(const QMatrix4x4& viewProjection);

    void renderPath(const QMatrix4x4& viewProjection);

    void renderCurrentTool(
        const QMatrix4x4& viewProjection);

    toolpath::PocketCamSimulationScene m_scene;
    ToolFrame m_frame;
    RenderMesh m_finalStockCpu;
    RenderMesh m_targetCpu;
    std::shared_ptr<const toolpath::MultiDexelStock>
        m_initialGpuStock;
    std::shared_ptr<const toolpath::MultiDexelStock>
        m_finalGpuStock;
    std::vector<std::shared_ptr<const IpwTransientFrameSource>>
        m_operationInitialIpwSources;
    std::unique_ptr<IpwDisplayMeshController>
        m_baselineController;
    QVector3D m_stockBoundsMinimum;
    QVector3D m_stockBoundsMaximum;
    bool m_stockBoundsValid = false;
    std::vector<RenderMesh> m_toolCpu;
    GpuMesh m_finalStockGpu;
    GpuMesh m_playbackStockGpu;
    bool m_usePlaybackMesh = false;
    QString m_playbackDisplayWarning;
    GpuMesh m_targetGpu;
    std::vector<GpuMesh> m_toolGpu;
    GLuint m_lineVao = 0u;
    GLuint m_lineVbo = 0u;
    GLuint m_screenVao = 0u;
    GLuint m_sceneFramebuffer = 0u;
    GLuint m_sceneColorTexture = 0u;
    GLuint m_sceneDepthTexture = 0u;
    GLsizei m_lineVertexCount = 0;
    int m_sceneFramebufferWidth = 0;
    int m_sceneFramebufferHeight = 0;
    QOpenGLShaderProgram m_plainProgram;
    QOpenGLShaderProgram m_lineProgram;
    QOpenGLShaderProgram m_postProgram;
    QOpenGLShaderProgram m_dexelComputeProgram;
    QOpenGLShaderProgram m_dexelSurfaceProgram;
    QOpenGLShaderProgram m_dexelRenderProgram;
    QOpenGLFunctions_4_3_Core* m_gl43 = nullptr;
    GLuint m_dexelIntervalsBuffer = 0u;
    GLuint m_dexelCountsBuffer = 0u;
    GLuint m_dexelBoundaryDistanceBuffer = 0u;
    GLuint m_dexelStockDomainBuffer = 0u;
    GLuint m_dexelCoarseMaskBuffer = 0u;
    GLuint m_dexelCoarseEntriesBuffer = 0u;
    GLuint m_dexelFineEntriesBuffer = 0u;
    GLuint m_dexelFineDrawCommandBuffer = 0u;
    std::array<GLsizeiptr, 8> m_dexelBufferBytes{};
    bool m_dexelComputeUniformsDirty = true;
    bool m_dexelSurfaceUniformsDirty = true;
    bool m_dexelRenderUniformsDirty = true;
    int m_dexelComputeTool = -1;
    GLuint m_dexelVao = 0u;
    std::uint32_t m_dexelFineCapacity = 0u;
    int m_dexelNx = 0;
    int m_dexelNy = 0;
    int m_dexelRenderNx = 0;
    int m_dexelRenderNy = 0;
    int m_dexelRenderStride = 1;
    int m_dexelMaximumIntervals = 0;
    float m_dexelResolution = 0.0f;
    QVector2D m_dexelGridMinimum;
    QVector2D m_dexelGridMaximum;
    gp_Pnt m_dexelOriginPoint;
    gp_Vec m_dexelUAxisVector;
    gp_Vec m_dexelVAxisVector;
    gp_Vec m_dexelWAxisVector;
    QVector3D m_dexelOrigin;
    QVector3D m_dexelUAxis;
    QVector3D m_dexelVAxis;
    QVector3D m_dexelWAxis;
    std::vector<std::size_t> m_motionEndSteps;
    std::vector<std::size_t> m_operationFirstMotionIndices;
    std::vector<AnalyticBoundarySnapshot>
        m_analyticBoundarySnapshots;
    int m_analyticBoundarySnapshotNx = 0;
    int m_analyticBoundarySnapshotNy = 0;
    QMatrix4x4 m_currentToolTransform;
    gp_Pnt m_currentTip;
    std::size_t m_currentStep = 0u;
    int m_currentToolIndex = -1;
    int m_currentOperationIndex = -1;
    int m_requestedBaselineOperationIndex = -1;
    std::size_t m_gpuDexelAppliedStep = 0u;
    std::size_t m_gpuDexelSourceIntervalCount = 0u;
    std::size_t m_gpuDexelOccupiedCellCount = 0u;
    bool m_gpuDexelSupported = false;
    bool m_gpuDexelReady = false;
    bool m_gpuDexelSurfaceDirty = false;
    bool m_waitingForOperationBaseline = false;
    bool m_gpuReady = false;
    QString m_preexistingOpenGlWarning;
    QString m_error;
};

} // namespace oap::app::pocket_cam_detail
