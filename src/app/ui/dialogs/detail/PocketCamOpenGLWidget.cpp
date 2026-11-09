#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <utility>
#include <QOpenGLContext>
#include <QSizePolicy>
#include <QSurfaceFormat>

namespace oap::app::pocket_cam_detail {

PocketCamOpenGLWidget::PocketCamOpenGLWidget(
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
    QWidget* parent)
    : QOpenGLWidget(parent), PocketCamView(interaction),
      m_scene(std::move(scene)),
      m_frame(makeToolFrame(m_scene.outwardToolAxis)),
      m_finalStockCpu(std::move(finalStock)),
      m_targetCpu(std::move(target)),
      m_initialGpuStock(std::move(initialIpwStock)),
      m_finalGpuStock(std::move(finalIpwStock)),
      m_operationInitialIpwSources(
          std::move(operationInitialIpwSources)),
      m_stockBoundsMinimum(stockBoundsMinimum),
      m_stockBoundsMaximum(stockBoundsMaximum),
      m_stockBoundsValid(stockBoundsValid)
{
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CoreProfile);
    // 优先使用 GPU Multi-Dexel；权威项目播放可在上传超预算时使用
    // 同一材料检查点的有界显示网格。两者均需要当前渲染上下文。
    format.setVersion(4, 3);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(0);
    // 请求 4x MSAA 作为离屏后处理不可用时的回退；正常路径还会在
    // 单采样颜色纹理上执行 FXAA，因此启用 SSAO 时也能平滑轮廓。
    format.setSamples(4);
    setFormat(format);
    setMinimumSize(240, 160);
    setSizePolicy(
        QSizePolicy::Expanding,
        QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);

    // 当前刀具显示仍使用一份回转网格；材料减除由 Compute Shader 完成。
    const int slices = m_scene.quality < 3.0
        ? 12 : (m_scene.quality < 7.0 ? 32 : 64);
    m_toolCpu.reserve(m_scene.tools.size());
    for (const toolpath::PocketCamTool& tool :
         m_scene.tools) {
        m_toolCpu.push_back(makeRevolvedTool(
            tool.profile, slices));
    }
    prepareInstanceTimeline();
    calculateBounds();
    rebuildVisibleInstances();
    if (!m_operationInitialIpwSources.empty()) {
        m_baselineController =
            std::make_unique<IpwDisplayMeshController>();
        connect(
            m_baselineController.get(),
            &IpwDisplayMeshController::resultReady,
            this,
            [this](const IpwDisplayMeshResult& result) {
                if (m_scene.authoritativeIpwPlayback) acceptAuthoritativeFrame(result);
                else acceptOperationBaseline(result);
            });
    }
}

PocketCamOpenGLWidget::~PocketCamOpenGLWidget()
{
    if (m_baselineController)
        m_baselineController->cancelAndWait();
    QOpenGLContext* glContext = context();
    if (!glContext || !glContext->isValid())
        return;
    makeCurrent();
    if (QOpenGLContext::currentContext() != glContext)
        return;
    destroyAnalyticBoundarySnapshots();
    destroyGpuMesh(m_finalStockGpu);
    destroyGpuMesh(m_playbackStockGpu);
    destroyGpuMesh(m_targetGpu);
    destroyGpuDexelState();
    for (GpuMesh& mesh : m_toolGpu)
        destroyGpuMesh(mesh);
    if (m_lineVao != 0u)
        glDeleteVertexArrays(1, &m_lineVao);
    if (m_lineVbo != 0u)
        glDeleteBuffers(1, &m_lineVbo);
    destroyPostFramebuffer();
    if (m_screenVao != 0u)
        glDeleteVertexArrays(1, &m_screenVao);
    doneCurrent();
}

std::size_t PocketCamOpenGLWidget::totalSteps() const noexcept
{
    return m_scene.totalSimulationSteps;
}

std::size_t PocketCamOpenGLWidget::currentStep() const noexcept
{
    return m_currentStep;
}

QString PocketCamOpenGLWidget::errorString() const
{
    return m_error;
}

QString PocketCamOpenGLWidget::takeOpenGlErrors()
{
    GLenum error = glGetError();
    if (error == GL_NO_ERROR)
        return {};

    QString errorCodes;
    while (error != GL_NO_ERROR) {
        if (!errorCodes.isEmpty())
            errorCodes += QStringLiteral(", ");
        errorCodes += QStringLiteral("0x%1").arg(
            static_cast<unsigned int>(error), 0, 16);
        error = glGetError();
    }
    return errorCodes;
}

void PocketCamOpenGLWidget::beginOpenGlStage(const QString& stage)
{
    const QString errorCodes = takeOpenGlErrors();
    if (errorCodes.isEmpty())
        return;
    if (m_preexistingOpenGlWarning.isEmpty()) {
        m_preexistingOpenGlWarning = QStringLiteral(
            "%1前存在外部/历史 OpenGL 错误 %2，"
            "已记录并继续检查本阶段。")
            .arg(stage, errorCodes);
    }
}

bool PocketCamOpenGLWidget::checkOpenGlStage(const QString& stage)
{
    const QString errorCodes = takeOpenGlErrors();
    if (errorCodes.isEmpty())
        return true;
    if (m_error.isEmpty()) {
        m_error = QStringLiteral(
            "%1失败，OpenGL 错误：%2")
            .arg(stage, errorCodes);
    }
    return false;
}

} // namespace oap::app::pocket_cam_detail
