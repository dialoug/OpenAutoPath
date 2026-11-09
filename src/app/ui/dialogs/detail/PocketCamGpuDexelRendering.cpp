#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <utility>
#include <QVector4D>

namespace oap::app::pocket_cam_detail {

bool PocketCamOpenGLWidget::dispatchGpuDexelMotion(
    const toolpath::PocketCamMotion& motion,
    double fromFraction,
    double toFraction)
{
    if (!m_gpuDexelReady || !m_gl43 ||
        motion.toolIndex < 0 ||
        static_cast<std::size_t>(motion.toolIndex) >=
            m_scene.tools.size() ||
        motion.move.type == toolpath::ToolpathMoveType::Rapid ||
        motion.move.type == toolpath::ToolpathMoveType::Retract) {
        return true;
    }
    const toolpath::PocketCamTool& tool =
        m_scene.tools[static_cast<std::size_t>(motion.toolIndex)];
    if (tool.profile.empty() || tool.profile.size() > 32u)
        return true;
    fromFraction = std::clamp(fromFraction, 0.0, 1.0);
    toFraction = std::clamp(toFraction, fromFraction, 1.0);
    const gp_Pnt from = motion.playbackPointAt(fromFraction);
    const gp_Pnt to = motion.playbackPointAt(toFraction);
    const QVector3D localFrom = gpuDexelCoordinates(from);
    const QVector3D localTo = gpuDexelCoordinates(to);
    const bool analyticArc =
        motion.analyticPlaybackTrusted &&
        tool.flatEnd &&
        motion.analyticSweep.kind ==
            toolpath::PocketCamAnalyticSweepKind::CircularArc &&
        motion.analyticSweep.sweepRadians > 0.0;
    const QVector3D localArcCenter = analyticArc
        ? gpuDexelCoordinates(motion.analyticSweep.center)
        : QVector3D();
    float arcOrientation = 1.0f;
    float arcSweepRadians = 0.0f;
    if (analyticArc) {
        gp_Vec arcAxis = motion.analyticSweep.axis;
        arcAxis.Normalize();
        const double direction =
            motion.analyticSweep.direction ==
                    toolpath::ToolpathArcDirection::Counterclockwise
                ? 1.0 : -1.0;
        arcOrientation = static_cast<float>(
            direction *
            (arcAxis.Dot(m_dexelWAxisVector) >= 0.0
                 ? 1.0 : -1.0));
        arcSweepRadians = static_cast<float>(
            motion.analyticSweep.sweepRadians *
            (toFraction - fromFraction));
    }
    float maximumRadius = 0.0f;
    std::array<QVector2D, 32> profile{};
    for (std::size_t index = 0u;
         index < tool.profile.size(); ++index) {
        profile[index] = QVector2D(
            static_cast<float>(tool.profile[index].first),
            static_cast<float>(tool.profile[index].second));
        maximumRadius = std::max(
            maximumRadius, profile[index].x());
    }
    const float narrowBand = 2.0f * m_dexelResolution;
    float pathULow = std::min(localFrom.x(), localTo.x());
    float pathUHigh = std::max(localFrom.x(), localTo.x());
    float pathVLow = std::min(localFrom.y(), localTo.y());
    float pathVHigh = std::max(localFrom.y(), localTo.y());
    if (analyticArc && arcSweepRadians > 0.0f) {
        const QVector2D radial(
            localFrom.x() - localArcCenter.x(),
            localFrom.y() - localArcCenter.y());
        const float centerRadius = radial.length();
        const float startAngle = std::atan2(
            radial.y(), radial.x());
        constexpr float twoPi =
            2.0f * static_cast<float>(std::numbers::pi);
        const auto directedAngle = [&](float candidate) {
            float delta = arcOrientation *
                (candidate - startAngle);
            while (delta < 0.0f)
                delta += twoPi;
            while (delta >= twoPi)
                delta -= twoPi;
            return delta;
        };
        for (int quadrant = 0; quadrant < 4; ++quadrant) {
            const float angle =
                0.5f * static_cast<float>(std::numbers::pi) *
                static_cast<float>(quadrant);
            if (directedAngle(angle) >
                arcSweepRadians + 1.0e-5f) {
                continue;
            }
            const float u = localArcCenter.x() +
                centerRadius * std::cos(angle);
            const float v = localArcCenter.y() +
                centerRadius * std::sin(angle);
            pathULow = std::min(pathULow, u);
            pathUHigh = std::max(pathUHigh, u);
            pathVLow = std::min(pathVLow, v);
            pathVHigh = std::max(pathVHigh, v);
        }
    }
    const float uLow =
        pathULow - maximumRadius - narrowBand;
    const float uHigh =
        pathUHigh + maximumRadius + narrowBand;
    const float vLow =
        pathVLow - maximumRadius - narrowBand;
    const float vHigh =
        pathVHigh + maximumRadius + narrowBand;
    const int iMin = std::clamp(
        static_cast<int>(std::floor(
            (uLow - m_dexelGridMinimum.x()) /
            m_dexelResolution)),
        0, m_dexelNx);
    const int iMax = std::clamp(
        static_cast<int>(std::ceil(
            (uHigh - m_dexelGridMinimum.x()) /
            m_dexelResolution)),
        0, m_dexelNx);
    const int jMin = std::clamp(
        static_cast<int>(std::floor(
            (vLow - m_dexelGridMinimum.y()) /
            m_dexelResolution)),
        0, m_dexelNy);
    const int jMax = std::clamp(
        static_cast<int>(std::ceil(
            (vHigh - m_dexelGridMinimum.y()) /
            m_dexelResolution)),
        0, m_dexelNy);
    if (iMin >= iMax || jMin >= jMax)
        return true;

    beginOpenGlStage(
        QStringLiteral("GPU Multi-Dexel 减料"));
    m_dexelComputeProgram.bind();
    if (m_dexelComputeUniformsDirty) {
        m_gl43->glUniform2i(m_dexelComputeProgram.uniformLocation("uGridSize"), m_dexelNx, m_dexelNy);
        m_dexelComputeProgram.setUniformValue("uResolution", m_dexelResolution);
        m_dexelComputeProgram.setUniformValue("uGridMinimum", m_dexelGridMinimum);
        m_dexelComputeProgram.setUniformValue("uMaximumIntervals", m_dexelMaximumIntervals);
        m_dexelComputeProgram.setUniformValue("uNarrowBand", narrowBand);
        m_dexelComputeUniformsDirty = false;
    }
    if (m_dexelComputeTool != motion.toolIndex) {
        m_dexelComputeProgram.setUniformValue("uProfileCount", static_cast<int>(tool.profile.size()));
        m_dexelComputeProgram.setUniformValueArray("uProfile", profile.data(), static_cast<int>(tool.profile.size()));
        m_dexelComputeProgram.setUniformValue("uMaximumRadius", maximumRadius);
        m_dexelComputeTool = motion.toolIndex;
    }
    m_gl43->glUniform2i(
        m_dexelComputeProgram.uniformLocation("uDispatchBase"),
        iMin, jMin);
    m_gl43->glUniform2i(
        m_dexelComputeProgram.uniformLocation("uDispatchSize"),
        iMax - iMin, jMax - jMin);
    m_dexelComputeProgram.setUniformValue(
        "uMoveFrom", localFrom);
    m_dexelComputeProgram.setUniformValue(
        "uMoveTo", localTo);
    m_dexelComputeProgram.setUniformValue(
        "uSweepKind", analyticArc ? 2 : 1);
    m_dexelComputeProgram.setUniformValue(
        "uArcCenter",
        QVector2D(
            localArcCenter.x(), localArcCenter.y()));
    m_dexelComputeProgram.setUniformValue(
        "uArcSweepRadians", arcSweepRadians);
    m_dexelComputeProgram.setUniformValue(
        "uArcOrientation", arcOrientation);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 0u,
        m_dexelIntervalsBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 1u,
        m_dexelCountsBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 5u,
        m_dexelBoundaryDistanceBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 6u,
        m_dexelStockDomainBuffer);
    if (!checkOpenGlStage(
            QStringLiteral("GPU Multi-Dexel 减料参数上传"))) {
        return false;
    }
    m_gl43->glDispatchCompute(
        static_cast<GLuint>((iMax - iMin + 15) / 16),
        static_cast<GLuint>((jMax - jMin + 15) / 16),
        1u);
    if (!checkOpenGlStage(
            QStringLiteral("GPU Multi-Dexel Compute 派发"))) {
        return false;
    }
    m_gl43->glMemoryBarrier(
        GL_SHADER_STORAGE_BARRIER_BIT);
    if (!checkOpenGlStage(
            QStringLiteral("GPU Multi-Dexel 减料同步"))) {
        return false;
    }
    m_gpuDexelSurfaceDirty = true;
    return true;
}

bool PocketCamOpenGLWidget::prepareGpuDexelSurface()
{
    if (!m_gpuDexelSurfaceDirty)
        return true;
    if (!m_gl43 || m_dexelCoarseMaskBuffer == 0u || m_dexelCoarseEntriesBuffer == 0u ||
        m_dexelFineEntriesBuffer == 0u ||
        m_dexelFineDrawCommandBuffer == 0u ||
        m_dexelFineCapacity == 0u) {
        return false;
    }

    beginOpenGlStage(
        QStringLiteral("GPU Multi-Dexel 自适应表面分类"));
    const std::array<std::uint32_t, 10> drawCommand{
        150u, 0u, 0u, 0u, 0u, 0u, 12u, 0u, 0u, 0u};
    m_gl43->glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    m_gl43->glBindBuffer(
        GL_SHADER_STORAGE_BUFFER,
        m_dexelFineDrawCommandBuffer);
    m_gl43->glBufferSubData(
        GL_SHADER_STORAGE_BUFFER, 0,
        static_cast<GLsizeiptr>(sizeof(drawCommand)),
        drawCommand.data());
    if (!checkOpenGlStage(QStringLiteral(
            "GPU Multi-Dexel 表面分类命令缓冲重置"))) {
        return false;
    }
    if (!m_dexelSurfaceProgram.bind()) {
        if (m_error.isEmpty()) {
            m_error = QStringLiteral(
                "GPU Multi-Dexel 表面分类程序绑定失败：%1")
                          .arg(m_dexelSurfaceProgram.log());
        }
        return false;
    }
    if (m_dexelSurfaceUniformsDirty) {
        m_gl43->glUniform2i(m_dexelSurfaceProgram.uniformLocation("uGridSize"), m_dexelNx, m_dexelNy);
        m_gl43->glUniform2i(m_dexelSurfaceProgram.uniformLocation("uRenderGridSize"), m_dexelRenderNx, m_dexelRenderNy);
        m_dexelSurfaceProgram.setUniformValue("uRenderStride", m_dexelRenderStride);
        m_dexelSurfaceProgram.setUniformValue("uMaximumIntervals", m_dexelMaximumIntervals);
        m_gl43->glUniform1ui(m_dexelSurfaceProgram.uniformLocation("uFineCapacity"), m_dexelFineCapacity);
        m_dexelSurfaceUniformsDirty = false;
    }
    if (!checkOpenGlStage(QStringLiteral(
            "GPU Multi-Dexel 表面分类 uniform 上传"))) {
        return false;
    }
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 0u,
        m_dexelIntervalsBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 1u,
        m_dexelCountsBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 2u,
        m_dexelCoarseMaskBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 3u,
        m_dexelFineEntriesBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 5u,
        m_dexelBoundaryDistanceBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 4u,
        m_dexelFineDrawCommandBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 7u,
        m_dexelCoarseEntriesBuffer);
    if (!checkOpenGlStage(
            QStringLiteral("GPU Multi-Dexel 表面分类 SSBO 绑定"))) {
        return false;
    }
    m_gl43->glDispatchCompute(
        static_cast<GLuint>((m_dexelRenderNx + 7) / 8),
        static_cast<GLuint>((m_dexelRenderNy + 7) / 8),
        static_cast<GLuint>(m_dexelMaximumIntervals));
    if (!checkOpenGlStage(
            QStringLiteral("GPU Multi-Dexel 表面分类派发"))) {
        return false;
    }
    m_gl43->glMemoryBarrier(
        GL_SHADER_STORAGE_BARRIER_BIT |
        GL_COMMAND_BARRIER_BIT);
    if (!checkOpenGlStage(
            QStringLiteral("GPU Multi-Dexel 表面分类同步"))) {
        return false;
    }
    m_gpuDexelSurfaceDirty = false;
    return true;
}

void PocketCamOpenGLWidget::renderGpuDexel(const QMatrix4x4& viewProjection)
{
    if (!m_gpuDexelReady || !m_gl43)
        return;
    if (!prepareGpuDexelSurface()) {
        if (m_error.isEmpty()) {
            m_error = QStringLiteral(
                "GPU Multi-Dexel 自适应表面无法生成。");
        }
        return;
    }
    beginOpenGlStage(
        QStringLiteral("GPU Multi-Dexel 绘制"));
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glDisable(GL_CULL_FACE);
    m_dexelRenderProgram.bind();
    m_dexelRenderProgram.setUniformValue(
        "uModelViewProjection", viewProjection * m_sceneTransform);
    m_dexelRenderProgram.setUniformValue(
        "uSceneTransform", m_sceneTransform);
    if (m_dexelRenderUniformsDirty) {
        m_dexelRenderProgram.setUniformValue(
            "uOrigin", m_dexelOrigin);
        m_dexelRenderProgram.setUniformValue(
            "uUAxis", m_dexelUAxis);
        m_dexelRenderProgram.setUniformValue(
            "uVAxis", m_dexelVAxis);
        m_dexelRenderProgram.setUniformValue(
            "uWAxis", m_dexelWAxis);
        m_gl43->glUniform2i(
            m_dexelRenderProgram.uniformLocation("uGridSize"),
            m_dexelNx, m_dexelNy);
        m_gl43->glUniform2i(
            m_dexelRenderProgram.uniformLocation("uRenderGridSize"),
            m_dexelRenderNx, m_dexelRenderNy);
        m_dexelRenderProgram.setUniformValue(
            "uRenderStride", m_dexelRenderStride);
        m_dexelRenderProgram.setUniformValue(
            "uMaximumIntervals", m_dexelMaximumIntervals);
        m_dexelRenderProgram.setUniformValue(
            "uResolution", m_dexelResolution);
        m_dexelRenderProgram.setUniformValue(
            "uGridMinimum", m_dexelGridMinimum);
        m_dexelRenderProgram.setUniformValue(
            "uGridMaximum", m_dexelGridMaximum);
        m_dexelRenderProgram.setUniformValue(
            "uColor", QVector4D(
                102.0f / 255.0f,
                208.0f / 255.0f,
                173.0f / 255.0f, 1.0f));
        m_dexelRenderUniformsDirty = false;
    }
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 0u,
        m_dexelIntervalsBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 1u,
        m_dexelCountsBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 2u,
        m_dexelCoarseMaskBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 3u,
        m_dexelFineEntriesBuffer);
    m_gl43->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER, 5u,
        m_dexelBoundaryDistanceBuffer);
    m_gl43->glBindVertexArray(m_dexelVao);
    m_gl43->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7u, m_dexelCoarseEntriesBuffer);
    if (!checkOpenGlStage(
            QStringLiteral("GPU Multi-Dexel 绘制参数上传"))) {
        m_gl43->glBindVertexArray(0u);
        glEnable(GL_CULL_FACE);
        return;
    }
    m_dexelRenderProgram.setUniformValue(
        "uFinePass", false);
    m_gl43->glBindBuffer(
        GL_DRAW_INDIRECT_BUFFER,
        m_dexelFineDrawCommandBuffer);
    m_gl43->glDrawArraysIndirect(
        GL_TRIANGLES, reinterpret_cast<const void*>(6 * sizeof(std::uint32_t)));
    m_dexelRenderProgram.setUniformValue(
        "uFinePass", true);
    m_gl43->glDrawArraysIndirect(
        GL_TRIANGLES, nullptr);
    m_gl43->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0u);
    checkOpenGlStage(
        QStringLiteral("GPU Multi-Dexel 实例绘制"));
    m_gl43->glBindVertexArray(0u);
    glEnable(GL_CULL_FACE);
}

} // namespace oap::app::pocket_cam_detail
