#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <utility>
#include <QOpenGLContext>

namespace oap::app::pocket_cam_detail {

QString PocketCamOpenGLWidget::materialStatusString() const
{
    if (m_scene.authoritativeIpwPlayback) {
        QString status = QStringLiteral("IPW 动画｜工序 %1｜%2｜刀具与装夹方向随工序切换")
            .arg(m_currentOperationIndex + 1)
            .arg(!m_error.isEmpty() ? QStringLiteral("材料帧显示失败") :
                 m_waitingForOperationBaseline ? QStringLiteral("正在更新材料帧") :
                 m_gpuDexelReady || m_usePlaybackMesh ? QStringLiteral("材料帧已就绪") : QStringLiteral("材料帧未就绪"));
        if (m_usePlaybackMesh)
            status += QStringLiteral("\n材料采用兼容网格显示（IPW 计算精度不变）：") + m_playbackDisplayWarning;
        return status;
    }
    if (!m_gpuDexelReady) {
        QString status = QStringLiteral("GPU 材料状态：未就绪");
        if (!m_preexistingOpenGlWarning.isEmpty())
            status += QStringLiteral("\nOpenGL 提示：") +
                m_preexistingOpenGlWarning;
        return status;
    }
    QString status = QStringLiteral(
        "GPU 材料状态：%1 × %2 网格，%3 个占用单元，%4 个材料区间，"
        "分辨率 %5 mm，TSDF 亚单元轮廓，显示步长 %6")
        .arg(m_dexelNx)
        .arg(m_dexelNy)
        .arg(static_cast<qulonglong>(
            m_gpuDexelOccupiedCellCount))
        .arg(static_cast<qulonglong>(
            m_gpuDexelSourceIntervalCount))
        .arg(m_dexelResolution, 0, 'f', 4)
        .arg(m_dexelRenderStride);
    if (m_scene.analyticLinearMotionCount > 0u ||
        m_scene.analyticArcMotionCount > 0u) {
        status += QStringLiteral(
            "\n解析边界缓存：G1 %1 段，G2/G3 %2 段"
            "（可信播放 %3，折线恢复 %4），平面回退 %5 段，"
            "工序检查点 %6/3")
            .arg(static_cast<qulonglong>(
                m_scene.analyticLinearMotionCount))
            .arg(static_cast<qulonglong>(
                m_scene.analyticArcMotionCount))
            .arg(static_cast<qulonglong>(
                m_scene.trustedAnalyticArcMotionCount))
            .arg(static_cast<qulonglong>(
                m_scene.recoveredAnalyticArcMotionCount))
            .arg(static_cast<qulonglong>(
                m_scene.analyticFallbackMotionCount))
            .arg(static_cast<qulonglong>(
                m_analyticBoundarySnapshots.size()));
    }
    if (!m_preexistingOpenGlWarning.isEmpty())
        status += QStringLiteral("\nOpenGL 提示：") +
            m_preexistingOpenGlWarning;
    return status;
}

void PocketCamOpenGLWidget::setCurrentStep(std::size_t step)
{
    if (m_scene.authoritativeIpwPlayback) {
        setAuthoritativeStep(step);
        return;
    }
    step = std::min(step, totalSteps());
    if (step == m_currentStep)
        return;
    const bool movingBackward = step < m_currentStep;
    if (movingBackward) {
        // GPU 材料减除不可逆；回退时恢复目标工序的 IPW 基线。
        m_gpuDexelReady = false;
    }
    m_currentStep = step;
    rebuildVisibleInstances();
    if (movingBackward) {
        if (m_currentStep == 0u) {
            if (m_baselineController)
                m_baselineController->cancel(false);
            m_waitingForOperationBaseline = false;
            m_requestedBaselineOperationIndex = -1;
        } else {
            requestOperationBaseline(m_currentOperationIndex);
        }
    } else if (!m_gpuDexelReady) {
        requestOperationBaseline(m_currentOperationIndex);
    }
    update();
}

void PocketCamOpenGLWidget::syncGpuDexelToCurrentStep()
{
    if (!m_gpuDexelReady ||
        m_currentStep < m_gpuDexelAppliedStep) {
        return;
    }
    if (m_currentStep == m_gpuDexelAppliedStep)
        return;
    const auto firstMotion = std::upper_bound(
        m_motionEndSteps.begin(),
        m_motionEndSteps.end(),
        m_gpuDexelAppliedStep);
    std::size_t motionIndex = static_cast<std::size_t>(
        std::distance(m_motionEndSteps.begin(), firstMotion));
    for (; motionIndex < m_scene.motions.size(); ++motionIndex) {
        const toolpath::PocketCamMotion& motion =
            m_scene.motions[motionIndex];
        const std::size_t motionStart =
            motion.firstSimulationStep;
        const std::size_t motionEnd =
            motionStart + static_cast<std::size_t>(
                motion.simulationStepCount);
        if (motionStart >= m_currentStep)
            break;
        const std::size_t segmentStart =
            std::max(m_gpuDexelAppliedStep, motionStart);
        const std::size_t segmentEnd =
            std::min(m_currentStep, motionEnd);
        if (segmentEnd <= segmentStart)
            continue;
        const double fromFraction =
            static_cast<double>(segmentStart - motionStart) /
            static_cast<double>(motion.simulationStepCount);
        const double toFraction =
            static_cast<double>(segmentEnd - motionStart) /
            static_cast<double>(motion.simulationStepCount);
        if (!dispatchGpuDexelMotion(
                motion,
                fromFraction,
                toFraction)) {
            return;
        }
        if (segmentEnd == motionEnd &&
            motionIndex + 1u < m_scene.motions.size()) {
            const int nextOperation =
                m_scene.motions[motionIndex + 1u]
                    .operationIndex;
            if (nextOperation >= 0 &&
                nextOperation != motion.operationIndex) {
                cacheAnalyticBoundarySnapshot(nextOperation);
            }
        }
    }
    m_gpuDexelAppliedStep = m_currentStep;
}

std::size_t PocketCamOpenGLWidget::operationFirstStep(int operationIndex) const
{
    if (operationIndex < 0 ||
        static_cast<std::size_t>(operationIndex) >=
            m_operationFirstMotionIndices.size()) {
        return 0u;
    }
    const std::size_t firstMotionIndex =
        m_operationFirstMotionIndices[
            static_cast<std::size_t>(operationIndex)];
    return firstMotionIndex < m_scene.motions.size()
        ? m_scene.motions[firstMotionIndex].firstSimulationStep
        : 0u;
}

bool PocketCamOpenGLWidget::requestOperationBaseline(int operationIndex)
{
    if (!m_baselineController || operationIndex < 0 ||
        static_cast<std::size_t>(operationIndex) >=
            m_operationInitialIpwSources.size()) {
        return false;
    }
    const auto& source = m_operationInitialIpwSources[
        static_cast<std::size_t>(operationIndex)];
    if (!source)
        return false;
    if (m_baselineController->inFlight() &&
        m_requestedBaselineOperationIndex == operationIndex) {
        m_waitingForOperationBaseline = true;
        return true;
    }

    IpwDisplayMeshRequest request;
    request.key.contentFingerprint =
        source->parentStock
        ? source->parentStock->contentFingerprint()
        : std::string{};
    request.key.variant =
        "pocket-cam-gpu-baseline:" +
        std::to_string(operationIndex) + ':' +
        std::to_string(source->completedMoveCount);
    request.key.kind =
        IpwDisplayArtifactKind::StockCheckpoint;
    request.transientSource = source;
    // 只派生 Multi-Dexel 材料，不构建额外的 OCCT 显示网格。
    request.displayMode = IpwOcctDisplayMode::Off;
    m_requestedBaselineOperationIndex = operationIndex;
    m_waitingForOperationBaseline = true;
    if (m_baselineController->submit(std::move(request)))
        return true;

    m_requestedBaselineOperationIndex = -1;
    m_waitingForOperationBaseline = false;
    return false;
}

void PocketCamOpenGLWidget::acceptOperationBaseline(
    const IpwDisplayMeshResult& result)
{
    const int operationIndex =
        m_requestedBaselineOperationIndex;
    m_requestedBaselineOperationIndex = -1;
    if (result.cancelled) {
        m_waitingForOperationBaseline = false;
        return;
    }
    if (operationIndex < 0 ||
        operationIndex != m_currentOperationIndex) {
        m_waitingForOperationBaseline = false;
        if (!m_gpuDexelReady)
            requestOperationBaseline(m_currentOperationIndex);
        update();
        return;
    }
    if (!result.ok() || !result.materialStock ||
        !result.materialStock->valid()) {
        // 检查点恢复失败时仍可从已有初始 IPW 重放；
        // 这条回退路径不会重新栅格化毛坯。
        m_waitingForOperationBaseline = false;
        update();
        return;
    }
    if (!context() || !context()->isValid()) {
        m_waitingForOperationBaseline = false;
        return;
    }

    makeCurrent();
    const bool uploaded =
        QOpenGLContext::currentContext() == context() &&
        uploadGpuDexelStock(
            *result.materialStock,
            operationFirstStep(operationIndex));
    if (uploaded) {
        restoreAnalyticBoundarySnapshot(operationIndex);
    }
    doneCurrent();
    m_waitingForOperationBaseline = false;
    if (!uploaded && m_error.isEmpty()) {
        m_error = QStringLiteral(
            "工序级 IPW 检查点无法上传到 GPU Multi-Dexel。");
    }
    update();
}

void PocketCamOpenGLWidget::prepareInstanceTimeline()
{
    m_motionEndSteps.clear();
    m_motionEndSteps.reserve(m_scene.motions.size());
    std::size_t operationCount =
        m_operationInitialIpwSources.size();
    for (const toolpath::PocketCamMotion& motion :
         m_scene.motions) {
        if (motion.operationIndex >= 0) {
            operationCount = std::max(
                operationCount,
                static_cast<std::size_t>(
                    motion.operationIndex + 1));
        }
    }
    m_operationFirstMotionIndices.assign(
        operationCount, m_scene.motions.size());
    for (std::size_t motionIndex = 0u;
         motionIndex < m_scene.motions.size(); ++motionIndex) {
        const toolpath::PocketCamMotion& motion =
            m_scene.motions[motionIndex];
        if (motion.operationIndex >= 0 &&
            static_cast<std::size_t>(motion.operationIndex) <
                m_operationFirstMotionIndices.size()) {
            std::size_t& first =
                m_operationFirstMotionIndices[
                    static_cast<std::size_t>(
                        motion.operationIndex)];
            first = std::min(first, motionIndex);
        }
        m_motionEndSteps.push_back(
            motion.firstSimulationStep +
            static_cast<std::size_t>(
                motion.simulationStepCount));
    }
}

void PocketCamOpenGLWidget::rebuildVisibleInstances()
{
    const auto completedEnd = std::upper_bound(
        m_motionEndSteps.begin(),
        m_motionEndSteps.end(),
        m_currentStep);
    const std::size_t completedMotionCount =
        static_cast<std::size_t>(
            std::distance(
                m_motionEndSteps.begin(),
                completedEnd));
    if (m_scene.motions.empty()) {
        m_currentToolIndex = -1;
        m_currentOperationIndex = -1;
    } else if (completedMotionCount >=
               m_scene.motions.size()) {
        m_currentToolIndex =
            m_scene.motions.back().toolIndex;
        m_currentOperationIndex =
            m_scene.motions.back().operationIndex;
    } else {
        m_currentToolIndex =
            m_scene.motions[completedMotionCount]
                .toolIndex;
        m_currentOperationIndex =
            m_scene.motions[completedMotionCount]
                .operationIndex;
    }

    m_currentTip = m_scene.motions.empty()
        ? gp_Pnt(0.0, 0.0, 0.0)
        : m_scene.motions.front().move.from;
    if (completedMotionCount >=
        m_scene.motions.size()) {
        if (!m_scene.motions.empty())
            m_currentTip =
                m_scene.motions.back().move.to;
    } else {
        const toolpath::PocketCamMotion& motion =
            m_scene.motions[completedMotionCount];
        if (m_currentStep > motion.firstSimulationStep) {
            const double fraction =
                static_cast<double>(
                    m_currentStep -
                    motion.firstSimulationStep) /
                static_cast<double>(
                    motion.simulationStepCount);
            m_currentTip = motion.playbackPointAt(fraction);
        } else {
            m_currentTip = motion.move.from;
        }
    }
    m_currentToolTransform =
        toolTransform(m_scene.authoritativeIpwPlayback && !m_scene.motions.empty()
            ? makeToolFrame(m_scene.motions[std::min(completedMotionCount, m_scene.motions.size() - 1u)].outwardToolAxis)
            : m_frame, m_currentTip);
}

void PocketCamOpenGLWidget::setAuthoritativeStep(std::size_t step)
{
    if (!m_gpuReady || !m_baselineController || m_scene.motions.empty()) return;
    step = std::min(step, totalSteps());
    if (step == m_currentStep) {
        m_baselineController->cancel(false);
        m_waitingForOperationBaseline = false;
        return;
    }
    IpwDisplayMeshRequest request;
    request.displayMode = IpwOcctDisplayMode::Off;
    m_requestedMaterialOperation = -1;
    m_requestedMaterialMoveCount = 0;
    if (step == 0) {
        request.stockSource = m_initialGpuStock;
    } else if (step == totalSteps()) {
        request.stockSource = m_finalGpuStock;
    } else {
        // Stop on an original-move prefix. Material and cutter position are
        // committed together; simplified display chords never drive removal.
        const auto end = std::lower_bound(m_motionEndSteps.begin(), m_motionEndSteps.end(), step);
        const auto index = static_cast<std::size_t>(std::distance(m_motionEndSteps.begin(), end));
        const auto& motion = m_scene.motions[index];
        step = *end;
        const auto operation = static_cast<std::size_t>(motion.operationIndex);
        if (operation >= m_operationInitialIpwSources.size() || !m_operationInitialIpwSources[operation]) {
            m_error = QStringLiteral("项目动画缺少工序起始 IPW，已停止播放。");
            return;
        }
        auto source = std::make_shared<IpwTransientFrameSource>(*m_operationInitialIpwSources[operation]);
        source->completedMoveCount += motion.sourceMoveEndCount;
        if (m_materialStock && m_materialOperation == motion.operationIndex &&
            source->completedMoveCount >= m_materialMoveCount) {
            source->parentStock = m_materialStock;
            source->firstMoveCount = m_materialMoveCount;
        }
        m_requestedMaterialMoveCount = source->completedMoveCount;
        m_requestedMaterialOperation = motion.operationIndex;
        request.transientSource = std::move(source);
    }
    m_requestedPlaybackStep = step;
    if (m_usePlaybackMesh) {
        request.displayMode = IpwOcctDisplayMode::SurfaceShaded;
        const auto& frameStock = request.stockSource
            ? request.stockSource : request.transientSource
                ? request.transientSource->parentStock : m_initialGpuStock;
        request.meshOptions = IpwDisplayMeshController::meshOptionsForQuality(
            1, frameStock ? frameStock->resolution() : m_initialGpuStock->resolution());
    }
    m_waitingForOperationBaseline = true;
    if (!m_baselineController->submit(std::move(request))) {
        m_waitingForOperationBaseline = false;
        m_error = QStringLiteral("无法启动项目 IPW 动画帧更新。");
    }
}

void PocketCamOpenGLWidget::acceptAuthoritativeFrame(const IpwDisplayMeshResult& result)
{
    m_waitingForOperationBaseline = false;
    if (result.cancelled) return;
    // A successfully computed empty checkpoint has no surface mesh to publish.
    const bool emptyMaterial = result.materialStock && result.materialStock->valid() &&
        result.materialStock->intervalCount() == 0;
    if ((!result.ok() && !emptyMaterial) || !result.materialStock || !result.materialStock->valid()) {
        m_error = QStringLiteral("项目 IPW 动画帧更新失败：%1").arg(QString::fromStdString(result.error));
        update();
        return;
    }
    makeCurrent();
    const bool uploaded = QOpenGLContext::currentContext() == context() &&
        uploadPlaybackStock(*result.materialStock, m_requestedPlaybackStep, result.mesh);
    doneCurrent();
    if (!uploaded) {
        m_error = QStringLiteral("项目 IPW 材料帧无法上传到 OpenGL：%1")
            .arg(m_error.isEmpty() ? QStringLiteral("无有效上下文或材料网格超过上传预算") : m_error);
        update();
        return;
    }
    m_materialStock = result.materialStock;
    m_materialOperation = m_requestedMaterialOperation;
    m_materialMoveCount = m_requestedMaterialMoveCount;
    m_currentStep = m_requestedPlaybackStep;
    rebuildVisibleInstances();
    update();
}

} // namespace oap::app::pocket_cam_detail
