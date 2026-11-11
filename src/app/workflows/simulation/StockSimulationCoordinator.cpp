// StockSimulationCoordinator 实现：仿真生命周期、显示管道与发布门禁。
#include "app/workflows/simulation/StockSimulationCoordinator.h"
#include <QCoreApplication>

#include "app/workflows/simulation/IpwDisplayMeshController.h"
#include "toolpath/core/HeavyJobScheduler.h"

#include <utility>

namespace oap::app {

StockSimulationCoordinator::StockSimulationCoordinator()
    : m_displayCache(std::make_unique<IpwDisplayArtifactCache>())
{
}

// 析构时先取消并等待显示线程退出，保证 worker 不再回调本对象。
StockSimulationCoordinator::~StockSimulationCoordinator()
{
    cancelDisplayAndWait();
}

// 开始一次仿真：旧取消句柄置位，新句柄返回给调用方注入 runner。
std::shared_ptr<std::atomic_bool>
StockSimulationCoordinator::beginSimulation()
{
    if (m_cancel)
        toolpath::cancelHeavyJob(m_cancel);
    m_cancel = std::make_shared<std::atomic_bool>(false);
    m_simulationRunning = true;
    return m_cancel;
}

void StockSimulationCoordinator::finishSimulation() noexcept
{
    m_simulationRunning = false;
    m_cancel.reset();
}

void StockSimulationCoordinator::requestCancel() noexcept
{
    if (m_cancel)
        toolpath::cancelHeavyJob(m_cancel);
    if (m_displayController)
        m_displayController->requestCancelCurrent();
}

void StockSimulationCoordinator::initializeDisplayPipeline()
{
    if (!m_displayController) {
        m_displayController =
            std::make_unique<IpwDisplayMeshController>();
    }
}

IpwDisplayMeshController*
StockSimulationCoordinator::displayMeshController() noexcept
{
    return m_displayController.get();
}

// 提交显示网格请求；控制器未就绪或线程未运行时拒绝并复位在途标记。
bool StockSimulationCoordinator::submitDisplay(
    IpwDisplayMeshRequest request)
{
    if (!m_displayController || !m_displayController->running()) {
        m_ipwDisplayMeshInFlight = false;
        return false;
    }
    const bool queued =
        m_displayController->submit(std::move(request));
    m_ipwDisplayMeshInFlight = queued;
    return queued;
}

void StockSimulationCoordinator::cancelDisplay(
    bool invalidateGeneration)
{
    if (m_displayController)
        m_displayController->cancel(invalidateGeneration);
    m_ipwDisplayMeshInFlight = false;
}

void StockSimulationCoordinator::cancelDisplayAndWait()
{
    if (m_displayController)
        m_displayController->cancelAndWait();
    m_ipwDisplayMeshInFlight = false;
}

void StockSimulationCoordinator::clearDisplayCache()
{
    m_displayCache->clear();
}

void StockSimulationCoordinator::setDisplayCacheBudgetBytes(
    std::size_t bytes)
{
    m_displayCache->setBudgetBytes(bytes);
}

std::size_t StockSimulationCoordinator::displayCacheBudgetBytes() const
{
    return m_displayCache->budgetBytes();
}

IpwDisplayCacheStats StockSimulationCoordinator::displayCacheStats() const
{
    return m_displayCache->stats();
}

Handle(Poly_Triangulation)
StockSimulationCoordinator::findDisplayMesh(
    const IpwDisplayArtifactKey& key)
{
    return m_displayCache->find(key);
}

IpwDisplayArtifact StockSimulationCoordinator::findDisplayArtifact(
    const IpwDisplayArtifactKey& key)
{
    return m_displayCache->findArtifact(key);
}

IpwDisplayPublication
StockSimulationCoordinator::evaluateDisplayResult(
    const IpwDisplayMeshResult& result,
    IpwOcctDisplayMode currentMode,
    bool triangulationEnabled)
{
    m_ipwDisplayMeshInFlight = false;
    IpwDisplayPublication publication;
    // 发布门禁：显示关闭、结果模式与当前模式不符，或表面模式
    // 但未启用三角化时，直接丢弃本次结果。
    if (currentMode == IpwOcctDisplayMode::Off ||
        result.displayMode != currentMode ||
        (isIpwSurfaceDisplayMode(result.displayMode) &&
         !triangulationEnabled)) {
        return publication;
    }
    if (result.cancelled) {
        publication.disposition =
            IpwDisplayPublicationDisposition::Cancelled;
        publication.status =
            QCoreApplication::translate("OpenAutoPath", "IPW 显示网格生成已取消。");
        return publication;
    }
    if (!result.ok()) {
        publication.disposition =
            IpwDisplayPublicationDisposition::Failed;
        publication.error = result.error.empty()
            ? QCoreApplication::translate("OpenAutoPath", "IPW 派生显示网格生成失败。")
            : QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(result.error).toUtf8().constData());
        publication.status = publication.error;
        return publication;
    }

    publication.disposition =
        IpwDisplayPublicationDisposition::Ready;
    const bool transientFrame = result.transientFrame;
    // 仅表面显示模式且非临时帧的产物写入 LRU 缓存。
    publication.cached =
        isIpwSurfaceDisplayMode(result.displayMode) &&
        !transientFrame &&
        m_displayCache->insert(
            result.key, displayArtifactFromResult(result));
    switch (result.key.kind) {
        case IpwDisplayArtifactKind::StockCheckpoint:
            publication.style.topLayer = true;
            break;
        case IpwDisplayArtifactKind::InitialMissing:
            publication.style = {0.95, 0.15, 0.15, 0.10, true};
            break;
        case IpwDisplayArtifactKind::RemainingAllowance:
            publication.style = {0.95, 0.58, 0.10, 0.28, true};
            break;
        case IpwDisplayArtifactKind::FeatureAllowance:
            publication.style = {1.0, 0.72, 0.08, 0.12, true};
            break;
        case IpwDisplayArtifactKind::Overcut:
            publication.style = {0.92, 0.10, 0.72, 0.10, true};
            break;
        case IpwDisplayArtifactKind::OperationOvercut:
            publication.style = {1.0, 0.08, 0.36, 0.02, true};
            break;
    }

    const IpwDisplayCacheStats cacheStats = m_displayCache->stats();
    if (!isIpwSurfaceDisplayMode(result.displayMode)) {
        if (result.displayMode == IpwOcctDisplayMode::BRepSnapshot) {
            publication.status = QCoreApplication::translate("OpenAutoPath", "%1%2已发布为 BRep 实体；后台重建 %3 ms。")
                .arg(transientFrame
                    ? QCoreApplication::translate("OpenAutoPath", "IPW 临时帧：")
                    : QCoreApplication::translate("OpenAutoPath", "IPW 检查点："))
                .arg(displayModeText(result.displayMode))
                .arg(result.metrics.milliseconds, 0, 'f', 1);
        } else {
            const IpwDisplayPrimitiveGeometry& geometry =
                *result.primitiveGeometry;
            const QString sampling = geometry.samplingStride > 1
                ? QCoreApplication::translate("OpenAutoPath", "，按 ×%1 采样")
                      .arg(geometry.samplingStride)
                : QString();
            publication.status = QCoreApplication::translate("OpenAutoPath", "%1%2：显示 %3/%4 个图元%5；后台生成 %6 ms%7。")
                .arg(transientFrame
                    ? QCoreApplication::translate("OpenAutoPath", "IPW 临时帧：")
                    : QCoreApplication::translate("OpenAutoPath", "IPW 检查点："))
                .arg(displayModeText(result.displayMode))
                .arg(geometry.displayedPrimitiveCount)
                .arg(geometry.sourcePrimitiveCount)
                .arg(sampling)
                .arg(result.metrics.milliseconds, 0, 'f', 1)
                .arg(transientFrame
                    ? QCoreApplication::translate("OpenAutoPath", "，已去除 %1 mm³")
                          .arg(result.removedVolume, 0, 'f', 3)
                    : QString());
        }
        return publication;
    }
    if (transientFrame) {
        publication.status = QCoreApplication::translate("OpenAutoPath", "IPW 临时帧 %1/%2：去除 %3 mm³，"
            "命中 %4 个 tile、变化 %5 个单元；"
            "材料更新 %6 ms，网格 %7 ms。")
            .arg(result.completedMoveCount)
            .arg(result.totalMoveCount)
            .arg(result.removedVolume, 0, 'f', 3)
            .arg(result.touchedTileCount)
            .arg(result.changedCellCount)
            .arg(result.materialUpdateMilliseconds, 0, 'f', 1)
            .arg(result.metrics.milliseconds, 0, 'f', 1);
    } else if (result.key.kind ==
               IpwDisplayArtifactKind::StockCheckpoint) {
        publication.status = QCoreApplication::translate("OpenAutoPath", "IPW 检查点网格：%1 个三角形，%2，后台生成 %3 ms；"
            "缓存 %4/%5 MiB。")
            .arg(result.metrics.triangleCount)
            .arg(publication.cached ? QCoreApplication::translate("OpenAutoPath", "已缓存")
                                    : QCoreApplication::translate("OpenAutoPath", "超过预算，未缓存"))
            .arg(result.metrics.milliseconds, 0, 'f', 1)
            .arg(cacheStats.residentBytes / (1024.0 * 1024.0),
                 0, 'f', 1)
            .arg(cacheStats.budgetBytes / (1024.0 * 1024.0),
                 0, 'f', 0);
    } else if (result.key.kind ==
               IpwDisplayArtifactKind::FeatureAllowance) {
        publication.status = QCoreApplication::translate("OpenAutoPath", "特征 %1 加工后余量：%2 个三角形，余量体积 %3 mm³；后台生成 %4 ms。")
            .arg(QString::fromStdString(result.key.variant))
            .arg(result.metrics.triangleCount)
            .arg(result.metrics.intervalVolume, 0, 'f', 3)
            .arg(result.metrics.milliseconds, 0, 'f', 1);
    } else if (result.key.kind ==
               IpwDisplayArtifactKind::OperationOvercut) {
        publication.status = QCoreApplication::translate("OpenAutoPath", "工序 %1 过切网格：%2 个三角形，显示体积 %3 mm³，"
            "%4；后台生成 %5 ms。")
            .arg(QString::fromStdString(result.key.variant))
            .arg(result.metrics.triangleCount)
            .arg(result.metrics.intervalVolume, 0, 'f', 3)
            .arg(publication.cached ? QCoreApplication::translate("OpenAutoPath", "已缓存")
                                    : QCoreApplication::translate("OpenAutoPath", "超过预算，未缓存"))
            .arg(result.metrics.milliseconds, 0, 'f', 1);
    } else {
        publication.status = QCoreApplication::translate("OpenAutoPath", "Z-map 差异网格：%1 个三角形，显示体积 %2 mm³，"
            "轴向误差 ≤%3 mm，平面步长 ×%4（误差上界 ≤%5 mm），"
            "%6；后台生成 %7 ms。")
            .arg(result.metrics.triangleCount)
            .arg(result.metrics.intervalVolume, 0, 'f', 3)
            .arg(result.metrics.maximumAxialError, 0, 'f', 3)
            .arg(result.metrics.planarDecimationStride)
            .arg(result.metrics.planarErrorUpperBound, 0, 'f', 3)
            .arg(publication.cached ? QCoreApplication::translate("OpenAutoPath", "已缓存")
                                    : QCoreApplication::translate("OpenAutoPath", "超过预算，未缓存"))
            .arg(result.metrics.milliseconds, 0, 'f', 1);
    }
    return publication;
}

QString StockSimulationCoordinator::displayModeText(
    IpwOcctDisplayMode mode)
{
    switch (mode) {
        case IpwOcctDisplayMode::SurfaceWithEdges:
            return QCoreApplication::translate("OpenAutoPath", "带网格表面");
        case IpwOcctDisplayMode::SurfaceShaded:
            return QCoreApplication::translate("OpenAutoPath", "无边线着色表面");
        case IpwOcctDisplayMode::SurfaceWireframe:
            return QCoreApplication::translate("OpenAutoPath", "纯三角线框");
        case IpwOcctDisplayMode::DexelColumns:
            return QCoreApplication::translate("OpenAutoPath", "Dexel 材料柱");
        case IpwOcctDisplayMode::IntervalEndpoints:
            return QCoreApplication::translate("OpenAutoPath", "区间端点点云");
        case IpwOcctDisplayMode::IntervalPrisms:
            return QCoreApplication::translate("OpenAutoPath", "材料体素柱");
        case IpwOcctDisplayMode::MidSection:
            return QCoreApplication::translate("OpenAutoPath", "W 向中剖面");
        case IpwOcctDisplayMode::BRepSnapshot:
            return QCoreApplication::translate("OpenAutoPath", "BRep 实体");
        case IpwOcctDisplayMode::Off:
            return QCoreApplication::translate("OpenAutoPath", "关闭");
    }
    return QCoreApplication::translate("OpenAutoPath", "未知形式");
}

QString StockSimulationCoordinator::displayRequestStatus(
    const IpwDisplayMeshRequest& request)
{
    QString artifactName = QCoreApplication::translate("OpenAutoPath", "IPW 派生");
    switch (request.key.kind) {
        case IpwDisplayArtifactKind::StockCheckpoint:
            artifactName = QCoreApplication::translate("OpenAutoPath", "IPW 检查点");
            break;
        case IpwDisplayArtifactKind::InitialMissing:
            artifactName = QCoreApplication::translate("OpenAutoPath", "初始缺料");
            break;
        case IpwDisplayArtifactKind::RemainingAllowance:
            artifactName = QCoreApplication::translate("OpenAutoPath", "剩余余量");
            break;
        case IpwDisplayArtifactKind::FeatureAllowance:
            artifactName = QCoreApplication::translate("OpenAutoPath", "特征加工后余量");
            break;
        case IpwDisplayArtifactKind::Overcut:
            artifactName = QCoreApplication::translate("OpenAutoPath", "过切");
            break;
        case IpwDisplayArtifactKind::OperationOvercut:
            artifactName = QCoreApplication::translate("OpenAutoPath", "逐工序过切");
            break;
    }
    if (request.transientSource)
        artifactName = QCoreApplication::translate("OpenAutoPath", "IPW 逐移动临时帧");
    return QCoreApplication::translate("OpenAutoPath", "正在后台生成%1的%2；可继续操作，切换视图会取消本次请求…")
        .arg(artifactName)
        .arg(displayModeText(request.displayMode));
}

} // namespace oap::app
