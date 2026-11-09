// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 职责：各仿真入口共享的毛坯/IPW 任务生命周期状态：仿真取消句柄、
//       IPW 显示网格管道（controller + worker）与显示产物 LRU 缓存。
// 线程：由 UI 线程持有；worker 结果回到 UI 线程后经 evaluateDisplayResult 发布。
// 拥有各仿真入口共享的毛坯/IPW 任务生命周期状态。
#pragma once

#include <atomic>
#include <cstddef>
#include <memory>

#include <QString>

#include "app/workflows/simulation/detail/IpwDisplayArtifactCache.h"
#include "app/workflows/simulation/IpwDisplayMeshWorker.h"

namespace oap::app {

class IpwDisplayMeshController;

// 显示发布处置：Discard=模式不匹配/未启用三角化，直接丢弃。
enum class IpwDisplayPublicationDisposition {
    Discard,
    Cancelled,
    Failed,
    Ready,
};

// 显示样式：RGB 颜色、透明度与是否置于顶层。
struct IpwDisplayStyle {
    double red = 0.20;
    double green = 0.76;
    double blue = 0.46;
    double transparency = 0.20;
    bool topLayer = false;
};

// 一次显示发布的决定：处置、样式、状态/错误文本与缓存命中标记。
struct IpwDisplayPublication {
    IpwDisplayPublicationDisposition disposition =
        IpwDisplayPublicationDisposition::Discard;
    IpwDisplayStyle style;
    QString status;
    QString error;
    bool cached = false;
};

// 毛坯/IPW 仿真入口的共享协调器。
class StockSimulationCoordinator {
public:
    StockSimulationCoordinator();
    ~StockSimulationCoordinator();

    StockSimulationCoordinator(const StockSimulationCoordinator&) = delete;
    StockSimulationCoordinator& operator=(
        const StockSimulationCoordinator&) = delete;

    // 开始一次仿真：旧任务先置为已取消，返回本次任务的可取消标志。
    std::shared_ptr<std::atomic_bool> beginSimulation();
    // 结束仿真并释放取消句柄。
    void finishSimulation() noexcept;
    // 请求取消当前仿真与在途显示网格生成。
    void requestCancel() noexcept;

    // 创建（或复用）IPW 显示网格控制器与后台线程。
    void initializeDisplayPipeline();
    IpwDisplayMeshController* displayMeshController() noexcept;
    // 提交显示网格请求；仅当控制器运行中才接收。
    bool submitDisplay(IpwDisplayMeshRequest request);
    // 取消在途显示请求；invalidateGeneration 为 true 时提升代次，使旧请求结果永不被发布。
    void cancelDisplay(bool invalidateGeneration = false);
    // 取消并等待显示线程退出。
    void cancelDisplayAndWait();

    // 显示产物缓存的清空、预算与统计。
    void clearDisplayCache();
    void setDisplayCacheBudgetBytes(std::size_t bytes);
    std::size_t displayCacheBudgetBytes() const;
    IpwDisplayCacheStats displayCacheStats() const;
    Handle(Poly_Triangulation) findDisplayMesh(
        const IpwDisplayArtifactKey& key);
    IpwDisplayArtifact findDisplayArtifact(
        const IpwDisplayArtifactKey& key);

    // 发布门禁：按显示模式/取消/失败判定处置，就绪时写入缓存并
    // 生成样式与状态文本。
    IpwDisplayPublication evaluateDisplayResult(
        const IpwDisplayMeshResult& result,
        IpwOcctDisplayMode currentMode,
        bool triangulationEnabled);

    static QString displayModeText(IpwOcctDisplayMode mode);
    static QString displayRequestStatus(
        const IpwDisplayMeshRequest& request);

    bool simulationRunning() const noexcept { return m_simulationRunning; }
    bool ipwDisplayMeshInFlight() const noexcept
    {
        return m_ipwDisplayMeshInFlight;
    }
    bool busy() const noexcept
    {
        return m_simulationRunning || m_ipwDisplayMeshInFlight;
    }

private:
    bool m_simulationRunning = false;
    bool m_ipwDisplayMeshInFlight = false;
    std::shared_ptr<std::atomic_bool> m_cancel;
    std::unique_ptr<IpwDisplayMeshController> m_displayController;
    std::unique_ptr<IpwDisplayArtifactCache> m_displayCache;
};

} // namespace oap::app
