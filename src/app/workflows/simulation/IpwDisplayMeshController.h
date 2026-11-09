// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 职责：托管 IPW 显示网格工作线程；每个请求携带自增 generation/requestId，
//       结果回到 UI 线程后不匹配即丢弃。由 StockSimulationCoordinator 创建并持有。
// 生命期：本对象拥有 QThread 与 worker，析构时 cancelAndWait。
// 拥有 IPW 显示网格工作线程，并拒绝过期的发布结果。
#pragma once

#include "app/workflows/simulation/IpwDisplayMeshWorker.h"

#include <memory>

#include <QObject>

class QThread;

namespace oap::app {

// IPW 显示网格控制器：协调网格派生任务与缓存。
class IpwDisplayMeshController final : public QObject {
    Q_OBJECT
public:
    explicit IpwDisplayMeshController(QObject* parent = nullptr);
    ~IpwDisplayMeshController() override;

    // Shared display policy for viewport meshes and playback fallbacks.
    // This only bounds derived geometry; it does not resample material state.
    static toolpath::DexelDisplayMeshOptions meshOptionsForQuality(
        int level, double gridResolution);

    IpwDisplayMeshController(const IpwDisplayMeshController&) = delete;
    IpwDisplayMeshController& operator=(
        const IpwDisplayMeshController&) = delete;

    // 提交单个显示请求：先取消在途请求，再以新 generation/requestId 投递。
    // 返回 false 表示线程不可用（请求未投递）。
    bool submit(IpwDisplayMeshRequest request);
    // 取消当前在途请求；invalidateGeneration 为 true 时同时提升代次，
    // 使旧请求结果永不被发布。
    void cancel(bool invalidateGeneration = false);
    // 请求取消而不使活动身份失效，这样所有者可以观察到被取消的结果，
    // 并完成关闭协调。
    void requestCancelCurrent() noexcept;
    void cancelAndWait();

    bool running() const noexcept;
    bool inFlight() const noexcept { return m_inFlight; }

signals:
    void resultReady(const oap::app::IpwDisplayMeshResult& result);

private:
    void handleResult(const IpwDisplayMeshResult& result);

    QThread* m_thread = nullptr;
    IpwDisplayMeshWorker* m_worker = nullptr;
    std::shared_ptr<std::atomic_bool> m_cancellation;
    quint64 m_generation = 0u;
    quint64 m_requestId = 0u;
    bool m_inFlight = false;
};

} // namespace oap::app
