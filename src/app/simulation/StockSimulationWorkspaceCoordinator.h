// 毛坯/IPW 状态、异步显示派生与 OCCT 呈现的应用边界。
// 职责:管理孔/槽毛坯仿真会话(毛坯定义、结果、指纹)、统一 Multi-Dexel
//   IPW 的异步显示网格派生,以及 OCCT 视口中的毛坯/IPW 呈现。
// 模块归属:src/app(simulation 聚焦工作区协调器),由 MainWindow 持有。
// 协作:StockSimulationSession 保存仿真结果;StockSimulationCoordinator
//   及其 IpwDisplayMeshController 在后台线程派生显示网格(带缓存);
//   StockSimulationPresenter 负责把毛坯/IPW 呈现到 OCCT 视口;
//   仿真执行委托给 Hole/PocketStockSimulationUseCase。
#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

#include <QObject>
#include <QString>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Triangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_View.hxx>
#include <gp_Trsf.hxx>

#include "app/workflows/simulation/StockSimulationCoordinator.h"
#include "app/workflows/simulation/StockSimulationSession.h"
#include "app/workflows/simulation/StockSimulationUseCases.h"
#include "app/workflows/simulation/StockSimulationViewModels.h"

namespace oap::manufacturing {
class MachiningJob;
}

namespace oap::app::presentation {
class OverlayController;
class StockSimulationPresenter;
}

namespace oap::app {

// 毛坯/IPW 工作区协调器:长期持有仿真会话、任务协调器与呈现器。
// 状态所有权:会话保存孔/槽毛坯定义、仿真结果与源指纹(用于判断结果
//   是否过期);任务协调器管理显示网格派生线程与缓存;呈现器管理
//   OCCT 对象。显示请求一律异步提交,完成后经 handleDisplayResult 发布。
class StockSimulationWorkspaceCoordinator final : public QObject {
    Q_OBJECT
public:
    StockSimulationWorkspaceCoordinator(
        presentation::OverlayController& overlays,
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view,
        QObject* parent = nullptr);
    StockSimulationWorkspaceCoordinator(
        presentation::OverlayController& overlays,
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view,
        StockSimulationSession initialSession,
        QObject* parent = nullptr);
    ~StockSimulationWorkspaceCoordinator() override;

    StockSimulationWorkspaceCoordinator(
        const StockSimulationWorkspaceCoordinator&) = delete;
    StockSimulationWorkspaceCoordinator& operator=(
        const StockSimulationWorkspaceCoordinator&) = delete;

    // 绑定/切换 OCCT 视图(呈现器与保存的视图同步更新)。
    void bindViewer(
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view);
    // 孔毛坯源设置/定义的读取与恢复(工程加载后还原)。
    StockSourceSettings holeSourceSettings() const;
    manufacturing::StockDefinition holeStockDefinition() const;
    void restoreHoleStockDefinition(
        const manufacturing::StockDefinition& definition);
    // 从已加载 Job 的不可变检查点链恢复统一 IPW 桌面会话；不重新执行刀路。
    // 链不完整或派生最终快照失败时清除旧会话并返回 false。
    bool installUnifiedIpwRuntime(
        const manufacturing::MachiningJob& job);
    // 仿真存在性/源指纹查询(指纹用于判断结果是否随源变化过期)。
    bool hasHoleSimulation() const noexcept
    {
        return m_session.hasHoleSimulation();
    }
    const QString& holeSourceFingerprint() const noexcept
    {
        return m_session.holeSourceFingerprint();
    }
    bool hasPocketSimulation() const noexcept
    {
        return m_session.hasPocketSimulation();
    }
    bool pocketUsesUnifiedIpw() const noexcept
    {
        return m_session.pocketUsesUnifiedIpw();
    }
    const QString& pocketSourceFingerprint() const noexcept
    {
        return m_session.pocketSourceFingerprint();
    }
    // 返回最近一次完整统一 IPW 的 B-Rep 派生快照。曲面精铣只接受
    // complete 且 finalStock 非空的结果，避免把显示网格当作材料真值。
    const TopoDS_Shape* latestCompleteIpwStock() const noexcept;
    // 对话框/摘要构建:给 UI 提供孔/槽仿真对话框所需的模型与文本。
    HoleStockDialogModel holeDialogModel(
        std::span<const HolePreviewEntry> previewEntries) const;
    QString holeSummary(
        std::span<const HolePreviewEntry> previewEntries,
        const HoleStockDisplaySelection& selection) const;
    // 从已完成的权威孔 IPW 快照中截取一个连续固定轴 Setup，构造 OpenGL
    // 后台准备请求；多轴程序必须由 UI 分 Setup 调用。
    bool buildHoleOpenGlRequest(
        const std::vector<toolpath::HoleMachiningOperation>& operations,
        const toolpath::ToolpathParams& fallbackParams,
        int setupIndex,
        const TopoDS_Shape& targetShape,
        HoleOpenGlSimulationRequest& request,
        QString& error) const;
    PocketStockSimulationDialogState pocketDialogState(
        std::span<const toolpath::PocketToolpathOperation> operations,
        const QString& browseDirectory) const;
    PocketStockSimulationDialogState profileDialogState(
        std::span<const toolpath::ProfileToolpathOperation> operations,
        const QString& browseDirectory) const;
    // 执行孔/槽毛坯仿真:把会话/任务协调器/任务注入用例执行,
    // 完成后发出 featureAllowancesChanged。
    HoleStockRunOutcome executeHoleSimulation(
        HoleStockSimulationUseCaseRequest request,
        manufacturing::MachiningJob& job);
    PocketStockSimulationUseCaseResult executePocketSimulation(
        PocketStockSimulationUseCaseRequest request,
        manufacturing::MachiningJob& job);
    ProjectStockSimulationUseCaseResult executeProjectSimulation(
        ProjectStockSimulationUseCaseRequest request,
        manufacturing::MachiningJob& job);
    ProfileStockSimulationUseCaseResult executeProfileSimulation(
        ProfileStockSimulationUseCaseRequest request,
        manufacturing::MachiningJob& job,
        const std::vector<toolpath::ProfileToolpathOperation>& operations,
        const toolpath::ToolpathParams& effectiveParams);

    // 任务状态:busy 是否有任务在跑;simulationRunning 仿真是否运行中;
    // requestCancel 异步取消;cancelAndWait 同步取消显示网格并等待;
    // resetForModel 模型切换时清空仿真/显示/缓存。
    bool busy() const noexcept { return m_taskCoordinator.busy(); }
    bool simulationRunning() const noexcept
    {
        return m_taskCoordinator.simulationRunning();
    }
    void requestCancel() noexcept;
    void cancelAndWait();
    void resetForModel();

    // 显示网格缓存预算与质量设置(网格细化档位、三角化开关、
    // OCCT 显示形式)。
    void setDisplayCacheBudgetBytes(std::size_t bytes);
    std::size_t displayCacheBudgetBytes() const;
    int displayQuality() const noexcept;
    void setDisplayQuality(int quality) noexcept;
    bool triangulationEnabled() const noexcept;
    void setTriangulationEnabled(bool enabled) noexcept;
    IpwOcctDisplayMode displayMode() const noexcept;
    void setDisplayMode(IpwOcctDisplayMode mode) noexcept;
    // OCCT 呈现是否有效(非 Off,且表面形式要求三角化时已开启三角化)。
    bool occtDisplayEnabled() const noexcept;

    // 孔仿真显示:清除、快照、对比视图(缺料/余量/过切)与过切归属。
    void clearHole(bool clearResult = true);
    void clearPocket(bool clearResult = true);
    void displayHoleSnapshot(int stepIndex, const gp_Trsf& modelTransform);
    void displayHoleComparison(int viewMode, const gp_Trsf& modelTransform);
    void displayHoleOperationOvercut(
        int attributionIndex, const gp_Trsf& modelTransform);
    // 特征余量列表与单项余量显示。
    std::vector<FeatureAllowanceViewItem>
    featureAllowanceItems() const;
    void displayFeatureAllowance(
        const FeatureAllowanceViewItem& item,
        const gp_Trsf& modelTransform);

    // 槽仿真显示:快照、操作步进/瞬时帧、刷新、时间线构建与结束帧。
    void displayPocketSnapshot(
        const manufacturing::MachiningJob& job,
        int stepIndex,
        const gp_Trsf& modelTransform);
    int pocketOperationMoveCount(
        const manufacturing::MachiningJob& job,
        int operationIndex) const;
    void displayPocketTransient(
        const manufacturing::MachiningJob& job,
        int operationIndex,
        int completedMoveCount,
        const gp_Trsf& modelTransform);
    void refreshPocketDisplay(
        const manufacturing::MachiningJob& job,
        const gp_Trsf& modelTransform);
    std::vector<IpwTimelineOperation> timelineOperations(
        const manufacturing::MachiningJob& job) const;
    void finishTimeline(
        const manufacturing::MachiningJob& job,
        const gp_Trsf& modelTransform);

    // 当前孔/槽呈现对象访问(供外部复用或查询)。
    Handle(AIS_Shape) holePresentation() const;
    Handle(AIS_Triangulation) pocketPresentation() const;

signals:
    // 状态文本(供状态栏展示)。
    void statusChanged(const QString& status);
    // 警告弹窗请求(标题/消息)。
    void warningRequested(const QString& title, const QString& message);
    // 异步显示网格派生完成(成功/取消/失败均发出)。
    void asynchronousDisplayFinished();
    // 特征余量数据发生变化(仿真执行/清除/重置后)。
    void featureAllowancesChanged();

private:
    // 以三角化网格/直接图元/BRep 形状发布槽呈现(失败时发状态消息)。
    bool presentPocketTriangulation(
        const Handle(Poly_Triangulation)& mesh,
        const Quantity_Color& color,
        double transparency,
        bool topLayer,
        IpwOcctDisplayMode displayMode,
        const gp_Trsf& modelTransform);
    bool presentPocketPrimitiveGeometry(
        const IpwDisplayPrimitiveGeometryPtr& geometry,
        const Quantity_Color& color,
        double transparency,
        bool topLayer,
        const gp_Trsf& modelTransform);
    bool presentPocketBRep(
        const TopoDS_Shape& shape,
        const Quantity_Color& color,
        double transparency,
        bool topLayer,
        const gp_Trsf& modelTransform);
    // 移除当前槽呈现对象。
    void removePocketPresentation();
    // 提交异步显示网格请求(记录模型变换、校验后台线程、失败回退)。
    void requestDisplay(
        IpwDisplayMeshRequest request,
        const gp_Trsf& modelTransform);
    // 消费后台显示网格结果:按处置(丢弃/取消/失败/发布)处理并呈现。
    void handleDisplayResult(const IpwDisplayMeshResult& result);

    StockSimulationSession m_session;         // 毛坯/IPW 仿真结果与指纹。
    StockSimulationCoordinator m_taskCoordinator;   // 任务与显示网格派生。
    std::unique_ptr<presentation::StockSimulationPresenter> m_presenter;  // OCCT 呈现器。
    Handle(V3d_View) m_view;                   // 目标视图。
    gp_Trsf m_requestedModelTransform;         // 最近一次显示请求的模型变换。
};

} // namespace oap::app
