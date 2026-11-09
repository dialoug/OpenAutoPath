// StockSimulationWorkspaceCoordinator 实现（oap_app_simulation）。
// 毛坯/IPW 状态、异步显示派生与 OCCT 呈现的应用边界，见 StockSimulationWorkspaceCoordinator.h。
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"

#include "app/presentation/StockSimulationPresenter.h"
#include "app/workflows/simulation/IpwDisplayMeshController.h"
#include "app/workflows/simulation/StockSimulationWorkflowController.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include <Graphic3d_ZLayerId.hxx>
#include <Quantity_Color.hxx>

namespace oap::app {
namespace {

// 各类毛坯/对比几何在视口中的呈现颜色:
// 孔毛坯青色、初始缺料红色、剩余余量橙色、过切紫、工序过切玫红、
// 槽毛坯绿色。
const Quantity_Color kHoleStockColor(
    0.18, 0.72, 0.92, Quantity_TOC_RGB);
const Quantity_Color kInitialMissingColor(
    0.95, 0.15, 0.15, Quantity_TOC_RGB);
const Quantity_Color kRemainingAllowanceColor(
    0.95, 0.58, 0.10, Quantity_TOC_RGB);
const Quantity_Color kOvercutColor(
    0.92, 0.10, 0.72, Quantity_TOC_RGB);
const Quantity_Color kOperationOvercutColor(
    1.0, 0.08, 0.36, Quantity_TOC_RGB);
const Quantity_Color kPocketStockColor(
    0.20, 0.76, 0.46, Quantity_TOC_RGB);

// 特征类型标识 → 中文显示名。
QString featureKindLabel(const std::string& kind)
{
    if (kind == "through_hole")
        return QStringLiteral("通孔");
    if (kind == "blind_hole")
        return QStringLiteral("盲孔");
    if (kind == "countersink_hole")
        return QStringLiteral("沉孔");
    if (kind == "slot")
        return QStringLiteral("槽");
    if (kind == "pocket")
        return QStringLiteral("腔");
    if (kind == "profile")
        return QStringLiteral("外轮廓");
    return QStringLiteral("特征");
}

} // namespace

// 委托构造:无会话参数版本复用带会话的构造,内部完成呈现器创建、
// 显示网格派生管道初始化,并把后台结果信号以队列方式接回主线程。
StockSimulationWorkspaceCoordinator::
StockSimulationWorkspaceCoordinator(
    presentation::OverlayController& overlays,
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view,
    QObject* parent)
    : StockSimulationWorkspaceCoordinator(
          overlays, context, view, StockSimulationSession{}, parent)
{
}

StockSimulationWorkspaceCoordinator::
StockSimulationWorkspaceCoordinator(
    presentation::OverlayController& overlays,
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view,
    StockSimulationSession initialSession,
    QObject* parent)
    : QObject(parent),
      m_session(std::move(initialSession)),
      m_presenter(
          std::make_unique<presentation::StockSimulationPresenter>(
              overlays, context, view)),
      m_view(view)
{
    // 初始化显示网格派生管道,并把后台线程的网格结果接到主线程处理。
    m_taskCoordinator.initializeDisplayPipeline();
    connect(
        m_taskCoordinator.displayMeshController(),
        &IpwDisplayMeshController::resultReady,
        this,
        &StockSimulationWorkspaceCoordinator::handleDisplayResult,
        Qt::QueuedConnection);
}

// 析构:取消显示网格派生并等待后台线程退出,避免访问已销毁的呈现器。
StockSimulationWorkspaceCoordinator::
~StockSimulationWorkspaceCoordinator()
{
    cancelAndWait();
}

void StockSimulationWorkspaceCoordinator::bindViewer(
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view)
{
    m_view = view;
    if (m_presenter)
        m_presenter->bindViewer(context, view);
}

StockSourceSettings
StockSimulationWorkspaceCoordinator::holeSourceSettings() const
{
    StockSourceSettings settings;
    settings.sourceMode = m_session.holeSourceMode();
    settings.marginX = m_session.holeMarginX();
    settings.marginY = m_session.holeMarginY();
    settings.marginZ = m_session.holeMarginZ();
    settings.stepPath = m_session.holeStockStepPath();
    return settings;
}

manufacturing::StockDefinition
StockSimulationWorkspaceCoordinator::holeStockDefinition() const
{
    return m_session.holeStockDefinition();
}

const TopoDS_Shape*
StockSimulationWorkspaceCoordinator::latestCompleteIpwStock() const noexcept
{
    const toolpath::StockSimulationResult& simulation =
        m_session.pocketSimulation();
    return m_session.hasPocketSimulation() &&
           m_session.pocketUsesUnifiedIpw() &&
           simulation.complete && !simulation.finalStock.IsNull()
        ? &simulation.finalStock
        : nullptr;
}

void StockSimulationWorkspaceCoordinator::restoreHoleStockDefinition(
    const manufacturing::StockDefinition& definition)
{
    m_session.restoreHoleStockDefinition(definition);
}

bool StockSimulationWorkspaceCoordinator::installUnifiedIpwRuntime(
    const manufacturing::MachiningJob& job)
{
    m_session.clearPocketResults();
    const manufacturing::ipw::IpwSession& ipw = job.ipwSession();
    if (!ipw.complete())
        return false;
    const auto root =
        job.runtime().ipwCheckpoints.find(ipw.initialStateKey());
    if (!root || !root->dexelStock)
        return false;

    toolpath::StockSimulationResult simulation;
    simulation.initialStock = root->stock;
    simulation.initialVolume = root->volume;
    simulation.finalStock = root->stock;
    simulation.finalVolume = root->volume;
    simulation.heightFieldSimulation = true;
    simulation.zMapResolution = root->dexelStock->resolution();
    simulation.zMapCellCount =
        root->dexelStock->nx() * root->dexelStock->ny();
    simulation.zMapActiveCellCount =
        root->dexelStock->activeCellCount();

    std::shared_ptr<const manufacturing::ipw::IpwStockCheckpoint> final =
        root;
    for (const manufacturing::ipw::IpwNode& node : ipw.nodes()) {
        const auto checkpoint =
            job.runtime().ipwCheckpoints.find(node.stateKey);
        if (!checkpoint || !checkpoint->dexelStock)
            return false;
        final = checkpoint;
        if (checkpoint->operationResult) {
            const toolpath::StockSimulationResult& operation =
                *checkpoint->operationResult;
            simulation.steps.insert(
                simulation.steps.end(),
                operation.steps.begin(), operation.steps.end());
            simulation.processedOperationCount +=
                operation.processedOperationCount;
            simulation.appliedOperationCount +=
                operation.appliedOperationCount;
            simulation.noMaterialOperationCount +=
                operation.noMaterialOperationCount;
            simulation.sweepSegmentCount += operation.sweepSegmentCount;
            simulation.zMapTileCount += operation.zMapTileCount;
            simulation.zMapUpdateMilliseconds +=
                operation.zMapUpdateMilliseconds;
            simulation.totalMilliseconds += operation.totalMilliseconds;
            simulation.maximumToolProfileAxialError = std::max(
                simulation.maximumToolProfileAxialError,
                operation.maximumToolProfileAxialError);
            simulation.maximumToolProfileRadialError = std::max(
                simulation.maximumToolProfileRadialError,
                operation.maximumToolProfileRadialError);
        }
    }
    simulation.finalVolume = final->volume;
    simulation.finalStock = final->stock;
    if (simulation.finalStock.IsNull()) {
        std::string snapshotError;
        if (!final->dexelStock->buildSnapshot(
                simulation.finalStock, nullptr, &snapshotError)) {
            return false;
        }
    }
    simulation.removedVolume = std::max(
        0.0, simulation.initialVolume - simulation.finalVolume);
    simulation.zMapResolution = final->dexelStock->resolution();
    simulation.zMapCellCount =
        final->dexelStock->nx() * final->dexelStock->ny();
    simulation.zMapActiveCellCount =
        final->dexelStock->activeCellCount();
    simulation.complete = true;
    m_session.pocketSimulation() = std::move(simulation);
    m_session.hasPocketSimulation() = true;
    m_session.pocketUsesUnifiedIpw() = true;
    m_session.pocketSourceFingerprint() =
        QString::fromStdString(ipw.initialStockFingerprint());
    m_session.pocketTransientOperations().assign(
        ipw.nodes().size(), {});
    return true;
}

HoleStockDialogModel StockSimulationWorkspaceCoordinator::holeDialogModel(
    std::span<const HolePreviewEntry> previewEntries) const
{
    HoleStockDialogModel model;
    model.hasSimulation = m_session.hasHoleSimulation();
    model.stepCount = static_cast<int>(
        m_session.holeSimulation().steps.size());
    model.overcutChoices =
        StockSimulationWorkflowController::holeOvercutChoices(
            m_session, previewEntries);
    return model;
}

QString StockSimulationWorkspaceCoordinator::holeSummary(
    std::span<const HolePreviewEntry> previewEntries,
    const HoleStockDisplaySelection& selection) const
{
    return StockSimulationWorkflowController::holeSummary(
        m_session, previewEntries, selection);
}

bool StockSimulationWorkspaceCoordinator::buildHoleOpenGlRequest(
    const std::vector<toolpath::HoleMachiningOperation>& operations,
    const toolpath::ToolpathParams& fallbackParams,
    int setupIndex,
    const TopoDS_Shape& targetShape,
    HoleOpenGlSimulationRequest& request,
    QString& error) const
{
    const toolpath::StockSimulationResult& simulation =
        m_session.holeSimulation();
    if (!m_session.hasHoleSimulation() ||
        simulation.initialStock.IsNull() || !simulation.complete) {
        error = QStringLiteral(
            "请先完成孔 IPW 仿真，再启动 OpenGL 刀具动画。");
        return false;
    }
    std::vector<std::size_t> selectedIndices;
    for (std::size_t index = 0u; index < operations.size(); ++index) {
        if (operations[index].setupIndex == setupIndex)
            selectedIndices.push_back(index);
    }
    if (selectedIndices.empty()) {
        error = QStringLiteral("所选 Setup 没有可播放的孔加工工序。");
        return false;
    }
    const std::size_t firstIndex = selectedIndices.front();
    const std::size_t lastIndex = selectedIndices.back();
    for (std::size_t index = firstIndex; index <= lastIndex; ++index) {
        if (operations[index].setupIndex != setupIndex) {
            error = QStringLiteral(
                "所选 Setup 的孔工序在程序中不是连续区间，"
                "无法建立单一 OpenGL 材料基线。");
            return false;
        }
    }
    if (simulation.steps.size() <= lastIndex) {
        error = QStringLiteral(
            "孔 IPW 快照数量少于当前孔程序，请重新执行孔 IPW。");
        return false;
    }

    TopoDS_Shape initialSnapshot = simulation.initialStock;
    if (firstIndex > 0u)
        initialSnapshot = simulation.steps[firstIndex - 1u].stockAfter;
    const TopoDS_Shape& finalSnapshot =
        simulation.steps[lastIndex].stockAfter;
    if (initialSnapshot.IsNull() || finalSnapshot.IsNull()) {
        error = QStringLiteral(
            "所选 Setup 缺少有效的加工前后孔 IPW 快照。");
        return false;
    }

    request = HoleOpenGlSimulationRequest{};
    request.initialStock = initialSnapshot;
    request.finalStock = finalSnapshot;
    request.targetShape = targetShape;
    request.operations.assign(
        operations.begin() + static_cast<std::ptrdiff_t>(firstIndex),
        operations.begin() + static_cast<std::ptrdiff_t>(lastIndex + 1u));
    request.fallbackParams = fallbackParams;
    request.displayQuality = displayQuality();
    return true;
}

PocketStockSimulationDialogState
StockSimulationWorkspaceCoordinator::pocketDialogState(
    std::span<const toolpath::PocketToolpathOperation> operations,
    const QString& browseDirectory) const
{
    return PocketStockSimulationUseCase::buildDialogState(
        m_session, operations, displayQuality(),
        triangulationEnabled(), displayCacheBudgetBytes(),
        browseDirectory);
}

PocketStockSimulationDialogState
StockSimulationWorkspaceCoordinator::profileDialogState(
    std::span<const toolpath::ProfileToolpathOperation> operations,
    const QString& browseDirectory) const
{
    return ProfileStockSimulationUseCase::buildDialogState(
        m_session, operations, displayQuality(),
        triangulationEnabled(), displayCacheBudgetBytes(),
        browseDirectory);
}

// 把仿真结果注入会话与任务协调器并执行:孔/槽用例内部使用这些引用
// 写入会话与派生显示请求,完成后通知特征余量变化。
HoleStockRunOutcome
StockSimulationWorkspaceCoordinator::executeHoleSimulation(
    HoleStockSimulationUseCaseRequest request,
    manufacturing::MachiningJob& job)
{
    request.session = &m_session;
    request.coordinator = &m_taskCoordinator;
    request.job = &job;
    HoleStockRunOutcome outcome =
        HoleStockSimulationUseCase::execute(request);
    emit featureAllowancesChanged();
    return outcome;
}

// 槽仿真在孔的基础上额外注入预览条目与生效参数作为输入。
PocketStockSimulationUseCaseResult
StockSimulationWorkspaceCoordinator::executePocketSimulation(
    PocketStockSimulationUseCaseRequest request,
    manufacturing::MachiningJob& job)
{
    request.session = &m_session;
    request.coordinator = &m_taskCoordinator;
    request.job = &job;
    PocketStockSimulationUseCaseResult result =
        PocketStockSimulationUseCase::execute(request);
    emit featureAllowancesChanged();
    return result;
}

ProfileStockSimulationUseCaseResult
StockSimulationWorkspaceCoordinator::executeProfileSimulation(
    ProfileStockSimulationUseCaseRequest request,
    manufacturing::MachiningJob& job,
    const std::vector<toolpath::ProfileToolpathOperation>& operations,
    const toolpath::ToolpathParams& effectiveParams)
{
    request.session = &m_session;
    request.coordinator = &m_taskCoordinator;
    request.job = &job;
    request.operations = &operations;
    request.effectiveParams = &effectiveParams;
    ProfileStockSimulationUseCaseResult result =
        ProfileStockSimulationUseCase::execute(request);
    emit featureAllowancesChanged();
    return result;
}

ProjectStockSimulationUseCaseResult
StockSimulationWorkspaceCoordinator::executeProjectSimulation(
    ProjectStockSimulationUseCaseRequest request, manufacturing::MachiningJob& job)
{
    request.session = &m_session;
    request.coordinator = &m_taskCoordinator;
    request.job = &job;
    auto result = ProjectStockSimulationUseCase::execute(request);
    emit featureAllowancesChanged();
    return result;
}

void StockSimulationWorkspaceCoordinator::requestCancel() noexcept
{
    m_taskCoordinator.requestCancel();
}

// 同步取消:先请求取消,再等待显示网格派生线程收尾(保证销毁安全)。
void StockSimulationWorkspaceCoordinator::cancelAndWait()
{
    m_taskCoordinator.requestCancel();
    m_taskCoordinator.cancelDisplayAndWait();
}

// 模型切换级联重置:取消显示派生、清空网格缓存、移除呈现对象并
// 复位会话(毛坯/结果/指纹),最后通知特征余量变化。
void StockSimulationWorkspaceCoordinator::resetForModel()
{
    m_taskCoordinator.cancelDisplay(true);
    m_taskCoordinator.clearDisplayCache();
    m_presenter->clearHole();
    m_presenter->removePocketObjects();
    m_presenter->resetDisplayedFrame();
    m_session.resetForModel();
    emit featureAllowancesChanged();
}

void StockSimulationWorkspaceCoordinator::setDisplayCacheBudgetBytes(
    std::size_t bytes)
{
    m_taskCoordinator.setDisplayCacheBudgetBytes(bytes);
}

std::size_t
StockSimulationWorkspaceCoordinator::displayCacheBudgetBytes() const
{
    return m_taskCoordinator.displayCacheBudgetBytes();
}

int StockSimulationWorkspaceCoordinator::displayQuality() const noexcept
{
    return m_presenter->displayQuality();
}

void StockSimulationWorkspaceCoordinator::setDisplayQuality(
    int quality) noexcept
{
    m_presenter->displayQuality() = std::clamp(quality, 1, 10);
}

bool StockSimulationWorkspaceCoordinator::
triangulationEnabled() const noexcept
{
    return m_presenter->triangulationEnabled();
}

void StockSimulationWorkspaceCoordinator::setTriangulationEnabled(
    bool enabled) noexcept
{
    m_presenter->triangulationEnabled() = enabled;
}

IpwOcctDisplayMode
StockSimulationWorkspaceCoordinator::displayMode() const noexcept
{
    return m_presenter->displayMode();
}

void StockSimulationWorkspaceCoordinator::setDisplayMode(
    IpwOcctDisplayMode mode) noexcept
{
    m_presenter->displayMode() = mode;
}

// OCCT 呈现是否可用:显示形式非 Off,且表面形式(需网格)时已开启三角化。
bool StockSimulationWorkspaceCoordinator::occtDisplayEnabled() const noexcept
{
    return displayMode() != IpwOcctDisplayMode::Off &&
        (!isIpwSurfaceDisplayMode(displayMode()) ||
         triangulationEnabled());
}

// 清除孔仿真:移除呈现,按需同时清空会话结果与指纹。
void StockSimulationWorkspaceCoordinator::clearHole(bool clearResult)
{
    m_presenter->clearHole();
    if (clearResult) {
        m_session.clearHoleResults();
        m_session.holeSourceFingerprint().clear();
        emit featureAllowancesChanged();
    }
    if (!m_view.IsNull())
        m_view->Redraw();
}

// 清除槽仿真:取消进行中的显示派生,移除呈现并按需清空会话结果。
void StockSimulationWorkspaceCoordinator::clearPocket(bool clearResult)
{
    m_taskCoordinator.cancelDisplay(clearResult);
    m_presenter->removePocketObjects();
    m_presenter->resetDisplayedFrame();
    if (clearResult) {
        m_session.clearPocketResults();
        m_session.pocketSourceFingerprint().clear();
        emit featureAllowancesChanged();
    }
}

// 显示孔快照:stepIndex 为 0 显示初始毛坯,否则取对应步的加工后毛坯;
// 先清掉槽呈现,再以半透明青色发布孔毛坯形状。
void StockSimulationWorkspaceCoordinator::displayHoleSnapshot(
    int stepIndex,
    const gp_Trsf& modelTransform)
{
    if (!m_session.hasHoleSimulation())
        return;
    clearPocket(false);
    TopoDS_Shape shape = m_session.holeSimulation().initialStock;
    if (stepIndex > 0 &&
        stepIndex <= static_cast<int>(
            m_session.holeSimulation().steps.size())) {
        shape = m_session.holeSimulation()
                    .steps[static_cast<std::size_t>(stepIndex - 1)]
                    .stockAfter;
    }
    if (shape.IsNull()) {
        clearHole(false);
        return;
    }
    const auto result = m_presenter->presentHoleShape(
        shape, kHoleStockColor, 0.48,
        Graphic3d_ZLayerId_Default, modelTransform);
    if (!result.ok) {
        emit statusChanged(QString::fromStdString(result.error));
    }
}

// 显示孔对比视图:viewMode 1/2/3 分别显示初始缺料/剩余余量/过切,
// 其他值回退为显示初始毛坯快照。
void StockSimulationWorkspaceCoordinator::displayHoleComparison(
    int viewMode,
    const gp_Trsf& modelTransform)
{
    clearPocket(false);
    TopoDS_Shape shape;
    Quantity_Color color = kInitialMissingColor;
    double transparency = 0.10;
    switch (viewMode) {
        case 1:
            shape = m_session.holeComparison().initialMissing;
            break;
        case 2:
            shape = m_session.holeComparison().remainingAllowance;
            color = kRemainingAllowanceColor;
            transparency = 0.28;
            break;
        case 3:
            shape = m_session.holeComparison().overcut;
            color = kOvercutColor;
            break;
        default:
            displayHoleSnapshot(0, modelTransform);
            return;
    }
    if (shape.IsNull()) {
        clearHole(false);
        return;
    }
    const auto result = m_presenter->presentHoleShape(
        shape, color, transparency, Graphic3d_ZLayerId_Top,
        modelTransform, viewMode == 2);
    if (!result.ok)
        emit statusChanged(QString::fromStdString(result.error));
}

// 显示某工序的过切归属形状;索引越界或形状为空时清空孔显示。
void StockSimulationWorkspaceCoordinator::
displayHoleOperationOvercut(
    int attributionIndex,
    const gp_Trsf& modelTransform)
{
    if (attributionIndex < 0 ||
        attributionIndex >= static_cast<int>(
            m_session.holeOvercutAttribution().operations.size())) {
        clearHole(false);
        return;
    }
    clearPocket(false);
    const TopoDS_Shape& shape =
        m_session.holeOvercutAttribution()
            .operations[static_cast<std::size_t>(attributionIndex)]
            .shape;
    if (shape.IsNull()) {
        clearHole(false);
        return;
    }
    const auto result = m_presenter->presentHoleShape(
        shape, kOperationOvercutColor, 0.02,
        Graphic3d_ZLayerId_Top, modelTransform);
    if (!result.ok)
        emit statusChanged(QString::fromStdString(result.error));
}

// 特征余量列表:优先取统一 IPW(槽)结果,否则取孔对比结果;
// 每个余量项带特征 id/工序 id、显示名、目标/剩余体积与完成率。
std::vector<FeatureAllowanceViewItem>
StockSimulationWorkspaceCoordinator::featureAllowanceItems() const
{
    const bool useUnified = m_session.hasPocketSimulation() &&
        !m_session.pocketComparison().featureAllowances.empty();
    const auto& allowances = useUnified
        ? m_session.pocketComparison().featureAllowances
        : m_session.holeComparison().featureAllowances;
    std::vector<FeatureAllowanceViewItem> items;
    items.reserve(allowances.size());
    for (std::size_t index = 0; index < allowances.size(); ++index) {
        const toolpath::StockFeatureAllowance& allowance =
            allowances[index];
        FeatureAllowanceViewItem item;
        item.resultIndex = static_cast<int>(index);
        item.unifiedIpw = useUnified;
        item.featureId = QString::fromStdString(allowance.featureId);
        item.operationId = QString::fromStdString(allowance.operationId);
        item.featureKind = featureKindLabel(allowance.featureKind);
        item.displayName = allowance.sourceIndex >= 0
            ? QStringLiteral("%1 %2")
                  .arg(item.featureKind)
                  .arg(allowance.sourceIndex + 1)
            : QStringLiteral("%1 · %2")
                  .arg(item.featureKind, item.operationId);
        item.targetRemovalVolume = allowance.targetRemovalVolume;
        item.remainingAllowanceVolume =
            allowance.remainingAllowanceVolume;
        item.completionRatio = allowance.completionRatio;
        item.approximateBoundary = allowance.approximateBoundary;
        items.push_back(std::move(item));
    }
    return items;
}

// 显示单个特征的剩余余量:索引越界/余量已清零/缺显示源时发状态消息;
// 统一 IPW 结果走高度场差网格(缓存优先,未命中则异步派生)。
void StockSimulationWorkspaceCoordinator::displayFeatureAllowance(
    const FeatureAllowanceViewItem& item,
    const gp_Trsf& modelTransform)
{
    const toolpath::StockComparisonResult& comparison = item.unifiedIpw
        ? m_session.pocketComparison()
        : m_session.holeComparison();
    if (item.resultIndex < 0 ||
        item.resultIndex >=
            static_cast<int>(comparison.featureAllowances.size())) {
        emit statusChanged(QStringLiteral("所选特征余量结果已失效，请重新打开视图。"));
        return;
    }
    const toolpath::StockFeatureAllowance& allowance =
        comparison.featureAllowances[
            static_cast<std::size_t>(item.resultIndex)];
    clearHole(false);
    m_taskCoordinator.cancelDisplay(false);
    // 余量体积极小视为加工完成,无可显示内容。
    if (allowance.remainingAllowanceVolume <= 1.0e-12) {
        removePocketPresentation();
        emit statusChanged(QStringLiteral(
            "%1 已加工完成，没有可显示的剩余余量。")
            .arg(item.displayName));
        return;
    }

    if (!item.unifiedIpw) {
        if (allowance.remainingAllowance.IsNull()) {
            removePocketPresentation();
            emit statusChanged(QStringLiteral("该特征没有可显示的 BRep 余量。"));
            return;
        }
        presentPocketBRep(
            allowance.remainingAllowance,
            kRemainingAllowanceColor, 0.12, true,
            modelTransform);
        emit statusChanged(QStringLiteral(
            "%1 加工后余量：%2 mm³，完成率 %3%。")
            .arg(item.displayName)
            .arg(allowance.remainingAllowanceVolume, 0, 'f', 3)
            .arg(allowance.completionRatio * 100.0, 0, 'f', 1));
        return;
    }

    if (!comparison.heightFieldDisplaySource) {
        removePocketPresentation();
        emit statusChanged(QStringLiteral("该特征缺少 Multi-Dexel 余量显示源。"));
        return;
    }
    // 特征余量网格依赖三角化的表面显示形式,不满足时给出警告。
    if (!isIpwSurfaceDisplayMode(displayMode()) ||
        !triangulationEnabled()) {
        removePocketPresentation();
        emit warningRequested(
            QStringLiteral("特征余量视图"),
            QStringLiteral(
                "特征余量网格需要启用三角化，并将 OCCT 显示形式设为带网格、着色或线框表面。"));
        return;
    }

    const IpwDisplayQualityProfile qualityProfile =
        ipwDisplayQualityProfile(
            displayQuality(), comparison.heightFieldResolution);
    IpwDisplayArtifactKey key;
    key.contentFingerprint =
        comparison.heightFieldDisplaySource->contentFingerprint();
    key.variant = allowance.featureId;
    key.kind = IpwDisplayArtifactKind::FeatureAllowance;
    key.quality = qualityProfile.quality;
    const Handle(Poly_Triangulation) cached =
        m_taskCoordinator.findDisplayMesh(key);
    if (!cached.IsNull()) {
        presentPocketTriangulation(
            cached, Quantity_Color(1.0, 0.72, 0.08, Quantity_TOC_RGB),
            0.12, true, displayMode(), modelTransform);
        emit statusChanged(QStringLiteral(
            "%1 加工后余量：%2 mm³，完成率 %3%（缓存）。")
            .arg(item.displayName)
            .arg(allowance.remainingAllowanceVolume, 0, 'f', 3)
            .arg(allowance.completionRatio * 100.0, 0, 'f', 1));
        return;
    }

    IpwDisplayMeshRequest request;
    request.key = std::move(key);
    request.differenceSource = comparison.heightFieldDisplaySource;
    request.differenceView =
        toolpath::PocketZMapDifferenceView::FeatureAllowance;
    request.operationId = allowance.featureId;
    request.meshOptions = qualityProfile.meshOptions;
    request.displayMode = displayMode();
    requestDisplay(std::move(request), modelTransform);
}

// 以三角化网格发布槽呈现(表面形式),失败时保留当前进程并上报状态。
bool StockSimulationWorkspaceCoordinator::presentPocketTriangulation(
    const Handle(Poly_Triangulation)& mesh,
    const Quantity_Color& color,
    double transparency,
    bool topLayer,
    IpwOcctDisplayMode mode,
    const gp_Trsf& modelTransform)
{
    const auto result = m_presenter->presentPocketTriangulation(
        mesh, color, transparency, topLayer, mode, modelTransform);
    if (!result.ok) {
        emit statusChanged(
            QStringLiteral("统一 IPW 网格发布失败，已保留当前进程：%1")
                .arg(QString::fromStdString(result.error)));
    }
    return result.ok;
}

// 以直接图元(柱/点云/体素柱)发布槽呈现。
bool StockSimulationWorkspaceCoordinator::
presentPocketPrimitiveGeometry(
    const IpwDisplayPrimitiveGeometryPtr& geometry,
    const Quantity_Color& color,
    double transparency,
    bool topLayer,
    const gp_Trsf& modelTransform)
{
    const auto result = m_presenter->presentPocketPrimitiveGeometry(
        geometry, color, transparency, topLayer, modelTransform);
    if (!result.ok) {
        emit statusChanged(
            QStringLiteral("Multi-Dexel 直接图元发布失败：%1")
                .arg(QString::fromStdString(result.error)));
    }
    return result.ok;
}

// 以 BRep 形状发布槽呈现(BRep 快照形式)。
bool StockSimulationWorkspaceCoordinator::presentPocketBRep(
    const TopoDS_Shape& shape,
    const Quantity_Color& color,
    double transparency,
    bool topLayer,
    const gp_Trsf& modelTransform)
{
    const auto result = m_presenter->presentPocketBRep(
        shape, color, transparency, topLayer, modelTransform);
    if (!result.ok) {
        emit statusChanged(
            QStringLiteral("Multi-Dexel BRep 发布失败：%1")
                .arg(QString::fromStdString(result.error)));
    }
    return result.ok;
}

// 移除当前槽呈现对象(不区分呈现形式)。
void StockSimulationWorkspaceCoordinator::removePocketPresentation()
{
    m_presenter->removePocketObjects();
}

// 提交异步显示网格请求:先取消旧显示并移除旧呈现,记录本次模型变换
// 供后台完成后按同一位姿发布;后台线程不可用或提交失败时回退并提示。
void StockSimulationWorkspaceCoordinator::requestDisplay(
    IpwDisplayMeshRequest request,
    const gp_Trsf& modelTransform)
{
    m_taskCoordinator.cancelDisplay(false);
    removePocketPresentation();
    m_requestedModelTransform = modelTransform;
    if (!m_taskCoordinator.displayMeshController() ||
        !m_taskCoordinator.displayMeshController()->running()) {
        emit statusChanged(
            QStringLiteral("IPW 显示网格后台线程不可用。"));
        return;
    }
    emit statusChanged(
        StockSimulationCoordinator::displayRequestStatus(request));
    if (!m_taskCoordinator.submitDisplay(std::move(request))) {
        m_taskCoordinator.cancelDisplay(false);
        emit statusChanged(
            QStringLiteral("无法提交 IPW 显示网格后台请求。"));
    }
}

// 消费后台显示网格结果:按处置分发——丢弃/取消/失败时清理并通知,
// 否则按显示形式(表面/网格/BRep/图元/Off)发布,最后统一发完成信号。
void StockSimulationWorkspaceCoordinator::handleDisplayResult(
    const IpwDisplayMeshResult& result)
{
    const IpwDisplayPublication publication =
        m_taskCoordinator.evaluateDisplayResult(
            result, displayMode(), triangulationEnabled());
    using Disposition = IpwDisplayPublicationDisposition;
    if (publication.disposition == Disposition::Discard) {
        removePocketPresentation();
        emit asynchronousDisplayFinished();
        return;
    }
    if (publication.disposition == Disposition::Cancelled) {
        emit statusChanged(publication.status);
        emit asynchronousDisplayFinished();
        return;
    }
    if (publication.disposition == Disposition::Failed) {
        emit statusChanged(publication.error);
        emit warningRequested(
            QStringLiteral("统一 IPW 显示网格"), publication.error);
        removePocketPresentation();
        emit asynchronousDisplayFinished();
        return;
    }

    const IpwDisplayStyle& style = publication.style;
    const Quantity_Color color(
        style.red, style.green, style.blue, Quantity_TOC_RGB);
    bool displayed = false;
    // 按显示形式选择发布路径,统一使用请求时记录的模型变换。
    if (isIpwSurfaceDisplayMode(result.displayMode)) {
        displayed = presentPocketTriangulation(
            result.mesh, color, style.transparency, style.topLayer,
            result.displayMode, m_requestedModelTransform);
    } else if (result.displayMode == IpwOcctDisplayMode::BRepSnapshot) {
        displayed = presentPocketBRep(
            result.shape, color, style.transparency, style.topLayer,
            m_requestedModelTransform);
    } else if (result.displayMode == IpwOcctDisplayMode::Off) {
        removePocketPresentation();
        displayed = true;
    } else {
        displayed = presentPocketPrimitiveGeometry(
            result.primitiveGeometry, color,
            style.transparency, style.topLayer,
            m_requestedModelTransform);
    }
    if (displayed)
        emit statusChanged(publication.status);
    emit asynchronousDisplayFinished();
}

// 显示槽(统一 IPW)检查点快照:按步号取 IPW 会话的检查点,记录当前
// 显示帧(非瞬时),优先走网格缓存,未命中则异步派生;非表面显示
// 形式一律走异步派生。
void StockSimulationWorkspaceCoordinator::displayPocketSnapshot(
    const manufacturing::MachiningJob& job,
    int stepIndex,
    const gp_Trsf& modelTransform)
{
    m_taskCoordinator.cancelDisplay(false);
    if (!m_session.hasPocketSimulation())
        return;
    // 记录“当前帧”语义:stepIndex + 非瞬时,供刷新/时间线重建。
    m_presenter->displayedStepIndex() = stepIndex;
    m_presenter->displayedOperationIndex() = -1;
    m_presenter->displayedCompletedMoveCount() = -1;
    m_presenter->displayedFrameIsTransient() = false;
    clearHole(false);

    if (displayMode() == IpwOcctDisplayMode::Off) {
        removePocketPresentation();
        emit statusChanged(
            QStringLiteral("Multi-Dexel 主视口显示已关闭。"));
        return;
    }
    if (isIpwSurfaceDisplayMode(displayMode()) &&
        !triangulationEnabled()) {
        removePocketPresentation();
        emit statusChanged(QStringLiteral(
            "当前表面形式需要三角化；可改选 Dexel 柱、点云、"
            "体素柱、剖面或 BRep。"));
        return;
    }

    std::shared_ptr<const manufacturing::ipw::IpwStockCheckpoint>
        checkpoint;
    std::string stateKey;
    const auto& ipwSession = job.ipwSession();
    // 统一 IPW:stepIndex 0 取初始检查点,否则取对应节点的状态检查点。
    if (m_session.pocketUsesUnifiedIpw() && ipwSession.configured()) {
        if (stepIndex == 0) {
            stateKey = ipwSession.initialStateKey();
        } else if (stepIndex > 0 &&
                   stepIndex <= static_cast<int>(ipwSession.nodes().size())) {
            stateKey = ipwSession.nodes()[
                static_cast<std::size_t>(stepIndex - 1)].stateKey;
        }
        if (!stateKey.empty())
            checkpoint = job.runtime().ipwCheckpoints.find(stateKey);
    }
    if (!checkpoint || !checkpoint->dexelStock) {
        removePocketPresentation();
        emit statusChanged(QStringLiteral(
            "该结果不是统一 Multi-Dexel IPW，已停止显示 BRep 毛坯。"));
        return;
    }

    const IpwDisplayQualityProfile qualityProfile =
        ipwDisplayQualityProfile(
            displayQuality(), checkpoint->dexelStock->resolution());
    // 以内容指纹+质量档位构成缓存键,表面形式可直接查缓存命中。
    IpwDisplayArtifactKey artifactKey;
    artifactKey.contentFingerprint =
        checkpoint->dexelStock->contentFingerprint();
    artifactKey.kind = IpwDisplayArtifactKind::StockCheckpoint;
    artifactKey.quality = qualityProfile.quality;
    if (!isIpwSurfaceDisplayMode(displayMode())) {
        IpwDisplayMeshRequest request;
        request.key = std::move(artifactKey);
        request.stockSource = checkpoint->dexelStock;
        request.meshOptions = qualityProfile.meshOptions;
        request.displayMode = displayMode();
        requestDisplay(std::move(request), modelTransform);
        return;
    }

    Handle(Poly_Triangulation) mesh =
        m_taskCoordinator.findDisplayMesh(artifactKey);
    if (mesh.IsNull()) {
        IpwDisplayMeshRequest request;
        request.key = std::move(artifactKey);
        request.stockSource = checkpoint->dexelStock;
        request.meshOptions = qualityProfile.meshOptions;
        request.displayMode = displayMode();
        requestDisplay(std::move(request), modelTransform);
        return;
    }

    presentPocketTriangulation(
        mesh, kPocketStockColor, 0.20, true,
        displayMode(), modelTransform);
    const IpwDisplayCacheStats cacheStats =
        m_taskCoordinator.displayCacheStats();
    emit statusChanged(QStringLiteral(
        "IPW 检查点网格：缓存命中（%1 项，%2 MiB）。")
        .arg(cacheStats.residentArtifactCount)
        .arg(cacheStats.residentBytes / (1024.0 * 1024.0),
             0, 'f', 1));
}

// 统计某槽操作的总移动数:优先用运行期捕获的瞬时操作,否则从刀路批次
// 按运行时 id 匹配统计;结果被钳制到 int 上限。
int StockSimulationWorkspaceCoordinator::pocketOperationMoveCount(
    const manufacturing::MachiningJob& job,
    int operationIndex) const
{
    const auto& ipwSession = job.ipwSession();
    const manufacturing::ipw::IpwNode* node = operationIndex >= 0
        ? ipwSession.node(static_cast<std::size_t>(operationIndex))
        : nullptr;
    if (!node) {
        return 0;
    }
    std::size_t count = 0u;
    if (node->operation.kind == manufacturing::ipw::IpwOperationKind::SurfaceFinishing) {
        const auto* finishing = job.currentFinishingResult();
        if (!finishing)
            return 0;
        for (const auto& operation : finishing->operations)
            if (manufacturing::runtimeOperationIdMatches(node->operation.id, operation.operationId))
                count += operation.result.moves.size();
        return static_cast<int>(std::min<std::size_t>(count,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    }
    if (node->operation.kind ==
        manufacturing::ipw::IpwOperationKind::Profile) {
        const toolpath::ProfileToolpathBatch* profile =
            job.profileResult();
        if (!profile)
            return 0;
        for (const auto& operation : profile->operations) {
            if (StockSimulationWorkflowController::pocketRuntimeIdMatches(
                    node->operation.id, operation.operationId)) {
                count += operation.contour.moves.size();
            }
        }
        if (operationIndex < static_cast<int>(
                m_session.pocketTransientOperations().size()) &&
            m_session.pocketTransientOperations()[
                static_cast<std::size_t>(operationIndex)]) {
            count = 0u;
            for (const auto& operation :
                 *m_session.pocketTransientOperations()[
                     static_cast<std::size_t>(operationIndex)]) {
                count += operation.clearing.moves.size();
            }
        }
        return static_cast<int>(std::min<std::size_t>(
            count, static_cast<std::size_t>(
                       std::numeric_limits<int>::max())));
    }
    const toolpath::PocketToolpathBatch* batch = job.pocketResult();
    if (!batch ||
        node->operation.kind != manufacturing::ipw::
            IpwOperationKind::Pocket) {
        return 0;
    }
    if (operationIndex < static_cast<int>(
            m_session.pocketTransientOperations().size()) &&
        m_session.pocketTransientOperations()[
            static_cast<std::size_t>(operationIndex)]) {
        for (const auto& operation :
             *m_session.pocketTransientOperations()[
                 static_cast<std::size_t>(operationIndex)]) {
            count += operation.clearing.moves.size();
        }
    } else {
        for (const auto& operation : batch->operations) {
            if (StockSimulationWorkflowController::pocketRuntimeIdMatches(
                    node->operation.id, operation.operationId)) {
                count += operation.clearing.moves.size();
            }
        }
    }
    return static_cast<int>(std::min<std::size_t>(
        count, static_cast<std::size_t>(
                   std::numeric_limits<int>::max())));
}

// 显示槽操作的瞬时帧(部分完成):以父检查点 + 已完成的移动数为输入
// 在后台派生插值网格;完成度为 0/全部时分别回退到前后相邻快照。
void StockSimulationWorkspaceCoordinator::displayPocketTransient(
    const manufacturing::MachiningJob& job,
    int operationIndex,
    int completedMoveCount,
    const gp_Trsf& modelTransform)
{
    if (!m_session.hasPocketSimulation() ||
        !m_session.pocketUsesUnifiedIpw()) {
        return;
    }
    const auto& ipwSession = job.ipwSession();
    const manufacturing::ipw::IpwNode* node = operationIndex >= 0
        ? ipwSession.node(static_cast<std::size_t>(operationIndex))
        : nullptr;
    if (!node) {
        emit statusChanged(
            QStringLiteral("当前 IPW 节点不存在。"));
        return;
    }

    toolpath::ToolpathParams effectiveParams;
    std::vector<toolpath::PocketToolpathOperation> captured;
    std::vector<toolpath::DexelToolProfileDefinition> toolProfiles;
    if (node->operation.kind ==
        manufacturing::ipw::IpwOperationKind::Profile) {
        const toolpath::ProfileToolpathBatch* profile =
            job.profileResult();
        if (!profile) {
            emit statusChanged(QStringLiteral(
                "当前 IPW 节点没有可重放的外轮廓刀路。"));
            return;
        }
        effectiveParams = profile->effectiveParams;
        if (operationIndex < static_cast<int>(
                m_session.pocketTransientOperations().size()) &&
            m_session.pocketTransientOperations()[
                static_cast<std::size_t>(operationIndex)]) {
            captured = *m_session.pocketTransientOperations()[
                static_cast<std::size_t>(operationIndex)];
        } else {
            for (const auto& operation : profile->operations) {
                if (StockSimulationWorkflowController::pocketRuntimeIdMatches(
                        node->operation.id, operation.operationId)) {
                    captured.push_back(
                        toolpath::legacyProfileSimulationOperation(operation));
                }
            }
        }
    } else if (node->operation.kind ==
               manufacturing::ipw::IpwOperationKind::Pocket) {
        const toolpath::PocketToolpathBatch* batch = job.pocketResult();
        if (!batch) {
            emit statusChanged(QStringLiteral(
                "当前 IPW 节点没有可重放的腔刀路。"));
            return;
        }
        effectiveParams = batch->effectiveParams;
        for (const auto& operation : batch->operations) {
            if (StockSimulationWorkflowController::pocketRuntimeIdMatches(
                    node->operation.id, operation.operationId)) {
                captured.push_back(operation);
            }
        }
    } else if (node->operation.kind ==
               manufacturing::ipw::IpwOperationKind::SurfaceFinishing) {
        const auto* finishing = job.currentFinishingResult();
        if (!finishing) {
            emit statusChanged(QStringLiteral("当前精加工刀路已失效，请重新生成。"));
            return;
        }
        for (const auto& operation : finishing->operations) {
            if (!manufacturing::runtimeOperationIdMatches(node->operation.id, operation.operationId))
                continue;
            effectiveParams = operation.result.toolpathParams();
            // Display-only carrier for the legacy transient mesh request. The
            // formal operation and native IPW never change their family.
            toolpath::PocketToolpathOperation carrier;
            carrier.operationId = operation.operationId;
            carrier.outwardToolAxis = gp_Vec(operation.workFrame.Direction());
            carrier.tool = operation.tool;
            carrier.clearing.moves = operation.result.moves;
            carrier.clearing.primitives = operation.result.primitives;
            captured.push_back(std::move(carrier));
            toolpath::DexelToolProfileDefinition profile;
            profile.diameter = effectiveParams.tool.diameter;
            profile.cornerRadius = effectiveParams.tool.cornerRadius;
            profile.cuttingLength = effectiveParams.tool.cuttingEdgeLength;
            toolProfiles.push_back(profile);
        }
    } else {
        emit statusChanged(QStringLiteral(
            "当前 IPW 节点不支持逐移动重放。"));
        return;
    }

    std::shared_ptr<const std::vector<
        toolpath::PocketToolpathOperation>> operations;
    if (node->operation.kind ==
            manufacturing::ipw::IpwOperationKind::Pocket &&
        operationIndex < static_cast<int>(
            m_session.pocketTransientOperations().size())) {
        operations = m_session.pocketTransientOperations()[
            static_cast<std::size_t>(operationIndex)];
    }
    if (!operations) {
        operations = std::make_shared<const std::vector<
            toolpath::PocketToolpathOperation>>(std::move(captured));
    }
    std::size_t totalMoveCount = 0u;
    for (const auto& operation : *operations)
        totalMoveCount += operation.clearing.moves.size();
    if (operations->empty()) {
        emit statusChanged(QStringLiteral(
            "当前 IPW 节点的运行期腔刀路已失效。"));
        return;
    }

    const std::size_t completed = std::min(
        static_cast<std::size_t>(std::max(0, completedMoveCount)),
        totalMoveCount);
    // 完成度边界:0 显示操作前快照,全部完成显示下一节点快照。
    if (completed == 0u) {
        displayPocketSnapshot(job, operationIndex, modelTransform);
        return;
    }
    if (completed >= totalMoveCount) {
        displayPocketSnapshot(job, operationIndex + 1, modelTransform);
        return;
    }

    // 记录“瞬时帧”语义,供刷新时原样重建。
    m_presenter->displayedStepIndex() = -1;
    m_presenter->displayedOperationIndex() = operationIndex;
    m_presenter->displayedCompletedMoveCount() =
        static_cast<int>(completed);
    m_presenter->displayedFrameIsTransient() = true;
    if (displayMode() == IpwOcctDisplayMode::Off) {
        m_taskCoordinator.cancelDisplay(false);
        removePocketPresentation();
        emit statusChanged(
            QStringLiteral("Multi-Dexel 主视口显示已关闭。"));
        return;
    }
    if (isIpwSurfaceDisplayMode(displayMode()) &&
        !triangulationEnabled()) {
        m_taskCoordinator.cancelDisplay(false);
        removePocketPresentation();
        emit statusChanged(QStringLiteral(
            "当前表面形式需要三角化；直接图元和 BRep 仍可使用。"));
        return;
    }

    const auto parent =
        job.runtime().ipwCheckpoints.find(node->parentStateKey);
    if (!parent || !parent->dexelStock) {
        emit statusChanged(
            QStringLiteral("IPW 临时帧缺少不可变父检查点。"));
        return;
    }
    // 构造瞬时帧源:父检查点 + 待重放操作 + 完成移动数,作为请求键的
    // variant 传入,使不同进度各有一份缓存产物。
    auto source = std::make_shared<IpwTransientFrameSource>();
    source->parentStock = parent->dexelStock;
    source->operations = std::move(operations);
    source->toolProfiles = std::move(toolProfiles);
    source->params = effectiveParams;
    source->options = m_session.pocketIpwOptions();
    source->completedMoveCount = completed;
    source->sequenceIndex = node->operation.sequence;

    const IpwDisplayQualityProfile qualityProfile =
        ipwDisplayQualityProfile(
            displayQuality(), parent->dexelStock->resolution());
    IpwDisplayMeshRequest request;
    request.key.contentFingerprint =
        parent->dexelStock->contentFingerprint();
    request.key.variant = node->stateKey + ':' +
        std::to_string(completed);
    request.key.kind = IpwDisplayArtifactKind::StockCheckpoint;
    request.key.quality = qualityProfile.quality;
    request.transientSource = std::move(source);
    request.meshOptions = qualityProfile.meshOptions;
    request.displayMode = displayMode();
    requestDisplay(std::move(request), modelTransform);
}

// 刷新槽显示:若当前显示的是瞬时帧则原样重建,否则按当前 stepIndex
// 重放快照(默认取最后一个节点)。
void StockSimulationWorkspaceCoordinator::refreshPocketDisplay(
    const manufacturing::MachiningJob& job,
    const gp_Trsf& modelTransform)
{
    m_taskCoordinator.cancelDisplay(false);
    if (displayMode() == IpwOcctDisplayMode::Off) {
        removePocketPresentation();
        emit statusChanged(
            QStringLiteral("Multi-Dexel 主视口显示已关闭。"));
        return;
    }
    if (!m_session.hasPocketSimulation() ||
        !m_session.pocketUsesUnifiedIpw()) {
        removePocketPresentation();
        emit statusChanged(
            QStringLiteral("Multi-Dexel 显示形式已设为“%1”。")
                .arg(StockSimulationCoordinator::displayModeText(
                    displayMode())));
        return;
    }
    if (m_presenter->displayedFrameIsTransient() &&
        m_presenter->displayedOperationIndex() >= 0 &&
        m_presenter->displayedCompletedMoveCount() >= 0) {
        displayPocketTransient(
            job, m_presenter->displayedOperationIndex(),
            m_presenter->displayedCompletedMoveCount(), modelTransform);
        return;
    }
    int stepIndex = m_presenter->displayedStepIndex();
    if (stepIndex < 0)
        stepIndex = static_cast<int>(job.ipwSession().nodes().size());
    displayPocketSnapshot(job, stepIndex, modelTransform);
}

// 构建时间线操作列表:全部正式 IPW 节点都进入视图；只有 Ready 且父/子
// 检查点完整的节点可回放，失效、失败或缓存缺失节点保留明确状态。
std::vector<IpwTimelineOperation>
StockSimulationWorkspaceCoordinator::timelineOperations(
    const manufacturing::MachiningJob& job) const
{
    std::vector<IpwTimelineOperation> operations;
    const auto& ipwSession = job.ipwSession();
    if (!m_session.pocketUsesUnifiedIpw() ||
        !ipwSession.configured() || ipwSession.nodes().empty()) {
        return operations;
    }
    operations.reserve(ipwSession.nodes().size());
    const auto statusText = [](manufacturing::ipw::IpwNodeStatus status) {
        using Status = manufacturing::ipw::IpwNodeStatus;
        switch (status) {
            case Status::Dirty: return QStringLiteral("待重建");
            case Status::Computing: return QStringLiteral("计算中");
            case Status::Ready: return QStringLiteral("就绪");
            case Status::Failed: return QStringLiteral("失败");
            case Status::Cancelled: return QStringLiteral("已取消");
        }
        return QStringLiteral("未知");
    };
    for (std::size_t index = 0; index < ipwSession.nodes().size(); ++index) {
        const auto& node = ipwSession.nodes()[index];
        const int moveCount = pocketOperationMoveCount(
            job, static_cast<int>(index));
        const bool hasParentCheckpoint =
            job.runtime().ipwCheckpoints.find(node.parentStateKey) != nullptr;
        const bool hasNodeCheckpoint =
            job.runtime().ipwCheckpoints.find(node.stateKey) != nullptr;
        const bool replayAvailable =
            node.status == manufacturing::ipw::IpwNodeStatus::Ready &&
            node.artifact.has_value() && hasParentCheckpoint &&
            hasNodeCheckpoint;
        QString status = statusText(node.status);
        if (node.status == manufacturing::ipw::IpwNodeStatus::Ready &&
            !replayAvailable) {
            status += QStringLiteral("，检查点缺失");
        }
        if (!node.message.empty()) {
            status += QStringLiteral("：%1").arg(
                QString::fromStdString(node.message));
        }
        operations.push_back({
            static_cast<int>(index), moveCount,
            QString::fromStdString(node.operation.id),
            QStringLiteral("#%1 · %2［%3］（%4 个移动）")
                .arg(node.operation.sequence + 1)
                .arg(QString::fromStdString(node.operation.id), status)
                .arg(moveCount),
            status, replayAvailable});
    }
    return operations;
}

// 时间线结束:取消进行中的派生并移除旧呈现,显示最后一个节点的快照。
void StockSimulationWorkspaceCoordinator::finishTimeline(
    const manufacturing::MachiningJob& job,
    const gp_Trsf& modelTransform)
{
    m_taskCoordinator.cancelDisplay(false);
    removePocketPresentation();
    displayPocketSnapshot(
        job,
        static_cast<int>(job.ipwSession().nodes().size()),
        modelTransform);
}

Handle(AIS_Shape)
StockSimulationWorkspaceCoordinator::holePresentation() const
{
    return m_presenter->holePresentation();
}

Handle(AIS_Triangulation)
StockSimulationWorkspaceCoordinator::pocketPresentation() const
{
    return m_presenter->pocketTriangulation();
}

} // namespace oap::app
