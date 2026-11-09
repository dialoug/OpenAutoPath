// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 职责：不依赖 MainWindow 状态的孔/腔毛坯仿真执行用例：
//       只依赖交互端口与协调器，由桌面工作区/对话框驱动。
// 不依赖 MainWindow 状态的桌面毛坯仿真执行用例。
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <span>

#include "app/workflows/pocket/PocketCamSimulatorTypes.h"
#include "app/workflows/simulation/StockSimulationCoordinator.h"
#include "app/workflows/simulation/StockSimulationViewModels.h"
#include "app/workflows/shared/WorkflowInteraction.h"

namespace oap::app {

// 回调：按来源设置构造初始毛坯，输出块数与错误文本。
using HoleInitialStockBuilder = std::function<bool(
    const StockSourceSettings& settings,
    TopoDS_Shape& stock,
    int& blockCount,
    QString& error)>;
using StockTargetBuilder = std::function<bool(
    TopoDS_Shape& target,
    int& solidCount,
    QString& error)>;
using HoleStockRunner = std::function<HoleStockSimulationRunResult(
    HoleStockSimulationRequest request,
    std::shared_ptr<std::atomic_bool> cancellation)>;
using IpwStockRunner = std::function<IpwSimulationRunResult(
    manufacturing::MachiningJob& job,
    IpwSimulationRequest request,
    std::shared_ptr<std::atomic_bool> cancellation)>;

// 孔毛坯仿真用例输入：交互端口、来源设置、会话/协调器/Job 指针
// 与各阶段回调（构造毛坯、构造目标、运行、关闭检测）。
struct HoleStockSimulationUseCaseRequest {
    WorkflowInteraction interaction;
    StockSourceSettings settings;
    StockSimulationSession* session = nullptr;
    StockSimulationCoordinator* coordinator = nullptr;
    manufacturing::MachiningJob* job = nullptr;
    HoleInitialStockBuilder buildInitialStock;
    StockTargetBuilder buildTarget;
    std::function<void()> beforeRun;
    std::function<bool()> closeRequested;
    HoleStockRunner runner;
};

// 孔毛坯仿真用例：执行校验→构造→运行→安装结果的主流程。
class HoleStockSimulationUseCase {
public:
    static HoleStockRunOutcome execute(
        const HoleStockSimulationUseCaseRequest& request);
};

// 腔仿真失败分类（None 表示成功）。
enum class PocketStockSimulationFailure {
    None,
    InvalidRequest,
    InvalidSource,
    StockLoadFailed,
    PreparationFailed,
    Cancelled,
    EmptyResult,
    CloseRequested,
};

// 腔仿真用例输入：对话框状态、会话/协调器/Job、预览工序、
// 生成的毛坯与目标实体及运行回调。
struct PocketStockSimulationUseCaseRequest {
    WorkflowInteraction interaction;
    PocketStockSimulationDialogState settings;
    StockSimulationSession* session = nullptr;
    StockSimulationCoordinator* coordinator = nullptr;
    manufacturing::MachiningJob* job = nullptr;
    TopoDS_Shape generatedInitialStock;
    TopoDS_Shape targetShape;
    std::optional<std::array<double, 6>> exactTargetBounds;
    bool persistSettings = true;
    std::function<void()> beforeRun;
    std::function<bool()> closeRequested;
    IpwStockRunner runner;
};

// 腔仿真用例输出：失败分类、错误/警告/摘要与相机输入。
struct PocketStockSimulationUseCaseResult {
    PocketStockSimulationFailure failure =
        PocketStockSimulationFailure::InvalidRequest;
    QString error;
    QString warning;
    QString summary;
    int targetSolidCount = 0;
    PocketSimulationPreparation preparation;
    IpwSimulationRunResult runResult;
    PocketCamSimulatorInput camInput;

    bool ok() const noexcept
    {
        return failure == PocketStockSimulationFailure::None;
    }
};

// 腔毛坯仿真用例：构建对话框状态、执行统一 IPW 并组装相机输入。
class PocketStockSimulationUseCase {
public:
    static PocketStockSimulationDialogState buildDialogState(
        const StockSimulationSession& session,
        std::span<const toolpath::PocketToolpathOperation> operations,
        int displayQuality,
        bool triangulationEnabled,
        std::size_t displayCacheBudgetBytes,
        const QString& browseDirectory);

    static void persistDisplaySettings(
        const PocketStockSimulationDialogState& settings);

    static PocketStockSimulationUseCaseResult execute(
        const PocketStockSimulationUseCaseRequest& request);

    static std::optional<PocketCamVisualAcceptanceView>
    selectVisualAcceptanceView(
        const std::vector<toolpath::PocketToolpathOperation>& operations,
        double toolDiameter);
};

// 轮廓毛坯仿真用例请求：区域、毛坯与仿真选项。
struct ProfileStockSimulationUseCaseRequest {
    WorkflowInteraction interaction;
    PocketStockSimulationDialogState settings;
    StockSimulationSession* session = nullptr;
    StockSimulationCoordinator* coordinator = nullptr;
    manufacturing::MachiningJob* job = nullptr;
    const std::vector<toolpath::ProfileToolpathOperation>* operations =
        nullptr;
    const toolpath::ToolpathParams* effectiveParams = nullptr;
    TopoDS_Shape generatedInitialStock;
    TopoDS_Shape targetShape;
    std::optional<std::array<double, 6>> exactTargetBounds;
    bool persistSettings = true;
    std::function<void()> beforeRun;
    std::function<bool()> closeRequested;
    IpwStockRunner runner;
};

// 轮廓毛坯仿真用例结果：IPW 结果、比较数据与统计。
struct ProfileStockSimulationUseCaseResult {
    PocketStockSimulationFailure failure =
        PocketStockSimulationFailure::InvalidRequest;
    QString error;
    QString warning;
    QString summary;
    ProfileSimulationPreparation preparation;
    IpwSimulationRunResult runResult;
    // 与槽腔仿真使用同一份 Multi-Dexel 检查点和逐移动重放输入；
    // operations 是 Profile 运行时语义到材料去除内核的只读适配结果。
    PocketCamSimulatorInput camInput;

    bool ok() const noexcept
    {
        return failure == PocketStockSimulationFailure::None;
    }
};

// 轮廓毛坯仿真用例：加载、执行与结果安装。
class ProfileStockSimulationUseCase {
public:
    static PocketStockSimulationDialogState buildDialogState(
        const StockSimulationSession& session,
        std::span<const toolpath::ProfileToolpathOperation> operations,
        int displayQuality,
        bool triangulationEnabled,
        std::size_t displayCacheBudgetBytes,
        const QString& browseDirectory);

    static ProfileStockSimulationUseCaseResult execute(
        const ProfileStockSimulationUseCaseRequest& request);
};

// Complete ordered Job simulation, including roughing and surface finishing.
struct ProjectStockSimulationUseCaseRequest {
    // Simulate the global prefix through the last selected engraving operation,
    // then animate only selected engraving operations from their parent IPW.
    bool engravingOnly = false;
    WorkflowInteraction interaction;
    PocketStockSimulationDialogState settings;
    StockSimulationSession* session = nullptr;
    StockSimulationCoordinator* coordinator = nullptr;
    manufacturing::MachiningJob* job = nullptr;
    TopoDS_Shape generatedInitialStock;
    TopoDS_Shape targetShape;
    std::optional<std::array<double, 6>> exactTargetBounds;
    bool persistSettings = true;
    std::function<void()> beforeRun;
    std::function<bool()> closeRequested;
    IpwStockRunner runner;
};

struct ProjectStockSimulationUseCaseResult {
    PocketStockSimulationFailure failure = PocketStockSimulationFailure::InvalidRequest;
    QString error;
    QString warning;
    QString summary;
    ProjectSimulationPreparation preparation;
    IpwSimulationRunResult runResult;
    PocketCamSimulatorInput camInput;
    QString playbackError;
    bool ok() const noexcept { return failure == PocketStockSimulationFailure::None; }
};

class ProjectStockSimulationUseCase {
public:
    static ProjectStockSimulationUseCaseResult execute(const ProjectStockSimulationUseCaseRequest& request);
};

} // namespace oap::app
