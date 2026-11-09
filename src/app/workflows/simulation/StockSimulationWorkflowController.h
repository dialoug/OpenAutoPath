// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 职责：毛坯仿真的与 UI 无关的准备、安装与报告：来源校验、设置应用、
//       结果安装（陈旧门禁）、摘要/状态文本与腔工序准备。
// 协作：操作 StockSimulationSession；由 StockSimulationUseCases 调用。
// 毛坯仿真的与 UI 无关的准备、安装与报告。
#pragma once

#include <string>
#include <span>
#include <vector>

#include <QString>

#include "app/workflows/hole/HoleStockSimulationRunner.h"
#include "app/workflows/simulation/detail/IpwDisplayArtifactCache.h"
#include "app/workflows/simulation/detail/IpwSimulationRunner.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "app/workflows/simulation/StockSimulationSession.h"

namespace oap::app {

// 毛坯来源设置：sourceMode 0=逐单元包围盒，1=模型盒，2=STEP 文件。
struct StockSourceSettings {
    int sourceMode = 0;
    double marginX = 0.0;
    double marginY = 0.0;
    double marginZ = 0.0;
    QString stepPath;
};

// 孔仿真显示选择：快照步、显示视图与过切归因索引。
struct HoleStockDisplaySelection {
    int stepIndex = 0;
    int viewMode = 0;
    int overcutAttributionIndex = -1;
};

// 过切归因下拉选项。
struct HoleStockOvercutChoice {
    QString label;
    int attributionIndex = -1;
};

// 腔仿真准备结果：选中工序、来源指纹、质量档位与构造好的请求。
struct PocketSimulationPreparation {
    bool ok = false;
    QString error;
    int selectedSetup = -1;
    std::string stockFingerprint;
    manufacturing::StockDefinition stockDefinition;
    toolpath::ToolpathParams effectiveParams;
    std::vector<toolpath::PocketToolpathOperation> operations;
    std::vector<std::string> includedOperationIds;
    IpwSimulationRequest request;
    IpwSimulationQualityProfile quality;
    toolpath::StockSimulationOptions pocketOptions;
};

// 轮廓仿真准备：仿真前的几何准备与校验。
struct ProfileSimulationPreparation {
    bool ok = false;
    QString error;
    int selectedSetup = -1;
    std::string stockFingerprint;
    manufacturing::StockDefinition stockDefinition;
    std::vector<toolpath::ProfileToolpathOperation> operations;
    std::vector<std::string> includedOperationIds;
    IpwSimulationRequest request;
    IpwSimulationQualityProfile quality;
    toolpath::StockSimulationOptions ipwOptions;
};

// Ordered formal Job simulation; preserves all operation families and setups.
struct ProjectSimulationPreparation {
    bool ok = false;
    QString error;
    std::string stockFingerprint;
    manufacturing::StockDefinition stockDefinition;
    std::vector<std::string> includedOperationIds;
    IpwSimulationRequest request;
    IpwSimulationQualityProfile quality;
    toolpath::StockSimulationOptions ipwOptions;
};

// 静态控制器：把 StockSimulationSession 与仿真结果之间的读写收敛到一处；
// install* 在装入结果的同时刷新 has*Simulation 门禁（以初始毛坯非空为准）。
class StockSimulationWorkflowController {
public:
    static ProjectSimulationPreparation prepareProject(
        const manufacturing::MachiningJob& job,
        const StockSourceSettings& source,
        const QString& importedSourceFingerprint,
        int displayQuality,
        const TopoDS_Shape& initialStock,
        const TopoDS_Shape& targetShape,
        bool engravingOnly = false,
        int selectedSetup = -1);
    static void installProjectResult(
        StockSimulationSession& session,
        const manufacturing::MachiningJob& job,
        const ProjectSimulationPreparation& preparation,
        IpwSimulationRunResult& result);
    static QString projectSummary(
        const StockSimulationSession& session,
        const manufacturing::MachiningJob& job,
        const ProjectSimulationPreparation& preparation,
        const IpwSimulationRunResult& result);

    // 校验来源设置：模式范围、STEP 文件存在性、余量非负且有限。
    static bool validateSource(
        const StockSourceSettings& settings,
        QString& error);

    // 把对话框来源设置写入会话（孔/腔各自）。
    static void applyHoleSettings(
        StockSimulationSession& session,
        const StockSourceSettings& settings);
    static void applyPocketSettings(
        StockSimulationSession& session,
        const StockSourceSettings& settings);

    // 移动孔仿真结果入会话并刷新门禁；sourceFingerprint 记录本次毛坯来源指纹。
    static void installHoleResult(
        StockSimulationSession& session,
        HoleStockSimulationRunResult& result,
        const QString& sourceFingerprint);

    static std::vector<HoleStockOvercutChoice>
    holeOvercutChoices(
        const StockSimulationSession& session,
        std::span<const HolePreviewEntry> previewEntries);
    static QString holeSummary(
        const StockSimulationSession& session,
        std::span<const HolePreviewEntry> previewEntries,
        const HoleStockDisplaySelection& selection);
    static QString holeCompletionStatus(
        const StockSimulationSession& session,
        bool cancelled);

    // 准备腔仿真：筛选所选装夹面工序、收集包含的工序 ID、合成来源指纹
    // 与质量档位，构造 IpwSimulationRequest。
    static PocketSimulationPreparation preparePocket(
        const manufacturing::MachiningJob& job,
        int selectedSetup,
        const StockSourceSettings& source,
        const QString& importedSourceFingerprint,
        int displayQuality,
        const TopoDS_Shape& initialStock,
        const TopoDS_Shape& targetShape);

    static ProfileSimulationPreparation prepareProfile(
        const manufacturing::MachiningJob& job,
        std::span<const toolpath::ProfileToolpathOperation> operations,
        int selectedSetup,
        const StockSourceSettings& source,
        const QString& importedSourceFingerprint,
        int displayQuality,
        const toolpath::ToolpathParams& effectiveParams,
        const TopoDS_Shape& initialStock,
        const TopoDS_Shape& targetShape);

    static void installProfileResult(
        StockSimulationSession& session,
        const manufacturing::MachiningJob& job,
        const ProfileSimulationPreparation& preparation,
        IpwSimulationRunResult& result);

    static QString profileSummary(
        const StockSimulationSession& session,
        const manufacturing::MachiningJob& job,
        const ProfileSimulationPreparation& preparation,
        const IpwSimulationRunResult& result);

    // 安装腔仿真结果，并按检查点节点捕获逐工序操作（供逐移动临时帧）。
    static void installPocketResult(
        StockSimulationSession& session,
        const manufacturing::MachiningJob& job,
        const PocketSimulationPreparation& preparation,
        IpwSimulationRunResult& result);

    static QString pocketSummary(
        const StockSimulationSession& session,
        const manufacturing::MachiningJob& job,
        const PocketSimulationPreparation& preparation,
        const IpwSimulationRunResult& result);

    // 判定运行期操作 ID 是否对应定义 ID：完全相等或带 "-NNN" 数字后缀。
    static bool pocketRuntimeIdMatches(
        const std::string& definitionId,
        const std::string& runtimeId);
};

} // namespace oap::app
