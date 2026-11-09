// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 职责：保存单个模型的孔/腔两套仿真结果、源配置与余量设置。
// 生命周期：由仿真工作区持有；模型切换时 resetForModel 清空结果。
// 陈旧结果门禁：hasHoleSimulation/hasPocketSimulation 与源指纹联合判定，
//       源配置变化后需重新仿真。
// 每个模型的毛坯/IPW 结果与源配置。
#pragma once

#include <memory>
#include <vector>

#include <QString>

#include "manufacturing/job/MachiningJob.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleStockSimulator.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/simulation/StockComparison.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::app {

// 孔/腔仿真的共享会话：各自独立保存仿真结果（StockSimulationResult）、
// 比较结果、过切归因、源模式（生成包围盒/模型盒/STEP）、余量与指纹。
class StockSimulationSession {
public:
    // 模型变更时清空两套仿真结果（保留边距与 STEP 路径等用户配置）。
    void resetForModel();
    void clearHoleResults();
    void clearPocketResults();
    // 由会话配置合成 machining::StockDefinition（含来源种类与路径）。
    manufacturing::StockDefinition holeStockDefinition() const;
    // 从 StockDefinition 恢复会话配置（供对话框回填）。
    void restoreHoleStockDefinition(
        const manufacturing::StockDefinition& definition);

    toolpath::HoleStockSimulationResult& holeSimulation() noexcept
    {
        return m_holeSimulation;
    }
    const toolpath::HoleStockSimulationResult& holeSimulation() const noexcept
    {
        return m_holeSimulation;
    }
    toolpath::StockComparisonResult& holeComparison() noexcept
    {
        return m_holeComparison;
    }
    const toolpath::StockComparisonResult& holeComparison() const noexcept
    {
        return m_holeComparison;
    }
    toolpath::StockOvercutAttributionResult& holeOvercutAttribution() noexcept
    {
        return m_holeOvercutAttribution;
    }
    const toolpath::StockOvercutAttributionResult&
    holeOvercutAttribution() const noexcept
    {
        return m_holeOvercutAttribution;
    }
    bool& hasHoleSimulation() noexcept { return m_hasHoleSimulation; }
    bool hasHoleSimulation() const noexcept { return m_hasHoleSimulation; }
    double& holeMarginX() noexcept { return m_holeMarginX; }
    double holeMarginX() const noexcept { return m_holeMarginX; }
    double& holeMarginY() noexcept { return m_holeMarginY; }
    double holeMarginY() const noexcept { return m_holeMarginY; }
    double& holeMarginZ() noexcept { return m_holeMarginZ; }
    double holeMarginZ() const noexcept { return m_holeMarginZ; }
    int& holeSourceMode() noexcept { return m_holeSourceMode; }
    int holeSourceMode() const noexcept { return m_holeSourceMode; }
    QString& holeStockStepPath() noexcept { return m_holeStockStepPath; }
    const QString& holeStockStepPath() const noexcept
    {
        return m_holeStockStepPath;
    }
    QString& holeSourceFingerprint() noexcept
    {
        return m_holeSourceFingerprint;
    }
    const QString& holeSourceFingerprint() const noexcept
    {
        return m_holeSourceFingerprint;
    }

    toolpath::StockSimulationResult& pocketSimulation() noexcept
    {
        return m_pocketSimulation;
    }
    const toolpath::StockSimulationResult& pocketSimulation() const noexcept
    {
        return m_pocketSimulation;
    }
    toolpath::StockComparisonResult& pocketComparison() noexcept
    {
        return m_pocketComparison;
    }
    const toolpath::StockComparisonResult& pocketComparison() const noexcept
    {
        return m_pocketComparison;
    }
    toolpath::StockOvercutAttributionResult& pocketOvercutAttribution() noexcept
    {
        return m_pocketOvercutAttribution;
    }
    const toolpath::StockOvercutAttributionResult&
    pocketOvercutAttribution() const noexcept
    {
        return m_pocketOvercutAttribution;
    }
    bool& hasPocketSimulation() noexcept { return m_hasPocketSimulation; }
    bool hasPocketSimulation() const noexcept { return m_hasPocketSimulation; }
    bool& pocketUsesUnifiedIpw() noexcept { return m_pocketUsesUnifiedIpw; }
    bool pocketUsesUnifiedIpw() const noexcept { return m_pocketUsesUnifiedIpw; }
    toolpath::StockSimulationOptions& pocketIpwOptions() noexcept
    {
        return m_pocketIpwOptions;
    }
    const toolpath::StockSimulationOptions& pocketIpwOptions() const noexcept
    {
        return m_pocketIpwOptions;
    }
    std::vector<std::shared_ptr<const std::vector<
        toolpath::PocketToolpathOperation>>>&
    pocketTransientOperations() noexcept
    {
        return m_pocketTransientOperations;
    }
    const std::vector<std::shared_ptr<const std::vector<
        toolpath::PocketToolpathOperation>>>&
    pocketTransientOperations() const noexcept
    {
        return m_pocketTransientOperations;
    }
    double& pocketMarginX() noexcept { return m_pocketMarginX; }
    double pocketMarginX() const noexcept { return m_pocketMarginX; }
    double& pocketMarginY() noexcept { return m_pocketMarginY; }
    double pocketMarginY() const noexcept { return m_pocketMarginY; }
    double& pocketMarginZ() noexcept { return m_pocketMarginZ; }
    double pocketMarginZ() const noexcept { return m_pocketMarginZ; }
    int& pocketSourceMode() noexcept { return m_pocketSourceMode; }
    int pocketSourceMode() const noexcept { return m_pocketSourceMode; }
    QString& pocketStockStepPath() noexcept { return m_pocketStockStepPath; }
    const QString& pocketStockStepPath() const noexcept
    {
        return m_pocketStockStepPath;
    }
    QString& pocketSourceFingerprint() noexcept
    {
        return m_pocketSourceFingerprint;
    }
    const QString& pocketSourceFingerprint() const noexcept
    {
        return m_pocketSourceFingerprint;
    }

private:
    toolpath::HoleStockSimulationResult m_holeSimulation;
    toolpath::StockComparisonResult m_holeComparison;
    toolpath::StockOvercutAttributionResult m_holeOvercutAttribution;
    bool m_hasHoleSimulation = false;
    double m_holeMarginX = 0.0;
    double m_holeMarginY = 0.0;
    double m_holeMarginZ = 0.0;
    int m_holeSourceMode = 0;
    QString m_holeStockStepPath;
    QString m_holeSourceFingerprint;

    toolpath::StockSimulationResult m_pocketSimulation;
    toolpath::StockComparisonResult m_pocketComparison;
    toolpath::StockOvercutAttributionResult m_pocketOvercutAttribution;
    bool m_hasPocketSimulation = false;
    bool m_pocketUsesUnifiedIpw = false;
    toolpath::StockSimulationOptions m_pocketIpwOptions;
    std::vector<std::shared_ptr<const std::vector<
        toolpath::PocketToolpathOperation>>> m_pocketTransientOperations;
    double m_pocketMarginX = 0.0;
    double m_pocketMarginY = 0.0;
    double m_pocketMarginZ = 0.0;
    int m_pocketSourceMode = 0;
    QString m_pocketStockStepPath;
    QString m_pocketSourceFingerprint;
};

} // namespace oap::app
