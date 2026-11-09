// StockSimulationSession 实现：结果清空与源配置的互转。
#include "StockSimulationSession.h"

namespace oap::app {

void StockSimulationSession::resetForModel()
{
    clearHoleResults();
    clearPocketResults();
    // 边距与导入的 STEP 路径属于用户配置，有意在模型变更后仍然保留，
    // 与原有 MainWindow 行为保持一致。
}

// 清空孔仿真结果：结果对象复位并关闭 hasHoleSimulation 门禁。
void StockSimulationSession::clearHoleResults()
{
    m_holeSimulation = toolpath::HoleStockSimulationResult{};
    m_holeComparison = toolpath::StockComparisonResult{};
    m_holeOvercutAttribution =
        toolpath::StockOvercutAttributionResult{};
    m_hasHoleSimulation = false;
}

// 清空腔仿真结果：结果、选项、临时工序与门禁一并复位。
void StockSimulationSession::clearPocketResults()
{
    m_pocketSimulation = toolpath::StockSimulationResult{};
    m_pocketComparison = toolpath::StockComparisonResult{};
    m_pocketOvercutAttribution =
        toolpath::StockOvercutAttributionResult{};
    m_hasPocketSimulation = false;
    m_pocketUsesUnifiedIpw = false;
    m_pocketIpwOptions = toolpath::StockSimulationOptions{};
    m_pocketTransientOperations.clear();
}

// 源模式映射：0=逐单元包围盒，1=模型盒，2=STEP 文件。
manufacturing::StockDefinition
StockSimulationSession::holeStockDefinition() const
{
    manufacturing::StockDefinition result;
    result.marginX = m_holeMarginX;
    result.marginY = m_holeMarginY;
    result.marginZ = m_holeMarginZ;
    if (m_holeSourceMode == 2) {
        result.sourceKind =
            manufacturing::StockSourceKind::StepModel;
        result.sourcePath = m_holeStockStepPath.toUtf8().toStdString();
    } else {
        result.sourceKind = m_holeSourceMode == 1
            ? manufacturing::StockSourceKind::ModelBox
            : manufacturing::StockSourceKind::PerUnitBoxes;
    }
    return result;
}

void StockSimulationSession::restoreHoleStockDefinition(
    const manufacturing::StockDefinition& definition)
{
    m_holeMarginX = definition.marginX;
    m_holeMarginY = definition.marginY;
    m_holeMarginZ = definition.marginZ;
    m_holeStockStepPath = QString::fromUtf8(definition.sourcePath);
    switch (definition.sourceKind) {
        case manufacturing::StockSourceKind::PerUnitBoxes:
            m_holeSourceMode = 0;
            break;
        case manufacturing::StockSourceKind::ModelBox:
            m_holeSourceMode = 1;
            break;
        case manufacturing::StockSourceKind::StepModel:
            m_holeSourceMode = 2;
            break;
    }
}

} // namespace oap::app
