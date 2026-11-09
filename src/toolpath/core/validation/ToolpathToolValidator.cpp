// 对比刀库实际有效刃长与工序几何需求；未知值只告警，已知但不足时阻止导出。
#include "toolpath/core/validation/ToolpathToolValidator.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace oap::toolpath {

// 对比刀具实际有效刃长与工序几何需求：未知值只告警，已知但不足时报错。
void validateCuttingEdgeRequirement(
    const ToolpathToolContext* operationTool,
    double fallbackCuttingEdgeLength,
    double minimumCuttingEdgeLength,
    const std::string& operationId,
    ToolpathValidationReport& report)
{
    if (!std::isfinite(minimumCuttingEdgeLength) ||
        minimumCuttingEdgeLength <= 0.0) {
        return;
    }

    // 已绑定刀具时不能借用程序级单刀参数，否则多刀程序会把另一把刀的刃长
    // 错当成当前刀具的真实几何。
    const double actual = operationTool
        ? operationTool->cuttingEdgeLength
        : fallbackCuttingEdgeLength;
    if (!std::isfinite(actual) || actual <= 0.0) {
        std::ostringstream message;
        message << std::fixed << std::setprecision(3)
                << "cutting-edge length is unknown; operation requires at least "
                << minimumCuttingEdgeLength
                << " mm. Preview and stock simulation may use the derived requirement, "
                   "but full tool-assembly reach is not verified";
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "unknown_cutting_edge_length", message.str(), operationId);
        return;
    }

    constexpr double tolerance = 1.0e-6;
    if (actual + tolerance < minimumCuttingEdgeLength) {
        std::ostringstream message;
        message << std::fixed << std::setprecision(3)
                << "cutting-edge length " << actual
                << " mm is shorter than the required "
                << minimumCuttingEdgeLength << " mm";
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error,
            "insufficient_cutting_edge_length", message.str(), operationId);
    }
}

// 对比刀尖至刀柄端已知总长与工序轴向可达需求：未知值只告警，
// 已知但不足时报错。
void validateToolReachRequirement(
    const ToolpathToolContext* operationTool,
    double fallbackOverallLength,
    double minimumToolReach,
    const std::string& operationId,
    ToolpathValidationReport& report)
{
    if (!std::isfinite(minimumToolReach) || minimumToolReach <= 0.0)
        return;

    const double actual = operationTool
        ? operationTool->overallLength
        : fallbackOverallLength;
    if (!std::isfinite(actual) || actual <= 0.0) {
        std::ostringstream message;
        message << std::fixed << std::setprecision(3)
                << "tool reach is unknown; operation requires at least "
                << minimumToolReach
                << " mm. Holder and shank reach are not fully verified";
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "unknown_tool_reach", message.str(), operationId);
        return;
    }

    constexpr double tolerance = 1.0e-6;
    if (actual + tolerance < minimumToolReach) {
        std::ostringstream message;
        message << std::fixed << std::setprecision(3)
                << "tool reach " << actual
                << " mm is shorter than the required "
                << minimumToolReach << " mm";
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error,
            "insufficient_tool_reach", message.str(), operationId);
    }
}

} // namespace oap::toolpath
