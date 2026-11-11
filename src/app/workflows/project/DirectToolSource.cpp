#include "app/workflows/project/DirectToolSource.h"
#include <QCoreApplication>
#include "tooling/adapter/AssignedToolContext.h"

#include <cmath>

namespace oap::app {
bool makeDirectToolLibrary(const toolpath::ToolpathToolContext& tool,
    DirectToolProcess process, tooling::ToolLibrary& library, QString& error)
{
    error.clear();
    const auto type = tooling::toolTypeFromId(QString::fromStdString(tool.toolType));
    const bool supported = type == tooling::ToolType::EndMill ||
        (process == DirectToolProcess::Hole ? type == tooling::ToolType::Drill
            : tooling::isSupportedMillingToolType(type));
    const auto nonnegative = [](double value) { return std::isfinite(value) && value >= 0; };
    if (!tool.valid || tool.toolNumber <= 0 || tool.toolNumber > 9999 || !supported ||
        !std::isfinite(tool.cuttingEdgeLength) || tool.cuttingEdgeLength <= 0 ||
        !std::isfinite(tool.horizontalFeed) || tool.horizontalFeed <= 0 ||
        !std::isfinite(tool.verticalFeed) || tool.verticalFeed <= 0 ||
        !nonnegative(tool.spindleSpeed) || !nonnegative(tool.horizontalRapid) ||
        !nonnegative(tool.verticalRapid) || !nonnegative(tool.peckDepth) ||
        static_cast<int>(tool.coolant) < 0 || static_cast<int>(tool.coolant) > 2 ||
        static_cast<int>(tool.drillCycle) < 0 || static_cast<int>(tool.drillCycle) > 3) {
        error = QCoreApplication::translate("OpenAutoPath", "单刀需要有效刀型、T1–T9999、正刃长和切削进给；孔支持钻头/平底铣刀，槽腔与外轮廓支持平底铣刀、球刀、圆鼻刀。");
        return false;
    }
    tooling::ToolBit bit;
    bit.id = QString::fromStdString(tool.toolId.empty() ? "direct-process-tool" : tool.toolId);
    bit.name = tool.toolName.empty() ? QCoreApplication::translate("OpenAutoPath", "单刀")
        : QString::fromStdString(tool.toolName);
    bit.type = type;
    auto& geometry = bit.geometry;
    geometry.diameter = tool.diameter;
    geometry.cuttingEdgeLength = tool.cuttingEdgeLength;
    geometry.overallLength = tool.overallLength;
    geometry.cornerRadius = tool.cornerRadius;
    geometry.tipAngle = tool.tipAngle;
    geometry.tipCompensation = tool.tipCompensation;
    geometry.shankDiameter = tool.shankDiameter;
    geometry.holderDiameter = tool.holderDiameter;
    geometry.holderLength = tool.holderLength;
    geometry.spindleEnvelopeDiameter = tool.spindleEnvelopeDiameter;
    geometry.spindleEnvelopeLength = tool.spindleEnvelopeLength;
    const auto geometryErrors = tooling::toolGeometryValidationErrors(type, geometry);
    if (!geometryErrors.isEmpty() || (type != tooling::ToolType::Drill &&
            !tooling::hasSupportedMillingGeometry(bit))) {
        error = geometryErrors.isEmpty() ? QCoreApplication::translate("OpenAutoPath", "刀具几何无效：平底刀圆角为零，球刀半径为直径的一半，圆鼻刀圆角须大于零且小于刀具半径，刃长须覆盖圆角高度。")
                                        : geometryErrors.join('\n');
        return false;
    }
    if (type == tooling::ToolType::BallEndMill) geometry.cornerRadius = geometry.diameter * 0.5;
    tooling::LibraryToolSlot slot;
    slot.toolNumber = tool.toolNumber;
    slot.toolId = bit.id;
    auto& cutting = slot.cuttingDefaults;
    cutting.spindleSpeed = tool.spindleSpeed;
    cutting.horizontalFeed = tool.horizontalFeed;
    cutting.verticalFeed = tool.verticalFeed;
    cutting.horizontalRapid = tool.horizontalRapid;
    cutting.verticalRapid = tool.verticalRapid;
    cutting.peckDepth = tool.peckDepth;
    switch (tool.drillCycle) {
    case toolpath::ToolpathDrillCyclePreference::Auto: cutting.drillCycle = tooling::DrillCycle::Auto; break;
    case toolpath::ToolpathDrillCyclePreference::Direct: cutting.drillCycle = tooling::DrillCycle::G81Direct; break;
    case toolpath::ToolpathDrillCyclePreference::Peck: cutting.drillCycle = tooling::DrillCycle::G83Peck; break;
    case toolpath::ToolpathDrillCyclePreference::ChipBreak: cutting.drillCycle = tooling::DrillCycle::G73Peck; break;
    }
    slot.coolantDefault = static_cast<tooling::CoolantMode>(tool.coolant);
    tooling::ToolLibrary candidate(QStringLiteral("internal-direct-tool"), QCoreApplication::translate("OpenAutoPath", "单刀"));
    if (!candidate.upsertTool(std::move(bit), &error) ||
        !candidate.assignToolToSlot(std::move(slot), &error) || !candidate.isValid(&error)) return false;
    library = std::move(candidate);
    return true;
}
} // namespace oap::app
