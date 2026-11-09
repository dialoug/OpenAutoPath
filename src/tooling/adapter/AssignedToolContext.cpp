// makeToolpathToolContext 实现（oap_toolpath_adapter）。
// 把库刀具 AssignedTool 及其 T 刀槽切削默认值物化为自包含的 CAM 值
// ToolpathToolContext，见 AssignedToolContext.h。
#include "tooling/adapter/AssignedToolContext.h"

#include <cmath>

namespace oap::tooling {

bool isSupportedMillingToolType(ToolType type)
{
    return type == ToolType::EndMill || type == ToolType::BallEndMill ||
        type == ToolType::BullNose;
}

bool hasSupportedMillingGeometry(const ToolBit& tool)
{
    if (!isSupportedMillingToolType(tool.type) ||
        !toolGeometryValidationErrors(tool.type, tool.geometry).isEmpty()) return false;
    const auto& geometry = tool.geometry;
    const double radius = tool.type == ToolType::BallEndMill
        ? geometry.diameter * 0.5 : geometry.cornerRadius;
    if (tool.type == ToolType::EndMill) return radius == 0.0;
    if (tool.type == ToolType::BullNose &&
        !(radius > 0.0 && radius < geometry.diameter * 0.5)) return false;
    // The full curved cutting envelope must fit within the cutting edge.
    return geometry.cuttingEdgeLength >= radius;
}

oap::toolpath::ToolpathToolContext makeToolpathToolContext(
    const AssignedTool& assigned)
{
    oap::toolpath::ToolpathToolContext result;
    result.valid = assigned.slot.toolNumber > 0 &&
        std::isfinite(assigned.tool.geometry.diameter) &&
        assigned.tool.geometry.diameter > 0.0;
    result.toolNumber = assigned.slot.toolNumber;
    result.toolId = assigned.tool.id.toUtf8().toStdString();
    result.toolName = assigned.tool.name.toUtf8().toStdString();
    result.toolType = toolTypeId(assigned.tool.type).toUtf8().toStdString();
    result.diameter = assigned.tool.geometry.diameter;
    result.cornerRadius = assigned.tool.geometry.cornerRadius;
    if (assigned.tool.type == ToolType::BallEndMill && result.cornerRadius == 0.0)
        result.cornerRadius = result.diameter * 0.5;
    result.spindleSpeed = assigned.slot.cuttingDefaults.spindleSpeed;
    result.horizontalFeed = assigned.slot.cuttingDefaults.horizontalFeed;
    result.verticalFeed = assigned.slot.cuttingDefaults.verticalFeed;
    result.horizontalRapid = assigned.slot.cuttingDefaults.horizontalRapid;
    result.verticalRapid = assigned.slot.cuttingDefaults.verticalRapid;
    result.cuttingEdgeLength = assigned.tool.geometry.cuttingEdgeLength;
    result.overallLength = assigned.tool.geometry.overallLength;
    result.shankDiameter = assigned.tool.geometry.shankDiameter;
    result.holderDiameter = assigned.tool.geometry.holderDiameter;
    result.holderLength = assigned.tool.geometry.holderLength;
    result.spindleEnvelopeDiameter =
        assigned.tool.geometry.spindleEnvelopeDiameter;
    result.spindleEnvelopeLength =
        assigned.tool.geometry.spindleEnvelopeLength;
    result.tipAngle = assigned.tool.geometry.tipAngle;
    result.tipCompensation = assigned.tool.geometry.tipCompensation;
    result.peckDepth = assigned.slot.cuttingDefaults.peckDepth;
    switch (assigned.slot.cuttingDefaults.drillCycle) {
        case DrillCycle::Auto:
            result.drillCycle =
                oap::toolpath::ToolpathDrillCyclePreference::Auto;
            break;
        case DrillCycle::G73Peck:
            result.drillCycle =
                oap::toolpath::ToolpathDrillCyclePreference::ChipBreak;
            break;
        case DrillCycle::G83Peck:
            result.drillCycle =
                oap::toolpath::ToolpathDrillCyclePreference::Peck;
            break;
        case DrillCycle::G81Direct:
            result.drillCycle =
                oap::toolpath::ToolpathDrillCyclePreference::Direct;
            break;
    }
    if (assigned.slot.coolantDefault == CoolantMode::ThroughTool ||
        assigned.tool.throughCoolant) {
        result.coolant = oap::toolpath::ToolpathCoolantMode::ThroughTool;
    } else if (assigned.slot.coolantDefault == CoolantMode::Flood) {
        result.coolant = oap::toolpath::ToolpathCoolantMode::Flood;
    }
    return result;
}

} // namespace oap::tooling
