#include "ToolContextSettingsJson.h"

#include <cmath>
#include <utility>

namespace oap::app::detail {
namespace {
using Tool = toolpath::ToolpathToolContext;
struct ToolField { const char* key; double Tool::*member; };
constexpr ToolField toolFields[] = {
    {"diameter", &Tool::diameter}, {"corner_radius", &Tool::cornerRadius},
    {"cutting_length", &Tool::cuttingEdgeLength}, {"overall_length", &Tool::overallLength},
    {"shank_diameter", &Tool::shankDiameter}, {"holder_diameter", &Tool::holderDiameter},
    {"holder_length", &Tool::holderLength}, {"spindle_diameter", &Tool::spindleEnvelopeDiameter},
    {"spindle_length", &Tool::spindleEnvelopeLength}, {"spindle_speed", &Tool::spindleSpeed},
    {"feed", &Tool::horizontalFeed}, {"plunge_feed", &Tool::verticalFeed},
    {"rapid", &Tool::horizontalRapid}, {"vertical_rapid", &Tool::verticalRapid},
    {"tip_angle", &Tool::tipAngle}, {"tip_compensation", &Tool::tipCompensation},
    {"peck_depth", &Tool::peckDepth}};

bool integer(const QJsonObject& object, const char* key, int maximum, int& value)
{
    const auto item = object.value(QString::fromLatin1(key));
    const double number = item.toDouble(-1);
    if (!item.isDouble() || !std::isfinite(number) || number < 0 ||
        number > maximum || std::floor(number) != number)
        return false;
    value = static_cast<int>(number);
    return true;
}
} // namespace

QJsonObject encodeToolContext(const toolpath::ToolpathToolContext& snapshot)
{
    QJsonObject tool{{"valid", snapshot.valid}, {"number", snapshot.toolNumber},
        {"id", QString::fromStdString(snapshot.toolId)},
        {"name", QString::fromStdString(snapshot.toolName)},
        {"type", QString::fromStdString(snapshot.toolType)},
        {"coolant", static_cast<int>(snapshot.coolant)},
        {"drill_cycle", static_cast<int>(snapshot.drillCycle)}};
    for (const auto& field : toolFields)
        tool.insert(QString::fromLatin1(field.key), snapshot.*field.member);
    return tool;
}
bool decodeToolContext(const QJsonObject& object, toolpath::ToolpathToolContext& value)
{
    int number = 0, coolant = 0, cycle = 0;
    if (!integer(object, "number", 9999, number) ||
        !integer(object, "coolant", 2, coolant) ||
        !integer(object, "drill_cycle", 3, cycle) ||
        !object.value("valid").isBool() || !object.value("id").isString() ||
        !object.value("name").isString() || !object.value("type").isString())
        return false;
    Tool candidate;
    for (const auto& field : toolFields) {
        const auto item = object.value(QString::fromLatin1(field.key));
        if (!item.isDouble() || !std::isfinite(item.toDouble()) || item.toDouble() < 0)
            return false;
        candidate.*field.member = item.toDouble();
    }
    candidate.valid = object.value("valid").toBool();
    candidate.toolNumber = number;
    candidate.toolId = object.value("id").toString().toStdString();
    candidate.toolName = object.value("name").toString().toStdString();
    candidate.toolType = object.value("type").toString().toStdString();
    candidate.coolant = static_cast<toolpath::ToolpathCoolantMode>(coolant);
    candidate.drillCycle = static_cast<toolpath::ToolpathDrillCyclePreference>(cycle);
    value = std::move(candidate);
    return true;
}
} // namespace oap::app::detail
