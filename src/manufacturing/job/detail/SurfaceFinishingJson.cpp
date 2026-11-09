#include "SurfaceFinishingJson.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "tooling/library/ToolLibrary.h"

namespace oap::manufacturing::detail {
namespace {
using P = toolpath::SurfaceFinishingParams;
struct Dimension { const char* name; double P::* value; };
constexpr Dimension dimensions[] = {
    {"diameter", &P::diameter}, {"cutting_length", &P::cuttingLength},
    {"corner_radius", &P::cornerRadius}, {"step_over", &P::stepOver},
    {"sample_spacing", &P::sampleSpacing}, {"mesh_deflection", &P::meshDeflection},
    {"allowance", &P::allowance}, {"safe_clearance", &P::safeClearance},
    {"feed", &P::feed}, {"plunge_feed", &P::plungeFeed},
    {"max_step_down", &P::maxStepDown}, {"roughing_allowance", &P::roughingAllowance}};
bool layeredDimension(const Dimension& dimension)
{
    return dimension.value == &P::maxStepDown || dimension.value == &P::roughingAllowance;
}
bool integer(const QJsonValue& value, double low, double high)
{
    return value.isDouble() && std::isfinite(value.toDouble()) &&
        std::floor(value.toDouble()) == value.toDouble() &&
        value.toDouble() >= low && value.toDouble() <= high;
}
}

bool validFinishingParameters(const P& p)
{
    for (const auto& dimension : dimensions) {
        const double value = p.*(dimension.value);
        const bool nonNegative = dimension.value == &P::cornerRadius || dimension.value == &P::allowance ||
            dimension.value == &P::roughingAllowance;
        if (!std::isfinite(value) || value < (nonNegative ? 0.0 : 1e-7) || value > 1e9)
            return false;
    }
    if ((p.mode != toolpath::EngravingMachiningMode::FinishOnly &&
         p.mode != toolpath::EngravingMachiningMode::StockRoughingAndFinishing) ||
        p.roughingAllowance > p.maxStepDown ||
        (p.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing && p.maxStepDown > p.cuttingLength) ||
        p.maxMoves < 4 || p.maxMoves > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        p.stepOver > p.diameter || p.maxSamples < 4 || p.maxTriangles == 0 ||
        p.maxSamples > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        p.maxTriangles > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        return false;
    switch (p.cutterKind) {
    case toolpath::FinishingCutterKind::Ball: return p.cuttingLength >= p.diameter / 2;
    case toolpath::FinishingCutterKind::Flat: return true;
    case toolpath::FinishingCutterKind::Bull:
        return p.cornerRadius > 0 && p.cornerRadius < p.diameter / 2 && p.cuttingLength >= p.cornerRadius;
    }
    return false;
}

bool finishingCutterMatchesTool(toolpath::FinishingCutterKind kind, const toolpath::ToolpathToolContext& tool)
{
    QString id = QString::fromStdString(tool.toolType).trimmed().toLower();
    // Preserve the initial standalone dialog's historical type spelling.
    if (id == "ballendmill") id = "ball_end_mill";
    if (id == "endmill") id = "end_mill";
    if (id == "bullnose") id = "bull_nose";
    const auto type = tooling::toolTypeFromId(id);
    return (kind == toolpath::FinishingCutterKind::Ball && type == tooling::ToolType::BallEndMill) ||
        (kind == toolpath::FinishingCutterKind::Flat && type == tooling::ToolType::EndMill) ||
        (kind == toolpath::FinishingCutterKind::Bull && type == tooling::ToolType::BullNose);
}

QJsonArray finishingOperationsToJson(const std::vector<SurfaceFinishingOperationDefinition>& operations)
{
    QJsonArray result;
    for (const auto& op : operations) {
        QJsonObject parameters;
        for (const auto& dimension : dimensions)
            parameters.insert(QString::fromLatin1(dimension.name), op.parameters.*(dimension.value));
        parameters.insert("cutter_kind", static_cast<int>(op.parameters.cutterKind));
        parameters.insert("mode", static_cast<int>(op.parameters.mode));
        parameters.insert("max_moves", static_cast<double>(op.parameters.maxMoves));
        parameters.insert("max_samples", static_cast<double>(op.parameters.maxSamples));
        parameters.insert("max_triangles", static_cast<double>(op.parameters.maxTriangles));
        QJsonArray units;
        for (const auto& id : op.target.unitIds)
            units.append(QString::fromStdString(id));
        result.append(QJsonObject{
            {"id", QString::fromStdString(op.id)}, {"setup_id", QString::fromStdString(op.setupId)},
            {"tool_controller_id", QString::fromStdString(op.toolControllerId)},
            {"algorithm_id", QString::fromStdString(op.algorithmId)}, {"parameters", parameters},
            {"target", QJsonObject{{"unit_ids", units}, {"geometry_fingerprint", QString::fromStdString(op.target.geometryFingerprint)}}},
            {"enabled", op.enabled}, {"sequence", op.sequence}, {"state", static_cast<int>(op.state)},
            {"input_fingerprint", QString::fromStdString(op.inputFingerprint)}});
    }
    return result;
}

std::vector<SurfaceFinishingOperationDefinition> finishingOperationsFromJson(const QJsonArray& array)
{
    // The shape and integral bounds are checked before conversion.
    std::vector<SurfaceFinishingOperationDefinition> result;
    for (const auto& value : array) {
        const auto object = value.toObject();
        SurfaceFinishingOperationDefinition op;
        op.id = object.value("id").toString().toStdString();
        op.setupId = object.value("setup_id").toString().toStdString();
        op.toolControllerId = object.value("tool_controller_id").toString().toStdString();
        op.algorithmId = object.value("algorithm_id").toString().toStdString();
        op.enabled = object.value("enabled").toBool();
        op.sequence = object.value("sequence").toInt();
        op.state = static_cast<JobOperationState>(object.value("state").toInt());
        op.inputFingerprint = object.value("input_fingerprint").toString().toStdString();
        const auto target = object.value("target").toObject();
        op.target.geometryFingerprint = target.value("geometry_fingerprint").toString().toStdString();
        for (const auto& unit : target.value("unit_ids").toArray())
            op.target.unitIds.push_back(unit.toString().toStdString());
        const auto p = object.value("parameters").toObject();
        for (const auto& dimension : dimensions) {
            const auto dimensionValue = p.value(QString::fromLatin1(dimension.name));
            if (!dimensionValue.isUndefined()) op.parameters.*(dimension.value) = dimensionValue.toDouble();
        }
        op.parameters.mode = static_cast<toolpath::EngravingMachiningMode>(p.value("mode").toInt(0));
        if (p.contains("max_moves"))
            op.parameters.maxMoves = static_cast<std::size_t>(p.value("max_moves").toDouble());
        op.parameters.cutterKind = static_cast<toolpath::FinishingCutterKind>(p.value("cutter_kind").toInt());
        op.parameters.maxSamples = static_cast<std::size_t>(p.value("max_samples").toDouble());
        op.parameters.maxTriangles = static_cast<std::size_t>(p.value("max_triangles").toDouble());
        result.push_back(std::move(op));
    }
    return result;
}

QString validateFinishingJson(const QJsonObject& job, bool required)
{
    const auto value = job.value("surface_finishing_operations");
    if (!required && value.isUndefined()) return {};
    if (!value.isArray()) return "surface_finishing_operations must be an array";
    if (!required && !value.toArray().isEmpty())
        return "OpenCAMLib engraving requires schema version 10";
    for (const auto& entry : value.toArray()) {
        if (!entry.isObject()) return "engraving operation must be an object";
        const auto op = entry.toObject();
        for (const auto* key : {"id", "setup_id", "tool_controller_id", "algorithm_id", "input_fingerprint"})
            if (!op.value(QLatin1String(key)).isString() || op.value(QLatin1String(key)).toString().isEmpty())
                return "engraving operation has an invalid reference";
        if (!op.value("enabled").isBool() || !integer(op.value("sequence"), 0, std::numeric_limits<int>::max()) ||
            !integer(op.value("state"), 0, static_cast<int>(JobOperationState::NeedsReview)))
            return "engraving operation has invalid state/order";
        if (!op.value("parameters").isObject() || !op.value("target").isObject())
            return "engraving operation is missing parameters/target";
        const auto p = op.value("parameters").toObject();
        const bool requireLayeredFields = job.value("schema_version").toInt() >= 12;
        for (const auto& dimension : dimensions) {
            const auto dimensionValue = p.value(QLatin1String(dimension.name));
            if (layeredDimension(dimension) && dimensionValue.isUndefined() && !requireLayeredFields) continue;
            if (!dimensionValue.isDouble()) return "engraving parameter must be a number";
        }
        if ((requireLayeredFields || p.contains("mode")) && !integer(p.value("mode"), 0, 1))
            return "engraving machining mode is invalid";
        if (!requireLayeredFields && p.value("mode").toInt() != 0)
            return "layered engraving requires schema version 12";
        if ((requireLayeredFields || p.contains("max_moves")) &&
            !integer(p.value("max_moves"), 4, std::numeric_limits<int>::max()))
            return "engraving move budget is invalid";
        if (!integer(p.value("cutter_kind"), 0, 2) || !integer(p.value("max_samples"), 4, std::numeric_limits<int>::max()) ||
            !integer(p.value("max_triangles"), 1, std::numeric_limits<int>::max()))
            return "engraving cutter or budget is invalid";
        const auto target = op.value("target").toObject();
        if (!target.value("geometry_fingerprint").isString() || !target.value("unit_ids").isArray() || target.value("unit_ids").toArray().isEmpty())
            return "engraving target is incomplete";
        for (const auto& unit : target.value("unit_ids").toArray())
            if (!unit.isString() || unit.toString().isEmpty()) return "engraving unit id is invalid";
    }
    return {};
}
} // namespace oap::manufacturing::detail
