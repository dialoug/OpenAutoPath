#include "app/workflows/project/EngravingSettingsStorage.h"
#include <QCoreApplication>

#include "app/workflows/project/detail/ToolContextSettingsJson.h"

#include <cmath>
#include <limits>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QSettings>

namespace oap::app {
namespace {
constexpr auto kKey = "toolpath/engraving_parameters";
using Params = toolpath::SurfaceFinishingParams;
struct ParameterField { const char* key; double Params::*member; };
constexpr ParameterField parameterFields[] = {
    {"diameter", &Params::diameter}, {"cutting_length", &Params::cuttingLength},
    {"corner_radius", &Params::cornerRadius}, {"step_over", &Params::stepOver},
    {"sample_spacing", &Params::sampleSpacing}, {"mesh_deflection", &Params::meshDeflection},
    {"allowance", &Params::allowance}, {"safe_clearance", &Params::safeClearance},
    {"feed", &Params::feed}, {"plunge_feed", &Params::plungeFeed}};

QJsonObject encode(const EngravingSettings& value)
{
    QJsonObject params;
    for (const auto& field : parameterFields)
        params.insert(QString::fromLatin1(field.key), value.parameters.*field.member);
    params.insert("cutter_kind", static_cast<int>(value.parameters.cutterKind));
    params.insert("max_samples", static_cast<double>(value.parameters.maxSamples));
    params.insert("max_triangles", static_cast<double>(value.parameters.maxTriangles));
    params.insert("machining_mode", static_cast<int>(value.parameters.mode));
    params.insert("max_step_down", value.parameters.maxStepDown);
    params.insert("roughing_allowance", value.parameters.roughingAllowance);
    params.insert("max_moves", static_cast<double>(value.parameters.maxMoves));
    const auto tool = detail::encodeToolContext(value.manualTool);
    QJsonArray faces;
    for (const auto face : value.projectFaces) faces.append(static_cast<int>(face));
    return {{"version", 1}, {"scope", static_cast<int>(value.scope)}, {"project_faces", faces},
        {"tool_mode", static_cast<int>(value.toolMode)},
        {"library_id", QString::fromStdString(value.libraryId)},
        {"parameters", params}, {"single_tool", tool}};
}

bool readNumber(const QJsonObject& object, const char* key, double& destination)
{
    const auto value = object.value(QString::fromLatin1(key));
    if (!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble() < 0)
        return false;
    destination = value.toDouble();
    return true;
}

bool readEnum(const QJsonObject& object, const char* key, int maximum, int& destination)
{
    double value = 0;
    if (!readNumber(object, key, value) || value > maximum || value != std::floor(value))
        return false;
    destination = static_cast<int>(value);
    return true;
}

bool decode(const QJsonObject& object, EngravingSettings& value)
{
    if (object.value("version").toInt() != 1 || !object.value("parameters").isObject() ||
        !object.value("single_tool").isObject() || !object.value("library_id").isString())
        return false;
    int scope = 0, mode = 0, cutter = 0;
    const auto params = object.value("parameters").toObject();
    const auto tool = object.value("single_tool").toObject();
    if (!readEnum(object, "scope", static_cast<int>(EngravingSetupScope::NegativeZ), scope) ||
        !readEnum(object, "tool_mode", 1, mode) ||
        !readEnum(params, "cutter_kind", 2, cutter) ||
        !detail::decodeToolContext(tool, value.manualTool))
        return false;
    for (const auto& field : parameterFields)
        if (!readNumber(params, field.key, value.parameters.*field.member)) return false;
    // Old preferences have none of these fields and retain FinishOnly defaults.
    int machiningMode = static_cast<int>(toolpath::EngravingMachiningMode::FinishOnly);
    int maxMoves = static_cast<int>(value.parameters.maxMoves);
    if ((params.contains("machining_mode") && !readEnum(params, "machining_mode", 1, machiningMode)) ||
        (params.contains("max_step_down") && !readNumber(params, "max_step_down", value.parameters.maxStepDown)) ||
        (params.contains("roughing_allowance") && !readNumber(params, "roughing_allowance", value.parameters.roughingAllowance)) ||
        (params.contains("max_moves") && !readEnum(params, "max_moves", std::numeric_limits<int>::max(), maxMoves)) ||
        value.parameters.maxStepDown < 1e-7 || value.parameters.maxStepDown > 1e9 ||
        value.parameters.roughingAllowance > 1e9 || maxMoves < 4)
        return false;
    value.parameters.mode = static_cast<toolpath::EngravingMachiningMode>(machiningMode);
    value.parameters.maxMoves = static_cast<std::size_t>(maxMoves);
    if (value.parameters.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing &&
        (value.parameters.roughingAllowance > value.parameters.maxStepDown ||
         (mode == static_cast<int>(EngravingToolMode::SingleTool) && value.manualTool.valid &&
          value.parameters.maxStepDown > value.manualTool.cuttingEdgeLength)))
        return false;
    int samples = 0, triangles = 0;
    if (!readEnum(params, "max_samples", 100000000, samples) || samples == 0 ||
        !readEnum(params, "max_triangles", 100000000, triangles) || triangles == 0 ||
        value.parameters.stepOver <= 0 || value.parameters.sampleSpacing <= 0 ||
        value.parameters.meshDeflection <= 0 || value.parameters.safeClearance <= 0)
        return false;
    value.scope = static_cast<EngravingSetupScope>(scope);
    if (!object.value("project_faces").isArray()) return false;
    for (const auto face : object.value("project_faces").toArray()) {
        if (!face.isDouble() || face.toDouble() < 0 || face.toDouble() > 5 ||
            face.toDouble() != std::floor(face.toDouble())) return false;
        value.projectFaces.push_back(static_cast<MoldSetupFace>(face.toInt()));
    }
    if (value.scope == EngravingSetupScope::PreserveProjectFaces && value.projectFaces.empty()) return false;
    value.toolMode = static_cast<EngravingToolMode>(mode);
    value.parameters.cutterKind = static_cast<toolpath::FinishingCutterKind>(cutter);
    value.parameters.maxSamples = static_cast<std::size_t>(samples);
    value.parameters.maxTriangles = static_cast<std::size_t>(triangles);
    value.libraryId = object.value("library_id").toString().toStdString();
    return true;
}
} // namespace

bool EngravingSettingsStorage::load(QSettings& settings, EngravingSettings& value, QString* error)
{
    if (error) error->clear();
    if (!settings.contains(kKey)) return true;
    const auto document = QJsonDocument::fromJson(settings.value(kKey).toByteArray());
    EngravingSettings candidate;
    if (settings.status() != QSettings::NoError || !document.isObject() || !decode(document.object(), candidate)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "雕刻参数配置无效，已保留当前参数。");
        return false;
    }
    value = std::move(candidate);
    return true;
}

bool EngravingSettingsStorage::save(QSettings& settings, const EngravingSettings& value, QString* error)
{
    if (error) error->clear();
    const auto object = encode(value);
    EngravingSettings validated;
    if (!decode(object, validated)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "雕刻参数配置无效，未覆盖已保存参数。");
        return false;
    }
    settings.setValue(kKey, QJsonDocument(object).toJson(QJsonDocument::Compact));
    settings.sync();
    if (settings.status() == QSettings::NoError) return true;
    if (error) *error = QCoreApplication::translate("OpenAutoPath", "无法保存雕刻参数配置。");
    return false;
}

bool EngravingSettingsStorage::loadDefault(EngravingSettings& value, QString* error)
{
    QSettings settings("OAP", "OpenAutoPath");
    return load(settings, value, error);
}

bool EngravingSettingsStorage::saveDefault(const EngravingSettings& value, QString* error)
{
    QSettings settings("OAP", "OpenAutoPath");
    return save(settings, value, error);
}

bool EngravingSettingsStorage::equal(const EngravingSettings& first, const EngravingSettings& second)
{
    return encode(first) == encode(second);
}
} // namespace oap::app
