#include "DirectToolSettingsStorage.h"
#include <QCoreApplication>
#include "detail/ToolContextSettingsJson.h"
#include <QJsonDocument>
#include <QSettings>

namespace oap::app {
namespace { constexpr auto key = "toolpath/direct_tools"; }
bool DirectToolSettingsStorage::load(QSettings& settings, toolpath::ToolpathToolContext& hole,
    toolpath::ToolpathToolContext& pocket, toolpath::ToolpathToolContext& profile, QString* error)
{
    if (error) error->clear();
    if (!settings.contains(key)) return true;
    const auto document = QJsonDocument::fromJson(settings.value(key).toByteArray());
    const auto object = document.object();
    toolpath::ToolpathToolContext h, p, r;
    if (settings.status() != QSettings::NoError || !document.isObject() || object.value("version").toInt() != 1 ||
        !detail::decodeToolContext(object.value("hole").toObject(), h) ||
        !detail::decodeToolContext(object.value("pocket").toObject(), p) ||
        !detail::decodeToolContext(object.value("profile").toObject(), r)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "单刀配置无效，已保留当前刀具参数。");
        return false;
    }
    hole = std::move(h); pocket = std::move(p); profile = std::move(r);
    return true;
}
bool DirectToolSettingsStorage::save(QSettings& settings, const toolpath::ToolpathToolContext& hole,
    const toolpath::ToolpathToolContext& pocket, const toolpath::ToolpathToolContext& profile, QString* error)
{
    if (error) error->clear();
    const QJsonObject object{{"version", 1}, {"hole", detail::encodeToolContext(hole)},
        {"pocket", detail::encodeToolContext(pocket)}, {"profile", detail::encodeToolContext(profile)}};
    settings.setValue(key, QJsonDocument(object).toJson(QJsonDocument::Compact));
    settings.sync();
    if (settings.status() == QSettings::NoError) return true;
    if (error) *error = QCoreApplication::translate("OpenAutoPath", "无法保存单刀配置。");
    return false;
}
bool DirectToolSettingsStorage::loadDefault(toolpath::ToolpathToolContext& hole,
    toolpath::ToolpathToolContext& pocket, toolpath::ToolpathToolContext& profile, QString* error)
{
    QSettings settings("OAP", "OpenAutoPath");
    return load(settings, hole, pocket, profile, error);
}
bool DirectToolSettingsStorage::saveDefault(const toolpath::ToolpathToolContext& hole,
    const toolpath::ToolpathToolContext& pocket, const toolpath::ToolpathToolContext& profile, QString* error)
{
    QSettings settings("OAP", "OpenAutoPath");
    return save(settings, hole, pocket, profile, error);
}
}
