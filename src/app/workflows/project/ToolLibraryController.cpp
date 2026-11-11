// ToolLibraryController 实现（oap_app_workflow_cam）。
// 桌面刀具库目录生命周期，独立于 MainWindow/对话框，见 ToolLibraryController.h。
#include "ToolLibraryController.h"
#include <QCoreApplication>

#include <algorithm>
#include <utility>

#include <QFileInfo>
#include <QResource>
#include <QSettings>

#include "tooling/library/ToolLibraryStorage.h"

// 显式引用静态库内的资源，确保最终程序与测试链接时均包含模拟刀库。
static void initializeSimulationToolLibraryResource()
{
    Q_INIT_RESOURCE(simulation_tool_library);
}

namespace oap::app {
namespace {

constexpr auto kOrganization = "OAP";
constexpr auto kApplication = "OpenAutoPath";
constexpr auto kActiveLibraryKey = "tooling/active_library_id";
constexpr auto kSimulationLibraryId = "simulation-general-mm-v1";

} // namespace

// 加载用户目录并补入缺失的模拟刀库；已有模拟刀库不覆盖用户修改。
// 读取失败时保留磁盘文件供用户恢复，旧目录仅升级存储结构。
QStringList ToolLibraryController::load()
{
    QStringList warnings;
    const QString catalogPath =
        tooling::ToolLibraryStorage::defaultCatalogPath();
    m_catalog = {};
    QString error;
    bool requiresCatalogRewrite = false;
    const bool catalogExisted = QFileInfo::exists(catalogPath);
    const bool loaded = catalogExisted &&
        tooling::ToolLibraryStorage::loadCatalog(
            catalogPath, m_catalog, &error, &requiresCatalogRewrite);
    if (!error.isEmpty())
        warnings.push_back(error);
    bool catalogChanged = loaded && requiresCatalogRewrite;
    if ((!catalogExisted || loaded) &&
        !m_catalog.libraryById(QString::fromLatin1(kSimulationLibraryId))) {
        static const bool resourceInitialized = [] {
            initializeSimulationToolLibraryResource();
            return true;
        }();
        Q_UNUSED(resourceInitialized);
        tooling::ToolLibrary simulationLibrary;
        if (tooling::ToolLibraryStorage::loadLibrary(
                QStringLiteral(":/tool-libraries/general-simulation.oaptl"),
                simulationLibrary, &error) &&
            m_catalog.addLibrary(std::move(simulationLibrary), &error)) {
            catalogChanged = true;
        } else {
            warnings.push_back(error);
        }
    }
    if (catalogChanged) {
        QString saveError;
        if (!save(&saveError))
            warnings.push_back(saveError);
    }

    QSettings settings(
        QString::fromLatin1(kOrganization),
        QString::fromLatin1(kApplication));
    m_activeLibraryId = settings.value(
        QString::fromLatin1(kActiveLibraryKey)).toString();
    if (!activeLibrary()) {
        m_activeLibraryId = m_catalog.libraries().isEmpty()
            ? QString{} : m_catalog.libraries().front().id();
        settings.setValue(
            QString::fromLatin1(kActiveLibraryKey),
            m_activeLibraryId);
    }
    return warnings;
}

bool ToolLibraryController::save(QString* error) const
{
    return tooling::ToolLibraryStorage::saveCatalog(
        tooling::ToolLibraryStorage::defaultCatalogPath(),
        m_catalog, error);
}

// 应用前先校验选中库存在;变更判定对比序列化内容而非指针,
// 以便识别"同一 id 但内容被编辑"的情况。
ToolLibraryApplyResult ToolLibraryController::apply(
    tooling::ToolAssetCatalog catalog,
    QString activeLibraryId)
{
    ToolLibraryApplyResult result;
    QString validationError;
    if (!catalog.synchronize(&validationError)) {
        result.error = validationError;
        return result;
    }
    const auto& libraries = catalog.libraries();
    const auto selected = std::find_if(
        libraries.cbegin(), libraries.cend(),
        [&activeLibraryId](const tooling::ToolLibrary& library) {
            return library.id() == activeLibraryId;
        });
    if (selected == libraries.cend()) {
        result.error = QCoreApplication::translate("OpenAutoPath", "所选刀库已不存在。");
        return result;
    }

    const auto previousSelected = std::find_if(
        m_catalog.libraries().cbegin(), m_catalog.libraries().cend(),
        [&activeLibraryId](const tooling::ToolLibrary& library) {
            return library.id() == activeLibraryId;
        });
    result.selectedLibraryChanged =
        m_activeLibraryId != activeLibraryId ||
        previousSelected == m_catalog.libraries().cend() ||
        tooling::ToolLibraryStorage::serializeLibrary(*previousSelected) !=
            tooling::ToolLibraryStorage::serializeLibrary(*selected);

    m_catalog = std::move(catalog);
    m_activeLibraryId = std::move(activeLibraryId);
    QSettings(
        QString::fromLatin1(kOrganization),
        QString::fromLatin1(kApplication))
        .setValue(
            QString::fromLatin1(kActiveLibraryKey),
            m_activeLibraryId);
    save(&result.warning);
    result.ok = true;
    return result;
}

ToolLibraryApplyResult ToolLibraryController::apply(
    QVector<tooling::ToolLibrary> libraries,
    QString activeLibraryId)
{
    tooling::ToolAssetCatalog catalog;
    QString error;
    if (!tooling::ToolAssetCatalog::fromLibraries(
            std::move(libraries), catalog, &error)) {
        ToolLibraryApplyResult result;
        result.error = error;
        return result;
    }
    // 兼容仍只提交刀库向量的旧调用方：未加入任何刀库的全局资产不能因此丢失。
    for (const tooling::ToolBit& asset : m_catalog.assets()) {
        if (!m_catalog.referencingLibraryIds(asset.id).isEmpty() ||
            catalog.assetById(asset.id)) {
            continue;
        }
        if (!catalog.upsertAsset(asset, &error)) {
            ToolLibraryApplyResult result;
            result.error = error;
            return result;
        }
    }
    return apply(std::move(catalog), std::move(activeLibraryId));
}

bool ToolLibraryController::restoreActiveLibrary(
    const QString& activeLibraryId)
{
    const bool exists = std::any_of(
        m_catalog.libraries().cbegin(), m_catalog.libraries().cend(),
        [&activeLibraryId](const tooling::ToolLibrary& library) {
            return library.id() == activeLibraryId;
        });
    if (!exists)
        return false;
    m_activeLibraryId = activeLibraryId;
    return true;
}

const tooling::ToolLibrary*
ToolLibraryController::activeLibrary() const noexcept
{
    const auto selected = std::find_if(
        m_catalog.libraries().cbegin(), m_catalog.libraries().cend(),
        [this](const tooling::ToolLibrary& library) {
            return library.id() == m_activeLibraryId;
        });
    return selected == m_catalog.libraries().cend() ? nullptr : &*selected;
}

} // namespace oap::app
