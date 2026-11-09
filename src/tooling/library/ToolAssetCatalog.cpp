// ToolAssetCatalog 实现（oap_toollibrary）。
// FreeCAD 式刀具资产目录：库引用、资产增删改与快照一致性维护，见
// ToolAssetCatalog.h。
#include "tooling/library/ToolAssetCatalog.h"

#include <algorithm>
#include <utility>

namespace oap::tooling {
namespace {

ToolLibrary emptyLibraryLike(const ToolLibrary& source)
{
    ToolLibrary result(source.id(), source.name(), source.machine());
    result.setDescription(source.description());
    for (auto it = source.attributes().cbegin();
         it != source.attributes().cend(); ++it) {
        result.setAttribute(it.key(), it.value());
    }
    return result;
}

QString conflictFreeAssetId(const QString& requestedId,
                            const QString& libraryId,
                            const QMap<QString, ToolBit>& assets)
{
    const QString stem = requestedId + QLatin1Char('@') + libraryId;
    QString candidate = stem;
    int suffix = 2;
    while (assets.contains(candidate))
        candidate = stem + QLatin1Char('-') + QString::number(suffix++);
    return candidate;
}

bool validateAsset(const ToolBit& tool, QString* error)
{
    ToolLibrary validator(QStringLiteral("asset-validator"),
                          QStringLiteral("asset-validator"));
    QString itemError;
    if (!validator.upsertTool(tool, &itemError) ||
        !validator.isValid(&itemError)) {
        if (error)
            *error = itemError;
        return false;
    }
    return true;
}

} // namespace

bool sameToolAsset(const ToolBit& first, const ToolBit& second) noexcept
{
    const ToolGeometry& a = first.geometry;
    const ToolGeometry& b = second.geometry;
    return first.id == second.id && first.name == second.name &&
           first.type == second.type && first.note == second.note &&
           first.throughCoolant == second.throughCoolant &&
           first.attributes == second.attributes &&
           a.diameter == b.diameter &&
           a.cuttingEdgeLength == b.cuttingEdgeLength &&
           a.overallLength == b.overallLength &&
           a.cornerRadius == b.cornerRadius &&
           a.tipAngle == b.tipAngle &&
           a.tipCompensation == b.tipCompensation &&
           a.shankDiameter == b.shankDiameter &&
           a.holderDiameter == b.holderDiameter &&
           a.holderLength == b.holderLength &&
           a.spindleEnvelopeDiameter == b.spindleEnvelopeDiameter &&
           a.spindleEnvelopeLength == b.spindleEnvelopeLength;
}

bool ToolAssetCatalog::fromLibraries(QVector<ToolLibrary> libraries,
                                     ToolAssetCatalog& catalog,
                                     QString* error,
                                     bool* identifiersMigrated)
{
    ToolAssetCatalog parsed;
    bool migrated = false;
    for (ToolLibrary& library : libraries) {
        bool itemMigrated = false;
        if (!parsed.addLibrary(std::move(library), error, &itemMigrated))
            return false;
        migrated = migrated || itemMigrated;
    }
    catalog = std::move(parsed);
    if (identifiersMigrated)
        *identifiersMigrated = migrated;
    return true;
}

const ToolBit* ToolAssetCatalog::assetById(const QString& assetId) const noexcept
{
    const auto found = m_assets.constFind(assetId);
    return found == m_assets.cend() ? nullptr : &found.value();
}

const ToolLibrary* ToolAssetCatalog::libraryById(
    const QString& libraryId) const noexcept
{
    const auto found = std::find_if(
        m_libraries.cbegin(), m_libraries.cend(),
        [&libraryId](const ToolLibrary& library) {
            return library.id() == libraryId;
        });
    return found == m_libraries.cend() ? nullptr : &*found;
}

ToolLibrary* ToolAssetCatalog::libraryById(const QString& libraryId) noexcept
{
    const auto found = std::find_if(
        m_libraries.begin(), m_libraries.end(),
        [&libraryId](const ToolLibrary& library) {
            return library.id() == libraryId;
        });
    return found == m_libraries.end() ? nullptr : &*found;
}

bool ToolAssetCatalog::addLibrary(ToolLibrary library,
                                  QString* error,
                                  bool* identifiersMigrated)
{
    if (identifiersMigrated)
        *identifiersMigrated = false;
    if (libraryById(library.id())) {
        if (error)
            *error = QStringLiteral("刀库 ID 已存在：%1").arg(library.id());
        return false;
    }

    QString validationError;
    if (!library.isValid(&validationError)) {
        if (error)
            *error = validationError;
        return false;
    }

    ToolAssetCatalog candidate = *this;
    ToolLibrary normalized = emptyLibraryLike(library);
    QMap<QString, QString> remappedIds;
    bool migrated = false;
    for (const ToolBit& source : library.tools()) {
        ToolBit tool = source;
        const ToolBit* existing = candidate.assetById(tool.id);
        if (existing && !sameToolAsset(*existing, tool)) {
            const QString oldId = tool.id;
            QString reusableId;
            for (const ToolBit& asset : candidate.m_assets) {
                if (asset.attributes.value(
                        QStringLiteral("legacy_tool_id")) != oldId) {
                    continue;
                }
                ToolBit remapped = tool;
                remapped.id = asset.id;
                remapped.attributes.insert(
                    QStringLiteral("legacy_tool_id"), oldId);
                if (sameToolAsset(asset, remapped)) {
                    reusableId = asset.id;
                    break;
                }
            }
            tool.id = reusableId.isEmpty()
                ? conflictFreeAssetId(oldId, library.id(),
                                      candidate.m_assets)
                : reusableId;
            tool.attributes.insert(
                QStringLiteral("legacy_tool_id"), oldId);
            remappedIds.insert(oldId, tool.id);
            migrated = true;
        }
        if (!normalized.upsertTool(tool, &validationError)) {
            if (error)
                *error = validationError;
            return false;
        }
        if (!candidate.m_assets.contains(tool.id))
            candidate.m_assets.insert(tool.id, tool);
    }

    for (const LibraryToolSlot& sourceSlot : library.toolSlots()) {
        LibraryToolSlot slot = sourceSlot;
        slot.toolId = remappedIds.value(slot.toolId, slot.toolId);
        if (!normalized.assignToolToSlot(slot, &validationError)) {
            if (error)
                *error = validationError;
            return false;
        }
    }
    if (!normalized.isValid(&validationError)) {
        if (error)
            *error = validationError;
        return false;
    }
    candidate.m_libraries.push_back(std::move(normalized));
    *this = std::move(candidate);
    if (identifiersMigrated)
        *identifiersMigrated = migrated;
    return true;
}

bool ToolAssetCatalog::replaceLibrary(ToolLibrary library,
                                      QString* error,
                                      bool* identifiersMigrated)
{
    ToolAssetCatalog candidate = *this;
    const QString id = library.id();
    const ToolLibrary* previous = candidate.libraryById(id);
    if (previous) {
        for (const ToolBit& incoming : library.tools()) {
            if (previous->toolById(incoming.id) &&
                candidate.referencingLibraryIds(incoming.id).size() == 1) {
                candidate.m_assets.remove(incoming.id);
            }
        }
    }
    candidate.m_libraries.erase(
        std::remove_if(candidate.m_libraries.begin(),
                       candidate.m_libraries.end(),
                       [&id](const ToolLibrary& item) {
                           return item.id() == id;
                       }),
        candidate.m_libraries.end());
    if (!candidate.addLibrary(std::move(library), error,
                              identifiersMigrated)) {
        return false;
    }
    *this = std::move(candidate);
    return true;
}

bool ToolAssetCatalog::removeLibrary(const QString& libraryId, QString* error)
{
    const qsizetype oldSize = m_libraries.size();
    m_libraries.erase(
        std::remove_if(m_libraries.begin(), m_libraries.end(),
                       [&libraryId](const ToolLibrary& library) {
                           return library.id() == libraryId;
                       }),
        m_libraries.end());
    if (m_libraries.size() == oldSize) {
        if (error)
            *error = QStringLiteral("刀库不存在：%1").arg(libraryId);
        return false;
    }
    return true;
}

bool ToolAssetCatalog::upsertAsset(ToolBit tool, QString* error)
{
    if (!validateAsset(tool, error))
        return false;
    ToolAssetCatalog candidate = *this;
    candidate.m_assets.insert(tool.id, tool);
    for (ToolLibrary& library : candidate.m_libraries) {
        if (library.toolById(tool.id) && !library.upsertTool(tool, error))
            return false;
    }
    *this = std::move(candidate);
    return true;
}

bool ToolAssetCatalog::removeAsset(const QString& assetId, QString* error)
{
    if (!m_assets.contains(assetId)) {
        if (error)
            *error = QStringLiteral("刀具资产不存在：%1").arg(assetId);
        return false;
    }
    ToolAssetCatalog candidate = *this;
    for (ToolLibrary& library : candidate.m_libraries) {
        if (library.toolById(assetId) &&
            !library.removeTool(assetId, error)) {
            return false;
        }
    }
    candidate.m_assets.remove(assetId);
    *this = std::move(candidate);
    return true;
}

bool ToolAssetCatalog::attachAsset(const QString& libraryId,
                                   const QString& assetId,
                                   QString* error)
{
    ToolLibrary* library = libraryById(libraryId);
    const ToolBit* tool = assetById(assetId);
    if (!library) {
        if (error)
            *error = QStringLiteral("刀库不存在：%1").arg(libraryId);
        return false;
    }
    if (!tool) {
        if (error)
            *error = QStringLiteral("刀具资产不存在：%1").arg(assetId);
        return false;
    }
    return library->upsertTool(*tool, error);
}

bool ToolAssetCatalog::detachAsset(const QString& libraryId,
                                   const QString& assetId,
                                   QString* error)
{
    ToolLibrary* library = libraryById(libraryId);
    if (!library) {
        if (error)
            *error = QStringLiteral("刀库不存在：%1").arg(libraryId);
        return false;
    }
    return library->removeTool(assetId, error);
}

QStringList ToolAssetCatalog::referencingLibraryIds(
    const QString& assetId) const
{
    QStringList result;
    for (const ToolLibrary& library : m_libraries) {
        if (library.toolById(assetId))
            result.push_back(library.id());
    }
    return result;
}

QStringList ToolAssetCatalog::referencingLibraryNames(
    const QString& assetId) const
{
    QStringList result;
    for (const ToolLibrary& library : m_libraries) {
        if (library.toolById(assetId))
            result.push_back(library.name());
    }
    return result;
}

bool ToolAssetCatalog::synchronize(QString* error)
{
    ToolAssetCatalog candidate = *this;
    for (ToolLibrary& library : candidate.m_libraries) {
        QVector<ToolBit> canonicalTools;
        canonicalTools.reserve(library.tools().size());
        for (const ToolBit& tool : library.tools()) {
            const ToolBit* canonical = candidate.assetById(tool.id);
            if (!canonical) {
                if (!validateAsset(tool, error))
                    return false;
                candidate.m_assets.insert(tool.id, tool);
                canonical = candidate.assetById(tool.id);
            }
            canonicalTools.push_back(*canonical);
        }
        for (const ToolBit& tool : canonicalTools) {
            if (!library.upsertTool(tool, error))
                return false;
        }
    }
    if (!candidate.isValid(error))
        return false;
    *this = std::move(candidate);
    return true;
}

QStringList ToolAssetCatalog::validationErrors() const
{
    QStringList errors;
    for (auto it = m_assets.cbegin(); it != m_assets.cend(); ++it) {
        if (it.key() != it.value().id) {
            errors.push_back(
                QStringLiteral("资产映射键与刀具 ID 不一致：%1").arg(it.key()));
        }
        QString assetError;
        if (!validateAsset(it.value(), &assetError))
            errors.push_back(assetError);
    }

    QMap<QString, int> libraryIds;
    for (const ToolLibrary& library : m_libraries) {
        libraryIds[library.id()] += 1;
        const QStringList libraryErrors = library.validationErrors();
        for (const QString& item : libraryErrors) {
            errors.push_back(QStringLiteral("刀库“%1”：%2")
                                 .arg(library.name(), item));
        }
        for (const ToolBit& tool : library.tools()) {
            const ToolBit* canonical = assetById(tool.id);
            if (!canonical) {
                errors.push_back(
                    QStringLiteral("刀库“%1”引用了不存在的资产：%2")
                        .arg(library.name(), tool.id));
            } else if (!sameToolAsset(*canonical, tool)) {
                errors.push_back(
                    QStringLiteral("刀库“%1”的刀具快照与全局资产不一致：%2")
                        .arg(library.name(), tool.id));
            }
        }
    }
    for (auto it = libraryIds.cbegin(); it != libraryIds.cend(); ++it) {
        if (it.value() > 1)
            errors.push_back(QStringLiteral("刀库 ID 重复：%1").arg(it.key()));
    }
    return errors;
}

bool ToolAssetCatalog::isValid(QString* error) const
{
    const QStringList errors = validationErrors();
    if (error)
        *error = errors.join(QLatin1Char('\n'));
    return errors.isEmpty();
}

} // namespace oap::tooling
