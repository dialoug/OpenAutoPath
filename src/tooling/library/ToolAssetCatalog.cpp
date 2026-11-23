// ToolAssetCatalog 实现（oap_toollibrary）。
// FreeCAD 式刀具资产目录：库引用、资产增删改与快照一致性维护，见
// ToolAssetCatalog.h。
#include "support/Text.h"
#include <format>
#include <map>
#include <vector>
#include <string>
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
        result.setAttribute(it->first, it->second);
    }
    return result;
}

std::string conflictFreeAssetId(const std::string& requestedId,
                            const std::string& libraryId,
                            const std::map<std::string, ToolBit>& assets)
{
    const std::string stem = requestedId + '@' + libraryId;
    std::string candidate = stem;
    int suffix = 2;
    while (assets.contains(candidate))
        candidate = stem + '-' + std::to_string(suffix++);
    return candidate;
}

bool validateAsset(const ToolBit& tool, std::string* error)
{
    ToolLibrary validator(std::string("asset-validator"),
                          std::string("asset-validator"));
    std::string itemError;
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

bool ToolAssetCatalog::fromLibraries(std::vector<ToolLibrary> libraries,
                                     ToolAssetCatalog& catalog,
                                     std::string* error,
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

const ToolBit* ToolAssetCatalog::assetById(const std::string& assetId) const noexcept
{
    const auto found = m_assets.find(assetId);
    return found == m_assets.cend() ? nullptr : &found->second;
}

const ToolLibrary* ToolAssetCatalog::libraryById(
    const std::string& libraryId) const noexcept
{
    const auto found = std::find_if(
        m_libraries.cbegin(), m_libraries.cend(),
        [&libraryId](const ToolLibrary& library) {
            return library.id() == libraryId;
        });
    return found == m_libraries.cend() ? nullptr : &*found;
}

ToolLibrary* ToolAssetCatalog::libraryById(const std::string& libraryId) noexcept
{
    const auto found = std::find_if(
        m_libraries.begin(), m_libraries.end(),
        [&libraryId](const ToolLibrary& library) {
            return library.id() == libraryId;
        });
    return found == m_libraries.end() ? nullptr : &*found;
}

bool ToolAssetCatalog::addLibrary(ToolLibrary library,
                                  std::string* error,
                                  bool* identifiersMigrated)
{
    if (identifiersMigrated)
        *identifiersMigrated = false;
    if (libraryById(library.id())) {
        if (error)
            *error = std::format("刀库 ID 已存在：{0}", library.id());
        return false;
    }

    std::string validationError;
    if (!library.isValid(&validationError)) {
        if (error)
            *error = validationError;
        return false;
    }

    ToolAssetCatalog candidate = *this;
    ToolLibrary normalized = emptyLibraryLike(library);
    std::map<std::string, std::string> remappedIds;
    bool migrated = false;
    for (const auto& [key, source] : library.tools()) {
        ToolBit tool = source;
        const ToolBit* existing = candidate.assetById(tool.id);
        if (existing && !sameToolAsset(*existing, tool)) {
            const std::string oldId = tool.id;
            std::string reusableId;
            for (const auto& [key, asset] : candidate.m_assets) {
                const auto legacy = asset.attributes.find("legacy_tool_id");
                if (legacy == asset.attributes.end() || legacy->second != oldId) {
                    continue;
                }
                ToolBit remapped = tool;
                remapped.id = asset.id;
                remapped.attributes.insert_or_assign(
                    std::string("legacy_tool_id"), oldId);
                if (sameToolAsset(asset, remapped)) {
                    reusableId = asset.id;
                    break;
                }
            }
            tool.id = reusableId.empty()
                ? conflictFreeAssetId(oldId, library.id(),
                                      candidate.m_assets)
                : reusableId;
            tool.attributes.insert_or_assign(
                std::string("legacy_tool_id"), oldId);
            remappedIds.insert_or_assign(oldId, tool.id);
            migrated = true;
        }
        if (!normalized.upsertTool(tool, &validationError)) {
            if (error)
                *error = validationError;
            return false;
        }
        if (!candidate.m_assets.contains(tool.id))
            candidate.m_assets.insert_or_assign(tool.id, tool);
    }

    for (const auto& [key, sourceSlot] : library.toolSlots()) {
        LibraryToolSlot slot = sourceSlot;
        if (const auto found = remappedIds.find(slot.toolId); found != remappedIds.end())
            slot.toolId = found->second;
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
                                      std::string* error,
                                      bool* identifiersMigrated)
{
    ToolAssetCatalog candidate = *this;
    const std::string id = library.id();
    const ToolLibrary* previous = candidate.libraryById(id);
    if (previous) {
        for (const auto& [key, incoming] : library.tools()) {
            if (previous->toolById(incoming.id) &&
                candidate.referencingLibraryIds(incoming.id).size() == 1) {
                candidate.m_assets.erase(incoming.id);
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

bool ToolAssetCatalog::removeLibrary(const std::string& libraryId, std::string* error)
{
    const std::size_t oldSize = m_libraries.size();
    m_libraries.erase(
        std::remove_if(m_libraries.begin(), m_libraries.end(),
                       [&libraryId](const ToolLibrary& library) {
                           return library.id() == libraryId;
                       }),
        m_libraries.end());
    if (m_libraries.size() == oldSize) {
        if (error)
            *error = std::format("刀库不存在：{0}", libraryId);
        return false;
    }
    return true;
}

bool ToolAssetCatalog::upsertAsset(ToolBit tool, std::string* error)
{
    if (!validateAsset(tool, error))
        return false;
    ToolAssetCatalog candidate = *this;
    candidate.m_assets.insert_or_assign(tool.id, tool);
    for (ToolLibrary& library : candidate.m_libraries) {
        if (library.toolById(tool.id) && !library.upsertTool(tool, error))
            return false;
    }
    *this = std::move(candidate);
    return true;
}

bool ToolAssetCatalog::removeAsset(const std::string& assetId, std::string* error)
{
    if (!m_assets.contains(assetId)) {
        if (error)
            *error = std::format("刀具资产不存在：{0}", assetId);
        return false;
    }
    ToolAssetCatalog candidate = *this;
    for (ToolLibrary& library : candidate.m_libraries) {
        if (library.toolById(assetId) &&
            !library.removeTool(assetId, error)) {
            return false;
        }
    }
    candidate.m_assets.erase(assetId);
    *this = std::move(candidate);
    return true;
}

bool ToolAssetCatalog::attachAsset(const std::string& libraryId,
                                   const std::string& assetId,
                                   std::string* error)
{
    ToolLibrary* library = libraryById(libraryId);
    const ToolBit* tool = assetById(assetId);
    if (!library) {
        if (error)
            *error = std::format("刀库不存在：{0}", libraryId);
        return false;
    }
    if (!tool) {
        if (error)
            *error = std::format("刀具资产不存在：{0}", assetId);
        return false;
    }
    return library->upsertTool(*tool, error);
}

bool ToolAssetCatalog::detachAsset(const std::string& libraryId,
                                   const std::string& assetId,
                                   std::string* error)
{
    ToolLibrary* library = libraryById(libraryId);
    if (!library) {
        if (error)
            *error = std::format("刀库不存在：{0}", libraryId);
        return false;
    }
    return library->removeTool(assetId, error);
}

std::vector<std::string> ToolAssetCatalog::referencingLibraryIds(
    const std::string& assetId) const
{
    std::vector<std::string> result;
    for (const ToolLibrary& library : m_libraries) {
        if (library.toolById(assetId))
            result.push_back(library.id());
    }
    return result;
}

std::vector<std::string> ToolAssetCatalog::referencingLibraryNames(
    const std::string& assetId) const
{
    std::vector<std::string> result;
    for (const ToolLibrary& library : m_libraries) {
        if (library.toolById(assetId))
            result.push_back(library.name());
    }
    return result;
}

bool ToolAssetCatalog::synchronize(std::string* error)
{
    ToolAssetCatalog candidate = *this;
    for (ToolLibrary& library : candidate.m_libraries) {
        std::vector<ToolBit> canonicalTools;
        canonicalTools.reserve(library.tools().size());
        for (const auto& [key, tool] : library.tools()) {
            const ToolBit* canonical = candidate.assetById(tool.id);
            if (!canonical) {
                if (!validateAsset(tool, error))
                    return false;
                candidate.m_assets.insert_or_assign(tool.id, tool);
                canonical = candidate.assetById(tool.id);
            }
            canonicalTools.push_back(*canonical);
        }
        for (const auto& tool : canonicalTools) {
            if (!library.upsertTool(tool, error))
                return false;
        }
    }
    if (!candidate.isValid(error))
        return false;
    *this = std::move(candidate);
    return true;
}

std::vector<std::string> ToolAssetCatalog::validationErrors() const
{
    std::vector<std::string> errors;
    for (auto it = m_assets.cbegin(); it != m_assets.cend(); ++it) {
        if (it->first != it->second.id) {
            errors.push_back(
                std::format("资产映射键与刀具 ID 不一致：{0}", it->first));
        }
        std::string assetError;
        if (!validateAsset(it->second, &assetError))
            errors.push_back(assetError);
    }

    std::map<std::string, int> libraryIds;
    for (const ToolLibrary& library : m_libraries) {
        libraryIds[library.id()] += 1;
        const std::vector<std::string> libraryErrors = library.validationErrors();
        for (const std::string& item : libraryErrors) {
            errors.push_back(std::format("刀库“{0}”：{1}", library.name(), item));
        }
        for (const auto& [key, tool] : library.tools()) {
            const ToolBit* canonical = assetById(tool.id);
            if (!canonical) {
                errors.push_back(
                    std::format("刀库“{0}”引用了不存在的资产：{1}", library.name(), tool.id));
            } else if (!sameToolAsset(*canonical, tool)) {
                errors.push_back(
                    std::format("刀库“{0}”的刀具快照与全局资产不一致：{1}", library.name(), tool.id));
            }
        }
    }
    for (auto it = libraryIds.cbegin(); it != libraryIds.cend(); ++it) {
        if (it->second > 1)
            errors.push_back(std::format("刀库 ID 重复：{0}", it->first));
    }
    return errors;
}

bool ToolAssetCatalog::isValid(std::string* error) const
{
    const std::vector<std::string> errors = validationErrors();
    if (error)
        *error = oap::joinLines(errors);
    return errors.empty();
}

} // namespace oap::tooling
