// FreeCAD 式刀具资产目录（oap_toollibrary）。
// ToolBit 是全局资产，ToolLibrary 只组织资产引用、T 刀号与切削默认值；
// 本类维持库对象内 ToolBit 物化快照与全局资产的一致性，并管理资产的
// 增删改、附加/解除引用与同步。
#pragma once

#include <map>
#include <vector>
#include <string>
#include "tooling/library/ToolLibrary.h"


namespace oap::tooling {

// FreeCAD 式刀具资产目录：ToolBit 是全局资产，ToolLibrary 只组织资产引用、
// T 刀号和机床相关的切削默认值。为兼容现有 CAM 规划器，库对象仍保留一份
// ToolBit 物化快照；本类负责维持这些快照与全局资产的一致性。
class ToolAssetCatalog {
public:
    static bool fromLibraries(std::vector<ToolLibrary> libraries,
                              ToolAssetCatalog& catalog,
                              std::string* error = nullptr,
                              bool* identifiersMigrated = nullptr);

    const std::map<std::string, ToolBit>& assets() const noexcept { return m_assets; }
    const std::vector<ToolLibrary>& libraries() const noexcept { return m_libraries; }

    // 仅供需要编辑库元数据、刀位的事务型界面使用；提交前必须调用 synchronize()。
    std::vector<ToolLibrary>& editableLibraries() noexcept { return m_libraries; }

    const ToolBit* assetById(const std::string& assetId) const noexcept;
    const ToolLibrary* libraryById(const std::string& libraryId) const noexcept;
    ToolLibrary* libraryById(const std::string& libraryId) noexcept;

    bool addLibrary(ToolLibrary library,
                    std::string* error = nullptr,
                    bool* identifiersMigrated = nullptr);
    bool replaceLibrary(ToolLibrary library,
                        std::string* error = nullptr,
                        bool* identifiersMigrated = nullptr);
    bool removeLibrary(const std::string& libraryId, std::string* error = nullptr);

    // 修改全局资产时同步更新所有引用该资产的刀库快照。
    bool upsertAsset(ToolBit tool, std::string* error = nullptr);
    // 删除全局资产会解除所有刀库引用和对应 T 刀位。
    bool removeAsset(const std::string& assetId, std::string* error = nullptr);
    bool attachAsset(const std::string& libraryId,
                     const std::string& assetId,
                     std::string* error = nullptr);
    // 仅从指定刀库解除引用，资产继续保留在“全部刀具”中。
    bool detachAsset(const std::string& libraryId,
                     const std::string& assetId,
                     std::string* error = nullptr);

    std::vector<std::string> referencingLibraryIds(const std::string& assetId) const;
    std::vector<std::string> referencingLibraryNames(const std::string& assetId) const;

    // 将通过 editableLibraries() 完成的刀位/元数据修改重新对齐到全局资产。
    bool synchronize(std::string* error = nullptr);
    std::vector<std::string> validationErrors() const;
    bool isValid(std::string* error = nullptr) const;

private:
    std::map<std::string, ToolBit> m_assets;
    std::vector<ToolLibrary> m_libraries;
};

bool sameToolAsset(const ToolBit& first, const ToolBit& second) noexcept;

} // namespace oap::tooling
