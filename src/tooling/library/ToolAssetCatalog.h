// FreeCAD 式刀具资产目录（oap_toollibrary）。
// ToolBit 是全局资产，ToolLibrary 只组织资产引用、T 刀号与切削默认值；
// 本类维持库对象内 ToolBit 物化快照与全局资产的一致性，并管理资产的
// 增删改、附加/解除引用与同步。
#pragma once

#include "tooling/library/ToolLibrary.h"

#include <QMap>
#include <QStringList>
#include <QVector>

namespace oap::tooling {

// FreeCAD 式刀具资产目录：ToolBit 是全局资产，ToolLibrary 只组织资产引用、
// T 刀号和机床相关的切削默认值。为兼容现有 CAM 规划器，库对象仍保留一份
// ToolBit 物化快照；本类负责维持这些快照与全局资产的一致性。
class ToolAssetCatalog {
public:
    static bool fromLibraries(QVector<ToolLibrary> libraries,
                              ToolAssetCatalog& catalog,
                              QString* error = nullptr,
                              bool* identifiersMigrated = nullptr);

    const QMap<QString, ToolBit>& assets() const noexcept { return m_assets; }
    const QVector<ToolLibrary>& libraries() const noexcept { return m_libraries; }

    // 仅供需要编辑库元数据、刀位的事务型界面使用；提交前必须调用 synchronize()。
    QVector<ToolLibrary>& editableLibraries() noexcept { return m_libraries; }

    const ToolBit* assetById(const QString& assetId) const noexcept;
    const ToolLibrary* libraryById(const QString& libraryId) const noexcept;
    ToolLibrary* libraryById(const QString& libraryId) noexcept;

    bool addLibrary(ToolLibrary library,
                    QString* error = nullptr,
                    bool* identifiersMigrated = nullptr);
    bool replaceLibrary(ToolLibrary library,
                        QString* error = nullptr,
                        bool* identifiersMigrated = nullptr);
    bool removeLibrary(const QString& libraryId, QString* error = nullptr);

    // 修改全局资产时同步更新所有引用该资产的刀库快照。
    bool upsertAsset(ToolBit tool, QString* error = nullptr);
    // 删除全局资产会解除所有刀库引用和对应 T 刀位。
    bool removeAsset(const QString& assetId, QString* error = nullptr);
    bool attachAsset(const QString& libraryId,
                     const QString& assetId,
                     QString* error = nullptr);
    // 仅从指定刀库解除引用，资产继续保留在“全部刀具”中。
    bool detachAsset(const QString& libraryId,
                     const QString& assetId,
                     QString* error = nullptr);

    QStringList referencingLibraryIds(const QString& assetId) const;
    QStringList referencingLibraryNames(const QString& assetId) const;

    // 将通过 editableLibraries() 完成的刀位/元数据修改重新对齐到全局资产。
    bool synchronize(QString* error = nullptr);
    QStringList validationErrors() const;
    bool isValid(QString* error = nullptr) const;

private:
    QMap<QString, ToolBit> m_assets;
    QVector<ToolLibrary> m_libraries;
};

bool sameToolAsset(const ToolBit& first, const ToolBit& second) noexcept;

} // namespace oap::tooling
