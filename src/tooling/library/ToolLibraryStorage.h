// 刀库 JSON 持久化与 FreeCAD .fctl/.fctb 兼容导入。
//
// 模块归属：oap_toollibrary。负责 .oaptl 单刀库 / catalog.json 目录的
// 读写与版本校验，及旧 FreeCAD 刀库的兼容导入。
#pragma once

#include "tooling/library/ToolLibrary.h"
#include "tooling/library/ToolAssetCatalog.h"

#include <QByteArray>
#include <QVector>

namespace oap::tooling {

// 刀库 JSON 持久化服务：.oaptl/目录读写、版本校验与 FreeCAD 兼容导入。
class ToolLibraryStorage {
public:
    static QByteArray serializeLibrary(const ToolLibrary& library);
    static bool deserializeLibrary(const QByteArray& data,
                                   ToolLibrary& library,
                                   QString* error = nullptr);

    // .oaptl 使用本项目的自包含 JSON 格式；读取 .fctl 时会解析相邻 Bit/*.fctb。
    static bool saveLibrary(const QString& filePath,
                            const ToolLibrary& library,
                            QString* error = nullptr);
    static bool loadLibrary(const QString& filePath,
                            ToolLibrary& library,
                            QString* error = nullptr);

    static bool saveCatalog(const QString& filePath,
                            const ToolAssetCatalog& catalog,
                            QString* error = nullptr);
    static bool saveCatalog(const QString& filePath,
                            const QVector<ToolLibrary>& libraries,
                            QString* error = nullptr);
    // 旧目录仍可读取；仅迁移存储结构，不填充刀具几何或切削参数。
    // requiresCatalogRewrite 告知调用方按当前版本重新保存。
    static bool loadCatalog(const QString& filePath,
                            ToolAssetCatalog& catalog,
                            QString* error = nullptr,
                            bool* requiresCatalogRewrite = nullptr);
    static bool loadCatalog(const QString& filePath,
                            QVector<ToolLibrary>& libraries,
                            QString* error = nullptr);

    static QString defaultCatalogPath();
};

} // namespace oap::tooling
