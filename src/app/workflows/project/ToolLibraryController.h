// 桌面刀具库目录生命周期,独立于 MainWindow/对话框。
#pragma once

#include <QVector>
#include <QString>
#include <QStringList>

#include "tooling/library/ToolAssetCatalog.h"

namespace oap::app {

// apply() 的结果:是否整体成功、选中库是否有实质变化,
// 以及错误/警告信息(保存失败仅告警,不阻止切换)。
struct ToolLibraryApplyResult {
    bool ok = false;
    bool selectedLibraryChanged = false;
    QString error;
    QString warning;
};

// 刀具库控制器：目录加载、迁移与持久化生命周期。
class ToolLibraryController {
public:
    // 加载用户目录并补入缺失的模拟刀库，保留已有刀库与活动库偏好。
    QStringList load();
    // 把当前内存中的刀具库目录持久化到默认目录。
    bool save(QString* error = nullptr) const;

    // 整体替换刀具库目录并切换活动库;校验选中库必须存在,
    // 比较新旧选中库的序列化内容以判定"是否发生实质变化"。
    ToolLibraryApplyResult apply(
        tooling::ToolAssetCatalog catalog,
        QString activeLibraryId);
    ToolLibraryApplyResult apply(
        QVector<tooling::ToolLibrary> libraries,
        QString activeLibraryId);

    // 恢复项目所属的选择,而不改写目录或用户的全局偏好。
    // 若项目引用的库在当前目录中不可用,则返回 false。
    bool restoreActiveLibrary(const QString& activeLibraryId);

    const QVector<tooling::ToolLibrary>& libraries() const noexcept
    {
        return m_catalog.libraries();
    }
    const tooling::ToolAssetCatalog& catalog() const noexcept { return m_catalog; }
    const QString& activeLibraryId() const noexcept
    {
        return m_activeLibraryId;
    }
    const tooling::ToolLibrary* activeLibrary() const noexcept;

private:
    tooling::ToolAssetCatalog m_catalog;
    QString m_activeLibraryId;
};

} // namespace oap::app
