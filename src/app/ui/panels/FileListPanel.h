// FileListPanel — 分组显示当前目录中的 STEP 文件，并提供相邻文件导航。
#pragma once

#include <QString>
#include <QStringList>
#include <QWidget>

class QEvent;
class QLabel;
class QTreeWidget;

// 文件列表面板：展示可加载的 STEP 文件列表。
class FileListPanel : public QWidget
{
    Q_OBJECT
public:
    explicit FileListPanel(QWidget* parent = nullptr);

    int setFolder(const QString& dir);
    void setCurrentFile(const QString& path);
    QStringList filePaths() const;
    QString currentFilePath() const;
    bool canActivateRelativeFile(int delta) const;
    bool activateRelativeFile(int delta);

signals:
    void fileActivated(const QString& path);

private slots:
    void onCurrentItemChanged();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void fitColumnsToViewport(int preferredCountWidth = -1);

    QLabel* m_header = nullptr;
    QTreeWidget* m_tree = nullptr;
    bool m_adjustingColumns = false;
};
