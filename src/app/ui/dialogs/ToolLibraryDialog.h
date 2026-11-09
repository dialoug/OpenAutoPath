// 刀库浏览与管理对话框。界面层只编辑 tooling 领域对象，文件格式和业务校验
// 仍由 oap_toollibrary 负责。
#pragma once

#include <QDialog>
#include <QStringList>
#include <QVector>

#include "tooling/library/ToolAssetCatalog.h"

class QAction;
class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTableWidget;

// 刀具库对话框：全局刀具资产管理器（编辑、引用与预览）。
class ToolLibraryDialog : public QDialog
{
public:
    explicit ToolLibraryDialog(oap::tooling::ToolAssetCatalog catalog,
                               const QString& activeLibraryId,
                               QWidget* parent = nullptr);

    const QVector<oap::tooling::ToolLibrary>& libraries() const noexcept
    {
        return m_catalog.libraries();
    }
    const oap::tooling::ToolAssetCatalog& catalog() const noexcept
    {
        return m_catalog;
    }
    QString selectedLibraryId() const;

    void accept() override;

private:
    oap::tooling::ToolLibrary* currentLibrary();
    const oap::tooling::ToolLibrary* currentLibrary() const;
    QVector<oap::tooling::ToolLibrary>& editableLibraries()
    {
        return m_catalog.editableLibraries();
    }
    QString selectedToolId() const;
    QStringList selectedToolIds() const;

    void refreshLibraryList(const QString& preferredId = {});
    void refreshTable(const QString& preferredToolId = {});
    void refreshAllToolsTable(const QString& preferredToolId = {});
    void updateSelectionState();
    void createLibrary();
    void renameLibrary();
    void deleteLibrary();
    void importLibrary();
    void exportLibrary();
    void addTool();
    void duplicateTool();
    void copyToolToLibrary();
    void moveToolToLibrary();
    void transferToolToLibrary(bool removeSource);
    void editTool();
    void removeTool();

    oap::tooling::ToolAssetCatalog m_catalog;

    QListWidget* m_libraryList = nullptr;
    QLabel* m_description = nullptr;
    QLineEdit* m_filter = nullptr;
    QComboBox* m_typeFilter = nullptr;
    QComboBox* m_sortMode = nullptr;
    QTableWidget* m_table = nullptr;
    QPushButton* m_useButton = nullptr;
    QPushButton* m_duplicateToolButton = nullptr;
    QPushButton* m_copyToLibraryButton = nullptr;
    QPushButton* m_moveToLibraryButton = nullptr;
    QPushButton* m_editButton = nullptr;
    QPushButton* m_removeToolButton = nullptr;
    QAction* m_editToolAction = nullptr;
    QAction* m_duplicateToolAction = nullptr;
    QAction* m_copyToLibraryAction = nullptr;
    QAction* m_moveToLibraryAction = nullptr;
    QAction* m_removeToolAction = nullptr;
};

// 独立的 CAM 活动刀库选择器。它只选择现有刀库，不承担资产编辑，避免
// 浏览/维护刀库时意外切换当前 Job 的自动选刀来源。
class ToolLibrarySelectionDialog : public QDialog
{
public:
    explicit ToolLibrarySelectionDialog(
        const QVector<oap::tooling::ToolLibrary>& libraries,
        const QString& activeLibraryId,
        QWidget* parent = nullptr);

    QString selectedLibraryId() const;
    void accept() override;

private:
    void updateSelectionState();

    QListWidget* m_list = nullptr;
    QLabel* m_summary = nullptr;
    QPushButton* m_selectButton = nullptr;
};
