// 文件导航 dock 所有权与激活边界（oap_app_document_ui）。
// 拥有文件列表面板，把文件选择/激活命令经回调端口转交
// ModelDocumentWorkspaceCoordinator，不持有文档状态。
#pragma once

#include <functional>

#include <QObject>
#include <QString>

class FileListPanel;
class QDockWidget;
class QMainWindow;

namespace oap::app {

// 文件导航 dock 控制器：文件列表面板与激活边界。
class DocumentUiController final : public QObject {
public:
    DocumentUiController(
        QMainWindow& window,
        std::function<void(const QString&)> fileActivated);

    FileListPanel& fileList() const noexcept;
    QDockWidget& dock() const noexcept;

private:
    FileListPanel* m_fileList = nullptr;
    QDockWidget* m_dock = nullptr;
};

} // namespace oap::app
