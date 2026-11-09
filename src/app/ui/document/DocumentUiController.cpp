// DocumentUiController 实现（oap_app_document_ui）。
// 文件导航 dock 与激活边界，见 DocumentUiController.h。
#include "app/ui/document/DocumentUiController.h"

#include "app/ui/panels/FileListPanel.h"

#include <QDockWidget>
#include <QMainWindow>
#include <utility>

namespace oap::app {

DocumentUiController::DocumentUiController(
    QMainWindow& window, std::function<void(const QString&)> fileActivated)
    : QObject(&window)
{
    m_fileList = new FileListPanel(&window);
    m_dock = new QDockWidget(QStringLiteral("文件"), &window);
    m_dock->setObjectName(QStringLiteral("fileDock"));
    m_dock->setWidget(m_fileList);
    m_dock->setAllowedAreas(
        Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    window.addDockWidget(Qt::LeftDockWidgetArea, m_dock);

    connect(m_fileList, &FileListPanel::fileActivated, this,
            [callback = std::move(fileActivated)](const QString& path) {
                if (callback) callback(path);
            });
}

FileListPanel& DocumentUiController::fileList() const noexcept { return *m_fileList; }
QDockWidget& DocumentUiController::dock() const noexcept { return *m_dock; }

} // namespace oap::app
