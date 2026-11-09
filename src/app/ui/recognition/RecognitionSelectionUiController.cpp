// RecognitionSelectionUiController 实现（oap_app_recognition_ui）。
// 模型/识别/选择 dock 族与面板回调，见 RecognitionSelectionUiController.h。
#include "app/ui/recognition/RecognitionSelectionUiController.h"

#include "app/panels/FeaturePanel.h"
#include "app/panels/GroupPanel.h"
#include "app/panels/ResultPanel.h"

#include <QDockWidget>
#include <QMainWindow>
#include <utility>

namespace oap::app {

RecognitionSelectionUiController::RecognitionSelectionUiController(
    QMainWindow& window, RecognitionSelectionUiCallbacks callbacks)
    : QObject(&window)
{
    m_resultPanel = new ResultPanel(&window);
    m_analysisDock = new QDockWidget(QStringLiteral("模型"), &window);
    m_analysisDock->setObjectName(QStringLiteral("analysisDock"));
    m_analysisDock->setWidget(m_resultPanel);
    m_analysisDock->setAllowedAreas(
        Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    window.addDockWidget(Qt::RightDockWidgetArea, m_analysisDock);

    m_featurePanel = new FeaturePanel(&window);
    m_featureDock = new QDockWidget(QStringLiteral("特征"), &window);
    m_featureDock->setObjectName(QStringLiteral("featureDock"));
    m_featureDock->setWidget(m_featurePanel);
    m_featureDock->setAllowedAreas(
        Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    window.addDockWidget(Qt::RightDockWidgetArea, m_featureDock);
    window.tabifyDockWidget(m_analysisDock, m_featureDock);

    m_groupPanel = new GroupPanel(&window);
    m_groupDock = new QDockWidget(QStringLiteral("分组"), &window);
    m_groupDock->setObjectName(QStringLiteral("groupDock"));
    m_groupDock->setWidget(m_groupPanel);
    m_groupDock->setAllowedAreas(
        Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    window.addDockWidget(Qt::RightDockWidgetArea, m_groupDock);
    window.tabifyDockWidget(m_analysisDock, m_groupDock);

    m_analysisDock->raise();

    connect(m_resultPanel, &ResultPanel::faceSelected, this,
            [callback = std::move(callbacks.faceSelected)](const QString& id) {
                if (callback) callback(id);
            });
    connect(m_resultPanel, &ResultPanel::contextMenuRequested, this,
            [callback = callbacks.contextMenuRequested](const QPoint& pos) {
                if (callback) callback(pos);
            });
    connect(m_featurePanel, &FeaturePanel::featureSelectionChanged, this,
            [callback = std::move(callbacks.featureSelectionChanged)] {
                if (callback) callback();
            });
    connect(m_featurePanel, &FeaturePanel::contextMenuRequested, this,
            [callback = callbacks.contextMenuRequested](const QPoint& pos) {
                if (callback) callback(pos);
            });
    connect(m_groupPanel, &GroupPanel::groupSelectionChanged, this,
            [callback = std::move(callbacks.groupSelectionChanged)] {
                if (callback) callback();
            });
    connect(m_groupPanel, &GroupPanel::contextMenuRequested, this,
            [callback = callbacks.contextMenuRequested](const QPoint& pos) {
                if (callback) callback(pos);
            });
}

ResultPanel& RecognitionSelectionUiController::resultPanel() const noexcept { return *m_resultPanel; }
FeaturePanel& RecognitionSelectionUiController::featurePanel() const noexcept { return *m_featurePanel; }
GroupPanel& RecognitionSelectionUiController::groupPanel() const noexcept { return *m_groupPanel; }
QDockWidget& RecognitionSelectionUiController::analysisDock() const noexcept { return *m_analysisDock; }
QDockWidget& RecognitionSelectionUiController::featureDock() const noexcept { return *m_featureDock; }
QDockWidget& RecognitionSelectionUiController::groupDock() const noexcept { return *m_groupDock; }

} // namespace oap::app
