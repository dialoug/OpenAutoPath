// MainWindow 识别适配器:把工作区值写入 Qt 控件。
#include "MainWindow.h"
#include <QCoreApplication>

#include "app/ui/view/OcctView.h"
#include "app/panels/FeaturePanel.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"

#include <QAction>
#include <QDockWidget>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>

#include <utility>

bool MainWindow::presentThreeAxisWireTraceResult(
    const oap::RecognitionResult& recognition,
    oap::ThreeAxisWireTraceUseCaseResult useCaseResult,
    bool showCompletionDialog,
    qint64 elapsedMillis)
{
    const QString title = QCoreApplication::translate("OpenAutoPath", "内外环结构识别");
    const oap::app::RecognitionInstallResult installed =
        m_recognitionWorkspace->installThreeAxisResult(
            recognition, std::move(useCaseResult),
            model(), moldFrame(), sceneController(), elapsedMillis,
            [this] {
                clearOverlay();
            });

    if (!installed.installed()) {
        statusBar()->showMessage(installed.message);
        if (showCompletionDialog) {
            if (installed.status ==
                oap::app::RecognitionInstallStatus::Failed) {
                QMessageBox::warning(this, title, installed.message);
            } else if (installed.status ==
                       oap::app::RecognitionInstallStatus::NoModel) {
                QMessageBox::information(this, title, installed.message);
            }
        }
        return false;
    }

    if (m_actFeatureColors) {
        QSignalBlocker block(m_actFeatureColors);
        m_actFeatureColors->setEnabled(
            installed.hasCombinedPresentation);
        m_actFeatureColors->setChecked(
            installed.hasCombinedPresentation);
    }
    refreshFeatureLegend();
    if (m_featurePanel) {
        m_featurePanel->setInnerOuterStructureResult(
            m_recognitionWorkspace->result(), {},
            m_recognitionWorkspace->throughStructureFaces(),
            m_recognitionWorkspace->wireTraceFeatureSubsets());
    }
    if (m_featureDock)
        m_featureDock->raise();
    if (m_view && !m_view->view().IsNull())
        m_view->view()->Redraw();

    statusBar()->showMessage(installed.message);
    if (showCompletionDialog)
        QMessageBox::information(this, title, installed.message);
    return true;
}
