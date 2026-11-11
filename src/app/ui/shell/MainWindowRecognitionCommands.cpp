// MainWindow 识别命令与展示适配器。
#include "MainWindow.h"
#include "app/ui/shell/MainWindowUiBuilder.h"
#include "app/ui/view/OcctView.h"
#include "app/panels/ResultPanel.h"
#include "app/panels/FeaturePanel.h"
#include "app/panels/GroupPanel.h"
#include "app/ui/panels/FileListPanel.h"
#include "app/ui/dialogs/ShortcutSettingsDialog.h"
#include "app/ui/panels/HoleToolTreePanel.h"
#include "app/ui/panels/HoleCoverageAuditPanel.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "app/workflows/hole/HoleJobSourceSynchronizer.h"
#include "app/presentation/OverlayController.h"

#include "recognition/FeatureRecognizer.h"
#include "analysis/AabbBoundaryFace.h"
#include "analysis/MoldSurfaceClassifier.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleMoldEnvelopeEntryResolver.h"
#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/pocket/simulation/PocketZMapComparison.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepPrimAPI_MakeBox.hxx>

#include <set>
#include <tuple>
#include <iterator>
#include <sstream>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <array>
#include <optional>
#include <unordered_set>

#include <QFileDialog>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QSettings>
#include <QDir>
#include <QDateTime>
#include <QMenuBar>
#include <QMessageBox>
#include <QToolBar>
#include <QLabel>
#include <QSlider>
#include <QDockWidget>
#include <QListWidget>
#include <QStatusBar>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QElapsedTimer>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QTabWidget>
#include <QScreen>
#include <QDebug>
#include <QMenu>
#include <QAction>
#include <QActionGroup>
#include <QSignalBlocker>
#include <QApplication>
#include <QProgressDialog>
#include <QStringList>
#include <QMetaObject>
#include <QTimer>
#include <QHash>

#include <STEPCAFControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <QCloseEvent>
#include <XCAFPrs_AISObject.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Line.hxx>
#include <AIS_Point.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Graphic3d_DisplayPriority.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Quantity_Color.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_PointAspect.hxx>
#include <Standard_Failure.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax1.hxx>
#include <gp.hxx>
#include <gp_Vec.hxx>
#include <TDF_LabelSequence.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <OSD.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <Standard_Failure.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Poly_Triangle.hxx>
#include <V3d.hxx>

namespace {
constexpr double kWitnessLineWidth = 1.0;
const Quantity_Color kColorWitnessLine(
    1.0, 0.95, 0.10, Quantity_TOC_sRGB);
const Quantity_Color kColorThroughStructure(
    0.53, 0.81, 0.98, Quantity_TOC_sRGB);
const Quantity_Color kColorAxisXPlane(Quantity_NOC_RED);
const Quantity_Color kColorAxisYPlane(Quantity_NOC_GREEN);
const Quantity_Color kColorAxisZPlane(Quantity_NOC_BLUE);
QString projectRootPath()
{
    const QStringList seeds{
        QDir::currentPath(), QCoreApplication::applicationDirPath()};
    for (const QString& seed : seeds) {
        QDir directory(seed);
        for (int level = 0; level < 8; ++level) {
            if (directory.exists(QStringLiteral("src")))
                return directory.absolutePath();
            if (!directory.cdUp())
                break;
        }
    }
    return QDir::currentPath();
}
bool writeProjectLog(
    const QString& fileName,
    const QStringList& lines,
    QString* writtenPath = nullptr)
{
    const QString path = QDir(projectRootPath()).filePath(fileName);
    if (writtenPath)
        *writtenPath = path;
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    const QByteArray utf8 =
        (lines.join(QLatin1Char('\n')) + QLatin1Char('\n')).toUtf8();
    return file.write(utf8) == utf8.size() && file.commit();
}
} // namespace

void MainWindow::recognizeInnerOuterStructures()
{
    runInnerOuterStructureRecognition(true);
}

void MainWindow::runInnerOuterStructureRecognition(
    bool showCompletionDialog)
{
    // 手动入口在加载期间不重入；自动入口由 handleAnalysisFinished 在模型与场景
    // 完成交接后调用，此时仍保留忙碌态以避免用户触发其它操作。
    if (m_uiBusy && showCompletionDialog)
        return;
    const QString title = QCoreApplication::translate("OpenAutoPath", "内外环结构识别");
    if (model().faces.empty() || model().units.empty()) {
        if (showCompletionDialog) {
            QMessageBox::information(
                this, title,
                QCoreApplication::translate("OpenAutoPath", "请先打开并分析一个 STEP 文件。"));
        }
        return;
    }

    const bool ownsLoadingState = !m_uiBusy;
    if (ownsLoadingState)
        setLoadingState(true);
    statusBar()->showMessage(
        QCoreApplication::translate("OpenAutoPath", "正在后台识别孔、通结构和三向槽腔…"));

    QElapsedTimer elapsed;
    elapsed.start();
    try {
        oap::app::ThreeAxisRecognitionRequest request;
        request.model = &model();
        request.modelIndex = &modelIndex();
        request.stepIndexer = &stepIndexer();
        const std::uint64_t requestId =
            m_recognitionWorkspace->start(request);

        QProgressDialog progress(
            QCoreApplication::translate("OpenAutoPath", "正在后台识别孔结构…"),
            QCoreApplication::translate("OpenAutoPath", "取消"), 0, 0, this);
        progress.setWindowTitle(title);
        progress.setWindowModality(Qt::WindowModal);
        progress.setMinimumDuration(showCompletionDialog ? 0 : 300);
        progress.setAutoClose(false);

        // 同步等待后台识别:用事件循环 + 30ms 定时轮询进度,期间保持 UI 响应
        // (进度对话框可取消);worker 就绪(ready)后退出循环取回结果。这样既能
        // 阻止用户在识别期间触发其它操作,又不会阻塞事件派发。
        QEventLoop waitLoop;
        QTimer poll;
        bool cancellationSent = false;
        connect(&poll, &QTimer::timeout, &waitLoop, [&] {
            const oap::app::ThreeAxisRecognitionProgress state =
                m_recognitionWorkspace->progress(requestId);
            if (!cancellationSent && state.phase ==
                oap::app::ThreeAxisRecognitionPhase::Queued) {
                progress.setLabelText(QCoreApplication::translate("OpenAutoPath", "正在等待其他计算任务完成，可取消本次识别…"));
            } else if (!cancellationSent && state.phase ==
                       oap::app::ThreeAxisRecognitionPhase::RecognizingHoles) {
                progress.setLabelText(QCoreApplication::translate("OpenAutoPath", "正在后台识别孔结构…"));
            }
            if (state.phase ==
                oap::app::ThreeAxisRecognitionPhase::TracingStructures) {
                const int maximum = std::max(
                    1, static_cast<int>(state.total));
                if (progress.maximum() != maximum ||
                    progress.minimum() != 0) {
                    progress.setRange(0, maximum);
                }
                progress.setValue(std::min(
                    maximum, static_cast<int>(state.completed)));
                if (!cancellationSent) {
                    progress.setLabelText(QCoreApplication::translate("OpenAutoPath", "正在后台识别通结构和三向槽腔…"));
                }
            }
            if (progress.wasCanceled() && !cancellationSent) {
                cancellationSent = true;
                m_recognitionWorkspace->cancel(requestId);
                progress.setRange(0, 0);
                progress.setLabelText(QCoreApplication::translate("OpenAutoPath", "正在取消，等待当前识别阶段安全结束…"));
            }
            if (m_recognitionWorkspace->ready(requestId))
                waitLoop.quit();
        });
        if (!m_recognitionWorkspace->ready(requestId)) {
            poll.start(30);
            waitLoop.exec();
        }
        poll.stop();
        progress.close();

        std::optional<oap::app::ThreeAxisRecognitionTaskResult> task =
            m_recognitionWorkspace->take(requestId);
        if (!task) {
            const QString message =
                QCoreApplication::translate("OpenAutoPath", "结构识别结果已过期，未发布到场景。");
            statusBar()->showMessage(message);
        } else if (task->cancelled) {
            statusBar()->showMessage(QCoreApplication::translate("OpenAutoPath", "结构识别已取消，上一次有效结果保持不变。"));
        } else if (!task->error.empty() || !task->wireTrace.ok()) {
            const std::string detail = !task->error.empty()
                ? task->error : task->wireTrace.error;
            const QString message = QCoreApplication::translate("OpenAutoPath", "后台结构识别失败：\n%1")
                .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(detail).toUtf8().constData()));
            statusBar()->showMessage(message);
            if (showCompletionDialog)
                QMessageBox::warning(this, title, message);
        } else {
            presentThreeAxisWireTraceResult(
                task->recognition, std::move(task->wireTrace),
                showCompletionDialog, elapsed.elapsed());
        }
    } catch (const std::exception& failure) {
        const QString message =
            QCoreApplication::translate("OpenAutoPath", "无法启动后台结构识别：\n") +
            QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromLocal8Bit(failure.what()).toUtf8().constData());
        statusBar()->showMessage(message);
        if (showCompletionDialog)
            QMessageBox::warning(this, title, message);
    } catch (...) {
        const QString message =
            QCoreApplication::translate("OpenAutoPath", "无法启动后台结构识别：未知异常。");
        statusBar()->showMessage(message);
        if (showCompletionDialog)
            QMessageBox::warning(this, title, message);
    }

    if (ownsLoadingState)
        finishLoadingState();
}

void MainWindow::applyRecognitionColors()
{
    if (m_recognitionWorkspace)
        m_recognitionWorkspace->applyRecognitionColors(sceneController());
}

void MainWindow::refreshRecognitionDisplay()
{
    // 按当前统一结构结果重置并重新染色，刷新识别面板。
    clearTransientOverlay();
    sceneController().resetColors();
    applyRecognitionColors();
    m_view->view()->Redraw();
    if (m_featurePanel) {
        m_featurePanel->setInnerOuterStructureResult(
            m_recognitionWorkspace->result(), {},
            m_recognitionWorkspace->throughStructureFaces(),
            m_recognitionWorkspace->wireTraceFeatureSubsets());
    }
    // 人工修正后视图处于识别染色态,同步开关为启用+勾选(屏蔽信号避免重入)。
    if (m_actFeatureColors) {
        QSignalBlocker block(m_actFeatureColors);
        m_actFeatureColors->setEnabled(true);
        m_actFeatureColors->setChecked(true);
    }
    refreshFeatureLegend();
    setVisibilityActionsEnabled();
    updateStatusBar();
    refreshCamWorkflowPanel();
}

void MainWindow::setFeatureTypeForSelection(oap::FeatureKind kind)
{
    const std::vector<std::string> ids = currentSelectedFaceIds();
    const oap::app::ManualRecognitionEditResult edit =
        m_recognitionWorkspace->applyManualFeatureEdit(
            kind, ids, model());

    switch (edit.status) {
        case oap::app::ManualRecognitionEditStatus::EmptySelection:
            return;
        case oap::app::ManualRecognitionEditStatus::RequiresSingleFloor:
            QMessageBox::information(
                this, QCoreApplication::translate("OpenAutoPath", "人工槽/腔识别"),
                QCoreApplication::translate("OpenAutoPath", "人工标记槽或腔时必须恰好选择一个平面，"
                    "该面将作为权威底面。"));
            return;
        case oap::app::ManualRecognitionEditStatus::InvalidFloor:
            QMessageBox::information(
                this, QCoreApplication::translate("OpenAutoPath", "人工槽/腔识别"),
                QCoreApplication::translate("OpenAutoPath", "选中面 %1 不存在或不是平面，无法作为槽/腔底面。")
                    .arg(QString::fromStdString(edit.floorFaceId)));
            return;
        case oap::app::ManualRecognitionEditStatus::AmbiguousFloor:
            QMessageBox::information(
                this, QCoreApplication::translate("OpenAutoPath", "人工槽/腔识别"),
                QCoreApplication::translate("OpenAutoPath", "选中面 %1 同时属于多个特征实例，"
                    "请先清理冲突后再指定底面。")
                    .arg(QString::fromStdString(edit.floorFaceId)));
            return;
        case oap::app::ManualRecognitionEditStatus::Updated:
        case oap::app::ManualRecognitionEditStatus::NoChange:
            break;
    }

    QStringList warnings;
    for (const oap::app::ManualRecognitionWarning& warning : edit.warnings) {
        if (warning.kind ==
            oap::app::ManualRecognitionWarningKind::ConflictingFloor) {
            warnings << QCoreApplication::translate("OpenAutoPath", "%1: 扩展面组包含其他人工槽/腔的底面 %2，未创建")
                .arg(QString::fromStdString(warning.seedFaceId))
                .arg(QString::fromStdString(warning.relatedFaceId));
        } else {
            warnings << QStringLiteral("%1: %2")
                .arg(QString::fromStdString(warning.seedFaceId))
                .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(warning.detail).toUtf8().constData()));
        }
    }

    if (!edit.updated()) {
        if (!warnings.isEmpty()) {
            QStringList log{
                QCoreApplication::translate("OpenAutoPath", "OpenAutoPath 腔清腔人工识别日志"),
                QCoreApplication::translate("OpenAutoPath", "时间: %1")
                    .arg(QDateTime::currentDateTime().toString(
                        Qt::ISODateWithMs)),
                QCoreApplication::translate("OpenAutoPath", "人工槽/腔未创建:")};
            log << warnings;
            writeProjectLog(QCoreApplication::translate("OpenAutoPath", "腔清腔日志.txt"), log);
            QMessageBox::information(
                this, QCoreApplication::translate("OpenAutoPath", "人工槽/腔识别"),
                warnings.join(QLatin1Char('\n')));
        }
        return;
    }

    refreshRecognitionDisplay();
    if (!m_machiningWorkspace->pocketPreviewActive() ||
        !edit.clearingKind) {
        QString message =
            QCoreApplication::translate("OpenAutoPath", "已更新 %1 个完整特征实例(人工复核)")
                .arg(edit.affectedInstanceCount);
        if (!warnings.isEmpty())
            message += QCoreApplication::translate("OpenAutoPath", "；部分底面无法扩展，详见腔清腔日志");
        statusBar()->showMessage(message);
        if (!warnings.isEmpty()) {
            QStringList log{
                QCoreApplication::translate("OpenAutoPath", "OpenAutoPath 腔清腔人工识别日志"),
                QCoreApplication::translate("OpenAutoPath", "时间: %1")
                    .arg(QDateTime::currentDateTime().toString(
                        Qt::ISODateWithMs)),
                QCoreApplication::translate("OpenAutoPath", "底面种子扩展失败:")};
            log << warnings;
            writeProjectLog(QCoreApplication::translate("OpenAutoPath", "腔清腔日志.txt"), log);
        }
    }
}
void MainWindow::clearFeatureTypeForSelection()
{
    const oap::app::ManualRecognitionEditResult edit =
        m_recognitionWorkspace->clearFeatureSelection(
            currentSelectedFaceIds());
    if (!edit.updated())
        return;
    refreshRecognitionDisplay();
    statusBar()->showMessage(
        QCoreApplication::translate("OpenAutoPath", "已清除 %1 个完整特征实例所含的 %2 个面(人工复核)")
            .arg(edit.affectedInstanceCount)
            .arg(edit.affectedFaceCount));
}
void MainWindow::refreshFeatureLegend()
{
    const bool visible = m_actFeatureColors && m_actFeatureColors->isChecked() &&
                         (!m_actShowFeatureLegend || m_actShowFeatureLegend->isChecked());
    if (!visible) {
        if (m_featureLegend) m_featureLegend->hide();
        return;
    }
    if (!m_featureLegend) {
        m_featureLegend = new QLabel(m_view);
        m_featureLegend->setTextFormat(Qt::RichText);
        m_featureLegend->setMargin(10);
        m_featureLegend->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        m_featureLegend->setStyleSheet(QStringLiteral(
            "QLabel { color:#f2f2f2; background:rgba(24,27,32,220);"
            " border:1px solid rgba(230,230,230,120); border-radius:4px; }"));
    }
    const auto colorHex = [](const Quantity_Color& color) {
        return QStringLiteral("#%1%2%3")
            .arg(qRound(color.Red() * 255.0), 2, 16, QLatin1Char('0'))
            .arg(qRound(color.Green() * 255.0), 2, 16, QLatin1Char('0'))
            .arg(qRound(color.Blue() * 255.0), 2, 16, QLatin1Char('0'));
    };
    const auto appendEntry =
        [&](QString& html, const Quantity_Color& color,
            const QString& name) {
            html += QStringLiteral(
                        "<br><span style='color:%1;font-size:15px'>■</span> %2")
                        .arg(colorHex(color), name);
        };
    const struct { oap::FeatureKind kind; const char* name; } entries[] = {
        {oap::FeatureKind::ThroughHole, "通孔"},
        {oap::FeatureKind::BlindHole, "盲孔"},
        {oap::FeatureKind::CountersinkHole, "沉孔"},
        {oap::FeatureKind::Slot, "槽"},
        {oap::FeatureKind::Pocket, "腔"}};
    QString html = QCoreApplication::translate("OpenAutoPath", "<b>内外环结构颜色</b>");
    for (const auto& entry : entries) {
        appendEntry(
            html, oap::featureColor(entry.kind),
            QCoreApplication::translate("OpenAutoPath", entry.name));
    }
    appendEntry(
        html, kColorThroughStructure,
        QCoreApplication::translate("OpenAutoPath", "通结构"));
    appendEntry(
        html, kColorAxisZPlane,
        QCoreApplication::translate("OpenAutoPath", "Z 向内部平面"));
    appendEntry(
        html, kColorAxisYPlane,
        QCoreApplication::translate("OpenAutoPath", "Y 向内部平面"));
    appendEntry(
        html, kColorAxisXPlane,
        QCoreApplication::translate("OpenAutoPath", "X 向内部平面"));
    m_featureLegend->setText(html);
    m_featureLegend->adjustSize();
    m_featureLegend->move(12, 12);
    m_featureLegend->show();
    m_featureLegend->raise();
}

void MainWindow::toggleFeatureColors(bool on)
{
    // 在“统一内外环结构染色 / 加载时原色”两种视图间切换。
    if (!m_recognitionWorkspace->hasResult()) return;
    clearOverlay();
    sceneController().resetColors();   // 两个分支都先回到原色基线
    if (on)
        applyRecognitionColors();
    m_view->view()->Redraw();
    refreshFeatureLegend();
    statusBar()->showMessage(on ? QCoreApplication::translate("OpenAutoPath", "已切换到内外环结构染色视图")
                                : QCoreApplication::translate("OpenAutoPath", "已切换到原始颜色视图"));
}

void MainWindow::recognizeBaseFaces()
{
    if (m_uiBusy)
        return;
    const oap::app::BaseFaceRecognitionResult recognized =
        m_recognitionWorkspace->recognizeBaseFaces(
            model(), stepIndexer());
    if (!recognized.installed) {
        QMessageBox::information(
            this, QCoreApplication::translate("OpenAutoPath", "识别基面"),
            QCoreApplication::translate("OpenAutoPath", "请先打开并分析一个 STEP 文件。"));
        return;
    }

    clearOverlay();
    sceneController().resetColors();
    if (m_actFeatureColors) {
        QSignalBlocker block(m_actFeatureColors);
        m_actFeatureColors->setChecked(false);
    }
    refreshFeatureLegend();

    const Quantity_Color black(0.0, 0.0, 0.0, Quantity_TOC_RGB);
    int colored = 0;
    for (const std::string& faceId : m_recognitionWorkspace->baseFaces()) {
        if (sceneController().isHidden(faceId))
            continue;
        sceneController().setFaceColor(faceId, black);
        ++colored;
    }
    m_view->view()->Redraw();

    statusBar()->showMessage(
        QCoreApplication::translate("OpenAutoPath", "基面识别完成:共 %1 个基面(涂黑,已显示 %2 个)。")
            .arg(recognized.baseFaceCount)
            .arg(colored));
    QMessageBox::information(
        this, QCoreApplication::translate("OpenAutoPath", "识别基面"),
        QCoreApplication::translate("OpenAutoPath", "识别出 %1 个与包围盒紧贴的基面,已染黑显示。")
            .arg(recognized.baseFaceCount));
}
void MainWindow::showFeatureBoxes()
{
    if (m_uiBusy)
        return;
    if (model().faces.empty() || model().units.empty())
    {
        QMessageBox::information(this, QCoreApplication::translate("OpenAutoPath", "显示包围盒"),
                                 QCoreApplication::translate("OpenAutoPath", "请先打开并分析一个 STEP 文件。"));
        return;
    }
    if (m_view->context().IsNull())
        return;

    // 清掉上一次的临时叠加(含可能已显示的包围盒),逐体重画。每个 unit 的盒子
    // 与基面、主外表面识别使用相同的轴对齐有效平面极值盒。
    clearOverlay();

    int drawn = 0;
    for (const oap::Unit& u : model().units)
    {
        const TopoDS_Shape* shape =
            stepIndexer().shapeForUnit(u.unit_id);
        if (!shape || shape->IsNull()) continue;

        Bnd_Box box = oap::buildAxisAlignedPlanarBox(*shape);
        if (box.IsVoid())
            BRepBndLib::AddOptimal(
                *shape, box,
                Standard_False, Standard_False);
        if (box.IsVoid()) continue;

        if (m_recognitionWorkspace->appendInspectionBoundingBox(
                box, kColorWitnessLine, kWitnessLineWidth,
                sceneController().modelTransform())) {
            ++drawn;
        }
    }
    m_view->view()->Redraw();

    statusBar()->showMessage(
        QCoreApplication::translate("OpenAutoPath", "已显示 %1 个包围盒(特征识别所用逐体 AABB,线框叠加)。").arg(drawn));
}
