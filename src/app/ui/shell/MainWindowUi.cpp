// MainWindow 菜单、工具栏、停靠面板、快捷键与 UI 状态适配器。
#include "MainWindow.h"
#include "app/ui/playback/PlaybackUiController.h"
#include "app/ui/recognition/RecognitionSelectionUiController.h"
#include "app/ui/document/DocumentUiController.h"
#include "app/ui/cam/CamUiController.h"
#include "app/ui/simulation/SimulationUiController.h"
#include "app/ui/shell/MainWindowUiBuilder.h"
#include "app/ui/view/OcctView.h"
#include "app/panels/ResultPanel.h"
#include "app/panels/FeaturePanel.h"
#include "app/panels/GroupPanel.h"
#include "app/ui/panels/FileListPanel.h"
#include "app/ui/dialogs/ShortcutSettingsDialog.h"
#include "app/ui/panels/HoleToolTreePanel.h"
#include "app/ui/panels/HoleCoverageAuditPanel.h"
#include "app/ui/panels/FeatureAllowancePanel.h"
#include "app/ui/panels/CamWorkflowPanel.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "app/workflows/hole/HoleToolpathCommandPolicy.h"
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
#include <QToolButton>
#include <QWidgetAction>
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
constexpr int kDefaultViewLayoutStateVersion = 1;
constexpr int kGraySliderMin = 0;
constexpr int kGraySliderMax = 100;
constexpr double kGraySliderScale = 100.0;
constexpr int kGraySliderWidth = 160;

QString shortcutSettingsKey(const QString& id)
{
    return QStringLiteral("shortcuts/%1").arg(id);
}
QString mouseSettingsKey(const QString& id)
{
    return QStringLiteral("mouse/%1").arg(id);
}
QString viewSettingsKey(const QString& id)
{
    return QStringLiteral("view/%1").arg(id);
}
Qt::MouseButton mouseButtonFromSettings(
    const QVariant& value,
    Qt::MouseButton fallback)
{
    if (!value.isValid()) return fallback;
    const Qt::MouseButton button = Qt::MouseButton(value.toInt());
    switch (button) {
        case Qt::NoButton:
        case Qt::LeftButton:
        case Qt::RightButton:
        case Qt::MiddleButton:
        case Qt::BackButton:
        case Qt::ForwardButton: return button;
        default: return fallback;
    }
}
bool isClickMouseAction(const QString& id)
{
    return id == QStringLiteral("mouse.select") ||
           id == QStringLiteral("mouse.context_menu");
}
bool isAllowedMouseSharing(const QStringList& ids)
{
    int clicks = 0;
    int drags = 0;
    for (const QString& id : ids) {
        if (isClickMouseAction(id))
            ++clicks;
        else
            ++drags;
    }
    return clicks <= 1 && drags <= 1;
}
} // namespace

void MainWindow::setupMenus()
{
    QMenu *fileMenu = menuBar()->addMenu(QStringLiteral("文件(&F)"));
    QAction *openAct = fileMenu->addAction(QStringLiteral("打开 STEP(&O)..."));
    connect(openAct, &QAction::triggered, this, &MainWindow::openStep);
    registerShortcut(QStringLiteral("file.open"), QStringLiteral("打开 STEP"),
                     openAct, QKeySequence::Open);
    QAction *openFolderAct = fileMenu->addAction(QStringLiteral("打开文件夹(&F)..."));
    connect(openFolderAct, &QAction::triggered, this, &MainWindow::openFolder);
    m_actPrevFile = fileMenu->addAction(QStringLiteral("上一个文件"));
    connect(m_actPrevFile, &QAction::triggered, this,
            [this] { navigateRelativeFile(-1); });
    registerShortcut(QStringLiteral("navigation.previous_file"),
                     QStringLiteral("上一个文件"), m_actPrevFile,
                     QKeySequence(Qt::Key_W));
    m_actNextFile = fileMenu->addAction(QStringLiteral("下一个文件"));
    connect(m_actNextFile, &QAction::triggered, this,
            [this] { navigateRelativeFile(1); });
    registerShortcut(QStringLiteral("navigation.next_file"),
                     QStringLiteral("下一个文件"), m_actNextFile,
                     QKeySequence(Qt::Key_S));
    // 旧版曾把 W 用于着色；文件导航接管 W 时只迁移这一默认值，
    // 用户明确保存的其它自定义键保持不变。
    {
        QSettings settings;
        const QString previousKey = shortcutSettingsKey(
            QStringLiteral("navigation.previous_file"));
        const QString shadedKey = shortcutSettingsKey(QStringLiteral("view.shaded"));
        if (!settings.contains(previousKey) && settings.contains(shadedKey) &&
            QKeySequence(settings.value(shadedKey).toString()) == QKeySequence(Qt::Key_W)) {
            settings.setValue(shadedKey,
                              QKeySequence(Qt::Key_T).toString(QKeySequence::PortableText));
        }
    }
    updateFileNavigationActions();
    fileMenu->addSeparator();
    QAction *quitAct = fileMenu->addAction(QStringLiteral("退出(&Q)"));
    connect(quitAct, &QAction::triggered, this, &QWidget::close);

    // CAM 菜单按标准加工流程排序。FreeCAD 的 Job/Setup/Tools/Operations/
    // Inspect/Post 层级在这里映射为当前项目的特征识别、六面装夹、自动选刀、
    // 覆盖审计/IPW 和 Reference NC 审查。
    QMenu *camMenu = menuBar()->addMenu(QStringLiteral("CAM(&C)"));
    m_actOpenMachiningJob = camMenu->addAction(
        QStringLiteral("1. 打开加工 Job(&O)..."));
    m_actOpenMachiningJob->setIconText(QStringLiteral("打开 Job"));
    connect(m_actOpenMachiningJob, &QAction::triggered,
            this, &MainWindow::openMachiningJob);
    m_actSaveMachiningJob = camMenu->addAction(
        QStringLiteral("保存加工 Job(&S)..."));
    m_actSaveMachiningJob->setIconText(QStringLiteral("保存 Job"));
    connect(m_actSaveMachiningJob, &QAction::triggered,
            this, &MainWindow::saveMachiningJob);
    camMenu->addSeparator();

    m_actRecognizeCamFeatures = camMenu->addAction(
        QStringLiteral("2. 识别加工特征(&R)"));
    m_actRecognizeCamFeatures->setIconText(QStringLiteral("特征识别"));
    connect(m_actRecognizeCamFeatures, &QAction::triggered,
            this, &MainWindow::recognizeInnerOuterStructures);
    m_actToolpathParams = camMenu->addAction(
        QStringLiteral("3. 加工参数与装夹(&P)..."));
    m_actToolpathParams->setIconText(QStringLiteral("参数 / 装夹"));
    connect(m_actToolpathParams, &QAction::triggered,
            this, &MainWindow::configureToolpathParams);
    auto* selectToolLibraryAction = camMenu->addAction(
        QStringLiteral("4. 选择活动刀库(&T)..."));
    selectToolLibraryAction->setIconText(QStringLiteral("选择刀库"));
    connect(selectToolLibraryAction, &QAction::triggered,
            this, &MainWindow::selectToolLibrary);
    m_actToolLibrary = camMenu->addAction(
        QStringLiteral("刀具资产管理(&L)..."));
    m_actToolLibrary->setIconText(QStringLiteral("刀库"));
    connect(m_actToolLibrary, &QAction::triggered,
            this, &MainWindow::openToolLibrary);
    camMenu->addSeparator();

    m_actHolePaths = camMenu->addAction(
        QStringLiteral("5. 生成 / 更新 / 显示孔刀路(&H)"));
    m_actHolePaths->setIconText(QStringLiteral("孔刀路"));
    m_actHolePaths->setCheckable(true);
    m_actHolePaths->setToolTip(
        QStringLiteral("识别孔、按参数页配置的刀具来源验证并生成统一孔程序；取消勾选仅隐藏显示"));
    connect(m_actHolePaths, &QAction::triggered,
            this, &MainWindow::showHolePaths);
    m_actPocketClearing = camMenu->addAction(
        QStringLiteral("生成 / 更新槽腔刀路(&C)"));
    m_actPocketClearing->setIconText(QStringLiteral("槽腔刀路"));
    m_actPocketClearing->setToolTip(
        QStringLiteral("为主外表面内环槽腔生成逐层清腔工序"));
    connect(m_actPocketClearing, &QAction::triggered,
            this, &MainWindow::showPocketClearing);
    m_actProfileContour = camMenu->addAction(
        QStringLiteral("生成 / 更新外轮廓刀路(&P)"));
    m_actProfileContour->setIconText(QStringLiteral("外轮廓刀路"));
    m_actProfileContour->setToolTip(
        QStringLiteral(
            "自动识别主外表面的闭合外边界，并在工件外侧生成分层 Profile 工序"));
    connect(m_actProfileContour, &QAction::triggered,
            this, &MainWindow::showProfileContour);
    m_actSurfaceFinishing = camMenu->addAction(QStringLiteral("生成雕刻刀路"));
    m_actSurfaceFinishing->setToolTip(
        QStringLiteral("按统一雕刻参数生成刀路，默认六面，也可选择任一单面；不要求孔槽特征识别"));
    connect(m_actSurfaceFinishing, &QAction::triggered,
            this, &MainWindow::generateEngravingToolpaths);
    m_actProjectStock = camMenu->addAction(QStringLiteral("加工项目 IPW（按全局工序顺序）…"));
    connect(m_actProjectStock, &QAction::triggered, this, &MainWindow::simulateProjectStock);
    camMenu->addSeparator();

    m_actHoleStock = camMenu->addAction(QStringLiteral("6. 孔 IPW 仿真..."));
    m_actHoleStock->setIconText(QStringLiteral("孔 IPW"));
    m_actHoleStock->setToolTip(
        QStringLiteral("自动生成或复用当前孔程序，逐工序减料并查看剩余毛坯"));
    connect(m_actHoleStock, &QAction::triggered,
            this, &MainWindow::simulateHoleStock);
    m_actPocketStock = camMenu->addAction(QStringLiteral("槽腔 IPW 仿真..."));
    m_actPocketStock->setIconText(QStringLiteral("槽腔 IPW"));
    m_actPocketStock->setToolTip(
        QStringLiteral("复用当前腔刀路，逐工序减料并查看剩余毛坯与过切"));
    connect(m_actPocketStock, &QAction::triggered,
            this, &MainWindow::simulatePocketStock);
    m_actProfileStock = camMenu->addAction(
        QStringLiteral("外轮廓 IPW 仿真..."));
    m_actProfileStock->setIconText(QStringLiteral("外轮廓 IPW"));
    m_actProfileStock->setToolTip(
        QStringLiteral(
            "复用当前外轮廓刀路，逐工序减料并查看剩余毛坯、余量与过切"));
    connect(m_actProfileStock, &QAction::triggered,
            this, &MainWindow::simulateProfileStock);
    m_actEngravingStock = camMenu->addAction(QStringLiteral("雕刻 IPW 仿真..."));
    m_actEngravingStock->setIconText(QStringLiteral("雕刻 IPW"));
    m_actEngravingStock->setToolTip(QStringLiteral("按所选雕刻方向接续前序剩余毛坯，显示材料去除与刀具动画"));
    connect(m_actEngravingStock, &QAction::triggered, this, &MainWindow::simulateEngravingStock);
    m_actFeatureAllowance = camMenu->addAction(
        QStringLiteral("特征余量视图(&A)..."));
    m_actFeatureAllowance->setToolTip(
        QStringLiteral("列出各个孔、槽、腔及外轮廓加工后的局部余量，并在主视口单独显示"));
    connect(m_actFeatureAllowance, &QAction::triggered,
            this, &MainWindow::showFeatureAllowanceView);
    camMenu->addSeparator();
    m_actExportToolpath = camMenu->addAction(
        QStringLiteral("8. 审查并导出当前程序(&E)..."));
    m_actExportToolpath->setIconText(QStringLiteral("审查 / 导出"));
    connect(m_actExportToolpath, &QAction::triggered,
            this, &MainWindow::exportCurrentToolpathPreview);

    m_viewMenu = menuBar()->addMenu(QStringLiteral("视图(&V)"));
    m_displayMenu = m_viewMenu->addMenu(QStringLiteral("显示(&D)"));
    m_panelMenu = m_viewMenu->addMenu(QStringLiteral("面板(&P)"));
    m_toolbarMenu = m_viewMenu->addMenu(QStringLiteral("工具栏(&T)"));
    QAction* shortcutAct = m_viewMenu->addAction(QStringLiteral("修改按键(&K)..."));
    shortcutAct->setToolTip(QStringLiteral("设置键盘快捷键和 3D 鼠标操作"));
    connect(shortcutAct, &QAction::triggered, this, &MainWindow::showShortcutSettings);
    registerShortcut(QStringLiteral("settings.shortcut_settings"),
                     QStringLiteral("修改按键"), shortcutAct,
                     QKeySequence(Qt::Key_L));
    m_viewMenu->addSeparator();
    QAction *resetLayoutAct = m_viewMenu->addAction(QStringLiteral("重置布局(&R)"));
    resetLayoutAct->setToolTip(QStringLiteral("恢复默认的面板与工具栏布局"));
    connect(resetLayoutAct, &QAction::triggered, this, &MainWindow::resetViewLayout);

    m_actFitAll = m_displayMenu->addAction(QStringLiteral("自动回正并适合窗口"));
    m_actFitAll->setToolTip(
        QStringLiteral("恢复初始标准轴测方向，并在 3D 视图中完整显示模型 (Home)"));
    connect(m_actFitAll, &QAction::triggered, this, [this]
            {
        if (m_view->view().IsNull())
            return;
        if (m_view->rotationMode() == OcctView::RotationMode::Model)
            m_view->setStandardOrientation(V3d_TypeOfOrientation_Zup_AxoRight);
        else
            m_view->view()->ResetViewOrientation();
        fitModelInView();
    });
    registerShortcut(QStringLiteral("view.fit_all"),
                     QStringLiteral("自动回正并适合窗口"),
                     m_actFitAll, QKeySequence(Qt::Key_Home));

    m_actNearestStandardView =
        m_displayMenu->addAction(QStringLiteral("最近方位并适合窗口"));
    m_actNearestStandardView->setToolTip(
        QStringLiteral("吸附到当前观察方向最接近的前、后、左、右、顶或底视图 (F8)"));
    connect(m_actNearestStandardView, &QAction::triggered,
            this, &MainWindow::snapToNearestStandardView);
    registerShortcut(QStringLiteral("view.nearest_standard"),
                     QStringLiteral("最近方位并适合窗口"),
                     m_actNearestStandardView, QKeySequence(Qt::Key_F8));

    QAction* focusSelectionAction =
        m_displayMenu->addAction(QStringLiteral("定位当前选择并居中"));
    focusSelectionAction->setToolTip(
        QStringLiteral("将当前选择的区域缩放并显示在视图中央 (F)"));
    connect(focusSelectionAction, &QAction::triggered,
            this, &MainWindow::focusCurrentSelection);
    registerShortcut(QStringLiteral("view.focus_selection"),
                     QStringLiteral("定位当前选择并居中"),
                     focusSelectionAction, QKeySequence(Qt::Key_F));

    QMenu* standardViewsMenu =
        m_displayMenu->addMenu(QStringLiteral("快速切换方位"));
    const auto addStandardView =
        [this, standardViewsMenu](const QString& id,
                                  const QString& text,
                                  V3d_TypeOfOrientation orientation) {
            QAction* action = standardViewsMenu->addAction(text);
            connect(action, &QAction::triggered, this,
                    [this, orientation] {
                m_view->setStandardOrientation(orientation);
                fitModelInView();
            });
            registerShortcut(QStringLiteral("view.%1").arg(id),
                             QStringLiteral("%1并适合窗口").arg(text),
                             action, QKeySequence());
        };
    addStandardView(QStringLiteral("front"), QStringLiteral("前视图"),
                    V3d_TypeOfOrientation_Zup_Front);
    addStandardView(QStringLiteral("back"), QStringLiteral("后视图"),
                    V3d_TypeOfOrientation_Zup_Back);
    addStandardView(QStringLiteral("left"), QStringLiteral("左视图"),
                    V3d_TypeOfOrientation_Zup_Left);
    addStandardView(QStringLiteral("right"), QStringLiteral("右视图"),
                    V3d_TypeOfOrientation_Zup_Right);
    addStandardView(QStringLiteral("top"), QStringLiteral("顶视图"),
                    V3d_TypeOfOrientation_Zup_Top);
    addStandardView(QStringLiteral("bottom"), QStringLiteral("底视图"),
                    V3d_TypeOfOrientation_Zup_Bottom);
    standardViewsMenu->addSeparator();
    addStandardView(QStringLiteral("axo_left"), QStringLiteral("左轴测视图"),
                    V3d_TypeOfOrientation_Zup_AxoLeft);
    addStandardView(QStringLiteral("axo_right"), QStringLiteral("右轴测视图"),
                    V3d_TypeOfOrientation_Zup_AxoRight);

    QMenu* toolpathLayerMenu =
        m_displayMenu->addMenu(QStringLiteral("刀路显示层级"));
    toolpathLayerMenu->setToolTip(
        QStringLiteral("控制刀路是否接受模型和余料的深度遮挡"));
    QActionGroup* toolpathLayerGroup =
        new QActionGroup(toolpathLayerMenu);
    toolpathLayerGroup->setExclusive(true);
    const auto addToolpathLayer =
        [this, toolpathLayerMenu, toolpathLayerGroup](
            const QString& text, const QString& toolTip, int layer) {
            QAction* action = toolpathLayerMenu->addAction(text);
            action->setCheckable(true);
            action->setData(layer);
            action->setToolTip(toolTip);
            action->setChecked(
                m_playbackWorkspace->displayLayer() == layer);
            toolpathLayerGroup->addAction(action);
        };
    addToolpathLayer(
        QStringLiteral("模型深度层（默认）"),
        QStringLiteral("刀路接受模型和余料的正常深度遮挡，不透过实体显示"),
        0);
    addToolpathLayer(
        QStringLiteral("分析顶层"),
        QStringLiteral("刀路显示在普通模型之上，并与 IPW 分析结果共享顶层"),
        1);
    addToolpathLayer(
        QStringLiteral("最顶层"),
        QStringLiteral("刀路始终置顶，可透过模型观察完整路径"),
        2);
    connect(
        toolpathLayerGroup, &QActionGroup::triggered,
        this, [this](QAction* action) {
            if (!action)
                return;
            m_playbackWorkspace->setDisplayLayer(
                action->data().toInt());
            QSettings(
                QStringLiteral("OAP"),
                QStringLiteral("OpenAutoPath"))
                .setValue(
                    QStringLiteral("toolpath/display_layer"),
                    m_playbackWorkspace->displayLayer());
            if (m_playbackUi)
                m_playbackUi->applyDisplayLayer();
            const QStringList names = {
                QStringLiteral("模型深度层"),
                QStringLiteral("分析顶层"),
                QStringLiteral("最顶层")};
            statusBar()->showMessage(
                QStringLiteral("刀路显示层级：%1")
                    .arg(names[m_playbackWorkspace->displayLayer()]));
        });

    QMenu* ipwDisplayModeMenu =
        m_displayMenu->addMenu(
            QStringLiteral("Multi-Dexel 显示形式"));
    ipwDisplayModeMenu->setToolTip(
        QStringLiteral(
            "选择统一 IPW 在 OCCT 主视口中的派生显示形式"));
    QActionGroup* ipwDisplayModeGroup =
        new QActionGroup(ipwDisplayModeMenu);
    ipwDisplayModeGroup->setExclusive(true);
    const auto addIpwDisplayMode =
        [this, ipwDisplayModeMenu, ipwDisplayModeGroup](
            const QString& text,
            const QString& toolTip,
            oap::app::IpwOcctDisplayMode mode) {
            QAction* action = ipwDisplayModeMenu->addAction(text);
            action->setCheckable(true);
            action->setData(static_cast<int>(mode));
            action->setToolTip(toolTip);
            action->setChecked(
                m_stockSimulationWorkspace->displayMode() == mode);
            ipwDisplayModeGroup->addAction(action);
        };
    addIpwDisplayMode(
        QStringLiteral("带网格表面（默认）"),
        QStringLiteral("三角化材料外表面，显示着色和完整三角边"),
        oap::app::IpwOcctDisplayMode::SurfaceWithEdges);
    addIpwDisplayMode(
        QStringLiteral("无边线着色表面"),
        QStringLiteral("三角化材料外表面，但隐藏三角边"),
        oap::app::IpwOcctDisplayMode::SurfaceShaded);
    addIpwDisplayMode(
        QStringLiteral("纯三角线框"),
        QStringLiteral("只显示三角网格边，不显示表面填充"),
        oap::app::IpwOcctDisplayMode::SurfaceWireframe);
    ipwDisplayModeMenu->addSeparator();
    addIpwDisplayMode(
        QStringLiteral("Dexel 材料柱"),
        QStringLiteral("每个材料区间显示为沿 W 轴的一条线段"),
        oap::app::IpwOcctDisplayMode::DexelColumns);
    addIpwDisplayMode(
        QStringLiteral("区间端点点云"),
        QStringLiteral("显示每个 Multi-Dexel 材料区间的上下端点"),
        oap::app::IpwOcctDisplayMode::IntervalEndpoints);
    addIpwDisplayMode(
        QStringLiteral("材料体素柱"),
        QStringLiteral("每个材料区间显示为一个 UV 单元棱柱"),
        oap::app::IpwOcctDisplayMode::IntervalPrisms);
    addIpwDisplayMode(
        QStringLiteral("W 向中剖面"),
        QStringLiteral("显示材料整体 W 范围 50% 位置的占用单元"),
        oap::app::IpwOcctDisplayMode::MidSection);
    addIpwDisplayMode(
        QStringLiteral("BRep 实体"),
        QStringLiteral("按需把材料区间重建为 OCCT 拓扑实体；大模型可能较慢"),
        oap::app::IpwOcctDisplayMode::BRepSnapshot);
    ipwDisplayModeMenu->addSeparator();
    addIpwDisplayMode(
        QStringLiteral("关闭 Multi-Dexel 显示"),
        QStringLiteral("保留材料检查点和审计结果，不在主视口发布 IPW"),
        oap::app::IpwOcctDisplayMode::Off);
    connect(
        ipwDisplayModeGroup, &QActionGroup::triggered,
        this, [this](QAction* action) {
            if (!action)
                return;
            m_stockSimulationWorkspace->setDisplayMode(
                static_cast<oap::app::IpwOcctDisplayMode>(
                    std::clamp(action->data().toInt(), 0, 8)));
            QSettings(
                QStringLiteral("OAP"),
                QStringLiteral("OpenAutoPath"))
                .setValue(
                    QStringLiteral("ipw/occt_display_mode"),
                    static_cast<int>(
                        m_stockSimulationWorkspace->displayMode()));
            refreshPocketIpwDisplayMode();
        });

    QMenu* rotationModeMenu =
        m_displayMenu->addMenu(QStringLiteral("旋转方式"));
    QActionGroup* rotationModeGroup = new QActionGroup(rotationModeMenu);
    rotationModeGroup->setExclusive(true);
    QSettings viewSettings;
    const bool useModelRotation =
        viewSettings.value(viewSettingsKey(QStringLiteral("rotation_mode")),
                           QStringLiteral("camera")).toString() ==
        QStringLiteral("model");
    m_actCameraRotation =
        rotationModeMenu->addAction(QStringLiteral("相机环绕（默认）"));
    m_actCameraRotation->setCheckable(true);
    m_actCameraRotation->setChecked(!useModelRotation);
    rotationModeGroup->addAction(m_actCameraRotation);
    m_actModelRotation =
        rotationModeMenu->addAction(QStringLiteral("固定相机，旋转模型"));
    m_actModelRotation->setCheckable(true);
    m_actModelRotation->setChecked(useModelRotation);
    rotationModeGroup->addAction(m_actModelRotation);
    rotationModeMenu->addSeparator();
    QAction* invertModelVerticalAction =
        rotationModeMenu->addAction(QStringLiteral("固定相机：上下旋转反转"));
    invertModelVerticalAction->setCheckable(true);
    const bool invertModelVertical =
        viewSettings.value(
            viewSettingsKey(QStringLiteral("model_rotation_invert_vertical")),
            false).toBool();
    invertModelVerticalAction->setChecked(invertModelVertical);
    m_view->setModelRotationVerticalInverted(invertModelVertical);
    m_view->setRotationMode(useModelRotation
                                ? OcctView::RotationMode::Model
                                : OcctView::RotationMode::Camera);
    connect(m_actCameraRotation, &QAction::triggered, this, [this] {
        m_view->setRotationMode(OcctView::RotationMode::Camera);
        QSettings().setValue(viewSettingsKey(QStringLiteral("rotation_mode")),
                             QStringLiteral("camera"));
    });
    connect(m_actModelRotation, &QAction::triggered, this, [this] {
        m_view->setRotationMode(OcctView::RotationMode::Model);
        QSettings().setValue(viewSettingsKey(QStringLiteral("rotation_mode")),
                             QStringLiteral("model"));
    });
    connect(invertModelVerticalAction, &QAction::toggled,
            this, [this](bool inverted) {
        m_view->setModelRotationVerticalInverted(inverted);
        QSettings().setValue(
            viewSettingsKey(QStringLiteral("model_rotation_invert_vertical")),
            inverted);
    });

}

void MainWindow::setupToolbars()
{
    auto addToolbarToggle = [this](QToolBar *bar, const QString& text) {
        if (!m_toolbarMenu) return;
        QAction *act = bar->toggleViewAction();
        act->setText(text);
        m_toolbarMenu->addAction(act);
    };

    // Keep everyday camera commands on the toolbar; appearance controls share
    // a popup so they do not compete with the viewport for horizontal space.
    QToolBar *viewBar = addToolBar(QStringLiteral("视图"));
    viewBar->setObjectName(QStringLiteral("viewToolbar"));
    addToolbarToggle(viewBar, QStringLiteral("视图控制"));
    m_actFitAll->setIconText(QStringLiteral("适合窗口"));
    viewBar->addAction(m_actFitAll);
    if (m_actNearestStandardView) {
        m_actNearestStandardView->setIconText(QStringLiteral("标准视图"));
        viewBar->addAction(m_actNearestStandardView);
    }
    auto* appearanceMenu = new QMenu(QStringLiteral("显示"), viewBar);
    auto* appearanceButton = new QToolButton(viewBar);
    appearanceButton->setText(QStringLiteral("显示"));
    appearanceButton->setToolTip(QStringLiteral("模型外观、结构颜色和背景灰度"));
    appearanceButton->setPopupMode(QToolButton::InstantPopup);
    appearanceButton->setMenu(appearanceMenu);
    viewBar->addWidget(appearanceButton);
    auto* backgroundControl = new QWidget(appearanceMenu);
    auto* backgroundLayout = new QHBoxLayout(backgroundControl);
    backgroundLayout->setContentsMargins(12, 8, 12, 8);
    backgroundLayout->addWidget(new QLabel(QStringLiteral("背景"), backgroundControl));
    QSlider *graySlider = new QSlider(Qt::Horizontal, backgroundControl);
    graySlider->setRange(kGraySliderMin, kGraySliderMax);
    graySlider->setValue(int(m_view->backgroundGray() * kGraySliderScale));
    graySlider->setFixedWidth(kGraySliderWidth);
    graySlider->setToolTip(QStringLiteral("背景灰度:黑到白"));
    backgroundLayout->addWidget(graySlider);
    auto* backgroundAction = new QWidgetAction(appearanceMenu);
    backgroundAction->setDefaultWidget(backgroundControl);
    appearanceMenu->addAction(backgroundAction);
    appearanceMenu->addSeparator();
    connect(graySlider, &QSlider::valueChanged, this, [this](int v)
            { m_view->setBackgroundGray(v / kGraySliderScale); });
    m_actFaded = appearanceMenu->addAction(QStringLiteral("褪色"));
    m_actFaded->setCheckable(true);
    m_actFaded->setChecked(true);
    m_actFaded->setToolTip(QStringLiteral("以淡灰色显示普通面，使特征颜色更突出"));
    connect(m_actFaded, &QAction::toggled, this, &MainWindow::toggleFaded);

    // CAM 特征识别已进入左侧标准工作流。顶部识别工具栏只保留
    // 识别结果的显示与诊断命令，避免同一操作出现两个按钮入口。
    QToolBar *recogBar = addToolBar(QStringLiteral("识别"));
    recogBar->setObjectName(QStringLiteral("recognitionToolbar"));
    m_recognitionToolbar = recogBar;
    addToolbarToggle(recogBar, QStringLiteral("识别"));
    recogBar->hide();
    m_actRecognizeCamFeatures->setToolTip(
        QStringLiteral("统一识别孔、通结构和 Z→Y→X 内外环结构并染色；"
                       "孔与通结构优先从内外环追踪入口排除"));
    // 结构染色 / 原始颜色视图切换：识别完成后启用。
    m_actFeatureColors = appearanceMenu->addAction(QStringLiteral("结构颜色"));
    m_actFeatureColors->setCheckable(true);
    m_actFeatureColors->setChecked(false);
    m_actFeatureColors->setEnabled(false);
    m_actFeatureColors->setToolTip(
        QStringLiteral("在内外环结构识别染色与底色之间切换 (Q)"));
    connect(m_actFeatureColors, &QAction::toggled, this, &MainWindow::toggleFeatureColors);
    registerShortcut(QStringLiteral("view.feature_colors"), QStringLiteral("结构颜色"),
                     m_actFeatureColors, QKeySequence(Qt::Key_Q));
    // 基面识别:把与所属体 AABB 紧贴的平面(基面)找出来涂黑。
    QAction *baseBtn = recogBar->addAction(QStringLiteral("识别基面"));
    baseBtn->setToolTip(QStringLiteral("识别与包围盒齐平的面并涂黑"));
    connect(baseBtn, &QAction::triggered, this, &MainWindow::recognizeBaseFaces);
    // 显示特征识别所用的逐体 AABB 包围盒(线框叠加,黄色)。
    QAction *boxBtn = recogBar->addAction(QStringLiteral("显示包围盒"));
    boxBtn->setToolTip(QStringLiteral("显示特征识别所用的逐体 AABB 包围盒"));
    connect(boxBtn, &QAction::triggered, this, &MainWindow::showFeatureBoxes);
    m_displayMenu->addAction(baseBtn);
    m_displayMenu->addAction(boxBtn);
    // 显示隐藏工具栏:对当前选择(面或单元)执行 Hide / Only / Show / Show All。
    // 同样的命令也出现在 3D 右键菜单中,二者共用下面的槽。
    QToolBar *visBar = addToolBar(QStringLiteral("可见性"));
    visBar->setObjectName(QStringLiteral("visibilityToolbar"));
    m_visibilityToolbar = visBar;
    addToolbarToggle(visBar, QStringLiteral("可见性"));
    m_actHide = visBar->addAction(QStringLiteral("隐藏"));
    m_actHide->setToolTip(QStringLiteral("隐藏所选 (B)"));
    connect(m_actHide, &QAction::triggered, this, &MainWindow::hideSelected);
    registerShortcut(QStringLiteral("visibility.hide"), QStringLiteral("隐藏"),
                     m_actHide, QKeySequence(Qt::Key_B));

    m_actOnly = visBar->addAction(QStringLiteral("仅显示"));
    m_actOnly->setToolTip(QStringLiteral("仅显示所选 (Shift+H)"));
    connect(m_actOnly, &QAction::triggered, this, &MainWindow::onlySelected);
    registerShortcut(QStringLiteral("visibility.only"), QStringLiteral("仅显示"),
                     m_actOnly, QKeySequence(Qt::SHIFT | Qt::Key_H));

    m_actInvertShow = visBar->addAction(QStringLiteral("反显示"));
    m_actInvertShow->setToolTip(QStringLiteral("可见与隐藏对象互换 (Ctrl+B)"));
    connect(m_actInvertShow, &QAction::triggered, this, &MainWindow::invertShow);
    registerShortcut(QStringLiteral("visibility.invert"), QStringLiteral("反显示"),
                     m_actInvertShow, QKeySequence(Qt::CTRL | Qt::Key_B));

    m_actShow = visBar->addAction(QStringLiteral("显示"));
    m_actShow->setToolTip(QStringLiteral("显示所选(还原选择中被隐藏的对象)"));
    connect(m_actShow, &QAction::triggered, this, &MainWindow::showSelected);
    registerShortcut(QStringLiteral("visibility.show"), QStringLiteral("显示"),
                     m_actShow, QKeySequence(Qt::Key_U));

    m_actShowAll = visBar->addAction(QStringLiteral("全部显示"));
    m_actShowAll->setToolTip(QStringLiteral("全部显示 (A)"));
    connect(m_actShowAll, &QAction::triggered, this, &MainWindow::showAll);
    registerShortcut(QStringLiteral("visibility.show_all"), QStringLiteral("全部显示"),
                     m_actShowAll, QKeySequence(Qt::Key_A));

    m_actUndo = visBar->addAction(QStringLiteral("撤销"));
    m_actUndo->setToolTip(
        QStringLiteral("撤销上一次可见性或孔段工艺操作 (Ctrl+Z)"));
    connect(m_actUndo, &QAction::triggered, this, &MainWindow::undoVisibility);
    registerShortcut(QStringLiteral("edit.undo"), QStringLiteral("撤销"),
                     m_actUndo, QKeySequence::Undo);

    appearanceMenu->addSeparator();
    m_actShaded = appearanceMenu->addAction(QStringLiteral("着色"));
    m_actShaded->setCheckable(true);
    m_actShaded->setChecked(true);   // 默认着色,与 SceneController 初始模式一致
    m_actShaded->setToolTip(QStringLiteral("切换着色 / 线框 (T)"));
    connect(m_actShaded, &QAction::toggled, this, &MainWindow::toggleShaded);
    registerShortcut(QStringLiteral("view.shaded"), QStringLiteral("着色 / 线框"),
                     m_actShaded, QKeySequence(Qt::Key_T));

    m_actEdges = appearanceMenu->addAction(QStringLiteral("边线"));
    m_actEdges->setCheckable(true);
    m_actEdges->setChecked(true);
    m_actEdges->setToolTip(QStringLiteral("在着色模型上叠加显示面的边线 (Ctrl+E)"));
    connect(m_actEdges, &QAction::toggled, this, &MainWindow::toggleEdges);
    registerShortcut(QStringLiteral("view.edges"), QStringLiteral("普通边线"),
                     m_actEdges, QKeySequence(Qt::CTRL | Qt::Key_E));

    m_actConvexityEdges = appearanceMenu->addAction(QStringLiteral("凹凸边"));
    m_actConvexityEdges->setCheckable(true);
    m_actConvexityEdges->setToolTip(
        QStringLiteral("凹边红色、光顺边蓝色、凸边绿色"));
    connect(m_actConvexityEdges, &QAction::toggled,
            this, &MainWindow::toggleConvexityEdges);

    if (m_displayMenu) {
        m_displayMenu->addSeparator();
        m_displayMenu->addAction(m_actFaded);
        m_displayMenu->addAction(m_actShaded);
        m_displayMenu->addAction(m_actEdges);
        m_displayMenu->addAction(m_actConvexityEdges);
        m_displayMenu->addSeparator();
        m_displayMenu->addAction(m_actFeatureColors);
        m_actShowFeatureLegend =
            m_displayMenu->addAction(
                QStringLiteral("显示结构颜色图例"));
        m_actShowFeatureLegend->setCheckable(true);
        m_actShowFeatureLegend->setChecked(true);
        connect(m_actShowFeatureLegend, &QAction::toggled,
                this, [this] { refreshFeatureLegend(); });

    }

    // 选择模式工具栏(需求 §11.3 / §9.7 快捷键 1-4)。互斥单选,默认 Face。
    QToolBar *modeBar = addToolBar(QStringLiteral("选择模式"));
    modeBar->setObjectName(QStringLiteral("selectionModeToolbar"));
    m_modeToolbar = modeBar;
    addToolbarToggle(modeBar, QStringLiteral("选择模式"));
    modeBar->addWidget(new QLabel(QStringLiteral(" 模式 "), modeBar));
    // QActionGroup(exclusive)保证任意时刻只有一个 mode 处于选中态:
    // 勾选某个会自动取消其它,杜绝多重选定。
    QActionGroup *modeGroup = new QActionGroup(this);
    modeGroup->setExclusive(true);
    auto addMode = [&](const QString &text, const QString &key, auto slot) {
        QAction *a = modeBar->addAction(text);
        a->setCheckable(true);
        a->setToolTip(text + QStringLiteral(" 选择 (") + key + ")");
        connect(a, &QAction::triggered, this, slot);
        modeGroup->addAction(a); // 纳入互斥组
        registerShortcut(QStringLiteral("selection.%1").arg(key),
                         QStringLiteral("%1选择").arg(text), a, QKeySequence(key));
        return a;
    };
    QAction *mFace = addMode(QStringLiteral("面"), "1", &MainWindow::setSelectionModeFace);
    mFace->setToolTip(
        mFace->toolTip() +
        QStringLiteral("\nShift+点击：累加选择整个识别特征"));
    addMode(QStringLiteral("边"), "2", &MainWindow::setSelectionModeEdge);
    addMode(QStringLiteral("线环"), "3", &MainWindow::setSelectionModeWire);
    addMode(QStringLiteral("单元"), "4", &MainWindow::setSelectionModeUnit);
    mFace->setChecked(true);
}

void MainWindow::registerShortcut(const QString& id, const QString& label,
                                  QAction* action,
                                  const QKeySequence& defaultSequence)
{
    if (!action) return;
    QSettings settings;
    const QVariant saved = settings.value(shortcutSettingsKey(id));
    action->setShortcut(saved.isValid() ? QKeySequence(saved.toString())
                                        : defaultSequence);
    action->setShortcutContext(Qt::WindowShortcut);
    m_shortcuts.push_back({id, label, action, defaultSequence});
}

void MainWindow::applyShortcut(const QString& id, const QKeySequence& sequence)
{
    for (ShortcutEntry& entry : m_shortcuts) {
        if (entry.id != id || !entry.action) continue;
        entry.action->setShortcut(sequence);
        QSettings settings;
        settings.setValue(shortcutSettingsKey(id),
                          sequence.toString(QKeySequence::PortableText));
        return;
    }
}

void MainWindow::setupMouseBindings()
{
    registerMouseBinding(QStringLiteral("mouse.select"), QStringLiteral("选择 / 拾取"),
                         OcctView::MouseAction::Select, Qt::LeftButton);
    registerMouseBinding(QStringLiteral("mouse.box_select"), QStringLiteral("框选"),
                         OcctView::MouseAction::BoxSelect, Qt::LeftButton);
    registerMouseBinding(QStringLiteral("mouse.rotate"), QStringLiteral("旋转视图"),
                         OcctView::MouseAction::Rotate, Qt::MiddleButton);
    registerMouseBinding(QStringLiteral("mouse.pan"), QStringLiteral("平移视图"),
                         OcctView::MouseAction::Pan, Qt::RightButton);
    registerMouseBinding(QStringLiteral("mouse.context_menu"), QStringLiteral("3D 右键菜单"),
                         OcctView::MouseAction::ContextMenu, Qt::RightButton);
}

void MainWindow::registerMouseBinding(const QString& id, const QString& label,
                                      OcctView::MouseAction action,
                                      Qt::MouseButton defaultButton)
{
    QSettings settings;
    const Qt::MouseButton button =
        mouseButtonFromSettings(settings.value(mouseSettingsKey(id)), defaultButton);
    m_view->setMouseButton(action, button);
    m_mouseBindings.push_back({id, label, action, defaultButton});
}

void MainWindow::applyMouseBinding(const QString& id, Qt::MouseButton button)
{
    for (MouseBindingEntry& entry : m_mouseBindings) {
        if (entry.id != id) continue;
        m_view->setMouseButton(entry.action, button);
        QSettings settings;
        settings.setValue(mouseSettingsKey(id), int(button));
        return;
    }
}

void MainWindow::showShortcutSettings()
{
    std::vector<ShortcutSettingsDialog::Row> rows;
    rows.reserve(m_shortcuts.size() + m_mouseBindings.size());
    for (const ShortcutEntry& entry : m_shortcuts) {
        const QKeySequence current = entry.action ? entry.action->shortcut() : QKeySequence();
        rows.push_back({entry.id, entry.label,
                        ShortcutSettingsDialog::RowKind::Keyboard,
                        current, current, entry.defaultSequence});
    }
    for (const MouseBindingEntry& entry : m_mouseBindings) {
        ShortcutSettingsDialog::Row row;
        row.id = entry.id;
        row.label = entry.label;
        row.kind = ShortcutSettingsDialog::RowKind::MouseButton;
        row.currentMouse = row.editedMouse = m_view->mouseButton(entry.action);
        row.defaultMouse = entry.defaultButton;
        rows.push_back(row);
    }

    ShortcutSettingsDialog dialog(rows, this);
    if (dialog.exec() != QDialog::Accepted) return;
    const auto edited = dialog.rows();

    // 收集编辑后的绑定,做两轮冲突检查:同一按键/组合键不能分配给多个动作
    // (键盘),同一鼠标键最多允许一个单击动作 + 一个拖动动作(鼠标)。
    QHash<QString, QStringList> keyboardUse;
    QHash<int, QStringList> mouseIds;
    QHash<int, QStringList> mouseLabels;
    for (const auto& row : edited) {
        if (row.kind == ShortcutSettingsDialog::RowKind::Keyboard) {
            const QString key = row.edited.toString(QKeySequence::PortableText);
            if (!key.isEmpty()) keyboardUse[key].append(row.label);
        } else if (row.editedMouse != Qt::NoButton) {
            mouseIds[int(row.editedMouse)].append(row.id);
            mouseLabels[int(row.editedMouse)].append(row.label);
        }
    }
    for (auto it = keyboardUse.cbegin(); it != keyboardUse.cend(); ++it) {
        if (it.value().size() <= 1) continue;
        QMessageBox::warning(this, QStringLiteral("快捷键冲突"),
            QStringLiteral("%1 同时分配给：%2")
                .arg(QKeySequence(it.key()).toString(QKeySequence::NativeText),
                     it.value().join(QStringLiteral("、"))));
        return;
    }
    for (auto it = mouseIds.cbegin(); it != mouseIds.cend(); ++it) {
        if (isAllowedMouseSharing(it.value())) continue;
        QMessageBox::warning(this, QStringLiteral("鼠标按键冲突"),
            QStringLiteral("同一鼠标键只能分配一个单击动作和一个拖动动作：%1")
                .arg(mouseLabels.value(it.key()).join(QStringLiteral("、"))));
        return;
    }
    for (const auto& row : edited) {
        if (row.kind == ShortcutSettingsDialog::RowKind::MouseButton)
            applyMouseBinding(row.id, row.editedMouse);
        else
            applyShortcut(row.id, row.edited);
    }
    statusBar()->showMessage(QStringLiteral("按键与鼠标设置已更新"));
}

void MainWindow::navigateRelativeFile(int delta)
{
    if (!m_uiBusy && m_fileList && m_fileList->activateRelativeFile(delta))
        updateFileNavigationActions();
}

void MainWindow::updateFileNavigationActions()
{
    const bool ready = !m_uiBusy && m_fileList && m_fileList->isEnabled();
    if (m_actPrevFile)
        m_actPrevFile->setEnabled(ready && m_fileList->canActivateRelativeFile(-1));
    if (m_actNextFile)
        m_actNextFile->setEnabled(ready && m_fileList->canActivateRelativeFile(1));
}

void MainWindow::setupDocks()
{
    setDockOptions(AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
    setTabPosition(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea, QTabWidget::North);
    setupUiSlices();
    // One shared sidebar leaves the model the full remaining width. Docks can
    // still be detached or dragged to the right for a two-column workspace.
    for (QDockWidget* dock : {m_analysisDock, m_featureDock, m_groupDock}) {
        addDockWidget(Qt::LeftDockWidgetArea, dock);
        tabifyDockWidget(m_camWorkflowDock, dock);
    }
    resizeDocks({m_camWorkflowDock}, {340}, Qt::Horizontal);
    m_camWorkflowDock->raise();
    m_playbackUi->dock().hide();

    auto* workspaceBar = addToolBar(QStringLiteral("工作区"));
    workspaceBar->setObjectName(QStringLiteral("workspaceToolbar"));
    workspaceBar->addAction(m_playbackUi->dock().toggleViewAction());
    m_toolbarMenu->addAction(workspaceBar->toggleViewAction());
    for (QToolBar* bar : findChildren<QToolBar*>()) {
        bar->setStyleSheet(QStringLiteral(
            "QToolBar { spacing: 3px; padding: 2px 3px; }"
            "QToolButton { padding: 4px 5px; }"));
    }
}

void MainWindow::refreshCamWorkflowPanel()
{
    if (!m_camWorkflowPanel || !m_machiningWorkspace)
        return;

    oap::app::CamWorkflowSummary summary;
    summary.hasModel = !model().faces.empty();
    summary.modelName = currentPath().isEmpty()
        ? QString() : QFileInfo(currentPath()).fileName();
    summary.projectName = m_currentMachiningJobPath.isEmpty()
        ? QString()
        : QFileInfo(m_currentMachiningJobPath).fileName();
    summary.busy = m_uiBusy;
    summary.hasProjectContent = m_machiningWorkspace->hasProjectContent();
    summary.pendingRegeneration = m_machiningWorkspace->hasPendingRegeneration();
    {
        switch (m_machiningWorkspace->activePreview()) {
        case ActiveToolpathPreview::Hole:
            summary.activeProgramAvailable = m_machiningWorkspace->hasHoleProgram();
            summary.activeProgramName = QStringLiteral("孔加工");
            break;
        case ActiveToolpathPreview::Pocket:
            summary.activeProgramAvailable = m_machiningWorkspace->currentPocketResult() != nullptr;
            summary.activeProgramName = QStringLiteral("槽腔");
            break;
        case ActiveToolpathPreview::Profile:
            summary.activeProgramAvailable = m_machiningWorkspace->hasProfilePreview();
            summary.activeProgramName = QStringLiteral("外轮廓");
            break;
        case ActiveToolpathPreview::SurfaceFinishing:
            summary.activeProgramAvailable = m_machiningWorkspace->hasFinishingPreview();
            summary.activeProgramName = QStringLiteral("雕刻");
            break;
        case ActiveToolpathPreview::None:
            break;
        }
    }
    if (m_machiningWorkspace->activePreviewNeedsRegeneration())
        summary.activeProgramAvailable = false;

    if (m_recognitionWorkspace) {
        summary.recognitionReady = m_recognitionWorkspace->hasResult();
        summary.recognizedFeatureCount = static_cast<int>(
            m_machiningWorkspace->recognizedHoleSourceCount() +
            m_recognitionWorkspace->pocketSources().size());
    }

    if (const oap::tooling::ToolLibrary* library = activeToolLibrary()) {
        summary.activeToolLibraryName = library->name();
        summary.assignedToolCount = library->assignedTools().size();
    }
    const bool activeLibraryHasAssignedTools =
        summary.assignedToolCount > 0;
    const auto toolAvailable = [&](const QString& family, oap::app::ToolSelectionMode mode,
                                    const oap::toolpath::ToolpathToolContext& tool) {
        if (mode != oap::app::ToolSelectionMode::SingleTool)
            return activeLibraryHasAssignedTools;
        const bool ready = tool.valid && tool.toolNumber > 0 && tool.diameter > 0 &&
            tool.cuttingEdgeLength > 0 && tool.spindleSpeed > 0 &&
            tool.horizontalFeed > 0 && tool.verticalFeed > 0;
        summary.directToolReady = summary.directToolReady || ready;
        if (!summary.directToolSummary.isEmpty()) summary.directToolSummary += QLatin1Char('\n');
        summary.directToolSummary += QStringLiteral("%1单刀：T%2 · Ø%3 mm")
            .arg(family).arg(tool.toolNumber).arg(tool.diameter, 0, 'f', 3);
        return ready;
    };
    const bool holeToolAvailable = toolAvailable(QStringLiteral("孔"),
        m_machiningWorkspace->holeToolSelectionMode(), m_machiningWorkspace->holeSingleTool());
    const bool pocketToolAvailable = toolAvailable(QStringLiteral("槽腔"),
        m_machiningWorkspace->pocketToolSelectionMode(), m_machiningWorkspace->pocketSingleTool());
    const bool profileToolAvailable = toolAvailable(QStringLiteral("外轮廓"),
        m_machiningWorkspace->profileToolSelectionMode(), m_machiningWorkspace->profileSingleTool());
    summary.holeGenerationReady =
        oap::app::holeToolpathGenerationAvailable(
            summary.recognitionReady,
            holeToolAvailable,
            summary.busy);
    summary.pocketGenerationReady =
        oap::app::pocketToolpathGenerationAvailable(
            summary.recognitionReady,
            pocketToolAvailable, summary.busy);
    summary.profileGenerationReady =
        oap::app::toolpathGenerationAvailable(
            summary.recognitionReady,
            profileToolAvailable, summary.busy);

    summary.setupFramesValid = moldSetupWorkFrames().isValid();
    const auto& engraving = m_machiningWorkspace->engravingSettings();
    if (engraving.toolMode == oap::app::EngravingToolMode::SingleTool) {
        summary.engravingToolReady = engraving.manualTool.valid &&
            engraving.manualTool.toolNumber > 0 && engraving.manualTool.diameter > 0 &&
            engraving.manualTool.cuttingEdgeLength > 0;
        summary.engravingToolSummary = QStringLiteral("雕刻单刀：T%1 · Ø%2 mm")
            .arg(engraving.manualTool.toolNumber).arg(engraving.manualTool.diameter, 0, 'f', 3);
    } else {
        const oap::tooling::ToolLibrary* engravingLibrary = activeToolLibrary();
        if (!engraving.libraryId.empty()) {
            engravingLibrary = nullptr;
            for (const auto& library : m_machiningWorkspace->toolLibraries()) {
                if (library.id().toStdString() == engraving.libraryId) {
                    engravingLibrary = &library;
                    break;
                }
            }
        }
        if (engravingLibrary) {
            const auto assigned = engravingLibrary->assignedTools();
            summary.engravingToolReady = std::any_of(assigned.begin(), assigned.end(), [](const auto& tool) {
                return tool.tool.type == oap::tooling::ToolType::BallEndMill ||
                    tool.tool.type == oap::tooling::ToolType::EndMill ||
                    tool.tool.type == oap::tooling::ToolType::BullNose;
            });
            summary.engravingToolSummary = QStringLiteral("雕刻刀库：%1").arg(engravingLibrary->name());
        } else {
            summary.engravingToolSummary = QStringLiteral("雕刻刀库不可用，请在雕刻参数页选择刀库或设置单刀。");
        }
    }
    summary.finishingGenerationReady = summary.hasModel && summary.setupFramesValid && !summary.busy;
    const std::array<oap::MoldSetupFace, 6> setupFaces{
        oap::MoldSetupFace::PositiveZ,
        oap::MoldSetupFace::PositiveX,
        oap::MoldSetupFace::PositiveY,
        oap::MoldSetupFace::NegativeX,
        oap::MoldSetupFace::NegativeY,
        oap::MoldSetupFace::NegativeZ,
    };
    for (const oap::MoldSetupFace face : setupFaces) {
        const oap::toolpath::MoldFaceSetupWorkFrame* frame =
            moldSetupWorkFrames().forFace(face);
        if (!frame || !frame->valid)
            continue;
        ++summary.availableSetupCount;
        if (frame->confirmed)
            ++summary.confirmedSetupCount;
    }

    const oap::app::MachiningWorkspaceStatus machiningStatus =
        m_machiningWorkspace->status();
    summary.holeOperationCount = machiningStatus.holeOperationCount;
    summary.pocketOperationCount = machiningStatus.pocketOperationCount;
    summary.profileOperationCount = machiningStatus.profileOperationCount;
    summary.finishingOperationCount = machiningStatus.finishingOperationCount;
    summary.coverageAudited = machiningStatus.coverageAudited;
    summary.coveragePassed = machiningStatus.coveragePassed;
    summary.coverageHasWarnings = machiningStatus.coverageHasWarnings;
    summary.allowanceAvailable = m_stockSimulationWorkspace &&
        !m_stockSimulationWorkspace->featureAllowanceItems().empty();

    m_camWorkflowPanel->setSummary(summary);

    const bool hasProfileProgram = summary.profileOperationCount > 0;
    const bool hasProgram = summary.holeOperationCount > 0 ||
                            summary.pocketOperationCount > 0 ||
                            hasProfileProgram || summary.finishingOperationCount > 0;
    if (m_actOpenMachiningJob)
        m_actOpenMachiningJob->setEnabled(
            summary.hasModel && !summary.busy);
    if (m_actSaveMachiningJob)
        m_actSaveMachiningJob->setEnabled(summary.hasProjectContent && !summary.pendingRegeneration && !summary.busy);
    if (m_actToolLibrary)
        m_actToolLibrary->setEnabled(!summary.busy);
    if (m_actRecognizeCamFeatures)
        m_actRecognizeCamFeatures->setEnabled(summary.hasModel && !summary.busy);
    if (m_actToolpathParams)
        m_actToolpathParams->setEnabled(summary.hasModel && !summary.busy);
    if (m_actHolePaths)
        m_actHolePaths->setEnabled(summary.holeGenerationReady);
    if (m_actPocketClearing)
        m_actPocketClearing->setEnabled(summary.pocketGenerationReady);
    if (m_actProfileContour)
        m_actProfileContour->setEnabled(summary.profileGenerationReady);
    if (m_actSurfaceFinishing)
        m_actSurfaceFinishing->setEnabled(summary.finishingGenerationReady);
    if (m_actProjectStock)
        m_actProjectStock->setEnabled(hasProgram && !summary.pendingRegeneration && !summary.busy);
    if (m_actEngravingStock)
        m_actEngravingStock->setEnabled(summary.finishingOperationCount > 0 && !summary.busy);
    if (m_actHoleStock)
        m_actHoleStock->setEnabled(
            summary.holeOperationCount > 0 && !summary.busy);
    if (m_actPocketStock)
        m_actPocketStock->setEnabled(
            summary.pocketOperationCount > 0 && !summary.busy);
    if (m_actProfileStock)
        m_actProfileStock->setEnabled(
            summary.profileOperationCount > 0 && !summary.busy);
    if (m_actFeatureAllowance)
        m_actFeatureAllowance->setEnabled(
            summary.allowanceAvailable && !summary.busy);
    if (m_actExportToolpath)
        m_actExportToolpath->setEnabled(summary.activeProgramAvailable && !summary.busy);
}

void MainWindow::connectSignals()
{
    connect(m_view, &OcctView::entityPicked, this, &MainWindow::onEntityPicked);
    connect(m_view, &OcctView::entitiesBoxPicked,
            this, &MainWindow::onEntitiesBoxPicked);
    connect(m_view, &OcctView::selectionCleared,
            this, &MainWindow::onSelectionCleared);
    connect(m_view, &OcctView::toolpathSegmentPicked,
            this, [this](const QString& description,
                         const QString& operationId, int) {
                statusBar()->showMessage(
                    description.simplified(), 12000);
                if (m_holeToolTreePanel && !operationId.isEmpty()) {
                    const QString formalOperationId =
                        m_machiningWorkspace->formalOperationIdForRuntime(
                            operationId);
                    synchronizeFormalOperationSelection(
                        formalOperationId.isEmpty()
                            ? operationId : formalOperationId);
                    m_holeToolTreePanel->selectOperation(operationId);
                }
            });
    connect(m_view, &OcctView::escapePressed,
            this, &MainWindow::clearHoleStageHighlights);
    connect(m_view, &OcctView::contextMenuRequested, this, &MainWindow::showContextMenu);
    connect(m_view, &OcctView::modelTransformChanged, this, [this] {
        sceneController().setModelTransform(m_view->modelTransform());
        applyModelTransformToOverlays();
        if (!m_view->context().IsNull() &&
            m_view->context()->NbSelected() > 0) {
            m_view->context()->UnhilightSelected(Standard_False);
            m_view->context()->HilightSelected(Standard_False);
        }
        if (!m_view->view().IsNull()) {
            m_view->view()->Invalidate();
            m_view->view()->Redraw();
        }
    });
}

void MainWindow::resetViewLayout()
{
    if (m_defaultViewLayoutState.isEmpty()) {
        statusBar()->showMessage(QStringLiteral("默认视图布局尚未初始化"));
        return;
    }
    setUpdatesEnabled(false);
    const bool restored = restoreState(
        m_defaultViewLayoutState, kDefaultViewLayoutStateVersion);
    if (m_camWorkflowPanel)
        m_camWorkflowPanel->resetSplitterLayout();
    setUpdatesEnabled(true);
    if (restored && m_camWorkflowDock)
        m_camWorkflowDock->raise();
    update();
    statusBar()->showMessage(
        restored ? QStringLiteral("已恢复默认视图布局")
                 : QStringLiteral("恢复默认视图布局失败"));
}

void MainWindow::showContextMenu(const QPoint &globalPos)
{
    if (model().faces.empty()) return;
    const bool hasSel = !currentSelectedFaceIds().empty();
    QMenu menu(this);
    QAction *aHide = menu.addAction(QStringLiteral("隐藏所选"));
    QAction *aOnly = menu.addAction(QStringLiteral("仅显示所选"));
    QAction *aShow = menu.addAction(QStringLiteral("显示所选"));
    menu.addSeparator();
    QAction *aInvert = menu.addAction(QStringLiteral("反显示"));
    QAction *aAll = menu.addAction(QStringLiteral("全部显示"));
    QAction *aClearSelection = menu.addAction(QStringLiteral("清除选择"));
    aHide->setEnabled(hasSel);
    aOnly->setEnabled(hasSel);
    aShow->setEnabled(hasSel && sceneController().hiddenCount() > 0);
    aAll->setEnabled(sceneController().hiddenCount() > 0);
    aClearSelection->setEnabled(hasSel);

    menu.addSeparator();
    struct KindItem { const char* label; oap::FeatureKind kind; };
    static const KindItem kItems[] = {
        {"通孔",     oap::FeatureKind::ThroughHole},
        {"盲孔",       oap::FeatureKind::BlindHole},
        {"沉孔", oap::FeatureKind::CountersinkHole},
        {"槽",             oap::FeatureKind::Slot},
        {"腔",           oap::FeatureKind::Pocket},
    };
    std::vector<QAction*> typeActs;
    std::vector<KindItem> activeKinds;
    QAction* aClearType = nullptr;
    if (m_machiningWorkspace->activePreview() == ActiveToolpathPreview::SurfaceFinishing) {
        statusBar()->showMessage(QStringLiteral("当前显示：雕刻加工刀路"));
    } else if (m_machiningWorkspace->activePreview() == ActiveToolpathPreview::Profile) {
        statusBar()->showMessage(QStringLiteral("当前显示：外轮廓刀路"));
    } else if (m_machiningWorkspace->activePreview() == ActiveToolpathPreview::Pocket) {
        QAction* sourceInfo = menu.addAction(
            QStringLiteral("清腔刀路仅来自主外表面内环"));
        sourceInfo->setEnabled(false);
    } else if (m_machiningWorkspace->activePreview() == ActiveToolpathPreview::Hole) {
        statusBar()->showMessage(QStringLiteral("当前显示：自动孔加工刀路"));
    } else {
        QMenu* typeMenu = menu.addMenu(QStringLiteral("设置特征类型"));
        typeMenu->setEnabled(hasSel);
        activeKinds.assign(std::begin(kItems), std::end(kItems));
        for (const KindItem& item : activeKinds)
            typeActs.push_back(typeMenu->addAction(QString::fromUtf8(item.label)));
        aClearType = menu.addAction(QStringLiteral("清除特征类型"));
        aClearType->setEnabled(hasSel);
    }

    QAction *chosen = menu.exec(globalPos);
    if (chosen == aHide) hideSelected();
    else if (chosen == aOnly) onlySelected();
    else if (chosen == aShow) showSelected();
    else if (chosen == aInvert) invertShow();
    else if (chosen == aAll) showAll();
    else if (chosen == aClearSelection) onSelectionCleared();
    else if (aClearType && chosen == aClearType) clearFeatureTypeForSelection();
    else {
        for (size_t i = 0; i < typeActs.size(); ++i)
            if (chosen == typeActs[i]) {
                setFeatureTypeForSelection(activeKinds[i].kind);
                break;
            }
    }
}

void MainWindow::setVisibilityActionsEnabled()
{
    const bool hasSel = !currentSelectedFaceIds().empty();
    const bool hasHidden = sceneController().hiddenCount() > 0;
    if (m_actHide) m_actHide->setEnabled(hasSel);
    if (m_actOnly) m_actOnly->setEnabled(hasSel);
    if (m_actShow) m_actShow->setEnabled(hasSel && hasHidden);
    if (m_actShowAll) m_actShowAll->setEnabled(hasHidden);
    if (m_actInvertShow) m_actInvertShow->setEnabled(sceneController().faceCount() > 0);
    if (m_actUndo) m_actUndo->setEnabled(!m_undoHistory.empty());
}

void MainWindow::updateStatusBar()
{
    const auto sel = currentSelectedFaceIds();
    QString mode = QStringLiteral("面");
    switch (m_view->selectionMode()) {
        case OcctView::SelMode::Face: mode = QStringLiteral("面"); break;
        case OcctView::SelMode::Edge: mode = QStringLiteral("边"); break;
        case OcctView::SelMode::Wire: mode = QStringLiteral("线环"); break;
        case OcctView::SelMode::Unit: mode = QStringLiteral("单元"); break;
    }
    statusBar()->showMessage(
        QStringLiteral("模式: %1   已选: %2 个面   已隐藏: %3 / %4 个面")
            .arg(mode)
            .arg((int)sel.size())
            .arg((int)sceneController().hiddenCount())
            .arg((int)sceneController().faceCount()));
}
