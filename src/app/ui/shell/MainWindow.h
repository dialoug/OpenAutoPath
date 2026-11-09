// MainWindow:桌面应用主窗口(QMainWindow),是 src/app/ui/shell 层的组合根与
// 事件适配器。职责:装配 OcctView、各停靠面板与工具栏,并把 Qt 控件事件翻译为
// 对各 WorkspaceCoordinator/Session 的调用(加载、识别、加工、仿真、播放)。
// 本类不持有长期业务状态——文档/识别/加工/仿真/播放状态分别归
// ModelDocumentWorkspaceCoordinator、RecognitionWorkspaceCoordinator、
// MachiningWorkspaceCoordinator、StockSimulationWorkspaceCoordinator、
// PlaybackWorkspaceCoordinator 等所有(MainWindow 仅以 unique_ptr 持有它们)。
// 生命周期:coordinator 各自拥有 Session 与 worker 线程;析构时本窗口逐个
// cancelAndWait 等待工作线程安全结束后再释放。
#pragma once

#include <QMainWindow>
#include <QElapsedTimer>
#include "app/presentation/StepLoadPreviewPresenter.h"

#include <TDocStd_Document.hxx>
#include <TopoDS_Shape.hxx>

#include "analysis/Model.h"
#include "analysis/ModelIndex.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/StepIndexer.h"
#include "manufacturing/job/MachiningJob.h"
#include "recognition/FeatureRecognizer.h"
#include "toolpath/core/MachiningSetup.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathMoves.h"
#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/workflow/ToolpathWorkflow.h"
#include "tooling/library/ToolLibrary.h"
#include "app/panels/FeaturePanel.h"
#include "app/document/ModelDocumentWorkspaceCoordinator.h"
#include "app/scene/SceneInteractionCoordinator.h"
#include "app/ui/view/OcctView.h"
#include "app/machining/MachiningWorkspaceCoordinator.h"
#include "app/workflows/project/MachiningProjectController.h"
#include "app/workflows/shared/UndoHistory.h"

#include <AIS_InteractiveObject.hxx>
#include <NCollection_Sequence.hxx>

#include <vector>
#include <utility>
#include <string>
#include <memory>
#include <optional>
#include <unordered_set>
#include <unordered_map>

#include <QKeySequence>

class ResultPanel;
class GroupPanel;
class FileListPanel;
class HoleToolTreePanel;
class HoleCoverageAuditPanel;
class FeatureAllowancePanel;
class QAction;
class QActionGroup;
class QDockWidget;
class QComboBox;
class QLabel;
class QListWidget;
class QMenu;
class QSlider;
class QTimer;
class QToolBar;
class QProgressDialog;
class QCloseEvent;
namespace oap {
struct ThreeAxisWireTraceUseCaseResult;
namespace app { class MainWindowUiBuilder; }
namespace app { enum class ToolpathParameterEditorPage; }
namespace app { class CamWorkflowPanel; }
namespace app { class PlaybackWorkspaceCoordinator; }
namespace app { class PlaybackUiController; }
namespace app { class RecognitionSelectionUiController; }
namespace app { class DocumentUiController; }
namespace app { class CamUiController; }
namespace app { class SimulationUiController; }
namespace app { class DesktopWorkspaceCoordinator; }
namespace app { struct HoleStageHighlightStats; }
namespace app { class RecognitionWorkspaceCoordinator; }
namespace app { class StockSimulationWorkspaceCoordinator; }
namespace app::presentation {
class OverlayController;
}
}

// 主窗口：UI 组合根与跨工作区命令适配器，不持有长生命周期状态。
class MainWindow : public QMainWindow
{
    Q_OBJECT
    friend class oap::app::MainWindowUiBuilder;
public:
    MainWindow();
    ~MainWindow() override;   // 退出工作线程并等待其结束

    // 提交 STEP 加载请求。若原生 OCCT 查看器尚未初始化(窗口未显示),
    // 路径先存入 m_pendingViewerLoadPath,待 viewerInitialized 后自动提交。
    void loadFile(const QString& path);

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void openStep();
    void openFolder();                               // 打开文件夹，侧边栏列出其中的 .stp 文件
    // 批量缓存构建进度回调(worker 线程 emit,队列连接到 UI 线程)。
    void onCacheBuildProgress(
        int done, int total,
        const QString& currentFile);
    void onCacheBuildFinished(
        int built, int skipped,
        int failed, bool cancelled);
    void recognizeInnerOuterStructures();            // 统一识别孔、通结构和三向内外环结构
    void showHolePaths();                            // 识别孔并把每个孔的沿轴刀路画为叠加线段
    void simulateHoleStock();                        // 生成/导入初始毛坯并逐孔减料
    void simulatePocketStock();                      // 按当前腔刀路逐工序减料
    void simulateProfileStock();                     // 按当前外轮廓刀路执行统一 IPW
    void showFeatureAllowanceView();                 // 按特征查看加工后的局部余量
    void openToolLibrary();                          // 浏览/维护刀库目录，不隐式切换活动库
    void selectToolLibrary();                        // 选择参与 CAM 自动选刀的活动刀库
    void configureToolpathParams();                  // 配置项目孔/槽腔刀路、装夹与输出参数
    void showPocketClearing();                       // 为主外表面内环追踪结构生成逐层截面清腔线
    void showProfileContour();                       // 自动识别主外表面 outer wire 并生成外轮廓刀路
    void generateEngravingToolpaths();               // 直接按统一参数生成雕刻工序
    void configureEngravingParams();                  // 打开统一参数页的雕刻配置
    void simulateProjectStock();
    void simulateEngravingStock();
    void simulateOrderedStock(bool engravingOnly);
    void exportCurrentToolpathPreview();             // 导出当前 Pocket/Hole 预览程序
    void saveMachiningJob();                         // 保存孔/槽腔加工定义，不保存预览几何
    void openMachiningJob();                         // 加载定义并从当前零件重建派生刀路
    // 刀路播放器信号:走刀位置变化 -> 只重绘当前段 + 刀尖标记;
    // 已完成段隐藏，序列清空时清叠加。
    void clearHoleStageHighlights();
    void toggleFeatureColors(bool on);               // 在“识别染色 / 原始颜色”两种视图间切换
    void recognizeBaseFaces();                       // 识别与包围盒紧贴的基面并染黑显示
    void showFeatureBoxes();                         // 显示特征识别所用的逐体 AABB 包围盒(线框叠加)
    // 3D 拾取(face/edge/wire) -> 同步选择。additive=true(按住 Ctrl)时面级拾取
    // 追加/切换到现有选择,而非清空重选。
    void onEntityPicked(const TopoDS_Shape& shape, int shapeType,
                        bool additive = false, bool wholeFeature = false);
    void onSelectionCleared();
    void onEntitiesBoxPicked(const std::vector<TopoDS_Shape>& shapes,
                             int shapeType);
    void onTreeFaceSelected(const QString& faceId); // 树中拾取 -> 在 3D 中高亮
    // 识别面板选择变化 -> 把选择来源切到识别面板,并在 3D 高亮其面集合。
    void onFeatureSelectionChanged();
    // 分组面板选择变化 -> 把选择来源切到分组面板,并在 3D 高亮其面集合。
    void onGroupSelectionChanged();

    // 显示隐藏(face / unit 粒度)。统一从当前选择读取目标对象。
    void hideSelected();
    void onlySelected();
    void showAll();
    void showSelected();   // 把当前选择中被隐藏的对象重新显示出来
    void invertShow();
    void undoVisibility();  // Ctrl+Z:撤回可见性操作
    void toggleShaded(bool shaded);   // 线框 / 着色 显示模式切换
    void toggleEdges(bool on);        // 着色时叠加面边界(线框+着色同时)
    void toggleConvexityEdges(bool on);
    void toggleFaded(bool on);
    void snapToNearestStandardView();
    void focusCurrentSelection();
    void resetViewLayout();           // 恢复默认 Dock / Toolbar 排列
    void showContextMenu(const QPoint& globalPos);   // 3D 右键菜单
    void setSelectionModeFace();
    void setSelectionModeEdge();
    void setSelectionModeWire();
    void setSelectionModeUnit();
    void showShortcutSettings();

private:
    void bindViewerDependencies();

    const Handle(TDocStd_Document)& document() const noexcept
    {
        return m_documentWorkspace->document();
    }
    const oap::Model& model() const noexcept
    {
        return m_documentWorkspace->model();
    }
    const oap::ModelIndex& modelIndex() const noexcept
    {
        return m_documentWorkspace->modelIndex();
    }
    const oap::StepIndexer& stepIndexer() const noexcept
    {
        return m_documentWorkspace->stepIndexer();
    }
    const oap::MoldCoordinateFrame& moldFrame() const noexcept
    {
        return m_documentWorkspace->moldFrame();
    }
    const oap::toolpath::MoldSetupWorkFrames&
    moldSetupWorkFrames() const noexcept
    {
        return m_documentWorkspace->setupWorkFrames();
    }
    const QString& currentPath() const noexcept
    {
        return m_documentWorkspace->sourcePath();
    }
    const oap::Bbox& documentBounds() const noexcept
    {
        return m_documentWorkspace->bounds();
    }

    // 构造期 UI 装配,从构造函数拆出以分离“建界面”与“接业务”。
    void setupMenus();        // File / View 菜单骨架
    void setupToolbars();     // View / Recognition / Visibility / Select Mode 工具栏
    void setupDocks();        // Analysis / Features / Nearest Neighbors / Files 停靠面板
    void setupUiSlices();     // 仅装配各独立 UI 控制器
    void connectSignals();    // 信号连接 + 缓存/格式初始化
    void registerShortcut(const QString& id, const QString& label, QAction* action,
                          const QKeySequence& defaultSequence);
    void applyShortcut(const QString& id, const QKeySequence& sequence);
    void setupMouseBindings();
    void registerMouseBinding(const QString& id, const QString& label,
                              OcctView::MouseAction action,
                              Qt::MouseButton defaultButton);
    void applyMouseBinding(const QString& id, Qt::MouseButton button);
    void navigateRelativeFile(int delta);
    void updateFileNavigationActions();
    void refreshFeatureLegend();
    bool fitModelInView();
    bool fitViewToFaces(const std::vector<std::string>& faceIds);
    void applyModelTransformToOverlays();
    // 分析 worker 在工作线程算完后回到 UI 线程的交接点:关闭旧 doc、交换成员、
    // 显示模型、刷新面板;失败/空模型则弹窗。随后解除忙碌状态。
    void onAnalysisFinished(
        std::shared_ptr<AnalysisLoadResult> result);
    void onAnalysisPreview(std::shared_ptr<AnalysisLoadPreview> preview);
    void handleAnalysisFinished(
        std::shared_ptr<AnalysisLoadResult> result);
    // 进入/退出加载忙碌态:切换等待光标、状态栏提示,并禁用会触发新加载的入口。
    void setLoadingState(bool loading);
    void finishLoadingState(const QString& requestedPath = {});
    void resetForDocumentReplacement();
    void loadToolLibraries();
    const oap::tooling::ToolLibrary* activeToolLibrary() const noexcept;
    // 手动入口显示完成对话框；加载完成后的自动入口只更新场景和状态栏。
    void runInnerOuterStructureRecognition(bool showCompletionDialog);
    bool presentThreeAxisWireTraceResult(
        const oap::RecognitionResult& recognition,
        oap::ThreeAxisWireTraceUseCaseResult useCaseResult,
        bool showCompletionDialog,
        qint64 elapsedMillis);

    // 可见性命令收尾:清理叠加后刷新视图与动作状态(四个可见性槽共用)。
    void refreshAfterVisibility();
    // 在改变可见性前调用:把当前隐藏集压入撤回栈(Ctrl+Z 用),超上限丢弃最旧一条。

    // 人工复核纠正:把当前选择的面改标为 kind / 清除其特征标记。直接编辑
    // m_recognition.faceKind，随后重新染色并刷新识别面板。
    void setFeatureTypeForSelection(oap::FeatureKind kind);
    void clearFeatureTypeForSelection();
    // 应用统一内外环结构颜色：轴向面/路径、通结构、孔依次覆盖。
    void applyRecognitionColors();
    // 按当前统一结构识别结果重置并重新染色未隐藏面，刷新识别面板。
    void refreshRecognitionDisplay();
    // 清除上一次拾取留下的临时 AIS 对象(高亮的邻接面 + 见证线)。
    void clearOverlay(bool resetHolePathAction = true);
    void clearTransientOverlay(bool resetHoleStageHighlight = true);
    void clearToolpathOverlay();
    void refreshPocketIpwDisplayMode();
    bool showPocketIpwTimeline(bool allowOpenGl = true);
    void showHoleOpenGlSimulator();
    bool buildHoleInitialStock(double marginX, double marginY, double marginZ,
                               bool perUnit, TopoDS_Shape& stock,
                               int& stockBlockCount, QString& error) const;
    bool buildHoleTargetShape(TopoDS_Shape& target, int& targetSolidCount,
                              QString& error) const;

    // 刀路播放工作区负责 Session、worker 与 Presenter；窗口只应用值帧适配结果。
    void rebuildHoleToolTree();
    void refreshHoleCoverageAudit();
    void refreshFeatureAllowanceView();
    void refreshCamWorkflowPanel();
    oap::app::WorkflowInteraction makeWorkflowInteraction();
    void synchronizeInvalidatedToolpathPreview();
    void locateHoleCoverageSource(int sourceHoleIndex);
    void synchronizeFormalOperationSelection(
        const QString& formalOperationId);
    // 为给定的面对键绘制 OCC 见证线,作为临时叠加显示。
    // 在 3D 中高亮一组面(选中其按面 AIS;隐藏的面跳过),并设置状态栏文本。
    void highlightFaces(const std::vector<std::string>& faceIds, const QString& status);
    // 返回包含命中面的全部统一结构实例面：孔/人工实例、通结构或内外环子集。
    std::vector<std::string> featureFacesContaining(
        const std::string& faceId) const;

    // 当前选择集合(展开为 face_id)。读取 ResultPanel 的树选择:
    // unit 节点展开为其全部面,face 节点取自身;3D 拾取已回写树选择故二者同步。
    std::vector<std::string> currentSelectedFaceIds() const;
    oap::app::SelectionSnapshot currentSelectionSnapshot() const;
    oap::SceneController& sceneController() noexcept;
    const oap::SceneController& sceneController() const noexcept;
    void updateStatusBar();                          // 刷新选择数 / 隐藏数
    void setVisibilityActionsEnabled();              // 按当前选择/数据启用禁用

    OcctView*                 m_view = nullptr;
    ResultPanel*              m_resultPanel = nullptr;
    QDockWidget*              m_analysisDock = nullptr;  // 承载 m_resultPanel
    FeaturePanel*             m_featurePanel = nullptr; // 自动孔识别 + 人工特征结果树
    QDockWidget*              m_featureDock = nullptr;  // 承载 m_featurePanel,识别后 raise
    GroupPanel*               m_groupPanel = nullptr;   // 按名字前缀分组的 unit 树
    QDockWidget*              m_groupDock = nullptr;    // 承载 m_groupPanel
    FileListPanel*            m_fileList = nullptr; // 侧边栏:当前文件夹中的 .stp 文件列表
    QDockWidget*              m_fileDock = nullptr; // 承载 m_fileList

    QMenu*                    m_viewMenu = nullptr;
    QMenu*                    m_displayMenu = nullptr;
    QMenu*                    m_panelMenu = nullptr;
    QMenu*                    m_toolbarMenu = nullptr;

    QToolBar*                 m_recognitionToolbar = nullptr;
    QToolBar*                 m_visibilityToolbar = nullptr;
    QToolBar*                 m_modeToolbar = nullptr;

    std::unique_ptr<oap::app::ModelDocumentWorkspaceCoordinator>
        m_documentWorkspace;
    oap::app::SceneInteractionCoordinator m_sceneInteraction;
    std::unique_ptr<oap::app::presentation::OverlayController>
        m_overlayController;
    std::unique_ptr<oap::app::RecognitionWorkspaceCoordinator>
        m_recognitionWorkspace;
    std::unique_ptr<oap::app::StockSimulationWorkspaceCoordinator>
        m_stockSimulationWorkspace;
    std::unique_ptr<oap::app::RecognitionSelectionUiController>
        m_recognitionSelectionUi;
    std::unique_ptr<oap::app::DocumentUiController> m_documentUi;
    std::unique_ptr<oap::app::CamUiController> m_camUi;
    std::unique_ptr<oap::app::SimulationUiController> m_simulationUi;

    // 通用 UI 忙碌态还覆盖同步等待的识别/生成流程；文档加载真值归工作区所有。
    bool            m_uiBusy = false;
    oap::StepLoadPreviewPresenter m_loadPreview;
    QElapsedTimer m_documentLoadTimer;
    std::optional<OcctView::RotationMode> m_loadRotationMode;
    bool            m_closeWhenIdle = false;
    // 公开的 loadFile() 可能在原生 OCCT 查看器创建之前被调用。
    // 保留最近一次请求,在 viewerInitialized() 之后提交。
    QString         m_pendingViewerLoadPath;
    // 批量缓存构建(Open Folder)期间的模态进度对话框;非运行期为空。
    QProgressDialog* m_cacheBuildDialog = nullptr;
    // setValue 在模态下会 processEvents；全命中时进度信号可能重入。
    bool m_inCacheBuildProgress = false;

    const oap::toolpath::PocketRegionWorkflowResult&
        cachedMainOuterInnerWirePocketRegions();

    QAction* m_actHide = nullptr;
    QAction* m_actOnly = nullptr;
    QAction* m_actShowAll = nullptr;
    QAction* m_actShow = nullptr;
    QAction* m_actInvertShow = nullptr;
    QAction* m_actUndo = nullptr;   // Ctrl+Z 撤回可见性操作
    QAction* m_actFitAll = nullptr;
    QAction* m_actNearestStandardView = nullptr;
    QAction* m_actCameraRotation = nullptr;
    QAction* m_actModelRotation = nullptr;
    QAction* m_actFaded = nullptr;
    QAction* m_actShaded = nullptr;
    QAction* m_actEdges = nullptr;
    QAction* m_actConvexityEdges = nullptr;
    QAction* m_actFeatureColors = nullptr; // “识别染色 / 原始颜色”视图切换(识别后启用)
    QAction* m_actShowFeatureLegend = nullptr;
    QLabel* m_featureLegend = nullptr;
    QAction* m_actHolePaths = nullptr;     // “显示孔刀路”高亮/隐藏开关
    QAction* m_actRecognizeCamFeatures = nullptr;
    QAction* m_actOpenMachiningJob = nullptr;
    QAction* m_actSaveMachiningJob = nullptr;
    QAction* m_actToolLibrary = nullptr;
    QAction* m_actToolpathParams = nullptr;
    QAction* m_actPocketClearing = nullptr;
    QAction* m_actProfileContour = nullptr;
    QAction* m_actSurfaceFinishing = nullptr;
    QAction* m_actProjectStock = nullptr;
    QAction* m_actHoleStock = nullptr;
    QAction* m_actPocketStock = nullptr;
    QAction* m_actProfileStock = nullptr;
    QAction* m_actEngravingStock = nullptr;
    QAction* m_actFeatureAllowance = nullptr;
    QAction* m_actExportToolpath = nullptr;
    QAction* m_actPrevFile = nullptr;
    QAction* m_actNextFile = nullptr;
    // 快捷键条目：动作与按键序列的绑定。
    struct ShortcutEntry {
        QString id;
        QString label;
        QAction* action = nullptr;
        QKeySequence defaultSequence;
    };
    std::vector<ShortcutEntry> m_shortcuts;
    // 鼠标绑定条目：动作与鼠标事件的绑定。
    struct MouseBindingEntry {
        QString id;
        QString label;
        OcctView::MouseAction action = OcctView::MouseAction::Select;
        Qt::MouseButton defaultButton = Qt::NoButton;
    };
    std::vector<MouseBindingEntry> m_mouseBindings;

    // 已生成刀路由 OverlayController::Toolpath 唯一持有。

    // 刀路播放器面板 + 其停靠窗口。生成腔清腔/孔刀路时装载播放序列。
    oap::app::CamWorkflowPanel* m_camWorkflowPanel = nullptr;
    QDockWidget*         m_camWorkflowDock = nullptr;
    HoleToolTreePanel*   m_holeToolTreePanel = nullptr;
    HoleCoverageAuditPanel* m_holeCoverageAuditPanel = nullptr;
    FeatureAllowancePanel* m_featureAllowancePanel = nullptr;
    QString              m_currentMachiningJobPath;
    QByteArray           m_defaultViewLayoutState;
    std::unique_ptr<oap::app::PlaybackWorkspaceCoordinator>
        m_playbackWorkspace;

    void rebuildPlaybackMovesFromProgram(
        const oap::toolpath::ToolpathProgram& program,
        bool limitPocketCutMoves);

    using PocketPreviewEntry = oap::app::PocketPreviewEntry;
    using RecognizedHoleSource = oap::app::RecognizedHoleSource;
    using HolePreviewEntry = oap::app::HolePreviewEntry;
    using ActiveToolpathPreview = oap::app::ActiveToolpathPreview;

    void configureToolpathParamsForPage(oap::app::ToolpathParameterEditorPage page);
    bool displayPocketPreviewEntry(const PocketPreviewEntry& entry);
    void rebuildPocketPlaybackMoves();
    oap::app::MachiningProjectDefinitionContext
        machiningProjectDefinitionContext() const;
    bool ensureMachiningJobDefinition(QString& error);
    bool synchronizePocketJobFromResult(QString& error);
    oap::app::MachiningProjectLoadRequest
        currentMachiningRegenerationRequest() const;

    void clearHoleStageHighlightDisplay(bool resetSources);
    oap::app::HoleStageHighlightStats rebuildHoleStageHighlightDisplay();
    void highlightHoleStageToolpathForFace(const std::string& faceId);
    void rebuildHolePreviewDisplay();
    void rebuildHolePlaybackMoves();
    bool ensureHoleProgram();
    void generateOrUpdateHolePaths();
    std::unique_ptr<oap::app::MachiningWorkspaceCoordinator>
        m_machiningWorkspace;
    std::unique_ptr<oap::app::DesktopWorkspaceCoordinator>
        m_workspaceCoordinator;
    std::unique_ptr<oap::app::PlaybackUiController> m_playbackUi;
    qint64 m_lastHolePreviewBuildMs = 0;
    // 播放 AIS 由 OverlayController 的 RemainingPath/Trail/Cursor 分组持有。
    // 模型可见性操作的撤销历史。
    oap::app::CommandHistory m_undoHistory;
};
