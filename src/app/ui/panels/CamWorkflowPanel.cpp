// CamWorkflowPanel 实现（oap_app_cam_ui）。
// CAM 工作流面板：把分散的加工入口整理为项目→装夹→刀具→工序→验证→输出
// 的 UI 投影，见 CamWorkflowPanel.h。
#include "app/ui/panels/CamWorkflowPanel.h"

#include "app/ui/panels/FeatureAllowancePanel.h"
#include "app/ui/panels/HoleCoverageAuditPanel.h"
#include "app/ui/panels/HoleToolTreePanel.h"

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPalette>
#include <QPushButton>
#include <QSettings>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSplitter>
#include <QStackedWidget>
#include <QStyle>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

namespace oap::app {
namespace {

QLabel* makeStatusLabel(QWidget* parent)
{
    auto* label = new QLabel(parent);
    label->setWordWrap(true);
    label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setObjectName(QStringLiteral("camWorkflowStatus"));
    return label;
}

QLabel* makeHint(const QString& text, QWidget* parent)
{
    auto* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setObjectName(QStringLiteral("camWorkflowHint"));
    QPalette hintPalette = label->palette();
    hintPalette.setColor(
        QPalette::WindowText,
        parent->palette().color(QPalette::Disabled, QPalette::Text));
    label->setPalette(hintPalette);
    return label;
}

QPushButton* makeAction(const QString& text, QWidget* parent,
                        bool primary = false)
{
    auto* button = new QPushButton(text, parent);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    if (primary)
        button->setProperty("camPrimary", true);
    return button;
}

QWidget* makeStepPage(QStackedWidget* parent, QVBoxLayout*& layout)
{
    auto* page = new QWidget(parent);
    page->setMinimumHeight(0);
    layout = new QVBoxLayout(page);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(9);
    return page;
}

void addStepPage(QStackedWidget* stack, QWidget* page)
{
    // On short windows the current step scrolls instead of clipping buttons.
    auto* scroll = new QScrollArea(stack);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setWidget(page);
    stack->addWidget(scroll);
}

QString completionMark(bool ready)
{
    return ready ? QStringLiteral("✓") : QStringLiteral("○");
}

constexpr auto kMainSplitterKey = "cam/workflow/mainSplitter";
constexpr auto kOperationsSplitterKey = "cam/workflow/operationsSplitter";
constexpr auto kVerificationSplitterKey = "cam/workflow/verificationSplitter";

void configureVerticalSplitter(QSplitter* splitter)
{
    splitter->setOrientation(Qt::Vertical);
    splitter->setChildrenCollapsible(false);
    splitter->setHandleWidth(7);
    splitter->setOpaqueResize(true);
}

} // namespace

CamWorkflowPanel::CamWorkflowPanel(QWidget* parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("camWorkflowPanel"));
    setMinimumWidth(300);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    auto* header = new QFrame(this);
    header->setObjectName(QStringLiteral("camWorkflowHeader"));
    auto* headerLayout = new QVBoxLayout(header);
    headerLayout->setContentsMargins(12, 7, 12, 7);
    headerLayout->setSpacing(0);
    auto* title = new QLabel(QStringLiteral("加工项目"), header);
    title->setObjectName(QStringLiteral("camWorkflowTitle"));
    headerLayout->addWidget(title);
    m_jobLabel = new QLabel(QStringLiteral("尚未加载零件"), header);
    m_jobLabel->setObjectName(QStringLiteral("camWorkflowJob"));
    m_jobLabel->setWordWrap(true);
    m_jobLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_jobLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    headerLayout->addWidget(m_jobLabel);
    root->addWidget(header);

    m_mainSplitter = new QSplitter(this);
    m_mainSplitter->setObjectName(QStringLiteral("camWorkflowMainSplitter"));
    configureVerticalSplitter(m_mainSplitter);
    root->addWidget(m_mainSplitter, 1);

    m_stageList = new QListWidget(m_mainSplitter);
    m_stageList->setObjectName(QStringLiteral("camWorkflowStages"));
    m_stageList->setSelectionMode(QAbstractItemView::SingleSelection);
    m_stageList->setUniformItemSizes(true);
    m_stageList->setViewMode(QListView::IconMode);
    m_stageList->setMovement(QListView::Static);
    m_stageList->setResizeMode(QListView::Adjust);
    m_stageList->setWrapping(true);
    m_stageList->setWordWrap(false);
    m_stageList->setGridSize(QSize(145, 32));
    m_stageList->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_stageList->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_stageList->setFocusPolicy(Qt::NoFocus);
    for (int index = 0; index < 6; ++index) {
        auto* item = new QListWidgetItem(m_stageList);
        item->setSizeHint(QSize(145, 32));
    }
    m_stageList->setMinimumHeight(46);
    m_stageList->setMaximumHeight(102);
    m_mainSplitter->addWidget(m_stageList);

    m_pages = new QStackedWidget(m_mainSplitter);
    m_pages->setObjectName(QStringLiteral("camWorkflowPages"));
    m_pages->setMinimumHeight(0);
    m_mainSplitter->addWidget(m_pages);
    m_mainSplitter->setStretchFactor(0, 0);
    m_mainSplitter->setStretchFactor(1, 1);
    m_mainSplitter->setCollapsible(0, true);
    m_mainSplitter->setCollapsible(1, false);
    connect(m_stageList, &QListWidget::currentRowChanged,
            m_pages, &QStackedWidget::setCurrentIndex);

    QVBoxLayout* pageLayout = nullptr;
    QWidget* projectPage = makeStepPage(m_pages, pageLayout);
    m_modelStatus = makeStatusLabel(projectPage);
    pageLayout->addWidget(m_modelStatus);
    m_openPart = makeAction(
        QStringLiteral("打开 STEP 零件…"), projectPage, true);
    m_openPart->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
    pageLayout->addWidget(m_openPart);
    auto* projectActions = new QHBoxLayout;
    m_openProject = makeAction(QStringLiteral("打开 Job…"), projectPage);
    m_saveProject = makeAction(QStringLiteral("保存 Job…"), projectPage);
    m_saveProject->setObjectName(QStringLiteral("saveMachiningProjectAction"));
    m_openProject->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
    m_saveProject->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
    projectActions->addWidget(m_openProject);
    projectActions->addWidget(m_saveProject);
    pageLayout->addLayout(projectActions);
    m_recognizeFeatures = makeAction(
        QStringLiteral("识别加工特征"), projectPage, true);
    m_recognizeFeatures->setToolTip(
        QStringLiteral("识别孔、通结构及主外表面内环槽腔来源"));
    pageLayout->addWidget(m_recognizeFeatures);
    pageLayout->addStretch(1);
    addStepPage(m_pages, projectPage);

    QWidget* setupPage = makeStepPage(m_pages, pageLayout);
    m_setupStatus = makeStatusLabel(setupPage);
    pageLayout->addWidget(m_setupStatus);
    m_configureSetup = makeAction(
        QStringLiteral("加工参数与装夹确认…"), setupPage, true);
    pageLayout->addWidget(m_configureSetup);
    pageLayout->addWidget(makeHint(
        QStringLiteral("集中设置孔/腔参数、G54–G59、六面工作坐标确认和参考 NC 输出选项。"),
        setupPage));
    pageLayout->addStretch(1);
    addStepPage(m_pages, setupPage);

    QWidget* toolingPage = makeStepPage(m_pages, pageLayout);
    m_toolStatus = makeStatusLabel(toolingPage);
    pageLayout->addWidget(m_toolStatus);
    m_selectTools = makeAction(
        QStringLiteral("选择活动刀库…"), toolingPage, true);
    pageLayout->addWidget(m_selectTools);
    m_manageTools = makeAction(
        QStringLiteral("管理刀具资产…"), toolingPage);
    pageLayout->addWidget(m_manageTools);
    pageLayout->addWidget(makeHint(
        QStringLiteral("管理器维护刀具资产；只有显式选择的活动刀库参与自动选刀。工序创建后保存不可变刀具快照。"),
        toolingPage));
    pageLayout->addStretch(1);
    addStepPage(m_pages, toolingPage);

    QWidget* operationPage = makeStepPage(m_pages, pageLayout);
    m_operationsSplitter = new QSplitter(operationPage);
    m_operationsSplitter->setObjectName(
        QStringLiteral("camWorkflowOperationsSplitter"));
    configureVerticalSplitter(m_operationsSplitter);
    pageLayout->addWidget(m_operationsSplitter, 1);
    auto* operationControls = new QWidget(m_operationsSplitter);
    operationControls->setMinimumHeight(0);
    auto* operationControlsLayout = new QVBoxLayout(operationControls);
    operationControlsLayout->setSizeConstraint(QLayout::SetMinimumSize);
    operationControlsLayout->setContentsMargins(0, 0, 0, 0);
    operationControlsLayout->setSpacing(7);
    m_operationStatus = makeStatusLabel(operationControls);
    operationControlsLayout->addWidget(m_operationStatus);
    auto* generationActions = new QGridLayout;
    generationActions->setContentsMargins(0, 0, 0, 0);
    generationActions->setHorizontalSpacing(7);
    generationActions->setVerticalSpacing(7);
    m_generateHole = makeAction(
        QStringLiteral("生成孔刀路"), operationControls, true);
    m_generateHole->setObjectName(
        QStringLiteral("generateHoleToolpathAction"));
    m_generateHole->setToolTip(
        QStringLiteral("从当前识别结果生成孔刀路；已有孔程序时重新规划并替换"));
    m_generatePocket = makeAction(
        QStringLiteral("生成槽腔刀路"), operationControls, true);
    m_generatePocket->setObjectName(
        QStringLiteral("generatePocketToolpathAction"));
    m_generatePocket->setToolTip(
        QStringLiteral("从已识别的主外表面内环生成槽腔清腔工序"));
    m_generateProfile = makeAction(
        QStringLiteral("生成外轮廓刀路"), operationControls, true);
    m_generateProfile->setObjectName(
        QStringLiteral("generateProfileToolpathAction"));
    m_generateProfile->setToolTip(
        QStringLiteral("从已识别的主外表面外环生成分层外轮廓工序"));
    generationActions->addWidget(m_generateHole, 0, 0);
    generationActions->addWidget(m_generatePocket, 0, 1);
    generationActions->addWidget(m_generateProfile, 1, 0);
    m_generateFinishing = makeAction(QStringLiteral("生成雕刻刀路"), operationControls, true);
    m_generateFinishing->setObjectName(QStringLiteral("generateEngravingAction"));
    m_generateFinishing->setToolTip(QStringLiteral("按雕刻参数直接生成完整模型刀路；默认六面，也可选择任一单面"));
    generationActions->addWidget(m_generateFinishing, 1, 1);
    operationControlsLayout->addLayout(generationActions);
    operationControlsLayout->addStretch(1);
    m_operationsSplitter->addWidget(operationControls);
    m_operationTree = new HoleToolTreePanel(m_operationsSplitter);
    m_operationTree->setObjectName(QStringLiteral("camOperationTree"));
    m_operationTree->setMinimumHeight(0);
    m_operationTree->setSizePolicy(
        QSizePolicy::Preferred, QSizePolicy::Ignored);
    m_operationsSplitter->addWidget(m_operationTree);
    m_operationsSplitter->setStretchFactor(0, 0);
    m_operationsSplitter->setStretchFactor(1, 1);
    m_operationsSplitter->setCollapsible(0, true);
    m_operationsSplitter->setCollapsible(1, false);
    addStepPage(m_pages, operationPage);

    QWidget* verificationPage = makeStepPage(m_pages, pageLayout);
    m_verificationSplitter = new QSplitter(verificationPage);
    m_verificationSplitter->setObjectName(
        QStringLiteral("camWorkflowVerificationSplitter"));
    configureVerticalSplitter(m_verificationSplitter);
    pageLayout->addWidget(m_verificationSplitter, 1);
    auto* verificationControls = new QWidget(m_verificationSplitter);
    verificationControls->setMinimumHeight(0);
    auto* verificationControlsLayout = new QVBoxLayout(verificationControls);
    verificationControlsLayout->setSizeConstraint(QLayout::SetMinimumSize);
    verificationControlsLayout->setContentsMargins(0, 0, 0, 0);
    verificationControlsLayout->setSpacing(7);
    m_verificationStatus = makeStatusLabel(verificationControls);
    verificationControlsLayout->addWidget(m_verificationStatus);
    auto* simulationActions = new QGridLayout;
    m_simulateHole = makeAction(QStringLiteral("孔 IPW"), verificationControls);
    m_simulatePocket = makeAction(QStringLiteral("槽腔 IPW"), verificationControls);
    m_simulateProfile = makeAction(
        QStringLiteral("外轮廓 IPW"), verificationControls);
    simulationActions->addWidget(m_simulateHole, 0, 0);
    simulationActions->addWidget(m_simulatePocket, 0, 1);
    simulationActions->addWidget(m_simulateProfile, 1, 0);
    m_simulateEngraving = makeAction(QStringLiteral("雕刻 IPW"), verificationControls);
    m_simulateEngraving->setObjectName(QStringLiteral("simulateEngravingIpwAction"));
    m_simulateEngraving->setToolTip(QStringLiteral("选择雕刻装夹方向，沿用前序工序剩余毛坯进行材料仿真和刀具动画"));
    simulationActions->addWidget(m_simulateEngraving, 1, 1);
    verificationControlsLayout->addLayout(simulationActions);
    m_simulateProject = makeAction(QStringLiteral("加工项目 IPW（按全局工序顺序）"), verificationControls);
    m_simulateProject->setToolTip(QStringLiteral(
        "从所选毛坯开始，按项目全局顺序连续模拟全部启用工序的材料去除。请先生成有效刀路。"));
    m_simulateProject->setObjectName(QStringLiteral("simulateProjectIpwAction"));
    verificationControlsLayout->addWidget(m_simulateProject);
    m_showAllowance = makeAction(
        QStringLiteral("查看逐特征余量"), verificationControls);
    verificationControlsLayout->addWidget(m_showAllowance);
    verificationControlsLayout->addStretch(1);
    m_verificationSplitter->addWidget(verificationControls);
    auto* validationTabs = new QTabWidget(m_verificationSplitter);
    validationTabs->setDocumentMode(true);
    validationTabs->setMinimumHeight(0);
    validationTabs->setSizePolicy(
        QSizePolicy::Preferred, QSizePolicy::Ignored);
    m_coverageAudit = new HoleCoverageAuditPanel(validationTabs);
    m_featureAllowance = new FeatureAllowancePanel(validationTabs);
    validationTabs->addTab(m_coverageAudit, QStringLiteral("覆盖审计"));
    validationTabs->addTab(m_featureAllowance, QStringLiteral("余量"));
    m_verificationSplitter->addWidget(validationTabs);
    m_verificationSplitter->setStretchFactor(0, 0);
    m_verificationSplitter->setStretchFactor(1, 1);
    m_verificationSplitter->setCollapsible(0, true);
    m_verificationSplitter->setCollapsible(1, false);
    addStepPage(m_pages, verificationPage);

    QWidget* outputPage = makeStepPage(m_pages, pageLayout);
    m_outputStatus = makeStatusLabel(outputPage);
    m_outputStatus->setObjectName(QStringLiteral("camWorkflowOutputStatus"));
    pageLayout->addWidget(m_outputStatus);
    m_exportProgram = makeAction(
        QStringLiteral("审查并导出当前程序…"), outputPage, true);
    m_exportProgram->setObjectName(QStringLiteral("exportCurrentProgramAction"));
    pageLayout->addWidget(m_exportProgram);
    auto* warning = makeHint(
        QStringLiteral("当前后处理输出为 Reference NC。写盘前必须通过结构审计并由操作者逐行确认；它不是可直接上机的控制器程序。"),
        outputPage);
    warning->setObjectName(QStringLiteral("camWorkflowWarning"));
    pageLayout->addWidget(warning);
    pageLayout->addStretch(1);
    addStepPage(m_pages, outputPage);

    setStyleSheet(QStringLiteral(
        "#camWorkflowHeader { background: palette(alternate-base); border-bottom: 1px solid palette(mid); }"
        "#camWorkflowTitle { font-size: 16px; font-weight: 600; }"
        "#camWorkflowJob { margin-top: 4px; font-weight: 600; }"
        "#camWorkflowStatus, #camWorkflowOutputStatus { padding: 6px; background: palette(base); border: 1px solid palette(midlight); border-radius: 3px; }"
        "#camWorkflowWarning { color: #a15c00; padding: 6px; border: 1px solid #c68a28; border-radius: 3px; }"
        "#camWorkflowStages { border: 0; border-bottom: 1px solid palette(mid); }"
        "#camWorkflowStages::item { padding-left: 8px; border-bottom: 1px solid palette(midlight); }"
        "#camWorkflowStages::item:selected { font-weight: 600; background: palette(highlight); color: palette(highlighted-text); }"
        "QSplitter#camWorkflowMainSplitter::handle:vertical,"
        "QSplitter#camWorkflowOperationsSplitter::handle:vertical,"
        "QSplitter#camWorkflowVerificationSplitter::handle:vertical {"
        "  background: palette(midlight); margin: 2px 0; }"
        "QSplitter#camWorkflowMainSplitter::handle:vertical:hover,"
        "QSplitter#camWorkflowOperationsSplitter::handle:vertical:hover,"
        "QSplitter#camWorkflowVerificationSplitter::handle:vertical:hover {"
        "  background: palette(highlight); }"
        "QPushButton[camPrimary=\"true\"] { font-weight: 600; min-height: 26px; }"));

    connect(m_openPart, &QPushButton::clicked,
            this, &CamWorkflowPanel::openPartRequested);
    connect(m_openProject, &QPushButton::clicked,
            this, &CamWorkflowPanel::openProjectRequested);
    connect(m_saveProject, &QPushButton::clicked,
            this, &CamWorkflowPanel::saveProjectRequested);
    connect(m_recognizeFeatures, &QPushButton::clicked,
            this, &CamWorkflowPanel::recognizeFeaturesRequested);
    connect(m_configureSetup, &QPushButton::clicked,
            this, &CamWorkflowPanel::configureSetupRequested);
    connect(m_selectTools, &QPushButton::clicked,
            this, &CamWorkflowPanel::selectToolsRequested);
    connect(m_manageTools, &QPushButton::clicked,
            this, &CamWorkflowPanel::manageToolsRequested);
    connect(m_generateHole, &QPushButton::clicked,
            this, &CamWorkflowPanel::generateHoleRequested);
    connect(m_generatePocket, &QPushButton::clicked,
            this, &CamWorkflowPanel::generatePocketRequested);
    connect(m_generateProfile, &QPushButton::clicked,
            this, &CamWorkflowPanel::generateProfileRequested);
    connect(m_generateFinishing, &QPushButton::clicked,
            this, &CamWorkflowPanel::generateFinishingRequested);
    connect(m_simulateProject, &QPushButton::clicked,
            this, &CamWorkflowPanel::simulateProjectRequested);
    connect(m_simulateHole, &QPushButton::clicked,
            this, &CamWorkflowPanel::simulateHoleRequested);
    connect(m_simulatePocket, &QPushButton::clicked,
            this, &CamWorkflowPanel::simulatePocketRequested);
    connect(m_simulateProfile, &QPushButton::clicked,
            this, &CamWorkflowPanel::simulateProfileRequested);
    connect(m_simulateEngraving, &QPushButton::clicked,
            this, &CamWorkflowPanel::simulateEngravingRequested);
    connect(m_showAllowance, &QPushButton::clicked,
            this, &CamWorkflowPanel::showAllowanceRequested);
    connect(m_exportProgram, &QPushButton::clicked,
            this, &CamWorkflowPanel::exportProgramRequested);

    const auto persistSplitter = [this](QSplitter* splitter,
                                         const char* settingsKey) {
        connect(splitter, &QSplitter::splitterMoved, this,
                [splitter, settingsKey] {
            QSettings settings(QStringLiteral("OAP"),
                               QStringLiteral("OpenAutoPath"));
            settings.setValue(QString::fromLatin1(settingsKey),
                              splitter->saveState());
        });
    };
    persistSplitter(m_mainSplitter, kMainSplitterKey);
    persistSplitter(m_operationsSplitter, kOperationsSplitterKey);
    persistSplitter(m_verificationSplitter, kVerificationSplitterKey);

    QTimer::singleShot(0, this, [this] {
        QSettings settings(QStringLiteral("OAP"),
                           QStringLiteral("OpenAutoPath"));
        const auto restoreOrDefault = [&settings](
                QSplitter* splitter, const char* settingsKey,
                const QList<int>& defaultSizes) {
            const QByteArray state = settings.value(
                QString::fromLatin1(settingsKey)).toByteArray();
            if (state.isEmpty() || !splitter->restoreState(state))
                splitter->setSizes(defaultSizes);
        };
        restoreOrDefault(m_mainSplitter, kMainSplitterKey, {102, 498});
        restoreOrDefault(
            m_operationsSplitter, kOperationsSplitterKey, {90, 360});
        restoreOrDefault(
            m_verificationSplitter, kVerificationSplitterKey, {120, 420});
    });

    setSummary({});
    m_stageList->setCurrentRow(0);
}

void CamWorkflowPanel::resetSplitterLayout()
{
    QSettings settings(QStringLiteral("OAP"),
                       QStringLiteral("OpenAutoPath"));
    settings.remove(QString::fromLatin1(kMainSplitterKey));
    settings.remove(QString::fromLatin1(kOperationsSplitterKey));
    settings.remove(QString::fromLatin1(kVerificationSplitterKey));
    m_mainSplitter->setSizes({102, 498});
    m_operationsSplitter->setSizes({90, 360});
    m_verificationSplitter->setSizes({120, 420});
}

QString CamWorkflowPanel::stageTitle(int number, const QString& title,
                                     const QString& state)
{
    return QStringLiteral("%1  %2  %3").arg(number).arg(title, state);
}

void CamWorkflowPanel::setSummary(const CamWorkflowSummary& summary)
{
    const QString model = summary.hasModel
        ? (summary.modelName.isEmpty() ? QStringLiteral("已加载零件")
                                       : summary.modelName)
        : QStringLiteral("尚未加载零件");
    const QString project = summary.projectName.isEmpty()
        ? QStringLiteral("内存 Job") : summary.projectName;
    m_jobLabel->setText(summary.hasModel
        ? QStringLiteral("%1  ·  %2").arg(project, model)
        : model);

    m_modelStatus->setText(summary.hasModel
        ? QStringLiteral("模型已就绪｜加工特征 %1 个%2")
              .arg(summary.recognizedFeatureCount)
              .arg(summary.recognitionReady
                       ? QStringLiteral("，识别完成")
                       : QStringLiteral("，雕刻可直接生成，其他工序待识别"))
        : QStringLiteral("先打开 STEP 零件；.oapjob 不嵌入模型，需在零件加载后恢复。"));
    m_setupStatus->setText(summary.setupFramesValid
        ? QStringLiteral("六面工作坐标已建立｜人工确认 %1/%2")
              .arg(summary.confirmedSetupCount)
              .arg(summary.availableSetupCount)
        : QStringLiteral("六面工作坐标尚未建立；加载零件后检查 WCS 与装夹方向。"));
    m_toolStatus->setText(summary.activeToolLibraryName.isEmpty()
        ? QStringLiteral("未启用刀库｜自动选刀不可用")
        : QStringLiteral("活动刀库：%1｜已分配 T 刀具 %2 把")
              .arg(summary.activeToolLibraryName)
              .arg(summary.assignedToolCount));
    if (!summary.engravingToolSummary.isEmpty())
        m_toolStatus->setText(m_toolStatus->text() + QStringLiteral("\n") + summary.engravingToolSummary);
    if (!summary.directToolSummary.isEmpty())
        m_toolStatus->setText(m_toolStatus->text() + QStringLiteral("\n") + summary.directToolSummary);
    m_operationStatus->setText(QStringLiteral(
        "已生成程序｜孔 %1 道 · 槽腔 %2 道 · 外轮廓 %3 道 · 雕刻 %4 道")
        .arg(summary.holeOperationCount)
        .arg(summary.pocketOperationCount)
        .arg(summary.profileOperationCount).arg(summary.finishingOperationCount));

    if (summary.pendingRegeneration) {
        m_operationStatus->setText(m_operationStatus->text() + QStringLiteral("\n有工序待重新生成"));
        m_verificationStatus->setText(QStringLiteral("项目含待重新生成工序｜重新生成后再运行项目 IPW"));
    } else if (!summary.coverageAudited) {
        m_verificationStatus->setText(
            summary.allowanceAvailable
                ? QStringLiteral("覆盖审计未运行｜已有 IPW 余量结果")
                : QStringLiteral("等待验证｜生成工序后执行覆盖审计与 IPW"));
    } else if (!summary.coveragePassed) {
        m_verificationStatus->setText(
            QStringLiteral("覆盖审计未通过｜请处理阻断项后再导出"));
    } else if (summary.coverageHasWarnings) {
        m_verificationStatus->setText(
            QStringLiteral("覆盖审计通过（有警告）｜导出前必须复核"));
    } else {
        m_verificationStatus->setText(
            QStringLiteral("覆盖审计通过%1")
                .arg(summary.allowanceAvailable
                    ? QStringLiteral("｜IPW 余量结果可用") : QString()));
    }

    const bool hasProgram = summary.holeOperationCount > 0 ||
                            summary.pocketOperationCount > 0 ||
                            summary.profileOperationCount > 0 || summary.finishingOperationCount > 0;
    const bool hasHoleProgram = summary.holeOperationCount > 0;
    const bool hasPocketProgram = summary.pocketOperationCount > 0;
    const bool hasProfileProgram = summary.profileOperationCount > 0;
    m_generateHole->setText(
        hasHoleProgram ? QStringLiteral("重新生成孔刀路")
                       : QStringLiteral("生成孔刀路"));
    m_generateHole->setToolTip(
        hasHoleProgram
            ? QStringLiteral("按孔参数页配置的刀具来源重新生成并替换孔程序")
            : QStringLiteral("从当前识别结果按配置的刀具来源生成孔程序"));
    m_generatePocket->setText(
        hasPocketProgram ? QStringLiteral("重新生成槽腔刀路")
                         : QStringLiteral("生成槽腔刀路"));
    m_generatePocket->setToolTip(
        hasPocketProgram
            ? QStringLiteral("按当前参数重新生成并替换槽腔程序")
            : QStringLiteral("从已识别的主外表面内环生成槽腔清腔工序"));
    m_generateProfile->setText(
        hasProfileProgram ? QStringLiteral("重新生成外轮廓刀路")
                          : QStringLiteral("生成外轮廓刀路"));
    m_generateProfile->setToolTip(
        hasProfileProgram
            ? QStringLiteral("按当前参数重新生成并替换外轮廓程序")
            : QStringLiteral("从已识别的主外表面外环生成分层外轮廓工序"));
    m_generateFinishing->setText(summary.finishingOperationCount > 0
        ? QStringLiteral("重新生成雕刻刀路") : QStringLiteral("生成雕刻刀路"));
    const QString activeProgramName = summary.activeProgramName.isEmpty()
        ? QStringLiteral("未选择") : summary.activeProgramName;
    QString outputState = summary.activeProgramAvailable
        ? QStringLiteral("可进入该程序的导出审查")
        : summary.pendingRegeneration ? QStringLiteral("当前程序不可导出，请重新生成受影响工序")
                                      : QStringLiteral("尚无可导出的活动刀路");
    if (summary.pendingRegeneration && summary.activeProgramAvailable)
        outputState += QStringLiteral("；项目中其他待重新生成工序须先更新，才能保存或运行项目 IPW");
    m_outputStatus->setText(QStringLiteral("当前预览：%1｜%2").arg(activeProgramName, outputState));

    m_openPart->setEnabled(!summary.busy);
    m_openProject->setEnabled(summary.hasModel && !summary.busy);
    m_saveProject->setEnabled(summary.hasProjectContent && !summary.pendingRegeneration && !summary.busy);
    m_recognizeFeatures->setEnabled(summary.hasModel && !summary.busy);
    m_configureSetup->setEnabled(summary.hasModel && !summary.busy);
    m_selectTools->setEnabled(!summary.busy);
    m_manageTools->setEnabled(!summary.busy);
    const bool generationEnabled = summary.hasModel && !summary.busy;
    m_generateHole->setEnabled(generationEnabled && summary.holeGenerationReady);
    m_generatePocket->setEnabled(generationEnabled && summary.pocketGenerationReady);
    m_generateProfile->setEnabled(generationEnabled && summary.profileGenerationReady);
    m_generateFinishing->setEnabled(generationEnabled && summary.finishingGenerationReady);
    m_simulateProject->setEnabled(hasProgram && !summary.pendingRegeneration && !summary.busy);
    m_operationTree->setEnabled(summary.hasModel && !summary.busy);
    m_operationTree->setGenerationAvailability(
        generationEnabled && summary.holeGenerationReady,
        generationEnabled && summary.pocketGenerationReady,
        generationEnabled && summary.profileGenerationReady,
        generationEnabled && summary.finishingGenerationReady);
    m_simulateHole->setEnabled(
        summary.holeOperationCount > 0 && !summary.busy);
    m_simulatePocket->setEnabled(
        summary.pocketOperationCount > 0 && !summary.busy);
    m_simulateProfile->setEnabled(
        summary.profileOperationCount > 0 && !summary.busy);
    m_simulateEngraving->setEnabled(summary.finishingOperationCount > 0 && !summary.busy);
    m_showAllowance->setEnabled(summary.allowanceAvailable && !summary.busy);
    m_exportProgram->setEnabled(summary.activeProgramAvailable && !summary.busy);
    updateStageTitles(summary);
}

void CamWorkflowPanel::focusStage(CamWorkflowStage stage)
{
    if (!m_stageList || !m_pages)
        return;
    const int index = static_cast<int>(stage);
    if (index >= 0 && index < m_pages->count())
        m_stageList->setCurrentRow(index);
}

void CamWorkflowPanel::updateStageTitles(
    const CamWorkflowSummary& summary)
{
    QString verificationMark = QStringLiteral("○");
    if (summary.pendingRegeneration) {
        verificationMark = QStringLiteral("!");
    } else if (summary.coverageAudited) {
        verificationMark = summary.coveragePassed
            ? (summary.coverageHasWarnings ? QStringLiteral("!")
                                           : QStringLiteral("✓"))
            : QStringLiteral("×");
    }
    m_stageList->item(0)->setText(stageTitle(
        1, QStringLiteral("项目与特征"),
        completionMark(summary.hasModel && (summary.recognitionReady || summary.finishingGenerationReady))));
    m_stageList->item(1)->setText(stageTitle(
        2, QStringLiteral("装夹与参数"),
        completionMark(summary.setupFramesValid)));
    m_stageList->item(2)->setText(stageTitle(
        3, QStringLiteral("刀具"),
        completionMark(summary.assignedToolCount > 0 || summary.engravingToolReady || summary.directToolReady)));
    m_stageList->item(3)->setText(stageTitle(
        4, QStringLiteral("加工工序"), summary.pendingRegeneration ? QStringLiteral("!")
            : completionMark(summary.hasProjectContent)));
    m_stageList->item(4)->setText(stageTitle(
        5, QStringLiteral("验证与仿真"), verificationMark));
    m_stageList->item(5)->setText(stageTitle(
        6, QStringLiteral("输出"), completionMark(summary.activeProgramAvailable)));
}

} // namespace oap::app
