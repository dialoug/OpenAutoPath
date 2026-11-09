// CAM 工作流面板：把分散的加工入口整理为项目→装夹→刀具→工序→验证→输出。
// 面板只负责 Qt 交互和状态呈现；刀路、Job、IPW 与 OCCT 状态仍由各 Workspace
// 持有。孔工序树、覆盖审计和特征余量复用现有专用面板，避免产生第二套结论。
#pragma once

#include <QString>
#include <QWidget>

class QLabel;
class QListWidget;
class QPushButton;
class QSplitter;
class QStackedWidget;
class HoleToolTreePanel;
class HoleCoverageAuditPanel;
class FeatureAllowancePanel;

namespace oap::app {

enum class CamWorkflowStage : int {
    Project = 0,
    Setup,
    Tooling,
    Operations,
    Verification,
    Output,
};

// CAM 工作流摘要：面板顶部的项目状态投影。
struct CamWorkflowSummary {
    QString modelName;
    QString projectName;
    QString activeProgramName;
    QString activeToolLibraryName;
    QString engravingToolSummary;
    bool engravingToolReady = false;
    QString directToolSummary;
    bool directToolReady = false;
    int assignedToolCount = 0;
    int recognizedFeatureCount = 0;
    int confirmedSetupCount = 0;
    int availableSetupCount = 0;
    int holeOperationCount = 0;
    int pocketOperationCount = 0;
    int profileOperationCount = 0;
    int finishingOperationCount = 0;
    bool hasModel = false;
    bool hasProjectContent = false;
    bool pendingRegeneration = false;
    bool activeProgramAvailable = false;
    bool recognitionReady = false;
    bool setupFramesValid = false;
    bool coverageAudited = false;
    bool coveragePassed = false;
    bool coverageHasWarnings = false;
    bool allowanceAvailable = false;
    bool holeGenerationReady = false;
    bool pocketGenerationReady = false;
    bool profileGenerationReady = false;
    bool finishingGenerationReady = false;
    bool busy = false;
};

// CAM 工作流面板：项目→装夹→刀具→工序→验证→输出的 UI 投影。
class CamWorkflowPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit CamWorkflowPanel(QWidget* parent = nullptr);

    HoleToolTreePanel* operationTree() const noexcept
    {
        return m_operationTree;
    }
    HoleCoverageAuditPanel* coverageAudit() const noexcept
    {
        return m_coverageAudit;
    }
    FeatureAllowancePanel* featureAllowance() const noexcept
    {
        return m_featureAllowance;
    }

    void setSummary(const CamWorkflowSummary& summary);
    void focusStage(CamWorkflowStage stage);
    void resetSplitterLayout();

signals:
    void openPartRequested();
    void openProjectRequested();
    void saveProjectRequested();
    void recognizeFeaturesRequested();
    void configureSetupRequested();
    void selectToolsRequested();
    void manageToolsRequested();
    void generateHoleRequested();
    void generatePocketRequested();
    void generateProfileRequested();
    void generateFinishingRequested();
    void simulateProjectRequested();
    void simulateHoleRequested();
    void simulatePocketRequested();
    void simulateProfileRequested();
    void simulateEngravingRequested();
    void showAllowanceRequested();
    void exportProgramRequested();

private:
    static QString stageTitle(int number, const QString& title,
                              const QString& state);
    void updateStageTitles(const CamWorkflowSummary& summary);

    QListWidget* m_stageList = nullptr;
    QSplitter* m_mainSplitter = nullptr;
    QSplitter* m_operationsSplitter = nullptr;
    QSplitter* m_verificationSplitter = nullptr;
    QStackedWidget* m_pages = nullptr;
    QLabel* m_jobLabel = nullptr;
    QLabel* m_modelStatus = nullptr;
    QLabel* m_setupStatus = nullptr;
    QLabel* m_toolStatus = nullptr;
    QLabel* m_operationStatus = nullptr;
    QLabel* m_verificationStatus = nullptr;
    QLabel* m_outputStatus = nullptr;
    QPushButton* m_openPart = nullptr;
    QPushButton* m_openProject = nullptr;
    QPushButton* m_saveProject = nullptr;
    QPushButton* m_recognizeFeatures = nullptr;
    QPushButton* m_configureSetup = nullptr;
    QPushButton* m_selectTools = nullptr;
    QPushButton* m_manageTools = nullptr;
    QPushButton* m_generateHole = nullptr;
    QPushButton* m_generatePocket = nullptr;
    QPushButton* m_generateProfile = nullptr;
    QPushButton* m_generateFinishing = nullptr;
    QPushButton* m_simulateProject = nullptr;
    QPushButton* m_simulateHole = nullptr;
    QPushButton* m_simulatePocket = nullptr;
    QPushButton* m_simulateProfile = nullptr;
    QPushButton* m_simulateEngraving = nullptr;
    QPushButton* m_showAllowance = nullptr;
    QPushButton* m_exportProgram = nullptr;
    HoleToolTreePanel* m_operationTree = nullptr;
    HoleCoverageAuditPanel* m_coverageAudit = nullptr;
    FeatureAllowancePanel* m_featureAllowance = nullptr;
};

} // namespace oap::app
