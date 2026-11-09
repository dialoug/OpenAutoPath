// HoleToolTreePanel — 按装夹面展示孔、槽腔刀路实际使用的刀具批次，并与播放位置同步。
#pragma once

#include <QString>
#include <QHash>
#include <QVector>
#include <QWidget>

#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"

class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QLabel;

// 孔刀具树面板：展示孔工序与刀具树。
class HoleToolTreePanel : public QWidget
{
    Q_OBJECT
public:
    explicit HoleToolTreePanel(QWidget* parent = nullptr);
    ~HoleToolTreePanel() override;

    void setProgram(
        const QVector<oap::app::HoleToolBatchView>& batches);
    void setPlaybackPosition(int completedMoveCount);
    void selectOperation(const QString& operationId);
    void selectFormalOperation(const QString& formalOperationId);
    QString selectedFormalOperationId() const noexcept;
    void setSoloOperation(const QString& operationId);
    void resetOperationVisibility();
    void setGenerationAvailability(bool hole, bool pocket, bool profile, bool engraving);
    void clear();

signals:
    void toolBatchActivated(int batchIndex);
    void operationActivated(const QString& operationId, int startMoveIndex);
    void formalOperationSelected(const QString& formalOperationId);
    void previewOperationRequested(const QString& formalOperationId);
    void operationVisibilityChanged(
        const QString& operationId, bool visible);
    void operationSoloRequested(const QString& operationId);
    void operationVisibilitySummaryChanged(
        int visibleCount, int totalCount, bool soloMode);
    void editHoleParametersRequested();
    void editPocketOperationRequested();
    void regeneratePocketRequested();
    void editProfileOperationRequested();
    void regenerateProfileRequested();
    void editFinishingOperationRequested(const QString& formalOperationId);
    void regenerateFinishingRequested(const QString& formalOperationId);
    void regenerateHoleRequested();

private slots:
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onCurrentItemChanged(
        QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void editSelectedOperation();
    void regenerateSelectedOperation();
    void previewSelectedOperation();

private:
    void clearTree();
    void showAllOperations();
    void setOperationDescendantsVisible(
        QTreeWidgetItem* item, bool visible);
    void updateOperationHierarchy();
    void updateOperationVisibilitySummary();
    void updateOperationPresentation();
    void updateOperationInspector(QTreeWidgetItem* item);

    QTreeWidget* m_tree = nullptr;
    QPushButton* m_showAllOperations = nullptr;
    QLabel* m_operationVisibilitySummary = nullptr;
    QLabel* m_operationInspectorTitle = nullptr;
    QLabel* m_operationInspectorDetails = nullptr;
    QLabel* m_operationInspectorHint = nullptr;
    QPushButton* m_editOperation = nullptr;
    QPushButton* m_regenerateOperation = nullptr;
    QPushButton* m_previewOperation = nullptr;
    QVector<oap::app::HoleToolBatchView> m_batches;
    QVector<QTreeWidgetItem*> m_batchItems;
    QHash<QString, QTreeWidgetItem*> m_operationItems;
    QHash<QString, QTreeWidgetItem*> m_formalOperationItems;
    QHash<QString, oap::app::ToolpathOperationView> m_operationViews;
    QHash<QString, bool> m_operationVisibility;
    QString m_soloOperationId;
    QString m_selectedOperationId;
    QString m_selectedFormalOperationId;
    bool m_holeGenerationAvailable = false;
    bool m_pocketGenerationAvailable = false;
    bool m_profileGenerationAvailable = false;
    bool m_engravingGenerationAvailable = false;
};
