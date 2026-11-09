// FeatureAllowancePanel — 展示逐特征加工后余量，并把选择交给主视口呈现。
#pragma once

#include <vector>

#include <QWidget>

#include "app/workflows/simulation/StockSimulationViewModels.h"

class QLabel;
class QTreeWidget;
class QTreeWidgetItem;

// 特征余量面板：展示与编辑特征余量。
class FeatureAllowancePanel final : public QWidget
{
    Q_OBJECT
public:
    explicit FeatureAllowancePanel(QWidget* parent = nullptr);

    void setItems(
        std::vector<oap::app::FeatureAllowanceViewItem> items);
    void clear();
    const oap::app::FeatureAllowanceViewItem* currentItem() const;
    void activateCurrent();

signals:
    void allowanceSelected(int row);

private:
    void updateCurrent(QTreeWidgetItem* current, bool publishSelection);

    std::vector<oap::app::FeatureAllowanceViewItem> m_items;
    QLabel* m_statusLabel = nullptr;
    QTreeWidget* m_tree = nullptr;
    QLabel* m_detailLabel = nullptr;
};
