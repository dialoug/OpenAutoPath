// FeatureAllowancePanel 实现 — 只显示工作流已计算的局部余量，不重复执行几何计算。
#include "app/ui/panels/FeatureAllowancePanel.h"

#include <algorithm>

#include <QAbstractItemView>
#include <QHeaderView>
#include <QLabel>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

namespace {
constexpr int kRoleResultRow = Qt::UserRole + 1;
}

FeatureAllowancePanel::FeatureAllowancePanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);

    m_statusLabel = new QLabel(this);
    m_statusLabel->setObjectName(QStringLiteral("featureAllowanceStatus"));
    m_statusLabel->setWordWrap(true);
    layout->addWidget(m_statusLabel);

    m_tree = new QTreeWidget(this);
    m_tree->setObjectName(QStringLiteral("featureAllowanceTree"));
    m_tree->setHeaderLabels({
        QStringLiteral("特征"), QStringLiteral("加工后余量"),
        QStringLiteral("完成率")});
    m_tree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tree->setRootIsDecorated(false);
    m_tree->setUniformRowHeights(true);
    m_tree->setAlternatingRowColors(true);
    m_tree->setTextElideMode(Qt::ElideRight);
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_tree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    connect(
        m_tree, &QTreeWidget::currentItemChanged,
        this, [this](QTreeWidgetItem* current, QTreeWidgetItem*) {
            updateCurrent(current, true);
        });
    layout->addWidget(m_tree, 1);

    m_detailLabel = new QLabel(this);
    m_detailLabel->setObjectName(QStringLiteral("featureAllowanceDetail"));
    m_detailLabel->setWordWrap(true);
    m_detailLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(m_detailLabel);
    clear();
}

void FeatureAllowancePanel::setItems(
    std::vector<oap::app::FeatureAllowanceViewItem> items)
{
    const QSignalBlocker blocker(m_tree);
    m_items = std::move(items);
    m_tree->clear();
    if (m_items.empty()) {
        m_statusLabel->setText(QStringLiteral(
            "等待余量结果｜请先执行孔、槽腔或外轮廓 IPW 仿真"));
        auto* empty = new QTreeWidgetItem(m_tree);
        empty->setText(0, QStringLiteral("尚无逐特征余量结果"));
        empty->setDisabled(true);
        m_detailLabel->setText(QStringLiteral(
            "仿真完成后，此处将列出每个孔、槽、腔及外轮廓加工后仍需去除的材料。"));
        return;
    }

    const int unfinished = static_cast<int>(std::count_if(
        m_items.begin(), m_items.end(),
        [](const oap::app::FeatureAllowanceViewItem& item) {
            return item.remainingAllowanceVolume > 1.0e-12;
        }));
    m_statusLabel->setText(QStringLiteral(
        "%1 个特征｜%2 个仍有加工余量；选择一项在主视口单独显示")
        .arg(m_items.size())
        .arg(unfinished));
    for (int row = 0; row < static_cast<int>(m_items.size()); ++row) {
        const auto& item = m_items[static_cast<std::size_t>(row)];
        auto* treeItem = new QTreeWidgetItem(m_tree);
        treeItem->setText(0, item.displayName);
        treeItem->setText(1, QStringLiteral("%1 mm³").arg(
            item.remainingAllowanceVolume, 0, 'f', 3));
        treeItem->setText(2, QStringLiteral("%1%").arg(
            item.completionRatio * 100.0, 0, 'f', 1));
        treeItem->setData(0, kRoleResultRow, row);
        treeItem->setToolTip(0, QStringLiteral("特征标识：%1").arg(item.featureId));
    }
    m_tree->setCurrentItem(m_tree->topLevelItem(0));
    updateCurrent(m_tree->currentItem(), false);
}

void FeatureAllowancePanel::clear()
{
    setItems({});
}

const oap::app::FeatureAllowanceViewItem*
FeatureAllowancePanel::currentItem() const
{
    const QTreeWidgetItem* current = m_tree->currentItem();
    if (!current)
        return nullptr;
    const QVariant value = current->data(0, kRoleResultRow);
    if (!value.isValid())
        return nullptr;
    const int row = value.toInt();
    if (row < 0 || row >= static_cast<int>(m_items.size()))
        return nullptr;
    return &m_items[static_cast<std::size_t>(row)];
}

void FeatureAllowancePanel::activateCurrent()
{
    const QTreeWidgetItem* current = m_tree->currentItem();
    if (!current)
        return;
    const QVariant value = current->data(0, kRoleResultRow);
    if (value.isValid())
        emit allowanceSelected(value.toInt());
}

void FeatureAllowancePanel::updateCurrent(
    QTreeWidgetItem* current,
    bool publishSelection)
{
    if (!current)
        return;
    const QVariant value = current->data(0, kRoleResultRow);
    if (!value.isValid())
        return;
    const int row = value.toInt();
    if (row < 0 || row >= static_cast<int>(m_items.size()))
        return;
    const auto& item = m_items[static_cast<std::size_t>(row)];
    m_detailLabel->setText(QStringLiteral(
        "%1\n工序：%2\n目标去除量：%3 mm³\n边界：%4")
        .arg(item.displayName)
        .arg(item.operationId.isEmpty() ? QStringLiteral("未命名")
                                        : item.operationId)
        .arg(item.targetRemovalVolume, 0, 'f', 3)
        .arg(item.approximateBoundary
            ? QStringLiteral("保守审计边界近似")
            : QStringLiteral("精确特征范围")));
    if (publishSelection)
        emit allowanceSelected(row);
}
