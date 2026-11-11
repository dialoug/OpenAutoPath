// HoleToolTreePanel 实现 — 参见 HoleToolTreePanel.h。
#include "app/ui/panels/HoleToolTreePanel.h"
#include <QCoreApplication>

#include <QAbstractItemView>
#include <QApplication>
#include <QBrush>
#include <QFont>
#include <QFrame>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMap>
#include <QPalette>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QStringList>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <algorithm>

namespace {
constexpr int kRoleBatchIndex = Qt::UserRole + 1;
constexpr int kRoleNodeKind = Qt::UserRole + 2;
constexpr int kRoleOperationId = Qt::UserRole + 4;
constexpr int kRoleStartMoveIndex = Qt::UserRole + 5;
constexpr int kRoleVisibilityGroup = Qt::UserRole + 6;
constexpr int kRoleFormalOperationId = Qt::UserRole + 7;
constexpr int kRoleHasPlayback = Qt::UserRole + 8;
constexpr int kToolNode = 2;
constexpr int kOperationNode = 4;

struct OperationVisibilityCounts {
    int visible = 0;
    int total = 0;
};

OperationVisibilityCounts updateVisibilityGroupState(QTreeWidgetItem* item)
{
    if (!item)
        return {};
    if (item->data(0, kRoleNodeKind).toInt() == kOperationNode) {
        return {item->checkState(0) == Qt::Checked ? 1 : 0, 1};
    }

    OperationVisibilityCounts counts;
    for (int index = 0; index < item->childCount(); ++index) {
        const OperationVisibilityCounts child =
            updateVisibilityGroupState(item->child(index));
        counts.visible += child.visible;
        counts.total += child.total;
    }
    if (counts.total > 0) {
        item->setData(0, kRoleVisibilityGroup, true);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(
            0, counts.visible == 0
                   ? Qt::Unchecked
                   : (counts.visible == counts.total
                          ? Qt::Checked : Qt::PartiallyChecked));
    }
    return counts;
}
} // namespace

HoleToolTreePanel::HoleToolTreePanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    auto* visibilityRow = new QHBoxLayout;
    visibilityRow->setContentsMargins(4, 0, 4, 2);
    m_operationVisibilitySummary =
        new QLabel(QCoreApplication::translate("OpenAutoPath", "可见工序 0/0"), this);
    m_operationVisibilitySummary->setObjectName(
        QStringLiteral("operationVisibilitySummary"));
    visibilityRow->addWidget(m_operationVisibilitySummary);
    visibilityRow->addStretch(1);
    m_showAllOperations =
        new QPushButton(QCoreApplication::translate("OpenAutoPath", "显示全部工序"), this);
    m_showAllOperations->setObjectName(
        QStringLiteral("showAllToolpathOperationsButton"));
    m_showAllOperations->setToolTip(
        QCoreApplication::translate("OpenAutoPath", "退出单独显示并勾选当前 Job 的全部加工工序"));
    m_showAllOperations->setEnabled(false);
    visibilityRow->addWidget(m_showAllOperations);
    layout->addLayout(visibilityRow);
    connect(m_showAllOperations, &QPushButton::clicked,
            this, &HoleToolTreePanel::showAllOperations);

    m_tree = new QTreeWidget(this);
    m_tree->setObjectName(QStringLiteral("holeToolTree"));
    m_tree->setColumnCount(2);
    m_tree->setHeaderLabels(
        {QCoreApplication::translate("OpenAutoPath", "工序 / 刀具批次"), QCoreApplication::translate("OpenAutoPath", "操作")});
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_tree->header()->setSectionResizeMode(1, QHeaderView::Interactive);
    m_tree->header()->setStretchLastSection(false);
    const int operationColumnWidth =
        m_tree->fontMetrics().horizontalAdvance(QCoreApplication::translate("OpenAutoPath", "合并操作")) + 12;
    m_tree->header()->resizeSection(1, operationColumnWidth);
    m_tree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tree->setUniformRowHeights(false);
    m_tree->setExpandsOnDoubleClick(true);
    connect(m_tree, &QTreeWidget::itemClicked,
            this, &HoleToolTreePanel::onItemClicked);
    connect(m_tree, &QTreeWidget::itemDoubleClicked,
            this, &HoleToolTreePanel::onItemDoubleClicked);
    connect(m_tree, &QTreeWidget::itemChanged,
            this, &HoleToolTreePanel::onItemChanged);
    connect(m_tree, &QTreeWidget::currentItemChanged,
            this, &HoleToolTreePanel::onCurrentItemChanged);

    auto* inspector = new QGroupBox(QCoreApplication::translate("OpenAutoPath", "工序属性"), this);
    inspector->setObjectName(QStringLiteral("toolpathOperationInspector"));
    auto* inspectorLayout = new QVBoxLayout(inspector);
    inspectorLayout->setContentsMargins(6, 4, 6, 6);
    inspectorLayout->setSpacing(4);
    m_operationInspectorTitle =
        new QLabel(QCoreApplication::translate("OpenAutoPath", "请选择一道加工工序"), inspector);
    QFont inspectorTitleFont = m_operationInspectorTitle->font();
    inspectorTitleFont.setBold(true);
    m_operationInspectorTitle->setFont(inspectorTitleFont);
    auto* inspectorScroll = new QScrollArea(inspector);
    inspectorScroll->setObjectName(
        QStringLiteral("toolpathOperationDetailsScroll"));
    inspectorScroll->setWidgetResizable(true);
    inspectorScroll->setFrameShape(QFrame::NoFrame);
    m_operationInspectorDetails = new QLabel(inspectorScroll);
    m_operationInspectorDetails->setWordWrap(true);
    m_operationInspectorDetails->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_operationInspectorDetails->setTextInteractionFlags(
        Qt::TextSelectableByMouse);
    inspectorScroll->setWidget(m_operationInspectorDetails);
    m_operationInspectorHint = new QLabel(
        QCoreApplication::translate("OpenAutoPath", "工序勾选控制可见性；“定位”控制播放位置。"),
        inspector);
    m_operationInspectorHint->setWordWrap(true);
    QPalette hintPalette = m_operationInspectorHint->palette();
    hintPalette.setColor(
        QPalette::WindowText,
        palette().color(QPalette::Disabled, QPalette::Text));
    m_operationInspectorHint->setPalette(hintPalette);
    inspectorLayout->addWidget(m_operationInspectorTitle);
    inspectorLayout->addWidget(inspectorScroll, 1);
    inspectorLayout->addWidget(m_operationInspectorHint);
    auto* inspectorActions = new QHBoxLayout;
    m_editOperation = new QPushButton(QCoreApplication::translate("OpenAutoPath", "编辑参数"), inspector);
    m_editOperation->setObjectName(
        QStringLiteral("editToolpathOperationButton"));
    m_regenerateOperation =
        new QPushButton(QCoreApplication::translate("OpenAutoPath", "应用并重建"), inspector);
    m_regenerateOperation->setObjectName(
        QStringLiteral("regenerateToolpathOperationButton"));
    m_previewOperation = new QPushButton(QCoreApplication::translate("OpenAutoPath", "显示此类刀路"), inspector);
    m_previewOperation->setObjectName(QStringLiteral("previewToolpathFamilyButton"));
    m_previewOperation->setToolTip(QCoreApplication::translate("OpenAutoPath", "切换到此类已有的有效刀路；不会重新生成或修改正式工序"));
    m_previewOperation->setEnabled(false);
    m_editOperation->setEnabled(false);
    m_regenerateOperation->setEnabled(false);
    inspectorActions->addWidget(m_editOperation);
    inspectorActions->addWidget(m_regenerateOperation);
    inspectorActions->addStretch(1);
    inspectorLayout->addLayout(inspectorActions);
    inspectorLayout->addWidget(m_previewOperation);
    connect(m_editOperation, &QPushButton::clicked,
            this, &HoleToolTreePanel::editSelectedOperation);
    connect(m_regenerateOperation, &QPushButton::clicked,
            this, &HoleToolTreePanel::regenerateSelectedOperation);
    connect(m_previewOperation, &QPushButton::clicked,
            this, &HoleToolTreePanel::previewSelectedOperation);
    auto* splitter = new QSplitter(Qt::Vertical, this);
    splitter->setObjectName(QStringLiteral("toolpathOperationSplitter"));
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(m_tree);
    splitter->addWidget(inspector);
    splitter->setStretchFactor(0, 4);
    splitter->setStretchFactor(1, 2);
    splitter->setSizes({420, 190});
    layout->addWidget(splitter, 1);
    updateOperationInspector(nullptr);
}

HoleToolTreePanel::~HoleToolTreePanel()
{
    // QWidget deletes its children after our QString/QHash members have been
    // destroyed. QTreeWidget may emit selection changes during that teardown;
    // disconnect while the inspector state is still alive.
    disconnect(m_tree, nullptr, this, nullptr);
}

void HoleToolTreePanel::clearTree()
{
    m_batches.clear();
    m_batchItems.clear();
    m_operationItems.clear();
    m_formalOperationItems.clear();
    m_operationViews.clear();
    m_selectedOperationId.clear();
    m_selectedFormalOperationId.clear();
    m_tree->clear();
    updateOperationVisibilitySummary();
    updateOperationInspector(nullptr);
}

void HoleToolTreePanel::clear()
{
    m_operationVisibility.clear();
    m_soloOperationId.clear();
    clearTree();
}

void HoleToolTreePanel::resetOperationVisibility()
{
    showAllOperations();
}

void HoleToolTreePanel::setGenerationAvailability(
    bool hole, bool pocket, bool profile, bool engraving)
{
    m_holeGenerationAvailable = hole;
    m_pocketGenerationAvailable = pocket;
    m_profileGenerationAvailable = profile;
    m_engravingGenerationAvailable = engraving;
    updateOperationInspector(m_tree->currentItem());
}

void HoleToolTreePanel::setProgram(
    const QVector<oap::app::HoleToolBatchView>& batches)
{
    const QString previouslySelectedFormalOperation =
        m_selectedFormalOperationId;
    clearTree();
    if (batches.isEmpty())
        return;
    m_batches = batches;
    std::stable_sort(m_batches.begin(), m_batches.end(),
                     [](const oap::app::HoleToolBatchView& left,
                        const oap::app::HoleToolBatchView& right) {
        if (left.sequence >= 0 && right.sequence >= 0 &&
            left.sequence != right.sequence) {
            return left.sequence < right.sequence;
        }
        if (left.setupOrder != right.setupOrder)
            return left.setupOrder < right.setupOrder;
        return left.startMoveIndex < right.startMoveIndex;
    });

    m_tree->setUpdatesEnabled(false);
    struct SetupNode {
        QString label;
        QTreeWidgetItem* item = nullptr;
    };
    const bool hasHoleBatches = std::any_of(
        m_batches.begin(), m_batches.end(),
        [](const oap::app::HoleToolBatchView& batch) {
            return batch.family == oap::app::ToolpathBatchFamily::Hole;
        });
    const bool hasPocketBatches = std::any_of(
        m_batches.begin(), m_batches.end(),
        [](const oap::app::HoleToolBatchView& batch) {
            return batch.family == oap::app::ToolpathBatchFamily::Pocket;
        });
    const bool hasProfileBatches = std::any_of(
        m_batches.begin(), m_batches.end(),
        [](const oap::app::HoleToolBatchView& batch) {
            return batch.family == oap::app::ToolpathBatchFamily::Profile;
        });
    const bool hasFinishingBatches = std::any_of(m_batches.begin(), m_batches.end(),
        [](const auto& batch) { return batch.family == oap::app::ToolpathBatchFamily::SurfaceFinishing; });
    auto* jobRoot = new QTreeWidgetItem(m_tree);
    jobRoot->setText(0, QCoreApplication::translate("OpenAutoPath", "加工 Job"));
    jobRoot->setToolTip(
        0, QCoreApplication::translate("OpenAutoPath", "当前内存 Job 的装夹、刀具与工序视图"));
    QFont jobFont = jobRoot->font(0);
    jobFont.setBold(true);
    jobRoot->setFont(0, jobFont);
    jobRoot->setExpanded(true);

    QTreeWidgetItem* holeRoot = nullptr;
    QTreeWidgetItem* pocketRoot = nullptr;
    QTreeWidgetItem* profileRoot = nullptr;
    QTreeWidgetItem* finishingRoot = nullptr;
    if (hasHoleBatches) {
        holeRoot = new QTreeWidgetItem(jobRoot);
        holeRoot->setText(0, QCoreApplication::translate("OpenAutoPath", "孔加工工序"));
        holeRoot->setExpanded(true);
    }
    if (hasPocketBatches) {
        pocketRoot = new QTreeWidgetItem(jobRoot);
        pocketRoot->setText(0, QCoreApplication::translate("OpenAutoPath", "槽腔加工工序"));
        pocketRoot->setExpanded(true);
    }
    if (hasProfileBatches) {
        profileRoot = new QTreeWidgetItem(jobRoot);
        profileRoot->setText(0, QCoreApplication::translate("OpenAutoPath", "外轮廓加工工序"));
        profileRoot->setExpanded(true);
    }
    if (hasFinishingBatches) {
        finishingRoot = new QTreeWidgetItem(jobRoot);
        finishingRoot->setText(0, QCoreApplication::translate("OpenAutoPath", "雕刻加工工序"));
        finishingRoot->setExpanded(true);
    }
    QMap<int, SetupNode> holeSetupNodes;
    QMap<int, SetupNode> pocketSetupNodes;
    QMap<int, SetupNode> profileSetupNodes;
    QMap<int, SetupNode> finishingSetupNodes;
    const auto nodesFor = [&](oap::app::ToolpathBatchFamily family)
        -> QMap<int, SetupNode>& {
        switch (family) {
            case oap::app::ToolpathBatchFamily::Hole:
                return holeSetupNodes;
            case oap::app::ToolpathBatchFamily::Pocket:
                return pocketSetupNodes;
            case oap::app::ToolpathBatchFamily::Profile:
                return profileSetupNodes;
            case oap::app::ToolpathBatchFamily::SurfaceFinishing:
                return finishingSetupNodes;
        }
        return pocketSetupNodes;
    };
    for (const oap::app::HoleToolBatchView& batch : m_batches) {
        QMap<int, SetupNode>& nodes = nodesFor(batch.family);
        nodes[batch.setupOrder].label = batch.setupLabel;
    }
    const auto createSetupItems = [](QTreeWidgetItem* root,
                                     QMap<int, SetupNode>& nodes) {
        for (auto setup = nodes.begin(); setup != nodes.end(); ++setup) {
            setup->item = new QTreeWidgetItem(root);
            setup->item->setText(0, setup->label);
            setup->item->setSizeHint(0, QSize(0, 26));
        }
    };
    createSetupItems(holeRoot, holeSetupNodes);
    createSetupItems(pocketRoot, pocketSetupNodes);
    createSetupItems(profileRoot, profileSetupNodes);
    createSetupItems(finishingRoot, finishingSetupNodes);

    m_batchItems.resize(m_batches.size());
    const auto addToolItem = [this](int index, QTreeWidgetItem* parent) {
        const oap::app::HoleToolBatchView& batch = m_batches[index];
        auto* toolItem = new QTreeWidgetItem(parent);
        toolItem->setText(0, batch.toolLabel);
        toolItem->setToolTip(0, batch.detailText);
        const bool batchHasPlayback = batch.batchIndex >= 0 && batch.endMoveIndex > batch.startMoveIndex;
        toolItem->setText(1, batchHasPlayback ? QCoreApplication::translate("OpenAutoPath", "定位") : QCoreApplication::translate("OpenAutoPath", "未载入预览"));
        toolItem->setData(0, kRoleHasPlayback, batchHasPlayback);
        toolItem->setData(0, kRoleBatchIndex, batch.batchIndex);
        toolItem->setData(0, kRoleNodeKind, kToolNode);
        for (const oap::app::ToolpathOperationView& operation :
             batch.operations) {
            auto* operationItem = new QTreeWidgetItem(toolItem);
            operationItem->setText(0, operation.label);
            const bool hasPlayback = operation.hasPlayback && operation.startMoveIndex >= 0 &&
                operation.endMoveIndex > operation.startMoveIndex;
            operationItem->setText(1, hasPlayback ? QCoreApplication::translate("OpenAutoPath", "定位") : QCoreApplication::translate("OpenAutoPath", "未载入预览"));
            operationItem->setData(0, kRoleHasPlayback, hasPlayback);
            operationItem->setToolTip(0, operation.detailText);
            operationItem->setData(0, kRoleNodeKind, kOperationNode);
            operationItem->setData(
                0, kRoleOperationId, operation.operationId);
            const QString formalOperationId =
                operation.formalOperationId.isEmpty()
                    ? operation.operationId
                    : operation.formalOperationId;
            operationItem->setData(
                0, kRoleFormalOperationId, formalOperationId);
            operationItem->setData(
                0, kRoleStartMoveIndex, operation.startMoveIndex);
            operationItem->setFlags(
                operationItem->flags() | Qt::ItemIsUserCheckable);
            if (!m_operationVisibility.contains(operation.operationId))
                m_operationVisibility.insert(operation.operationId, true);
            operationItem->setCheckState(
                0, m_operationVisibility.value(operation.operationId)
                       ? Qt::Checked : Qt::Unchecked);
            m_operationItems.insert(operation.operationId, operationItem);
            if (!formalOperationId.isEmpty() &&
                !m_formalOperationItems.contains(formalOperationId)) {
                m_formalOperationItems.insert(
                    formalOperationId, operationItem);
            }
            m_operationViews.insert(operation.operationId, operation);
        }
        m_batchItems[index] = toolItem;
    };

    for (int index = 0; index < m_batches.size(); ++index) {
        const auto& batch = m_batches[index];
        addToolItem(index, nodesFor(batch.family)[batch.setupOrder].item);
    }

    if (holeRoot && holeRoot->childCount() > 0)
        holeRoot->child(0)->setExpanded(true);
    if (pocketRoot && pocketRoot->childCount() > 0)
        pocketRoot->child(0)->setExpanded(true);
    if (profileRoot && profileRoot->childCount() > 0)
        profileRoot->child(0)->setExpanded(true);
    m_tree->setUpdatesEnabled(true);

    for (auto it = m_operationVisibility.begin();
         it != m_operationVisibility.end();) {
        if (!m_operationItems.contains(it.key()))
            it = m_operationVisibility.erase(it);
        else
            ++it;
    }
    if (!m_soloOperationId.isEmpty() &&
        !m_operationItems.contains(m_soloOperationId)) {
        m_soloOperationId.clear();
        emit operationSoloRequested({});
    }
    updateOperationPresentation();
    if (!previouslySelectedFormalOperation.isEmpty() &&
        m_formalOperationItems.contains(previouslySelectedFormalOperation)) {
        selectFormalOperation(previouslySelectedFormalOperation);
    } else {
        setPlaybackPosition(0);
    }
}

void HoleToolTreePanel::setPlaybackPosition(int completedMoveCount)
{
    if (m_batches.isEmpty() || m_batchItems.size() != m_batches.size())
        return;

    QVector<int> playable;
    for (int index = 0; index < m_batches.size(); ++index) {
        if (m_batches[index].batchIndex >= 0 &&
            m_batches[index].endMoveIndex >
                m_batches[index].startMoveIndex) {
            playable.push_back(index);
        }
    }
    if (playable.isEmpty()) {
        updateOperationInspector(nullptr);
        return;
    }

    const int position = std::max(0, completedMoveCount);
    int currentIndex = playable.front();
    if (position >= m_batches[playable.back()].endMoveIndex) {
        currentIndex = playable.back();
    } else {
        for (const int index : playable) {
            if (position < m_batches[index].endMoveIndex) {
                currentIndex = index;
                break;
            }
        }
    }

    QSignalBlocker blocker(m_tree);
    m_tree->clearSelection();
    const QBrush completedBrush(
        palette().color(QPalette::Disabled, QPalette::Text));
    for (const int index : playable) {
        QTreeWidgetItem* item = m_batchItems[index];
        const bool completed = m_batches[index].endMoveIndex <= position;
        const bool current = index == currentIndex;
        item->setForeground(0, completed ? completedBrush : QBrush());
        item->setData(
            0, Qt::AccessibleDescriptionRole,
            completed
                ? (current ? QCoreApplication::translate("OpenAutoPath", "已完成，当前刀具")
                           : QCoreApplication::translate("OpenAutoPath", "已完成"))
                : (current ? QCoreApplication::translate("OpenAutoPath", "当前刀具")
                           : QCoreApplication::translate("OpenAutoPath", "未执行")));
        QFont font = item->font(0);
        font.setBold(current);
        item->setFont(0, font);
    }

    QTreeWidgetItem* currentItem = m_batchItems[currentIndex];
    const QVector<oap::app::ToolpathOperationView>& operations =
        m_batches[currentIndex].operations;
    for (int operationIndex = 0;
         operationIndex < operations.size(); ++operationIndex) {
        const oap::app::ToolpathOperationView& operation =
            operations[operationIndex];
        if (!operation.hasPlayback || operation.endMoveIndex <= operation.startMoveIndex)
            continue;
        if (position < operation.endMoveIndex ||
            (operationIndex == operations.size() - 1 &&
             position == operation.endMoveIndex)) {
            currentItem = m_operationItems.value(
                operation.operationId, currentItem);
            break;
        }
    }
    currentItem->setSelected(true);
    m_tree->setCurrentItem(currentItem);
    for (QTreeWidgetItem* parent = currentItem->parent(); parent;
         parent = parent->parent()) {
        parent->setExpanded(true);
    }
    m_tree->scrollToItem(currentItem, QAbstractItemView::EnsureVisible);
    updateOperationInspector(currentItem);
}

void HoleToolTreePanel::selectOperation(const QString& operationId)
{
    QTreeWidgetItem* item = m_operationItems.value(operationId, nullptr);
    if (!item) {
        selectFormalOperation(operationId);
        return;
    }
    QSignalBlocker blocker(m_tree);
    m_tree->clearSelection();
    item->setSelected(true);
    m_tree->setCurrentItem(item);
    for (QTreeWidgetItem* parent = item->parent(); parent;
         parent = parent->parent()) {
        parent->setExpanded(true);
    }
    m_tree->scrollToItem(item, QAbstractItemView::EnsureVisible);
    updateOperationInspector(item);
}

void HoleToolTreePanel::selectFormalOperation(
    const QString& formalOperationId)
{
    QTreeWidgetItem* item = m_formalOperationItems.value(
        formalOperationId, nullptr);
    if (!item)
        return;
    QSignalBlocker blocker(m_tree);
    m_tree->clearSelection();
    item->setSelected(true);
    m_tree->setCurrentItem(item);
    for (QTreeWidgetItem* parent = item->parent(); parent;
         parent = parent->parent()) {
        parent->setExpanded(true);
    }
    m_tree->scrollToItem(item, QAbstractItemView::EnsureVisible);
    updateOperationInspector(item);
}

QString HoleToolTreePanel::selectedFormalOperationId() const noexcept
{
    return m_selectedFormalOperationId;
}

void HoleToolTreePanel::setSoloOperation(const QString& operationId)
{
    if (m_soloOperationId == operationId)
        return;
    m_soloOperationId = operationId;
    updateOperationPresentation();
}

void HoleToolTreePanel::showAllOperations()
{
    const bool clearSolo = !m_soloOperationId.isEmpty();
    m_soloOperationId.clear();
    QStringList changedIds;
    {
        const QSignalBlocker blocker(m_tree);
        for (auto it = m_operationItems.cbegin();
             it != m_operationItems.cend(); ++it) {
            if (!m_operationVisibility.value(it.key(), true))
                changedIds.push_back(it.key());
            m_operationVisibility.insert(it.key(), true);
            it.value()->setCheckState(0, Qt::Checked);
        }
    }
    if (clearSolo)
        emit operationSoloRequested({});
    for (const QString& operationId : changedIds)
        emit operationVisibilityChanged(operationId, true);
    updateOperationPresentation();
}

void HoleToolTreePanel::setOperationDescendantsVisible(
    QTreeWidgetItem* item, bool visible)
{
    if (!item)
        return;
    if (item->data(0, kRoleNodeKind).toInt() == kOperationNode) {
        const QString operationId =
            item->data(0, kRoleOperationId).toString();
        if (operationId.isEmpty())
            return;
        const bool previous =
            m_operationVisibility.value(operationId, true);
        m_operationVisibility.insert(operationId, visible);
        item->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
        if (previous != visible)
            emit operationVisibilityChanged(operationId, visible);
        return;
    }
    for (int index = 0; index < item->childCount(); ++index)
        setOperationDescendantsVisible(item->child(index), visible);
}

void HoleToolTreePanel::updateOperationHierarchy()
{
    if (!m_tree)
        return;
    const QSignalBlocker blocker(m_tree);
    QTreeWidgetItem* root = m_tree->invisibleRootItem();
    for (int index = 0; index < root->childCount(); ++index)
        updateVisibilityGroupState(root->child(index));
}

void HoleToolTreePanel::updateOperationVisibilitySummary()
{
    const int totalCount = m_operationItems.size();
    int visibleCount = 0;
    if (!m_soloOperationId.isEmpty() &&
        m_operationItems.contains(m_soloOperationId)) {
        visibleCount = 1;
    } else {
        for (auto it = m_operationItems.cbegin();
             it != m_operationItems.cend(); ++it) {
            if (m_operationVisibility.value(it.key(), true))
                ++visibleCount;
        }
    }
    const bool soloMode = !m_soloOperationId.isEmpty();
    if (m_operationVisibilitySummary) {
        m_operationVisibilitySummary->setText(
            soloMode
                ? QCoreApplication::translate("OpenAutoPath", "单独显示 1/%1 工序").arg(totalCount)
                : QCoreApplication::translate("OpenAutoPath", "可见工序 %1/%2")
                      .arg(visibleCount).arg(totalCount));
        m_operationVisibilitySummary->setToolTip(
            visibleCount == 0 && totalCount > 0
                ? QCoreApplication::translate("OpenAutoPath", "全部刀路已由工序树隐藏")
                : QCoreApplication::translate("OpenAutoPath", "工序勾选状态独立于刀路段起始/数量范围"));
    }
    if (m_showAllOperations)
        m_showAllOperations->setEnabled(
            totalCount > 0 &&
            (visibleCount < totalCount || soloMode));
    emit operationVisibilitySummaryChanged(
        visibleCount, totalCount, soloMode);
}

void HoleToolTreePanel::updateOperationPresentation()
{
    {
        const QSignalBlocker blocker(m_tree);
        for (auto it = m_operationItems.cbegin();
             it != m_operationItems.cend(); ++it) {
            QTreeWidgetItem* item = it.value();
            const bool solo = !m_soloOperationId.isEmpty() &&
                it.key() == m_soloOperationId;
            QFont font = item->font(0);
            font.setBold(solo);
            item->setFont(0, font);
            item->setText(
                1, !item->data(0, kRoleHasPlayback).toBool() ? QCoreApplication::translate("OpenAutoPath", "未载入预览")
                        : solo ? QCoreApplication::translate("OpenAutoPath", "定位 · 单独") : QCoreApplication::translate("OpenAutoPath", "定位"));
        }
    }
    updateOperationHierarchy();
    updateOperationVisibilitySummary();
}

void HoleToolTreePanel::updateOperationInspector(QTreeWidgetItem* item)
{
    m_selectedOperationId.clear();
    m_selectedFormalOperationId.clear();
    if (!item ||
        item->data(0, kRoleNodeKind).toInt() != kOperationNode) {
        m_operationInspectorTitle->setText(
            QCoreApplication::translate("OpenAutoPath", "请选择一道加工工序"));
        m_operationInspectorDetails->clear();
        m_operationInspectorDetails->setToolTip({});
        m_operationInspectorHint->setText(
            QCoreApplication::translate("OpenAutoPath", "工序勾选控制可见性；“定位”控制播放位置。"));
        m_editOperation->setText(QCoreApplication::translate("OpenAutoPath", "编辑参数"));
        m_regenerateOperation->setText(QCoreApplication::translate("OpenAutoPath", "应用并重建"));
        m_editOperation->setEnabled(false);
        m_regenerateOperation->setEnabled(false);
        m_previewOperation->setEnabled(false);
        return;
    }

    const QString operationId =
        item->data(0, kRoleOperationId).toString();
    const auto found = m_operationViews.constFind(operationId);
    if (found == m_operationViews.cend()) {
        updateOperationInspector(nullptr);
        return;
    }

    const oap::app::ToolpathOperationView& operation = found.value();
    m_selectedOperationId = operation.operationId;
    m_selectedFormalOperationId = operation.formalOperationId.isEmpty()
        ? operation.operationId : operation.formalOperationId;
    // An inactive family has no playback rows, but can still have valid stored
    // results. The application validates those results before switching.
    m_previewOperation->setEnabled(!m_selectedFormalOperationId.isEmpty());
    m_operationInspectorTitle->setText(operation.label);
    m_operationInspectorDetails->setText(operation.detailText);
    m_operationInspectorDetails->setToolTip(operation.detailText);
    if (operation.family == oap::app::ToolpathBatchFamily::Hole) {
        m_operationInspectorHint->setText(QCoreApplication::translate("OpenAutoPath", "孔加工使用项目孔参数配置的刀库或单刀；修改和重新生成作用于整个孔工序组。"));
        m_editOperation->setText(QCoreApplication::translate("OpenAutoPath", "编辑项目孔参数"));
        m_regenerateOperation->setText(QCoreApplication::translate("OpenAutoPath", "重建全部孔工序"));
        m_editOperation->setEnabled(true);
        m_regenerateOperation->setEnabled(m_holeGenerationAvailable);
        m_editOperation->setToolTip(QCoreApplication::translate("OpenAutoPath", "打开统一参数页的孔参数，包括刀库或单刀来源"));
        m_regenerateOperation->setToolTip(QCoreApplication::translate("OpenAutoPath", "按当前孔参数与刀具来源重建整个孔工序组"));
    } else if (operation.family ==
               oap::app::ToolpathBatchFamily::Pocket) {
        m_operationInspectorHint->setText(
            QCoreApplication::translate("OpenAutoPath", "槽腔使用项目槽腔参数配置的刀库或单刀；修改和重新生成作用于整个槽腔工序组。"));
        m_editOperation->setText(QCoreApplication::translate("OpenAutoPath", "编辑槽腔参数"));
        m_regenerateOperation->setText(QCoreApplication::translate("OpenAutoPath", "重建全部槽腔工序"));
        m_editOperation->setEnabled(true);
        m_regenerateOperation->setEnabled(m_pocketGenerationAvailable);
        m_editOperation->setToolTip(
            QCoreApplication::translate("OpenAutoPath", "打开统一参数页的槽腔参数，包括刀库或单刀来源"));
        m_regenerateOperation->setToolTip(
            QCoreApplication::translate("OpenAutoPath", "按当前槽腔参数与刀具来源重建整个槽腔工序组"));
    } else if (operation.family ==
               oap::app::ToolpathBatchFamily::Profile) {
        m_operationInspectorHint->setText(
            QCoreApplication::translate("OpenAutoPath", "外轮廓工序使用当前项目参数快照；保存候选参数后必须重新生成并验证其下游 IPW。"));
        m_editOperation->setText(QCoreApplication::translate("OpenAutoPath", "编辑外轮廓参数"));
        m_regenerateOperation->setText(QCoreApplication::translate("OpenAutoPath", "重建全部外轮廓工序"));
        m_editOperation->setEnabled(true);
        m_regenerateOperation->setEnabled(m_profileGenerationAvailable);
        m_editOperation->setToolTip(
            QCoreApplication::translate("OpenAutoPath", "打开项目刀路参数编辑器并定位外轮廓参数"));
        m_regenerateOperation->setToolTip(
            QCoreApplication::translate("OpenAutoPath", "按当前外轮廓参数与刀具来源重建整个外轮廓工序组"));
    } else if (operation.family == oap::app::ToolpathBatchFamily::SurfaceFinishing) {
        m_operationInspectorHint->setText(QCoreApplication::translate("OpenAutoPath", "在统一刀路参数页配置雕刻刀库或单刀及加工面；重新生成整个雕刻工序组，并使变化后的 IPW 失效。"));
        m_editOperation->setText(QCoreApplication::translate("OpenAutoPath", "编辑雕刻参数"));
        m_regenerateOperation->setText(QCoreApplication::translate("OpenAutoPath", "重建全部雕刻工序"));
        m_editOperation->setEnabled(true);
        m_regenerateOperation->setEnabled(m_engravingGenerationAvailable);
        m_editOperation->setToolTip(QCoreApplication::translate("OpenAutoPath", "打开统一刀路参数的雕刻页"));
        m_regenerateOperation->setToolTip(QCoreApplication::translate("OpenAutoPath", "按当前雕刻配置重新生成整个雕刻工序组"));
    }
}

void HoleToolTreePanel::onCurrentItemChanged(
    QTreeWidgetItem* current, QTreeWidgetItem*)
{
    updateOperationInspector(current);
    if (!m_selectedFormalOperationId.isEmpty())
        emit formalOperationSelected(m_selectedFormalOperationId);
}

void HoleToolTreePanel::editSelectedOperation()
{
    if (!m_editOperation->isEnabled())
        return;
    const auto found = m_operationViews.constFind(m_selectedOperationId);
    if (found == m_operationViews.cend())
        return;
    const oap::app::ToolpathOperationView& operation = found.value();
    if (operation.family == oap::app::ToolpathBatchFamily::Hole) {
        emit editHoleParametersRequested();
        return;
    }
    switch (operation.family) {
        case oap::app::ToolpathBatchFamily::Pocket:
            emit editPocketOperationRequested();
            break;
        case oap::app::ToolpathBatchFamily::Profile:
            emit editProfileOperationRequested();
            break;
        case oap::app::ToolpathBatchFamily::SurfaceFinishing:
            emit editFinishingOperationRequested(m_selectedFormalOperationId);
            break;
        case oap::app::ToolpathBatchFamily::Hole:
            break;
    }
}

void HoleToolTreePanel::previewSelectedOperation()
{
    if (m_previewOperation->isEnabled() && !m_selectedFormalOperationId.isEmpty())
        emit previewOperationRequested(m_selectedFormalOperationId);
}

void HoleToolTreePanel::regenerateSelectedOperation()
{
    if (!m_regenerateOperation->isEnabled())
        return;
    const auto found = m_operationViews.constFind(m_selectedOperationId);
    if (found == m_operationViews.cend())
        return;
    switch (found->family) {
        case oap::app::ToolpathBatchFamily::Hole:
            emit regenerateHoleRequested();
            break;
        case oap::app::ToolpathBatchFamily::Pocket:
            emit regeneratePocketRequested();
            break;
        case oap::app::ToolpathBatchFamily::Profile:
            emit regenerateProfileRequested();
            break;
        case oap::app::ToolpathBatchFamily::SurfaceFinishing:
            emit regenerateFinishingRequested(m_selectedFormalOperationId);
            break;
    }
}

void HoleToolTreePanel::onItemClicked(QTreeWidgetItem* item, int column)
{
    if (!item)
        return;
    const int nodeKind = item->data(0, kRoleNodeKind).toInt();
    if (nodeKind == kToolNode && column == 1 && item->data(0, kRoleHasPlayback).toBool()) {
        const int batchIndex = item->data(0, kRoleBatchIndex).toInt();
        if (batchIndex >= 0)
            emit toolBatchActivated(batchIndex);
    } else if (nodeKind == kOperationNode && column == 1 && item->data(0, kRoleHasPlayback).toBool()) {
        const QString operationId =
            item->data(0, kRoleOperationId).toString();
        const int startMoveIndex =
            item->data(0, kRoleStartMoveIndex).toInt();
        if (!operationId.isEmpty() && startMoveIndex >= 0) {
            emit operationActivated(
                operationId, startMoveIndex);
        }
    }
}

void HoleToolTreePanel::onItemDoubleClicked(QTreeWidgetItem* item, int)
{
    if (!item)
        return;
    const int nodeKind = item->data(0, kRoleNodeKind).toInt();
    if (nodeKind == kOperationNode && item->data(0, kRoleHasPlayback).toBool()) {
        const QString operationId =
            item->data(0, kRoleOperationId).toString();
        if (operationId.isEmpty())
            return;
        const QString soloOperationId =
            m_soloOperationId == operationId ? QString{} : operationId;
        setSoloOperation(soloOperationId);
        emit operationSoloRequested(soloOperationId);
    }
}

void HoleToolTreePanel::onItemChanged(QTreeWidgetItem* item, int column)
{
    if (!item || column != 0)
        return;

    const int nodeKind = item->data(0, kRoleNodeKind).toInt();
    const bool visibilityGroup =
        item->data(0, kRoleVisibilityGroup).toBool();
    if (nodeKind != kOperationNode && !visibilityGroup)
        return;

    if (!m_soloOperationId.isEmpty()) {
        m_soloOperationId.clear();
        emit operationSoloRequested({});
    }

    if (visibilityGroup) {
        const bool visible = item->checkState(0) == Qt::Checked;
        {
            const QSignalBlocker blocker(m_tree);
            setOperationDescendantsVisible(item, visible);
        }
        updateOperationPresentation();
        return;
    }

    const QString operationId =
        item->data(0, kRoleOperationId).toString();
    if (operationId.isEmpty())
        return;
    const bool visible = item->checkState(0) == Qt::Checked;
    if (m_operationVisibility.value(operationId, true) == visible)
        return;
    m_operationVisibility.insert(operationId, visible);
    emit operationVisibilityChanged(operationId, visible);
    updateOperationPresentation();
}
