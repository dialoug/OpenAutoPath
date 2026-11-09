// FeaturePanel 实现 — 参见 FeaturePanel.h。
#include "FeaturePanel.h"

#include "StickyTreeWidget.h"

#include <QVBoxLayout>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>
#include <QSignalBlocker>

#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <functional>

namespace {
// 在每个树节点上存放其 face_id(仅面叶子)与 kind 标记,使选择能映射回模型。
constexpr int kRoleId   = Qt::UserRole + 1; // 面叶子的 face_id
constexpr int kRoleKind = Qt::UserRole + 2; // "face"(叶子) 或 "group"(分组/类别/实例)

// 特征类别 -> 树中显示的中文名。
const char* kindLabel(oap::FeatureKind k) {
    switch (k) {
        case oap::FeatureKind::ThroughHole:     return "通孔";
        case oap::FeatureKind::BlindHole:       return "盲孔";
        case oap::FeatureKind::CountersinkHole: return "沉孔";
        case oap::FeatureKind::Slot:            return "槽";
        case oap::FeatureKind::Pocket:          return "腔";
    }
    return "特征";
}
} // namespace

FeaturePanel::FeaturePanel(QWidget* parent) : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_tree = new StickyTreeWidget(this);
    m_tree->setHeaderLabels({QStringLiteral("特征 / 面"), QStringLiteral("数量")});
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_tree->setColumnWidth(1, 60);
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection); // Ctrl/Shift 多选
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &FeaturePanel::onItemSelectionChanged);
    connect(m_tree, &QWidget::customContextMenuRequested, this,
            [this](const QPoint& pos) {
                emit contextMenuRequested(m_tree->viewport()->mapToGlobal(pos));
            });
    layout->addWidget(m_tree);
}

void FeaturePanel::clear()
{
    m_tree->clear();
}

bool FeaturePanel::isEmpty() const
{
    return m_tree->topLevelItemCount() == 0;
}

void FeaturePanel::setResult(
    const oap::RecognitionResult& res,
    const std::unordered_map<std::string, std::string>& holeStrategyBadges)
{
    m_tree->clear();
    if (res.instances.empty())
        return;
    m_tree->setUpdatesEnabled(false);
    appendRecognitionResult(
        m_tree->invisibleRootItem(), res,
        holeStrategyBadges);
    m_tree->setUpdatesEnabled(true);
    m_tree->expandToDepth(0);
}

void FeaturePanel::appendRecognitionResult(
    QTreeWidgetItem* parent,
    const oap::RecognitionResult& res,
    const std::unordered_map<std::string, std::string>& holeStrategyBadges)
{
    // 类别显示顺序固定：孔（通/盲/沉），然后槽、腔。
    static const oap::FeatureKind kOrder[] = {
        oap::FeatureKind::ThroughHole, oap::FeatureKind::BlindHole,
        oap::FeatureKind::CountersinkHole, oap::FeatureKind::Slot,
        oap::FeatureKind::Pocket};

    // 按类别归集实例(保持识别/添加顺序,即 instances 中的次序)。
    std::map<int, std::vector<const oap::RecognitionResult::Instance*>> byKind;
    for (const auto& inst : res.instances)
        byKind[int(inst.kind)].push_back(&inst);

    auto addFaceLeaf = [](QTreeWidgetItem* parent, const std::string& fid) {
        auto* it = new QTreeWidgetItem(parent);
        it->setText(0, QString::fromStdString(fid));
        it->setData(0, kRoleId, QString::fromStdString(fid));
        it->setData(0, kRoleKind, QStringLiteral("face"));
    };

    // "Holes" 顶层分组,下挂通/盲/沉三个子类别。其余类别与 Holes 平级。
    QTreeWidgetItem* holesRoot = nullptr;
    auto holeGroup = [&]() -> QTreeWidgetItem* {
        if (!holesRoot) {
            holesRoot = new QTreeWidgetItem(parent);
            holesRoot->setText(0, QStringLiteral("孔"));
            holesRoot->setData(0, kRoleKind, QStringLiteral("group"));
        }
        return holesRoot;
    };

    for (oap::FeatureKind k : kOrder) {
        auto it = byKind.find(int(k));
        if (it == byKind.end() || it->second.empty()) continue;
        const bool isHole = (k == oap::FeatureKind::ThroughHole ||
                             k == oap::FeatureKind::BlindHole ||
                             k == oap::FeatureKind::CountersinkHole);
        QTreeWidgetItem* categoryParent =
            isHole ? holeGroup() : parent;
        // 类别节点:Count 列显示该类别的"结构实例数"。
        auto* cat = new QTreeWidgetItem(categoryParent);
        cat->setText(0, QString::fromUtf8(kindLabel(k)));
        cat->setText(1, QString::number((int)it->second.size()));
        cat->setData(0, kRoleKind, QStringLiteral("group"));
        // 每个实例一个节点:命名 "<类别> <序号>",下挂其组成面;Count 列为面数。
        int idx = 1;
        for (const oap::RecognitionResult::Instance* inst : it->second) {
            auto* node = new QTreeWidgetItem(cat);
            // 人工复核新建的结构以 ⚙ 前缀标记,与自动识别区分。
            const QString prefix = inst->manual ? QStringLiteral("⚙ ") : QString();
            QString strategyBadge;
            for (const std::string& faceId : inst->faceIds) {
                const auto badge = holeStrategyBadges.find(faceId);
                if (badge != holeStrategyBadges.end()) {
                    strategyBadge = QStringLiteral(" · ⚙ %1")
                                        .arg(QString::fromStdString(badge->second));
                    break;
                }
            }
            QString floorBadge;
            if (inst->manual && !inst->floorFaceId.empty() &&
                (k == oap::FeatureKind::Slot || k == oap::FeatureKind::Pocket)) {
                floorBadge = QStringLiteral(" · 底面 %1")
                                 .arg(QString::fromStdString(inst->floorFaceId));
            }
            node->setText(0, QStringLiteral("%1%2 %3")
                                  .arg(prefix)
                                  .arg(QString::fromUtf8(kindLabel(k)))
                                  .arg(idx++) + floorBadge + strategyBadge);
            node->setText(1, QString::number((int)inst->faceIds.size()));
            node->setData(0, kRoleKind, QStringLiteral("group"));
            for (const auto& fid : inst->faceIds) addFaceLeaf(node, fid);
        }
    }
    if (holesRoot) {
        int n = 0;
        for (int i = 0; i < holesRoot->childCount(); ++i)
            n += holesRoot->child(i)->childCount();
        holesRoot->setText(1, QString::number(n));
        holesRoot->setExpanded(true);
    }
}

void FeaturePanel::setWireTraceFeatureSubsets(
    const std::vector<WireTraceFeatureSubset>& subsets)
{
    m_tree->clear();
    if (subsets.empty())
        return;
    m_tree->setUpdatesEnabled(false);
    appendWireTraceFeatureSubsets(
        m_tree->invisibleRootItem(), subsets);
    m_tree->setUpdatesEnabled(true);
    m_tree->expandToDepth(1);
}

void FeaturePanel::appendWireTraceFeatureSubsets(
    QTreeWidgetItem* parent,
    const std::vector<WireTraceFeatureSubset>& subsets)
{
    auto addFaceLeaf = [](QTreeWidgetItem* parent, const std::string& faceId) {
        auto* item = new QTreeWidgetItem(parent);
        item->setText(0, QString::fromStdString(faceId));
        item->setData(0, kRoleId, QString::fromStdString(faceId));
        item->setData(0, kRoleKind, QStringLiteral("face"));
    };
    const auto featureLabel = [](oap::AxisWireTraceFeatureKind kind) {
        return kind == oap::AxisWireTraceFeatureKind::BlindSlot
                   ? QStringLiteral("盲槽")
                   : QStringLiteral("盲腔");
    };
    struct RootGroup {
        std::string axisLabel;
        std::string rootFaceId;
        std::vector<const WireTraceFeatureSubset*> subsets;
    };
    std::vector<RootGroup> rootGroups;
    std::map<std::pair<std::string, std::string>, std::size_t> rootGroupIndices;
    for (const WireTraceFeatureSubset& subset : subsets) {
        const auto key = std::make_pair(subset.axisLabel, subset.rootFaceId);
        auto [groupIt, inserted] =
            rootGroupIndices.emplace(key, rootGroups.size());
        if (inserted) {
            rootGroups.push_back(
                RootGroup{
                    subset.axisLabel, subset.rootFaceId, {}});
        }
        rootGroups[groupIt->second].subsets.push_back(&subset);
    }

    for (const RootGroup& group : rootGroups) {
        auto* root = new QTreeWidgetItem(parent);
        root->setText(
            0, QStringLiteral("%1向根面 %2")
                   .arg(QString::fromStdString(group.axisLabel),
                        QString::fromStdString(group.rootFaceId)));
        root->setText(1, QString::number(static_cast<int>(group.subsets.size())));
        root->setData(0, kRoleKind, QStringLiteral("group"));

        int innerOrdinal = 0;
        int outerOrdinal = 0;
        int mergedPocketOrdinal = 0;
        for (const WireTraceFeatureSubset* subset : group.subsets) {
            int& ordinal =
                subset->fromInnerWire ? innerOrdinal : outerOrdinal;
            auto* node = new QTreeWidgetItem(root);
            if (subset->featureKind ==
                    oap::AxisWireTraceFeatureKind::BlindPocket &&
                subset->mergedSourceCount > 1) {
                ++mergedPocketOrdinal;
                node->setText(
                    0, QStringLiteral(
                           "连通盲腔 %1 · 合并 %2 个原始子集")
                           .arg(mergedPocketOrdinal)
                           .arg(static_cast<qulonglong>(
                               subset->mergedSourceCount)));
            } else {
                ++ordinal;
                node->setText(
                    0, QStringLiteral("%1环特征子集 %2 · %3")
                           .arg(subset->fromInnerWire
                                    ? QStringLiteral("内")
                                    : QStringLiteral("外"))
                           .arg(ordinal)
                           .arg(featureLabel(subset->featureKind)));
            }
            node->setText(
                1, QString::number(static_cast<int>(subset->faceIds.size())));
            node->setData(0, kRoleKind, QStringLiteral("group"));
            for (const std::string& faceId : subset->faceIds)
                addFaceLeaf(node, faceId);
        }
        root->setExpanded(true);
    }
}

void FeaturePanel::setInnerOuterStructureResult(
    const oap::RecognitionResult& res,
    const std::unordered_map<std::string, std::string>&
        holeStrategyBadges,
    const std::unordered_set<std::string>& throughStructureFaceIds,
    const std::vector<WireTraceFeatureSubset>& wireTraceSubsets)
{
    m_tree->clear();
    m_tree->setUpdatesEnabled(false);

    appendRecognitionResult(
        m_tree->invisibleRootItem(), res,
        holeStrategyBadges);

    if (!throughStructureFaceIds.empty()) {
        auto* throughRoot = new QTreeWidgetItem(m_tree);
        throughRoot->setText(0, QStringLiteral("通结构"));
        throughRoot->setText(
            1, QString::number(
                   static_cast<int>(
                       throughStructureFaceIds.size())));
        throughRoot->setData(
            0, kRoleKind, QStringLiteral("group"));
        std::vector<std::string> sortedFaceIds(
            throughStructureFaceIds.begin(),
            throughStructureFaceIds.end());
        std::sort(
            sortedFaceIds.begin(), sortedFaceIds.end());
        for (const std::string& faceId : sortedFaceIds) {
            auto* item = new QTreeWidgetItem(throughRoot);
            item->setText(
                0, QString::fromStdString(faceId));
            item->setData(
                0, kRoleId,
                QString::fromStdString(faceId));
            item->setData(
                0, kRoleKind, QStringLiteral("face"));
        }
        throughRoot->setExpanded(true);
    }

    if (!wireTraceSubsets.empty()) {
        auto* wireRoot = new QTreeWidgetItem(m_tree);
        wireRoot->setText(
            0, QStringLiteral("内外环结构"));
        wireRoot->setText(
            1, QString::number(
                   static_cast<int>(
                       wireTraceSubsets.size())));
        wireRoot->setData(
            0, kRoleKind, QStringLiteral("group"));
        appendWireTraceFeatureSubsets(
            wireRoot, wireTraceSubsets);
        wireRoot->setExpanded(true);
    }

    m_tree->setUpdatesEnabled(true);
    m_tree->expandToDepth(1);
}

void FeaturePanel::selectFaceById(const QString& faceId)
{
    selectFacesByIds(
        {faceId.toStdString()});
}

void FeaturePanel::selectFacesByIds(
    const std::vector<std::string>& faceIds)
{
    // 3D 拾取某面 -> 在本树中选中对应的面叶子(可能跨多个实例)。屏蔽信号、
    // 不回发 3D(由拾取来源负责高亮),避免选择来源相互触发成环。
    const std::unordered_set<std::string> wanted(
        faceIds.begin(), faceIds.end());
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
    QTreeWidgetItem* firstHit = nullptr;
    QTreeWidgetItemIterator it(m_tree);
    for (; *it; ++it) {
        if ((*it)->data(0, kRoleKind).toString() != "face") continue;
        if (!wanted.contains(
                (*it)->data(0, kRoleId)
                    .toString().toStdString()))
            continue;
        (*it)->setSelected(true);
        for (QTreeWidgetItem* parent = (*it)->parent();
             parent; parent = parent->parent()) {
            parent->setExpanded(true);
        }
        if (!firstHit) firstHit = *it;
    }
    if (firstHit) {
        m_tree->setCurrentItem(firstHit);
        m_tree->scrollToItem(firstHit);
    }
}

void FeaturePanel::clearSelection()
{
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
}

std::vector<std::string> FeaturePanel::selectedFaceIds() const
{
    std::vector<std::string> out;
    std::set<std::string> seen;
    auto add = [&](const std::string& id) {
        if (!id.empty() && seen.insert(id).second) out.push_back(id);
    };
    // 递归收集某节点下所有面叶子的 face_id。
    std::function<void(QTreeWidgetItem*)> collect = [&](QTreeWidgetItem* it) {
        if (it->data(0, kRoleKind).toString() == "face") {
            add(it->data(0, kRoleId).toString().toStdString());
            return;
        }
        for (int i = 0; i < it->childCount(); ++i) collect(it->child(i));
    };
    for (QTreeWidgetItem* it : m_tree->selectedItems()) collect(it);
    return out;
}

void FeaturePanel::onItemSelectionChanged()
{
    // 任意节点选择变化都通知 MainWindow:把本面板设为选择来源,在 3D 高亮当前面集合。
    emit featureSelectionChanged();
}
