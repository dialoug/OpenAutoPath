// GroupPanel 实现 — 参见 GroupPanel.h。
#include "GroupPanel.h"
#include <QCoreApplication>

#include "StickyTreeWidget.h"
#include "app/workflows/shared/UnitDisplayGrouping.h"

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
// 在每个树节点上存放 id 与 kind,使选择能映射回模型。
constexpr int kRoleId   = Qt::UserRole + 1; // unit 节点存 unit_id;face 叶子存 face_id
constexpr int kRoleKind = Qt::UserRole + 2; // 节点类型："group" / "unit" / "face"

// "Count" 列宽(像素)。
constexpr int kCountColumnWidth = 60;
// "Type" 列宽(像素)。
constexpr int kTypeColumnWidth = 90;

} // namespace

GroupPanel::GroupPanel(QWidget* parent) : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_tree = new StickyTreeWidget(this);
    m_tree->setHeaderLabels({QCoreApplication::translate("OpenAutoPath", "分组 / 单元 / 面"), QCoreApplication::translate("OpenAutoPath", "数量"), QCoreApplication::translate("OpenAutoPath", "类型")});
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_tree->setColumnWidth(1, kCountColumnWidth);
    m_tree->setColumnWidth(2, kTypeColumnWidth);
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection); // Ctrl/Shift 多选
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &GroupPanel::onItemSelectionChanged);
    connect(m_tree, &QWidget::customContextMenuRequested, this,
            [this](const QPoint& pos) {
                emit contextMenuRequested(m_tree->viewport()->mapToGlobal(pos));
            });
    layout->addWidget(m_tree);
}

void GroupPanel::clear()
{
    m_model = nullptr;
    m_tree->clear();
}

bool GroupPanel::isEmpty() const
{
    return m_tree->topLevelItemCount() == 0;
}

void GroupPanel::setModel(const oap::Model* model)
{
    m_tree->clear();
    m_model = model;
    if (!model) return;

    // 按组键归集 unit 指针(保持模型顺序,稍后组内再按 unit 号排序)。
    std::map<std::string, std::vector<const oap::Unit*>> byGroup;
    for (const auto& u : model->units)
        byGroup[
            oap::app::unitDisplayGroupKey(u)]
            .push_back(&u);

    // 组排序:细分组(数字/未命名/其他)固定殿后;同档内按"成员数降序、键升序"。
    std::vector<const std::string*> keys;
    keys.reserve(byGroup.size());
    for (const auto& kv : byGroup) keys.push_back(&kv.first);
    std::sort(keys.begin(), keys.end(),
              [&](const std::string* a, const std::string* b) {
                  const int ra =
                      oap::app::unitDisplayGroupRank(*a);
                  const int rb =
                      oap::app::unitDisplayGroupRank(*b);
                  if (ra != rb) return ra < rb;
                  const std::size_t na = byGroup[*a].size(), nb = byGroup[*b].size();
                  if (na != nb) return na > nb;       // 成员多的在前
                  return *a < *b;                      // 键升序
              });

    // 面 id -> surface_type 的快速查找(避免每个面线性扫描 model->faces)。
    std::map<std::string, std::string> faceType;
    for (const auto& f : model->faces) faceType[f.face_id] = f.surface_type;

    m_tree->setUpdatesEnabled(false);
    for (const std::string* key : keys) {
        auto units = byGroup[*key];
        std::sort(units.begin(), units.end(),
                  [](const oap::Unit* a, const oap::Unit* b) {
                      return oap::app::stableIdLess(
                          a->unit_id, b->unit_id);
                  });

        auto* gItem = new QTreeWidgetItem(m_tree);
        gItem->setText(0, QCoreApplication::translate("OpenAutoPath", key->c_str()));
        gItem->setText(1, QString::number((int)units.size())); // 组内 unit 数
        gItem->setData(0, kRoleKind, QStringLiteral("group"));

        for (const oap::Unit* u : units) {
            auto* uItem = new QTreeWidgetItem(gItem);
            const QString uname = u->decoded_name.empty()
                ? QString::fromStdString(u->unit_id)
                : QString::fromStdString(u->unit_id + "  " + u->decoded_name);
            uItem->setText(0, uname);
            uItem->setText(1, QString::number((int)u->face_ids.size())); // 面数
            uItem->setText(2, QCoreApplication::translate("OpenAutoPathLabel", (u->unit_geometry_class).c_str()));
            uItem->setData(0, kRoleId, QString::fromStdString(u->unit_id));
            uItem->setData(0, kRoleKind, QStringLiteral("unit"));

            for (const auto& fid : u->face_ids) {
                auto* fItem = new QTreeWidgetItem(uItem);
                fItem->setText(0, QString::fromStdString(fid));
                auto it = faceType.find(fid);
                fItem->setText(2, it != faceType.end()
                                      ? QCoreApplication::translate("OpenAutoPathLabel", (it->second).c_str()) : QString());
                fItem->setData(0, kRoleId, QString::fromStdString(fid));
                fItem->setData(0, kRoleKind, QStringLiteral("face"));
            }
        }
    }
    m_tree->setUpdatesEnabled(true);
    m_tree->expandToDepth(0); // 默认展开到组层
}

void GroupPanel::selectUnitById(const QString& unitId)
{
    // 3D/其它面板拾取某 unit -> 在本树中选中对应 unit 节点。屏蔽信号、不回发 3D。
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
    QTreeWidgetItemIterator it(m_tree);
    for (; *it; ++it) {
        if ((*it)->data(0, kRoleKind).toString() != "unit") continue;
        if ((*it)->data(0, kRoleId).toString() != unitId) continue;
        (*it)->setSelected(true);
        if ((*it)->parent()) (*it)->parent()->setExpanded(true);
        m_tree->setCurrentItem(*it);
        m_tree->scrollToItem(*it);
        break;
    }
}

// 批量选中多个 unit 节点(多选场景)。屏蔽信号、不回发 3D;
// 滚动定位到首个命中项。
void GroupPanel::selectUnitsByIds(const std::vector<std::string>& unitIds)
{
    const std::set<std::string> wanted(unitIds.begin(), unitIds.end());
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
    QTreeWidgetItem* first = nullptr;
    for (QTreeWidgetItemIterator it(m_tree); *it; ++it) {
        if ((*it)->data(0, kRoleKind).toString() != QStringLiteral("unit")) continue;
        if (!wanted.contains((*it)->data(0, kRoleId).toString().toStdString())) continue;
        (*it)->setSelected(true);
        if (!first) first = *it;
    }
    if (first) {
        if (first->parent()) first->parent()->setExpanded(true);
        m_tree->setCurrentItem(first, 0, QItemSelectionModel::NoUpdate);
        m_tree->scrollToItem(first);
    }
}

void GroupPanel::clearSelection()
{
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
}

std::vector<std::string> GroupPanel::selectedFaceIds() const
{
    std::vector<std::string> out;
    if (!m_model) return out;
    std::set<std::string> seen;
    auto add = [&](const std::string& id) {
        if (!id.empty() && seen.insert(id).second) out.push_back(id);
    };
    // unit 节点 -> 从模型展开其 face_ids。
    auto addUnit = [&](const std::string& uid) {
        for (const auto& u : m_model->units)
            if (u.unit_id == uid) {
                for (const auto& fid : u.face_ids) add(fid);
                break;
            }
    };
    // 递归收集:group -> 其全部 unit 的面;unit -> 自身面;face -> 自身。
    std::function<void(QTreeWidgetItem*)> collect = [&](QTreeWidgetItem* it) {
        const QString kind = it->data(0, kRoleKind).toString();
        if (kind == "face") {
            add(it->data(0, kRoleId).toString().toStdString());
            return;
        }
        if (kind == "unit") {
            addUnit(it->data(0, kRoleId).toString().toStdString());
            return;
        }
        for (int i = 0; i < it->childCount(); ++i) collect(it->child(i));
    };
    for (QTreeWidgetItem* it : m_tree->selectedItems()) collect(it);
    return out;
}

void GroupPanel::onItemSelectionChanged()
{
    // 任意节点选择变化都通知 MainWindow:把本面板设为选择来源,在 3D 高亮当前面集合。
    emit groupSelectionChanged();
}
