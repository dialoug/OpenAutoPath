// ResultPanel 实现 — 参见 ResultPanel.h。
#include "ResultPanel.h"
#include <QCoreApplication>

#include "StickyTreeWidget.h"

#include <QVBoxLayout>
#include <QSplitter>
#include <QLineEdit>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QItemSelectionModel>
#include <QTextBrowser>
#include <QHeaderView>

#include <set>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
namespace {
// 在每个树节点上用角色（role）存放记录的 id 与 kind，使选择能映射回模型。
constexpr int kRoleId   = Qt::UserRole + 1;
constexpr int kRoleKind = Qt::UserRole + 2; // "unit" 或 "face"

// 一般属性数值显示的小数位数。
constexpr int kPropertyPrecision = 3;
// "Type" 列宽（像素）。
constexpr int kTypeColumnWidth = 90;
// 树:属性区垂直拉伸比 3:2。
constexpr int kTreeStretch = 3;
constexpr int kPropsStretch = 2;
// HTML 诊断文字配色。
constexpr const char* kColorDiagnostics = "#c60"; // 诊断信息（橙褐）

QString fmt(double v) { return QString::number(v, 'f', kPropertyPrecision); }
QString htmlText(const QString& value) { return value.toHtmlEscaped(); }
QString htmlText(const std::string& value) {
    return QString::fromStdString(value).toHtmlEscaped();
}

}

ResultPanel::ResultPanel(QWidget* parent) : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 搜索框:按 Unit / Face 序号定位节点。输入 "F123"/"123"(面)、"U12"(单元)
    // 等,回车后选中对应节点并联动 3D;同一关键词重复回车在多个匹配间循环。
    m_search = new QLineEdit(this);
    m_search->setPlaceholderText(QCoreApplication::translate("OpenAutoPath", "搜索 Unit / Face 序号 (如 F123、123、U12)，回车定位"));
    m_search->setClearButtonEnabled(true);
    connect(m_search, &QLineEdit::returnPressed, this, &ResultPanel::onSearchSubmitted);
    layout->addWidget(m_search);

    auto* splitter = new QSplitter(Qt::Vertical, this);

    m_tree = new StickyTreeWidget(splitter);
    m_tree->setHeaderLabels({QCoreApplication::translate("OpenAutoPath", "单元 / 面"), QCoreApplication::translate("OpenAutoPath", "类型"), QCoreApplication::translate("OpenAutoPath", "角色")});
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_tree->setColumnWidth(1, kTypeColumnWidth);
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection); // Ctrl/Shift 多选
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &ResultPanel::onItemSelectionChanged);
    connect(m_tree, &QWidget::customContextMenuRequested, this,
            [this](const QPoint& pos) {
                emit contextMenuRequested(m_tree->viewport()->mapToGlobal(pos));
            });

    m_props = new QTextBrowser(splitter);
    m_props->setOpenExternalLinks(false);
    splitter->addWidget(m_tree);
    splitter->addWidget(m_props);
    splitter->setStretchFactor(0, kTreeStretch);
    splitter->setStretchFactor(1, kPropsStretch);
    layout->addWidget(splitter);
}

void ResultPanel::clear()
{
    m_model   = nullptr;
    m_stepIndexer = nullptr;
    m_faceItems.clear();
    m_unitItems.clear();
    m_surfacePropertiesCache.clear();
    m_tree->clear();
    m_props->clear();
}

void ResultPanel::setModel(const oap::Model* model, const oap::StepIndexer* indexer)
{
    m_tree->clear();
    m_props->clear();
    m_faceItems.clear();
    m_unitItems.clear();
    m_surfacePropertiesCache.clear();
    m_model   = model;
    m_stepIndexer = indexer;
    if (!model) return;

    // 预建 face_id → surface_type 映射，避免在双层循环中线性扫描 model->faces（O(faces²)）。
    std::unordered_map<std::string, const std::string*> faceType;
    faceType.reserve(model->faces.size());
    for (const auto& f : model->faces)
        faceType.emplace(f.face_id, &f.surface_type);

    // 同时建立 id→item 查找映射,供后续 3D 拾取 O(1) 定位节点。
    m_unitItems.reserve(model->units.size());
    m_faceItems.reserve(model->faces.size());

    // 按单元（unit）索引各面，使每个单元节点列出自身的面。
    m_tree->setUpdatesEnabled(false);
    for (const auto& u : model->units) {
        auto* uItem = new QTreeWidgetItem(m_tree);
        const QString uname = u.decoded_name.empty()
            ? QString::fromStdString(u.unit_id)
            : QString::fromStdString(u.unit_id + "  " + u.decoded_name);
        uItem->setText(0, uname);
        uItem->setText(1, QCoreApplication::translate("OpenAutoPathLabel", (u.unit_geometry_class).c_str()));
        uItem->setText(2, QCoreApplication::translate("OpenAutoPathLabel", (u.unit_role_hint).c_str()));
        uItem->setData(0, kRoleId, QString::fromStdString(u.unit_id));
        uItem->setData(0, kRoleKind, QStringLiteral("unit"));
        m_unitItems.emplace(u.unit_id, uItem);

        for (const auto& fid : u.face_ids) {
            // 该面的曲面类型：O(1) 查映射。
            auto it = faceType.find(fid);
            auto* fItem = new QTreeWidgetItem(uItem);
            fItem->setText(0, QString::fromStdString(fid));
            fItem->setText(1, it != faceType.end() ? QCoreApplication::translate("OpenAutoPathLabel", (*it->second).c_str()) : QString());
            fItem->setData(0, kRoleId, QString::fromStdString(fid));
            fItem->setData(0, kRoleKind, QStringLiteral("face"));
            m_faceItems.emplace(fid, fItem);
        }
    }
    m_tree->setUpdatesEnabled(true);

}

// 树选择变化:刷新对应属性视图并发出 faceSelected/unitSelected 联动 3D。
void ResultPanel::onItemSelectionChanged()
{
    auto items = m_tree->selectedItems();
    if (items.isEmpty()) return;
    QTreeWidgetItem* item = items.first();
    const QString id = item->data(0, kRoleId).toString();
    const QString kind = item->data(0, kRoleKind).toString();
    if (kind == "face") {
        showFaceProperties(id);
        emit faceSelected(id);
    } else if (kind == "unit") {
        showUnitProperties(id);
        emit unitSelected(id);
    }
}

namespace {
// 搜索匹配:节点 id 与查询是否指向同一对象。规则(均忽略大小写):
//   1) 与 id 完全相等(如 "F123" 命中面 "F123");
//   2) 查询为纯数字时,与 id 去掉首字母后相等(如 "123" 命中 "F123"/"U123")。
bool searchMatches(const QString& id, const QString& query)
{
    if (id.compare(query, Qt::CaseInsensitive) == 0) return true;
    bool allDigits = !query.isEmpty();
    for (const QChar c : query) if (!c.isDigit()) { allDigits = false; break; }
    if (allDigits && !id.isEmpty() && id.at(0).isLetter())
        return id.mid(1).compare(query, Qt::CaseInsensitive) == 0;
    return false;
}
} // namespace

void ResultPanel::onSearchSubmitted()
{
    QString q = m_search->text().trimmed();
    if (q.isEmpty()) return;
    // 按显式前缀限定查找类型:f/F->仅面,u/U->仅单元;否则两者都找。
    QString wantKind;
    if (q.size() >= 2 && q.at(0).toLower() == QLatin1Char('u') && q.at(1).isDigit())
        wantKind = QStringLiteral("unit");
    else if (q.size() >= 2 && q.at(0).toLower() == QLatin1Char('f') && q.at(1).isDigit())
        wantKind = QStringLiteral("face");

    // 按树顺序收集所有匹配节点。
    std::vector<QTreeWidgetItem*> hits;
    QTreeWidgetItem* current = m_tree->currentItem();
    int currentPos = -1;
    QTreeWidgetItemIterator it(m_tree);
    for (; *it; ++it) {
        const QString kind = (*it)->data(0, kRoleKind).toString();
        if (!wantKind.isEmpty() && kind != wantKind) continue;
        if (!searchMatches((*it)->data(0, kRoleId).toString(), q)) continue;
        if (*it == current) currentPos = int(hits.size());
        hits.push_back(*it);
    }
    if (hits.empty()) return;
    // 重复提交同一关键词时循环到下一个匹配(当前项已是某匹配则前进一位)。
    QTreeWidgetItem* target = hits[(currentPos + 1) % hits.size()];
    const QString id = target->data(0, kRoleId).toString();
    if (target->data(0, kRoleKind).toString() == "unit")
        selectUnitById(id);
    else
        selectFaceById(id);
}

void ResultPanel::selectFaceById(const QString& faceId)
{
    // O(1) 取面节点并选中,不再向 3D 视图回发信号。
    auto found = m_faceItems.find(faceId.toStdString());
    if (found == m_faceItems.end()) return;
    QTreeWidgetItem* item = found->second;
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
    item->setSelected(true);      // 显式选中:行高亮 + 进入 selectedItems()(供 Hide/Only)
    m_tree->setCurrentItem(item);
    if (item->parent()) item->parent()->setExpanded(true);
    m_tree->scrollToItem(item);
    showFaceProperties(faceId);
}

void ResultPanel::selectFacesByIds(const std::vector<std::string>& faceIds)
{
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
    QTreeWidgetItem* first = nullptr;
    for (const std::string& faceId : faceIds) {
        auto found = m_faceItems.find(faceId);
        if (found == m_faceItems.end()) continue;
        found->second->setSelected(true);
        if (!first) first = found->second;
    }
    if (first) {
        m_tree->setCurrentItem(first, 0, QItemSelectionModel::NoUpdate);
        if (first->parent()) first->parent()->setExpanded(true);
        m_tree->scrollToItem(first);
    }
    if (first) showFaceProperties(first->data(0, kRoleId).toString());
}

void ResultPanel::clearSelection()
{
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
}

void ResultPanel::toggleFaceById(const QString& faceId)
{
    // 多选:翻转目标面节点的选中态,保留其它已选面。不向 3D 回发信号
    // (3D 高亮由 OCCT 原生选择驱动,此处只同步树镜像供 Hide/Only 读取)。
    auto found = m_faceItems.find(faceId.toStdString());
    if (found == m_faceItems.end()) return;
    QTreeWidgetItem* item = found->second;
    const QSignalBlocker signalBlocker(m_tree);
    const bool nowSel = !item->isSelected();
    item->setSelected(nowSel);   // 翻转:已选->取消、未选->追加
    if (nowSel) {
        // 仅移动"当前项"焦点,绝不改动选择集:不带 command 的 setCurrentItem
        // 会按当前键盘修饰键决定命令,而多选时用户正按住 Ctrl,默认命令为
        // Toggle —— 会把刚 setSelected 的这个面立即翻转回未选,导致多选退化为
        // 只剩首个面。NoUpdate 让焦点跟随而选择集保持累积。
        m_tree->setCurrentItem(item, 0, QItemSelectionModel::NoUpdate);
        if (item->parent()) item->parent()->setExpanded(true);
        m_tree->scrollToItem(item);
    }
    if (nowSel) showFaceProperties(faceId);
}

void ResultPanel::selectUnitById(const QString& unitId)
{
    auto found = m_unitItems.find(unitId.toStdString());
    if (found == m_unitItems.end()) return;
    QTreeWidgetItem* item = found->second;
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
    item->setSelected(true);      // 显式选中:行高亮 + 进入 selectedItems()
    m_tree->setCurrentItem(item);
    m_tree->scrollToItem(item);
    showUnitProperties(unitId);
}

void ResultPanel::selectUnitsByIds(const std::vector<std::string>& unitIds)
{
    const QSignalBlocker signalBlocker(m_tree);
    m_tree->clearSelection();
    QTreeWidgetItem* first = nullptr;
    for (const std::string& unitId : unitIds) {
        auto found = m_unitItems.find(unitId);
        if (found == m_unitItems.end()) continue;
        found->second->setSelected(true);
        if (!first) first = found->second;
    }
    if (first) {
        m_tree->setCurrentItem(first, 0, QItemSelectionModel::NoUpdate);
        m_tree->scrollToItem(first);
    }
}

std::vector<std::string> ResultPanel::selectedFaceIds() const
{
    // unit 节点展开为其全部面;face 节点取自身。去重后返回。
    std::vector<std::string> out;
    if (!m_model) return out;
    std::set<std::string> seen;
    auto add = [&](const std::string& id) {
        if (seen.insert(id).second) out.push_back(id);
    };
    for (QTreeWidgetItem* item : m_tree->selectedItems()) {
        const QString kind = item->data(0, kRoleKind).toString();
        const std::string id = item->data(0, kRoleId).toString().toStdString();
        if (kind == "face") {
            add(id);
        } else if (kind == "unit") {
            for (const auto& u : m_model->units)
                if (u.unit_id == id) {
                    for (const auto& fid : u.face_ids) add(fid);
                    break;
                }
        }
    }
    return out;
}

// 在属性视图渲染某面的几何属性;面积/面积质心首次显示时惰性计算并缓存。
void ResultPanel::showFaceProperties(const QString& faceId)
{
    if (!m_model) return;
    const std::string fid = faceId.toStdString();
    const oap::Face* f = nullptr;
    for (const auto& rec : m_model->faces)
        if (rec.face_id == fid) { f = &rec; break; }
    if (!f) { m_props->clear(); return; }

    QString h;
    h += QCoreApplication::translate("OpenAutoPath", "<b>面 %1</b> &nbsp; (单元 %2)<br>")
             .arg(htmlText(faceId), htmlText(f->unit_id));
    h += QCoreApplication::translate("OpenAutoPath", "曲面类型: <b>%1</b><br>").arg(QCoreApplication::translate("OpenAutoPathLabel", f->surface_type.c_str()).toHtmlEscaped());
    // 面积与面积质心在首次显示时通过同一次 SurfaceProperties 调用惰性计算，
    // 结果只缓存于面板，不回写只读 m_model。
    auto cachedProperties =
        m_surfacePropertiesCache.find(fid);
    if (cachedProperties ==
        m_surfacePropertiesCache.end()) {
        SurfaceProperties properties;
        const TopoDS_Face* topo =
            m_stepIndexer
                ? m_stepIndexer->faceForId(fid)
                : nullptr;
        if (topo) {
            try {
                GProp_GProps props;
                BRepGProp::SurfaceProperties(*topo, props);
                const double area = props.Mass();
                if (std::isfinite(area) && area >= 0.0) {
                    properties.area = area;
                    properties.hasArea = true;
                }
                if (std::isfinite(area) &&
                    std::abs(area) > 1.0e-12) {
                    const gp_Pnt centroid =
                        props.CentreOfMass();
                    if (std::isfinite(centroid.X()) &&
                        std::isfinite(centroid.Y()) &&
                        std::isfinite(centroid.Z())) {
                        properties.areaCentroid = {
                            centroid.X(), centroid.Y(),
                            centroid.Z()};
                        properties.hasAreaCentroid = true;
                    }
                }
            } catch (...) {
            }
        }
        cachedProperties =
            m_surfacePropertiesCache
                .emplace(fid, properties)
                .first;
    }
    const SurfaceProperties& surfaceProperties =
        cachedProperties->second;

    if (f->area >= 0) {
        h += QCoreApplication::translate("OpenAutoPath", "面积: %1<br>").arg(fmt(f->area));
    } else if (surfaceProperties.hasArea) {
        h += QCoreApplication::translate("OpenAutoPath", "面积: %1<br>")
                 .arg(fmt(surfaceProperties.area));
    }
    if (f->radius >= 0) h += QCoreApplication::translate("OpenAutoPath", "半径: %1<br>").arg(fmt(f->radius));
    if (f->has_center)
        h += QCoreApplication::translate("OpenAutoPath", "包围盒中心: [%1, %2, %3]<br>")
                 .arg(fmt(f->center[0]), fmt(f->center[1]), fmt(f->center[2]));
    if (surfaceProperties.hasAreaCentroid) {
        h += QCoreApplication::translate("OpenAutoPath", "面积质心: [%1, %2, %3]<br>")
                 .arg(
                     fmt(surfaceProperties.areaCentroid[0]),
                     fmt(surfaceProperties.areaCentroid[1]),
                     fmt(surfaceProperties.areaCentroid[2]));
    } else {
        h += QCoreApplication::translate("OpenAutoPath", "面积质心: 不可用<br>");
    }
    if (f->has_normal)
        h += QCoreApplication::translate("OpenAutoPath", "法向: [%1, %2, %3]<br>")
                 .arg(fmt(f->normal[0]), fmt(f->normal[1]), fmt(f->normal[2]));
    if (f->bbox.valid()) {
        auto s = f->bbox.size();
        h += QCoreApplication::translate("OpenAutoPath", "包围盒: [%1, %2, %3] .. [%4, %5, %6]<br>")
                 .arg(fmt(f->bbox.xmin), fmt(f->bbox.ymin), fmt(f->bbox.zmin),
                      fmt(f->bbox.xmax), fmt(f->bbox.ymax), fmt(f->bbox.zmax));
        h += QCoreApplication::translate("OpenAutoPath", "尺寸: [%1, %2, %3]<br>").arg(fmt(s[0]), fmt(s[1]), fmt(s[2]));
    }
    m_props->setHtml(h);
}

// 在属性视图渲染某 unit 的属性。
void ResultPanel::showUnitProperties(const QString& unitId)
{
    if (!m_model) return;
    const std::string uid = unitId.toStdString();
    const oap::Unit* u = nullptr;
    for (const auto& rec : m_model->units)
        if (rec.unit_id == uid) { u = &rec; break; }
    if (!u) { m_props->clear(); return; }

    QString h;
    h += QCoreApplication::translate("OpenAutoPath", "<b>单元 %1</b><br>").arg(htmlText(unitId));
    h += QCoreApplication::translate("OpenAutoPath", "名称: %1<br>").arg(htmlText(u->decoded_name));
    h += QCoreApplication::translate("OpenAutoPath", "前缀 / 族: %1 / %2<br>")
             .arg(htmlText(u->name_prefix), QCoreApplication::translate("OpenAutoPathLabel", u->name_family.c_str()).toHtmlEscaped());
    h += QCoreApplication::translate("OpenAutoPath", "角色提示: <b>%1</b> (%2, 置信 %3)<br>")
             .arg(QCoreApplication::translate("OpenAutoPathLabel", u->unit_role_hint.c_str()).toHtmlEscaped(),
                   QCoreApplication::translate("OpenAutoPathLabel", u->role_hint_source.c_str()).toHtmlEscaped(), fmt(u->role_hint_confidence));
    h += QCoreApplication::translate("OpenAutoPath", "几何类别: %1<br>").arg(QCoreApplication::translate("OpenAutoPathLabel", u->unit_geometry_class.c_str()).toHtmlEscaped());
    h += QCoreApplication::translate("OpenAutoPath", "面数: %1<br>").arg(int(u->face_ids.size()));
    if (!u->role_hint_diagnostics.empty()) {
        QStringList d;
        for (const auto& s : u->role_hint_diagnostics) d << QCoreApplication::translate("OpenAutoPathLabel", s.c_str()).toHtmlEscaped();
        h += QCoreApplication::translate("OpenAutoPath", "<span style='color:%1'>诊断: %2</span><br>").arg(kColorDiagnostics).arg(d.join(", "));
    }
    if (u->bbox.valid()) {
        auto s = u->bbox.size();
        h += QCoreApplication::translate("OpenAutoPath", "尺寸: [%1, %2, %3]<br>").arg(fmt(s[0]), fmt(s[1]), fmt(s[2]));
    }

    m_props->setHtml(h);
}

// 在属性视图渲染边几何属性(Edge 模式拾取时调用,不联动树选择)。
void ResultPanel::showEdgeProperties(const QString& edgeId)
{
    if (!m_model) return;
    const std::string eid = edgeId.toStdString();
    const oap::Edge* e = nullptr;
    for (const auto& rec : m_model->edges)
        if (rec.edge_id == eid) { e = &rec; break; }
    if (!e) { m_props->clear(); return; }

    QString h;
    h += QCoreApplication::translate("OpenAutoPath", "<b>边 %1</b> &nbsp; (单元 %2)<br>")
             .arg(edgeId, QString::fromStdString(e->unit_id));
    h += QCoreApplication::translate("OpenAutoPath", "曲线类型: <b>%1</b><br>").arg(QCoreApplication::translate("OpenAutoPathLabel", (e->curve_type).c_str()));
    if (e->length >= 0) h += QCoreApplication::translate("OpenAutoPath", "长度: %1<br>").arg(fmt(e->length));
    if (e->has_start)
        h += QCoreApplication::translate("OpenAutoPath", "起点: [%1, %2, %3]<br>")
                 .arg(fmt(e->start_point[0]), fmt(e->start_point[1]), fmt(e->start_point[2]));
    if (e->has_end)
        h += QCoreApplication::translate("OpenAutoPath", "终点: [%1, %2, %3]<br>")
                 .arg(fmt(e->end_point[0]), fmt(e->end_point[1]), fmt(e->end_point[2]));
    {
        QStringList fl; for (const auto& s : e->face_ids) fl << QString::fromStdString(s);
        h += QCoreApplication::translate("OpenAutoPath", "面 (%1): %2<br>").arg(fl.size()).arg(fl.join(", "));
        QStringList wl; for (const auto& s : e->wire_ids) wl << QString::fromStdString(s);
        h += QCoreApplication::translate("OpenAutoPath", "线框 (%1): %2<br>").arg(wl.size()).arg(wl.join(", "));
    }
    m_props->setHtml(h);
}

// 在属性视图渲染线框几何属性(Wire 模式拾取时调用,不联动树选择)。
void ResultPanel::showWireProperties(const QString& wireId)
{
    if (!m_model) return;
    const std::string wid = wireId.toStdString();
    const oap::Wire* w = nullptr;
    for (const auto& rec : m_model->wires)
        if (rec.wire_id == wid) { w = &rec; break; }
    if (!w) { m_props->clear(); return; }

    QString h;
    h += QCoreApplication::translate("OpenAutoPath", "<b>线框 %1</b> &nbsp; (单元 %2, 面 %3)<br>")
             .arg(wireId, QString::fromStdString(w->unit_id), QString::fromStdString(w->face_id));
    h += QCoreApplication::translate("OpenAutoPath", "角色: <b>%1</b>%2<br>")
             .arg(QCoreApplication::translate("OpenAutoPathLabel", (w->wire_role).c_str()), w->is_closed ? QCoreApplication::translate("OpenAutoPath", " (闭合)") : QString());
    {
        QStringList el; for (const auto& s : w->edge_ids) el << QString::fromStdString(s);
        h += QCoreApplication::translate("OpenAutoPath", "边 (%1): %2<br>").arg(el.size()).arg(el.join(", "));
    }
    if (w->bbox.valid()) {
        auto s = w->bbox.size();
        h += QCoreApplication::translate("OpenAutoPath", "尺寸: [%1, %2, %3]<br>").arg(fmt(s[0]), fmt(s[1]), fmt(s[2]));
    }
    m_props->setHtml(h);
}
