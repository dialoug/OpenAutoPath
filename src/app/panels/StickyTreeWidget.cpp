// StickyTreeWidget 实现 — 参见 StickyTreeWidget.h。
#include "StickyTreeWidget.h"

#include <QPainter>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QStyle>
#include <QHeaderView>
#include <QScrollBar>

namespace {
// 行高回退时附加的额外边距（像素）。
constexpr int kRowHeightPadding = 6;
// 命中测试的 x 偏移（像素，避开左边缘）。
constexpr int kHitTestX = 2;
// 悬浮行文本的左右内边距（像素）。
constexpr int kTextPadding = 4;
} // namespace

// 视口顶部的轻量悬浮绘制层:不拦截滚轮/不参与布局,仅绘制祖先链并把点击回传。
class StickyOverlay : public QWidget
{
public:
    explicit StickyOverlay(StickyTreeWidget* tree)
        : QWidget(tree->viewport()), m_tree(tree)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents, false);
        setVisible(false);
    }

protected:
    void paintEvent(QPaintEvent*) override { m_tree->paintStickyOverlay(this); }
    void mousePressEvent(QMouseEvent* e) override { m_tree->stickyClicked(e->pos()); }

private:
    StickyTreeWidget* m_tree;
};

StickyTreeWidget::StickyTreeWidget(QWidget* parent) : QTreeWidget(parent)
{
    m_overlay = new StickyOverlay(this);
    // 展开/折叠/数据变化都会改变可见项,需重算悬浮层。
    connect(this, &QTreeWidget::itemExpanded,  this, [this](QTreeWidgetItem*){ updateSticky(); });
    connect(this, &QTreeWidget::itemCollapsed, this, [this](QTreeWidgetItem*){ updateSticky(); });
    connect(verticalScrollBar(), &QScrollBar::valueChanged, this, [this](int){ updateSticky(); });
}

void StickyTreeWidget::scrollContentsBy(int dx, int dy)
{
    QTreeWidget::scrollContentsBy(dx, dy);
    updateSticky();
}

void StickyTreeWidget::resizeEvent(QResizeEvent* e)
{
    QTreeWidget::resizeEvent(e);
    updateSticky();
}

int StickyTreeWidget::rowPixelHeight() const
{
    // 取顶部可见项的行高;退化时回退到字体行高 + 少量边距。
    const QModelIndex top = indexAt(QPoint(0, 0));
    if (top.isValid()) {
        const int h = rowHeight(top);
        if (h > 0) return h;
    }
    return fontMetrics().height() + kRowHeightPadding;
}

void StickyTreeWidget::updateSticky()
{
    // 顶部可见项的祖先链(不含项自身)即为应悬浮的父节点序列。
    m_chain.clear();
    const int rowH = rowPixelHeight();
    // 取略低于第一行处的项,避免正好命中将要被悬浮覆盖的那一行。
    QModelIndex idx = indexAt(QPoint(kHitTestX, rowH / 2));
    if (idx.isValid()) {
        QModelIndex p = idx.parent();
        while (p.isValid()) {
            m_chain.prepend(p);   // 顶层在前
            p = p.parent();
        }
    }
    if (m_chain.isEmpty()) {
        m_overlay->setVisible(false);
        return;
    }
    const int w = viewport()->width();
    m_overlay->setGeometry(0, 0, w, rowH * m_chain.size());
    m_overlay->raise();
    m_overlay->setVisible(true);
    m_overlay->update();
}

void StickyTreeWidget::paintStickyOverlay(QWidget* overlay)
{
    QPainter painter(overlay);
    const int rowH = rowPixelHeight();
    const QColor bg = palette().color(QPalette::Base);
    const QColor fg = palette().color(QPalette::Text);
    const QColor line = palette().color(QPalette::Mid);

    for (int i = 0; i < m_chain.size(); ++i) {
        const QModelIndex idx = m_chain[i];
        const QRect rowRect(0, i * rowH, overlay->width(), rowH);
        // 不透明背景遮住其下滚动的内容,显出"悬浮固定"效果。
        painter.fillRect(rowRect, bg);

        // 按层级缩进(与树的缩进步距一致),再画展开箭头占位 + 文本。
        const int depth = i; // 链中第 i 项即第 i 层
        const int baseIndent = indentation() * (depth + 1);
        const QString text = model()->data(idx, Qt::DisplayRole).toString();
        QRect textRect = rowRect.adjusted(baseIndent + kTextPadding, 0, -kTextPadding, 0);
        painter.setPen(fg);
        painter.drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft,
                         fontMetrics().elidedText(text, Qt::ElideRight, textRect.width()));
    }
    // 底部分隔线,从视觉上把悬浮区与正常滚动区分开。
    painter.setPen(line);
    painter.drawLine(0, m_chain.size() * rowH - 1,
                     overlay->width(), m_chain.size() * rowH - 1);
}

void StickyTreeWidget::stickyClicked(const QPoint& pos)
{
    // 点击悬浮区第 row 行 -> 跳转到对应祖先:展开、选中、滚动到可见。
    const int rowH = rowPixelHeight();
    const int row = pos.y() / rowH;
    if (row < 0 || row >= m_chain.size()) return;
    const QModelIndex idx = m_chain[row];
    if (!idx.isValid()) return;
    scrollTo(idx, QAbstractItemView::PositionAtTop);
    setCurrentIndex(idx);
    if (QTreeWidgetItem* it = itemFromIndex(idx)) {
        clearSelection();
        it->setSelected(true);
    }
    updateSticky();
}
