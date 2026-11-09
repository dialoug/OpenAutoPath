// StickyTreeWidget — 带"粘性父节点"显示的 QTreeWidget。
// 浏览较长的子节点列表时,当前顶部可见项的祖先链会悬浮固定在视图顶部
// (类似 VSCode 的 sticky scroll / 函数名悬浮),便于定位叶子节点的父节点。
// 悬浮行可点击,点击后跳转(滚动)到对应祖先并选中它。
#pragma once

#include <QTreeWidget>
#include <QModelIndex>
#include <QVector>

class QWidget;
class QPoint;

// 粘性树控件：跨刷新保持选择与展开状态。
class StickyTreeWidget : public QTreeWidget
{
    Q_OBJECT
public:
    explicit StickyTreeWidget(QWidget* parent = nullptr);

protected:
    // 滚动 / 尺寸变化时重算悬浮祖先链。
    void scrollContentsBy(int dx, int dy) override;
    void resizeEvent(QResizeEvent* e) override;

private slots:
    void updateSticky();   // 重算顶部可见项的祖先链并刷新悬浮层

private:
    friend class StickyOverlay;
    // 由悬浮层回调:把当前祖先链绘制为顶部固定行。
    void paintStickyOverlay(QWidget* overlay);
    // 由悬浮层回调:点击悬浮区域第 row 行 -> 跳转到该祖先并选中。
    void stickyClicked(const QPoint& pos);

    int  rowPixelHeight() const;   // 单行像素高(取顶部可见项,退化时回退字体高)

    QWidget*               m_overlay = nullptr;  // 视口顶部的悬浮绘制层
    QVector<QModelIndex>   m_chain;              // 顶->底 的祖先链(顶层在前)
};
