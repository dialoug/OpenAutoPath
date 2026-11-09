// GroupPanel — 按名字前缀对 Unit 批量分组的停靠面板。
//   组 ├ Unit ├ Face   三层树。组键取自 Unit.name_prefix:
//     - 字母/复合前缀(DIE / PH / MI_LP ...) 各成一组;
//     - name_prefix 为 BODY_GENERIC 的(纯数字名 / 空名 / Body / 体)再按 decoded_name
//       细分为 "(数字)" / "(未命名)" / "(其他)" 三组。
// 选中组节点 = 选中其下所有 unit 的全部面;选中 unit = 该 unit 的面;选中 face = 该面。
// 三者都通过 selectedFaceIds() 暴露,供 Hide/Only 等命令使用(与 FeaturePanel 同契约)。
#pragma once

#include <QWidget>
#include <QString>

#include <vector>
#include <string>

#include "analysis/Model.h"

class QTreeWidget;
class QTreeWidgetItem;

// 分组面板：按单元分组展示识别结果。
class GroupPanel : public QWidget
{
    Q_OBJECT
public:
    explicit GroupPanel(QWidget* parent = nullptr);

    // 用模型重建分组树。model 的生命周期必须长于本面板使用期(MainWindow 持有,
    // 替换前先 clear)。空模型则清空。
    void setModel(const oap::Model* model);
    void clear();

    // 以编程方式选中某 unit 节点(3D 拾取该 unit 的面后回显到本树)。屏蔽信号、
    // 不回发 3D,避免选择来源相互触发成环。展开其所在组并滚动可见。
    void selectUnitById(const QString& unitId);
    void selectUnitsByIds(const std::vector<std::string>& unitIds);
    void clearSelection();

    // 当前选择展开后的 face_id 集合:组节点展开为其全部 unit 的面,unit 节点展开为
    // 自身面,face 叶子取自身。去重后返回。
    std::vector<std::string> selectedFaceIds() const;

    // 树当前是否为空(无任何可展示结果)。
    bool isEmpty() const;

signals:
    // 树选择变化(任意节点)。MainWindow 据此把本面板设为选择来源并在 3D 高亮。
    void groupSelectionChanged();
    void contextMenuRequested(const QPoint& globalPos); // 树右键菜单(复用 Hide/Only)

private slots:
    void onItemSelectionChanged();

private:
    QTreeWidget*     m_tree  = nullptr;
    const oap::Model* m_model = nullptr;
};
