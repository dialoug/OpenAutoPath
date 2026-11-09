// ResultPanel — 一个停靠（dock）控件，用于展示加载后的 Model：units->faces 树，外加
// 所选 face/unit 的属性视图。发出选择信号以实现 3D <-> 树的同步。
#pragma once

#include <QWidget>
#include <QString>

#include <array>
#include <vector>
#include <string>
#include <unordered_map>

#include "analysis/Model.h"
#include "analysis/StepIndexer.h"

class QTreeWidget;
class QTreeWidgetItem;
class QTextBrowser;
class QLineEdit;

// 结果面板：展示面/单元属性与搜索结果。
class ResultPanel : public QWidget
{
    Q_OBJECT
public:
    explicit ResultPanel(QWidget* parent = nullptr);

    // 用加载完成的模型填充树。两个指针的生命周期都必须长于本面板的
    // 使用期（MainWindow 持有它们，并在替换前先清空面板）。
    void setModel(const oap::Model* model, const oap::StepIndexer* indexer);
    void clear();

    // 以编程方式选中某个面（当用户在 3D 视图中拾取某个面时调用）。
    void selectFaceById(const QString& faceId);
    void selectFacesByIds(const std::vector<std::string>& faceIds);
    void clearSelection();

    // 多选:切换某面的选中态而不影响其它已选面(3D 中按住 Ctrl 拾取时调用)。
    // 该面已选则取消、未选则追加。与 selectFaceById 一样不回发 3D 信号。
    void toggleFaceById(const QString& faceId);

    // 以编程方式选中某个 unit 节点(unit 选择模式下 3D 拾取面后定位其所属 unit)。
    void selectUnitById(const QString& unitId);
    void selectUnitsByIds(const std::vector<std::string>& unitIds);

    // 在属性面板渲染边/线的几何属性(Edge/Wire 模式下 3D 拾取后调用)。
    // 这两者不是树节点,故仅更新属性视图;调用方负责在树中定位其所属面以保持选择同步。
    void showEdgeProperties(const QString& edgeId);
    void showWireProperties(const QString& wireId);

    // 当前树选择展开后的 face_id 集合:unit 节点展开为其全部面,face 节点取自身。
    // 供 MainWindow 的显示隐藏命令统一读取选择目标。
    std::vector<std::string> selectedFaceIds() const;

signals:
    void faceSelected(const QString& faceId);
    void unitSelected(const QString& unitId);
    void contextMenuRequested(const QPoint& globalPos); // 树右键菜单

private slots:
    void onItemSelectionChanged();
    // 搜索框回车/文本提交:按输入的序号定位 Unit 或 Face 节点并选中(联动 3D)。
    // 支持 "F123"/"123"(面)、"U12"/"unit 12"(单元)等;大小写、前缀 U/F 可省略。
    // 同一关键词重复提交时在多个匹配间循环跳转。
    void onSearchSubmitted();
private:
    void showFaceProperties(const QString& faceId);
    void showUnitProperties(const QString& unitId);
    const oap::Model*       m_model   = nullptr;
    const oap::StepIndexer* m_stepIndexer = nullptr;
    QLineEdit*             m_search  = nullptr;
    QTreeWidget*           m_tree    = nullptr;
    QTextBrowser*          m_props   = nullptr;
    // 树节点的 id->item 查找映射,在 setModel 填充树时一次建立。供 3D 拾取
    // 时按 id 定位节点,免去每次回调对整棵树的 QTreeWidgetItemIterator 线性扫描。
    // 随 setModel 重建、clear 清空;指向 m_tree 拥有的节点,生命周期随树。
    std::unordered_map<std::string, QTreeWidgetItem*> m_faceItems;
    std::unordered_map<std::string, QTreeWidgetItem*> m_unitItems;

    // 表面属性：面板展示用的面属性快照。
    struct SurfaceProperties {
        double area = -1.0;
        bool hasArea = false;
        std::array<double, 3> areaCentroid{};
        bool hasAreaCentroid = false;
    };
    // 面积与面积质心在首次显示该面属性时通过同一次 SurfaceProperties 调用计算，
    // 不写回只读 m_model。失败结果也缓存，避免重复触发昂贵或异常的几何计算。
    std::unordered_map<std::string, SurfaceProperties>
        m_surfacePropertiesCache;
};
