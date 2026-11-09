// FeaturePanel — 统一结构识别结果的停靠面板。孔、通结构和三向内外环特征
// 分组展示，每个结构节点下挂 face 叶子。选择分组会展开为全部后代面，
// 选择叶子只取该面；两者都通过 selectedFaceIds() 暴露给 Hide/Only 等命令。
#pragma once

#include <QWidget>

#include <cstddef>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "recognition/FeatureRecognizer.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"

class QTreeWidget;
class QTreeWidgetItem;

using WireTraceFeatureSubset = oap::ThreeAxisFeatureSubset;

// 特征面板：展示识别出的孔/槽/腔等特征。
class FeaturePanel : public QWidget
{
    Q_OBJECT
public:
    explicit FeaturePanel(QWidget* parent = nullptr);

    // 用识别结果重建树。空结果则清空。
    void setResult(
        const oap::RecognitionResult& res,
        const std::unordered_map<std::string, std::string>& holeStrategyBadges = {});
    // 按“轴向 + 代表来源根平面”组织追踪结果；盲槽保持原分支，盲腔在
    // 最终面归属后按共享拓扑边合并，树选择映射到合并后的全部走过面。
    void setWireTraceFeatureSubsets(
        const std::vector<WireTraceFeatureSubset>& subsets);
    // 在同一棵树中展示孔、通结构和三向内外环追踪结果。
    void setInnerOuterStructureResult(
        const oap::RecognitionResult& res,
        const std::unordered_map<std::string, std::string>&
            holeStrategyBadges,
        const std::unordered_set<std::string>& throughStructureFaceIds,
        const std::vector<WireTraceFeatureSubset>& wireTraceSubsets);
    void clear();

    // 以编程方式选中某 face_id 的面叶子(3D 拾取该面时回显到本树)。该面可能出现在
    // 多个实例下(同一面被多个特征共享时),全部一并选中并滚动到首个;屏蔽信号、
    // 不回发 3D,避免与拾取来源相互触发。若该面不属于任何已识别特征则清空本树选择。
    void selectFaceById(const QString& faceId);
    // Shift 整特征拾取时同步选中一组面；一个面出现在多个结构下时全部命中。
    void selectFacesByIds(
        const std::vector<std::string>& faceIds);
    void clearSelection();

    // 当前选择展开后的 face_id 集合:类别/分组节点展开为其全部后代面,面叶子取自身。去重。
    std::vector<std::string> selectedFaceIds() const;

    // 树当前是否为空(无任何可展示结果)。
    bool isEmpty() const;

signals:
    // 树选择变化(任意节点)。MainWindow 据此把本面板设为选择来源并在 3D 高亮。
    void featureSelectionChanged();
    void contextMenuRequested(const QPoint& globalPos); // 树右键菜单(复用 Hide/Only)

private slots:
    void onItemSelectionChanged();

private:
    void appendRecognitionResult(
        QTreeWidgetItem* parent,
        const oap::RecognitionResult& res,
        const std::unordered_map<std::string, std::string>&
            holeStrategyBadges);
    void appendWireTraceFeatureSubsets(
        QTreeWidgetItem* parent,
        const std::vector<WireTraceFeatureSubset>& subsets);

    QTreeWidget* m_tree = nullptr;
};
