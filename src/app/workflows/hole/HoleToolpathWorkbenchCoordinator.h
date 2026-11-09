// 孔刀路工作台的值模型和事务边界；不访问 QWidget、AIS 或 OCCT Context。
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <QString>
#include <QStringList>
#include <QVector>

#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "app/workflows/playback/PlaybackSession.h"

namespace oap::app {

// 刀具批次的特征族：用于区分孔程序批次与槽/腔批次。
enum class ToolpathBatchFamily {
    Hole,
    Pocket,
    Profile,
    SurfaceFinishing,
};

enum class ToolpathOperationIssueSeverity {
    Info,
    Warning,
    Error,
};

// 正式工序属性中的结构化问题。code 稳定供测试/后续图标使用，message
// 只负责面向操作者解释，不允许 UI 反向解析文本决定业务状态。
struct ToolpathOperationIssueView {
    QString code;
    QString message;
    ToolpathOperationIssueSeverity severity =
        ToolpathOperationIssueSeverity::Info;
};

// 刀具批次内的可执行工序。operationId 是运行期展开 ID；跨视图选择、
// 持久化和重建必须使用 formalOperationId。
struct ToolpathOperationView {
    QString operationId;
    QString formalOperationId;
    QString label;
    QString detailText;
    QString stageKey;
    QString setupLabel;
    QString toolLabel;
    QString phaseLabel;
    ToolpathBatchFamily family = ToolpathBatchFamily::Hole;
    int sourceIndex = -1;
    int sequence = -1;
    manufacturing::JobOperationState state =
        manufacturing::JobOperationState::Planned;
    bool enabled = true;
    bool hasPlayback = true;
    int targetCount = 0;
    int targetFaceCount = 0;
    int protectedFaceCount = 0;
    int boundaryFaceCount = 0;
    QString targetSummary;
    QString boundaryMode;
    QString predecessorOperationId;
    QString parentIpwStateKey;
    QString ipwStateKey;
    QString ipwStatus;
    bool parentCheckpointAvailable = false;
    bool operationCheckpointAvailable = false;
    QVector<ToolpathOperationIssueView> issues;
    int startMoveIndex = 0;
    int endMoveIndex = 0;
};

// 刀具批次的树节点展示值：对应回放移动段区间。
struct HoleToolBatchView {
    int batchIndex = -1;
    int startMoveIndex = 0;
    int endMoveIndex = 0;
    QString setupLabel;
    QString toolLabel;
    QString detailText;
    ToolpathBatchFamily family = ToolpathBatchFamily::Hole;
    int setupOrder = 6;
    int sequence = -1;
    QVector<ToolpathOperationView> operations;
};

// 工具树完整模型：刀具批次列表（正式工序）。
struct HoleToolTreeModel {
    QVector<HoleToolBatchView> batches;
};

// 孔覆盖审计与正式 Job 工序之间的稳定选择投影。即使工序禁用、失效或
// 没有回放移动，绑定仍保留，供审计树明确展示其状态。
struct HoleCoverageOperationBinding {
    QString formalOperationId;
    QVector<int> sourceHoleIndices;
    int sequence = -1;
    manufacturing::JobOperationState state =
        manufacturing::JobOperationState::Planned;
    bool enabled = true;
    bool hasPlayback = false;
};

// 孔刀路工作台协调器（静态门面）：负责工作台值模型（工具树）的构建、
// 正式工序树与覆盖审计绑定；
// 不访问 QWidget、AIS 或 OCCT Context。
class HoleToolpathWorkbenchCoordinator {
public:
    // 从工作台识别源导出孔输入向量。
    static std::vector<toolpath::HoleMachiningInput> sourceInputs(
        const MachiningWorkspaceSession& workspace);

    // 由选中面 ID 定位孔段索引；命中多个不同孔段时返回 -1。
    static int selectedStageIndex(
        const MachiningWorkspaceSession& workspace,
        const std::vector<std::string>& selectedFaceIds);

    // 判断预览条目是否属于某刀具批次的移动段区间。
    static bool previewEntryBelongsToToolBatch(
        const HolePreviewEntry& entry,
        const std::vector<PlaybackMove>& moves,
        const std::vector<int>& toolBatchStarts,
        int toolBatchIndex);

    // 构建工具树模型（正式批次，按装夹面与序号排序）。
    static HoleToolTreeModel buildTreeModel(
        const MachiningWorkspaceSession& workspace,
        const std::vector<PlaybackMove>& moves,
        const std::vector<int>& toolBatchStarts);

    // 构建正式孔工序到识别孔段的稳定绑定；运行期后缀只用于判断是否有
    // 可播放结果，不会成为选择键。
    static QVector<HoleCoverageOperationBinding>
    buildCoverageOperationBindings(
        const MachiningWorkspaceSession& workspace,
        const std::vector<PlaybackMove>& moves);

};

} // namespace oap::app
