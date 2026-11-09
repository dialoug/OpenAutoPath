// HoleToolpathWorkbenchCoordinator 实现（oap_app_workflow_cam）。
// 孔刀路工作台的值模型和事务边界，见 HoleToolpathWorkbenchCoordinator.h。
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"

#include <algorithm>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "manufacturing/hole/HoleMachiningJobService.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "tooling/library/ToolLibrary.h"

namespace oap::app {
namespace {

bool isHoleKind(FeatureKind kind) noexcept
{
    return kind == FeatureKind::ThroughHole ||
        kind == FeatureKind::BlindHole ||
        kind == FeatureKind::CountersinkHole;
}

QString featureKindText(FeatureKind kind)
{
    switch (kind) {
        case FeatureKind::ThroughHole: return QStringLiteral("通孔");
        case FeatureKind::BlindHole: return QStringLiteral("盲孔");
        case FeatureKind::CountersinkHole: return QStringLiteral("沉孔");
        case FeatureKind::Slot: return QStringLiteral("槽");
        case FeatureKind::Pocket: return QStringLiteral("腔");
    }
    return QStringLiteral("特征");
}

QString phaseText(
    toolpath::HoleMachiningPhase phase,
    ToolpathBatchFamily family)
{
    if (family == ToolpathBatchFamily::Profile)
        return QStringLiteral("外轮廓加工");
    if (family == ToolpathBatchFamily::SurfaceFinishing)
        return QStringLiteral("雕刻加工");
    switch (phase) {
        case toolpath::HoleMachiningPhase::CenterSpot:
            return QStringLiteral("定位阶段");
        case toolpath::HoleMachiningPhase::Drill:
            return QStringLiteral("钻孔阶段");
        case toolpath::HoleMachiningPhase::FlatEndMill:
            return QStringLiteral("平底刀阶段");
        case toolpath::HoleMachiningPhase::Standard:
            return family == ToolpathBatchFamily::Hole
                ? QStringLiteral("单刀孔加工阶段")
                : QStringLiteral("槽/腔加工阶段");
    }
    return QStringLiteral("加工阶段");
}

ToolpathBatchFamily operationFamily(const PlaybackMove& move) noexcept
{
    switch (move.operationType) {
        case toolpath::ToolpathOperationType::HoleMachining:
            return ToolpathBatchFamily::Hole;
        case toolpath::ToolpathOperationType::PocketClearing:
            return ToolpathBatchFamily::Pocket;
        case toolpath::ToolpathOperationType::ProfileContour:
            return ToolpathBatchFamily::Profile;
        case toolpath::ToolpathOperationType::SurfaceFinishing:
            return ToolpathBatchFamily::SurfaceFinishing;
        case toolpath::ToolpathOperationType::Generic:
            return isHoleKind(move.kind)
                ? ToolpathBatchFamily::Hole
                : ToolpathBatchFamily::Pocket;
    }
    return ToolpathBatchFamily::Pocket;
}

ToolpathBatchFamily operationFamily(
    manufacturing::JobOperationKind kind) noexcept
{
    switch (kind) {
        case manufacturing::JobOperationKind::Hole:
            return ToolpathBatchFamily::Hole;
        case manufacturing::JobOperationKind::Pocket:
            return ToolpathBatchFamily::Pocket;
        case manufacturing::JobOperationKind::Profile:
            return ToolpathBatchFamily::Profile;
        case manufacturing::JobOperationKind::SurfaceFinishing:
            return ToolpathBatchFamily::SurfaceFinishing;
    }
    return ToolpathBatchFamily::Pocket;
}

QString operationStateText(manufacturing::JobOperationState state)
{
    switch (state) {
        case manufacturing::JobOperationState::Planned:
            return QStringLiteral("待生成");
        case manufacturing::JobOperationState::Dirty:
            return QStringLiteral("已修改");
        case manufacturing::JobOperationState::Generated:
            return QStringLiteral("已生成");
        case manufacturing::JobOperationState::Warning:
            return QStringLiteral("有警告");
        case manufacturing::JobOperationState::Error:
            return QStringLiteral("失败");
        case manufacturing::JobOperationState::Stale:
            return QStringLiteral("下游失效");
        case manufacturing::JobOperationState::NeedsReview:
            return QStringLiteral("需复核");
    }
    return QStringLiteral("未知");
}

QString ipwStatusText(manufacturing::ipw::IpwNodeStatus status)
{
    using Status = manufacturing::ipw::IpwNodeStatus;
    switch (status) {
        case Status::Dirty: return QStringLiteral("待重建");
        case Status::Computing: return QStringLiteral("计算中");
        case Status::Ready: return QStringLiteral("就绪");
        case Status::Failed: return QStringLiteral("失败");
        case Status::Cancelled: return QStringLiteral("已取消");
    }
    return QStringLiteral("未知");
}

void addOperationIssue(
    ToolpathOperationView& view,
    QString code,
    QString message,
    ToolpathOperationIssueSeverity severity)
{
    view.issues.push_back({
        std::move(code), std::move(message), severity});
}

template <typename Operations>
const typename Operations::value_type* operationById(
    const Operations& operations,
    const std::string& operationId)
{
    const auto found = std::find_if(
        operations.begin(), operations.end(),
        [&operationId](const auto& operation) {
            return operation.id == operationId;
        });
    return found == operations.end() ? nullptr : &*found;
}

void projectOperationTargets(
    const manufacturing::MachiningJobDefinition& definition,
    const manufacturing::JobOperationDescriptor& descriptor,
    ToolpathOperationView& view)
{
    using Kind = manufacturing::JobOperationKind;
    switch (descriptor.kind) {
        case Kind::SurfaceFinishing: {
            const auto* operation = operationById(definition.finishingOperations, descriptor.id);
            if (!operation)
                return;
            view.targetCount = static_cast<int>(operation->target.unitIds.size());
            view.targetSummary = QStringLiteral("%1 个完整实体").arg(view.targetCount);
            view.boundaryMode = QStringLiteral("固定刀轴曲面平行扫描");
            return;
        }
        case Kind::Hole: {
            const auto* operation = operationById(
                definition.holeOperations, descriptor.id);
            if (!operation)
                return;
            view.targetCount = static_cast<int>(operation->targets.size());
            for (const auto& target : operation->targets) {
                view.targetFaceCount += static_cast<int>(
                    target.hole.stageWallFaceIds.empty()
                        ? target.hole.sourceFaceIds.size()
                        : target.hole.stageWallFaceIds.size());
            }
            view.targetSummary = QStringLiteral("%1 个孔目标 · %2 个来源面")
                .arg(view.targetCount)
                .arg(view.targetFaceCount);
            view.boundaryMode = QStringLiteral("孔圆柱/入口几何");
            return;
        }
        case Kind::Pocket: {
            const auto* operation = operationById(
                definition.pocketOperations, descriptor.id);
            if (!operation)
                return;
            view.targetCount = 1;
            view.targetFaceCount = static_cast<int>(
                operation->target.sourceFaceIds.size());
            view.targetSummary = QStringLiteral("1 个槽腔区域 · %1 个来源面")
                .arg(view.targetFaceCount);
            view.boundaryMode = operation->target.sourceKind ==
                    manufacturing::PocketRegionSourceKind::RootInnerWire
                ? QStringLiteral("根面内环")
                : (operation->target.allowOpenSectionContours
                       ? QStringLiteral("特征轮廓（允许开放截面）")
                       : QStringLiteral("特征闭合轮廓"));
            return;
        }
        case Kind::Profile: {
            const auto* operation = operationById(
                definition.profileOperations, descriptor.id);
            if (!operation)
                return;
            view.targetCount = 1;
            view.targetFaceCount = static_cast<int>(
                operation->target.machiningFaceIds.size());
            view.targetSummary = QStringLiteral("1 个外轮廓 · %1 个加工面")
                .arg(view.targetFaceCount);
            view.boundaryMode = QStringLiteral("根面真实外环");
            return;
        }
    }
}

void projectOperationStateIssues(
    const MachiningWorkspaceSession& workspace,
    const manufacturing::JobOperationDescriptor& descriptor,
    bool hasPlayback,
    ToolpathOperationView& view)
{
    using Severity = ToolpathOperationIssueSeverity;
    using State = manufacturing::JobOperationState;
    if (!descriptor.enabled) {
        addOperationIssue(
            view, QStringLiteral("formal.operation_disabled"),
            QStringLiteral("正式工序已禁用，不进入运行期程序或 IPW 计划。"),
            Severity::Info);
    }
    switch (descriptor.state) {
        case State::Planned:
            addOperationIssue(
                view, QStringLiteral("formal.not_generated"),
                QStringLiteral("正式工序尚未生成。"), Severity::Info);
            break;
        case State::Dirty:
            addOperationIssue(
                view, QStringLiteral("formal.definition_dirty"),
                QStringLiteral("正式输入已修改，需要重新生成并验证。"),
                Severity::Warning);
            break;
        case State::Generated:
            break;
        case State::Warning:
            addOperationIssue(
                view, QStringLiteral("formal.generation_warning"),
                QStringLiteral("工序已生成，但保留非阻断验证警告。"),
                Severity::Warning);
            break;
        case State::Error:
            addOperationIssue(
                view, QStringLiteral("formal.generation_error"),
                QStringLiteral("工序生成或安全验证失败。"), Severity::Error);
            break;
        case State::Stale:
            addOperationIssue(
                view, QStringLiteral("formal.upstream_stale"),
                QStringLiteral("定义或上游材料状态已变化，必须从此处重建。"),
                Severity::Warning);
            break;
        case State::NeedsReview:
            addOperationIssue(
                view, QStringLiteral("formal.review_required"),
                QStringLiteral("稳定引用或装夹确认需要人工复核。"),
                Severity::Warning);
            break;
    }

    const manufacturing::SetupDefinition* setup =
        workspace.job().setupById(descriptor.setupId);
    if (!setup) {
        addOperationIssue(
            view, QStringLiteral("formal.setup_missing"),
            QStringLiteral("工序引用的 Setup 快照不存在。"),
            Severity::Error);
    } else if (!setup->valid) {
        addOperationIssue(
            view, QStringLiteral("formal.setup_invalid"),
            QStringLiteral("工序引用的 Setup 快照无效。"),
            Severity::Error);
    } else if (!setup->confirmation.confirmed) {
        addOperationIssue(
            view, QStringLiteral("formal.setup_unconfirmed"),
            QStringLiteral("工序 Setup 尚未完成人工确认。"),
            Severity::Warning);
    }

    const manufacturing::JobToolController* controller =
        workspace.job().toolControllerById(descriptor.toolControllerId);
    if (!controller) {
        addOperationIssue(
            view, QStringLiteral("formal.tool_missing"),
            QStringLiteral("工序引用的刀具控制器快照不存在。"),
            Severity::Error);
    } else if (!controller->tool.valid) {
        addOperationIssue(
            view, QStringLiteral("formal.tool_invalid"),
            QStringLiteral("工序引用的刀具装配快照无效。"),
            Severity::Error);
    }

    if (descriptor.enabled && !hasPlayback) {
        addOperationIssue(
            view, QStringLiteral("playback.result_missing"),
            QStringLiteral("当前没有可播放的已验证运行期移动。"),
            descriptor.state == State::Generated ||
                    descriptor.state == State::Warning
                ? Severity::Warning : Severity::Info);
    }
}

void projectOperationIpw(
    const MachiningWorkspaceSession& workspace,
    const manufacturing::JobOperationDescriptor& descriptor,
    ToolpathOperationView& view)
{
    using Severity = ToolpathOperationIssueSeverity;
    using Status = manufacturing::ipw::IpwNodeStatus;
    const auto& session = workspace.job().ipwSession();
    if (!descriptor.enabled) {
        view.ipwStatus = QStringLiteral("未进入计划（已禁用）");
        return;
    }
    if (!session.configured()) {
        view.ipwStatus = QStringLiteral("未配置");
        addOperationIssue(
            view, QStringLiteral("ipw.session_unconfigured"),
            QStringLiteral("统一 IPW 会话尚未配置。"), Severity::Info);
        return;
    }

    const auto& nodes = session.nodes();
    const auto found = std::find_if(
        nodes.begin(), nodes.end(),
        [&descriptor](const manufacturing::ipw::IpwNode& node) {
            return node.operation.id == descriptor.id;
        });
    if (found == nodes.end()) {
        view.ipwStatus = QStringLiteral("计划节点缺失");
        addOperationIssue(
            view, QStringLiteral("ipw.node_missing"),
            QStringLiteral("启用的正式工序没有对应 IPW 计划节点。"),
            Severity::Warning);
        return;
    }

    const std::size_t nodeIndex = static_cast<std::size_t>(
        std::distance(nodes.begin(), found));
    view.parentIpwStateKey = QString::fromStdString(found->parentStateKey);
    view.ipwStateKey = QString::fromStdString(found->stateKey);
    view.ipwStatus = ipwStatusText(found->status);
    if (nodeIndex == 0) {
        view.predecessorOperationId = QStringLiteral("初始毛坯");
    } else {
        const auto& predecessor = nodes[nodeIndex - 1u];
        if (predecessor.stateKey == found->parentStateKey) {
            view.predecessorOperationId = QString::fromStdString(
                predecessor.operation.id);
        } else {
            view.predecessorOperationId = QStringLiteral("父链不一致");
            addOperationIssue(
                view, QStringLiteral("ipw.parent_chain_mismatch"),
                QStringLiteral("IPW 节点父状态键未指向前一道计划工序。"),
                Severity::Error);
        }
    }
    view.parentCheckpointAvailable =
        workspace.job().runtime().ipwCheckpoints.contains(
            found->parentStateKey);
    view.operationCheckpointAvailable =
        workspace.job().runtime().ipwCheckpoints.contains(found->stateKey);

    switch (found->status) {
        case Status::Dirty:
            addOperationIssue(
                view, QStringLiteral("ipw.node_dirty"),
                QStringLiteral("IPW 节点等待从父检查点重建。"),
                Severity::Warning);
            break;
        case Status::Computing:
            addOperationIssue(
                view, QStringLiteral("ipw.node_computing"),
                QStringLiteral("IPW 节点正在计算，尚不可回放。"),
                Severity::Info);
            break;
        case Status::Ready:
            break;
        case Status::Failed:
            addOperationIssue(
                view, QStringLiteral("ipw.node_failed"),
                found->message.empty()
                    ? QStringLiteral("IPW 节点执行失败。")
                    : QString::fromStdString(found->message),
                Severity::Error);
            break;
        case Status::Cancelled:
            addOperationIssue(
                view, QStringLiteral("ipw.node_cancelled"),
                found->message.empty()
                    ? QStringLiteral("IPW 节点执行已取消。")
                    : QString::fromStdString(found->message),
                Severity::Warning);
            break;
    }
    if (!view.parentCheckpointAvailable) {
        addOperationIssue(
            view, QStringLiteral("ipw.parent_checkpoint_missing"),
            QStringLiteral("父 IPW 检查点不在当前运行期缓存中。"),
            found->status == Status::Ready
                ? Severity::Error : Severity::Warning);
    }
    if (found->status == Status::Ready &&
        !view.operationCheckpointAvailable) {
        addOperationIssue(
            view, QStringLiteral("ipw.checkpoint_missing"),
            QStringLiteral("就绪节点缺少自身不可变检查点。"),
            Severity::Error);
    }
}

QString formalInspectorDetails(const ToolpathOperationView& view)
{
    const QString predecessor = view.predecessorOperationId.isEmpty()
        ? QStringLiteral("无对应节点") : view.predecessorOperationId;
    const QString parentKey = view.parentIpwStateKey.isEmpty()
        ? QStringLiteral("—") : view.parentIpwStateKey;
    const QString stateKey = view.ipwStateKey.isEmpty()
        ? QStringLiteral("—") : view.ipwStateKey;
    const QString parentCheckpoint = view.parentIpwStateKey.isEmpty()
        ? QStringLiteral("—")
        : (view.parentCheckpointAvailable
               ? QStringLiteral("存在") : QStringLiteral("缺失"));
    const QString operationCheckpoint = view.ipwStateKey.isEmpty()
        ? QStringLiteral("—")
        : (view.operationCheckpointAvailable
               ? QStringLiteral("存在") : QStringLiteral("缺失"));
    QString text = QStringLiteral(
        "正式输入与依赖\n目标：%1\n边界：%2\n前序 IPW：%3\n"
        "父状态键：%4\n父检查点：%5\n本状态键：%6\n本检查点：%7\n"
        "IPW 状态：%8")
        .arg(view.targetSummary.isEmpty()
                 ? QStringLiteral("未投影") : view.targetSummary)
        .arg(view.boundaryMode.isEmpty()
                 ? QStringLiteral("未投影") : view.boundaryMode)
        .arg(predecessor, parentKey, parentCheckpoint)
        .arg(stateKey, operationCheckpoint)
        .arg(view.ipwStatus.isEmpty()
                 ? QStringLiteral("未投影") : view.ipwStatus);
    if (!view.issues.isEmpty()) {
        QStringList issues;
        for (const ToolpathOperationIssueView& issue : view.issues) {
            QString severity;
            switch (issue.severity) {
                case ToolpathOperationIssueSeverity::Info:
                    severity = QStringLiteral("提示");
                    break;
                case ToolpathOperationIssueSeverity::Warning:
                    severity = QStringLiteral("警告");
                    break;
                case ToolpathOperationIssueSeverity::Error:
                    severity = QStringLiteral("错误");
                    break;
            }
            issues.push_back(QStringLiteral("- %1 [%2] %3")
                .arg(severity, issue.code, issue.message));
        }
        text += QStringLiteral("\n状态原因：\n%1").arg(
            issues.join(QLatin1Char('\n')));
    } else {
        text += QStringLiteral("\n状态原因：无");
    }
    return text;
}

void enrichFormalOperationView(
    const MachiningWorkspaceSession& workspace,
    const manufacturing::JobOperationDescriptor& descriptor,
    ToolpathOperationView& view)
{
    projectOperationTargets(workspace.job().definition(), descriptor, view);
    projectOperationStateIssues(
        workspace, descriptor, view.hasPlayback, view);
    projectOperationIpw(workspace, descriptor, view);
    view.detailText += QStringLiteral("\n\n%1").arg(
        formalInspectorDetails(view));
}

const manufacturing::JobOperationDescriptor* descriptorForRuntime(
    const std::vector<manufacturing::JobOperationDescriptor>& descriptors,
    const std::string& runtimeId)
{
    const manufacturing::JobOperationDescriptor* result = nullptr;
    for (const auto& descriptor : descriptors) {
        if (manufacturing::MachiningOperationService::
                runtimeOperationIdMatches(descriptor.id, runtimeId) &&
            (!result || descriptor.id.size() > result->id.size())) {
            result = &descriptor;
        }
    }
    return result;
}

const manufacturing::SetupDefinition* setupForDescriptor(
    const MachiningWorkspaceSession& workspace,
    const manufacturing::JobOperationDescriptor& descriptor)
{
    const auto& setups = workspace.job().definition().setups;
    const auto found = std::find_if(
        setups.begin(), setups.end(),
        [&descriptor](const manufacturing::SetupDefinition& setup) {
            return setup.id == descriptor.setupId;
        });
    return found == setups.end() ? nullptr : &*found;
}

const manufacturing::JobToolController* toolForDescriptor(
    const MachiningWorkspaceSession& workspace,
    const manufacturing::JobOperationDescriptor& descriptor)
{
    const auto& controllers = workspace.job().definition().toolControllers;
    const auto found = std::find_if(
        controllers.begin(), controllers.end(),
        [&descriptor](const manufacturing::JobToolController& controller) {
            return controller.id == descriptor.toolControllerId;
        });
    return found == controllers.end() ? nullptr : &*found;
}

QString toolLabel(const toolpath::ToolpathToolContext& tool)
{
    if (!tool.valid)
        return QStringLiteral("未绑定刀具");
    const tooling::ToolType type = tooling::toolTypeFromId(
        QString::fromStdString(tool.toolType));
    return QStringLiteral("T%1 · %2 · Ø%3 mm")
        .arg(tool.toolNumber)
        .arg(tooling::toolTypeDisplayName(type))
        .arg(tool.diameter, 0, 'f', 3);
}

// 取刀具批次对应的回放移动段区间 [start, end)。
std::pair<int, int> toolBatchRange(
    const std::vector<PlaybackMove>& moves,
    const std::vector<int>& toolBatchStarts,
    int toolBatchIndex)
{
    const int total = static_cast<int>(moves.size());
    if (toolBatchIndex < 0 ||
        toolBatchIndex >= static_cast<int>(toolBatchStarts.size())) {
        return {0, 0};
    }
    const int start = toolBatchStarts[toolBatchIndex];
    const int end = toolBatchIndex + 1 <
            static_cast<int>(toolBatchStarts.size())
        ? toolBatchStarts[toolBatchIndex + 1]
        : total;
    return {
        std::clamp(start, 0, total),
        std::clamp(end, 0, total)};
}

QString setupText(MoldSetupFace setupFace)
{
    return setupFace == MoldSetupFace::Unresolved
        ? QStringLiteral("未解析装夹面")
        : QStringLiteral("%1 装夹面").arg(
              QString::fromLatin1(moldSetupFaceName(setupFace)));
}

} // namespace

std::vector<toolpath::HoleMachiningInput>
HoleToolpathWorkbenchCoordinator::sourceInputs(
    const MachiningWorkspaceSession& workspace)
{
    std::vector<toolpath::HoleMachiningInput> holes;
    holes.reserve(workspace.recognizedHoleSources().size());
    for (const RecognizedHoleSource& source :
         workspace.recognizedHoleSources()) {
        holes.push_back(source.hole);
    }
    return holes;
}

int HoleToolpathWorkbenchCoordinator::selectedStageIndex(
    const MachiningWorkspaceSession& workspace,
    const std::vector<std::string>& selectedFaceIds)
{
    if (workspace.activePreview() != ActiveToolpathPreview::Hole)
        return -1;
    const std::unordered_set<std::string> selected(
        selectedFaceIds.begin(), selectedFaceIds.end());
    int match = -1;
    for (int index = 0;
         index < static_cast<int>(workspace.recognizedHoleSources().size());
         ++index) {
        const RecognizedHoleSource& source =
            workspace.recognizedHoleSources()[static_cast<std::size_t>(index)];
        const std::vector<std::string>& walls =
            source.hole.stageWallFaceIds.empty()
                ? source.sourceFaceIds : source.hole.stageWallFaceIds;
        const bool matched = std::any_of(
            walls.begin(), walls.end(),
            [&selected](const std::string& faceId) {
                return selected.contains(faceId);
            });
        if (!matched)
            continue;
        // 命中多个不同孔段（选中面横跨多个孔）时返回 -1，要求唯一定位。
        if (match >= 0)
            return -1;
        match = index;
    }
    return match;
}

bool HoleToolpathWorkbenchCoordinator::previewEntryBelongsToToolBatch(
    const HolePreviewEntry& entry,
    const std::vector<PlaybackMove>& moves,
    const std::vector<int>& toolBatchStarts,
    int toolBatchIndex)
{
    // 先按批次区间扫回放移动，确认该预览条目确实落在本批次内。
    const auto [start, end] = toolBatchRange(
        moves, toolBatchStarts, toolBatchIndex);
    for (int index = start; index < end; ++index) {
        const PlaybackMove& move = moves[index];
        if (move.sourceIndex != entry.sourceIndex ||
            move.setupIndex != entry.operation.setupIndex ||
            move.phase != entry.operation.phase ||
            move.tool.valid != entry.operation.tool.valid) {
            continue;
        }
        if (move.tool.valid &&
            move.tool.toolNumber != entry.operation.tool.toolNumber) {
            continue;
        }
        return true;
    }
    return false;
}

HoleToolTreeModel HoleToolpathWorkbenchCoordinator::buildTreeModel(
    const MachiningWorkspaceSession& workspace,
    const std::vector<PlaybackMove>& moves,
    const std::vector<int>& toolBatchStarts)
{
    HoleToolTreeModel model;
    const std::vector<manufacturing::JobOperationDescriptor> descriptors =
        manufacturing::MachiningOperationService::operations(
            workspace.job().definition());
    std::unordered_set<std::string> representedDefinitions;
    model.batches.reserve(
        static_cast<qsizetype>(toolBatchStarts.size() + descriptors.size()));
    // 为每个批次区间生成批次视图：装夹面、刀具、阶段与来源。
    const int totalMoves = static_cast<int>(moves.size());
    for (int batchIndex = 0;
         batchIndex < static_cast<int>(toolBatchStarts.size());
         ++batchIndex) {
        const auto [start, end] = toolBatchRange(
            moves, toolBatchStarts, batchIndex);
        if (start < 0 || start >= totalMoves || end <= start)
            continue;
        const PlaybackMove& move = moves[start];
        HoleToolBatchView view;
        view.batchIndex = batchIndex;
        view.startMoveIndex = start;
        view.endMoveIndex = end;
        view.setupLabel = setupText(move.setupFace);
        const manufacturing::JobOperationDescriptor* batchDescriptor =
            descriptorForRuntime(descriptors, move.operationId);
        view.family = batchDescriptor
            ? operationFamily(batchDescriptor->kind)
            : operationFamily(move);
        view.sequence = batchDescriptor
            ? batchDescriptor->sequence : -1;
        QStringList details{
            QStringLiteral("阶段：%1").arg(
                phaseText(move.phase, view.family))};
        if (move.tool.valid) {
            const tooling::ToolType type = tooling::toolTypeFromId(
                QString::fromStdString(move.tool.toolType));
            view.toolLabel = QStringLiteral("T%1 · %2 · Ø%3 mm")
                .arg(move.tool.toolNumber)
                .arg(tooling::toolTypeDisplayName(type))
                .arg(move.tool.diameter, 0, 'f', 3);
            if (!move.tool.toolName.empty()) {
                details << QStringLiteral("名称：%1").arg(
                    QString::fromStdString(move.tool.toolName));
            }
            if (!move.tool.toolId.empty()) {
                details << QStringLiteral("刀具 ID：%1").arg(
                    QString::fromStdString(move.tool.toolId));
            }
        } else {
            view.toolLabel = QStringLiteral("未绑定刀具");
            details << QStringLiteral("缺少刀库快照，请重新生成");
        }
        view.detailText = details.join(QLatin1Char('\n'));
        view.setupOrder = static_cast<int>(move.setupFace);
        for (int operationStart = start; operationStart < end;) {
            const PlaybackMove& operationMove = moves[operationStart];
            int operationEnd = operationStart + 1;
            while (operationEnd < end &&
                   moves[operationEnd].operationId ==
                       operationMove.operationId) {
                ++operationEnd;
            }
            if (!operationMove.operationId.empty()) {
                ToolpathOperationView operationView;
                operationView.operationId = QString::fromStdString(
                    operationMove.operationId);
                const manufacturing::JobOperationDescriptor* descriptor =
                    descriptorForRuntime(
                        descriptors, operationMove.operationId);
                if (descriptor) {
                    representedDefinitions.insert(descriptor->id);
                    operationView.formalOperationId =
                        QString::fromStdString(descriptor->id);
                    operationView.family = operationFamily(descriptor->kind);
                    operationView.sequence = descriptor->sequence;
                    operationView.state = descriptor->state;
                    operationView.enabled = descriptor->enabled;
                    if (view.sequence < 0 ||
                        descriptor->sequence < view.sequence) {
                        view.sequence = descriptor->sequence;
                    }
                } else {
                    operationView.formalOperationId =
                        operationView.operationId;
                    operationView.family = view.family;
                }
                operationView.phaseLabel = phaseText(
                    operationMove.phase, operationView.family);
                operationView.label = operationView.sequence >= 0
                    ? QStringLiteral("#%1 · %2 · %3［%4］")
                          .arg(operationView.sequence + 1)
                          .arg(operationView.phaseLabel,
                               operationView.operationId,
                               operationStateText(operationView.state))
                    : QStringLiteral("%1 · %2")
                          .arg(operationView.phaseLabel,
                               operationView.operationId);
                operationView.sourceIndex = operationMove.sourceIndex;
                operationView.setupLabel = view.setupLabel;
                operationView.toolLabel = view.toolLabel;
                QString sourceText;
                QString editText;
                if (view.family == ToolpathBatchFamily::Hole) {
                    const int sourceIndex = operationView.sourceIndex;
                    if (sourceIndex >= 0 && sourceIndex < static_cast<int>(
                            workspace.recognizedHoleSources().size())) {
                        const RecognizedHoleSource& source =
                            workspace.recognizedHoleSources()[
                                static_cast<std::size_t>(sourceIndex)];
                        operationView.stageKey =
                            QString::fromStdString(source.key);
                        sourceText = QStringLiteral("孔段 #%1 · %2")
                            .arg(sourceIndex + 1)
                            .arg(featureKindText(source.hole.kind));
                        editText = QStringLiteral("参数作用域：自动孔加工；修改项目参数后重新生成");
                    } else {
                        sourceText = QStringLiteral("孔段来源未解析");
                        editText = QStringLiteral("当前工序不可直接编辑");
                    }
                } else if (view.family == ToolpathBatchFamily::Pocket) {
                    sourceText = operationView.sourceIndex >= 0
                        ? QStringLiteral("槽/腔来源 #%1")
                              .arg(operationView.sourceIndex + 1)
                        : QStringLiteral("槽/腔批次");
                    editText = QStringLiteral(
                        "参数作用域：槽/腔批次；修改后需重新生成");
                } else if (view.family == ToolpathBatchFamily::Profile) {
                    sourceText = QStringLiteral("正式外轮廓工序");
                    editText = QStringLiteral(
                        "参数作用域：外轮廓批次；修改后需重新生成");
                } else if (view.family == ToolpathBatchFamily::SurfaceFinishing) {
                    sourceText = QStringLiteral("正式雕刻加工工序");
                    editText = QStringLiteral("参数作用域：雕刻加工工序组；采样接触结果需结合IPW与碰撞检查审查");
                }
                const QString cuttingText = operationMove.tool.valid
                    ? QStringLiteral(
                          "主轴：%1 rpm · 水平/垂直进给：%2/%3 mm/min")
                          .arg(operationMove.tool.spindleSpeed, 0, 'f', 0)
                          .arg(operationMove.tool.horizontalFeed, 0, 'f', 1)
                          .arg(operationMove.tool.verticalFeed, 0, 'f', 1)
                    : QStringLiteral("切削参数：缺少刀库快照");
                operationView.detailText = QStringLiteral(
                    "运行期 ID：%1\n正式工序：%2\n状态：%3%4\n来源：%5\n阶段：%6\n装夹：%7\n刀具：%8\n"
                    "%9\n移动范围：#%10–#%11（%12 段）\n%13")
                    .arg(operationView.operationId)
                    .arg(operationView.formalOperationId)
                    .arg(operationStateText(operationView.state))
                    .arg(operationView.enabled
                             ? QString{} : QStringLiteral("（已禁用）"))
                    .arg(sourceText)
                    .arg(operationView.phaseLabel)
                    .arg(operationView.setupLabel)
                    .arg(operationView.toolLabel)
                    .arg(cuttingText)
                    .arg(operationStart + 1)
                    .arg(operationEnd)
                    .arg(operationEnd - operationStart)
                    .arg(editText);
                operationView.startMoveIndex = operationStart;
                operationView.endMoveIndex = operationEnd;
                if (descriptor)
                    enrichFormalOperationView(
                        workspace, *descriptor, operationView);
                view.operations.push_back(std::move(operationView));
            }
            operationStart = operationEnd;
        }
        model.batches.push_back(std::move(view));
    }

    // 没有活动播放结果的正式定义仍必须留在工序树中，供用户查看禁用、
    // 待生成或下游失效状态。每个合成批次只承载一个正式工序，不伪造移动。
    for (const manufacturing::JobOperationDescriptor& descriptor :
         descriptors) {
        if (representedDefinitions.contains(descriptor.id))
            continue;
        HoleToolBatchView view;
        view.batchIndex = -1;
        view.startMoveIndex = totalMoves;
        view.endMoveIndex = totalMoves;
        view.family = operationFamily(descriptor.kind);
        view.sequence = descriptor.sequence;
        const manufacturing::SetupDefinition* setup = setupForDescriptor(
            workspace, descriptor);
        view.setupOrder = setup
            ? static_cast<int>(setup->face) : 6;
        view.setupLabel = setup
            ? setupText(setup->face) : QStringLiteral("缺失 Setup");
        const manufacturing::JobToolController* controller =
            toolForDescriptor(workspace, descriptor);
        view.toolLabel = controller
            ? toolLabel(controller->tool) : QStringLiteral("缺失刀具快照");
        view.detailText = QStringLiteral(
            "正式工序 #%1 · %2\n状态：%3%4\n算法：%5\n当前没有可播放的已验证移动")
            .arg(descriptor.sequence + 1)
            .arg(QString::fromStdString(descriptor.id))
            .arg(operationStateText(descriptor.state))
            .arg(descriptor.enabled
                     ? QString{} : QStringLiteral("（已禁用）"))
            .arg(QString::fromStdString(descriptor.algorithmId));
        ToolpathOperationView operationView;
        operationView.operationId = QString::fromStdString(descriptor.id);
        operationView.formalOperationId = operationView.operationId;
        operationView.label = QStringLiteral("#%1 · %2［%3%4］")
            .arg(descriptor.sequence + 1)
            .arg(operationView.operationId)
            .arg(operationStateText(descriptor.state))
            .arg(descriptor.enabled
                     ? QString{} : QStringLiteral("，已禁用"));
        operationView.detailText = view.detailText;
        operationView.setupLabel = view.setupLabel;
        operationView.toolLabel = view.toolLabel;
        operationView.phaseLabel = phaseText(
            toolpath::HoleMachiningPhase::Standard, view.family);
        operationView.family = view.family;
        operationView.sequence = descriptor.sequence;
        operationView.state = descriptor.state;
        operationView.enabled = descriptor.enabled;
        operationView.hasPlayback = false;
        operationView.startMoveIndex = -1;
        operationView.endMoveIndex = -1;
        enrichFormalOperationView(
            workspace, descriptor, operationView);
        view.operations.push_back(std::move(operationView));
        model.batches.push_back(std::move(view));
    }

    return model;
}

QVector<HoleCoverageOperationBinding>
HoleToolpathWorkbenchCoordinator::buildCoverageOperationBindings(
    const MachiningWorkspaceSession& workspace,
    const std::vector<PlaybackMove>& moves)
{
    QVector<HoleCoverageOperationBinding> bindings;
    const auto& operations = workspace.job().definition().holeOperations;
    bindings.reserve(static_cast<qsizetype>(operations.size()));
    for (const manufacturing::HoleOperationDefinition& operation :
         operations) {
        HoleCoverageOperationBinding binding;
        binding.formalOperationId = QString::fromStdString(operation.id);
        binding.sequence = operation.sequence;
        binding.state = operation.state;
        binding.enabled = operation.enabled;
        std::vector<int> sourceIndices;
        for (const manufacturing::HoleTargetReference& target :
             operation.targets) {
            if (target.sourceHoleIndex >= 0)
                sourceIndices.push_back(target.sourceHoleIndex);
            sourceIndices.insert(
                sourceIndices.end(),
                target.coveredSourceHoleIndices.begin(),
                target.coveredSourceHoleIndices.end());
        }
        std::erase_if(
            sourceIndices, [](int sourceIndex) { return sourceIndex < 0; });
        std::sort(sourceIndices.begin(), sourceIndices.end());
        sourceIndices.erase(
            std::unique(sourceIndices.begin(), sourceIndices.end()),
            sourceIndices.end());
        for (int sourceIndex : sourceIndices)
            binding.sourceHoleIndices.push_back(sourceIndex);
        binding.hasPlayback = std::any_of(
            moves.begin(), moves.end(),
            [&operation](const PlaybackMove& move) {
                return manufacturing::MachiningOperationService::
                    runtimeOperationIdMatches(
                        operation.id, move.operationId);
            });
        bindings.push_back(std::move(binding));
    }
    std::stable_sort(
        bindings.begin(), bindings.end(),
        [](const HoleCoverageOperationBinding& left,
           const HoleCoverageOperationBinding& right) {
            if (left.sequence != right.sequence)
                return left.sequence < right.sequence;
            return left.formalOperationId < right.formalOperationId;
        });
    return bindings;
}

} // namespace oap::app
