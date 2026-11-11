// StockSimulationWorkflowController 实现：来源校验、结果安装与摘要报告。
#include "app/workflows/simulation/StockSimulationWorkflowController.h"
#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>

#include <QFileInfo>
#include <QStringList>

#include "toolpath/hole/HoleMachining.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h"

namespace oap::app {
namespace {

// 减料状态 → 中文显示文本。
QString removalStatusText(toolpath::StockRemovalStatus status)
{
    switch (status) {
        case toolpath::StockRemovalStatus::Applied:
            return QCoreApplication::translate("OpenAutoPath", "已减料");
        case toolpath::StockRemovalStatus::NoMaterialRemoved:
            return QCoreApplication::translate("OpenAutoPath", "未接触当前毛坯");
        case toolpath::StockRemovalStatus::InvalidStock:
            return QCoreApplication::translate("OpenAutoPath", "毛坯无效");
        case toolpath::StockRemovalStatus::InvalidRemovalVolume:
            return QCoreApplication::translate("OpenAutoPath", "切削体无效");
        case toolpath::StockRemovalStatus::BooleanFailed:
            return QCoreApplication::translate("OpenAutoPath", "布尔减料失败");
        case toolpath::StockRemovalStatus::InvalidResult:
            return QCoreApplication::translate("OpenAutoPath", "减料结果无效");
        case toolpath::StockRemovalStatus::Cancelled:
            return QCoreApplication::translate("OpenAutoPath", "已取消");
    }
    return QCoreApplication::translate("OpenAutoPath", "未知");
}

// 累计诊断中的警告/错误计数。
void accumulateDiagnostics(
    const std::vector<toolpath::ToolpathDiagnostic>& diagnostics,
    int& warningCount,
    int& errorCount)
{
    for (const toolpath::ToolpathDiagnostic& diagnostic : diagnostics) {
        if (diagnostic.severity ==
            toolpath::ToolpathDiagnosticSeverity::Error) {
            ++errorCount;
        } else if (diagnostic.severity ==
                   toolpath::ToolpathDiagnosticSeverity::Warning) {
            ++warningCount;
        }
    }
}

// 刀具显示文本：编号刀具用 T 号+名称，缺少刀具快照时显示直径。
QString holeToolLabel(const toolpath::HoleMachiningOperation& operation)
{
    if (operation.tool.toolNumber > 0) {
        return QStringLiteral("T%1 %2")
            .arg(operation.tool.toolNumber)
            .arg(QString::fromStdString(operation.tool.toolName));
    }
    return QCoreApplication::translate("OpenAutoPath", "未绑定刀具 Ø%1")
        .arg(operation.tool.diameter, 0, 'f', 3);
}

// 逐特征余量摘要：最多列出 maximumItems 项，超出部分省略。
QString featureAllowanceSummary(
    const std::vector<toolpath::StockFeatureAllowance>& allowances,
    int maximumItems)
{
    if (allowances.empty())
        return {};
    QStringList items;
    const int count = std::min(
        maximumItems, static_cast<int>(allowances.size()));
    for (int index = 0; index < count; ++index) {
        const toolpath::StockFeatureAllowance& item =
            allowances[static_cast<std::size_t>(index)];
        QString label;
        if (item.sourceIndex >= 0) {
            label = QCoreApplication::translate("OpenAutoPath", "孔 %1").arg(item.sourceIndex + 1);
        } else {
            const QString kind = item.featureKind == "slot"
                ? QCoreApplication::translate("OpenAutoPath", "槽")
                : QCoreApplication::translate("OpenAutoPath", "腔");
            QString identity = QString::fromStdString(
                item.operationId.empty()
                    ? item.featureId : item.operationId);
            if (identity.size() > 24)
                identity = identity.left(21) + QStringLiteral("…");
            label = QStringLiteral("%1 %2").arg(kind, identity);
        }
        items.push_back(QStringLiteral("%1：%2%3 mm³（%4%）")
            .arg(label)
            .arg(item.approximateBoundary ? QCoreApplication::translate("OpenAutoPath", "约 ") : QString{})
            .arg(item.remainingAllowanceVolume, 0, 'f', 3)
            .arg(100.0 * item.completionRatio, 0, 'f', 1));
    }
    QString text = QCoreApplication::translate("OpenAutoPath", "\n逐特征余量：") +
        items.join(QStringLiteral("；"));
    if (count < static_cast<int>(allowances.size())) {
        text += QCoreApplication::translate("OpenAutoPath", "；另有 %1 项")
            .arg(static_cast<int>(allowances.size()) - count);
    }
    return text;
}

bool selectPocketSimulationOperations(
    const manufacturing::MachiningJob& job,
    const toolpath::PocketToolpathBatch& batch,
    int selectedSetup,
    PocketSimulationPreparation& result)
{
    std::vector<const manufacturing::PocketOperationDefinition*> definitions;
    for (const auto& definition : job.definition().pocketOperations) {
        const auto* setup = job.setupById(definition.setupId);
        if (definition.enabled && setup && setup->setupIndex == selectedSetup)
            definitions.push_back(&definition);
    }
    std::stable_sort(definitions.begin(), definitions.end(),
        [](const auto* left, const auto* right) { return left->sequence < right->sequence; });
    std::vector<std::string> pocketOperationIds;
    std::vector<bool> consumed(batch.operations.size(), false);
    std::unordered_set<std::string> runtimeIds;
    for (const auto* definition : definitions) {
        const auto before = result.operations.size();
        for (std::size_t index = 0; index < batch.operations.size(); ++index) {
            const auto& operation = batch.operations[index];
            if (!manufacturing::runtimeOperationIdMatches(definition->id, operation.operationId))
                continue;
            if (consumed[index] || operation.setupIndex != selectedSetup ||
                !runtimeIds.insert(operation.operationId).second) {
                result.error = QCoreApplication::translate("OpenAutoPath", "槽腔工序 %1 的运行期工序 %2 重复匹配或装夹不一致（预期 %3，实际 %4）。")
                    .arg(QString::fromStdString(definition->id))
                    .arg(QString::fromStdString(operation.operationId))
                    .arg(selectedSetup).arg(operation.setupIndex);
                return false;
            }
            consumed[index] = true;
            result.operations.push_back(operation);
        }
        if (before == result.operations.size()) {
            result.error = QCoreApplication::translate("OpenAutoPath", "槽腔工序 %1 在装夹 %2 中没有对应的运行期刀路。")
                .arg(QString::fromStdString(definition->id)).arg(selectedSetup);
            return false;
        }
        pocketOperationIds.push_back(definition->id);
    }
    if (result.operations.empty()) {
        result.error = QCoreApplication::translate("OpenAutoPath", "装夹 %1 没有已启用的槽腔工序。")
            .arg(selectedSetup);
        return false;
    }
    result.includedOperationIds.reserve(
        pocketOperationIds.size() +
        job.definition().holeOperations.size());
    for (const manufacturing::HoleOperationDefinition& definition :
         job.definition().holeOperations) {
        const manufacturing::SetupDefinition* setup =
            job.setupById(definition.setupId);
        if (definition.enabled && setup &&
            setup->setupIndex == selectedSetup) {
            result.includedOperationIds.push_back(definition.id);
        }
    }
    result.includedOperationIds.insert(
        result.includedOperationIds.end(),
        pocketOperationIds.begin(), pocketOperationIds.end());

    return true;
}

template <typename Preparation>
bool preparePocketStockSource(
    const manufacturing::MachiningJob& job,
    const StockSourceSettings& source,
    const QString& importedSourceFingerprint,
    Preparation& result)
{
    result.stockDefinition.marginX = source.marginX;
    result.stockDefinition.marginY = source.marginY;
    result.stockDefinition.marginZ = source.marginZ;
    if (source.sourceMode == 2) {
        if (importedSourceFingerprint.isEmpty()) {
            result.error = QCoreApplication::translate("OpenAutoPath", "无法计算毛坯 STEP 指纹。");
            return false;
        }
        result.stockFingerprint =
            importedSourceFingerprint.toStdString();
        result.stockDefinition.sourceKind =
            manufacturing::StockSourceKind::StepModel;
        result.stockDefinition.sourcePath =
            source.stepPath.toUtf8().toStdString();
    } else {
        // 生成毛坯指纹：模型来源指纹 + 来源模式 + 余量，保证
        // 相同设置复用同一检查点链。
        std::ostringstream fingerprint;
        fingerprint.precision(17);
        fingerprint << "generated-pocket-stock-v1|"
                    << job.definition().part.sourceFingerprint << '|'
                    << source.sourceMode << '|' << source.marginX << '|'
                    << source.marginY << '|' << source.marginZ;
        result.stockFingerprint = fingerprint.str();
        result.stockDefinition.sourceKind = source.sourceMode == 0
            ? manufacturing::StockSourceKind::PerUnitBoxes
            : manufacturing::StockSourceKind::ModelBox;
    }

    return true;
}

void configurePocketSimulationRequest(
    int displayQuality,
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape,
    PocketSimulationPreparation& result)
{
    result.quality = ipwSimulationQualityProfile(
        displayQuality, result.effectiveParams.tool.diameter);
    result.request.initialStock = initialStock;
    result.request.targetShape = targetShape;
    result.request.initialStockFingerprint = result.stockFingerprint;
    result.request.options.includedOperationIds =
        result.includedOperationIds;
    result.request.options.safeMixedTopologyFallback = false;
    result.request.options.pocket.method =
        toolpath::StockSimulationMethod::HeightField;
    result.request.options.pocket.runParallel = true;
    result.request.options.pocket.zMapResolution =
        result.quality.requestedResolution;
    result.request.options.pocket.zMapMaxCellCount =
        result.quality.maximumCellCount;
    int cuttingMoveCount = 0;
    for (const toolpath::PocketToolpathOperation& operation :
         result.operations) {
        for (const toolpath::ToolpathMove& move : operation.clearing.moves) {
            if (move.type == toolpath::ToolpathMoveType::Cut ||
                move.type == toolpath::ToolpathMoveType::Plunge) {
                ++cuttingMoveCount;
            }
        }
    }
    // 依 Cut/Plunge 移动总数估算扫描段预算（额外留 16 个余量）。
    result.request.options.pocket.maxSweepSegments =
        std::clamp(cuttingMoveCount + 16, 20000, 5000000);
    result.request.comparisonOptions.runParallel = true;
    result.pocketOptions = result.request.options.pocket;
}

} // namespace

// 校验来源设置：模式范围、STEP 文件存在性、余量非负且有限。
bool StockSimulationWorkflowController::validateSource(
    const StockSourceSettings& settings,
    QString& error)
{
    error.clear();
    if (settings.sourceMode < 0 || settings.sourceMode > 2) {
        error = QCoreApplication::translate("OpenAutoPath", "未知的毛坯来源。");
        return false;
    }
    if (settings.sourceMode == 2) {
        if (settings.stepPath.trimmed().isEmpty() ||
            !QFileInfo::exists(settings.stepPath.trimmed())) {
            error = QCoreApplication::translate("OpenAutoPath", "请选择存在的加工前毛坯 STEP 文件。");
            return false;
        }
        return true;
    }
    if (!std::isfinite(settings.marginX) ||
        !std::isfinite(settings.marginY) ||
        !std::isfinite(settings.marginZ) ||
        settings.marginX < 0.0 || settings.marginY < 0.0 ||
        settings.marginZ < 0.0) {
        error = QCoreApplication::translate("OpenAutoPath", "毛坯余量必须是有限的非负数。");
        return false;
    }
    return true;
}

void StockSimulationWorkflowController::applyHoleSettings(
    StockSimulationSession& session,
    const StockSourceSettings& settings)
{
    session.holeSourceMode() = settings.sourceMode;
    session.holeMarginX() = settings.marginX;
    session.holeMarginY() = settings.marginY;
    session.holeMarginZ() = settings.marginZ;
    session.holeStockStepPath() = settings.stepPath.trimmed();
}

void StockSimulationWorkflowController::applyPocketSettings(
    StockSimulationSession& session,
    const StockSourceSettings& settings)
{
    session.pocketSourceMode() = settings.sourceMode;
    session.pocketMarginX() = settings.marginX;
    session.pocketMarginY() = settings.marginY;
    session.pocketMarginZ() = settings.marginZ;
    session.pocketStockStepPath() = settings.stepPath.trimmed();
}

// 安装孔仿真结果：移动结果入会话、记录来源指纹并按初始毛坯非空刷新门禁。
void StockSimulationWorkflowController::installHoleResult(
    StockSimulationSession& session,
    HoleStockSimulationRunResult& result,
    const QString& sourceFingerprint)
{
    session.holeSimulation() = std::move(result.simulation);
    session.holeComparison() = std::move(result.comparison);
    session.holeOvercutAttribution() =
        std::move(result.overcutAttribution);
    session.holeSourceFingerprint() = sourceFingerprint;
    session.hasHoleSimulation() =
        !session.holeSimulation().initialStock.IsNull();
}

std::vector<HoleStockOvercutChoice>
StockSimulationWorkflowController::holeOvercutChoices(
    const StockSimulationSession& session,
    std::span<const HolePreviewEntry> previewEntries)
{
    std::vector<HoleStockOvercutChoice> result;
    const auto& attributions =
        session.holeOvercutAttribution().operations;
    result.reserve(attributions.size());
    for (int index = 0; index < static_cast<int>(attributions.size());
         ++index) {
        const toolpath::StockOperationOvercut& attribution =
            attributions[static_cast<std::size_t>(index)];
        QString hole = QCoreApplication::translate("OpenAutoPath", "未知孔");
        QString tool = QCoreApplication::translate("OpenAutoPath", "未绑定刀具");
        if (attribution.sequenceIndex >= 0 &&
            attribution.sequenceIndex <
                static_cast<int>(previewEntries.size())) {
            const toolpath::HoleMachiningOperation& operation =
                previewEntries[static_cast<std::size_t>(
                    attribution.sequenceIndex)].operation;
            hole = operation.sourceHoleIndex >= 0
                ? QCoreApplication::translate("OpenAutoPath", "孔 %1")
                      .arg(operation.sourceHoleIndex + 1)
                : QCoreApplication::translate("OpenAutoPath", "未知孔");
            tool = holeToolLabel(operation);
        }
        result.push_back({
            QStringLiteral("%1 | %2 | %3 | %4 mm³")
                .arg(QString::fromStdString(attribution.operationId))
                .arg(hole)
                .arg(tool)
                .arg(attribution.volume, 0, 'f', 3),
            index});
    }
    return result;
}

QString StockSimulationWorkflowController::holeSummary(
    const StockSimulationSession& session,
    std::span<const HolePreviewEntry> previewEntries,
    const HoleStockDisplaySelection& selection)
{
    if (!session.hasHoleSimulation())
        return QCoreApplication::translate("OpenAutoPath", "尚未执行仿真。");

    const auto& simulation = session.holeSimulation();
    const auto& comparison = session.holeComparison();
    const auto& overcut = session.holeOvercutAttribution();
    const int total = static_cast<int>(simulation.steps.size());
    const int step = std::clamp(selection.stepIndex, 0, total);
    QString detail;
    if (step == 0 || total == 0) {
        detail = QCoreApplication::translate("OpenAutoPath", "快照 0/%1：初始毛坯，体积 %2 mm³。")
                     .arg(total)
                     .arg(simulation.initialVolume, 0, 'f', 3);
    } else {
        const toolpath::StockRemovalRecord& record =
            simulation.steps[static_cast<std::size_t>(step - 1)];
        detail = QCoreApplication::translate("OpenAutoPath", "快照 %1/%2：工序 %3，%4。\n"
                     "体积 %5 → %6 mm³，本工序去除 %7 mm³。")
                     .arg(step)
                     .arg(total)
                     .arg(QString::fromStdString(record.operationId))
                     .arg(removalStatusText(record.status))
                     .arg(record.volumeBefore, 0, 'f', 3)
                     .arg(record.volumeAfter, 0, 'f', 3)
                     .arg(record.removedVolume, 0, 'f', 3);
        if (record.status != toolpath::StockRemovalStatus::Applied &&
            !record.message.empty()) {
            detail += QCoreApplication::translate("OpenAutoPath", "\n原因：%1")
                          .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(record.message).toUtf8().constData()));
        }
    }

    int warningCount = 0;
    int errorCount = 0;
    accumulateDiagnostics(
        simulation.diagnostics, warningCount, errorCount);
    accumulateDiagnostics(
        comparison.diagnostics, warningCount, errorCount);
    accumulateDiagnostics(
        overcut.diagnostics, warningCount, errorCount);
    const QString completionState = simulation.cancelled
        ? QCoreApplication::translate("OpenAutoPath", "仿真已取消，已保留完成快照")
        : (simulation.complete ? QCoreApplication::translate("OpenAutoPath", "仿真完成")
                               : QCoreApplication::translate("OpenAutoPath", "仿真未完整完成"));
    detail += QCoreApplication::translate("OpenAutoPath", "\n总计：初始 %1，最终 %2，去除 %3 mm³；"
                  "成功 %4，空切 %5，警告 %6，错误 %7，%8。")
                  .arg(simulation.initialVolume, 0, 'f', 3)
                  .arg(simulation.finalVolume, 0, 'f', 3)
                  .arg(simulation.removedVolume, 0, 'f', 3)
                  .arg(simulation.appliedOperationCount)
                  .arg(simulation.noMaterialOperationCount)
                  .arg(warningCount)
                  .arg(errorCount)
                  .arg(completionState);
    if (comparison.complete) {
        detail += QCoreApplication::translate("OpenAutoPath", "\n差异检查：目标包含率 %1%，初始缺料 %2，"
                      "剩余加工余量 %3，过切 %4 mm³。")
                      .arg(100.0 * comparison.targetContainmentRatio,
                           0, 'f', 4)
                      .arg(comparison.initialMissingVolume, 0, 'f', 3)
                      .arg(comparison.remainingAllowanceVolume, 0, 'f', 3)
                      .arg(comparison.overcutVolume, 0, 'f', 3);
        detail += featureAllowanceSummary(
            comparison.featureAllowances, 12);
        if (comparison.hasBounds) {
            detail += QCoreApplication::translate("OpenAutoPath", "\nAABB 中心偏移：X=%1，Y=%2，Z=%3 mm；"
                          "目标包络%4初始毛坯包络。")
                          .arg(comparison.boundsCenterOffset[0], 0, 'f', 3)
                          .arg(comparison.boundsCenterOffset[1], 0, 'f', 3)
                          .arg(comparison.boundsCenterOffset[2], 0, 'f', 3)
                          .arg(comparison.targetBoundsInsideInitialBounds
                                   ? QCoreApplication::translate("OpenAutoPath", "位于")
                                   : QCoreApplication::translate("OpenAutoPath", "超出"));
        }
    } else if (comparison.cancelled) {
        detail += QCoreApplication::translate("OpenAutoPath", "\n毛坯差异检查已取消；孔工序快照仍然有效。");
    } else if (!comparison.diagnostics.empty()) {
        detail += QCoreApplication::translate("OpenAutoPath", "\n毛坯差异检查未完成，请查看错误计数。");
    }
    if (overcut.complete) {
        detail += QCoreApplication::translate("OpenAutoPath", "\n过切归因：%1 道工序，合计 %2 mm³。")
                      .arg(static_cast<int>(overcut.operations.size()))
                      .arg(overcut.attributedVolume, 0, 'f', 3);
    } else if (overcut.cancelled) {
        detail += QCoreApplication::translate("OpenAutoPath", "\n逐工序过切归因已取消。");
    }

    switch (selection.viewMode) {
        case 1:
            detail += QCoreApplication::translate("OpenAutoPath", "\n当前显示：初始毛坯缺失的目标材料。");
            break;
        case 2:
            detail += QCoreApplication::translate("OpenAutoPath", "\n当前显示：当前孔工序后仍在成品目标外的加工余量。");
            break;
        case 3:
            detail += QCoreApplication::translate("OpenAutoPath", "\n当前显示：当前孔刀路误删的目标材料。");
            break;
        case 4: {
            const int index = selection.overcutAttributionIndex;
            if (index < 0 || index >= static_cast<int>(overcut.operations.size()))
                break;
            const toolpath::StockOperationOvercut& attribution =
                overcut.operations[static_cast<std::size_t>(index)];
            QString operationDetail = QCoreApplication::translate("OpenAutoPath", "\n当前显示：工序 %1，新增过切 %2 mm³。")
                .arg(QString::fromStdString(attribution.operationId))
                .arg(attribution.volume, 0, 'f', 3);
            if (attribution.sequenceIndex >= 0 &&
                attribution.sequenceIndex <
                    static_cast<int>(previewEntries.size())) {
                const toolpath::HoleMachiningOperation& operation =
                    previewEntries[static_cast<std::size_t>(
                        attribution.sequenceIndex)].operation;
                const QString holeSource = operation.sourceHoleIndex >= 0
                    ? QString::number(operation.sourceHoleIndex + 1)
                    : QCoreApplication::translate("OpenAutoPath", "未关联");
                operationDetail += QCoreApplication::translate("OpenAutoPath", " 孔源=%1，策略=%2，刀具=%3，直径=%4 mm。")
                    .arg(holeSource)
                    .arg(QCoreApplication::translate("OpenAutoPath",
                        toolpath::holeMachiningStrategyZh(
                            operation.strategy)))
                    .arg(holeToolLabel(operation))
                    .arg(operation.tool.diameter, 0, 'f', 3);
            }
            detail += operationDetail;
            break;
        }
        default:
            break;
    }
    return detail;
}

QString StockSimulationWorkflowController::holeCompletionStatus(
    const StockSimulationSession& session,
    bool cancelled)
{
    const auto& simulation = session.holeSimulation();
    const auto& comparison = session.holeComparison();
    const auto& attribution = session.holeOvercutAttribution();
    QString summary = QCoreApplication::translate("OpenAutoPath", "%1：%2/%3 个操作成功减料，去除 %4 mm³，"
        "初始缺料 %5 mm³，过切 %6 mm³（%7 道工序）")
        .arg(cancelled ? QCoreApplication::translate("OpenAutoPath", "孔毛胚仿真已取消")
                       : QCoreApplication::translate("OpenAutoPath", "孔毛胚仿真完成"))
        .arg(simulation.appliedOperationCount)
        .arg(simulation.processedOperationCount)
        .arg(simulation.removedVolume, 0, 'f', 3)
        .arg(comparison.initialMissingVolume, 0, 'f', 3)
        .arg(comparison.overcutVolume, 0, 'f', 3)
        .arg(static_cast<int>(attribution.operations.size()));
    summary += featureAllowanceSummary(
        comparison.featureAllowances, 6);
    return summary;
}

// 准备腔仿真：筛选所选装夹面的腔工序与 Job 定义核对数量，合成来源指纹
// （STEP 用文件指纹，生成毛坯用稳定的生成参数指纹），构造请求与质量档。
ProjectSimulationPreparation StockSimulationWorkflowController::prepareProject(
    const manufacturing::MachiningJob& job,
    const StockSourceSettings& source,
    const QString& importedSourceFingerprint,
    int displayQuality,
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape,
    bool engravingOnly,
    int selectedSetup)
{
    ProjectSimulationPreparation result;
    if (!validateSource(source, result.error))
        return result;
    if (initialStock.IsNull() || targetShape.IsNull()) {
        result.error = QCoreApplication::translate("OpenAutoPath", "毛坯或目标实体为空。");
        return result;
    }
    const auto definitionErrors = job.validationErrors();
    if (!definitionErrors.empty()) {
        result.error = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(definitionErrors.front()).toUtf8().constData());
        return result;
    }
    auto definitions = manufacturing::MachiningOperationService::operations(job.definition());
    if (engravingOnly) {
        const auto last = std::find_if(definitions.rbegin(), definitions.rend(), [&](const auto& operation) {
            const auto* setup = job.setupById(operation.setupId);
            return operation.enabled && operation.kind == manufacturing::JobOperationKind::SurfaceFinishing &&
                setup && (selectedSetup < 0 || setup->setupIndex == selectedSetup);
        });
        if (last == definitions.rend()) {
            result.error = QCoreApplication::translate("OpenAutoPath", "所选装夹范围没有启用的雕刻工序，请先生成雕刻刀路。");
            return result;
        }
        definitions.erase(last.base(), definitions.end());
    }
    double minimumDiameter = std::numeric_limits<double>::infinity();
    std::size_t moveCount = 0;
    for (const auto& definition : definitions) {
        if (!definition.enabled)
            continue;
        const auto operationId = QString::fromStdString(definition.id);
        using State = manufacturing::JobOperationState;
        switch (definition.state) {
        case State::Planned:
            result.error = QCoreApplication::translate("OpenAutoPath", "工序 %1 尚未生成刀路，请先生成对应工序刀路。").arg(operationId);
            break;
        case State::Dirty:
        case State::Stale:
            result.error = QCoreApplication::translate("OpenAutoPath", "工序 %1 的刀路已失效，请按当前模型和参数重新生成。").arg(operationId);
            break;
        case State::Error:
            result.error = QCoreApplication::translate("OpenAutoPath", "工序 %1 的刀路生成或校验失败，请查看工序诊断，修正后重新生成。").arg(operationId);
            break;
        case State::NeedsReview: {
            const auto* setup = job.setupById(definition.setupId);
            if (!setup || !setup->valid) {
                result.error = QCoreApplication::translate("OpenAutoPath", "工序 %1 的装夹无效，请在“加工参数与装夹确认…”中修正装夹后重新生成刀路。").arg(operationId);
            } else if (!setup->confirmation.confirmed) {
                result.error = QCoreApplication::translate("OpenAutoPath", "工序 %1 的 %2 方向装夹待确认，请在“加工参数与装夹确认…”→“装夹”中核对该方向并勾选“已确认”，应用后重试。")
                    .arg(operationId, QString::fromLatin1(moldSetupFaceName(setup->face)));
            } else {
                result.error = QCoreApplication::translate("OpenAutoPath", "工序 %1 需要复核，请检查工序的稳定引用和装夹状态，必要时重新生成刀路。").arg(operationId);
            }
            break;
        }
        case State::Generated:
        case State::Warning:
            break;
        }
        if (!result.error.isEmpty()) {
            return result;
        }
        toolpath::ToolpathProgram program;
        std::string expected, stored;
        using Kind = manufacturing::JobOperationKind;
        switch (definition.kind) {
        case Kind::Hole:
            if (const auto* batch = job.holeResult())
                program = batch->toProgram();
            expected = manufacturing::MachiningJobStorage::holeExecutionFingerprint(job.definition()).toStdString();
            stored = job.runtime().holeExecutionFingerprint;
            break;
        case Kind::Pocket:
            if (const auto* batch = job.currentPocketResult())
                program = batch->toProgram();
            expected = manufacturing::MachiningJobStorage::pocketExecutionFingerprint(job.definition()).toStdString();
            stored = job.runtime().pocketExecutionFingerprint;
            break;
        case Kind::Profile:
            if (const auto* batch = job.profileResult())
                program = batch->toProgram();
            expected = manufacturing::MachiningJobStorage::profileExecutionFingerprint(job.definition()).toStdString();
            stored = job.runtime().profileExecutionFingerprint;
            break;
        case Kind::SurfaceFinishing:
            if (const auto* batch = job.currentFinishingResult())
                program = batch->toProgram();
            expected = manufacturing::MachiningJobStorage::finishingExecutionFingerprint(job.definition()).toStdString();
            stored = job.runtime().finishingExecutionFingerprint;
            break;
        }
        bool found = false;
        for (const auto& operation : program.operations) {
            if (!manufacturing::runtimeOperationIdMatches(definition.id, operation.operationId))
                continue;
            found = found || !operation.moves.empty();
            moveCount += operation.moves.size();
            if (operation.tool.valid && operation.tool.diameter > 0)
                minimumDiameter = std::min(minimumDiameter, operation.tool.diameter);
        }
        if (!found || stored.empty() || stored != expected) {
            result.error = QCoreApplication::translate("OpenAutoPath", "工序 %1 缺少当前有效刀路，请重新生成。").arg(
                QString::fromStdString(definition.id));
            return result;
        }
        result.includedOperationIds.push_back(definition.id);
    }
    if (result.includedOperationIds.empty()) {
        result.error = QCoreApplication::translate("OpenAutoPath", "加工项目没有启用的正式工序。");
        return result;
    }
    if (!preparePocketStockSource(job, source, importedSourceFingerprint, result))
        return result;
    result.quality = ipwSimulationQualityProfile(displayQuality,
        std::isfinite(minimumDiameter) ? minimumDiameter : 1.0);
    result.request.initialStock = initialStock;
    result.request.targetShape = targetShape;
    result.request.initialStockFingerprint = result.stockFingerprint;
    result.request.options.includedOperationIds = result.includedOperationIds;
    result.request.options.safeMixedTopologyFallback = true;
    auto& options = result.request.options.pocket;
    options.method = toolpath::StockSimulationMethod::HeightField;
    options.runParallel = true;
    options.zMapResolution = result.quality.requestedResolution;
    options.zMapMaxCellCount = result.quality.maximumCellCount;
    options.maxSweepSegments = static_cast<int>(std::clamp<std::size_t>(
        moveCount + 16, 20000, 5000000));
    result.request.options.hole = options;
    result.request.comparisonOptions.runParallel = true;
    result.ipwOptions = options;
    result.ok = true;
    return result;
}

void StockSimulationWorkflowController::installProjectResult(
    StockSimulationSession& session,
    const manufacturing::MachiningJob&,
    const ProjectSimulationPreparation& preparation,
    IpwSimulationRunResult& result)
{
    session.pocketSimulation() = std::move(result.simulation);
    session.pocketComparison() = std::move(result.comparison);
    session.pocketOvercutAttribution() = std::move(result.overcutAttribution);
    session.pocketSourceFingerprint() = QString::fromStdString(preparation.stockFingerprint);
    session.hasPocketSimulation() = !session.pocketSimulation().initialStock.IsNull();
    session.pocketUsesUnifiedIpw() = session.hasPocketSimulation();
    session.pocketIpwOptions() = preparation.ipwOptions;
    // The timeline resolves each family's authoritative runtime moves on demand.
    session.pocketTransientOperations().clear();
}

QString StockSimulationWorkflowController::projectSummary(
    const StockSimulationSession& session,
    const manufacturing::MachiningJob& job,
    const ProjectSimulationPreparation&,
    const IpwSimulationRunResult& result)
{
    return QCoreApplication::translate("OpenAutoPath", "加工项目 IPW：%1 道工序，执行 %2，道路检查点复用 %3，去除 %4 mm³，剩余余量 %5 mm³，过切 %6 mm³。")
        .arg(job.ipwSession().nodes().size())
        .arg(result.execution.executedNodeCount)
        .arg(result.execution.reusedCheckpointCount)
        .arg(session.pocketSimulation().removedVolume, 0, 'f', 3)
        .arg(session.pocketComparison().remainingAllowanceVolume, 0, 'f', 3)
        .arg(session.pocketComparison().overcutVolume, 0, 'f', 3);
}

PocketSimulationPreparation
StockSimulationWorkflowController::preparePocket(
    const manufacturing::MachiningJob& job,
    int selectedSetup,
    const StockSourceSettings& source,
    const QString& importedSourceFingerprint,
    int displayQuality,
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape)
{
    PocketSimulationPreparation result;
    result.selectedSetup = selectedSetup;
    if (!validateSource(source, result.error))
        return result;
    if (initialStock.IsNull() || targetShape.IsNull()) {
        result.error = QCoreApplication::translate("OpenAutoPath", "毛坯或目标实体为空。");
        return result;
    }

    std::string runtimeError;
    const auto* batch = job.currentPocketResult(&runtimeError);
    if (!batch) {
        result.error = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(runtimeError).toUtf8().constData());
        return result;
    }
    result.effectiveParams = batch->effectiveParams;
    if (!selectPocketSimulationOperations(job, *batch, selectedSetup, result))
        return result;

    if (!preparePocketStockSource(job, source, importedSourceFingerprint, result))
        return result;

    configurePocketSimulationRequest(displayQuality, initialStock, targetShape, result);
    result.ok = true;
    return result;
}

ProfileSimulationPreparation
StockSimulationWorkflowController::prepareProfile(
    const manufacturing::MachiningJob& job,
    std::span<const toolpath::ProfileToolpathOperation> previewOperations,
    int selectedSetup,
    const StockSourceSettings& source,
    const QString& importedSourceFingerprint,
    int displayQuality,
    const toolpath::ToolpathParams& effectiveParams,
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape)
{
    ProfileSimulationPreparation result;
    result.selectedSetup = selectedSetup;
    if (!validateSource(source, result.error))
        return result;
    if (initialStock.IsNull() || targetShape.IsNull()) {
        result.error = QCoreApplication::translate("OpenAutoPath", "毛坯或目标实体为空。");
        return result;
    }
    for (const toolpath::ProfileToolpathOperation& operation :
         previewOperations) {
        if (operation.setupIndex == selectedSetup)
            result.operations.push_back(operation);
    }
    if (result.operations.empty()) {
        result.error = QCoreApplication::translate("OpenAutoPath", "所选装夹面没有可仿真的外轮廓刀路工序。");
        return result;
    }

    std::vector<std::string> profileOperationIds;
    for (const manufacturing::ProfileOperationDefinition& definition :
         job.definition().profileOperations) {
        const manufacturing::SetupDefinition* setup =
            job.setupById(definition.setupId);
        if (definition.enabled && setup &&
            setup->setupIndex == selectedSetup) {
            profileOperationIds.push_back(definition.id);
        }
    }
    if (profileOperationIds.size() != result.operations.size()) {
        result.error = QCoreApplication::translate("OpenAutoPath", "所选 Setup 的 Job 外轮廓工序与当前预览不一致"
            "（Job %1，道路 %2）。请重新生成外轮廓刀路。")
            .arg(profileOperationIds.size())
            .arg(result.operations.size());
        return result;
    }

    // Profile 独立入口执行同一 Setup 中排在最后一道所选 Profile 前面的
    // 孔/槽腔节点；这样多道 Profile 之间夹有其他工序时父 IPW 仍连续。
    const auto appendPriorDefinition =
        [&](const auto& definitions, int lastProfileSequence) {
            for (const auto& definition : definitions) {
                const manufacturing::SetupDefinition* setup =
                    job.setupById(definition.setupId);
                if (definition.enabled && setup &&
                    setup->setupIndex == selectedSetup &&
                    definition.sequence < lastProfileSequence) {
                    result.includedOperationIds.push_back(definition.id);
                }
            }
        };
    int lastProfileSequence = std::numeric_limits<int>::min();
    for (const manufacturing::ProfileOperationDefinition& definition :
         job.definition().profileOperations) {
        if (std::find(profileOperationIds.begin(), profileOperationIds.end(),
                      definition.id) != profileOperationIds.end()) {
            lastProfileSequence = std::max(
                lastProfileSequence, definition.sequence);
        }
    }
    appendPriorDefinition(
        job.definition().holeOperations, lastProfileSequence);
    appendPriorDefinition(
        job.definition().pocketOperations, lastProfileSequence);
    appendPriorDefinition(
        job.definition().finishingOperations, lastProfileSequence);
    result.includedOperationIds.insert(
        result.includedOperationIds.end(),
        profileOperationIds.begin(), profileOperationIds.end());

    result.stockDefinition.marginX = source.marginX;
    result.stockDefinition.marginY = source.marginY;
    result.stockDefinition.marginZ = source.marginZ;
    if (source.sourceMode == 2) {
        if (importedSourceFingerprint.isEmpty()) {
            result.error = QCoreApplication::translate("OpenAutoPath", "无法计算毛坯 STEP 指纹。");
            return result;
        }
        result.stockFingerprint =
            importedSourceFingerprint.toStdString();
        result.stockDefinition.sourceKind =
            manufacturing::StockSourceKind::StepModel;
        result.stockDefinition.sourcePath =
            source.stepPath.toUtf8().toStdString();
    } else {
        std::ostringstream fingerprint;
        fingerprint.precision(17);
        fingerprint << "generated-profile-stock-v1|"
                    << job.definition().part.sourceFingerprint << '|'
                    << source.sourceMode << '|' << source.marginX << '|'
                    << source.marginY << '|' << source.marginZ;
        result.stockFingerprint = fingerprint.str();
        result.stockDefinition.sourceKind = source.sourceMode == 0
            ? manufacturing::StockSourceKind::PerUnitBoxes
            : manufacturing::StockSourceKind::ModelBox;
    }

    result.quality = ipwSimulationQualityProfile(
        displayQuality, effectiveParams.tool.diameter);
    result.request.initialStock = initialStock;
    result.request.targetShape = targetShape;
    result.request.initialStockFingerprint = result.stockFingerprint;
    result.request.options.includedOperationIds =
        result.includedOperationIds;
    // 外轮廓常位于新的固定轴方向；允许从父检查点派生 BRep 过渡并重新光栅化。
    result.request.options.safeMixedTopologyFallback = true;
    result.request.options.pocket.method =
        toolpath::StockSimulationMethod::HeightField;
    result.request.options.pocket.runParallel = true;
    result.request.options.pocket.zMapResolution =
        result.quality.requestedResolution;
    result.request.options.pocket.zMapMaxCellCount =
        result.quality.maximumCellCount;
    int cuttingMoveCount = 0;
    for (const toolpath::ProfileToolpathOperation& operation :
         result.operations) {
        for (const toolpath::ToolpathMove& move : operation.contour.moves) {
            if (move.type == toolpath::ToolpathMoveType::Cut ||
                move.type == toolpath::ToolpathMoveType::Plunge) {
                ++cuttingMoveCount;
            }
        }
    }
    result.request.options.pocket.maxSweepSegments =
        std::clamp(cuttingMoveCount + 16, 20000, 5000000);
    result.request.comparisonOptions.runParallel = true;
    result.ipwOptions = result.request.options.pocket;
    result.ok = true;
    return result;
}

void StockSimulationWorkflowController::installProfileResult(
    StockSimulationSession& session,
    const manufacturing::MachiningJob& job,
    const ProfileSimulationPreparation& preparation,
    IpwSimulationRunResult& result)
{
    session.pocketSimulation() = std::move(result.simulation);
    session.pocketComparison() = std::move(result.comparison);
    session.pocketOvercutAttribution() =
        std::move(result.overcutAttribution);
    session.pocketSourceFingerprint() =
        QString::fromStdString(preparation.stockFingerprint);
    session.hasPocketSimulation() =
        !session.pocketSimulation().initialStock.IsNull();
    session.pocketUsesUnifiedIpw() = session.hasPocketSimulation();
    session.pocketIpwOptions() = preparation.ipwOptions;
    session.pocketTransientOperations().clear();

    // 与槽腔安装流程一样，为每个 IPW 节点冻结一份可重放的运行时操作。
    // Profile 保持自身 Job/IPW 类型，只在材料内核边界适配为 Pocket 操作载体。
    const toolpath::ProfileToolpathBatch* runtimeBatch =
        job.profileResult();
    if (!runtimeBatch)
        return;
    const toolpath::PocketToolpathBatch* pocketBatch =
        job.pocketResult();
    const auto& nodes = job.ipwSession().nodes();
    session.pocketTransientOperations().resize(nodes.size());
    for (std::size_t nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex) {
        std::vector<toolpath::PocketToolpathOperation> captured;
        if (nodes[nodeIndex].operation.kind ==
            manufacturing::ipw::IpwOperationKind::Profile) {
            for (const toolpath::ProfileToolpathOperation& operation :
                 runtimeBatch->operations) {
                if (pocketRuntimeIdMatches(
                        nodes[nodeIndex].operation.id,
                        operation.operationId)) {
                    captured.push_back(
                        toolpath::legacyProfileSimulationOperation(operation));
                }
            }
        } else if (pocketBatch &&
                   nodes[nodeIndex].operation.kind ==
                       manufacturing::ipw::IpwOperationKind::Pocket) {
            for (const toolpath::PocketToolpathOperation& operation :
                 pocketBatch->operations) {
                if (pocketRuntimeIdMatches(
                        nodes[nodeIndex].operation.id,
                        operation.operationId)) {
                    captured.push_back(operation);
                }
            }
        }
        if (!captured.empty()) {
            session.pocketTransientOperations()[nodeIndex] =
                std::make_shared<const std::vector<
                    toolpath::PocketToolpathOperation>>(
                        std::move(captured));
        }
    }
}

QString StockSimulationWorkflowController::profileSummary(
    const StockSimulationSession& session,
    const manufacturing::MachiningJob& job,
    const ProfileSimulationPreparation& preparation,
    const IpwSimulationRunResult& result)
{
    const double actualResolution =
        session.pocketSimulation().zMapResolution;
    return QCoreApplication::translate("OpenAutoPath", "外轮廓 IPW：Setup %1，检查点 %2，Profile 工序 %3，"
        "复用 %4，去除 %5 mm³，余量 %6 mm³，过切上界 %7 mm³；"
        "XY 请求 %8 mm、实际 %9 mm。")
        .arg(preparation.selectedSetup + 1)
        .arg(job.ipwSession().nodes().size())
        .arg(preparation.operations.size())
        .arg(result.execution.reusedCheckpointCount)
        .arg(session.pocketSimulation().removedVolume, 0, 'f', 3)
        .arg(session.pocketComparison().remainingAllowanceVolume,
             0, 'f', 3)
        .arg(session.pocketComparison().overcutVolume, 0, 'f', 3)
        .arg(preparation.quality.requestedResolution, 0, 'f', 3)
        .arg(actualResolution, 0, 'f', 3);
}

// 安装腔仿真结果：移动结果入会话、刷新统一 IPW 门禁，
// 并按检查点节点捕获逐工序操作（供逐移动临时帧重放）。
void StockSimulationWorkflowController::installPocketResult(
    StockSimulationSession& session,
    const manufacturing::MachiningJob& job,
    const PocketSimulationPreparation& preparation,
    IpwSimulationRunResult& result)
{
    session.pocketSimulation() = std::move(result.simulation);
    session.pocketComparison() = std::move(result.comparison);
    session.pocketOvercutAttribution() =
        std::move(result.overcutAttribution);
    session.pocketSourceFingerprint() =
        QString::fromStdString(preparation.stockFingerprint);
    session.hasPocketSimulation() =
        !session.pocketSimulation().initialStock.IsNull();
    session.pocketUsesUnifiedIpw() = session.hasPocketSimulation();
    session.pocketIpwOptions() = preparation.pocketOptions;
    session.pocketTransientOperations().clear();

    const toolpath::PocketToolpathBatch* runtimeBatch =
        job.pocketResult();
    if (!runtimeBatch)
        return;
    const auto& nodes = job.ipwSession().nodes();
    session.pocketTransientOperations().resize(nodes.size());
    for (std::size_t nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex) {
        std::vector<toolpath::PocketToolpathOperation> captured;
        for (const toolpath::PocketToolpathOperation& operation :
             runtimeBatch->operations) {
            if (pocketRuntimeIdMatches(
                    nodes[nodeIndex].operation.id,
                    operation.operationId)) {
                captured.push_back(operation);
            }
        }
        session.pocketTransientOperations()[nodeIndex] =
            std::make_shared<const std::vector<
                toolpath::PocketToolpathOperation>>(std::move(captured));
    }
}

QString StockSimulationWorkflowController::pocketSummary(
    const StockSimulationSession& session,
    const manufacturing::MachiningJob& job,
    const PocketSimulationPreparation& preparation,
    const IpwSimulationRunResult& result)
{
    const double actualResolution =
        session.pocketSimulation().zMapResolution;
    const double planarErrorUpperBound =
        std::isfinite(actualResolution) && actualResolution > 0.0
        ? actualResolution / std::sqrt(2.0)
        : 0.0;
    QString summary = QCoreApplication::translate("OpenAutoPath", "统一 IPW：Setup %1，检查点 %2，工序 %3，"
        "复用 %4，去除 %5 mm³，余量 %6 mm³，"
        "过切上界 %7 mm³（确定 %8，边界不确定 %9）；"
        "XY 请求 %10 mm、实际 %11 mm，单元 %12，"
        "平面边界误差上界约 %13 mm；孔原生节点 %14。")
        .arg(preparation.selectedSetup + 1)
        .arg(job.ipwSession().nodes().size())
        .arg(preparation.includedOperationIds.size())
        .arg(result.execution.reusedCheckpointCount)
        .arg(session.pocketSimulation().removedVolume, 0, 'f', 3)
        .arg(session.pocketComparison().remainingAllowanceVolume,
             0, 'f', 3)
        .arg(session.pocketComparison().overcutVolume, 0, 'f', 3)
        .arg(session.pocketComparison().heightFieldCertainOvercutVolume,
             0, 'f', 3)
        .arg(session.pocketComparison()
                 .heightFieldBoundaryUncertaintyVolume,
             0, 'f', 3)
        .arg(preparation.quality.requestedResolution, 0, 'f', 3)
        .arg(actualResolution, 0, 'f', 3)
        .arg(session.pocketSimulation().zMapCellCount)
        .arg(planarErrorUpperBound, 0, 'f', 3)
        .arg(result.execution.nativeHoleNodeCount);
    summary += featureAllowanceSummary(
        session.pocketComparison().featureAllowances, 6);
    return summary;
}

// 判定运行期操作 ID 是否对应定义 ID：完全相等，或定义 ID 后
// 紧跟 "-" + 全数字后缀，或 "-r" + 全数字后缀。
bool StockSimulationWorkflowController::pocketRuntimeIdMatches(
    const std::string& definitionId,
    const std::string& runtimeId)
{
    return manufacturing::runtimeOperationIdMatches(definitionId, runtimeId);
}

} // namespace oap::app
