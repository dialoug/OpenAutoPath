// HoleCoverageAuditPanel 实现 — 只展示 HoleCoverageReport，不重复计算覆盖状态。
#include "app/ui/panels/HoleCoverageAuditPanel.h"

#include "toolpath/core/ToolpathDiagnostics.h"

#include <QAbstractItemView>
#include <QBrush>
#include <QColor>
#include <QHeaderView>
#include <QHash>
#include <QLabel>
#include <QPalette>
#include <QSignalBlocker>
#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>

#include <array>

namespace {
constexpr int kRoleSourceHoleIndex = Qt::UserRole + 1;
constexpr int kRoleFormalOperationId = Qt::UserRole + 2;

enum class IssueGroup : int {
    Tool = 0,
    Setup,
    Collision,
    Generation,
    Count,
};

IssueGroup issueGroup(oap::toolpath::ToolpathDiagnosticCode code)
{
    using Code = oap::toolpath::ToolpathDiagnosticCode;
    switch (code) {
        case Code::InvalidToolDiameter:
        case Code::ToolExceedsHole:
        case Code::ForcedStrategyNoClearance:
        case Code::InvalidToolAssembly:
        case Code::AutoToolLibraryEmpty:
        case Code::AutoToolUnavailable:
        case Code::AutoHoleBelowToolDiameter:
        case Code::AutoToolReachInsufficient:
            return IssueGroup::Tool;

        case Code::DuplicateHoleStageKey:
            return IssueGroup::Generation;

        case Code::HoleOpeningUnresolved:
        case Code::HoleMoldEnvelopeProjectionFailed:
        case Code::InvalidSetupAxis:
        case Code::SetupAxisMismatch:
            return IssueGroup::Setup;

        case Code::NonCuttingMoveCollision:
        case Code::CuttingMoveCollision:
        case Code::CollisionPlanningUnavailable:
        case Code::CollisionCheckFailed:
        case Code::RapidLinkCollision:
            return IssueGroup::Collision;

        default:
            return IssueGroup::Generation;
    }
}

QString issueGroupTitle(IssueGroup group)
{
    switch (group) {
        case IssueGroup::Tool: return QStringLiteral("缺刀 / 刀具问题");
        case IssueGroup::Setup: return QStringLiteral("装夹 / 入口问题");
        case IssueGroup::Collision: return QStringLiteral("碰撞问题");
        case IssueGroup::Generation: return QStringLiteral("生成阻断项");
        case IssueGroup::Count: break;
    }
    return QStringLiteral("阻断项");
}

QTreeWidgetItem* addCategory(QTreeWidget* tree,
                             const QString& label,
                             int count,
                             const QColor& color)
{
    auto* item = new QTreeWidgetItem(tree);
    item->setText(0, label);
    item->setText(1, QStringLiteral("%1 项").arg(count));
    item->setForeground(0, QBrush(color));
    item->setForeground(1, QBrush(color));
    item->setExpanded(true);
    return item;
}

void addHoleIndices(QTreeWidget* tree,
                    const QString& label,
                    const std::vector<int>& indices,
                    const QString& detail,
                    const QColor& color,
                    const QHash<int, QString>& formalOperationBySource)
{
    if (indices.empty())
        return;
    QTreeWidgetItem* category = addCategory(
        tree, label, static_cast<int>(indices.size()), color);
    for (int sourceIndex : indices) {
        auto* item = new QTreeWidgetItem(category);
        item->setText(0, QStringLiteral("孔段 %1").arg(sourceIndex + 1));
        item->setText(1, detail);
        item->setData(0, kRoleSourceHoleIndex, sourceIndex);
        item->setData(
            0, kRoleFormalOperationId,
            formalOperationBySource.value(sourceIndex));
        item->setToolTip(0, detail);
        item->setToolTip(1, detail);
    }
}

QString operationStateText(oap::manufacturing::JobOperationState state)
{
    using State = oap::manufacturing::JobOperationState;
    switch (state) {
        case State::Planned: return QStringLiteral("待生成");
        case State::Dirty: return QStringLiteral("已修改");
        case State::Generated: return QStringLiteral("已生成");
        case State::Warning: return QStringLiteral("有警告");
        case State::Error: return QStringLiteral("失败");
        case State::Stale: return QStringLiteral("下游失效");
        case State::NeedsReview: return QStringLiteral("需复核");
    }
    return QStringLiteral("未知");
}
} // namespace

HoleCoverageAuditPanel::HoleCoverageAuditPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);

    m_statusLabel = new QLabel(this);
    m_statusLabel->setObjectName(QStringLiteral("holeCoverageAuditStatus"));
    m_statusLabel->setWordWrap(true);
    m_statusLabel->setMargin(4);
    layout->addWidget(m_statusLabel);

    m_tree = new QTreeWidget(this);
    m_tree->setObjectName(QStringLiteral("holeCoverageAuditTree"));
    m_tree->setHeaderLabels(
        {QStringLiteral("审计项目"), QStringLiteral("明细")});
    m_tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_tree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_tree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tree->setUniformRowHeights(true);
    m_tree->setTextElideMode(Qt::ElideRight);
    connect(
        m_tree, &QTreeWidget::currentItemChanged, this,
        [this](QTreeWidgetItem* current, QTreeWidgetItem*) {
            if (!current)
                return;
            const QVariant sourceValue =
                current->data(0, kRoleSourceHoleIndex);
            if (sourceValue.isValid()) {
                const int sourceHoleIndex = sourceValue.toInt();
                if (sourceHoleIndex >= 0)
                    emit sourceHoleSelected(sourceHoleIndex);
            }
            const QString formalOperationId =
                current->data(0, kRoleFormalOperationId).toString();
            if (!formalOperationId.isEmpty())
                emit formalOperationSelected(formalOperationId);
        });
    layout->addWidget(m_tree, 1);

    clear();
}

void HoleCoverageAuditPanel::clear()
{
    m_tree->clear();
    m_statusLabel->setText(
        QStringLiteral("等待审计｜生成孔刀路后显示覆盖明细"));
    QPalette statusPalette = m_statusLabel->palette();
    statusPalette.setColor(
        QPalette::WindowText,
        palette().color(QPalette::Disabled, QPalette::Text));
    m_statusLabel->setPalette(statusPalette);

    auto* item = new QTreeWidgetItem(m_tree);
    item->setText(0, QStringLiteral("尚未生成孔加工审计结果"));
    item->setText(1, QStringLiteral("参考 NC 导出时将提示复核"));
}

void HoleCoverageAuditPanel::setReport(
    const oap::toolpath::HoleCoverageReport& report,
    const QVector<oap::app::HoleCoverageOperationBinding>& bindings)
{
    const QString previousFormalOperation =
        selectedFormalOperationId();
    m_tree->clear();
    if (!report.audited) {
        clear();
        return;
    }

    const bool passed = report.complete();
    const QColor passColor(26, 128, 56);
    const QColor errorColor(190, 35, 35);
    const QColor warningColor(190, 105, 0);
    const bool hasWarnings =
        !report.outsideSixFaceSourceHoleIndices.empty() ||
        !report.warningIssues.empty();
    const QColor statusColor = passed
        ? (hasWarnings ? warningColor : passColor) : errorColor;
    QHash<int, QString> formalOperationBySource;
    for (const oap::app::HoleCoverageOperationBinding& binding : bindings) {
        for (int sourceIndex : binding.sourceHoleIndices) {
            if (!formalOperationBySource.contains(sourceIndex)) {
                formalOperationBySource.insert(
                    sourceIndex, binding.formalOperationId);
            }
        }
    }

    if (passed) {
        m_statusLabel->setText(
            QStringLiteral("审计通过%1｜六面孔段覆盖 %2/%3 · 六面外警告 %4")
                .arg(hasWarnings ? QStringLiteral("（有警告）") : QString())
                .arg(report.coveredHoleSegmentCount)
                .arg(report.sixFaceHoleSegmentCount)
                .arg(static_cast<int>(
                    report.outsideSixFaceSourceHoleIndices.size())));
    } else {
        m_statusLabel->setText(
            QStringLiteral(
                "审计未通过｜漏孔 %1 · 未解析装夹 %2 · 跳过 %3 · 问题项 %4 · 六面外警告 %5")
                .arg(static_cast<int>(
                    report.uncoveredSourceHoleIndices.size()))
                .arg(static_cast<int>(
                    report.unresolvedSourceHoleIndices.size()))
                .arg(report.skippedOperationCount)
                .arg(static_cast<int>(report.blockingIssues.size()))
                .arg(static_cast<int>(
                    report.outsideSixFaceSourceHoleIndices.size())));
    }
    QPalette statusPalette = m_statusLabel->palette();
    statusPalette.setColor(QPalette::WindowText, statusColor);
    m_statusLabel->setPalette(statusPalette);

    auto* summary = new QTreeWidgetItem(m_tree);
    summary->setText(0, QStringLiteral("审计范围"));
    summary->setText(
        1, passed ? QStringLiteral("通过") : QStringLiteral("未通过"));
    summary->setForeground(1, QBrush(statusColor));
    summary->setExpanded(true);

    const std::array<std::pair<QString, QString>, 5> summaryRows{{
        {QStringLiteral("识别孔段"),
         QStringLiteral("%1").arg(report.inputHoleSegmentCount)},
        {QStringLiteral("六面加工孔段"),
         QStringLiteral("%1").arg(report.sixFaceHoleSegmentCount)},
        {QStringLiteral("物理孔"),
         QStringLiteral("%1").arg(report.physicalHoleCount)},
        {QStringLiteral("已覆盖孔段"),
         QStringLiteral("%1 / %2")
             .arg(report.coveredHoleSegmentCount)
             .arg(report.sixFaceHoleSegmentCount)},
        {QStringLiteral("跳过加工操作"),
         QStringLiteral("%1").arg(report.skippedOperationCount)},
    }};
    for (const auto& [label, value] : summaryRows) {
        auto* item = new QTreeWidgetItem(summary);
        item->setText(0, label);
        item->setText(1, value);
    }

    auto* collisionScope = new QTreeWidgetItem(m_tree);
    collisionScope->setText(0, QStringLiteral("碰撞审计范围"));
    collisionScope->setText(
        1, QStringLiteral("仅检查刀具装配与工件；夹具/机床结构未评估"));
    collisionScope->setForeground(0, QBrush(warningColor));
    collisionScope->setForeground(1, QBrush(warningColor));
    collisionScope->setToolTip(
        0, QStringLiteral("采用六面装夹假设：加工一个面时默认其余五面被夹持；该假设不阻断导出"));
    collisionScope->setToolTip(1, collisionScope->toolTip(0));

    if (!bindings.isEmpty()) {
        auto* operationRoot = new QTreeWidgetItem(m_tree);
        operationRoot->setText(0, QStringLiteral("正式孔工序归属"));
        operationRoot->setText(
            1, QStringLiteral("%1 道正式工序").arg(bindings.size()));
        operationRoot->setExpanded(true);
        for (const oap::app::HoleCoverageOperationBinding& binding :
             bindings) {
            auto* item = new QTreeWidgetItem(operationRoot);
            QStringList holes;
            for (int sourceIndex : binding.sourceHoleIndices)
                holes.push_back(QString::number(sourceIndex + 1));
            item->setText(
                0, QStringLiteral("#%1 · %2")
                       .arg(binding.sequence + 1)
                       .arg(binding.formalOperationId));
            QString state = operationStateText(binding.state);
            if (!binding.enabled)
                state += QStringLiteral("，已禁用");
            if (!binding.hasPlayback)
                state += QStringLiteral("，无可回放结果");
            item->setText(
                1, holes.isEmpty()
                       ? QStringLiteral("未绑定孔段｜%1").arg(state)
                       : QStringLiteral("孔段 %1｜%2")
                             .arg(holes.join(QStringLiteral("、")), state));
            item->setData(
                0, kRoleFormalOperationId,
                binding.formalOperationId);
            if (binding.sourceHoleIndices.size() == 1) {
                item->setData(
                    0, kRoleSourceHoleIndex,
                    binding.sourceHoleIndices.front());
            }
            item->setToolTip(0, item->text(0));
            item->setToolTip(1, item->text(1));
        }
    }

    addHoleIndices(
        m_tree, QStringLiteral("六面外孔（警告）"),
        report.outsideSixFaceSourceHoleIndices,
        QStringLiteral("当前仅处理六面加工，不参与覆盖率；参考 NC 中提示复核"),
        warningColor, formalOperationBySource);
    addHoleIndices(
        m_tree, QStringLiteral("漏孔 / 未形成去料工序"),
        report.uncoveredSourceHoleIndices,
        QStringLiteral("没有进入实际材料去除工序"), errorColor,
        formalOperationBySource);
    addHoleIndices(
        m_tree, QStringLiteral("未解析装夹"),
        report.unresolvedSourceHoleIndices,
        QStringLiteral("装夹面或孔入口未可靠确定"), errorColor,
        formalOperationBySource);

    if (report.skippedOperationCount > 0) {
        QTreeWidgetItem* skipped = addCategory(
            m_tree, QStringLiteral("跳过的加工操作"),
            report.skippedOperationCount, warningColor);
        auto* detail = new QTreeWidgetItem(skipped);
        detail->setText(0, QStringLiteral("生成阶段跳过"));
        detail->setText(
            1, QStringLiteral("%1 道操作未进入最终程序")
                   .arg(report.skippedOperationCount));
    }

    int nonScopeWarningCount = 0;
    for (const oap::toolpath::HoleCoverageIssue& issue :
         report.warningIssues) {
        if (issue.code !=
            oap::toolpath::ToolpathDiagnosticCode::HoleOutsideSixFace) {
            ++nonScopeWarningCount;
        }
    }
    QTreeWidgetItem* warningRoot = nullptr;
    if (nonScopeWarningCount > 0) {
        warningRoot = addCategory(
            m_tree, QStringLiteral("非阻断警告"),
            nonScopeWarningCount, warningColor);
    }
    for (const oap::toolpath::HoleCoverageIssue& issue :
         report.warningIssues) {
        if (!warningRoot || issue.code ==
                oap::toolpath::ToolpathDiagnosticCode::HoleOutsideSixFace) {
            continue;
        }
        auto* item = new QTreeWidgetItem(warningRoot);
        const QString reason = QString::fromUtf8(
            oap::toolpath::toolpathDiagnosticDefaultMessageZh(issue.code));
        item->setText(
            0, issue.sourceHoleIndex >= 0
                   ? QStringLiteral("孔段 %1 · %2")
                         .arg(issue.sourceHoleIndex + 1)
                         .arg(reason)
                   : reason);
        const QString code = QString::fromLatin1(
            oap::toolpath::toolpathDiagnosticCodeName(issue.code));
        const QString context = QString::fromStdString(issue.message);
        item->setText(1, context.isEmpty()
                             ? QStringLiteral("[%1]").arg(code)
                             : QStringLiteral("[%1] %2").arg(code, context));
        item->setData(0, kRoleSourceHoleIndex, issue.sourceHoleIndex);
        item->setData(
            0, kRoleFormalOperationId,
            formalOperationBySource.value(issue.sourceHoleIndex));
        item->setToolTip(0, item->text(0));
        item->setToolTip(1, item->text(1));
    }

    std::array<QTreeWidgetItem*,
               static_cast<std::size_t>(IssueGroup::Count)> issueRoots{};
    for (const oap::toolpath::HoleCoverageIssue& issue :
         report.blockingIssues) {
        const IssueGroup group = issueGroup(issue.code);
        const std::size_t groupIndex = static_cast<std::size_t>(group);
        QTreeWidgetItem*& root = issueRoots[groupIndex];
        if (!root) {
            root = new QTreeWidgetItem(m_tree);
            root->setText(0, issueGroupTitle(group));
            root->setForeground(0, QBrush(errorColor));
            root->setExpanded(true);
        }
        auto* item = new QTreeWidgetItem(root);
        const QString reason = QString::fromUtf8(
            oap::toolpath::toolpathDiagnosticDefaultMessageZh(issue.code));
        item->setText(
            0, issue.sourceHoleIndex >= 0
                   ? QStringLiteral("孔段 %1 · %2")
                         .arg(issue.sourceHoleIndex + 1)
                         .arg(reason)
                   : reason);
        const QString code = QString::fromLatin1(
            oap::toolpath::toolpathDiagnosticCodeName(issue.code));
        const QString context = QString::fromStdString(issue.message);
        item->setText(1, context.isEmpty()
                             ? QStringLiteral("[%1]").arg(code)
                             : QStringLiteral("[%1] %2").arg(code, context));
        item->setData(0, kRoleSourceHoleIndex, issue.sourceHoleIndex);
        item->setData(
            0, kRoleFormalOperationId,
            formalOperationBySource.value(issue.sourceHoleIndex));
        item->setToolTip(0, item->text(0));
        item->setToolTip(1, item->text(1));
    }
    for (QTreeWidgetItem* root : issueRoots) {
        if (root)
            root->setText(1, QStringLiteral("%1 项").arg(root->childCount()));
    }

    if (passed) {
        auto* item = new QTreeWidgetItem(m_tree);
        item->setText(
            0, QStringLiteral("六面加工范围内未发现漏孔、缺刀、跳过或装夹阻断"));
        item->setText(1, hasWarnings
                             ? QStringLiteral("覆盖审计通过，保留警告")
                             : QStringLiteral("覆盖审计通过"));
        item->setForeground(0, QBrush(statusColor));
        item->setForeground(1, QBrush(statusColor));
    } else if (report.inputHoleSegmentCount <= 0) {
        auto* item = new QTreeWidgetItem(m_tree);
        item->setText(0, QStringLiteral("没有可审计的识别孔段"));
        item->setText(1, QStringLiteral("请先完成孔识别"));
        item->setForeground(0, QBrush(errorColor));
    }
    if (!previousFormalOperation.isEmpty())
        selectFormalOperation(previousFormalOperation);
}

void HoleCoverageAuditPanel::selectFormalOperation(
    const QString& formalOperationId)
{
    if (formalOperationId.isEmpty())
        return;
    for (QTreeWidgetItemIterator it(m_tree); *it; ++it) {
        QTreeWidgetItem* item = *it;
        if (item->data(0, kRoleFormalOperationId).toString() !=
            formalOperationId) {
            continue;
        }
        const QSignalBlocker blocker(m_tree);
        m_tree->clearSelection();
        item->setSelected(true);
        m_tree->setCurrentItem(item);
        for (QTreeWidgetItem* parent = item->parent(); parent;
             parent = parent->parent()) {
            parent->setExpanded(true);
        }
        m_tree->scrollToItem(item, QAbstractItemView::EnsureVisible);
        return;
    }
    const QSignalBlocker blocker(m_tree);
    m_tree->clearSelection();
    m_tree->setCurrentItem(nullptr);
}

QString HoleCoverageAuditPanel::selectedFormalOperationId() const
{
    const QTreeWidgetItem* current = m_tree->currentItem();
    return current
        ? current->data(0, kRoleFormalOperationId).toString()
        : QString{};
}
