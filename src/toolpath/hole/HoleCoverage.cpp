// HoleCoverage —— 孔加工覆盖审计实现。属于 oap_toolpath_hole 模块：将识别
// 出的孔段与实际上料工序比对，区分六面内/六面外/人工排除孔段，并按诊断
// 严重度拆分阻断与警告，供 Reference-NC 导出前的复核门禁使用。
#include "toolpath/hole/HoleCoverage.h"

#include <algorithm>
#include <sstream>
#include <unordered_set>

namespace oap::toolpath {
namespace {

// 判断诊断码是否为覆盖阻断码（导致孔无法被覆盖的错误类别）。
bool isCoverageBlockingCode(ToolpathDiagnosticCode code) noexcept
{
    switch (code) {
        case ToolpathDiagnosticCode::HoleOpeningUnresolved:
        case ToolpathDiagnosticCode::HoleMoldEnvelopeProjectionFailed:
        case ToolpathDiagnosticCode::InvalidHoleRadius:
        case ToolpathDiagnosticCode::InvalidHoleGeometry:
        case ToolpathDiagnosticCode::ToolExceedsHole:
        case ToolpathDiagnosticCode::ForcedStrategyNoClearance:
        case ToolpathDiagnosticCode::EmptyHoleOperation:
        case ToolpathDiagnosticCode::InvalidSetupAxis:
        case ToolpathDiagnosticCode::SetupAxisMismatch:
        case ToolpathDiagnosticCode::NonCuttingMoveCollision:
        case ToolpathDiagnosticCode::CuttingMoveCollision:
        case ToolpathDiagnosticCode::CollisionPlanningUnavailable:
        case ToolpathDiagnosticCode::CollisionCheckFailed:
        case ToolpathDiagnosticCode::RapidLinkCollision:
        case ToolpathDiagnosticCode::AutoToolLibraryEmpty:
        case ToolpathDiagnosticCode::AutoHoleBelowToolDiameter:
        case ToolpathDiagnosticCode::AutoToolReachInsufficient:
        case ToolpathDiagnosticCode::DuplicateHoleStageKey:
        case ToolpathDiagnosticCode::AlgorithmUnavailable:
        case ToolpathDiagnosticCode::AlgorithmExecutionFailed:
            return true;
        default:
            return false;
    }
}

void appendIssue(const ToolpathDiagnostic& diagnostic,
                 std::vector<HoleCoverageIssue>& issues)
{
    const auto duplicate = std::find_if(
        issues.begin(), issues.end(), [&](const HoleCoverageIssue& issue) {
            return issue.code == diagnostic.code &&
                   issue.sourceHoleIndex == diagnostic.sourceIndex &&
                   issue.message == diagnostic.message;
        });
    if (duplicate == issues.end()) {
        issues.push_back(
            {diagnostic.code, diagnostic.sourceIndex, diagnostic.message});
    }
}

// 把规划/加工诊断分流为阻断或警告：被人工排除的孔段忽略；六面外孔段和
// 刀具装配警告降级为警告；当前批次不存在任何六面内孔时，所有本应阻断的
// 诊断也降级为警告，避免无六面孔的输入被陈旧诊断误拦截。
void appendAuditDiagnostics(
    const std::vector<ToolpathDiagnostic>& diagnostics,
    const std::unordered_set<int>& outsideSixFace,
    const std::unordered_set<int>& excluded,
    bool hasSixFaceHoles,
    std::vector<HoleCoverageIssue>& blockingIssues,
    std::vector<HoleCoverageIssue>& warningIssues)
{
    for (const ToolpathDiagnostic& diagnostic : diagnostics) {
        if (diagnostic.sourceIndex >= 0 &&
            excluded.contains(diagnostic.sourceIndex)) {
            continue;
        }
        const bool outsideScope = diagnostic.sourceIndex >= 0 &&
            outsideSixFace.contains(diagnostic.sourceIndex);
        const bool assemblyGeometryWarning =
            diagnostic.code == ToolpathDiagnosticCode::InvalidToolAssembly &&
            diagnostic.severity == ToolpathDiagnosticSeverity::Warning;
        const bool normallyBlocking =
            diagnostic.severity == ToolpathDiagnosticSeverity::Error ||
            isCoverageBlockingCode(diagnostic.code);
        if (outsideScope || assemblyGeometryWarning ||
            (!hasSixFaceHoles && normallyBlocking)) {
            appendIssue(diagnostic, warningIssues);
            continue;
        }
        if (normallyBlocking)
            appendIssue(diagnostic, blockingIssues);
    }
}

} // namespace

bool holeOperationRemovesMaterial(
    const HoleMachiningOperation& operation) noexcept
{
    return operation.phase != HoleMachiningPhase::CenterSpot &&
        std::any_of(
            operation.moves.begin(), operation.moves.end(),
            [](const ToolpathMove& move) {
                return move.type == ToolpathMoveType::Cut;
            });
}

bool HoleCoverageReport::complete() const noexcept
{
    return audited && inputHoleSegmentCount > 0 &&
           coveredHoleSegmentCount == sixFaceHoleSegmentCount &&
           uncoveredSourceHoleIndices.empty() &&
           unresolvedSourceHoleIndices.empty() &&
           skippedOperationCount == 0 && blockingIssues.empty();
}

std::string HoleCoverageReport::summary() const
{
    std::ostringstream output;
    if (!audited) {
        output << "hole coverage was not audited";
        return output.str();
    }
    output << "six-face hole coverage " << coveredHoleSegmentCount << '/'
           << sixFaceHoleSegmentCount << " supported source segments"
           << ", recognized=" << inputHoleSegmentCount;
    if (physicalHoleCount > 0)
        output << ", physical holes=" << physicalHoleCount;
    if (!uncoveredSourceHoleIndices.empty())
        output << ", uncovered=" << uncoveredSourceHoleIndices.size();
    if (!unresolvedSourceHoleIndices.empty())
        output << ", unresolved=" << unresolvedSourceHoleIndices.size();
    if (!outsideSixFaceSourceHoleIndices.empty())
        output << ", outside-six-face="
               << outsideSixFaceSourceHoleIndices.size();
    if (!excludedSourceHoleIndices.empty())
        output << ", manually excluded="
               << excludedSourceHoleIndices.size();
    if (skippedOperationCount > 0)
        output << ", skipped=" << skippedOperationCount;
    if (!blockingIssues.empty())
        output << ", blocking issues=" << blockingIssues.size();
    if (!warningIssues.empty())
        output << ", warnings=" << warningIssues.size();
    return output.str();
}

HoleCoverageReport auditHoleCoverage(
    const std::vector<HoleMachiningInput>& holes,
    const std::vector<HoleMachiningOperation>& operations,
    int physicalHoleCount,
    int skippedOperationCount,
    const std::vector<ToolpathDiagnostic>& planningDiagnostics,
    const std::vector<ToolpathDiagnostic>& machiningDiagnostics,
    const std::vector<int>& excludedSourceHoleIndices)
{
    HoleCoverageReport report;
    report.audited = true;
    report.inputHoleSegmentCount = static_cast<int>(holes.size());
    const int reportedPhysicalHoleCount = std::max(0, physicalHoleCount);
    const int reportedSkippedOperationCount =
        std::max(0, skippedOperationCount);

    std::unordered_set<int> outsideSixFace;
    for (int index = 0; index < report.inputHoleSegmentCount; ++index) {
        if (holes[static_cast<std::size_t>(index)].setupFace ==
            MoldSetupFace::Unresolved) {
            outsideSixFace.insert(index);
            report.outsideSixFaceSourceHoleIndices.push_back(index);
        }
    }
    std::unordered_set<int> excluded;
    for (const int index : excludedSourceHoleIndices) {
        if (index < 0 || index >= report.inputHoleSegmentCount ||
            outsideSixFace.contains(index) ||
            !excluded.insert(index).second) {
            continue;
        }
        report.excludedSourceHoleIndices.push_back(index);
    }
    std::sort(
        report.excludedSourceHoleIndices.begin(),
        report.excludedSourceHoleIndices.end());
    const int recognizedSixFaceHoleSegmentCount =
        report.inputHoleSegmentCount - static_cast<int>(outsideSixFace.size());
    report.sixFaceHoleSegmentCount =
        recognizedSixFaceHoleSegmentCount -
        static_cast<int>(excluded.size());
    report.physicalHoleCount = std::min(
        reportedPhysicalHoleCount, recognizedSixFaceHoleSegmentCount);

    std::unordered_set<int> covered;
    for (const HoleMachiningOperation& operation : operations) {
        // 点钻或只有快移/进退刀的空壳工序都不代表材料已经去除。
        if (!holeOperationRemovesMaterial(operation) ||
            operation.sourceHoleIndex < 0) {
            continue;
        }
        if (operation.sourceHoleIndex < report.inputHoleSegmentCount &&
            !outsideSixFace.contains(operation.sourceHoleIndex) &&
            !excluded.contains(operation.sourceHoleIndex)) {
            covered.insert(operation.sourceHoleIndex);
        }
        for (const int sourceIndex : operation.coveredSourceHoleIndices) {
            if (sourceIndex >= 0 &&
                sourceIndex < report.inputHoleSegmentCount &&
                !outsideSixFace.contains(sourceIndex) &&
                !excluded.contains(sourceIndex)) {
                covered.insert(sourceIndex);
            }
        }
    }

    for (int index = 0; index < report.inputHoleSegmentCount; ++index) {
        if (outsideSixFace.contains(index) || excluded.contains(index))
            continue;
        if (holes[static_cast<std::size_t>(index)].entryResolution ==
                HoleEntryResolution::Unresolved) {
            report.unresolvedSourceHoleIndices.push_back(index);
        }
        if (!covered.contains(index))
            report.uncoveredSourceHoleIndices.push_back(index);
    }
    report.coveredHoleSegmentCount = static_cast<int>(covered.size());
    // 生成器的旧统计只有总跳过数，无法区分六面外过滤和六面内失败。
    // 最终门禁以六面内未覆盖孔段为准：可被其他有效工序覆盖的内部尝试、
    // 以及六面外孔的跳过，都不再制造独立阻断。
    report.skippedOperationCount = std::min(
        reportedSkippedOperationCount,
        static_cast<int>(report.uncoveredSourceHoleIndices.size()));
    const bool hasSixFaceHoles = report.sixFaceHoleSegmentCount > 0;
    appendAuditDiagnostics(
        planningDiagnostics, outsideSixFace, excluded, hasSixFaceHoles,
        report.blockingIssues, report.warningIssues);
    appendAuditDiagnostics(
        machiningDiagnostics, outsideSixFace, excluded, hasSixFaceHoles,
        report.blockingIssues, report.warningIssues);

    for (int index : report.outsideSixFaceSourceHoleIndices) {
        const auto found = std::find_if(
            report.warningIssues.begin(), report.warningIssues.end(),
            [index](const HoleCoverageIssue& issue) {
                return issue.code == ToolpathDiagnosticCode::HoleOutsideSixFace &&
                       issue.sourceHoleIndex == index;
            });
        if (found == report.warningIssues.end()) {
            report.warningIssues.push_back(
                {ToolpathDiagnosticCode::HoleOutsideSixFace, index,
                 "hole is outside the supported six-face setups"});
        }
    }
    return report;
}

} // namespace oap::toolpath
