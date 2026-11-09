// 验证报告的汇总、合并和统一问题追加实现。
#include "toolpath/core/validation/ToolpathValidationTypes.h"

#include <iterator>
#include <sstream>
#include <utility>

namespace oap::toolpath {

std::string ToolpathValidationReport::summary() const
{
    std::ostringstream output;
    output << errorCount << " error(s), "
           << warningCount << " warning(s)";
    return output.str();
}

// 追加另一份报告的问题与计数，保持相对顺序；用于并行分块后的合并。
void ToolpathValidationReport::append(ToolpathValidationReport other)
{
    warningCount += other.warningCount;
    errorCount += other.errorCount;
    issues.insert(
        issues.end(),
        std::make_move_iterator(other.issues.begin()),
        std::make_move_iterator(other.issues.end()));
}

void addToolpathIssue(
    ToolpathValidationReport& report,
    ToolpathIssueSeverity severity,
    std::string code,
    std::string message,
    std::string operationId,
    int moveIndex)
{
    ToolpathIssue issue;
    issue.severity = severity;
    issue.code = std::move(code);
    issue.message = std::move(message);
    issue.operationId = std::move(operationId);
    issue.moveIndex = moveIndex;
    report.issues.push_back(std::move(issue));

    if (severity == ToolpathIssueSeverity::Error)
        ++report.errorCount;
    else
        ++report.warningCount;
}

} // namespace oap::toolpath
