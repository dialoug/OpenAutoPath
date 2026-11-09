// 各类刀路验证器共享的稳定问题与报告模型。
#pragma once

#include <string>
#include <vector>

namespace oap::toolpath {

enum class ToolpathIssueSeverity {
    Warning,
    Error,
};

// 单条验证问题：code 是稳定机器码，message 仅供 UI/日志展示；
// operationId/moveIndex 用于定位问题来源，未绑定时为 -1/默认值。
struct ToolpathIssue {
    ToolpathIssueSeverity severity = ToolpathIssueSeverity::Warning;
    std::string code;
    std::string message;
    std::string operationId;
    int moveIndex = -1;
};

// 验证报告：按追加顺序持有问题，并缓存 Error/Warning 计数。
// ok() 仅检查 errorCount，Warning 不阻止后续流程。
struct ToolpathValidationReport {
    std::vector<ToolpathIssue> issues;
    int warningCount = 0;
    int errorCount = 0;

    bool ok() const { return errorCount == 0; }
    std::string summary() const;
    void append(ToolpathValidationReport other);
};

void addToolpathIssue(
    ToolpathValidationReport& report,
    ToolpathIssueSeverity severity,
    std::string code,
    std::string message,
    std::string operationId = {},
    int moveIndex = -1);

} // namespace oap::toolpath
