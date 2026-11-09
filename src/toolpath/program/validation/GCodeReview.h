// 参考 NC 文本的结构审计。该检查不解释控制器运动学，只负责在文件写出前
// 发现缺少安全模态、工件坐标、固定循环 R 平面和程序结束等确定性问题。
//
// 模块归属:oap_toolpath_program(GCode 后处理与程序导出)的 validation 层。
// 职责:对纯文本 NC 程序做纯结构检查(单位/绝对坐标/安全模态复位/
// 固定循环 R 平面/注释括号配对/程序结束等),不依赖任何控制器模型。
// 线程安全:本类无状态,review() 可并发调用。
#pragma once

#include <string>
#include <vector>

namespace oap::toolpath {

// 审计问题严重级别:Error 视为程序不可用(使报告 ok() 为假),
// Warning 仅作提示(如参考 NC 不可上机的说明)。
enum class GCodeReviewSeverity {
    Warning,
    Error,
};

// 单条审计结论:严重级别 + 行号(1-based,0 表示整份程序级)+ 机器可读
// 的稳定代码与给人读的消息,便于调用方分类处理。
struct GCodeReviewIssue {
    GCodeReviewSeverity severity = GCodeReviewSeverity::Warning;
    int line = 0; // 1-based；0 表示整份程序。
    std::string code;
    std::string message;
};

// 整份程序的审计报告:总行数、两类问题计数与问题列表。
// ok() 约定:存在任何 Error 即视为程序不合格。
struct GCodeReviewReport {
    int lineCount = 0;
    int warningCount = 0;
    int errorCount = 0;
    std::vector<GCodeReviewIssue> issues;

    bool ok() const noexcept { return errorCount == 0; }
};

// 参考 NC 文本的结构审查器。
class GCodeReview {
public:
    // 审查整段 NC 程序文本(按行解析),返回结构化报告。
    // 参数:program 待审查的 NC 文本。
    // 返回:report 包含行数、问题列表与 ok() 判定,不会抛异常。
    GCodeReviewReport review(const std::string& program) const;
};

} // namespace oap::toolpath
