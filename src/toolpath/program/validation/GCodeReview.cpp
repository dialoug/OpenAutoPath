// 模块归属:oap_toolpath_program(GCode 后处理与程序导出)的 validation 层。
// 职责:逐行解析 NC 文本,以词法级(大小写不敏感、注释剥离)检查安全
// 模态、工件坐标偏置、固定循环 R 平面与括号配对等结构性问题。审查是
// 纯文本分析,不做运动学解释;无状态,线程安全。
#include "toolpath/program/validation/GCodeReview.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string_view>

namespace oap::toolpath {
namespace {

// 大写化整行文本,使后续查找不区分大小写。
std::string upper(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char character) {
        return static_cast<char>(std::toupper(character));
    });
    return value;
}

// 剥离括号注释与分号尾部注释,返回大写后的纯代码部分,便于词法查找。
std::string codeOnly(const std::string& line)
{
    std::string result;
    result.reserve(line.size());
    bool comment = false;
    for (char character : line) {
        if (character == '(') {
            comment = true;
            continue;
        }
        if (character == ')') {
            comment = false;
            continue;
        }
        if (!comment && character != ';')
            result.push_back(character);
        else if (!comment && character == ';')
            break;
    }
    return upper(result);
}

// 词边界匹配:要求 word 两侧不是字母/数字/./-,避免 G81 命中 G810 之类
// 的长词误报。
bool hasWord(std::string_view line, std::string_view word)
{
    std::size_t position = line.find(word);
    while (position != std::string_view::npos) {
        const auto boundary = [](char character) {
            return !std::isalnum(static_cast<unsigned char>(character)) &&
                   character != '.' && character != '-';
        };
        const bool before = position == 0 || boundary(line[position - 1]);
        const std::size_t end = position + word.size();
        const bool after = end == line.size() || boundary(line[end]);
        if (before && after)
            return true;
        position = line.find(word, position + 1);
    }
    return false;
}

// 检查是否出现了任一工件坐标偏置 G54-G59。
bool hasAnyWorkOffset(std::string_view line)
{
    return hasWord(line, "G54") || hasWord(line, "G55") ||
           hasWord(line, "G56") || hasWord(line, "G57") ||
           hasWord(line, "G58") || hasWord(line, "G59");
}

// 登记一条审计问题:同步累计对应严重级别的计数并追加到报告列表。
void addIssue(GCodeReviewReport& report, GCodeReviewSeverity severity,
              int line, std::string code, std::string message)
{
    if (severity == GCodeReviewSeverity::Error)
        ++report.errorCount;
    else
        ++report.warningCount;
    report.issues.push_back(
        {severity, line, std::move(code), std::move(message)});
}

} // namespace

// 审查主流程:先把整段文本切分成行,空程序直接判 Error;随后逐行做
// 括号配对检查与模态标志收集(单位/绝对坐标/安全复位/偏置/循环 R
// 平面/程序结束),最后汇总成缺失项问题。
GCodeReviewReport GCodeReview::review(const std::string& program) const
{
    GCodeReviewReport report;
    std::istringstream input(program);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line))
        lines.push_back(line);
    report.lineCount = static_cast<int>(lines.size());
    if (lines.empty()) {
        addIssue(report, GCodeReviewSeverity::Error, 0, "empty_program",
                 "NC program is empty");
        return report;
    }

    // 各安全模态/关键元素是否出现的标志集。
    bool hasUnits = false;
    bool hasAbsolute = false;
    bool hasWorkOffset = false;
    bool hasPlane = false;
    bool hasCutterCancel = false;
    bool hasLengthCancel = false;
    bool hasCycleCancel = false;
    bool hasProgramEnd = false;
    int referenceNcWarningLine = 0;
    int parenthesisBalance = 0;

    for (std::size_t index = 0; index < lines.size(); ++index) {
        const int lineNumber = static_cast<int>(index) + 1;
        if (referenceNcWarningLine == 0 &&
            upper(lines[index]).find("REFERENCE NC - NOT FOR MACHINE EXECUTION") !=
                std::string::npos) {
            referenceNcWarningLine = lineNumber;
        }
        for (char character : lines[index]) {
            if (character == '(')
                ++parenthesisBalance;
            else if (character == ')') {
                --parenthesisBalance;
                if (parenthesisBalance < 0) {
                    addIssue(report, GCodeReviewSeverity::Error, lineNumber,
                             "comment_balance",
                             "NC comment closes without an opening parenthesis");
                    parenthesisBalance = 0;
                }
            }
        }

        // 逐行词法检查:更新模态标志;对钻削固定循环行额外校验 R 平面。
        const std::string code = codeOnly(lines[index]);
        if (code.empty())
            continue;
        hasUnits = hasUnits || hasWord(code, "G20") || hasWord(code, "G21");
        hasAbsolute = hasAbsolute || hasWord(code, "G90");
        hasWorkOffset = hasWorkOffset || hasAnyWorkOffset(code);
        hasPlane = hasPlane || hasWord(code, "G17");
        hasCutterCancel = hasCutterCancel || hasWord(code, "G40");
        hasLengthCancel = hasLengthCancel || hasWord(code, "G49");
        hasCycleCancel = hasCycleCancel || hasWord(code, "G80");
        hasProgramEnd = hasProgramEnd || hasWord(code, "M30") ||
                        hasWord(code, "M2");
        if ((hasWord(code, "G81") || hasWord(code, "G82") ||
             hasWord(code, "G83") || hasWord(code, "G73")) &&
            code.find('R') == std::string::npos) {
            addIssue(report, GCodeReviewSeverity::Error, lineNumber,
                     "cycle_missing_r",
                     "drilling cycle has no R retract plane");
        }
    }

    if (parenthesisBalance != 0) {
        addIssue(report, GCodeReviewSeverity::Error, report.lineCount,
                 "comment_balance",
                 "NC comment is missing a closing parenthesis");
    }
    if (!hasUnits)
        addIssue(report, GCodeReviewSeverity::Error, 0, "missing_units",
                 "program does not select G20 or G21 units");
    if (!hasAbsolute)
        addIssue(report, GCodeReviewSeverity::Error, 0, "missing_g90",
                 "program does not select G90 absolute coordinates");
    if (!hasProgramEnd)
        addIssue(report, GCodeReviewSeverity::Error, report.lineCount,
                 "missing_end", "program does not end with M2 or M30");
    if (referenceNcWarningLine > 0) {
        addIssue(report, GCodeReviewSeverity::Warning,
                 referenceNcWarningLine, "reference_nc_only",
                 "reference NC omits production commissioning data and must not be executed directly");
    }

    if (!hasWorkOffset)
        addIssue(report, GCodeReviewSeverity::Error, 0,
                 "missing_work_offset",
                 "reference NC does not select G54-G59");
    if (!hasPlane || !hasCutterCancel || !hasLengthCancel ||
        !hasCycleCancel) {
        addIssue(report, GCodeReviewSeverity::Error, 0,
                 "missing_safe_modal_reset",
                 "reference NC is missing G17/G40/G49/G80 safe modal reset");
    }
    return report;
}

} // namespace oap::toolpath
