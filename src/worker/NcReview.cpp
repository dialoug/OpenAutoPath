#include "support/Files.h"
#include "support/Json.h"
#include <string>
#include "NcReview.h"
#include "toolpath/program/validation/GCodeReview.h"
#include <stdexcept>

namespace oap::worker {
oap::JsonObject reviewNc(const std::string &text, const std::string &filename, const std::string &setup,
                     const std::string &tool, const oap::JsonArray &operations, oap::JsonArray warnings)
{
    const auto report = toolpath::GCodeReview().review(text);
    oap::JsonArray errors;
    for (const auto &issue : report.issues) {
        auto &rows = issue.severity == toolpath::GCodeReviewSeverity::Error ? errors : warnings;
        rows.append(oap::JsonObject{{"code", issue.code}, {"line", issue.line},
                                {"message", issue.message}});
    }
    return {{"filename", filename}, {"setup", setup}, {"tool", tool}, {"operations", operations},
            {"lines", report.lineCount}, {"warnings", warnings}, {"errors", errors}, {"text", text}};
}

void writeNcReport(const oap::JsonArray &programs)
{
    const auto bytes = oap::JsonDocument(programs).toJson(oap::JsonDocument::Compact);
    if (!oap::writeFileAtomically("programs.json", bytes))
        throw std::runtime_error("无法保存 NC 审查结果");
}

int reviewSingleNc(const std::string &setup, const std::string &tool, const oap::JsonArray &operations, oap::JsonArray warnings)
{
    std::string text;
    if (!oap::readFile("reference.nc", text))
        throw std::runtime_error("无法读取工序参考 NC");
    const auto program = reviewNc(text, "reference.nc", setup, tool, operations, warnings);
    writeNcReport({program});
    return static_cast<int>(program["errors"].toArray().size());
}
}
