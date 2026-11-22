#include "NcReview.h"
#include "toolpath/program/validation/GCodeReview.h"
#include <QFile>
#include <QJsonDocument>
#include <QSaveFile>
#include <stdexcept>

namespace oap::worker {
QJsonObject reviewNc(const std::string &text, const QString &filename, const QString &setup,
                     const QString &tool, const QJsonArray &operations, QJsonArray warnings)
{
    const auto report = toolpath::GCodeReview().review(text);
    QJsonArray errors;
    for (const auto &issue : report.issues) {
        auto &rows = issue.severity == toolpath::GCodeReviewSeverity::Error ? errors : warnings;
        rows.append(QJsonObject{{"code", QString::fromStdString(issue.code)}, {"line", issue.line},
                                {"message", QString::fromStdString(issue.message)}});
    }
    return {{"filename", filename}, {"setup", setup}, {"tool", tool}, {"operations", operations},
            {"lines", report.lineCount}, {"warnings", warnings}, {"errors", errors}, {"text", QString::fromStdString(text)}};
}

void writeNcReport(const QJsonArray &programs)
{
    QSaveFile file("programs.json");
    const auto bytes = QJsonDocument(programs).toJson(QJsonDocument::Compact);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
        throw std::runtime_error("无法保存 NC 审查结果");
}

int reviewSingleNc(const QString &setup, const QString &tool, const QJsonArray &operations, QJsonArray warnings)
{
    QFile file("reference.nc");
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("无法读取工序参考 NC");
    const auto text = file.readAll();
    if (file.error() != QFileDevice::NoError)
        throw std::runtime_error("读取工序参考 NC 失败");
    const auto program = reviewNc(text.toStdString(), "reference.nc", setup, tool, operations, warnings);
    writeNcReport({program});
    return static_cast<int>(program["errors"].toArray().size());
}
}
