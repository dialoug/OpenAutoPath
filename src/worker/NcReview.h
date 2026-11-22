#pragma once
#include <QJsonArray>
#include <QJsonObject>
#include <string>

namespace oap::worker {
QJsonObject reviewNc(const std::string &text, const QString &filename, const QString &setup,
                     const QString &tool, const QJsonArray &operations, QJsonArray warnings);
void writeNcReport(const QJsonArray &programs);
int reviewSingleNc(const QString &setup, const QString &tool, const QJsonArray &operations, QJsonArray warnings = {});
}
