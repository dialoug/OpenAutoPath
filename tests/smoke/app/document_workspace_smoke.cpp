#include "app/document/ModelDocumentWorkspaceCoordinator.h"

#include <iostream>
#include <memory>
#include <string_view>

#include <QCoreApplication>
#include <QDir>
#include <QTemporaryDir>

#include <XCAFApp_Application.hxx>

namespace {

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

std::shared_ptr<AnalysisLoadResult> successfulResult(
    const QString& path)
{
    auto result = std::make_shared<AnalysisLoadResult>();
    result->ok = true;
    result->sourcePath = path;
    XCAFApp_Application::GetApplication()->NewDocument(
        "MDTV-XCAF", result->document);
    result->stepIndexer = std::make_shared<oap::StepIndexer>();
    return result;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    QTemporaryDir cacheRoot;
    if (!expect(cacheRoot.isValid(), "temporary cache root must exist"))
        return 1;
    const QString unicodeCacheRoot = cacheRoot.filePath(
        QStringLiteral("缓存_快取_𠮷_🙂"));
    if (!expect(QDir().mkpath(unicodeCacheRoot),
                "full-Unicode cache root must be creatable")) {
        return 1;
    }

    oap::app::ModelDocumentWorkspaceCoordinator workspace(
        unicodeCacheRoot.toUtf8().toStdString());
    if (!expect(QDir(unicodeCacheRoot).exists(QStringLiteral("cache")),
                "analysis cache must preserve its Unicode root")) {
        return 1;
    }

    const QString firstPath = QDir(unicodeCacheRoot).filePath(
        QStringLiteral("首个模型_繁體_𠮷_🙂.stp"));
    if (!expect(workspace.beginLoad(firstPath),
                "first load request must be accepted")) {
        return 1;
    }
    bool prepared = false;
    const oap::app::DocumentLoadCompletion installed =
        workspace.completeLoad(
            successfulResult(firstPath),
            [&prepared] { prepared = true; });
    if (!expect(
            installed.installed() && prepared && !workspace.busy() &&
                workspace.sourcePath() == firstPath,
            "successful result must replace the document atomically")) {
        return 1;
    }

    const QString secondPath = QDir(unicodeCacheRoot).filePath(
        QStringLiteral("第二模型_繁體_𠮷_🙂.stp"));
    if (!expect(workspace.beginLoad(secondPath),
                "second load request must be accepted after completion")) {
        return 1;
    }
    auto failedResult = std::make_shared<AnalysisLoadResult>();
    failedResult->sourcePath = secondPath;
    failedResult->errorMessage = QStringLiteral("expected failure");
    const oap::app::DocumentLoadCompletion failed =
        workspace.completeLoad(failedResult);
    if (!expect(
            failed.status == oap::app::DocumentLoadStatus::Failed &&
                workspace.sourcePath() == firstPath,
            "failed result must preserve the current document")) {
        return 1;
    }

    if (!expect(workspace.beginLoad(secondPath),
                "cancelled load request must start")) {
        return 1;
    }
    auto cancelledResult = std::make_shared<AnalysisLoadResult>();
    cancelledResult->sourcePath = secondPath;
    cancelledResult->cancelled = true;
    const oap::app::DocumentLoadCompletion cancelled =
        workspace.completeLoad(cancelledResult);
    if (!expect(
            cancelled.status == oap::app::DocumentLoadStatus::Cancelled &&
                workspace.sourcePath() == firstPath,
            "cancelled result must preserve the current document")) {
        return 1;
    }

    workspace.cancelAndWait();
    std::cout << "PASS: document workspace checks\n";
    return 0;
}
