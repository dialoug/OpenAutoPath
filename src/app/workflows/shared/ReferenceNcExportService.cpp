// 参考 NC 批次的纯工作流实现，不访问 MainWindow、对话框或场景状态。
#include "app/workflows/shared/ReferenceNcExportService.h"

#include <sstream>

#include <QDir>
#include <QFileInfo>

#include "toolpath/program/ToolpathProgramExport.h"

namespace oap::app {
namespace {

// 装夹面到文件令牌的映射:未解析时退化为 setup 序号,保证唯一。
QString setupFaceToken(oap::MoldSetupFace face, int setupIndex)
{
    switch (face) {
        case oap::MoldSetupFace::PositiveZ: return QStringLiteral("posZ");
        case oap::MoldSetupFace::PositiveX: return QStringLiteral("posX");
        case oap::MoldSetupFace::PositiveY: return QStringLiteral("posY");
        case oap::MoldSetupFace::NegativeX: return QStringLiteral("negX");
        case oap::MoldSetupFace::NegativeY: return QStringLiteral("negY");
        case oap::MoldSetupFace::NegativeZ: return QStringLiteral("negZ");
        case oap::MoldSetupFace::Unresolved:
            return QStringLiteral("setup%1").arg(setupIndex + 1);
    }
    return QStringLiteral("setup%1").arg(setupIndex + 1);
}

// 分组令牌:装夹序号_装夹面_刀号(或 unbound)_阶段,作为多文件
// 导出的文件名后缀,保证同一程序拆分后各文件互不冲突。
QString exportGroupToken(const oap::toolpath::ToolpathOperation& operation)
{
    const QString toolToken =
        operation.tool.valid && operation.tool.toolNumber > 0
        ? QStringLiteral("T%1").arg(operation.tool.toolNumber)
        : QStringLiteral("unbound");
    return QStringLiteral("setup%1_%2_%3_%4")
        .arg(operation.setupIndex + 1)
        .arg(setupFaceToken(operation.setupFace, operation.setupIndex))
        .arg(toolToken)
        .arg(QString::fromLatin1(
            oap::toolpath::holeMachiningPhaseName(operation.holePhase)));
}

QString splitOutputPath(
    const QString& selectedPath,
    const oap::toolpath::ToolpathOperation& operation)
{
    const QFileInfo info(selectedPath);
    const QString extension = info.suffix().isEmpty()
        ? QStringLiteral("nc") : info.suffix();
    return QDir(info.absolutePath()).filePath(
        QStringLiteral("%1_%2.%3")
            .arg(info.completeBaseName(), exportGroupToken(operation),
                 extension));
}

} // namespace

// prepare 阶段为纯内存操作:拆分、绑定工位、生成 NC 文本,
// 任何校验失败都会清空已收集的文件并返回错误原因。
ReferenceNcExportPlan ReferenceNcExportService::prepare(
    const oap::toolpath::ToolpathProgram& program,
    const oap::toolpath::ToolpathParams& params,
    const oap::toolpath::MoldSetupWorkFrames& setupWorkFrames,
    const oap::toolpath::GCodeProgramOptions& options,
    const QString& selectedOutputPath) const
{
    ReferenceNcExportPlan result;
    const std::vector<oap::toolpath::ToolpathProgram> exportGroups =
        oap::toolpath::splitToolpathProgramBySetupToolAndPhase(program);
    if (exportGroups.empty()) {
        result.error = "reference NC export has no program groups";
        return result;
    }

    result.files.reserve(exportGroups.size());
    oap::toolpath::ReferenceNcGCodePostProcessor postProcessor;
    for (const oap::toolpath::ToolpathProgram& exportGroup : exportGroups) {
        if (exportGroup.operations.empty()) {
            result.error = "reference NC export group contains no operations";
            result.files.clear();
            return result;
        }

        const oap::toolpath::ToolpathOperation& firstOperation =
            exportGroup.operations.front();
        if (firstOperation.setupFace == oap::MoldSetupFace::Unresolved) {
            result.error = "toolpath setup direction is unresolved";
            result.files.clear();
            return result;
        }
        const oap::toolpath::MoldFaceSetupWorkFrame* setupFrame =
            setupWorkFrames.forFace(firstOperation.setupFace);
        if (!setupFrame || !setupFrame->valid) {
            result.error =
                "toolpath setup direction has no valid independent work frame";
            result.files.clear();
            return result;
        }

        ReferenceNcExportFile file;
        // 多组导出必须拆分文件名;程序名也追加分组令牌,
        // 避免后处理中的程序名冲突。
        file.outputPath = exportGroups.size() > 1
            ? splitOutputPath(selectedOutputPath, firstOperation)
            : selectedOutputPath;
        file.program = exportGroup;
        file.setup = setupFrame->machiningSetup();
        file.options = options;
        if (exportGroups.size() > 1) {
            file.options.programName += "_" +
                exportGroupToken(firstOperation).toStdString();
        }

        std::ostringstream buffer;
        if (!postProcessor.write(
                buffer, file.program, params, file.setup, file.options,
                result.error)) {
            result.files.clear();
            return result;
        }
        file.ncText = QString::fromStdString(buffer.str());
        result.files.push_back(std::move(file));
    }
    return result;
}

// write 阶段:逐个写盘;文件级失败(权限/路径等)立即返回 false,
// 由调用方决定是否清理已写出的文件。
bool ReferenceNcExportService::write(
    const ReferenceNcExportPlan& plan,
    const oap::toolpath::ToolpathParams& params,
    std::string& error) const
{
    error.clear();
    if (!plan.ok()) {
        error = plan.error.empty()
            ? "reference NC export plan is empty" : plan.error;
        return false;
    }

    oap::toolpath::ReferenceNcGCodePostProcessor postProcessor;
    for (const ReferenceNcExportFile& file : plan.files) {
        if (!oap::toolpath::ToolpathProgramExport{}.writeGCode(
                file.program, params, file.setup, file.options,
                file.outputPath.toUtf8().toStdString(), error,
                &postProcessor)) {
            return false;
        }
    }
    return true;
}

} // namespace oap::app
