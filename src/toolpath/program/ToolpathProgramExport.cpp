// 程序级事务导出：先完整验证和内存后处理，成功后才覆盖目标文件。
//
// 模块归属:oap_toolpath_program(GCode 后处理与程序导出)。
// 事务策略:验证失败、后处理失败、目录创建失败或写盘失败时一律返回
// false 且不触碰目标文件;只有全部成功后才以截断写覆盖,避免把半成品
// 内容残留给已存在的 NC 文件。
#include "ToolpathProgramExport.h"

#include "analysis/Utf8Path.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace oap::toolpath {
namespace {

// 打开目标文件(二进制 + 截断):先把 UTF-8 路径转为本机 filesystem 路径,
// 按需递归创建父目录,失败时返回可读原因。
bool openOutput(const std::string& outPath, std::ofstream& output,
                std::string& error)
{
    const std::filesystem::path path = oap::utf8Path(outPath);
    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            error = "cannot create output directory: " + ec.message();
            return false;
        }
    }

    output.open(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        error = "cannot open output file: " + outPath;
        return false;
    }
    return true;
}

} // namespace

// 便捷重载:以默认装夹/选项导出,内部转发到完整入口。
bool ToolpathProgramExport::writeGCode(
    const ToolpathProgram& program,
    const ToolpathParams& params,
    const std::string& outPath,
    std::string& error) const
{
    MachiningSetup setup;
    return writeGCode(program, params, setup, GCodeProgramOptions{}, outPath,
                      error);
}

bool ToolpathProgramExport::writeGCode(
    const ToolpathProgram& program,
    const ToolpathParams& params,
    const MachiningSetup& setup,
    const GCodeProgramOptions& options,
    const std::string& outPath,
    std::string& error,
    const GCodePostProcessor* postProcessor) const
{
    // 第一步:程序级验证。任一 Error 级别问题都直接终止导出。
    const ToolpathValidationReport validation =
        ToolpathProgramValidator{}.validate(program, params);
    if (!validation.ok()) {
        error = "toolpath program validation failed: " + validation.summary();
        for (const ToolpathIssue& issue : validation.issues) {
            if (issue.severity != ToolpathIssueSeverity::Error)
                continue;
            error += "; [" + issue.code + "] " + issue.message;
            break;
        }
        return false;
    }

    // 第二步:在内存缓冲区中后处理(可注入自定义后处理器)。
    // 先入内存而非直写文件,是「事务式导出」的关键:后处理失败时
    // 目标文件保持原样。
    ReferenceNcGCodePostProcessor reference;
    const GCodePostProcessor& selected = postProcessor
        ? *postProcessor : static_cast<const GCodePostProcessor&>(reference);
    std::ostringstream buffer;
    if (!selected.write(buffer, program, params, setup, options, error))
        return false;
    if (!buffer) {
        error = "post processor failed while buffering output";
        return false;
    }

    // 第三步:后处理成功后,才打开目标文件并把缓冲区内容一次性写入。
    std::ofstream output;
    if (!openOutput(outPath, output, error))
        return false;
    output << buffer.str();
    if (!output) {
        error = "cannot write output file: " + outPath;
        return false;
    }
    return true;
}

} // namespace oap::toolpath
