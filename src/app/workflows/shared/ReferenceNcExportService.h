// 参考 NC 的批量导出工作流：按装夹/刀具/阶段拆分中立程序，绑定对应
// 工件坐标系，并在写盘前生成可审查的最终文本。
#pragma once

#include <string>
#include <vector>

#include <QString>

#include "toolpath/program/GCodePostProcessor.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::app {

// 一次参考 NC 导出得到的单个文件:输出路径、该组的程序/装夹/
// 程序选项,以及写盘前生成的完整 NC 文本(供审查)。
struct ReferenceNcExportFile {
    QString outputPath;
    oap::toolpath::ToolpathProgram program;
    oap::toolpath::MachiningSetup setup;
    oap::toolpath::GCodeProgramOptions options;
    QString ncText;
};

// 导出计划:由多个分组文件组成,error 非空即整体失败。
struct ReferenceNcExportPlan {
    std::vector<ReferenceNcExportFile> files;
    std::string error;

    bool ok() const noexcept
    {
        return error.empty() && !files.empty();
    }
};

// 参考 NC 批量导出服务(纯工作流,无 UI 依赖):prepare 阶段在
// 内存中拆分并生成可审查文本,write 阶段才真正落盘。
class ReferenceNcExportService {
public:
    // 把完整程序按"装夹/刀具/阶段"拆分为多个导出组,为每组
    // 绑定对应工位坐标系并生成 NC 文本;单组时沿用所选输出路径,
    // 多组时在文件名上追加分组令牌避免覆盖。
    ReferenceNcExportPlan prepare(
        const oap::toolpath::ToolpathProgram& program,
        const oap::toolpath::ToolpathParams& params,
        const oap::toolpath::MoldSetupWorkFrames& setupWorkFrames,
        const oap::toolpath::GCodeProgramOptions& options,
        const QString& selectedOutputPath) const;

    // 把计划中的每组文本写入磁盘;任一组失败即中止并返回原因。
    bool write(const ReferenceNcExportPlan& plan,
               const oap::toolpath::ToolpathParams& params,
               std::string& error) const;
};

} // namespace oap::app
