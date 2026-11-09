// 程序级刀路的文件导出边界，供 Pocket、Hole 及后续工序共享。
//
// 模块归属:oap_toolpath_program(GCode 后处理与程序导出)。
// 职责:作为「程序 → NC 文件」的单一出口,依次完成程序级验证、内存
// 后处理、目标文件写入,并对所有失败路径给出可读错误。本身无状态,
// 线程安全;文件路径统一采用 UTF-8 编码。
#pragma once

#include <string>

#include "GCodePostProcessor.h"
#include "toolpath/core/ToolpathParams.h"

namespace oap::toolpath {

// 程序导出器:把 ToolpathProgram 经后处理器写入磁盘文件。
// 核心约定:成功后目标文件才被覆盖,失败时绝不留下半成品文件。
class ToolpathProgramExport {
public:
    // outPath 采用 UTF-8。先完成程序级验证和内存后处理，再打开目标文件，避免失败导出截断
    // 已存在的 NC 文件。
    // 便捷重载:使用空装夹配置与默认导出选项。
    // 参数:program 待导出的刀路程序;params 工艺参数;outPath 输出
    //      文件路径(UTF-8);error 失败原因输出。
    // 返回:true 表示文件已成功写入。
    bool writeGCode(const ToolpathProgram& program,
                    const ToolpathParams& params,
                    const std::string& outPath,
                    std::string& error) const;

    // 完整导出入口:先做程序级验证,再在内存中后处理,最后写入文件。
    // 参数:setup 装夹配置(工件坐标变换、G54-G59 偏置来源);
    //       options 导出选项;postProcessor 可注入自定义后处理器,
    //       为空时使用内置 ReferenceNcGCodePostProcessor。
    // 返回:true 表示文件已成功写入;false 时 error 给出失败原因。
    bool writeGCode(const ToolpathProgram& program,
                    const ToolpathParams& params,
                    const MachiningSetup& setup,
                    const GCodeProgramOptions& options,
                    const std::string& outPath,
                    std::string& error,
                    const GCodePostProcessor* postProcessor = nullptr) const;
};

} // namespace oap::toolpath
