// PocketToolpathExport —— 已生成腔清移动的序列化边界。
// 导出过程不会重新生成几何，只写出 PocketClearingResult 中已经验证的
// 移动顺序和进给值。
//
// 属于 oap_toolpath_pocket 的程序导出层：JSONL 由本文件直写，
// G-code 统一委托 toolpath/program/ToolpathProgramExport 的事务式
// 导出流程（经 PocketProgramAdapter 投影后的中立程序）。
#pragma once

#include <string>
#include <vector>

#include "toolpath/program/GCodePostProcessor.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"

namespace oap::toolpath {

// 槽腔刀路导出：把槽腔工序导出为程序/NC。
class PocketToolpathExport {
public:
    // outPath 采用 UTF-8。JSONL 保留操作和移动元数据，供诊断及下游工具使用。
    // 仅在成功打开输出文件后才替换已有文件。
    bool writeJsonl(const std::vector<PocketToolpathOperation>& operations,
                    const ToolpathParams& params,
                    const std::string& outPath,
                    std::string& error) const;

    // 输出采用毫米制和绝对坐标的最小 G 代码程序（G0/G1）。
    bool writeGCode(const std::vector<PocketToolpathOperation>& operations,
                    const ToolpathParams& params,
                    const std::string& outPath,
                    std::string& error) const;

    // 带安装环境与后处理器的完整 G-code 导出：setup 提供工件坐标系/原点，
    // options 控制程序头尾格式，postProcessor 非空时在写出前做后处理转换。
    bool writeGCode(const std::vector<PocketToolpathOperation>& operations,
                    const ToolpathParams& params,
                    const MachiningSetup& setup,
                    const GCodeProgramOptions& options,
                    const std::string& outPath,
                    std::string& error,
                    const GCodePostProcessor* postProcessor = nullptr) const;
};

} // namespace oap::toolpath
