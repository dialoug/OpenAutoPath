// 参考 NC 后处理扩展点。几何生成器不感知输出语法。
//
// 模块归属:oap_toolpath_program(GCode 后处理与程序导出)。
// 职责:定义「把中立的 ToolpathProgram 渲染成 NC 文本」的抽象扩展点,
// 使上层(程序导出、程序审查)不关心具体 NC 语法;具体方言由
// ReferenceNcGCodePostProcessor 等实现类提供。生命周期由调用方持有,
// 无共享状态,可安全地在任意线程中构造与使用。
#pragma once

#include <iosfwd>
#include <string>

#include "toolpath/core/MachiningSetup.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathProgram.h"

namespace oap::toolpath {

// GCode 后处理器抽象基类:把中立的刀具路径程序序列化为目标 NC 方言。
// 核心不变量:只读取 program/params/setup/options,不修改任何输入;
// 所有派生实现都必须满足「成功时返回 true 且 error 为空」的契约。
class GCodePostProcessor {
public:
    virtual ~GCodePostProcessor() = default;

    // 将 program 后处理为 NC 文本写入 output 流。
    // 参数:output 目标输出流;program 待导出的中立刀路程序;
    //       params 刀具与工艺参数(如未随操作绑定刀具时的回退来源);
    //       setup 装夹与工件坐标系(世界→工件坐标变换来源);
    //       options 导出选项(程序名、刀具回退、冷却液等)。
    // 返回:true 表示成功;false 时经 error 返回可读原因,output 内容无效。
    // 前置条件:program 至少含一个操作、setup.workOffset 非 None。
    virtual bool write(
        std::ostream& output,
        const ToolpathProgram& program,
        const ToolpathParams& params,
        const MachiningSetup& setup,
        const GCodeProgramOptions& options,
        std::string& error) const = 0;

};

// 与仓库 G-code 参考文件一致的轻量 NC 格式。该格式使用 G28、M07/M09、
// G98 固定循环和尾部刀具/最低点/时间摘要；每个输出程序应只包含一个
// 装夹、一个刀具和一个孔加工阶段。
class ReferenceNcGCodePostProcessor final : public GCodePostProcessor {
public:
    bool write(std::ostream& output,
               const ToolpathProgram& program,
               const ToolpathParams& params,
               const MachiningSetup& setup,
               const GCodeProgramOptions& options,
               std::string& error) const override;
};

} // namespace oap::toolpath
