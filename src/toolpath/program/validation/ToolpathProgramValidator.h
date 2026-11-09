// 仅依赖中立 ToolpathProgram 的程序级验证器。
//
// 模块归属:oap_toolpath_program(GCode 后处理与程序导出)的 validation 层。
// 职责:在不依赖任何生成器内部类型的前提下,校验整个刀具路径程序
// (工序 ID 唯一性、装夹一致性、刀具上下文与逐移动/原语结构)。
// 供程序导出与批执行前做统一把关。无状态,线程安全。
#pragma once

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

// 程序级验证器:聚合参数/移动/原语/刀具等子验证器,输出统一报告。
class ToolpathProgramValidator {
public:
    // 校验整个程序,聚合所有问题(不因首个错误而中断)。
    // 参数:program 待校验的中立刀路程序;params 工艺参数。
    // 返回:含全部 issues 与 ok() 判定的报告。
    ToolpathValidationReport validate(
        const ToolpathProgram& program,
        const ToolpathParams& params) const;
};

} // namespace oap::toolpath
