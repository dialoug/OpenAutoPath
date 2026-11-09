// 所有刀路家族的通用校验边界（oap_app_workflow_cam）。
// 领域校验报告保留结构/几何检查，程序报告应用与预览/导出相同的中立程序
// 契约；两份报告都通过时候选才可安装。
#pragma once

#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"

namespace oap::app {

// Common validation boundary for every generated toolpath family. The
// service report retains domain-specific structural/geometric checks, while
// the program report applies the same neutral-program contract used by
// preview/export. A candidate is installable only when both reports pass.
struct ToolpathGenerationValidation {
    toolpath::ToolpathValidationReport service;
    toolpath::ToolpathValidationReport program;

    bool ok() const noexcept
    {
        return service.ok() && program.ok();
    }

    toolpath::ToolpathValidationReport combined() const
    {
        toolpath::ToolpathValidationReport result = service;
        result.append(program);
        return result;
    }
};

inline ToolpathGenerationValidation validateToolpathCandidate(
    const toolpath::ToolpathValidationReport& serviceValidation,
    const toolpath::ToolpathProgram& program,
    const toolpath::ToolpathParams& params)
{
    ToolpathGenerationValidation result;
    result.service = serviceValidation;
    result.program =
        toolpath::ToolpathProgramValidator().validate(program, params);
    return result;
}

} // namespace oap::app
