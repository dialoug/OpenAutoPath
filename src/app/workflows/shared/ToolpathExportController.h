// 导出活动预览前的与 UI 无关的准备与校验。
#pragma once

#include <vector>

#include <QString>
#include <QStringList>

#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::app {

// 导出准备结果:程序、参数与校验报告;ok 为 false 时 error 说明原因,
// referenceWarnings 携带参考 NC 特有的前置条件提示。
struct ToolpathExportPreparation {
    bool ok = false;
    QString error;
    toolpath::ToolpathProgram program;
    std::vector<toolpath::PocketToolpathOperation> pocketOperations;
    toolpath::ToolpathParams params;
    toolpath::ToolpathValidationReport validation;
    QString programKind;
    QString suggestedSuffix;
    QStringList referenceWarnings;
    bool supportsJsonl = false;
    bool holeProgram = false;
    int setupCount = 0;
};

// 导出控制器:从工作区会话读取当前激活预览(孔或槽腔),
// 组装待导出的程序并完成统一校验,全部通过才置 ok。
class ToolpathExportController {
public:
    // 按 activePreview 分支组装导出内容并校验,失败返回带错误说明
    // 的准备结果;不访问任何 UI 或场景状态。
    static ToolpathExportPreparation prepare(
        const MachiningWorkspaceSession& workspace);
};

} // namespace oap::app
