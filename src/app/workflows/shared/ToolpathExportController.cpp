// ToolpathExportController 实现（oap_app_workflow_base）。
// 导出活动预览前的与 UI 无关的准备与校验，见 ToolpathExportController.h。
#include "app/workflows/shared/ToolpathExportController.h"
#include <QCoreApplication>

#include "toolpath/workflow/ToolpathProgramAdapters.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"

namespace oap::app {

// 活动预览选择导出领域；槽腔数据取自 Job 完整结果，不受显示过滤影响。
ToolpathExportPreparation ToolpathExportController::prepare(
    const MachiningWorkspaceSession& workspace)
{
    ToolpathExportPreparation result;
    const auto family = workspace.activePreview() == ActiveToolpathPreview::None
        ? ActiveToolpathPreview::Hole : workspace.activePreview();
    if (workspace.toolpathNeedsRegeneration(family)) {
        result.error = QCoreApplication::translate("OpenAutoPath", "当前加工参数已变更，请成功重新生成刀路后再导出。");
        return result;
    }
    if (workspace.activePreview() == ActiveToolpathPreview::SurfaceFinishing) {
        std::string error;
        const auto* batch = workspace.job().currentFinishingResult(&error);
        if (!batch) {
            result.error = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(error).toUtf8().constData());
            return result;
        }
        result.program = batch->toProgram();
        result.params = batch->effectiveParams;
        result.programKind = QCoreApplication::translate("OpenAutoPath", "雕刻加工");
        result.suggestedSuffix = QStringLiteral("_engraving.nc");
        result.referenceWarnings << QCoreApplication::translate("OpenAutoPath", "雕刻加工仅完成采样刀刃接触计算；IPW减料不等于连续刀柄、夹具碰撞验证。");
    } else if (workspace.activePreview() == ActiveToolpathPreview::Profile) {
        if (!workspace.hasProfilePreviewParams() ||
            workspace.profilePreviewOperations().empty()) {
            result.error = QCoreApplication::translate("OpenAutoPath", "当前外轮廓预览没有可导出的移动。");
            return result;
        }
        result.program = toolpath::makeToolpathProgram(
            workspace.profilePreviewOperations());
        result.params = workspace.profilePreviewEffectiveParams();
        result.programKind = QCoreApplication::translate("OpenAutoPath", "外轮廓");
        result.suggestedSuffix = QStringLiteral("_profile_toolpath.nc");
    } else if (workspace.activePreview() == ActiveToolpathPreview::Pocket) {
        const auto& job = workspace.job();
        std::string error;
        const auto* pockets = job.currentPocketResult(&error);
        if (!pockets) {
            result.error = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(error).toUtf8().constData());
            return result;
        }
        result.pocketOperations = pockets->operations;
        result.program = pockets->toProgram();
        result.params = pockets->effectiveParams;
        result.programKind = QCoreApplication::translate("OpenAutoPath", "槽/腔");
        result.suggestedSuffix = QStringLiteral("_pocket_toolpath.nc");
        result.supportsJsonl = true;
    } else {
        const toolpath::HoleToolpathBatch* holes =
            workspace.job().holeResult();
        if (!workspace.hasHolePreviewParams() || !holes) {
            result.error =
                QCoreApplication::translate("OpenAutoPath", "当前孔加工预览没有可导出的移动。");
            return result;
        }
        // 参考 NC 不等于生产放行:把未完成的审计项列为警告,
        // 而不是阻止导出。
        if (!holes->coverage.complete()) {
            result.referenceWarnings << QCoreApplication::translate("OpenAutoPath", "孔覆盖审计未完成；参考文件只包含当前已生成工序。");
        }
        result.referenceWarnings << QCoreApplication::translate("OpenAutoPath", "参考格式不要求零件文件指纹和当前加工任务一致。");
        if (!holes->productionSafety.complete()) {
            result.referenceWarnings << QCoreApplication::translate("OpenAutoPath", "刀具装配实体扫掠审计未完成；参考文件不可据此视为已通过生产碰撞验证。");
        }
        result.referenceWarnings << QCoreApplication::translate("OpenAutoPath", "参考格式不要求先完成毛坯减料审计。");
        result.program = holes->toProgram();
        result.params = holes->effectiveParams;
        result.programKind = QCoreApplication::translate("OpenAutoPath", "孔加工");
        result.suggestedSuffix = QStringLiteral("_hole_toolpath.nc");
        result.holeProgram = true;
    }

    // 统一校验:任何 Error 级问题都视为导出失败,仅返回摘要文本。
    result.setupCount = static_cast<int>(
        toolpath::splitToolpathProgramBySetup(result.program).size());
    result.validation = toolpath::ToolpathProgramValidator{}.validate(
        result.program, result.params);
    if (!result.validation.ok()) {
        result.error = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(
            result.validation.summary()).toUtf8().constData());
        return result;
    }
    result.ok = true;
    return result;
}

} // namespace oap::app
