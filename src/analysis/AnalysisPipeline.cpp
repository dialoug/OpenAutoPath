// 模型加载用例：执行 STEP 读取、XCAF 转换和拓扑索引，
// 并把阶段状态转换为可供 UI/CLI 使用的报告。
#include "AnalysisPipeline.h"

#include <chrono>

#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDF_LabelSequence.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

namespace oap {
namespace {

using Clock = std::chrono::steady_clock;

std::int64_t elapsedMillis(Clock::time_point start)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               Clock::now() - start)
        .count();
}

} // namespace

// 流水线真实墙钟耗时，包含名称提取、预览准备与阶段间开销。
std::int64_t AnalysisPipelineReport::totalMillis() const noexcept
{
    return wallMillis;
}

AnalysisPipelineReport AnalysisPipeline::execute(
    const std::string& sourcePath,
    Handle(TDocStd_Document)& doc,
    StepIndexer& indexer,
    Model& model,
    const AnalysisPipelineControl* control) const
{
    AnalysisPipelineReport report;
    const auto pipelineStart = Clock::now();
    const auto finish = [&] { report.wallMillis = elapsedMillis(pipelineStart); return report; };
    const auto cancelled = [&] { return control && control->index.cancelled(); };
    const auto stage = [&](AnalysisStage value) {
        if (control && control->reportStage) control->reportStage(value);
    };
    if (cancelled()) { report.status = AnalysisPipelineStatus::Cancelled; return finish(); }

    // 文档由调用方最终关闭。该约定让单文件加载可将文档移交给视图，而批处理
    // 可以在每个文件结束时立即释放它。
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    app->NewDocument("MDTV-XCAF", doc);

    // XCAF 读取器同时启用颜色/名称/图层/材质/属性模式，确保装配层级与
    // 语义信息完整进入 XDE 文档，供 StepIndexer 命名与分组使用。
    STEPCAFControl_Reader reader;
    reader.SetColorMode(Standard_True);
    reader.SetNameMode(Standard_True);
    reader.SetLayerMode(Standard_True);
    reader.SetMatMode(Standard_True);
    reader.SetPropsMode(Standard_True);
    Interface_Static::SetIVal("read.stepcaf.subshapes.name", 1);

    auto stageStart = Clock::now();
    stage(AnalysisStage::Reading);
    if (reader.ReadFile(sourcePath.c_str()) != IFSelect_RetDone) {
        report.status = AnalysisPipelineStatus::ReadFailed;
        return finish();
    }
    report.readMillis = elapsedMillis(stageStart);
    if (cancelled()) { report.status = AnalysisPipelineStatus::Cancelled; return finish(); }

    stageStart = Clock::now();
    stage(AnalysisStage::Transferring);
    if (!reader.Transfer(doc)) {
        report.status = AnalysisPipelineStatus::TransferFailed;
        return finish();
    }
    report.transferMillis = elapsedMillis(stageStart);
    if (cancelled()) { report.status = AnalysisPipelineStatus::Cancelled; return finish(); }

    // 从 XDE 文档取出顶层自由形状（不含仅作为实例/装配结构的空容器），
    // 作为后续 unit 级索引的输入。
    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.IsEmpty()) {
        report.status = AnalysisPipelineStatus::NoFreeShapes;
        return finish();
    }
    report.freeShapeCount = freeShapes.Size();

    stageStart = Clock::now();
    StepIndexer::SolidNameMap solidNames =
        StepIndexer::solidNamesFromReader(reader);
    report.namesMillis = elapsedMillis(stageStart);

    if (control && control->preparePreview && !cancelled()) {
        stage(AnalysisStage::PreparingPreview);
        stageStart = Clock::now();
        control->preparePreview(doc);
        report.previewMillis = elapsedMillis(stageStart);
    }
    if (cancelled()) { report.status = AnalysisPipelineStatus::Cancelled; return finish(); }

    stage(AnalysisStage::Indexing);
    stageStart = Clock::now();
    if (!indexer.build(doc, model, &solidNames,
                       control ? &control->index : nullptr, &report.geometry)) {
        report.status = cancelled() ? AnalysisPipelineStatus::Cancelled
                                    : AnalysisPipelineStatus::NoAnalyzableShapes;
        return finish();
    }
    report.indexMillis = elapsedMillis(stageStart);

    report.unitCount = model.units.size();
    report.faceCount = model.faces.size();
    report.status = AnalysisPipelineStatus::Ok;
    return finish();
}

} // namespace oap
