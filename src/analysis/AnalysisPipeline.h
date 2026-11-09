// AnalysisPipeline —— STEP/XCAF 加载和分析领域流程。
// 不依赖 Qt 或界面对象；调用方负责线程调度、缓存策略和用户提示。
// 属于 oap_analysis 模块，执行 STEP 读取、XCAF 转换和模型拓扑索引。
// 生命周期约定：XDE 文档由调用方创建并传入，失败后也由调用方关闭释放，
// 本类不持有任何资源。
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <TDocStd_Document.hxx>

#include "Model.h"
#include "StepIndexer.h"

namespace oap {

// 流水线执行结果状态；Ok 以外的值表示某阶段失败，
// 调用方可通过 report 中的计数字段判断已加载到哪一步。
enum class AnalysisPipelineStatus {
    Ok,
    ReadFailed,
    TransferFailed,
    NoFreeShapes,
    NoAnalyzableShapes,
    Cancelled,
};

enum class AnalysisStage { Reading, Transferring, PreparingPreview, Indexing };
struct AnalysisPipelineControl {
    StepIndexBuildControl index;
    std::function<void(AnalysisStage)> reportStage;
    // 同步回调；预览发布前必须深拷贝几何，不能把此文档交给渲染线程。
    std::function<void(const Handle(TDocStd_Document)&)> preparePreview;
};

// 保留加载阶段的规模和耗时信息，使 UI/CLI 可按自己的日志框架展示诊断。
struct AnalysisPipelineReport {
    AnalysisPipelineStatus status = AnalysisPipelineStatus::ReadFailed;
    int freeShapeCount = 0;
    std::size_t unitCount = 0;
    std::size_t faceCount = 0;
    std::int64_t readMillis = 0;
    std::int64_t transferMillis = 0;
    std::int64_t indexMillis = 0;
    std::int64_t namesMillis = 0;
    std::int64_t previewMillis = 0;
    std::int64_t wallMillis = 0;
    StepIndexBuildStatistics geometry;

    bool ok() const noexcept { return status == AnalysisPipelineStatus::Ok; }
    std::int64_t totalMillis() const noexcept;
};

// STEP/XCAF 加载与模型索引的领域流水线编排（文件头有完整生命周期说明）。
class AnalysisPipeline {
public:
    // sourcePath 必须是 UTF-8。创建 XDE 文档、读取 STEP、构建索引完成模型加载。调用方在失败后
    // 仍须关闭非空的 doc；这样可由 UI worker 和批处理工具统一管理文档生命周期。
    AnalysisPipelineReport execute(const std::string& sourcePath,
                                   Handle(TDocStd_Document)& doc,
                                   StepIndexer& indexer,
                                   Model& model,
                                   const AnalysisPipelineControl* control = nullptr) const;
};

} // namespace oap
