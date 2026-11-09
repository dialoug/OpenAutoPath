// 从 STEP 分析工作线程移交给文档工作区的值结果。
// 由 worker 线程构造并整体经队列信号移交 UI 线程,UI 负责显示与释放。
#pragma once

#include <memory>

#include <QString>
#include <QMetaType>

#include <TDocStd_Document.hxx>

#include "analysis/Model.h"
#include "analysis/StepIndexer.h"

// 分析加载结果：状态、报告与耗时统计。
struct AnalysisLoadResult {
    quint64 requestId = 0;
    qint64 workerMillis = 0;
    bool ok = false;        // 整体是否成功
    bool fromCache = false; // 结果是否来自缓存命中
    bool cancelled = false; // 是否因取消而中止
    QString sourcePath;     // 源 STEP 文件路径
    QString errorMessage;   // 失败原因(用户可读)
    QString warningMessage; // 非致命警告(如未找到可分析实体)

    Handle(TDocStd_Document) document;            // 读取/重建的 XCAF 文档
    std::shared_ptr<oap::StepIndexer> stepIndexer; // 拓扑索引(face id <-> TopoDS 面)
    oap::Model model;                              // 分析后的领域模型
};

Q_DECLARE_METATYPE(std::shared_ptr<AnalysisLoadResult>)

// 预览仅持有独立几何，不含可用于加工的 Model 或索引。
struct AnalysisLoadPreview {
    quint64 requestId = 0;
    QString sourcePath;
    TopoDS_Shape shape;
    oap::Bbox bounds;
    qint64 elapsedMillis = 0;
    double meshDeflection = 0.0;
};
Q_DECLARE_METATYPE(std::shared_ptr<AnalysisLoadPreview>)
