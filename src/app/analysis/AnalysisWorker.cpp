// AnalysisWorker 实现 —— 线程调度、缓存策略和 Qt 提示留在 app；
// STEP 读取与领域分析由 analysis::AnalysisPipeline 执行。
#include "AnalysisWorker.h"
#include <QCoreApplication>
#include "StepAnalysisCache.h"

#include "analysis/AnalysisPipeline.h"
#include "toolpath/core/ToolpathParallel.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <QDebug>
#include <QElapsedTimer>
#include <cmath>

#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <XCAFApp_Application.hxx>

namespace {

// 加载复用进程预算，最多四个工作槽；没有可借槽时原线程串行执行。
oap::AnalysisPipelineControl loadControl(const std::function<bool()>& cancelled)
{
    oap::AnalysisPipelineControl control;
    control.index.isCancellationRequested = cancelled;
    control.index.executeRange = [](std::size_t count, const std::function<void(std::size_t)>& task) {
        const auto workers = std::min<std::size_t>({4, count, oap::toolpath::toolpathHardwareParallelism()});
        oap::toolpath::ToolpathParallelLease lease(nullptr, workers > 0 ? workers-1 : 0,
                                                 workers > 0 ? workers-1 : 0);
        oap::toolpath::toolpathParallelForWorkers(count, lease.parallelism(), [&](std::size_t index) {
            OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
            task(index);
        });
    };
    return control;
}

// 只为复杂模型创建预览；UI 只收到独立的 TShape/几何，不共享 worker 文档。
std::shared_ptr<AnalysisLoadPreview> makePreview(const Handle(TDocStd_Document)& document)
{
    auto tool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    TDF_LabelSequence labels;
    tool->GetFreeShapes(labels);
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    int complex = 0;
    for (const auto& label : labels) {
        const auto shape = tool->GetShape(label);
        if (shape.IsNull()) continue;
        builder.Add(compound, shape);
        for (TopExp_Explorer face(shape, TopAbs_FACE); face.More() && complex < 4; face.Next()) {
            const auto type = BRepAdaptor_Surface(TopoDS::Face(face.Current())).GetType();
            if (type == GeomAbs_BSplineSurface || type == GeomAbs_BezierSurface ||
                type == GeomAbs_SurfaceOfRevolution || type == GeomAbs_SurfaceOfExtrusion ||
                type == GeomAbs_OtherSurface) ++complex;
        }
    }
    if (complex < 4) return {};
    auto result = std::make_shared<AnalysisLoadPreview>();
    result->shape = BRepBuilderAPI_Copy(compound, true, false).Shape();
    Bnd_Box bounds;
    BRepBndLib::Add(result->shape, bounds, false);
    if (bounds.IsVoid()) return {};
    auto& box = result->bounds;
    bounds.Get(box.xmin,box.ymin,box.zmin,box.xmax,box.ymax,box.zmax);
    const auto size = box.size();
    result->meshDeflection = std::sqrt(size[0]*size[0]+size[1]*size[1]+size[2]*size[2])*0.001;
    // 在后台预剖分，发布后不再触碰此几何；避免 UI 预览启动另一套并行池。
    if (result->meshDeflection > 0)
        BRepMesh_IncrementalMesh(result->shape, result->meshDeflection, false, 0.5, false);
    return result;
}
// 把流水线状态码映射为用户可读的错误 / 警告消息(写进出参)。
void assignPipelineFailure(oap::AnalysisPipelineStatus status,
                           const QString& sourcePath,
                           QString& errorMessage,
                           QString& warningMessage)
{
    switch (status) {
    case oap::AnalysisPipelineStatus::ReadFailed:
        errorMessage =
            QCoreApplication::translate("OpenAutoPath", "读取 STEP 文件失败:\n") +
            sourcePath;
        return;
    case oap::AnalysisPipelineStatus::TransferFailed:
        errorMessage =
            QCoreApplication::translate("OpenAutoPath", "传输 STEP 数据失败:\n") +
            sourcePath;
        return;
    case oap::AnalysisPipelineStatus::NoFreeShapes:
        warningMessage =
            QCoreApplication::translate("OpenAutoPath", "未找到任何形状:\n") +
            sourcePath;
        return;
    case oap::AnalysisPipelineStatus::NoAnalyzableShapes:
        warningMessage =
            QCoreApplication::translate("OpenAutoPath", "未找到可分析的实体:\n") +
            sourcePath;
        return;
    case oap::AnalysisPipelineStatus::Cancelled:
    case oap::AnalysisPipelineStatus::Ok:
        return;
    }
}

// 在 worker 线程执行完整 STEP 读取 + 分析流水线;成功返回 true 并输出
// 阶段耗时统计日志,失败则经 errorMessage / warningMessage 返回原因。
bool executeAnalysisPipeline(const QString& sourcePath,
                             Handle(TDocStd_Document)& document,
                             oap::StepIndexer& stepIndexer,
                             oap::Model& model,
                             QString& errorMessage,
                             QString& warningMessage,
                             const oap::AnalysisPipelineControl& control)
{
    const oap::AnalysisPipelineReport report = oap::AnalysisPipeline().execute(
        sourcePath.toUtf8().toStdString(),
        document, stepIndexer, model, &control);
    if (!report.ok()) {
        assignPipelineFailure(
            report.status, sourcePath,
            errorMessage, warningMessage);
        return false;
    }

    qDebug().nospace()
        << "[OpenAutoPath] load " << sourcePath << "\n"
        << "  free_shapes=" << report.freeShapeCount
        << "  units=" << static_cast<qulonglong>(report.unitCount)
        << "  faces=" << static_cast<qulonglong>(report.faceCount) << "\n"
        << "  ms: read=" << report.readMillis
        << " transfer=" << report.transferMillis
        << " names=" << report.namesMillis
        << " preview_prepare=" << report.previewMillis
        << " bounds_prepare=" << report.geometry.boundsPreparationMillis
        << " face_bounds=" << report.geometry.faceBoundsMillis
        << " isolated_faces=" << report.geometry.isolatedFaceCount
        << " index=" << report.indexMillis
        << " total=" << report.totalMillis();
    return true;
}

// 构造 OCCT 异常的用户可读消息(前缀 + 异常文本)。
QString failureMessage(const char* prefix, const Standard_Failure& failure)
{
    const char* message = failure.GetMessageString();
    return QCoreApplication::translate("OpenAutoPath", prefix) +
           QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromLocal8Bit(message ? message : "OCCT failure").toUtf8().constData());
}

void closeDocumentNoThrow(
    const Handle(XCAFApp_Application)& application,
    Handle(TDocStd_Document)& document)
{
    if (application.IsNull() || document.IsNull())
        return;
    try {
        OCC_CATCH_SIGNALS
        application->Close(document);
    } catch (...) {
        // 异常路径中的清理必须保持 noexcept，文档会随应用退出统一释放。
    }
    document.Nullify();
}

} // namespace

void AnalysisWorker::loadStepFile(
    const QString& sourcePath,
    oap::StepAnalysisCache* analysisCache, quint64 requestId)
{
    auto result =
        std::make_shared<AnalysisLoadResult>();
    result->sourcePath = sourcePath;
    result->requestId = requestId;
    QElapsedTimer workerTimer;
    workerTimer.start();
    const auto finish = [&] {
        result->workerMillis = workerTimer.elapsed();
        qDebug().nospace() << "[OpenAutoPath] worker_end request=" << requestId
            << " wall_ms=" << result->workerMillis << " ok=" << result->ok
            << " cancelled=" << result->cancelled;
        emit loadFinished(result);
    };
    const auto progress = [&](const QString& stage, int done = 0, int total = 0) {
        emit loadProgress(requestId, sourcePath, stage, done, total);
    };
    auto control = loadControl([this] { return m_cancelRun.load(); });
    control.reportStage = [&](oap::AnalysisStage stage) {
        switch (stage) {
        case oap::AnalysisStage::Reading: progress(QCoreApplication::translate("OpenAutoPath", "读取 STEP")); break;
        case oap::AnalysisStage::Transferring: progress(QCoreApplication::translate("OpenAutoPath", "转换几何")); break;
        case oap::AnalysisStage::PreparingPreview: progress(QCoreApplication::translate("OpenAutoPath", "准备模型预览")); break;
        case oap::AnalysisStage::Indexing: progress(QCoreApplication::translate("OpenAutoPath", "精确几何分析")); break;
        }
    };
    int lastPercent = -1;
    control.index.reportProgress = [&](std::size_t done, std::size_t total) {
        // 几何层保证进度回调串行；大模型也只向 UI 排队约一百次更新。
        const int percent = total ? static_cast<int>(100 * done / total) : 100;
        if (percent == lastPercent && done != total) return;
        lastPercent = percent;
        progress(QCoreApplication::translate("OpenAutoPath", "精确几何分析"), static_cast<int>(done), static_cast<int>(total));
    };
    control.preparePreview = [&](const Handle(TDocStd_Document)& doc) {
        try {
            auto preview = makePreview(doc);
            if (preview && !m_cancelRun.load()) {
                preview->requestId = requestId;
                preview->sourcePath = sourcePath;
                preview->elapsedMillis = workerTimer.elapsed();
                emit previewReady(std::move(preview));
            }
        } catch (...) {
            qWarning() << "[OpenAutoPath] preview unavailable; continuing exact analysis";
        }
    };
    // 已在排队期间取消时，避免启动缓存 I/O 和 STEP 分析。
    if (m_cancelRun.load()) {
        result->cancelled = true;
        finish();
        return;
    }
    result->stepIndexer = std::make_shared<oap::StepIndexer>();

    Handle(XCAFApp_Application) application;
    Handle(TDocStd_Document) cachedDocument;
    Handle(TDocStd_Document) document;
    try {
        OCC_CATCH_SIGNALS
        // Windows 的 SE translator 是线程局部状态；每个使用 OCCT 的 QThread
        // 都必须单独初始化，不能依赖 main() 中的调用。
        OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
        application =
            XCAFApp_Application::GetApplication();
        const std::string nativeSourcePath =
            sourcePath.toUtf8().toStdString();

        // 缓存查询:命中则跳过 STEP 解析 + Transfer + 整条分析流水线。
        std::string contentHash;
        progress(QCoreApplication::translate("OpenAutoPath", "检查模型缓存"));
        QElapsedTimer lookupTimer; lookupTimer.start();
        const bool cacheHit = analysisCache && analysisCache->lookup(nativeSourcePath, contentHash);
        qDebug() << "[OpenAutoPath] cache_lookup_ms=" << lookupTimer.elapsed();
        if (cacheHit) {
            progress(QCoreApplication::translate("OpenAutoPath", "恢复模型缓存"));
            QElapsedTimer timer;
            timer.start();
            oap::Model cachedModel;
            const bool retrieved =
                analysisCache->retrieve(
                    contentHash, application,
                    cachedDocument, cachedModel);
            const qint64 retrieveMillis = timer.restart();
            if (retrieved &&
                result->stepIndexer->restoreMaps(
                    cachedDocument, cachedModel)) {
                if (m_cancelRun.load()) {
                    result->cancelled = true;
                    closeDocumentNoThrow(
                        application, cachedDocument);
                    finish();
                    return;
                }
                const qint64 restoreMillis = timer.elapsed();
                result->document = cachedDocument;
                cachedDocument.Nullify();
                result->model = std::move(cachedModel);
                result->ok = true;
                result->fromCache = true;
                qDebug().nospace()
                    << "[OpenAutoPath] cache HIT "
                    << sourcePath
                    << "  units=" << static_cast<qulonglong>(result->model.units.size())
                    << " faces=" << static_cast<qulonglong>(result->model.faces.size())
                    << "  ms: retrieve(open+readModel)=" << retrieveMillis
                    << " restoreMaps=" << restoreMillis
                    << " restore_total=" << (retrieveMillis + restoreMillis);
                finish();
                return;
            }

            // Open 已成功但 Model / 映射恢复失败时，应用对象仍持有该文档。
            closeDocumentNoThrow(
                application, cachedDocument);
        }

        QString errorMessage;
        QString warningMessage;
        if (!executeAnalysisPipeline(
                sourcePath, document,
                *result->stepIndexer, result->model,
                errorMessage, warningMessage, control)) {
            closeDocumentNoThrow(
                application, document);
            result->cancelled = m_cancelRun.load();
            result->errorMessage = std::move(errorMessage);
            result->warningMessage = std::move(warningMessage);
            finish();
            return;
        }

        if (m_cancelRun.load()) {
            result->cancelled = true;
            closeDocumentNoThrow(
                application, document);
            finish();
            return;
        }

        // 缓存写入属于 I/O 与 XCAF 序列化，必须在 worker 中完成；否则分析完成后
        // 主窗口仍会因大模型落盘而卡顿。缓存失败不影响正常显示。
        progress(QCoreApplication::translate("OpenAutoPath", "保存模型缓存"));
        QElapsedTimer storeTimer; storeTimer.start();
        if (analysisCache && !contentHash.empty() &&
            analysisCache->store(
                contentHash, application,
                document,
                result->model)) {
            analysisCache->enforceBudget(
                oap::StepAnalysisCache::
                    kDefaultBudgetBytes);
            qDebug().nospace()
                << "[OpenAutoPath] cache STORE "
                << sourcePath << " ok ms=" << storeTimer.elapsed();
        }

        if (m_cancelRun.load()) {
            result->cancelled = true;
            closeDocumentNoThrow(
                application, document);
            finish();
            return;
        }

        result->document = document;
        document.Nullify();
        result->ok = true;
    } catch (const Standard_Failure& failure) {
        if (m_cancelRun.load()) result->cancelled = true;
        else result->errorMessage = failureMessage("OCCT 后台分析失败:\n", failure);
    } catch (const std::exception& failure) {
        if (m_cancelRun.load()) result->cancelled = true;
        else result->errorMessage = QCoreApplication::translate("OpenAutoPath", "后台分析失败:\n") +
                                    QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromLocal8Bit(failure.what()).toUtf8().constData());
    } catch (...) {
        if (m_cancelRun.load()) result->cancelled = true;
        else result->errorMessage = QCoreApplication::translate("OpenAutoPath", "后台分析发生未知异常。");
    }

    closeDocumentNoThrow(
        application, cachedDocument);
    closeDocumentNoThrow(
        application, document);
    finish();
}

// 批量构建缓存:逐文件查缓存,命中则跳过,未命中则完整分析后存储。
// 在 worker 线程串行执行(OCCT 单例约束);UI 此间模态阻塞,不会并发 load。
void AnalysisWorker::buildStepCaches(
    const QStringList& sourcePaths,
    oap::StepAnalysisCache* analysisCache)
{
    if (m_cancelCacheBuild.load()) {
        emit cacheBuildFinished(0, 0, 0, true);
        return;
    }
    const auto control = loadControl([this] { return m_cancelCacheBuild.load(); });
    int built = 0;
    int skipped = 0;
    int failed = 0;
    bool cancelled = false;
    Handle(XCAFApp_Application) application;

    try {
        OCC_CATCH_SIGNALS
        OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
        application =
            XCAFApp_Application::GetApplication();
    } catch (...) {
        emit cacheBuildFinished(
            0, 0, sourcePaths.size(), false);
        return;
    }

    for (int fileIndex = 0;
         fileIndex < sourcePaths.size();
         ++fileIndex) {
        if (m_cancelCacheBuild.load()) {
            cancelled = true;
            break;
        }
        emit cacheBuildProgress(
            fileIndex, sourcePaths.size(),
            sourcePaths[fileIndex]);

        Handle(TDocStd_Document) document;
        try {
            OCC_CATCH_SIGNALS
            const std::string nativeSourcePath =
                sourcePaths[fileIndex].toUtf8().toStdString();
            std::string contentHash;
            if (analysisCache &&
                analysisCache->lookup(
                    nativeSourcePath,
                    contentHash)) {
                ++skipped;
                continue;
            }

            oap::StepIndexer stepIndexer;
            oap::Model model;
            QString errorMessage;
            QString warningMessage;
            if (!executeAnalysisPipeline(
                    sourcePaths[fileIndex],
                    document, stepIndexer, model,
                    errorMessage, warningMessage, control)) {
                if (m_cancelCacheBuild.load()) {
                    cancelled = true;
                    closeDocumentNoThrow(application, document);
                    break;
                }
                ++failed;
                qDebug().nospace()
                    << "[OpenAutoPath] cache build FAIL "
                    << sourcePaths[fileIndex] << "  "
                    << (errorMessage.isEmpty()
                            ? warningMessage
                            : errorMessage);
                closeDocumentNoThrow(
                    application, document);
                continue;
            }
            if (m_cancelCacheBuild.load()) {
                cancelled = true;
                closeDocumentNoThrow(
                    application, document);
                break;
            }
            const bool stored =
                analysisCache &&
                !contentHash.empty() &&
                analysisCache->store(
                    contentHash,
                    application, document, model);
            // 每个文件结束后立即释放 XCAF 文档，限制批处理峰值内存。
            closeDocumentNoThrow(
                application, document);
            if (stored) {
                ++built;
                qDebug().nospace()
                    << "[OpenAutoPath] cache build STORE "
                    << sourcePaths[fileIndex]
                    << " ok";
            } else {
                ++failed;
                qWarning().nospace()
                    << "[OpenAutoPath] cache build STORE "
                    << sourcePaths[fileIndex]
                    << " failed";
            }
        } catch (const Standard_Failure& failure) {
            ++failed;
            qWarning().noquote() << failureMessage("[OpenAutoPath] cache build OCCT failure: ",
                                                   failure);
            closeDocumentNoThrow(
                application, document);
        } catch (const std::exception& failure) {
            ++failed;
            qWarning().noquote() << "[OpenAutoPath] cache build failure:" << failure.what();
            closeDocumentNoThrow(
                application, document);
        } catch (...) {
            ++failed;
            qWarning() << "[OpenAutoPath] cache build unknown failure";
            closeDocumentNoThrow(
                application, document);
        }
    }

    if (analysisCache) {
        try {
            analysisCache->enforceBudget(
                oap::StepAnalysisCache::
                    kDefaultBudgetBytes);
        } catch (...) {
            qWarning() << "[OpenAutoPath] cache budget enforcement failed";
        }
    }
    emit cacheBuildProgress(
        sourcePaths.size(), sourcePaths.size(),
        QString());
    emit cacheBuildFinished(
        built, skipped, failed, cancelled);
}
