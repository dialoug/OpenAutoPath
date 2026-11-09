// AnalysisWorker — 在独立工作线程上执行 STEP 加载与整条分析流水线,
// 使 UI 线程在加载期间不冻结。worker 只操作自己的局部 OCC 对象
// (新建 document / 局部 StepIndexer / 局部 Model),算完通过 loadFinished 信号把结果
// 整体移交回 UI 线程;UI 线程再负责显示(displayModel/FitAll/Redraw)与弹窗。
//
// 线程约束:OCC 对象非线程亲和。靠"加载中忽略新请求"保证任意时刻只有一个 worker,
// 故全局 XCAFApp_Application 单例不会被并发访问。worker 绝不触碰 MainWindow 的成员
// (尤其是仍被视图引用的 m_document)。
#pragma once

#include <atomic>
#include <memory>

#include <QObject>
#include <QString>
#include <QStringList>

#include <TDocStd_Document.hxx>

#include "analysis/Model.h"
#include "analysis/StepIndexer.h"
#include "app/analysis/AnalysisLoadResult.h"

namespace oap { class StepAnalysisCache; }

// 分析后台工作线程：QObject 封装，异步执行 STEP 分析。
class AnalysisWorker : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;

public slots:
    // 在工作线程执行:缓存查询 -> 命中则取出,未命中则 STEP 读取/Transfer/索引。
    // 结果填入 AnalysisLoadResult 后发回 UI；analysisCache 可为空。
    void loadStepFile(
        const QString& sourcePath,
        oap::StepAnalysisCache* analysisCache,
        quint64 requestId = 0);

    // 在工作线程执行:为 sourcePaths 中每个文件预建缓存。已命中缓存的文件跳过;
    // 否则跑完整流水线并存储。
    void buildStepCaches(
        const QStringList& sourcePaths,
        oap::StepAnalysisCache* analysisCache);

public:
    // 由 owner 在单飞任务入队前重置；执行任务时不得再次清除取消请求。
    void prepareRun() { m_cancelRun.store(false); }
    void prepareCacheBuild() {
        m_cancelCacheBuild.store(false);
    }
    // 请求中止正在运行的缓存构建。可从任意线程调用(原子写);worker 在文件
    // 边界处检查它。不能用 queued slot —— worker 在循环中忙,排队消息要等循环
    // 返回才会处理,无法及时取消。
    void requestCancelCacheBuild() {
        m_cancelCacheBuild.store(true);
    }
    // 请求中止正在运行的加载或缓存构建。可从任意线程调用(原子写);
    // 与 requestCancelCacheBuild 的区别:它也覆盖单文件加载(loadStepFile)。
    void requestCancel() {
        m_cancelRun.store(true);
        m_cancelCacheBuild.store(true);
    }

signals:
    void previewReady(std::shared_ptr<AnalysisLoadPreview> preview);
    void loadProgress(quint64 requestId, const QString& sourcePath,
                      const QString& stage, int completed, int total);
    // 加载完成(成功 / 缓存命中 / 失败 / 取消均发出),结果整体移交 UI 线程。
    void loadFinished(
        std::shared_ptr<AnalysisLoadResult> result);

    // 缓存构建进度:已处理 done 个(共 total),currentFile 为即将处理的文件。
    void cacheBuildProgress(
        int done, int total,
        const QString& currentFile);
    // 收尾统计:built 新建 / skipped 已缓存跳过 / failed 失败。
    void cacheBuildFinished(
        int built, int skipped,
        int failed, bool cancelled);

private:
    std::atomic<bool> m_cancelRun{false};
    std::atomic<bool> m_cancelCacheBuild{false};
};

// 跨线程(队列连接 / invokeMethod)传递这些类型需先注册到 Qt 元对象系统。
// 缓存仅前向声明,故以不透明指针方式登记(队列连接只搬运指针本身)。
Q_DECLARE_OPAQUE_POINTER(oap::StepAnalysisCache*)
Q_DECLARE_METATYPE(oap::StepAnalysisCache*)
