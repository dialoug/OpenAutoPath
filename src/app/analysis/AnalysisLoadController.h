// 拥有 STEP 分析工作线程及其取消/关闭生命周期。
// STEP 加载与缓存构建在独立 QThread 上执行,控制器负责派发请求、转发结果
// 信号与统一取消;分析任务全部由后台 AnalysisWorker 完成,UI 线程不冻结。
#pragma once

#include "app/analysis/AnalysisWorker.h"

#include <memory>

#include <QObject>

class QThread;

namespace oap::app {

// 任务控制器(生命周期由 owner 线程管理):创建后台线程与 worker,
// 通过 queued invokeMethod 派发任务,把 worker 的完成信号转发到 UI 线程。
// 提交/取消/关闭均由 owner 线程调用；owner 必须保证一次只有一个任务，
// 收到完成信号后才能再次提交（文档工作区通过 Activity 状态机保证）。
class AnalysisLoadController final : public QObject {
    Q_OBJECT
public:
    explicit AnalysisLoadController(QObject* parent = nullptr);
    ~AnalysisLoadController() override;

    AnalysisLoadController(const AnalysisLoadController&) = delete;
    AnalysisLoadController& operator=(
        const AnalysisLoadController&) = delete;

    // 在后台线程加载单个 STEP 文件并跑完整分析流水线。
    // 前置条件:线程正在运行(running())。analysisCache 可为空(跳过缓存)。
    // 返回 false 表示派发失败(线程已停止);结果经 loadFinished 信号回传。
    bool loadStepFile(
        QString sourcePath,
        std::shared_ptr<StepAnalysisCache> analysisCache);
    // 在后台线程为 sourcePaths 逐文件预建缓存(已命中则跳过)。
    // 进度与收尾统计分别经 cacheBuildProgress / cacheBuildFinished 信号上报。
    bool buildStepCaches(
        QStringList sourcePaths,
        std::shared_ptr<StepAnalysisCache> analysisCache);

    // 请求取消当前加载(原子置位,worker 在检查点处响应)。
    void requestCancelLoad() noexcept;
    // 请求取消当前缓存构建。
    void requestCancelCacheBuild() noexcept;
    // 同时请求取消加载与缓存构建。
    void requestCancelAll() noexcept;
    // 请求取消并等待后台线程退出(阻塞,析构时调用);随后 worker 被释放。
    void cancelAndWait();
    // 后台线程是否正在运行。
    bool running() const noexcept;
    quint64 currentRequestId() const noexcept { return m_requestId; }

signals:
    void previewReady(std::shared_ptr<AnalysisLoadPreview> preview);
    void loadProgress(quint64 requestId, const QString& sourcePath,
                      const QString& stage, int completed, int total);
    // 单文件加载完成(成功/失败/取消均发出),result 整体移交 UI 线程。
    void loadFinished(std::shared_ptr<AnalysisLoadResult> result);
    // 缓存构建逐文件进度:已处理 done 个(共 total),currentFile 为当前文件。
    void cacheBuildProgress(
        int done, int total, const QString& currentFile);
    // 缓存构建收尾统计:built 新建 / skipped 跳过 / failed 失败 / cancelled 取消。
    void cacheBuildFinished(
        int built, int skipped, int failed, bool cancelled);

private:
    QThread* m_thread = nullptr;
    AnalysisWorker* m_worker = nullptr;
    quint64 m_requestId = 0;
};

} // namespace oap::app

