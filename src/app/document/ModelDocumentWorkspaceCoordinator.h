// 已分析文档的所有权与异步加载生命周期的应用边界。
// 职责:作为活动模型文档(ModelDocumentSession)的唯一拥有者,统一管理
//   STEP 文件加载与缓存构建异步任务:发起、取消、结果安装与状态查询。
// 模块归属:src/app(document 聚焦工作区协调器),桌面层,由 MainWindow
//   创建并持有;下层依赖 app/analysis 的分析加载控制器与步骤分析缓存。
// 协作:AnalysisLoadController 执行后台任务,StepAnalysisCache 提供缓存,
//   ModelDocumentSession 保存已安装的文档/模型/索引;加载结果经
//   completeLoad 原子替换会话内容,完成状态由 Activity 状态机跟踪。
#pragma once

#include <functional>
#include <memory>

#include <QObject>
#include <QString>
#include <QStringList>

#include "app/workflows/shared/ModelDocumentSession.h"
#include "app/analysis/AnalysisLoadResult.h"

namespace oap {
class StepAnalysisCache;
}

namespace oap::app {

class AnalysisLoadController;

// 单次文档加载的最终状态:
//   Ignored —— 未处于加载活动或结果与请求不匹配,直接忽略;
//   Cancelled —— 后台任务被取消;
//   Failed —— 解析/分析失败或缺少文档/索引;
//   Warning —— 带警告降级处理;
//   Installed —— 会话已成功替换。
enum class DocumentLoadStatus {
    Ignored,
    Cancelled,
    Failed,
    Warning,
    Installed,
};

// 文档加载完成的结果描述:状态、请求/实际路径、消息文本与是否来自缓存;
// installed() 便捷判断是否安装成功。
struct DocumentLoadCompletion {
    DocumentLoadStatus status = DocumentLoadStatus::Ignored;
    QString requestedPath;
    QString sourcePath;
    QString message;
    bool fromCache = false;

    bool installed() const noexcept
    {
        return status == DocumentLoadStatus::Installed;
    }
};

// 活动模型文档工作区协调器。
// 状态所有权:长期持有 ModelDocumentSession(已安装文档)、
//   StepAnalysisCache(共享分析缓存)与 AnalysisLoadController(后台任务);
//   一次只允许一个加载/建缓存活动(Activity 状态机)。
// 生命周期:由 MainWindow 持有,析构时自动取消并等待后台任务结束。
class ModelDocumentWorkspaceCoordinator final : public QObject {
    Q_OBJECT
public:
    explicit ModelDocumentWorkspaceCoordinator(
        std::string cacheRoot,
        QObject* parent = nullptr);
    ~ModelDocumentWorkspaceCoordinator() override;

    ModelDocumentWorkspaceCoordinator(
        const ModelDocumentWorkspaceCoordinator&) = delete;
    ModelDocumentWorkspaceCoordinator& operator=(
        const ModelDocumentWorkspaceCoordinator&) = delete;

    // 会话是否为空(尚无已安装文档)。
    bool empty() const noexcept { return m_session.empty(); }
    // 已安装文档的源文件路径(会话为空时为空串)。
    const QString& sourcePath() const noexcept
    {
        return m_session.sourcePath();
    }
    // 已安装的 OCCT 文档句柄(会话为空时为 Null)。
    const Handle(TDocStd_Document)& document() const noexcept
    {
        return m_session.document();
    }
    // 已安装的模型(面/边等几何分析结果)。
    const Model& model() const noexcept { return m_session.model(); }
    // 稳定的模型索引(顶点/边/面索引映射),随文档一起安装。
    const ModelIndex& modelIndex() const noexcept
    {
        return m_session.modelIndex();
    }
    // 步骤索引器(STEP 实体与模型元素的对应关系)。
    const StepIndexer& stepIndexer() const noexcept
    {
        return m_session.stepIndexer();
    }
    // 模具坐标系(基准面/原点),来自分析结果。
    const MoldCoordinateFrame& moldFrame() const noexcept
    {
        return m_session.moldFrame();
    }
    // 装夹工位坐标系集合(加工前由调用方安装)。
    const toolpath::MoldSetupWorkFrames& setupWorkFrames() const noexcept
    {
        return m_session.setupWorkFrames();
    }
    // 模型包围盒,用于视图初始取景等。
    const Bbox& bounds() const noexcept { return m_session.bounds(); }
    // 安装装夹工位坐标系(按值传入并移动赋值),不依赖加载状态。
    void installSetupWorkFrames(toolpath::MoldSetupWorkFrames workFrames);

    // 发起 STEP 文件加载:要求路径非空、当前不忙且控制器存在;
    // 成功后进入 LoadingFile 活动,分析结果经 analysisFinished 信号返回,
    // 由 completeLoad 安装。
    bool beginLoad(const QString& sourcePath);
    // 发起分析缓存批量构建(预建缓存供后续加载复用);
    // 成功返回 true 并进入 BuildingCaches 活动,进度与结束经信号上报。
    bool beginCacheBuild(
        const QStringList& sourcePaths);
    // 完成加载:消费后台分析结果并原子安装到会话。
    // beforeCommit 在安装前同步回调(如 UI 提前清理旧状态),若其抛异常
    // 则终止活动且不安装;仅当处于 LoadingFile 活动且结果与请求路径匹配
    // 时才安装,否则返回 Ignored。
    DocumentLoadCompletion completeLoad(
        const std::shared_ptr<AnalysisLoadResult>& result,
        const std::function<void()>& beforeCommit = {});

    // 活动状态查询:busy 表示正在执行任一任务,
    // loadingFile/buildingCaches 细分当前活动类型。
    bool busy() const noexcept;
    bool loadingFile() const noexcept;
    bool buildingCaches() const noexcept;
    // 最近一次 beginLoad 的请求路径(供 completeLoad 结果校验)。
    const QString& pendingPath() const noexcept { return m_pendingPath; }
    bool acceptsPreview(const AnalysisLoadPreview& preview) const noexcept;

    // 请求取消当前活动(异步、非阻塞);cancelAndWait 同步取消全部
    // 后台任务并等待其结束,析构时调用。
    void requestCancelLoad() noexcept;
    void requestCancelCacheBuild() noexcept;
    void requestCancelAll() noexcept;
    void cancelAndWait();

signals:
    void previewReady(std::shared_ptr<AnalysisLoadPreview> preview);
    void loadProgress(const QString& stage, int completed, int total);
    // 后台加载分析完成(含文档/模型/索引/错误信息),由 completeLoad 消费。
    void analysisFinished(std::shared_ptr<AnalysisLoadResult> result);
    // 缓存构建进度(done/total/当前文件名)。
    void cacheBuildProgress(
        int done, int total, const QString& currentFile);
    // 缓存构建结束(成功数/跳过数/失败数/是否被取消)。
    void cacheBuildFinished(
        int built, int skipped, int failed, bool cancelled);

private:
    // 协调器活动状态机:一次只允许一种后台活动。
    enum class Activity {
        Idle,
        LoadingFile,
        BuildingCaches,
    };

    // 结束当前活动并清除待处理路径,回到 Idle。
    void finishActivity() noexcept;

    ModelDocumentSession m_session;                          // 已安装文档会话(长期状态)。
    std::shared_ptr<StepAnalysisCache> m_cache;              // 共享分析缓存。
    std::unique_ptr<AnalysisLoadController> m_controller;    // 后台任务控制器。
    Activity m_activity = Activity::Idle;                    // 当前活动状态。
    QString m_pendingPath;                                   // 待安装的请求路径。
    quint64 m_requestId = 0;
    bool m_cancelRequested = false;
};

} // namespace oap::app
