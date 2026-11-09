// ModelDocumentWorkspaceCoordinator 实现（oap_app_document）。
// 已分析文档的所有权与异步加载生命周期，见 ModelDocumentWorkspaceCoordinator.h。
#include "ModelDocumentWorkspaceCoordinator.h"

#include <utility>

#include <BinXCAFDrivers.hxx>
#include <XCAFApp_Application.hxx>

#include "app/analysis/AnalysisLoadController.h"
#include "app/analysis/StepAnalysisCache.h"

namespace oap::app {

// 构造函数:创建共享分析缓存与后台加载控制器,注册 XCAF 格式驱动,
// 并把控制器的加载完成/缓存进度信号转发为协调器信号;缓存构建结束
// 信号在仍处于 BuildingCaches 活动时先复位活动状态再转发。
ModelDocumentWorkspaceCoordinator::ModelDocumentWorkspaceCoordinator(
    std::string cacheRoot,
    QObject* parent)
    : QObject(parent),
      m_cache(std::make_shared<StepAnalysisCache>(std::move(cacheRoot))),
      m_controller(std::make_unique<AnalysisLoadController>())
{
    BinXCAFDrivers::DefineFormat(XCAFApp_Application::GetApplication());
    connect(
        m_controller.get(), &AnalysisLoadController::loadFinished,
        this, &ModelDocumentWorkspaceCoordinator::analysisFinished);
    connect(m_controller.get(), &AnalysisLoadController::previewReady,
            this, [this](std::shared_ptr<AnalysisLoadPreview> preview) {
        if (preview && acceptsPreview(*preview)) emit previewReady(std::move(preview));
    });
    connect(m_controller.get(), &AnalysisLoadController::loadProgress,
            this, [this](quint64 id, const QString& path, const QString& stage, int done, int total) {
        if (loadingFile() && !m_cancelRequested && id == m_requestId && path == m_pendingPath)
            emit loadProgress(stage, done, total);
    });
    connect(
        m_controller.get(), &AnalysisLoadController::cacheBuildProgress,
        this, &ModelDocumentWorkspaceCoordinator::cacheBuildProgress);
    connect(
        m_controller.get(), &AnalysisLoadController::cacheBuildFinished,
        this,
        [this](int built, int skipped, int failed, bool cancelled) {
            if (m_activity == Activity::BuildingCaches)
                finishActivity();
            emit cacheBuildFinished(
                built, skipped, failed, cancelled);
        });
}

// 析构:同步取消并等待后台任务结束,避免悬挂的异步回调访问已销毁成员。
ModelDocumentWorkspaceCoordinator::~ModelDocumentWorkspaceCoordinator()
{
    cancelAndWait();
}

void ModelDocumentWorkspaceCoordinator::installSetupWorkFrames(
    toolpath::MoldSetupWorkFrames workFrames)
{
    m_session.setupWorkFrames() = std::move(workFrames);
}

// 前置条件检查:路径非空、无并发活动、控制器存在,不满足则保持 Idle。
bool ModelDocumentWorkspaceCoordinator::beginLoad(
    const QString& sourcePath)
{
    if (sourcePath.isEmpty() || busy() || !m_controller)
        return false;
    // 先置 LoadingFile 活动并记录请求路径,再启动后台加载;
    // 启动失败时回退到 Idle。
    m_activity = Activity::LoadingFile;
    m_pendingPath = sourcePath;
    m_cancelRequested = false;
    if (m_controller->loadStepFile(sourcePath, m_cache)) {
        m_requestId = m_controller->currentRequestId();
        return true;
    }
    finishActivity();
    return false;
}

// 缓存构建的发起逻辑同 beginLoad:先进入 BuildingCaches 活动,
// 成功后按进度信号推进,结束时回到 Idle。
bool ModelDocumentWorkspaceCoordinator::beginCacheBuild(
    const QStringList& sourcePaths)
{
    if (sourcePaths.isEmpty() || busy() || !m_controller)
        return false;
    m_activity = Activity::BuildingCaches;
    m_pendingPath.clear();
    if (m_controller->buildStepCaches(sourcePaths, m_cache))
        return true;
    finishActivity();
    return false;
}

DocumentLoadCompletion ModelDocumentWorkspaceCoordinator::completeLoad(
    const std::shared_ptr<AnalysisLoadResult>& result,
    const std::function<void()>& beforeCommit)
{
    DocumentLoadCompletion completion;
    completion.requestedPath = m_pendingPath;
    // 陈旧结果门禁:仅当仍处于 LoadingFile 活动且后台结果路径与请求路径
    // 一致时才处理,否则返回 Ignored,绝不触碰已安装的会话。
    if (m_activity != Activity::LoadingFile)
        return completion;
    if (result && result->requestId != 0 && result->requestId != m_requestId)
        return completion;
    if (result && !m_pendingPath.isEmpty() &&
        !result->sourcePath.isEmpty() &&
        result->sourcePath != m_pendingPath) {
        return completion;
    }

    const auto finish = [this] { finishActivity(); };
    // 结果校验按严重程度递减:无结果 > 取消 > 错误 > 警告/失败 > 缺文档/索引。
    if (!result) {
        completion.status = DocumentLoadStatus::Failed;
        completion.message = QStringLiteral("后台分析未返回结果。");
        finish();
        return completion;
    }
    completion.sourcePath = result->sourcePath;
    completion.fromCache = result->fromCache;
    if (result->cancelled || m_cancelRequested) {
        // 取消可能发生在 worker 已完成并移交文档之后，仍需释放 XCAF 注册的文档。
        if (!result->document.IsNull()) {
            XCAFApp_Application::GetApplication()->Close(result->document);
            result->document.Nullify();
        }
        completion.status = DocumentLoadStatus::Cancelled;
        finish();
        return completion;
    }
    if (!result->errorMessage.isEmpty()) {
        completion.status = DocumentLoadStatus::Failed;
        completion.message = result->errorMessage;
        finish();
        return completion;
    }
    if (!result->ok) {
        completion.status = result->warningMessage.isEmpty()
            ? DocumentLoadStatus::Failed
            : DocumentLoadStatus::Warning;
        completion.message = result->warningMessage;
        finish();
        return completion;
    }
    if (result->document.IsNull() || !result->stepIndexer) {
        completion.status = DocumentLoadStatus::Failed;
        completion.message = QStringLiteral(
            "后台分析结果缺少文档或稳定索引。");
        finish();
        return completion;
    }

    try {
        if (beforeCommit)
            beforeCommit();
        // 原子安装:先让调用方在 beforeCommit 中清理旧状态,再以移动语义
        // 整体替换会话(源路径/文档/模型/索引);观察者要么看到旧会话,
        // 要么看到完整的新会话。安装期间异常则终止活动并向上抛出。
        m_session.replace(
            result->sourcePath, result->document,
            std::move(result->model),
            std::move(*result->stepIndexer));
    } catch (...) {
        finish();
        throw;
    }
    completion.status = DocumentLoadStatus::Installed;
    finish();
    return completion;
}

bool ModelDocumentWorkspaceCoordinator::busy() const noexcept
{
    return m_activity != Activity::Idle;
}

bool ModelDocumentWorkspaceCoordinator::loadingFile() const noexcept
{
    return m_activity == Activity::LoadingFile;
}

bool ModelDocumentWorkspaceCoordinator::buildingCaches() const noexcept
{
    return m_activity == Activity::BuildingCaches;
}

void ModelDocumentWorkspaceCoordinator::requestCancelLoad() noexcept
{
    m_cancelRequested = true;
    if (m_controller)
        m_controller->requestCancelLoad();
}

void ModelDocumentWorkspaceCoordinator::requestCancelCacheBuild() noexcept
{
    if (m_controller)
        m_controller->requestCancelCacheBuild();
}

void ModelDocumentWorkspaceCoordinator::requestCancelAll() noexcept
{
    m_cancelRequested = true;
    if (m_controller)
        m_controller->requestCancelAll();
}

void ModelDocumentWorkspaceCoordinator::cancelAndWait()
{
    if (m_controller)
        m_controller->cancelAndWait();
    finishActivity();
}

// 复位活动状态与待处理路径;所有加载/建缓存结束路径共用此出口。
void ModelDocumentWorkspaceCoordinator::finishActivity() noexcept
{
    m_activity = Activity::Idle;
    m_pendingPath.clear();
    m_requestId = 0;
}

bool ModelDocumentWorkspaceCoordinator::acceptsPreview(const AnalysisLoadPreview& preview) const noexcept
{
    return loadingFile() && !m_cancelRequested && preview.requestId != 0 &&
           preview.requestId == m_requestId && preview.sourcePath == m_pendingPath;
}

} // namespace oap::app
