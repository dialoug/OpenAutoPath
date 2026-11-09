// 桌面识别任务生命周期：拥有线程、代次和取消，不拥有 QWidget 或 AIS。
// start() 每次提交一个新代次到共享重计算队列，提交前取消并等待旧代次；
// 后台 runner 持有模型/索引快照 + TaskControl 原子状态，进度经
// 共享 TaskState 暴露给 progress()/ready(),结果经 take() 一次性取走。
#pragma once

#include "recognition/FeatureRecognizer.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>

namespace oap {

class ModelIndex;
class StepIndexer;
struct Model;

namespace app {

// 识别流水线的执行阶段,用于进度展示。
enum class ThreeAxisRecognitionPhase {
    Queued,
    RecognizingHoles,  // 孔/槽/腔特征识别(FeatureRecognizer)
    TracingStructures, // 三向内外环线框追踪(ThreeAxisWireTraceUseCase)
    Finished,          // 任务结束(无论成败或取消)
};

// 提交输入：start() 返回前复制模型/拓扑索引并重建 ModelIndex。
// 调用方只需保证这些只读指针在 start() 期间有效。
struct ThreeAxisRecognitionRequest {
    const Model* model = nullptr;              // 分析后的模型
    const ModelIndex* modelIndex = nullptr;    // 拓扑索引(线框追踪需要)
    const StepIndexer* stepIndexer = nullptr;  // STEP 索引(特征识别需要)
};

// 后台任务结果,经 take() 取走后由调用方拥有。
struct ThreeAxisRecognitionTaskResult {
    std::uint64_t requestId = 0;        // 对应请求(代次)号
    RecognitionResult recognition;      // 孔/槽/腔特征识别结果
    ThreeAxisWireTraceUseCaseResult wireTrace; // 三向线框追踪结果
    bool cancelled = false;             // 是否被取消
    std::string error;                  // 失败原因(空表示无错误)

    bool ok() const noexcept
    {
        return !cancelled && error.empty() && wireTrace.ok();
    }
};

// 运行中任务的进度快照(由 owner 线程轮询)。
struct ThreeAxisRecognitionProgress {
    std::uint64_t requestId = 0;        // 对应请求(代次)号
    ThreeAxisRecognitionPhase phase =
        ThreeAxisRecognitionPhase::Queued;
    std::size_t completed = 0;   // 已完成单元数
    std::size_t total = 0;       // 总单元数
    bool ready = false;          // 结果是否已就绪
};

// 后台回调句柄:取消查询与进度上报,由 runner 在线程内调用(线程安全)。
struct ThreeAxisRecognitionTaskControl {
    std::function<bool()> isCancellationRequested; // 外部取消查询
    std::function<void(ThreeAxisRecognitionPhase phase,
                       std::size_t completed,
                       std::size_t total)> reportProgress; // 进度上报

    bool cancelled() const
    {
        return isCancellationRequested && isCancellationRequested();
    }
};

// 所有方法由 owner 线程调用。后台 runner 收到自有快照的只读引用和独立 control。
class ThreeAxisRecognitionController {
public:
    // 后台执行器:输入 request 只读引用 + control,返回任务结果。
    using Runner = std::function<ThreeAxisRecognitionTaskResult(
        const ThreeAxisRecognitionRequest&,
        const ThreeAxisRecognitionTaskControl&)>;

    // runner 为空时使用默认 runDefault;构造函数仅记录回调,不启动线程。
    explicit ThreeAxisRecognitionController(Runner runner = {});
    // 析构:取消并等待后台任务结束。
    ~ThreeAxisRecognitionController();

    ThreeAxisRecognitionController(
        const ThreeAxisRecognitionController&) = delete;
    ThreeAxisRecognitionController& operator=(
        const ThreeAxisRecognitionController&) = delete;

    // 启动新代次任务,返回新请求号(旧代次会先被取消并等待结束)。
    std::uint64_t start(ThreeAxisRecognitionRequest request);
    // 请求取消指定代次(requestId 为 0 表示当前代次)。
    void cancel(std::uint64_t requestId = 0);
    // 取消并等待任务结束,随后清空控制器内部状态。
    void cancelAndWait();

    // 查询指定代次的进度快照(未运行 / 代次不匹配时返回仅含 requestId 的空快照)。
    ThreeAxisRecognitionProgress progress(
        std::uint64_t requestId) const;
    // 指定代次的结果是否已就绪(可被 take)。
    bool ready(std::uint64_t requestId) const;
    // 取走已就绪的结果(一次性);未就绪返回 nullopt,成功后重置内部状态。
    std::optional<ThreeAxisRecognitionTaskResult> take(
        std::uint64_t requestId);

    // 当前活动代次号。
    std::uint64_t currentRequestId() const noexcept
    {
        return m_activeRequestId;
    }

private:
    // 后台与 owner 共享的原子任务状态(取消标志 + 进度)。
    struct TaskState {
        std::atomic_bool cancelled{false};
        std::atomic<ThreeAxisRecognitionPhase> phase{
            ThreeAxisRecognitionPhase::Queued};
        std::atomic_size_t completed{0};
        std::atomic_size_t total{0};
    };

    // 默认执行器:孔特征识别 + 三向线框追踪,阶段间检查取消。
    static ThreeAxisRecognitionTaskResult runDefault(
        const ThreeAxisRecognitionRequest& request,
        const ThreeAxisRecognitionTaskControl& control);

    Runner m_runner;                    // 后台执行器(默认 runDefault)
    std::uint64_t m_generation = 0;     // 代次计数器(每次 start +1)
    std::uint64_t m_activeRequestId = 0; // 当前活动代次号
    std::shared_ptr<TaskState> m_state;   // 共享原子状态
    std::future<ThreeAxisRecognitionTaskResult> m_future; // 后台任务句柄
};

} // namespace app
} // namespace oap
