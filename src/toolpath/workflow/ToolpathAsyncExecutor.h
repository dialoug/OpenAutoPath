// 非阻塞刀路执行器。后台任务只处理纯刀路数据，不创建 Qt/OCCT 显示对象；
// progress 回调运行于后台线程，UI 调用方应以队列信号转发到主线程。
//
// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)。
// 职责:把槽腔/孔/外轮廓刀路服务包装为可取消、可轮询的异步任务;任务持有
// 服务对象的按值副本与共享取消标志,因此执行器本身可安全复制/复用。
// 注意:取消是协作式的——算法只在可控点检查标志,不能强杀线程。
#pragma once

#include <atomic>
#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "toolpath/core/HeavyJobScheduler.h"
#include "toolpath/workflow/pocket/PocketToolpathService.h"
#include "toolpath/workflow/profile/ProfileToolpathService.h"

namespace oap::toolpath {

// 异步任务句柄:包装 std::future 与共享取消标志,提供非阻塞的
// ready()/cancel() 查询与阻塞的 get()。仅可移动,不可复制。
template <typename Result>
// 异步刀路任务句柄：future 与取消标记。
class ToolpathAsyncTask {
public:
    ToolpathAsyncTask() = default;
    ToolpathAsyncTask(std::future<Result> future,
                      std::shared_ptr<std::atomic_bool> cancellation)
        : m_future(std::move(future)),
          m_cancellation(std::move(cancellation))
    {
    }

    ToolpathAsyncTask(const ToolpathAsyncTask&) = delete;
    ToolpathAsyncTask& operator=(const ToolpathAsyncTask&) = delete;
    ToolpathAsyncTask(ToolpathAsyncTask&&) noexcept = default;
    ToolpathAsyncTask& operator=(ToolpathAsyncTask&& other) noexcept
    {
        if (this != &other) {
            cancelAndWait();
            m_future = std::move(other.m_future);
            m_cancellation = std::move(other.m_cancellation);
        }
        return *this;
    }
    ~ToolpathAsyncTask() { cancelAndWait(); }

    // 是否持有有效的后台任务。
    bool valid() const noexcept { return m_future.valid(); }
    bool queued() const
    {
        return m_future.valid() && heavyJobScheduler().isPending(m_cancellation);
    }
    // 非阻塞查询任务是否已完成(不消费结果)。
    bool ready() const
    {
        return m_future.valid() &&
               m_future.wait_for(std::chrono::seconds(0)) ==
                   std::future_status::ready;
    }
    // 请求协作式取消:置位共享原子标志,算法在受控点响应。
    void cancel() noexcept
    {
        cancelHeavyJob(m_cancellation);
    }
    // 阻塞等待并取得结果(只能调用一次,行为同 std::future::get)。
    Result get() { return m_future.get(); }

private:
    void cancelAndWait() noexcept
    {
        if (m_future.valid()) {
            cancel();
            m_future.wait();
        }
    }

    std::future<Result> m_future;
    std::shared_ptr<std::atomic_bool> m_cancellation;
};

// 进度回调:completed 已完成的孔/区域数,total 总数;在后台线程调用,
// UI 调用方须转发到主线程再更新控件。
using ToolpathProgressCallback =
    std::function<void(std::size_t completed, std::size_t total)>;
using PocketToolpathAsyncTask = ToolpathAsyncTask<PocketToolpathBatch>;
using HoleToolpathAsyncTask = ToolpathAsyncTask<HoleToolpathBatch>;
using ProfileToolpathAsyncTask = ToolpathAsyncTask<ProfileToolpathBatch>;

// 异步刀路执行器：按值持有输入与服务副本，共用有界重计算作业队列。
// 排队取消不运行算法；销毁任务句柄会取消并等待，回调不得捕获更短寿命的数据。
class ToolpathAsyncExecutor {
public:
    // 默认构造:使用默认算法注册表与内置算法。
    ToolpathAsyncExecutor();
    // 显式构造:注入自定义注册表与算法选择。
    ToolpathAsyncExecutor(
        std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
        ToolpathAlgorithmSelection selection = {});

    // 异步生成槽腔刀路。
    // 参数:regions 已构建的槽腔区域(按值转移给后台);params 工艺参数;
    //       operationPrefix 工序 ID 前缀;progress 后台进度回调。
    // 返回:可取消的任务句柄;异常被捕获为批内诊断,不会抛出。
    PocketToolpathAsyncTask generatePockets(
        std::vector<PocketRegion> regions,
        ToolpathParams params,
        std::string operationPrefix = "POCKET",
        ToolpathProgressCallback progress = {}) const;

    // 异步生成孔加工刀路。
    // 参数:inputs 孔输入;params 工艺参数;rapidContext 可选的孔间快速
    //       移动规划上下文;strategyOverrides 可选的逐孔策略覆盖;
    //       progress 后台进度回调。
    // 返回:可取消的任务句柄;异常被捕获为批内诊断,不会抛出。
    HoleToolpathAsyncTask generateHoles(
        std::vector<HoleMachiningInput> inputs,
        ToolpathParams params,
        std::optional<HoleRapidPlanningContext> rapidContext = std::nullopt,
        std::vector<HoleMachiningStrategy> strategyOverrides = {},
        ToolpathProgressCallback progress = {}) const;

    // 异步生成外轮廓刀路。区域和参数按值转移到后台线程，生成、
    // Profile 安全验证与取消均由 ProfileToolpathService 统一处理。
    ProfileToolpathAsyncTask generateProfiles(
        std::vector<ProfileRegion> regions,
        ToolpathParams params,
        std::string operationPrefix = "PROFILE",
        ToolpathProgressCallback progress = {}) const;

private:
    PocketToolpathService m_pocketService;
    HoleToolpathService m_holeService;
    ProfileToolpathService m_profileService;
};

} // namespace oap::toolpath
