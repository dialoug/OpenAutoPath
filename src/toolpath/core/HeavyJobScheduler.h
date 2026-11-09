// Bounded admission for independent, expensive jobs. Nested algorithm workers
// continue to use ToolpathParallel; they must not submit another heavy job.
#pragma once

#include <atomic>
#include <cstddef>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>

namespace oap::toolpath {

enum class HeavyJobRejection { Cancelled, QueueFull, Stopping, Reentrant };

const char* heavyJobRejectionMessage(HeavyJobRejection reason) noexcept;

class HeavyJobScheduler {
public:
    // One persistent worker; pending jobs own snapshots, not waiting threads.
    explicit HeavyJobScheduler(std::size_t pendingCapacity = 32u);
    ~HeavyJobScheduler();
    HeavyJobScheduler(const HeavyJobScheduler&) = delete;
    HeavyJobScheduler& operator=(const HeavyJobScheduler&) = delete;

    // onNotStarted returns a domain result and may run on the submitting or
    // cancelling thread. It must not access UI objects. Both callbacks' captures
    // are released before the result becomes ready, including cancellation.
    template <class Work, class OnNotStarted>
    auto submit(std::shared_ptr<std::atomic_bool> cancellation,
                Work&& work, OnNotStarted&& onNotStarted)
        -> std::future<std::invoke_result_t<std::decay_t<Work>&>>
    {
        using Function = std::decay_t<Work>;
        using RejectionFunction = std::decay_t<OnNotStarted>;
        using Result = std::invoke_result_t<Function&>;
        static_assert(!std::is_reference_v<Result>,
                      "heavy jobs must return owned values");
        struct State {
            std::optional<Function> work;
            std::optional<RejectionFunction> reject;
            std::promise<Result> promise;

            void clearCallbacks() noexcept { work.reset(); reject.reset(); }
            void fail() noexcept
            {
                const auto failure = std::current_exception();
                clearCallbacks();
                promise.set_exception(failure);
            }
            void run() noexcept
            {
                try {
                    if constexpr (std::is_void_v<Result>) {
                        std::invoke(*work);
                        clearCallbacks();
                        promise.set_value();
                    } else {
                        Result result = std::invoke(*work);
                        clearCallbacks();
                        promise.set_value(std::move(result));
                    }
                } catch (...) { fail(); }
            }
            void notStarted(HeavyJobRejection reason) noexcept
            {
                // A cancelled queued job must release model/input references
                // before its owner can observe readiness and destroy them.
                work.reset();
                try {
                    if constexpr (std::is_void_v<Result>) {
                        std::invoke(*reject, reason);
                        clearCallbacks();
                        promise.set_value();
                    } else {
                        Result result = std::invoke(*reject, reason);
                        clearCallbacks();
                        promise.set_value(std::move(result));
                    }
                } catch (...) { fail(); }
            }
        };
        auto state = std::make_shared<State>(State{
            std::optional<Function>(std::in_place, std::forward<Work>(work)),
            std::optional<RejectionFunction>(std::in_place,
                                            std::forward<OnNotStarted>(onNotStarted)),
            {}});
        auto future = state->promise.get_future();
        enqueue({std::move(cancellation),
                 [state] { state->run(); },
                 [state](HeavyJobRejection reason) { state->notStarted(reason); }});
        return future;
    }

    // Unlike a raw atomic store, cancel also removes pending work immediately.
    // Running work observes the same atomic flag at its cooperative checkpoints.
    void cancel(const std::shared_ptr<std::atomic_bool>& cancellation) noexcept;
    // Reject queued jobs, cancel the running job, and join it. Call from an
    // owning thread, never from this scheduler's work callback. Idempotent.
    void shutdown();
    std::size_t pendingCount() const;
    bool isPending(const std::shared_ptr<std::atomic_bool>& cancellation) const;

private:
    struct Entry {
        std::shared_ptr<std::atomic_bool> cancellation;
        std::function<void()> work;
        std::function<void(HeavyJobRejection)> reject;
    };
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    void enqueue(Entry entry);
};

HeavyJobScheduler& heavyJobScheduler();

template <class Work, class OnNotStarted>
auto submitHeavyJob(std::shared_ptr<std::atomic_bool> cancellation,
                    Work&& work, OnNotStarted&& onNotStarted)
{
    return heavyJobScheduler().submit(std::move(cancellation),
                                     std::forward<Work>(work),
                                     std::forward<OnNotStarted>(onNotStarted));
}

// Cancelling an unsubmitted token never constructs the global worker.
void cancelHeavyJob(const std::shared_ptr<std::atomic_bool>& cancellation) noexcept;

// Promise futures do not join on destruction, unlike std::async futures.
// Install this guard wherever work borrows progress variables or other local
// state, so an exceptional exit cannot leave callbacks pointing at dead objects.
template <class Result>
class HeavyJobWaitGuard {
public:
    HeavyJobWaitGuard(std::future<Result>& future,
                      std::shared_ptr<std::atomic_bool> cancellation,
                      HeavyJobScheduler& scheduler = heavyJobScheduler()) noexcept
        : m_future(future), m_cancellation(std::move(cancellation)),
          m_scheduler(scheduler) {}
    ~HeavyJobWaitGuard()
    {
        if (m_future.valid()) {
            m_scheduler.cancel(m_cancellation);
            m_future.wait();
        }
    }
    HeavyJobWaitGuard(const HeavyJobWaitGuard&) = delete;
    HeavyJobWaitGuard& operator=(const HeavyJobWaitGuard&) = delete;
private:
    std::future<Result>& m_future;
    std::shared_ptr<std::atomic_bool> m_cancellation;
    HeavyJobScheduler& m_scheduler;
};

} // namespace oap::toolpath
