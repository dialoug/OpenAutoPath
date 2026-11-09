#include "toolpath/core/HeavyJobScheduler.h"

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using namespace std::chrono_literals;
using oap::toolpath::HeavyJobRejection;
using oap::toolpath::HeavyJobScheduler;

bool expect(bool condition, const char* message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

int rejected(HeavyJobRejection reason)
{
    return -1 - static_cast<int>(reason);
}

auto cancellation()
{
    return std::make_shared<std::atomic_bool>(false);
}

// Always unblock before the scheduler's destructor, including failed checks.
struct Gate {
    std::promise<void> entered;
    std::promise<void> release;
    std::shared_future<void> released = release.get_future().share();
    bool opened = false;
    ~Gate() { open(); }
    void open()
    {
        if (!opened) { opened = true; release.set_value(); }
    }
    void block()
    {
        entered.set_value();
        if (released.wait_for(5s) != std::future_status::ready)
            throw std::runtime_error("test blocker timed out");
    }
    bool waitStarted()
    {
        return entered.get_future().wait_for(2s) == std::future_status::ready;
    }
};

bool boundedFifoAndSingleWorker()
{
    HeavyJobScheduler scheduler(3);
    Gate gate;
    std::thread::id worker;
    std::atomic_int executed{0};
    std::vector<int> order;
    auto first = scheduler.submit(cancellation(), [&] {
        worker = std::this_thread::get_id();
        gate.block();
        return 10;
    }, rejected);
    bool ok = expect(gate.waitStarted(), "first job must start");
    std::vector<std::future<int>> queued;
    for (int index = 0; index < 3; ++index) {
        queued.push_back(scheduler.submit(cancellation(), [&, index] {
            if (worker != std::this_thread::get_id())
                return -100;
            order.push_back(index);
            ++executed;
            return index;
        }, rejected));
    }
    auto overflow = scheduler.submit(cancellation(), [] { return 100; }, rejected);
    ok &= expect(overflow.wait_for(0s) == std::future_status::ready &&
                     overflow.get() == rejected(HeavyJobRejection::QueueFull),
                 "full queue returns a ready domain result without running work");
    ok &= expect(scheduler.pendingCount() == 3 && executed == 0,
                 "pending jobs remain unstarted behind the single running job");
    gate.open();
    ok &= expect(first.get() == 10, "running job result survives queue rejection");
    for (int index = 0; index < 3; ++index)
        ok &= expect(queued[index].get() == index, "queued work uses the same worker");
    ok &= expect(order == std::vector<int>({0, 1, 2}), "pending jobs start FIFO");
    return ok;
}

bool queuedCancelReleasesInputsAndGuardWaits()
{
    HeavyJobScheduler scheduler;
    Gate gate;
    auto first = scheduler.submit(cancellation(), [&] { gate.block(); return 1; }, rejected);
    bool ok = expect(gate.waitStarted(), "cancel test blocker must start");
    auto payload = std::make_shared<int>(7);
    std::weak_ptr<int> weak = payload;
    auto token = cancellation();
    std::atomic_bool ran{false};
    bool releasedBeforeRejection = false;
    auto queued = scheduler.submit(token, [payload, &ran] { ran = true; return *payload; },
        [&](HeavyJobRejection reason) {
            releasedBeforeRejection = weak.expired();
            return rejected(reason);
        });
    payload.reset();
    ok &= expect(scheduler.isPending(token) && !scheduler.isPending(nullptr),
                 "queued status identifies the submitted cancellation token");
    scheduler.cancel(token);
    ok &= expect(queued.wait_for(0s) == std::future_status::ready &&
                     queued.get() == rejected(HeavyJobRejection::Cancelled),
                 "pending cancellation completes while the earlier job remains blocked");
    ok &= expect(!ran && weak.expired() && releasedBeforeRejection,
                 "cancel releases captured inputs before completing its result");
    ok &= expect(!scheduler.isPending(token), "cancelled jobs no longer report queued");

    auto guardToken = cancellation();
    auto guarded = scheduler.submit(guardToken, [&] { ran = true; return 2; }, rejected);
    try {
        oap::toolpath::HeavyJobWaitGuard<int> guard(guarded, guardToken, scheduler);
        throw std::runtime_error("owner failed while waiting");
    } catch (const std::runtime_error&) {}
    ok &= expect(guarded.wait_for(0s) == std::future_status::ready &&
                     guarded.get() == rejected(HeavyJobRejection::Cancelled) && !ran,
                 "exceptional owner exit cancels and joins its queued job");
    gate.open();
    ok &= expect(first.get() == 1, "cancelling pending work leaves active work alone");
    return ok;
}

bool exceptionsReentryAndRawCancellation()
{
    HeavyJobScheduler scheduler;
    auto nested = scheduler.submit(cancellation(), [&] {
        return scheduler.submit(cancellation(), [] { return 1; }, rejected).get();
    }, rejected);
    bool ok = expect(nested.get() == rejected(HeavyJobRejection::Reentrant),
                     "nested submission fails promptly instead of deadlocking");
    auto failure = scheduler.submit(cancellation(), []() -> int {
        throw std::runtime_error("work failure");
    }, rejected);
    bool caught = false;
    try { (void)failure.get(); } catch (const std::runtime_error&) { caught = true; }
    ok &= expect(caught, "work exceptions reach the result future");
    auto cancelled = cancellation();
    cancelled->store(true);
    auto rejectionFailure = scheduler.submit(cancelled, [] { return 1; },
        [](HeavyJobRejection) -> int { throw std::runtime_error("rejection failure"); });
    caught = false;
    try { (void)rejectionFailure.get(); } catch (const std::runtime_error&) { caught = true; }
    ok &= expect(caught, "rejection callback exceptions also complete the future");

    Gate gate;
    auto first = scheduler.submit(cancellation(), [&] { gate.block(); return 1; }, rejected);
    ok &= expect(gate.waitStarted(), "raw cancellation test blocker must start");
    auto rawToken = cancellation();
    auto raw = scheduler.submit(rawToken, [] { return 9; }, rejected);
    rawToken->store(true);
    gate.open();
    (void)first.get();
    ok &= expect(raw.get() == rejected(HeavyJobRejection::Cancelled),
                 "legacy raw cancellation is checked before queued work starts");
    ok &= expect(!scheduler.isPending(rawToken), "completed jobs no longer report queued");
    auto recovered = scheduler.submit(cancellation(), [] { return 42; }, rejected);
    ok &= expect(recovered.get() == 42, "scheduler remains usable after failures");
    return ok;
}

bool shutdownCancelsAndCompletesEveryHandle()
{
    HeavyJobScheduler scheduler;
    auto activeToken = cancellation();
    std::promise<void> entered;
    auto active = scheduler.submit(activeToken, [&] {
        entered.set_value();
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!activeToken->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(1ms);
        return activeToken->load() ? 1 : 0;
    }, rejected);
    bool ok = expect(entered.get_future().wait_for(2s) == std::future_status::ready,
                     "shutdown test running job must start");
    auto pending = scheduler.submit(cancellation(), [] { return 100; },
        [&](HeavyJobRejection reason) {
            // Rejections must execute outside all scheduler locks, including
            // the lock serializing joins during shutdown.
            scheduler.shutdown();
            return rejected(reason);
        });
    scheduler.shutdown();
    ok &= expect(active.get() == 1, "shutdown cooperatively cancels and joins active work");
    ok &= expect(pending.get() == rejected(HeavyJobRejection::Stopping),
                 "shutdown completes pending handles without running them");
    auto late = scheduler.submit(cancellation(), [] { return 100; }, rejected);
    ok &= expect(late.get() == rejected(HeavyJobRejection::Stopping),
                 "submission after shutdown completes with a stopping result");
    scheduler.shutdown();
    return ok;
}
} // namespace

int main()
{
    try {
        auto unsubmitted = cancellation();
        oap::toolpath::cancelHeavyJob(nullptr);
        oap::toolpath::cancelHeavyJob(unsubmitted);
        if (!expect(unsubmitted->load(), "cancellation works before global scheduler startup"))
            return 1;
        bool ok = boundedFifoAndSingleWorker();
        ok &= queuedCancelReleasesInputsAndGuardWaits();
        ok &= exceptionsReentryAndRawCancellation();
        ok &= shutdownCancelsAndCompletesEveryHandle();
        return ok ? 0 : 1;
    } catch (const std::exception& failure) {
        std::cerr << "Unexpected test failure: " << failure.what() << '\n';
        return 1;
    }
}
