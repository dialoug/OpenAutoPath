#include "toolpath/core/HeavyJobScheduler.h"

#include <condition_variable>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace oap::toolpath {
namespace {
thread_local const void* currentScheduler = nullptr;
std::atomic<HeavyJobScheduler*> globalScheduler{nullptr};
}

const char* heavyJobRejectionMessage(HeavyJobRejection reason) noexcept
{
    switch (reason) {
    case HeavyJobRejection::Cancelled: return "Heavy job cancelled before execution";
    case HeavyJobRejection::QueueFull: return "Heavy job queue is full; retry after a running job finishes";
    case HeavyJobRejection::Stopping: return "Heavy job scheduler is stopping";
    case HeavyJobRejection::Reentrant: return "Nested heavy job submission is not supported; call the synchronous service";
    }
    return "Heavy job could not start";
}

struct HeavyJobScheduler::Impl {
    const std::size_t capacity;
    mutable std::mutex mutex;
    std::mutex shutdownMutex;
    std::condition_variable changed;
    std::deque<Entry> pending;
    std::shared_ptr<std::atomic_bool> activeCancellation;
    bool stopping = false;
    std::thread worker;

    explicit Impl(std::size_t pendingCapacity) : capacity(pendingCapacity)
    {
        worker = std::thread([this] { run(); });
    }

    void run()
    {
        currentScheduler = this;
        for (;;) {
            Entry entry;
            {
                std::unique_lock lock(mutex);
                changed.wait(lock, [this] { return stopping || !pending.empty(); });
                if (stopping)
                    break;
                entry = std::move(pending.front());
                pending.pop_front();
                activeCancellation = entry.cancellation;
            }
            if (entry.cancellation && entry.cancellation->load(std::memory_order_relaxed))
                entry.reject(HeavyJobRejection::Cancelled);
            else
                entry.work();
            {
                std::lock_guard lock(mutex);
                activeCancellation.reset();
            }
        }
        currentScheduler = nullptr;
    }
};

HeavyJobScheduler::HeavyJobScheduler(std::size_t pendingCapacity)
    : m_impl(std::make_unique<Impl>(pendingCapacity)) {}

HeavyJobScheduler::~HeavyJobScheduler() { shutdown(); }

void HeavyJobScheduler::enqueue(Entry entry)
{
    std::optional<HeavyJobRejection> reason;
    {
        std::lock_guard lock(m_impl->mutex);
        if (entry.cancellation && entry.cancellation->load(std::memory_order_relaxed))
            reason = HeavyJobRejection::Cancelled;
        else if (m_impl->stopping)
            reason = HeavyJobRejection::Stopping;
        else if (currentScheduler == m_impl.get())
            reason = HeavyJobRejection::Reentrant;
        else if (m_impl->pending.size() >= m_impl->capacity)
            reason = HeavyJobRejection::QueueFull;
        else
            m_impl->pending.push_back(std::move(entry));
    }
    if (reason)
        entry.reject(*reason);
    else
        m_impl->changed.notify_one();
}

void HeavyJobScheduler::cancel(const std::shared_ptr<std::atomic_bool>& cancellation) noexcept
{
    if (!cancellation)
        return;
    cancellation->store(true, std::memory_order_relaxed);
    // Detach one item at a time: cancellation is also used by noexcept task
    // destructors and must not allocate a temporary collection of jobs.
    for (;;) {
        std::optional<Entry> cancelled;
        {
            std::lock_guard lock(m_impl->mutex);
            for (auto it = m_impl->pending.begin(); it != m_impl->pending.end(); ++it) {
                if (it->cancellation.get() == cancellation.get()) {
                    cancelled.emplace(std::move(*it));
                    m_impl->pending.erase(it);
                    break;
                }
            }
        }
        if (!cancelled)
            return;
        cancelled->reject(HeavyJobRejection::Cancelled);
    }
}

void HeavyJobScheduler::shutdown()
{
    if (currentScheduler == m_impl.get())
        throw std::logic_error("A heavy job cannot shut down its own scheduler");
    std::deque<Entry> rejected;
    {
        std::lock_guard lock(m_impl->mutex);
        m_impl->stopping = true;
        if (m_impl->activeCancellation)
            m_impl->activeCancellation->store(true, std::memory_order_relaxed);
        rejected.swap(m_impl->pending);
    }
    m_impl->changed.notify_one();
    for (auto& entry : rejected)
        entry.reject(HeavyJobRejection::Stopping);
    // Rejection callbacks run without any scheduler lock. Serialize only the
    // join, so a callback can itself cancel or shut down the scheduler safely.
    std::lock_guard shutdownLock(m_impl->shutdownMutex);
    if (m_impl->worker.joinable())
        m_impl->worker.join();
}

std::size_t HeavyJobScheduler::pendingCount() const
{
    std::lock_guard lock(m_impl->mutex);
    return m_impl->pending.size();
}

bool HeavyJobScheduler::isPending(
    const std::shared_ptr<std::atomic_bool>& cancellation) const
{
    if (!cancellation)
        return false;
    std::lock_guard lock(m_impl->mutex);
    for (const auto& entry : m_impl->pending) {
        if (entry.cancellation.get() == cancellation.get())
            return true;
    }
    return false;
}

HeavyJobScheduler& heavyJobScheduler()
{
    struct Holder {
        HeavyJobScheduler scheduler;
        Holder() { globalScheduler.store(&scheduler, std::memory_order_release); }
        ~Holder() { globalScheduler.store(nullptr, std::memory_order_release); }
    };
    static Holder holder;
    return holder.scheduler;
}

void cancelHeavyJob(const std::shared_ptr<std::atomic_bool>& cancellation) noexcept
{
    if (!cancellation)
        return;
    cancellation->store(true, std::memory_order_relaxed);
    if (auto* scheduler = globalScheduler.load(std::memory_order_acquire))
        scheduler->cancel(cancellation);
}

} // namespace oap::toolpath
