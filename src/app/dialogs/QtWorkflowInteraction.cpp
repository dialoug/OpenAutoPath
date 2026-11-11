// QtWorkflowInteraction 实现:模态进度对话框 + 忙碌光标。
// 进度由后台任务经快照回调驱动,轮询(30ms)驱动事件循环使取消按钮可用。
#include "app/dialogs/QtWorkflowInteraction.h"
#include <QCoreApplication>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include <QApplication>
#include <QEventLoop>
#include <QProgressDialog>
#include <QTimer>

namespace oap::app {
namespace {

// 同一交互端口的显式忙状态与进度对话框共享层数,避免内层进度结束
// 提前恢复外层任务的光标/按钮状态。多余的 false 不弹出别人的光标。
struct BusyState {
    WorkflowBusyStateHandler observer;
    std::size_t depth = 0;

    void set(bool busy)
    {
        if (busy) {
            if (depth++ != 0)
                return;
            QApplication::setOverrideCursor(Qt::WaitCursor);
            if (observer)
                observer(true);
        } else {
            if (depth == 0 || --depth != 0)
                return;
            QApplication::restoreOverrideCursor();
            if (observer)
                observer(false);
        }
    }
};

struct ProgressBusyGuard {
    explicit ProgressBusyGuard(BusyState& state) : state(state)
    {
        state.set(true);
    }
    ~ProgressBusyGuard() { state.set(false); }
    BusyState& state;
};

// 把 size_t 进度值裁剪到 int 可表示范围,避免溢出 QProgressDialog。
int progressValue(std::size_t value)
{
    return static_cast<int>(std::min<std::size_t>(
        value, static_cast<std::size_t>(std::numeric_limits<int>::max())));
}

// 模态显示进度对话框:用 QEventLoop + QTimer 轮询任务快照直到 ready()。
// 取消(按钮或外部标志)只发送一次 request.cancel(),随后禁用取消入口。
void showModalProgress(
    QWidget* parent,
    const WorkflowProgressRequest& request)
{
    if (!request.ready || request.ready())
        return;

    const int initialMaximum = request.indeterminate
        ? 0
        : std::max(1, progressValue(request.initialTotal));
    QProgressDialog progress(
        request.label, QCoreApplication::translate("OpenAutoPath", "取消"),
        0, initialMaximum, parent);
    progress.setWindowTitle(request.title);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.setAutoClose(false);
    // Reaching the current maximum does not necessarily mean that the
    // background task is finished: post-processing may still be running.
    // Keep QProgressDialog from resetting itself back to zero in that case.
    progress.setAutoReset(false);

    QEventLoop loop;
    QTimer poll;
    bool cancellationSent = false; // 保证 cancel 回调只触发一次
    QString displayedLabel = request.label;
    QObject::connect(&poll, &QTimer::timeout, &loop, [&] {
        QString nextLabel = request.label;
        if (request.snapshot) {
            const WorkflowProgressSnapshot snapshot = request.snapshot();
            if (!request.indeterminate) {
                const int maximum = std::max(1, progressValue(snapshot.total));
                progress.setMaximum(maximum);
                progress.setValue(std::min(
                    maximum, progressValue(snapshot.completed)));
            }
            if (!snapshot.label.isEmpty())
                nextLabel = snapshot.label;
        }
        if (request.queued && request.queued())
            nextLabel = QCoreApplication::translate("OpenAutoPath", "正在等待其他计算任务完成，可取消本次请求…");
        if (!cancellationSent && nextLabel != displayedLabel) {
            displayedLabel = nextLabel;
            progress.setLabelText(displayedLabel);
        }

        const bool externallyCancelled =
            request.cancellationRequested &&
            request.cancellationRequested();
        if ((progress.wasCanceled() || externallyCancelled) &&
            !cancellationSent) {
            cancellationSent = true;
            if (request.cancel)
                request.cancel();
            progress.setCancelButton(nullptr);
            progress.setWindowFlag(Qt::WindowCloseButtonHint, false);
            if (!request.cancellingLabel.isEmpty())
                progress.setLabelText(request.cancellingLabel);
            progress.show();
        }
        if (cancellationSent && !progress.isVisible())
            progress.show();
        if (request.ready())
            loop.quit();
    });
    poll.start(30);
    loop.exec();
    poll.stop();
    progress.close();
}

} // namespace

// 组装 WorkflowInteraction:showProgress 用模态进度对话框,setBusy 切等待光标。
WorkflowInteraction makeQtWorkflowInteraction(
    QWidget* parent,
    WorkflowBusyStateHandler onBusy,
    std::function<bool()> externalCancellation)
{
    const auto busy = std::make_shared<BusyState>();
    busy->observer = std::move(onBusy);
    WorkflowInteraction interaction;
    interaction.showProgress = [parent, busy,
                                externalCancellation = std::move(externalCancellation)](
                                   const WorkflowProgressRequest& request) {
        ProgressBusyGuard guard(*busy);
        WorkflowProgressRequest merged = request;
        if (externalCancellation) {
            merged.cancellationRequested =
                [requested = request.cancellationRequested, externalCancellation] {
                    return (requested && requested()) || externalCancellation();
                };
        }
        showModalProgress(parent, merged);
    };
    interaction.setBusy = [busy](bool value) {
        busy->set(value);
    };
    return interaction;
}

} // namespace oap::app
