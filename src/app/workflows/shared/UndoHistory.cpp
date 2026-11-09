// UndoHistory 实现（oap_app_workflow_base）。
// 主窗口会话撤销历史：按时间统一保存可见性和加工 Job 操作，见 UndoHistory.h。
#include "app/workflows/shared/UndoHistory.h"

#include <utility>

namespace oap::app {

void CommandHistory::pushVisibility(
    std::unordered_set<std::string> hiddenFaces)
{
    UndoRecord record;
    record.kind = UndoRecord::Kind::Visibility;
    record.hiddenFaces = std::move(hiddenFaces);
    record.description = QStringLiteral("可见性更改");
    push(std::move(record));
}

void CommandHistory::pushMachiningJob(
    manufacturing::MachiningJob job,
    QString description)
{
    UndoRecord record;
    record.kind = UndoRecord::Kind::MachiningJob;
    record.machiningJob = std::move(job);
    record.description = std::move(description);
    push(std::move(record));
}

std::optional<UndoRecord> CommandHistory::takeLast()
{
    if (m_records.empty())
        return std::nullopt;
    UndoRecord record = std::move(m_records.back());
    m_records.pop_back();
    return record;
}

void CommandHistory::putBack(UndoRecord record)
{
    push(std::move(record));
}

void CommandHistory::clear() noexcept
{
    m_records.clear();
}

bool CommandHistory::empty() const noexcept
{
    return m_records.empty();
}

// 统一入栈:超限时淘汰队首(最旧)记录,保持历史容量有界。
void CommandHistory::push(UndoRecord record)
{
    m_records.push_back(std::move(record));
    if (m_records.size() > kLimit)
        m_records.erase(m_records.begin());
}

} // namespace oap::app
