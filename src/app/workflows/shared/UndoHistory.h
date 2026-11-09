// 主窗口会话撤销历史：按时间统一保存可见性和加工 Job 快照。
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <QString>

#include "manufacturing/job/MachiningJob.h"

namespace oap::app {

// 一条撤销记录：可见性快照，或包含定义、运行期刀路
// 与统一 IPW 的完整加工 Job 快照。
struct UndoRecord {
    enum class Kind {
        Visibility,
        MachiningJob,
    };

    Kind kind = Kind::Visibility;
    std::unordered_set<std::string> hiddenFaces;
    std::optional<manufacturing::MachiningJob> machiningJob;
    QString description;
};

// 撤销历史:按时间保存记录,容量受限(超出淘汰最旧记录);
// takeLast/putBack 配合实现"撤销执行可回退"的事务语义。
class CommandHistory {
public:
    // 记录一次可见性更改(隐藏面集合的快照)。
    void pushVisibility(std::unordered_set<std::string> hiddenFaces);
    // 记录完整加工 Job（定义、运行期刀路与统一 IPW）供工序管理撤销。
    void pushMachiningJob(
        manufacturing::MachiningJob job,
        QString description);
    // 弹出最近一条记录用于执行撤销;为空时返回 std::nullopt。
    std::optional<UndoRecord> takeLast();
    // 撤销执行未提交时把刚弹出的记录原位放回，不消耗用户的撤销点。
    void putBack(UndoRecord record);
    // 清空全部历史(切换模型/会话时调用)。
    void clear() noexcept;
    bool empty() const noexcept;
    std::size_t size() const noexcept { return m_records.size(); }

private:
    void push(UndoRecord record);

    static constexpr std::size_t kLimit = 50;
    std::vector<UndoRecord> m_records;
};

// 为已归档的移植笔记与树外调用方保留的兼容名称。
using UndoHistory = CommandHistory;

} // namespace oap::app
