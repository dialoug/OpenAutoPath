// 中立程序模型的查询、展平和按装夹拆分实现。
#include "ToolpathProgram.h"

#include <algorithm>

namespace oap::toolpath {

bool ToolpathProgram::empty() const noexcept
{
    return operations.empty() || moveCount() == 0;
}

std::size_t ToolpathProgram::moveCount() const noexcept
{
    std::size_t count = 0;
    for (const ToolpathOperation& operation : operations)
        count += operation.moves.size();
    return count;
}

std::vector<ToolpathMove> ToolpathProgram::flattenedMoves() const
{
    std::vector<ToolpathMove> result;
    result.reserve(moveCount());
    for (const ToolpathOperation& operation : operations)
        result.insert(result.end(), operation.moves.begin(), operation.moves.end());
    return result;
}

const ToolpathOperation* ToolpathProgram::operationById(
    const std::string& operationId) const noexcept
{
    const auto it = std::find_if(
        operations.begin(), operations.end(), [&](const ToolpathOperation& operation) {
            return operation.operationId == operationId;
        });
    return it == operations.end() ? nullptr : &*it;
}

OperationToolContextMode operationToolContextMode(
    const ToolpathProgram& program) noexcept
{
    // 逐工序扫描 tool.valid：只要同时出现绑定与未绑定刀具即判定 Mixed，
    // 后处理必须拒绝该模式，避免沿用错误刀具参数。
    bool hasBoundTool = false;
    bool hasUnboundTool = false;
    for (const ToolpathOperation& operation : program.operations) {
        hasBoundTool = hasBoundTool || operation.tool.valid;
        hasUnboundTool = hasUnboundTool || !operation.tool.valid;
        if (hasBoundTool && hasUnboundTool)
            return OperationToolContextMode::Mixed;
    }
    return hasBoundTool ? OperationToolContextMode::PerOperation
                        : OperationToolContextMode::None;
}

std::vector<ToolpathProgram> splitToolpathProgramBySetup(
    const ToolpathProgram& program)
{
    // 按 setupIndex 首次出现顺序建组，保持原始工序相对顺序。
    std::vector<int> setupIndices;
    std::vector<ToolpathProgram> result;
    for (const ToolpathOperation& operation : program.operations) {
        auto setup = std::find(
            setupIndices.begin(), setupIndices.end(), operation.setupIndex);
        if (setup == setupIndices.end()) {
            setupIndices.push_back(operation.setupIndex);
            result.emplace_back();
            setup = std::prev(setupIndices.end());
        }
        result[static_cast<std::size_t>(setup - setupIndices.begin())]
            .operations.push_back(operation);
    }
    return result;
}

std::vector<ToolpathProgram> splitToolpathProgramBySetupToolAndPhase(
    const ToolpathProgram& program)
{
    // 分组键＝装夹 + 刀具（快照绑定标志/刀号/刀库 ID）+ 孔加工阶段，
    // 防止单个文件在没有 T/M6/G43 的情况下跨刀具或跨工序阶段运行。
    struct GroupKey {
        int setupIndex = 0;
        bool boundTool = false;
        int toolNumber = 0;
        std::string toolId;
        HoleMachiningPhase phase = HoleMachiningPhase::Standard;

        bool operator==(const GroupKey& other) const
        {
            return setupIndex == other.setupIndex &&
                   boundTool == other.boundTool &&
                   toolNumber == other.toolNumber &&
                   toolId == other.toolId && phase == other.phase;
        }
    };

    std::vector<GroupKey> keys;
    std::vector<ToolpathProgram> result;
    for (const ToolpathOperation& operation : program.operations) {
        GroupKey key;
        key.setupIndex = operation.setupIndex;
        key.boundTool = operation.tool.valid;
        key.toolNumber = operation.tool.valid ? operation.tool.toolNumber : 0;
        key.toolId = operation.tool.valid ? operation.tool.toolId : std::string{};
        key.phase = operation.holePhase;
        auto group = std::find(keys.begin(), keys.end(), key);
        if (group == keys.end()) {
            keys.push_back(std::move(key));
            result.emplace_back();
            group = std::prev(keys.end());
        }
        result[static_cast<std::size_t>(group - keys.begin())]
            .operations.push_back(operation);
    }
    return result;
}

const char* toolpathOperationTypeName(ToolpathOperationType type) noexcept
{
    switch (type) {
        case ToolpathOperationType::Generic: return "generic";
        case ToolpathOperationType::PocketClearing: return "pocket_clearing";
        case ToolpathOperationType::ProfileContour: return "profile_contour";
        case ToolpathOperationType::HoleMachining: return "hole_machining";
        case ToolpathOperationType::SurfaceFinishing: return "surface_finishing";
    }
    return "unknown";
}

} // namespace oap::toolpath
