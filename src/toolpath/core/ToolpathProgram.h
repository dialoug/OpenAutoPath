// 程序级刀路中间表示。具体几何生成器仍可保留自己的丰富结果类型，
// 在预览、统一验证和后处理边界处再投影为该模型。
#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "analysis/FeatureTypes.h"
#include "ToolpathMoves.h"
#include "ToolpathPrimitives.h"
#include "ToolpathToolContext.h"

namespace oap::toolpath {

enum class ToolpathOperationType {
    Generic,
    PocketClearing,
    ProfileContour,
    HoleMachining,
    SurfaceFinishing,
};

// 单个工序：一个几何源（特征/区域）加上刀具快照和该工序的完整移动序列。
// operationId 在程序内唯一，是验证诊断、进度回调与后处理追溯的稳定句柄；
// primitives 非空时优先作为语义流，moves 始终保留为预览/回退轨迹。
struct ToolpathOperation {
    std::string operationId;
    ToolpathOperationType type = ToolpathOperationType::Generic;
    FeatureKind featureKind = FeatureKind::Pocket;
    int sourceIndex = -1;
    int setupIndex = 0;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    HoleMachiningPhase holePhase = HoleMachiningPhase::Standard;
    ToolpathToolContext tool;
    // 工序几何要求的最小有效刃长；0 表示该类工序未提供此约束。
    double minimumCuttingEdgeLength = 0.0;
    std::vector<std::string> sourceFeatureIds;
    std::map<std::string, std::string> metadata;
    std::vector<ToolpathMove> moves;
    // 非空时是后处理的首选有序语义流；moves 始终保留为预览和回退轨迹。
    std::vector<ToolpathPrimitive> primitives;
};

// 程序级刀路集合：按工序顺序持有全部操作。值语义容器，由生成器构建、
// 验证器/后处理只读消费；可通过扁平化移动序列做统一线性校验。
struct ToolpathProgram {
    std::vector<ToolpathOperation> operations;

    // 无工序或总移动数为零时视为空程序。
    bool empty() const noexcept;
    // 仅统计 moves 条数（不展开 primitives）。
    std::size_t moveCount() const noexcept;
    // 按工序顺序拼接全部 moves，供预览和线性安全校验使用。
    std::vector<ToolpathMove> flattenedMoves() const;
    // 线性查找首个匹配的 operationId，未找到返回 nullptr。
    const ToolpathOperation* operationById(const std::string& operationId) const noexcept;
};

enum class OperationToolContextMode {
    None,         // 所有工序都使用程序级手动刀具参数。
    PerOperation, // 所有工序都携带完整刀具快照。
    Mixed         // 两种模式混用；后处理必须拒绝，避免沿用错误刀具。
};

OperationToolContextMode operationToolContextMode(
    const ToolpathProgram& program) noexcept;

// 按 setupIndex 首次出现顺序拆分程序；每个结果只包含一个装夹，供尚未
// 建立六面坐标变换时安全地逐装夹导出。
std::vector<ToolpathProgram> splitToolpathProgramBySetup(
    const ToolpathProgram& program);

// 参考 NC 文件按装夹、刀具和孔加工阶段分别输出，防止一个文件在没有
// T/M6/G43 的情况下跨刀具或跨工序阶段运行。结果保持各组首次出现顺序。
std::vector<ToolpathProgram> splitToolpathProgramBySetupToolAndPhase(
    const ToolpathProgram& program);

const char* toolpathOperationTypeName(ToolpathOperationType type) noexcept;

} // namespace oap::toolpath
