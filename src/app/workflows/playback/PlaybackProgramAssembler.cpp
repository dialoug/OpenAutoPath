// PlaybackProgramAssembler 实现：程序展平、预览抽稀与换刀批次划分。
#include "app/workflows/playback/PlaybackProgramAssembler.h"

#include "toolpath/core/ToolpathPrimitiveMapping.h"

#include <algorithm>
#include <cmath>

#include "recognition/FeatureRecognizer.h"

namespace oap::app {
namespace {

// 孔类特征（通孔/盲孔/沉孔）判定，用于换刀批次分组。
bool isHoleKind(FeatureKind kind) noexcept
{
    return kind == FeatureKind::ThroughHole ||
        kind == FeatureKind::BlindHole ||
        kind == FeatureKind::CountersinkHole;
}

// 从工序元数据解析孔加工策略；缺省回退 SmallDrill。
toolpath::HoleMachiningStrategy holeStrategy(
    const toolpath::ToolpathOperation& operation)
{
    const auto found = operation.metadata.find("strategy");
    if (found == operation.metadata.end())
        return toolpath::HoleMachiningStrategy::SmallDrill;
    if (found->second == "medium_helical")
        return toolpath::HoleMachiningStrategy::MediumHelical;
    if (found->second == "large_layered_spiral")
        return toolpath::HoleMachiningStrategy::LargeLayeredSpiral;
    if (found->second == "auto")
        return toolpath::HoleMachiningStrategy::Auto;
    return toolpath::HoleMachiningStrategy::SmallDrill;
}

// 判断 current 是否开启新换刀批次：装夹面变化、孔/腔类型切换、
// 刀具号或工序阶段变化都视为新批次。
bool startsNewToolBatch(
    const PlaybackMove* previous,
    const PlaybackMove& current) noexcept
{
    if (!previous)
        return true;
    if (previous->setupIndex != current.setupIndex)
        return true;
    if (previous->operationType != current.operationType)
        return true;
    if (previous->tool.valid != current.tool.valid)
        return true;
    if (!current.tool.valid)
        return previous->phase != current.phase;

    return previous->tool.toolNumber != current.tool.toolNumber ||
        previous->phase != current.phase;
}

} // namespace

std::vector<toolpath::ToolpathMoveSemantic>
PlaybackProgramAssembler::mapMoveSemantics(
    const std::vector<toolpath::ToolpathMove>& moves,
    const std::vector<toolpath::ToolpathPrimitive>& primitives)
{
    return toolpath::mapToolpathMovesToPrimitives(moves, primitives);
}

PlaybackProgramSnapshot PlaybackProgramAssembler::build(
    const toolpath::ToolpathProgram& program,
    const PlaybackProgramBuildOptions& options)
{
    PlaybackProgramSnapshot snapshot;
    std::size_t totalMoveCount = 0u;
    int totalPocketCutMoves = 0;
    if (options.limitPocketCutMoves) {
        for (const toolpath::ToolpathOperation& operation :
             program.operations) {
            totalMoveCount += operation.moves.size();
            if (operation.type !=
                toolpath::ToolpathOperationType::PocketClearing) {
                continue;
            }
            for (const toolpath::ToolpathMove& move : operation.moves) {
                if (move.type == toolpath::ToolpathMoveType::Cut)
                    ++totalPocketCutMoves;
            }
        }
    } else {
        totalMoveCount = program.moveCount();
    }
    // 口袋预览预算与具体路径模式无关。Cut 移动超过预算时按
    // ceil(总数/预算) 均匀抽样，快速/进退刀移动始终保留。
    const int budget = std::max(1, options.previewCutMoveBudget);
    const int cutStride =
        options.limitPocketCutMoves && totalPocketCutMoves > budget
            ? std::max(2, static_cast<int>(std::ceil(
                  static_cast<double>(totalPocketCutMoves) / budget)))
            : 1;

    // 装夹面数量：仅孔加工工序的 setupIndex 计入，供显示层切换使用。
    int setupCount = 0;
    for (const toolpath::ToolpathOperation& operation : program.operations) {
        if (operation.setupFace != MoldSetupFace::Unresolved)
            setupCount = std::max(setupCount, operation.setupIndex + 1);
    }

    const std::size_t sampledPocketCuts = cutStride > 1
        ? (static_cast<std::size_t>(totalPocketCutMoves) +
           static_cast<std::size_t>(cutStride) - 1u) /
              static_cast<std::size_t>(cutStride)
        : static_cast<std::size_t>(totalPocketCutMoves);
    const std::size_t reserveCount = cutStride > 1
        ? totalMoveCount - static_cast<std::size_t>(totalPocketCutMoves) +
              sampledPocketCuts
        : totalMoveCount;
    snapshot.moves.reserve(reserveCount);
    int cutOrdinal = 0;
    for (int operationIndex = 0;
         operationIndex < static_cast<int>(program.operations.size());
         ++operationIndex) {
        const toolpath::ToolpathOperation& operation =
            program.operations[operationIndex];
        const bool lightweightPocketSemantics =
            cutStride > 1 &&
            operation.type ==
                toolpath::ToolpathOperationType::PocketClearing;
        const std::vector<toolpath::ToolpathMoveSemantic> semantics =
            lightweightPocketSemantics
                ? std::vector<toolpath::ToolpathMoveSemantic>{}
                : mapMoveSemantics(
                      operation.moves, operation.primitives);
        const bool isHole =
            operation.type == toolpath::ToolpathOperationType::HoleMachining;
        for (int operationMoveIndex = 0;
             operationMoveIndex < static_cast<int>(operation.moves.size());
             ++operationMoveIndex) {
            const toolpath::ToolpathMove& move =
                operation.moves[static_cast<std::size_t>(operationMoveIndex)];
            if (options.limitPocketCutMoves &&
                operation.type ==
                    toolpath::ToolpathOperationType::PocketClearing &&
                move.type == toolpath::ToolpathMoveType::Cut &&
                (cutOrdinal++ % cutStride) != 0) {
                continue;
            }

            PlaybackMove playback;
            playback.move = move;
            if (!semantics.empty()) {
                playback.semantic = semantics[
                    static_cast<std::size_t>(operationMoveIndex)];
            } else {
                // Avoid allocating diagnostic strings for every source move
                // in a multi-million-line display path.
                playback.semantic.commandName =
                    move.type == toolpath::ToolpathMoveType::Rapid
                        ? "G0" : "G1";
            }
            playback.operationId = operation.operationId;
            playback.operationMoveIndex = operationMoveIndex;
            playback.operationType = operation.type;
            playback.color = options.verticalColor;
            // 配色约定：Cut 用特征色，Rapid 用孔/腔各自的快速移动色。
            if (move.type == toolpath::ToolpathMoveType::Cut)
                playback.color = operation.type == toolpath::ToolpathOperationType::SurfaceFinishing
                    ? Quantity_Color(0.30, 0.86, 0.72, Quantity_TOC_sRGB)
                    : featureColor(operation.featureKind);
            else if (move.type == toolpath::ToolpathMoveType::Rapid)
                playback.color = isHole
                    ? options.holeRapidColor : options.pocketRapidColor;
            playback.kind = operation.featureKind;
            playback.holeStrategy = isHole
                ? holeStrategy(operation)
                : toolpath::HoleMachiningStrategy::SmallDrill;
            playback.sourceIndex = operation.sourceIndex >= 0
                ? operation.sourceIndex : operationIndex;
            const bool hasSetup =
                operation.setupFace != MoldSetupFace::Unresolved;
            playback.setupIndex = hasSetup ? operation.setupIndex : -1;
            playback.setupCount = hasSetup ? setupCount : 0;
            playback.setupFace = operation.setupFace;
            playback.phase = isHole
                ? operation.holePhase
                : toolpath::HoleMachiningPhase::Standard;
            playback.tool = operation.tool;
            const PlaybackMove* previous = snapshot.moves.empty()
                ? nullptr : &snapshot.moves.back();
            // 批次切分：新批次在此记录起始下标，随后为当前移动打上批次号。
            if (startsNewToolBatch(previous, playback)) {
                snapshot.toolBatchStarts.push_back(
                    static_cast<int>(snapshot.moves.size()));
            }
            playback.toolBatchIndex =
                static_cast<int>(snapshot.toolBatchStarts.size()) - 1;
            snapshot.moves.push_back(std::move(playback));
        }
    }
    return snapshot;
}

void PlaybackProgramAssembler::replaceProgram(
    PlaybackSession& session,
    const toolpath::ToolpathProgram& program,
    const PlaybackProgramBuildOptions& options)
{
    PlaybackProgramSnapshot snapshot = build(program, options);
    session.clearProgram();
    session.moves() = std::move(snapshot.moves);
    session.toolBatchStarts() = std::move(snapshot.toolBatchStarts);
}

} // namespace oap::app
