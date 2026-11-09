#include "app/workflows/playback/PlaybackSession.h"
#include "app/workflows/playback/PlaybackProgramAssembler.h"

#include <cmath>
#include <iostream>
#include <string_view>

namespace {

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool sameColor(const Quantity_Color& left, const Quantity_Color& right)
{
    return std::abs(left.Red() - right.Red()) < 1.0e-9 &&
        std::abs(left.Green() - right.Green()) < 1.0e-9 &&
        std::abs(left.Blue() - right.Blue()) < 1.0e-9;
}

} // namespace

int main()
{
    oap::app::PlaybackSession session;
    session.moves().resize(5);
    session.toolBatchStarts() = {0, 2, 4};
    if (!expect(session.toolBatchRange(1) == std::pair{2, 4},
                "tool batch range must be stable")) {
        return 1;
    }

    const auto first = session.requestCursor(1, 0.25);
    if (!expect(first.dispatch && first.dispatch->index == 1,
                "the first cursor request must dispatch")) {
        return 1;
    }
    const auto coalesced = session.requestCursor(3, 0.75);
    if (!expect(!coalesced.dispatch,
                "an in-flight cursor request must be coalesced")) {
        return 1;
    }
    const auto next = session.completeCursorRequest();
    if (!expect(next && next->index == 3 && next->fraction == 0.75,
                "the newest pending cursor request must win")) {
        return 1;
    }
    // completeCursorRequest() 已派发移动 3，因此仍有一个 worker 请求在途。
    // 回退必须立即取代该过期请求，而不是等待其结果。
    session.trailFlushIndex() = 4;
    const auto rewound = session.requestCursor(2, 1.0);
    if (!expect(rewound.rewound && rewound.dispatch &&
                    rewound.dispatch->index == 2 &&
                    session.trailFlushIndex() == 2,
                "rewind must supersede an in-flight cursor request")) {
        return 1;
    }
    session.cursorDispatchFailed();

    session.clearProgram();
    if (!expect(session.moves().empty() &&
                    session.toolBatchStarts().empty() &&
                    session.currentToolBatchIndex() == -1,
                "program clear must reset program-scoped playback state")) {
        return 1;
    }

    oap::toolpath::ToolpathProgram holeProgram;
    const auto makeHoleOperation = [](
        oap::toolpath::HoleMachiningPhase phase,
        oap::toolpath::ToolpathMoveType moveType) {
        oap::toolpath::ToolpathOperation operation;
        operation.type = oap::toolpath::ToolpathOperationType::HoleMachining;
        operation.featureKind = oap::FeatureKind::CountersinkHole;
        operation.setupIndex = 0;
        operation.setupFace = oap::MoldSetupFace::PositiveZ;
        operation.holePhase = phase;
        operation.tool.valid = true;
        operation.tool.toolNumber = 7;
        operation.metadata["strategy"] = "medium_helical";
        oap::toolpath::ToolpathMove move;
        move.type = moveType;
        move.to = gp_Pnt(1.0, 0.0, 0.0);
        operation.moves.push_back(move);
        return operation;
    };
    holeProgram.operations.push_back(makeHoleOperation(
        oap::toolpath::HoleMachiningPhase::CenterSpot,
        oap::toolpath::ToolpathMoveType::Rapid));
    holeProgram.operations.push_back(makeHoleOperation(
        oap::toolpath::HoleMachiningPhase::Drill,
        oap::toolpath::ToolpathMoveType::Cut));
    holeProgram.operations.push_back(makeHoleOperation(
        oap::toolpath::HoleMachiningPhase::FlatEndMill,
        oap::toolpath::ToolpathMoveType::Plunge));
    holeProgram.operations[0].operationId = "center-spot-op";
    holeProgram.operations[1].operationId = "drill-op";
    holeProgram.operations[2].operationId = "mill-op";
    oap::toolpath::ToolpathDrillCycle peckCycle;
    peckCycle.type = oap::toolpath::ToolpathDrillCycleType::Peck;
    peckCycle.peckDepth = 2.0;
    holeProgram.operations[1].primitives.push_back(
        oap::toolpath::makeDrillCyclePrimitive(
            peckCycle, holeProgram.operations[1].moves));

    const oap::app::PlaybackProgramSnapshot holeSnapshot =
        oap::app::PlaybackProgramAssembler::build(holeProgram);
    if (!expect(holeSnapshot.moves.size() == 3 &&
                    holeSnapshot.toolBatchStarts == std::vector<int>({0, 1, 2}),
                "automatic hole phases each begin their own tool batch")) {
        return 1;
    }
    if (!expect(
            holeSnapshot.moves[0].toolBatchIndex == 0 &&
                holeSnapshot.moves[1].toolBatchIndex == 1 &&
                holeSnapshot.moves[2].toolBatchIndex == 2 &&
                holeSnapshot.moves[1].operationId == "drill-op" &&
                holeSnapshot.moves[1].operationMoveIndex == 0 &&
                holeSnapshot.moves[1].semantic.commandName == "G83" &&
                holeSnapshot.moves[1].semantic.primitiveIndex == 0,
            "assembled moves must carry stable automatic batch identities")) {
        return 1;
    }
    const oap::app::PlaybackProgramBuildOptions defaultColors;
    if (!expect(sameColor(holeSnapshot.moves[0].color,
                          defaultColors.holeRapidColor) &&
                    sameColor(holeSnapshot.moves[2].color,
                              defaultColors.verticalColor),
                "hole rapid and vertical moves must use semantic playback colors")) {
        return 1;
    }
    if (!expect(
            holeSnapshot.moves[0].holeStrategy ==
                oap::toolpath::HoleMachiningStrategy::MediumHelical,
            "hole strategy metadata must survive playback assembly")) {
        return 1;
    }

    oap::toolpath::ToolpathProgram mixedProgram;
    oap::toolpath::ToolpathOperation unboundHole;
    unboundHole.type = oap::toolpath::ToolpathOperationType::HoleMachining;
    unboundHole.featureKind = oap::FeatureKind::BlindHole;
    unboundHole.setupIndex = 0;
    unboundHole.setupFace = oap::MoldSetupFace::PositiveZ;
    unboundHole.moves.push_back(oap::toolpath::ToolpathMove{});
    oap::toolpath::ToolpathOperation unboundPocket;
    unboundPocket.type = oap::toolpath::ToolpathOperationType::PocketClearing;
    unboundPocket.featureKind = oap::FeatureKind::Pocket;
    unboundPocket.setupIndex = 0;
    unboundPocket.setupFace = oap::MoldSetupFace::PositiveZ;
    unboundPocket.moves.push_back(oap::toolpath::ToolpathMove{});
    oap::toolpath::ToolpathOperation secondSetupPocket = unboundPocket;
    secondSetupPocket.setupIndex = 1;
    secondSetupPocket.setupFace = oap::MoldSetupFace::PositiveX;
    mixedProgram.operations.push_back(std::move(unboundHole));
    mixedProgram.operations.push_back(std::move(unboundPocket));
    mixedProgram.operations.push_back(std::move(secondSetupPocket));
    const oap::app::PlaybackProgramSnapshot mixedSnapshot =
        oap::app::PlaybackProgramAssembler::build(mixedProgram);
    if (!expect(
            mixedSnapshot.toolBatchStarts ==
                std::vector<int>({0, 1, 2}),
            "unbound hole/pocket families and pocket setups must remain distinct tool batches")) {
        return 1;
    }

    oap::toolpath::ToolpathProgram profileProgram;
    const auto appendProfile = [&profileProgram](
        const char* operationId, oap::MoldSetupFace setupFace) {
        oap::toolpath::ToolpathOperation operation;
        operation.operationId = operationId;
        operation.type =
            oap::toolpath::ToolpathOperationType::ProfileContour;
        operation.featureKind = oap::FeatureKind::Pocket;
        operation.setupFace = setupFace;
        operation.setupIndex = static_cast<int>(setupFace);
        operation.tool.valid = true;
        operation.tool.toolNumber = 6;
        operation.moves.push_back(oap::toolpath::ToolpathMove{});
        profileProgram.operations.push_back(std::move(operation));
    };
    appendProfile("PROFILE1", oap::MoldSetupFace::PositiveZ);
    appendProfile("PROFILE2", oap::MoldSetupFace::PositiveX);
    appendProfile("PROFILE3", oap::MoldSetupFace::NegativeY);
    appendProfile("PROFILE4", oap::MoldSetupFace::NegativeZ);
    const oap::app::PlaybackProgramSnapshot profileSnapshot =
        oap::app::PlaybackProgramAssembler::build(profileProgram);
    if (!expect(
            profileSnapshot.moves.size() == 4 &&
                profileSnapshot.toolBatchStarts ==
                    std::vector<int>({0, 1, 2, 3}) &&
                profileSnapshot.moves[0].operationId == "PROFILE1" &&
                profileSnapshot.moves[0].setupFace ==
                    oap::MoldSetupFace::PositiveZ,
            "profile playback must retain every setup, including the leading +Z operation")) {
        return 1;
    }

    oap::toolpath::ToolpathProgram pocketProgram;
    oap::toolpath::ToolpathOperation pocket;
    pocket.type = oap::toolpath::ToolpathOperationType::PocketClearing;
    pocket.featureKind = oap::FeatureKind::Pocket;
    for (int index = 0; index < 10; ++index) {
        oap::toolpath::ToolpathMove move;
        move.type = oap::toolpath::ToolpathMoveType::Cut;
        move.from = gp_Pnt(index, 0.0, 0.0);
        move.to = gp_Pnt(index + 1.0, 0.0, 0.0);
        pocket.moves.push_back(move);
    }
    pocketProgram.operations.push_back(std::move(pocket));
    oap::app::PlaybackProgramBuildOptions limited;
    limited.limitPocketCutMoves = true;
    limited.previewCutMoveBudget = 3;
    oap::app::PlaybackProgramAssembler::replaceProgram(
        session, pocketProgram, limited);
    if (!expect(session.moves().size() == 3 &&
                    session.toolBatchStarts() == std::vector<int>({0}) &&
                    session.moves()[0].operationMoveIndex == 0 &&
                    session.moves()[1].operationMoveIndex == 4 &&
                    session.moves()[2].operationMoveIndex == 8,
                "dense pocket playback must honor the preview cut budget")) {
        return 1;
    }

    std::cout << "PASS: playback session and program assembly checks\n";
    return 0;
}
