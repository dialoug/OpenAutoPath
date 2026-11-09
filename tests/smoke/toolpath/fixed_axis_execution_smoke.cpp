#include "toolpath/core/simulation/FixedAxisToolpathView.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/profile/simulation/ProfileSimulationAdapter.h"
#include "toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h"

#include <BRepPrimAPI_MakeBox.hxx>

#include <iostream>
#include <string>

namespace {
using namespace oap::toolpath;
int failures = 0;

void expect(bool value, const char* message)
{
    if (!value) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void equivalent(const MultiDexelApplyResult& native,
                const MultiDexelApplyResult& compatibility)
{
    expect(native.record.status == compatibility.record.status &&
           native.record.operationId == compatibility.record.operationId &&
           native.record.sequenceIndex == compatibility.record.sequenceIndex &&
           native.record.volumeAfter == compatibility.record.volumeAfter &&
           native.cuttingMoveCount == compatibility.cuttingMoveCount &&
           native.changedCellCount == compatibility.changedCellCount,
           "neutral execution preserves removal results and identity");
    expect(static_cast<bool>(native.stock) == static_cast<bool>(compatibility.stock),
           "neutral execution preserves publication state");
    if (native.stock && compatibility.stock)
        expect(native.stock->contentFingerprint() == compatibility.stock->contentFingerprint(),
               "neutral and legacy execution produce identical material intervals");
}
} // namespace

int main()
{
    StockSimulationOptions options;
    options.zMapResolution = 1.0;
    options.zMapMaxCellCount = 100000;
    options.zMapTileSize = 8;
    options.runParallel = false;
    std::string error;
    const auto stock = MultiDexelStock::fromShape(
        BRepPrimAPI_MakeBox(24.0, 20.0, 10.0).Shape(),
        gp_Vec(0, 0, 1), 4.0, options, nullptr, &error);
    if (!stock || !stock->valid()) {
        std::cerr << "cannot build fixture: " << error << '\n';
        return 1;
    }
    const auto original = stock->contentFingerprint();
    ProfileToolpathOperation profile;
    profile.operationId = "profile-native";
    profile.outwardToolAxis = gp_Vec(0, 0, 1);
    profile.contour.ok = true;
    ToolpathMove move;
    move.type = ToolpathMoveType::Cut;
    move.from = gp_Pnt(2, 10, 4);
    move.to = gp_Pnt(22, 10, 4);
    move.feed = 300;
    profile.contour.moves.push_back(move);
    move.from = move.to;
    move.to = gp_Pnt(22, 16, 4);
    profile.contour.moves.push_back(move);
    ToolpathParams params;
    params.tool.diameter = 4;
    params.tool.cuttingEdgeLength = 8;
    const auto view = profileSimulationView(profile);
    const auto legacy = legacyProfileSimulationOperation(profile);
    expect(view.moves.data() == profile.contour.moves.data() &&
           view.operationId.data() == profile.operationId.data(),
           "Profile adapter borrows storage without constructing a Pocket operation");
    const auto native = stock->applyFixedAxis(view, params, 7, options);
    expect(native.stock && native.record.status == StockRemovalStatus::Applied &&
           native.stock->volume() < stock->volume(), "native Profile path removes material");
    equivalent(native, stock->applyPocket(legacy, params, 7, options));

    DexelToolProfileDefinition tool;
    tool.diameter = 4;
    tool.cuttingLength = 8;
    tool.cornerRadius = 1;
    equivalent(stock->applyAxialToolpath(view, params, tool, 7, options),
               stock->applyAxialToolpath(legacy, params, tool, 7, options));
    for (const std::size_t count : {0u, 1u, 2u}) {
        equivalent(stock->applyToolpathPrefix(view, params, tool, 7, count, options),
                   stock->applyToolpathPrefix(legacy, params, tool, 7, count, options));
    }
    profile.outwardToolAxis = gp_Vec(1, 0, 0);
    const auto rotated = profileSimulationView(profile);
    const auto rotatedLegacy = legacyProfileSimulationOperation(profile);
    const auto oriented = stock->applyOrientedToolpath(rotated, params, tool, 8, options);
    expect(oriented.stock && oriented.record.status == StockRemovalStatus::Applied,
           "neutral oriented execution succeeds");
    equivalent(oriented, stock->applyOrientedToolpath(rotatedLegacy, params, tool, 8, options));
    equivalent(stock->applyToolpathPrefix(rotated, params, tool, 8, 1, options),
               stock->applyToolpathPrefix(rotatedLegacy, params, tool, 8, 1, options));
    const auto cancelled = stock->applyFixedAxis(view, params, 7, options, [] { return true; });
    expect(cancelled.record.status == StockRemovalStatus::Cancelled && !cancelled.stock,
           "cancelled neutral execution publishes no partial state");
    expect(stock->contentFingerprint() == original, "parent checkpoint stays immutable");
    return failures == 0 ? 0 : 1;
}
