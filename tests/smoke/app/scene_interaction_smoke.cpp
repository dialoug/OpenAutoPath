#include "app/scene/SceneInteractionCoordinator.h"
#include "analysis/ModelIndex.h"
#include "analysis/StepIndexer.h"

#include <iostream>
#include <string_view>

namespace {

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

} // namespace

int main()
{
    oap::app::SceneInteractionCoordinator coordinator;

    oap::app::SelectionSnapshot selection;
    selection.analysis = {"F1"};
    selection.features = {"F2", "F3"};
    coordinator.setSelectionSource(oap::app::SelectionSource::Features);
    if (!expect(
            coordinator.currentSelection(selection) ==
                std::vector<std::string>({"F2", "F3"}),
            "scene workspace must own selection-source policy")) {
        return 1;
    }

    oap::app::CommandHistory history;
    history.pushVisibility({"obsolete-face"});
    const oap::app::SceneUndoResult visibilityUndo =
        coordinator.takeUndo(history);
    if (!expect(
            visibilityUndo.restoredVisibility &&
                !visibilityUndo.deferredRecord,
            "visibility undo must be consumed inside the scene workspace")) {
        return 1;
    }

    history.pushMachiningJob(
        oap::manufacturing::MachiningJob{}, QStringLiteral("operation edit"));
    oap::app::SceneUndoResult deferred = coordinator.takeUndo(history);
    if (!expect(
            !deferred.restoredVisibility && deferred.deferredRecord &&
                deferred.deferredRecord->kind ==
                    oap::app::UndoRecord::Kind::MachiningJob,
            "non-visibility undo must be returned without reinterpretation")) {
        return 1;
    }
    history.putBack(std::move(*deferred.deferredRecord));
    if (!expect(history.size() == 1,
                "deferred undo must remain recoverable")) {
        return 1;
    }

    const oap::StepIndexer emptyStepIndexer;
    const oap::ModelIndex emptyModelIndex;
    const oap::app::ScenePickResult nullPick = coordinator.mapPick(
        TopoDS_Shape{}, 0, false, false, false, {},
        emptyStepIndexer, emptyModelIndex, {});
    if (!expect(!nullPick.accepted(), "null picks must be ignored"))
        return 1;

    std::cout << "PASS: scene interaction workspace checks\n";
    return 0;
}
