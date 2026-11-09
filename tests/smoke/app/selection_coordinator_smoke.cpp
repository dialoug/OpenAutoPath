#include "app/workflows/shared/SelectionCoordinator.h"

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
    oap::app::SelectionCoordinator coordinator;
    oap::app::SelectionSnapshot snapshot;
    snapshot.analysis = {"F1"};
    snapshot.features = {"F2", "F3"};
    coordinator.setSource(oap::app::SelectionSource::Features);
    if (!expect(
            coordinator.current(snapshot) ==
                std::vector<std::string>({"F2", "F3"}),
            "the active non-empty source must win")) {
        return 1;
    }

    snapshot.features.clear();
    if (!expect(
            coordinator.current(snapshot) ==
                std::vector<std::string>({"F1"}),
            "an empty source must fall back to analysis")) {
        return 1;
    }

    coordinator.reset();
    snapshot.analysis.clear();
    snapshot.groups = {"F4"};
    if (!expect(
            coordinator.current(snapshot) ==
                std::vector<std::string>({"F4"}),
            "stable fallback order must reach groups")) {
        return 1;
    }
    std::cout << "PASS: selection coordinator checks\n";
    return 0;
}
