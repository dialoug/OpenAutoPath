#pragma once

#include <string>
#include <vector>

#include "toolpath/finishing/SurfaceFinishing.h"

namespace oap::app {

// Serialized values 0..2 are retained for existing user preferences.
enum class EngravingSetupScope {
    AllSixFaces = 0, PositiveZ = 1, PreserveProjectFaces = 2,
    NegativeX = 3, PositiveX = 4, NegativeY = 5, PositiveY = 6, NegativeZ = 7
};
enum class EngravingToolMode { AutomaticLibrary, SingleTool };

// Process configuration for the normal CAM workflow. An empty library ID uses
// the active library; SingleTool freezes the explicitly entered tool context.
struct EngravingSettings {
    toolpath::SurfaceFinishingParams parameters = [] {
        toolpath::SurfaceFinishingParams value;
        value.stepOver = 0.4;
        value.sampleSpacing = 0.25;
        value.meshDeflection = 0.03;
        return value;
    }();
    EngravingSetupScope scope = EngravingSetupScope::AllSixFaces;
    EngravingToolMode toolMode = EngravingToolMode::AutomaticLibrary;
    std::string libraryId;
    toolpath::ToolpathToolContext manualTool;
    std::vector<MoldSetupFace> projectFaces;
};

} // namespace oap::app
