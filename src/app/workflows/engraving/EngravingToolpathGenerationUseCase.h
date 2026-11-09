#pragma once

#include <QString>
#include <vector>

#include "app/workflows/engraving/EngravingSettings.h"
#include "app/workflows/project/MachiningProjectController.h"
#include "app/workflows/shared/WorkflowInteraction.h"
#include "tooling/library/ToolLibrary.h"

namespace oap::app {

struct EngravingToolpathGenerationRequest {
    WorkflowInteraction interaction;
    std::vector<TopoDS_Shape> targets;
    std::vector<std::string> unitIds;
    EngravingSettings settings;
    const tooling::ToolLibrary* library = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
    const manufacturing::MachiningJob* currentJob = nullptr;
    MachiningProjectDefinitionContext definition;
};

struct EngravingToolpathGenerationResult {
    bool ok = false;
    bool cancelled = false;
    QString error;
    QString summary;
    manufacturing::MachiningJob job;
    toolpath::SurfaceFinishingBatch batch;
    toolpath::ToolpathProgram program;
    toolpath::ToolpathToolContext tool;
    std::string libraryId;
};

// Builds all requested setup faces in one scheduled task. A complete candidate
// replaces the existing engraving family; failures leave the current Job intact.
class EngravingToolpathGenerationUseCase {
public:
    static EngravingToolpathGenerationResult execute(
        const EngravingToolpathGenerationRequest& request);
};

} // namespace oap::app
