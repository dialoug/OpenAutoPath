#pragma once

#include "analysis/StepIndexer.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::manufacturing {

// Capture receives the complete target unit set used to generate the result.
// An empty replaceOperationId appends a new operation; a nonempty one edits it.
struct SurfaceFinishingJobCaptureRequest {
    const toolpath::SurfaceFinishingResult* result = nullptr;
    const toolpath::ToolpathToolContext* tool = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    std::vector<std::string> unitIds;
    std::string replaceOperationId;
    std::string libraryId;
};

class SurfaceFinishingMachiningJobService {
public:
    // Reconstructs only the persistent planning stock. Simulation overrides and
    // IPW runtime checkpoints are deliberately not inputs. External STEP content
    // is checked and its identity is recorded on the candidate definition.
    static TopoDS_Shape buildPlanningStock(
        StockDefinition& stock, const std::vector<TopoDS_Shape>& completeTargetUnits,
        std::string* error = nullptr,
        const toolpath::ToolpathGenerationControl* control = nullptr);
    bool capture(MachiningJob& job, const SurfaceFinishingJobCaptureRequest& request,
                 std::string* error = nullptr) const;
    // Atomically replaces all engraving faces, retaining IDs and sequence for
    // existing matching faces and preserving all other operation families.
    bool replaceFamily(MachiningJob& job,
        const std::vector<SurfaceFinishingJobCaptureRequest>& requests,
        std::string* error = nullptr) const;
    toolpath::SurfaceFinishingBatch regenerate(
        MachiningJob& job, const StepIndexer& indexer,
        const toolpath::ToolpathGenerationControl* control = nullptr) const;
    static std::string operationFingerprint(
        const MachiningJobDefinition& definition,
        const SurfaceFinishingOperationDefinition& operation);
};

} // namespace oap::manufacturing
