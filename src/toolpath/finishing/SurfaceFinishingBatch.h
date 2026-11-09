#pragma once

#include "toolpath/finishing/SurfaceFinishing.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

struct SurfaceFinishingOperation {
    std::string operationId;
    int setupIndex = 0;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    gp_Ax3 workFrame;
    SurfaceFinishingResult result;
    ToolpathToolContext tool;

    ToolpathProgram toProgram() const
    {
        auto program = result.toProgram(operationId, tool);
        for (auto& operation : program.operations) {
            operation.setupIndex = setupIndex;
            operation.setupFace = setupFace;
        }
        return program;
    }
};

struct SurfaceFinishingBatch {
    bool cancelled = false;
    std::vector<SurfaceFinishingOperation> operations;
    ToolpathValidationReport validation;
    ToolpathParams effectiveParams;

    ToolpathProgram toProgram() const
    {
        ToolpathProgram program;
        if (cancelled || !validation.ok())
            return program;
        for (const auto& source : operations) {
            auto part = source.toProgram();
            for (auto& operation : part.operations)
                program.operations.push_back(std::move(operation));
        }
        return program;
    }
};

} // namespace oap::toolpath
