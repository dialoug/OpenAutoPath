# Public headers owned by the toolpath module.
list(APPEND OAP_PUBLIC_TARGET_HEADERS
    src/toolpath/finishing/SurfaceFinishingBatch.h
    src/toolpath/finishing/SurfaceFinishing.h
    src/toolpath/core/simulation/FixedAxisToolpathView.h
    src/toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h
    src/toolpath/core/HeavyJobScheduler.h
    src/toolpath/core/collision/CollisionScene.h
    src/toolpath/core/MachiningSetup.h
    src/toolpath/core/setup/MoldSetupWorkFrames.h
    src/toolpath/core/simulation/DexelToolProfile.h
    src/toolpath/core/simulation/InProcessStock.h
    src/toolpath/core/simulation/StockBoxBuilder.h
    src/toolpath/core/simulation/StockComparison.h
    src/toolpath/core/simulation/StockModelLoader.h
    src/toolpath/core/simulation/StockSimulationTypes.h
    src/toolpath/core/ToolpathDiagnostics.h
    src/toolpath/core/ToolpathExecution.h
    src/toolpath/core/ToolpathParallel.h
    src/toolpath/core/ToolpathOcctParallel.h
    src/toolpath/core/ToolpathMoves.h
    src/toolpath/core/ToolpathPrimitiveMapping.h
    src/toolpath/core/PocketToolBinding.h
    src/toolpath/core/ToolpathParamNormalizer.h
    src/toolpath/core/ToolpathParams.h
    src/toolpath/core/ToolpathPrimitives.h
    src/toolpath/core/ToolpathProgram.h
    src/toolpath/core/ToolpathToolContext.h
    src/toolpath/core/validation/ToolpathMoveValidator.h
    src/toolpath/core/validation/ToolpathParameterValidator.h
    src/toolpath/core/validation/ToolpathPrimitiveValidator.h
    src/toolpath/core/validation/ToolpathToolValidator.h
    src/toolpath/core/validation/ToolpathValidationTypes.h
    src/toolpath/program/GCodePostProcessor.h
    src/toolpath/program/ToolpathProgramExport.h
    src/toolpath/program/validation/GCodeReview.h
    src/toolpath/program/validation/ToolpathProgramValidator.h
    src/toolpath/hole/DrillTipParameterBinding.h
    src/toolpath/hole/HoleBatchExecution.h
    src/toolpath/hole/HoleCoverage.h
    src/toolpath/hole/HoleMachining.h
    src/toolpath/hole/HoleMachiningInput.h
    src/toolpath/hole/HoleMoldEnvelopeEntryResolver.h
    src/toolpath/hole/HolePhysicalGrouping.h
    src/toolpath/hole/HoleProgramAdapter.h
    src/toolpath/hole/HoleStageWallHighlightResolver.h
    src/toolpath/hole/HoleStockSimulator.h
    src/toolpath/hole/HoleToolpathValidator.h
    src/toolpath/pocket/simulation/DexelDisplayMesh.h
    src/toolpath/pocket/simulation/DexelIntervalOps.h
    src/toolpath/pocket/simulation/MultiDexelStock.h
    src/toolpath/pocket/simulation/StockTargetCache.h
    src/toolpath/pocket/simulation/PocketCamSimulationScene.h
    src/toolpath/pocket/planning/PocketClearing.h
    src/toolpath/pocket/planning/CutterCenterClearingPattern.h
    src/toolpath/pocket/planning/PocketLayerSlicer.h
    src/toolpath/pocket/program/PocketProgramAdapter.h
    src/toolpath/pocket/geometry/OcctAreaAdapter.h
    src/toolpath/pocket/geometry/PocketRegionBuilder.h
    src/toolpath/pocket/validation/PocketSafetyValidator.h
    src/toolpath/pocket/program/PocketToolpathExport.h
    src/toolpath/pocket/program/PocketToolpathOperation.h
    src/toolpath/pocket/validation/PocketToolpathValidator.h
    src/toolpath/pocket/simulation/PocketZMapComparison.h
    src/toolpath/pocket/simulation/fixedaxis/FixedAxisBRepStockSimulator.h
    src/toolpath/profile/geometry/ProfileRegion.h
    src/toolpath/profile/geometry/ProfileRegionBuilder.h
    src/toolpath/profile/planning/ProfileContour.h
    src/toolpath/profile/planning/ProfileDirectionPlanner.h
    src/toolpath/profile/program/ProfileProgramAdapter.h
    src/toolpath/profile/program/ProfileToolpathOperation.h
    src/toolpath/profile/simulation/ProfileSimulationAdapter.h
    src/toolpath/profile/validation/ProfileSafetyValidator.h












    src/toolpath/algorithms/ToolpathAlgorithm.h
    src/toolpath/workflow/hole/HoleToolpathService.h
    src/toolpath/workflow/hole/HoleSimulationAdapter.h
    src/toolpath/workflow/pocket/PocketToolpathService.h
    src/toolpath/workflow/profile/ProfileToolpathService.h

    src/toolpath/workflow/ToolpathProgramAdapters.h
    src/toolpath/workflow/HoleBatchExecutor.h
    src/toolpath/workflow/ToolpathAsyncExecutor.h
    src/toolpath/workflow/ToolpathWorkflow.h
)
