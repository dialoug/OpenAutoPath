# Source ownership for analysis targets.

oap_module_sources(oap_analysis OAP_ANALYSIS_SOURCES
    src/analysis/AnalysisPipeline.cpp
    src/analysis/AnalysisPipeline.h
    src/analysis/FeatureTypes.h
    src/analysis/HoleGeometry.h
    src/analysis/Model.h
    src/analysis/MoldCoordinateFrame.cpp
    src/analysis/MoldCoordinateFrame.h
    src/analysis/detail/MoldSurfaceClassifier.cpp
    src/analysis/MoldSurfaceClassifier.h
    src/analysis/detail/AabbBoundaryFace.cpp
    src/analysis/AabbBoundaryFace.h
    src/analysis/ModelIndex.cpp
    src/analysis/ModelIndex.h
    src/analysis/Naming.cpp
    src/analysis/Naming.h
    src/analysis/StepIndexer.cpp
    src/analysis/StepIndexer.h
    src/analysis/detail/StepIndexGeometry.cpp
    src/analysis/detail/StepIndexGeometry.h
    src/analysis/Utf8Path.h
    src/analysis/ContentHash.cpp
    src/analysis/ContentHash.h
    src/analysis/Json.cpp
    src/analysis/Json.h
    src/analysis/SurfaceClassify.cpp
    src/analysis/SurfaceClassify.h
    src/analysis/ModelIO.cpp
    src/analysis/ModelIO.h)
