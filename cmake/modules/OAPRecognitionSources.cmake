# Source ownership for recognition targets.

oap_module_sources(oap_recognition OAP_RECOGNITION_SOURCES
    src/recognition/AAG.cpp
    src/recognition/AAG.h
    src/recognition/AxisParallelPlaneRecognizer.cpp
    src/recognition/AxisParallelPlaneRecognizer.h
    src/recognition/AxisWireTraceTracker.cpp
    src/recognition/AxisWireTraceTracker.h
    src/recognition/ConnectedBlindPocketMerger.cpp
    src/recognition/ConnectedBlindPocketMerger.h
    src/recognition/DihedralAngle.cpp
    src/recognition/DihedralAngle.h
    src/recognition/FeatureRecognizer.cpp
    src/recognition/FeatureRecognizer.h
    src/recognition/OpeningTransitionRecognizer.cpp
    src/recognition/OpeningTransitionRecognizer.h
    src/recognition/RecognitionGeometryCache.cpp
    src/recognition/RecognitionGeometryCache.h
    src/recognition/Recognition.cpp
    src/recognition/Recognition.h
    src/recognition/ThroughStructureRecognizer.cpp
    src/recognition/ThroughStructureRecognizer.h
    src/recognition/ThreeAxisWireTraceUseCase.cpp
    src/recognition/ThreeAxisWireTraceUseCase.h
    src/recognition/detail/CountersinkRelationMatcher.cpp
    src/recognition/detail/CountersinkRelationMatcher.h
    src/recognition/detail/HoleToolpathBuilder.cpp
    src/recognition/detail/HoleToolpathBuilder.h
    src/recognition/detail/HoleToolpathRelations.cpp
    src/recognition/detail/HoleToolpathRelations.h)
