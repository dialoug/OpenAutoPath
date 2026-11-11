# Source ownership for manufacturing targets.

oap_module_sources(oap_manufacturing OAP_MANUFACTURING_SOURCES
    src/manufacturing/job/detail/SurfaceFinishingJson.h
    src/manufacturing/job/detail/SurfaceFinishingJson.cpp
    src/manufacturing/finishing/SurfaceFinishingMachiningJobService.h
    src/manufacturing/finishing/SurfaceFinishingMachiningJobService.cpp
    src/manufacturing/ipw/IpwCheckpointStore.cpp
    src/manufacturing/ipw/IpwCheckpointStore.h
    src/manufacturing/ipw/IpwExecutor.cpp
    src/manufacturing/ipw/IpwExecutor.h
    src/manufacturing/ipw/HoleMultiDexelUpdater.cpp
    src/manufacturing/ipw/HoleMultiDexelUpdater.h
    src/manufacturing/ipw/IpwSession.cpp
    src/manufacturing/ipw/IpwSession.h


    src/manufacturing/job/MachiningJob.cpp
    src/manufacturing/job/MachiningJob.h
    src/manufacturing/job/MachiningJobApplicationService.cpp
    src/manufacturing/job/MachiningJobApplicationService.h
    src/manufacturing/job/MachiningOperationService.cpp
    src/manufacturing/job/MachiningOperationService.h
    src/manufacturing/job/MachiningJobStorage.cpp
    src/manufacturing/job/MachiningJobStorage.h
    src/manufacturing/hole/HoleJobRebuildReview.cpp
    src/manufacturing/hole/HoleJobRebuildReview.h
    src/manufacturing/hole/HoleMachiningJobService.cpp
    src/manufacturing/hole/HoleMachiningJobService.h
    src/manufacturing/pocket/PocketMachiningJobService.cpp
    src/manufacturing/pocket/PocketMachiningJobService.h
    src/manufacturing/profile/ProfileMachiningJobService.cpp
    src/manufacturing/profile/ProfileMachiningJobService.h
    src/manufacturing/program/MachiningProgramAssembler.cpp
    src/manufacturing/program/MachiningProgramAssembler.h
    src/manufacturing/program/MachiningProgramPartitioner.cpp
    src/manufacturing/program/MachiningProgramPartitioner.h


    src/manufacturing/hole/detail/HoleJobFoundation.h
    src/manufacturing/hole/detail/HoleJobFoundation.cpp
    src/manufacturing/hole/detail/HoleJobEnablement.h
    src/manufacturing/hole/detail/HoleJobEnablement.cpp
    src/manufacturing/hole/detail/HoleJobIdentity.h
    src/manufacturing/hole/detail/HoleJobIdentity.cpp
    src/manufacturing/hole/detail/HoleJobOperationOrder.h
    src/manufacturing/hole/detail/HoleJobOperationOrder.cpp
    src/manufacturing/hole/detail/HoleJobActiveIpw.h
    src/manufacturing/hole/detail/HoleJobActiveIpw.cpp
    src/manufacturing/hole/HoleJobSelection.cpp
    src/manufacturing/hole/HoleJobRegeneration.cpp
    src/manufacturing/hole/detail/HoleJobTypes.h)
