# Public headers owned by the manufacturing module.
list(APPEND OAP_PUBLIC_TARGET_HEADERS
    src/manufacturing/finishing/SurfaceFinishingMachiningJobService.h
    src/manufacturing/hole/HoleMachiningJobService.h
    src/manufacturing/ipw/HoleMultiDexelUpdater.h
    src/manufacturing/ipw/IpwExecutor.h
    src/manufacturing/ipw/IpwSession.h

    src/manufacturing/job/MachiningJob.h
    src/manufacturing/job/MachiningJobApplicationService.h
    src/manufacturing/job/MachiningOperationService.h
    src/manufacturing/job/MachiningJobStorage.h
    src/manufacturing/pocket/PocketMachiningJobService.h
    src/manufacturing/profile/ProfileMachiningJobService.h
    src/manufacturing/program/MachiningProgramAssembler.h
    src/manufacturing/program/MachiningProgramPartitioner.h

)
