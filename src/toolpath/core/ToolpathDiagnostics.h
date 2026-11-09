// 刀路模块统一诊断模型。
// code 供 UI、测试和导出逻辑稳定判断；message 仅用于补充具体上下文，
// 调用方不应再通过解析 message 决定业务分支。
#pragma once

#include <string>

namespace oap::toolpath {

enum class ToolpathDiagnosticSeverity {
    Info,
    Warning,
    Error,
};

enum class ToolpathDiagnosticCode {
    None,

    // 参数归一化。
    InvalidToolDiameter,
    FeedRateAdjusted,
    SpindleSpeedAdjusted,
    SafeHeightAdjusted,
    ClearanceHeightAdjusted,
    ClearanceRaisedToSafeHeight,
    DepthParameterAdjusted,
    StepOverAdjusted,
    OffsetAdjusted,
    PrecisionAdjusted,
    SamplingParameterAdjusted,
    SamplingBudgetAdjusted,
    HoleAllowanceAdjusted,
    HoleRatioAdjusted,
    HoleDiameterToleranceAdjusted,

    // 算法发现、选择和扩展边界。
    AlgorithmUnavailable,
    AlgorithmExecutionFailed,

    // 区域构建。
    UnsupportedFeature,
    ExplicitFloorNotInFeature,
    ManualFloorNotFound,
    ManualFloorNotPlanar,
    ManualFloorDisconnected,
    MissingFaceGeometry,
    MissingPlanarFloor,
    InvalidExplicitFloor,
    MissingOuterBoundary,
    DuplicateRegion,
    UnknownRegionBuildFailure,

    // 孔加工和连接安全。
    InvalidHoleRadius,
    InvalidHoleGeometry,
    HoleOpeningUnresolved,
    HoleOpeningFallback,
    HoleMoldEnvelopeProjectionFailed,
    HoleOutsideSixFace,
    ToolExceedsHole,
    ForcedStrategyNoClearance,
    EmptyHoleOperation,
    SamplingBudgetExceeded,
    InvalidSetupAxis,
    SetupAxisMismatch,
    InvalidToolAssembly,
    NonCuttingMoveCollision,
    CuttingMoveCollision,
    CollisionPlanningUnavailable,
    CollisionCheckFailed,
    RapidLinkCollision,

    // 在制毛坯材料去除。
    InvalidInitialStock,
    StockCuttingLengthInferred,
    StockSweepConstructionFailed,
    StockInvalidRemovalSolidSkipped,
    StockBooleanFailed,
    StockNoMaterialRemoved,
    StockComparisonFailed,
    StockTargetOutsideInitial,
    StockRemainingAllowance,
    StockOvercutDetected,

    // 自动选刀规划。
    AutoToolLibraryEmpty,
    AutoToolUnavailable,
    AutoHoleBelowToolDiameter,
    AutoToolReachInsufficient,
    AutoCenterSpotSkipped,
    AutoToolAssignment,

    // 孔段稳定身份校验。
    DuplicateHoleStageKey,

    // 工作流与安全验证。
    GenerationCancelled,
    PocketBoundaryViolation,
    PocketSafetySamplingLimited,
    ProfileFinishedPartIntrusion,
    ProfileCuttingEdgeReachInsufficient,
    ProfileToolReachInsufficient,
    ProfileSafetySamplingLimited,
    RoundedCutterResidual,
};

// 单条诊断：code 供 UI/测试/导出稳定判断；severity 决定是否阻断；
// message 是面向操作者的补充上下文；sourceIndex/depthIndex/moveIndex
// 用于定位到特征/深度层/移动段，未绑定时为 -1。
struct ToolpathDiagnostic {
    ToolpathDiagnosticCode code = ToolpathDiagnosticCode::None;
    ToolpathDiagnosticSeverity severity = ToolpathDiagnosticSeverity::Info;
    std::string message;
    std::string operationId;
    int sourceIndex = -1;
    int depthIndex = -1;
    int moveIndex = -1;
};

// 稳定机器码（如 "invalid_tool_diameter"）供日志/UI/测试使用。
const char* toolpathDiagnosticCodeName(ToolpathDiagnosticCode code) noexcept;
// 每种诊断码的中文默认文案；具体上下文由 message 另行补充。
const char* toolpathDiagnosticDefaultMessageZh(ToolpathDiagnosticCode code) noexcept;

} // namespace oap::toolpath
