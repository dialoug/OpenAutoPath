// 稳定刀路诊断码到文本标识的集中映射，供日志、UI 和测试共享。
#include "ToolpathDiagnostics.h"

namespace oap::toolpath {

// 诊断码 → 英文机器码（snake_case）映射，供日志/UI/测试稳定引用。
const char* toolpathDiagnosticCodeName(ToolpathDiagnosticCode code) noexcept
{
    switch (code) {
        case ToolpathDiagnosticCode::RoundedCutterResidual: return "rounded_cutter_residual";
        case ToolpathDiagnosticCode::None: return "none";
        case ToolpathDiagnosticCode::InvalidToolDiameter: return "invalid_tool_diameter";
        case ToolpathDiagnosticCode::FeedRateAdjusted: return "feed_rate_adjusted";
        case ToolpathDiagnosticCode::SpindleSpeedAdjusted: return "spindle_speed_adjusted";
        case ToolpathDiagnosticCode::SafeHeightAdjusted: return "safe_height_adjusted";
        case ToolpathDiagnosticCode::ClearanceHeightAdjusted: return "clearance_height_adjusted";
        case ToolpathDiagnosticCode::ClearanceRaisedToSafeHeight: return "clearance_raised_to_safe_height";
        case ToolpathDiagnosticCode::DepthParameterAdjusted: return "depth_parameter_adjusted";
        case ToolpathDiagnosticCode::StepOverAdjusted: return "step_over_adjusted";
        case ToolpathDiagnosticCode::OffsetAdjusted: return "offset_adjusted";
        case ToolpathDiagnosticCode::PrecisionAdjusted: return "precision_adjusted";
        case ToolpathDiagnosticCode::SamplingParameterAdjusted: return "sampling_parameter_adjusted";
        case ToolpathDiagnosticCode::SamplingBudgetAdjusted: return "sampling_budget_adjusted";
        case ToolpathDiagnosticCode::HoleAllowanceAdjusted: return "hole_allowance_adjusted";
        case ToolpathDiagnosticCode::HoleRatioAdjusted: return "hole_ratio_adjusted";
        case ToolpathDiagnosticCode::HoleDiameterToleranceAdjusted: return "hole_diameter_tolerance_adjusted";
        case ToolpathDiagnosticCode::AlgorithmUnavailable: return "algorithm_unavailable";
        case ToolpathDiagnosticCode::AlgorithmExecutionFailed: return "algorithm_execution_failed";
        case ToolpathDiagnosticCode::UnsupportedFeature: return "unsupported_feature";
        case ToolpathDiagnosticCode::ExplicitFloorNotInFeature: return "explicit_floor_not_in_feature";
        case ToolpathDiagnosticCode::ManualFloorNotFound: return "manual_floor_not_found";
        case ToolpathDiagnosticCode::ManualFloorNotPlanar: return "manual_floor_not_planar";
        case ToolpathDiagnosticCode::ManualFloorDisconnected: return "manual_floor_disconnected";
        case ToolpathDiagnosticCode::MissingFaceGeometry: return "missing_face_geometry";
        case ToolpathDiagnosticCode::MissingPlanarFloor: return "missing_planar_floor";
        case ToolpathDiagnosticCode::InvalidExplicitFloor: return "invalid_explicit_floor";
        case ToolpathDiagnosticCode::MissingOuterBoundary: return "missing_outer_boundary";
        case ToolpathDiagnosticCode::DuplicateRegion: return "duplicate_region";
        case ToolpathDiagnosticCode::UnknownRegionBuildFailure: return "unknown_region_build_failure";
        case ToolpathDiagnosticCode::InvalidHoleRadius: return "invalid_hole_radius";
        case ToolpathDiagnosticCode::InvalidHoleGeometry: return "invalid_hole_geometry";
        case ToolpathDiagnosticCode::HoleOpeningUnresolved: return "hole_opening_unresolved";
        case ToolpathDiagnosticCode::HoleOpeningFallback: return "hole_opening_fallback";
        case ToolpathDiagnosticCode::HoleMoldEnvelopeProjectionFailed: return "hole_mold_envelope_projection_failed";
        case ToolpathDiagnosticCode::HoleOutsideSixFace: return "hole_outside_six_face";
        case ToolpathDiagnosticCode::ToolExceedsHole: return "tool_exceeds_hole";
        case ToolpathDiagnosticCode::ForcedStrategyNoClearance: return "forced_strategy_no_clearance";
        case ToolpathDiagnosticCode::EmptyHoleOperation: return "empty_hole_operation";
        case ToolpathDiagnosticCode::SamplingBudgetExceeded: return "sampling_budget_exceeded";
        case ToolpathDiagnosticCode::InvalidSetupAxis: return "invalid_setup_axis";
        case ToolpathDiagnosticCode::SetupAxisMismatch: return "setup_axis_mismatch";
        case ToolpathDiagnosticCode::InvalidToolAssembly: return "invalid_tool_assembly";
        case ToolpathDiagnosticCode::NonCuttingMoveCollision: return "non_cutting_move_collision";
        case ToolpathDiagnosticCode::CuttingMoveCollision: return "cutting_move_collision";
        case ToolpathDiagnosticCode::CollisionPlanningUnavailable: return "collision_planning_unavailable";
        case ToolpathDiagnosticCode::CollisionCheckFailed: return "collision_check_failed";
        case ToolpathDiagnosticCode::RapidLinkCollision: return "rapid_link_collision";
        case ToolpathDiagnosticCode::InvalidInitialStock: return "invalid_initial_stock";
        case ToolpathDiagnosticCode::StockCuttingLengthInferred: return "stock_cutting_length_inferred";
        case ToolpathDiagnosticCode::StockSweepConstructionFailed: return "stock_sweep_construction_failed";
        case ToolpathDiagnosticCode::StockInvalidRemovalSolidSkipped: return "stock_invalid_removal_solid_skipped";
        case ToolpathDiagnosticCode::StockBooleanFailed: return "stock_boolean_failed";
        case ToolpathDiagnosticCode::StockNoMaterialRemoved: return "stock_no_material_removed";
        case ToolpathDiagnosticCode::StockComparisonFailed: return "stock_comparison_failed";
        case ToolpathDiagnosticCode::StockTargetOutsideInitial: return "stock_target_outside_initial";
        case ToolpathDiagnosticCode::StockRemainingAllowance: return "stock_remaining_allowance";
        case ToolpathDiagnosticCode::StockOvercutDetected: return "stock_overcut_detected";
        case ToolpathDiagnosticCode::AutoToolLibraryEmpty: return "auto_tool_library_empty";
        case ToolpathDiagnosticCode::AutoToolUnavailable: return "auto_tool_unavailable";
        case ToolpathDiagnosticCode::AutoHoleBelowToolDiameter: return "auto_hole_below_tool_diameter";
        case ToolpathDiagnosticCode::AutoToolReachInsufficient: return "auto_tool_reach_insufficient";
        case ToolpathDiagnosticCode::AutoCenterSpotSkipped: return "auto_center_spot_skipped";
        case ToolpathDiagnosticCode::AutoToolAssignment: return "auto_tool_assignment";
        case ToolpathDiagnosticCode::DuplicateHoleStageKey: return "duplicate_hole_stage_key";
        case ToolpathDiagnosticCode::GenerationCancelled: return "generation_cancelled";
        case ToolpathDiagnosticCode::PocketBoundaryViolation: return "pocket_boundary_violation";
        case ToolpathDiagnosticCode::PocketSafetySamplingLimited: return "pocket_safety_sampling_limited";
        case ToolpathDiagnosticCode::ProfileFinishedPartIntrusion: return "profile_finished_part_intrusion";
        case ToolpathDiagnosticCode::ProfileCuttingEdgeReachInsufficient: return "profile_cutting_edge_reach_insufficient";
        case ToolpathDiagnosticCode::ProfileToolReachInsufficient: return "profile_tool_reach_insufficient";
        case ToolpathDiagnosticCode::ProfileSafetySamplingLimited: return "profile_safety_sampling_limited";
    }
    return "unknown";
}

// 诊断码 → 中文默认文案映射；具体上下文由调用方在 message 中补充。
const char* toolpathDiagnosticDefaultMessageZh(ToolpathDiagnosticCode code) noexcept
{
    switch (code) {
        case ToolpathDiagnosticCode::RoundedCutterResidual: return "球刀或圆鼻刀分层加工可能在槽底、根部留下余料，请使用 IPW 检查";
        case ToolpathDiagnosticCode::None: return "无诊断";
        case ToolpathDiagnosticCode::InvalidToolDiameter: return "刀具直径必须为有限正数";
        case ToolpathDiagnosticCode::FeedRateAdjusted: return "无效进给速度已归一化";
        case ToolpathDiagnosticCode::SpindleSpeedAdjusted: return "无效主轴转速已归一化";
        case ToolpathDiagnosticCode::SafeHeightAdjusted: return "安全高度已归一化";
        case ToolpathDiagnosticCode::ClearanceHeightAdjusted: return "清空高度已归一化";
        case ToolpathDiagnosticCode::ClearanceRaisedToSafeHeight: return "清空高度已抬高到安全高度";
        case ToolpathDiagnosticCode::DepthParameterAdjusted: return "深度参数已归一化";
        case ToolpathDiagnosticCode::StepOverAdjusted: return "步距已归一化";
        case ToolpathDiagnosticCode::OffsetAdjusted: return "偏置参数已归一化";
        case ToolpathDiagnosticCode::PrecisionAdjusted: return "离散精度已归一化";
        case ToolpathDiagnosticCode::SamplingParameterAdjusted: return "采样参数已归一化";
        case ToolpathDiagnosticCode::SamplingBudgetAdjusted: return "采样点预算已归一化";
        case ToolpathDiagnosticCode::HoleAllowanceAdjusted: return "孔加工余量已归一化";
        case ToolpathDiagnosticCode::HoleRatioAdjusted: return "孔策略直径比已归一化";
        case ToolpathDiagnosticCode::HoleDiameterToleranceAdjusted: return "小孔直径容差已归一化";
        case ToolpathDiagnosticCode::AlgorithmUnavailable: return "请求的刀路算法未注册";
        case ToolpathDiagnosticCode::AlgorithmExecutionFailed: return "刀路算法执行失败";
        case ToolpathDiagnosticCode::UnsupportedFeature: return "实例不是槽或腔";
        case ToolpathDiagnosticCode::ExplicitFloorNotInFeature: return "指定底面不属于该特征";
        case ToolpathDiagnosticCode::ManualFloorNotFound: return "未找到人工指定底面";
        case ToolpathDiagnosticCode::ManualFloorNotPlanar: return "人工指定底面不是平面";
        case ToolpathDiagnosticCode::ManualFloorDisconnected: return "人工底面没有拓扑相邻面";
        case ToolpathDiagnosticCode::MissingFaceGeometry: return "特征没有可用的面几何";
        case ToolpathDiagnosticCode::MissingPlanarFloor: return "特征没有平面底面候选";
        case ToolpathDiagnosticCode::InvalidExplicitFloor: return "指定底面不是有效平面候选";
        case ToolpathDiagnosticCode::MissingOuterBoundary: return "底面没有可用外边界";
        case ToolpathDiagnosticCode::DuplicateRegion: return "自动区域与已有区域共享底面";
        case ToolpathDiagnosticCode::UnknownRegionBuildFailure: return "区域构建失败";
        case ToolpathDiagnosticCode::InvalidHoleRadius: return "孔半径无效";
        case ToolpathDiagnosticCode::InvalidHoleGeometry: return "孔深度、端点或轴向无效";
        case ToolpathDiagnosticCode::HoleOpeningUnresolved: return "未能可靠确定孔的外部入口";
        case ToolpathDiagnosticCode::HoleOpeningFallback: return "孔入口由模具厚度边界推断";
        case ToolpathDiagnosticCode::HoleMoldEnvelopeProjectionFailed: return "孔入口无法投影到指定模具外包络面";
        case ToolpathDiagnosticCode::HoleOutsideSixFace: return "孔不属于当前支持的六面加工范围";
        case ToolpathDiagnosticCode::ToolExceedsHole: return "刀具与余量超过孔半径";
        case ToolpathDiagnosticCode::ForcedStrategyNoClearance: return "指定铣孔策略没有径向空间";
        case ToolpathDiagnosticCode::EmptyHoleOperation: return "孔加工未生成有效移动";
        case ToolpathDiagnosticCode::SamplingBudgetExceeded: return "孔刀路超过采样点预算";
        case ToolpathDiagnosticCode::InvalidSetupAxis: return "加工轴无效";
        case ToolpathDiagnosticCode::SetupAxisMismatch: return "加工轴与当前装夹方向不一致";
        case ToolpathDiagnosticCode::InvalidToolAssembly: return "刀具/刀柄碰撞包络参数不完整";
        case ToolpathDiagnosticCode::NonCuttingMoveCollision: return "非切削移动的连续扫掠体发生碰撞";
        case ToolpathDiagnosticCode::CuttingMoveCollision: return "切削移动的连续扫掠体发生碰撞";
        case ToolpathDiagnosticCode::CollisionPlanningUnavailable: return "避障规划不可用";
        case ToolpathDiagnosticCode::CollisionCheckFailed: return "碰撞检测计算失败";
        case ToolpathDiagnosticCode::RapidLinkCollision: return "孔间快移无法找到无碰撞路径";
        case ToolpathDiagnosticCode::InvalidInitialStock: return "初始毛坯不是有效正体积实体";
        case ToolpathDiagnosticCode::StockCuttingLengthInferred: return "刀具刃长未知，材料去除体使用刀路与毛坯轴向范围推导长度";
        case ToolpathDiagnosticCode::StockSweepConstructionFailed: return "无法构造切削材料去除体";
        case ToolpathDiagnosticCode::StockInvalidRemovalSolidSkipped: return "无法修复的退化减料实体已跳过，结果只增加欠切";
        case ToolpathDiagnosticCode::StockBooleanFailed: return "在制毛坯布尔减料失败";
        case ToolpathDiagnosticCode::StockNoMaterialRemoved: return "该工序没有从当前毛坯去除材料";
        case ToolpathDiagnosticCode::StockComparisonFailed: return "毛坯与目标实体差异计算失败";
        case ToolpathDiagnosticCode::StockTargetOutsideInitial: return "目标实体有部分位于初始毛坯之外";
        case ToolpathDiagnosticCode::StockRemainingAllowance: return "仿真后仍有目标实体外的剩余加工余量";
        case ToolpathDiagnosticCode::StockOvercutDetected: return "仿真去除了目标实体应保留的材料";
        case ToolpathDiagnosticCode::AutoToolLibraryEmpty: return "活动刀库没有可参与孔加工的已分配刀具";
        case ToolpathDiagnosticCode::AutoToolUnavailable: return "当前工序没有可用刀具";
        case ToolpathDiagnosticCode::AutoHoleBelowToolDiameter: return "目标孔径小于当前刀类的最小刀具";
        case ToolpathDiagnosticCode::AutoToolReachInsufficient: return "尺寸兼容刀具的有效刃长或总可达长度不足";
        case ToolpathDiagnosticCode::AutoCenterSpotSkipped: return "物理孔没有可用中心钻定位";
        case ToolpathDiagnosticCode::AutoToolAssignment: return "自动选刀分配完成";
        case ToolpathDiagnosticCode::DuplicateHoleStageKey: return "当前孔段会话键不唯一";
        case ToolpathDiagnosticCode::GenerationCancelled: return "刀路生成已取消";
        case ToolpathDiagnosticCode::PocketBoundaryViolation: return "切削移动超出刀心安全区域";
        case ToolpathDiagnosticCode::PocketSafetySamplingLimited: return "安全采样达到预算上限";
        case ToolpathDiagnosticCode::ProfileFinishedPartIntrusion: return "外轮廓刀路侵入成品材料包络";
        case ToolpathDiagnosticCode::ProfileCuttingEdgeReachInsufficient: return "外轮廓加工的单层轴向切削量超过刀具有效刃长";
        case ToolpathDiagnosticCode::ProfileToolReachInsufficient: return "外轮廓总深度超过刀具可达长度";
        case ToolpathDiagnosticCode::ProfileSafetySamplingLimited: return "外轮廓安全采样达到预算上限";
    }
    return "未知诊断";
}

} // namespace oap::toolpath
