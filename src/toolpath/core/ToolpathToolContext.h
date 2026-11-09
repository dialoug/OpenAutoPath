// 单个加工操作携带的刀具快照。该值类型不依赖 Qt 刀库模型，刀库适配层在
// 规划时复制必要字段，保证异步生成、验证和后处理使用同一份稳定参数。
#pragma once

#include <string>

namespace oap::toolpath {

// 孔加工阶段：同一物理孔在不同刀具下分阶段加工，后处理按阶段分组输出。
enum class HoleMachiningPhase {
    Standard,
    CenterSpot,
    Drill,
    FlatEndMill,
};

// 冷却方式：Off 不输出 M07/M08，Flood 输出 M08，ThroughTool 输出内冷。
enum class ToolpathCoolantMode {
    Off,
    Flood,
    ThroughTool,
};

// 钻削循环偏好：Auto 按孔深/刀径比自动选择 G81/G73/G83，其余为显式指定。
enum class ToolpathDrillCyclePreference {
    Auto,
    Direct,
    Peck,
    ChipBreak,
};

// CAM 刀具上下文快照：刀号、名称、类型与几何参数。
struct ToolpathToolContext {
    bool valid = false;
    int toolNumber = 0;
    std::string toolId;
    std::string toolName;
    std::string toolType;
    double diameter = 0.0;
    double cornerRadius = 0.0;
    double spindleSpeed = 0.0;
    double horizontalFeed = 0.0;
    double verticalFeed = 0.0;
    double horizontalRapid = 0.0;
    double verticalRapid = 0.0;
    double cuttingEdgeLength = 0.0;
    double overallLength = 0.0;
    double shankDiameter = 0.0;
    double holderDiameter = 0.0;
    double holderLength = 0.0;
    double spindleEnvelopeDiameter = 0.0;
    double spindleEnvelopeLength = 0.0;
    double tipAngle = 0.0;
    double tipCompensation = 0.0;
    double peckDepth = 0.0;
    ToolpathDrillCyclePreference drillCycle =
        ToolpathDrillCyclePreference::Auto;
    ToolpathCoolantMode coolant = ToolpathCoolantMode::Off;
};

const char* holeMachiningPhaseName(HoleMachiningPhase phase) noexcept;

// 生成覆盖全部刀具、切削和装配字段的稳定快照键。
// 该值不是密码学摘要；调用方可对其求哈希以生成短 ID。
std::string toolpathToolContextKey(const ToolpathToolContext& tool);

} // namespace oap::toolpath
