// 识别、刀路和显示层共用的中立特征几何类型。
// 这些值类型不依赖具体识别算法，下游模块只为描述输入时无需包含
// recognition/FeatureRecognizer.h。
// 属于 oap_analysis 模块的对外值类型契约：recognition/ 生成这些值，刀路规划与
// 显示层消费它们；均为纯值语义、无所有权，可跨模块复制与持久化。
#pragma once

#include <string>
#include <vector>

#include <gp_Pnt.hxx>

namespace oap {

// 稳定的应用层特征类别。它与 recognition/ 内部实现枚举有意分离。
enum class FeatureKind {
    ThroughHole,
    BlindHole,
    CountersinkHole,
    Slot,
    Pocket
};

// 一个由相连面组成的应用层特征实例。识别器、人工修正和刀路区域构建
// 通过该结构交换数据，不要求下游依赖 FeatureRecognizer。
struct FeatureInstance {
    FeatureKind kind = FeatureKind::Slot;
    std::vector<std::string> faceIds;
    bool manual = false;
    // 人工槽/腔指定的权威底面；为空时允许区域构建器自动选择。
    std::string floorFaceId;
};

enum class HoleEntryResolution {
    Parametric,           // 通孔端点，方向稍后按模具坐标系统一。
    TopologyOpening,      // 圆柱壁与外表面的凸边确定了入口。
    MoldBoundaryFallback, // 拓扑提示缺失时由模具厚度边界推断入口。
    MoldSurfaceNearest,   // 端点按模具六个理想外包络面重新确定入口与装夹面。
    CountersinkLargestStage, // 沉头孔内部段继承最大直径段的入口方向与装夹面。
    CountersinkFloorOpening, // 偏心子孔从父沉孔平底面继承装夹面与入口方向。
    MoldEnvelopeProjected,// 物理孔最外侧段已沿孔轴恢复到对应外包络面。
    Unresolved            // 无可靠外部入口，不应生成加工刀路。
};

// 孔段与父级孔的关系；用于复合沉孔/偏心孔内部段的归组。
enum class HoleParentRelation {
    None,
    CoaxialStage,
    CounterboreFloorChild
};

// 复合沉孔中的圆柱段职责独立于通孔/盲孔语义，供选择、人工计划和显示层
// 区分大径扩大段与小径钻孔段。
enum class HoleStageRole {
    Standard,
    CountersinkLargeDiameter,
    CountersinkSmallDiameter,
    CounterboreFloorChild
};

// 关系枚举 -> 稳定字符串名，用于 Job 持久化与诊断输出。
inline const char* holeParentRelationName(HoleParentRelation relation) noexcept
{
    switch (relation) {
        case HoleParentRelation::None: return "none";
        case HoleParentRelation::CoaxialStage: return "coaxial_stage";
        case HoleParentRelation::CounterboreFloorChild:
            return "counterbore_floor_child";
    }
    return "none";
}

// 刀具所在的模具外表面。顺序同时也是装夹调度的稳定优先级。
enum class MoldSetupFace {
    PositiveZ,
    PositiveX,
    PositiveY,
    NegativeX,
    NegativeY,
    NegativeZ,
    Unresolved
};

// 装夹面决策依据：记录装夹面是依最近距离、拓扑破平还是稳定顺序选定，
// 供人工计划与显示层解释。
enum class MoldSetupFaceDecision {
    None,
    NearestDistance,
    TopologyTieBreak,
    StableFaceOrder
};

// 孔段轴向终点的真实拓扑轮廓。自动选刀只把明确的平面底/平面肩部
// 交给平底刀；Unknown 不允许按孔类型猜测。
enum class HoleBottomProfile {
    Open,
    Planar,
    Conical,
    Unknown
};

// 装夹面枚举 -> 稳定字符串名；"+Z" 等方向名用于 UI 与输出。
inline const char* moldSetupFaceName(MoldSetupFace face) noexcept
{
    switch (face) {
        case MoldSetupFace::PositiveZ: return "+Z";
        case MoldSetupFace::PositiveX: return "+X";
        case MoldSetupFace::PositiveY: return "+Y";
        case MoldSetupFace::NegativeX: return "-X";
        case MoldSetupFace::NegativeY: return "-Y";
        case MoldSetupFace::NegativeZ: return "-Z";
        case MoldSetupFace::Unresolved: return "unresolved";
    }
    return "unresolved";
}

// 返回相反模具面（+Z <-> -Z），用于判断孔能否从对侧装夹。
inline MoldSetupFace oppositeMoldSetupFace(MoldSetupFace face) noexcept
{
    switch (face) {
        case MoldSetupFace::PositiveZ: return MoldSetupFace::NegativeZ;
        case MoldSetupFace::PositiveX: return MoldSetupFace::NegativeX;
        case MoldSetupFace::PositiveY: return MoldSetupFace::NegativeY;
        case MoldSetupFace::NegativeX: return MoldSetupFace::PositiveX;
        case MoldSetupFace::NegativeY: return MoldSetupFace::PositiveY;
        case MoldSetupFace::NegativeZ: return MoldSetupFace::PositiveZ;
        case MoldSetupFace::Unresolved: return MoldSetupFace::Unresolved;
    }
    return MoldSetupFace::Unresolved;
}

// 决策枚举 -> 稳定字符串名。
inline const char* moldSetupFaceDecisionName(
    MoldSetupFaceDecision decision) noexcept
{
    switch (decision) {
        case MoldSetupFaceDecision::None: return "none";
        case MoldSetupFaceDecision::NearestDistance: return "nearest_distance";
        case MoldSetupFaceDecision::TopologyTieBreak: return "topology_tie_break";
        case MoldSetupFaceDecision::StableFaceOrder: return "stable_face_order";
    }
    return "none";
}

// 解析孔段角色；兼容尚未保存 stageRole 的旧 Job 数据。
template <typename Hole>
inline HoleStageRole resolvedHoleStageRole(
    const Hole& hole) noexcept
{
    if (hole.stageRole != HoleStageRole::Standard)
        return hole.stageRole;
    if (hole.parentRelation == HoleParentRelation::CoaxialStage)
        return HoleStageRole::CountersinkSmallDiameter;
    if (hole.parentRelation == HoleParentRelation::CounterboreFloorChild)
        return HoleStageRole::CounterboreFloorChild;
    if (hole.kind == FeatureKind::CountersinkHole)
        return HoleStageRole::CountersinkLargeDiameter;
    return HoleStageRole::Standard;
}

// 判断孔段几何是否允许沿同一孔轴从相反模具面装夹；人工计划和入口换向共用。
template <typename Hole>
inline bool holeSupportsOppositeSetup(
    const Hole& hole) noexcept
{
    return hole.kind == FeatureKind::ThroughHole ||
           hole.bottomProfile == HoleBottomProfile::Open;
}

} // namespace oap
