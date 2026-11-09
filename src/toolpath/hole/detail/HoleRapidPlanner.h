// 可复用的孔操作间避碰连接规划器。
// 它先检测刀具中心的直接连线，再尝试经过障碍包络外全局安全平面的三段路径。
// 属于 oap_toolpath_hole 模块 detail 私有部分；由 HoleMachining 按装夹创建，
// 生命周期限于单个装夹批次，通过引用聚合到生成结果中写入诊断与统计。
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <gp_Vec.hxx>

#include "toolpath/hole/HoleMachining.h"

namespace oap::toolpath::detail {

enum class SegmentSafety { Safe, Collision, Failed };

// 单段移动的安全判定结果：Safe/Collision/Failed 与距离、阈值和归属组件。
struct SegmentCheck {
    SegmentSafety safety = SegmentSafety::Failed;
    double distance = std::numeric_limits<double>::quiet_NaN();
    bool innerSolution = false;   // 刀路端点是否已落入障碍内部。
    double threshold = std::numeric_limits<double>::quiet_NaN();
    double axialOffset = 0.0;     // 参与判定的刀具包络站位轴向偏移。
    std::string component;        // 触碰到刀具组件的名称（cutter/shank/...）。
    std::string bodyId;
};

// 孔间快速连接方案：直接连线（safe/lifted=false）或“抬升—横移—下降”三段。
struct RapidLinkPlan {
    bool safe = false;
    bool lifted = false;          // true 表示采用抬升三段路径。
    bool unavailable = false;     // true 表示无法给出安全结论（如已取消）。
    gp_Pnt fromLifted;            // 起点沿装夹外向轴抬升后的点。
    gp_Pnt toLifted;              // 终点沿装夹外向轴抬升后的点。
    SegmentCheck direct;
    SegmentCheck sourceLift;
    SegmentCheck topTraverse;
    SegmentCheck targetDescent;
    double length = std::numeric_limits<double>::infinity();
};

// 孔间避碰连接规划器：冻结容差、障碍包围盒与刀具包络，为孔间快速移动
// 提供直接连线优先、碰撞时抬升绕行的三段路径，并支持整组非切削/切削移动
// 的固实扫掠验证。同一规划器实例只服务一个装夹轴向（acceptAxis 锁定）。
class HoleRapidPlanner {
public:
    HoleRapidPlanner(const HoleRapidPlanningContext* context,
                     const ToolpathParams& params,
                     HoleMachiningResult& result,
                     HoleRapidLinkMode linkMode =
                         HoleRapidLinkMode::DirectWhenSafe,
                     const ToolpathGenerationControl* control = nullptr);

    bool acceptAxis(const gp_Vec& inwardAxis, const std::string& holeLabel);
    double minimumClearanceOffset() const;
    bool canPlanLinks() const;
    RapidLinkPlan evaluateLink(const gp_Pnt& from, const gp_Pnt& to);
    bool appendLink(std::vector<ToolpathMove>& moves,
                    const gp_Pnt& from,
                    const gp_Pnt& to,
                    int operationIndex);
    bool validateNonCuttingMoves(const std::vector<ToolpathMove>& moves,
                                 const std::string& operationId);
    bool validateCuttingGeometry(
        const std::vector<ToolpathMove>& moves,
        const std::vector<ToolpathPrimitive>& primitives,
        const std::string& operationId);

private:
    struct EnvelopeStation {
        double axialOffset = 0.0;
        double radius = 0.0;
        std::string component;
    };
    struct ToolSolidPart {
        TopoDS_Shape shape;
        std::string component;
    };
    struct CollisionBodyBounds {
        bool valid = false;
        double xMin = 0.0;
        double yMin = 0.0;
        double zMin = 0.0;
        double xMax = 0.0;
        double yMax = 0.0;
        double zMax = 0.0;
    };
    struct RapidLinkKey {
        std::array<std::uint64_t, 6> values{};
        bool operator==(const RapidLinkKey& other) const noexcept {
            return values == other.values;
        }
    };
    struct RapidLinkKeyHash {
        size_t operator()(const RapidLinkKey& key) const noexcept;
    };

    RapidLinkPlan evaluateCanonicalLink(const gp_Pnt& from, const gp_Pnt& to);
    std::string segmentCheckText(const char* label,
                                 const SegmentCheck& check) const;
    double projection(const gp_Pnt& point) const;
    void computeGlobalClearanceProjection();
    gp_Pnt liftToGlobalPlane(const gp_Pnt& point) const;
    SegmentCheck segmentSafety(const gp_Pnt& from, const gp_Pnt& to) const;
    SegmentCheck solidSweepSafety(const gp_Pnt& from,
                                  const gp_Pnt& to,
                                  bool allowCutterWorkpiece) const;
    SegmentCheck solidSweepSafety(
        const std::vector<gp_Pnt>& path,
        const std::vector<double>& segmentInflations,
        bool allowCutterWorkpiece) const;
    SegmentCheck arcSweepSafety(const ToolpathCircularArc& arc) const;
    bool stationMayOverlapBody(
        const EnvelopeStation& station,
        const std::vector<gp_Pnt>& path,
        const std::vector<double>& segmentInflations,
        const CollisionBodyBounds& bodyBounds) const;
    ToolSolidPart buildSweptToolSolid(
        const EnvelopeStation& station,
        const std::vector<gp_Pnt>& path,
        const std::vector<double>& segmentInflations) const;
    bool buildToolEnvelope();
    void appendEnvelopeSection(double startOffset,
                               double endOffset,
                               double radius,
                               const char* component);
    void appendRapidMove(std::vector<ToolpathMove>& moves,
                         const gp_Pnt& from,
                         const gp_Pnt& to,
                         int operationIndex,
                         double feed) const;
    bool cancellationRequested() const;

    const HoleRapidPlanningContext* m_context = nullptr;
    const ToolpathParams& m_params;
    HoleMachiningResult& m_result;
    HoleRapidLinkMode m_linkMode =
        HoleRapidLinkMode::DirectWhenSafe;
    const ToolpathGenerationControl* m_control = nullptr;
    bool m_ready = false;
    bool m_toolEnvelopeValid = true;
    bool m_hasSetupAxis = false;
    gp_Vec m_inwardAxis;
    gp_Vec m_outwardAxis;
    double m_tolerance = 1.0e-4;
    double m_axisToleranceDeg = 1.0;
    double m_globalClearanceProjection = 0.0;
    double m_maxEnvelopeRadius = 0.0;
    double m_xMin = 0.0;
    double m_yMin = 0.0;
    double m_zMin = 0.0;
    double m_xMax = 0.0;
    double m_yMax = 0.0;
    double m_zMax = 0.0;
    std::vector<EnvelopeStation> m_envelopeStations;
    std::vector<CollisionBody> m_collisionBodies;
    std::vector<CollisionBodyBounds> m_collisionBodyBounds;
    TopoDS_Shape m_combinedObstacle;
    std::unordered_map<RapidLinkKey, RapidLinkPlan, RapidLinkKeyHash> m_linkCache;
};

} // namespace oap::toolpath::detail
