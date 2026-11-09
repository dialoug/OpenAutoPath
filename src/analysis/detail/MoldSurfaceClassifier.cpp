// detail/MoldSurfaceClassifier 实现 —— 按模具局部六面包络对孔入口进行分类,
// 并记录几何、拓扑和回退判定来源。内部实现细节:为孔特征识别提供装夹面判定
// 与投影的几何原语,不直接暴露给 UI 层。
#include "analysis/MoldSurfaceClassifier.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <Precision.hxx>
#include <gp_Vec.hxx>

namespace oap {
namespace {

// 局部坐标系(模具装夹坐标系)下的包围盒:把世界 bbox 的 8 个角点变换后取极值。
struct LocalBounds {
    std::array<double, 3> minimum{};
    std::array<double, 3> maximum{};
    bool valid = false;
};

// 按轴下标取坐标(0=X, 1=Y, 2=Z),与 std::array<double,3> 的索引约定一致。
double coordinate(const gp_Pnt& point, int axis)
{
    if (axis == 0) return point.X();
    if (axis == 1) return point.Y();
    return point.Z();
}

// 将世界坐标 bbox 的 8 个角点变换到模具局部系并求极值盒;变换结果含非有限值时无效。
LocalBounds localBounds(const Bbox& bounds, const MoldCoordinateFrame& frame)
{
    LocalBounds result;
    if (!bounds.valid() || !frame.isValid)
        return result;
    result.minimum.fill(std::numeric_limits<double>::infinity());
    result.maximum.fill(-std::numeric_limits<double>::infinity());
    for (int xi = 0; xi < 2; ++xi) {
        for (int yi = 0; yi < 2; ++yi) {
            for (int zi = 0; zi < 2; ++zi) {
                const gp_Pnt local = frame.worldToLocal(gp_Pnt(
                    xi ? bounds.xmax : bounds.xmin,
                    yi ? bounds.ymax : bounds.ymin,
                    zi ? bounds.zmax : bounds.zmin));
                for (int axis = 0; axis < 3; ++axis) {
                    result.minimum[axis] = std::min(
                        result.minimum[axis], coordinate(local, axis));
                    result.maximum[axis] = std::max(
                        result.maximum[axis], coordinate(local, axis));
                }
            }
        }
    }
    result.valid = std::all_of(
        result.minimum.begin(), result.minimum.end(), [](double value) {
            return std::isfinite(value);
        }) && std::all_of(
        result.maximum.begin(), result.maximum.end(), [](double value) {
            return std::isfinite(value);
        });
    return result;
}

// 轴下标与方向 -> 模具装夹面枚举(局部 +X/+Y/+Z 与 -X/-Y/-Z 共六个面)。
MoldSetupFace positiveFace(int axis)
{
    if (axis == 0) return MoldSetupFace::PositiveX;
    if (axis == 1) return MoldSetupFace::PositiveY;
    return MoldSetupFace::PositiveZ;
}

// 轴下标与方向 -> 模具装夹面枚举(局部 +X/+Y/+Z 与 -X/-Y/-Z 共六个面)。
MoldSetupFace negativeFace(int axis)
{
    if (axis == 0) return MoldSetupFace::NegativeX;
    if (axis == 1) return MoldSetupFace::NegativeY;
    return MoldSetupFace::NegativeZ;
}

// 装夹面枚举 -> (轴下标, 正/负方向);Unresolved 返回 false。
bool faceAxis(MoldSetupFace face, int& axis, bool& positive)
{
    switch (face) {
        case MoldSetupFace::PositiveX: axis = 0; positive = true; return true;
        case MoldSetupFace::NegativeX: axis = 0; positive = false; return true;
        case MoldSetupFace::PositiveY: axis = 1; positive = true; return true;
        case MoldSetupFace::NegativeY: axis = 1; positive = false; return true;
        case MoldSetupFace::PositiveZ: axis = 2; positive = true; return true;
        case MoldSetupFace::NegativeZ: axis = 2; positive = false; return true;
        case MoldSetupFace::Unresolved: return false;
    }
    return false;
}

// 角度容差(度) -> 最小允许余弦:孔段/外法向与局部主轴夹角的判定下限。
double minimumAxisDot(double axisToleranceDeg)
{
    constexpr double kPi = 3.14159265358979323846;
    const double clamped = std::clamp(axisToleranceDeg, 0.01, 45.0);
    return std::cos(clamped * kPi / 180.0);
}

} // namespace

// 依据模具局部包络对孔段(first -> second)判定入口与装夹面,步骤为:
//   1) 孔段方向须近似平行于某一局部主轴(容差 axisToleranceDeg);
//   2) 两个端点分别与该轴正/负包络面比较,取距离最近的端点为入口;
//   3) 距离并列时优先用识别阶段提供的拓扑开口点(topologyOpening)消歧,
//      仍并列则按固定面序稳定选择,保证重复运行结果一致。
MoldSurfaceClassification classifyHoleSegmentByMoldSurface(
    const Bbox& worldBounds,
    const MoldCoordinateFrame& frame,
    const gp_Pnt& first,
    const gp_Pnt& second,
    const gp_Pnt* topologyOpening,
    double axisToleranceDeg)
{
    MoldSurfaceClassification result;
    const LocalBounds bounds = localBounds(worldBounds, frame);
    gp_Vec segment(first, second);
    if (!bounds.valid || segment.Magnitude() <= Precision::Confusion())
        return result;
    segment.Normalize();

    // 只有近似平行于模具局部主轴的孔段才能可靠映射到六个装夹面。
    const std::array<gp_Vec, 3> axes{
        gp_Vec(frame.xAxis), gp_Vec(frame.yAxis), gp_Vec(frame.zAxis)};
    int dominantAxis = 0;
    double dominantDot = std::abs(segment.Dot(axes[0]));
    for (int axis = 1; axis < 3; ++axis) {
        const double dot = std::abs(segment.Dot(axes[axis]));
        if (dot > dominantDot) {
            dominantAxis = axis;
            dominantDot = dot;
        }
    }
    if (dominantDot < minimumAxisDot(axisToleranceDeg))
        return result;

    const std::array<gp_Pnt, 2> endpoints{first, second};
    const std::array<gp_Pnt, 2> localEndpoints{
        frame.worldToLocal(first), frame.worldToLocal(second)};
    const double span = bounds.maximum[dominantAxis] -
                        bounds.minimum[dominantAxis];
    const double tolerance = std::max(
        10.0 * Precision::Confusion(), std::abs(span) * 1.0e-9);

    // 两个端点分别与该主轴正/负包络面比较，共形成四个入口候选。
    struct Candidate {
        int endpoint = 0;
        MoldSetupFace face = MoldSetupFace::Unresolved;
        double distance = 0.0;
    };
    std::array<Candidate, 4> candidates;
    int candidateCount = 0;
    for (int endpoint = 0; endpoint < 2; ++endpoint) {
        const double value = coordinate(localEndpoints[endpoint], dominantAxis);
        candidates[candidateCount++] = Candidate{
            endpoint, positiveFace(dominantAxis),
            std::abs(value - bounds.maximum[dominantAxis])};
        candidates[candidateCount++] = Candidate{
            endpoint, negativeFace(dominantAxis),
            std::abs(value - bounds.minimum[dominantAxis])};
    }

    double minimumDistance = candidates[0].distance;
    for (const Candidate& candidate : candidates)
        minimumDistance = std::min(minimumDistance, candidate.distance);

    // 距离无法区分端点时，优先使用识别阶段提供的拓扑开口点消除歧义。
    int topologyEndpoint = -1;
    if (topologyOpening) {
        const double firstDistance = topologyOpening->Distance(first);
        const double secondDistance = topologyOpening->Distance(second);
        if (std::abs(firstDistance - secondDistance) > tolerance)
            topologyEndpoint = firstDistance < secondDistance ? 0 : 1;
    }

    std::vector<const Candidate*> tied;
    for (const Candidate& candidate : candidates) {
        if (std::abs(candidate.distance - minimumDistance) <= tolerance)
            tied.push_back(&candidate);
    }
    const bool distanceTie = tied.size() > 1;
    bool usedTopology = false;
    if (distanceTie && topologyEndpoint >= 0) {
        const bool hasTopologyCandidate = std::any_of(
            tied.begin(), tied.end(), [&](const Candidate* candidate) {
                return candidate->endpoint == topologyEndpoint;
            });
        if (hasTopologyCandidate) {
            tied.erase(std::remove_if(
                           tied.begin(), tied.end(), [&](const Candidate* candidate) {
                               return candidate->endpoint != topologyEndpoint;
                           }),
                       tied.end());
            usedTopology = true;
        }
    }
    // 完全对称且无拓扑线索时按面枚举和端点索引稳定排序，保证重复运行一致。
    std::stable_sort(tied.begin(), tied.end(), [](const Candidate* firstCandidate,
                                                   const Candidate* secondCandidate) {
        const int firstRank = static_cast<int>(firstCandidate->face);
        const int secondRank = static_cast<int>(secondCandidate->face);
        if (firstRank != secondRank)
            return firstRank < secondRank;
        return firstCandidate->endpoint < secondCandidate->endpoint;
    });
    if (tied.empty())
        return result;

    const Candidate& selected = *tied.front();
    const int otherEndpoint = 1 - selected.endpoint;
    const double otherCoordinate = coordinate(
        localEndpoints[otherEndpoint], dominantAxis);
    result.valid = true;
    result.entry = endpoints[selected.endpoint];
    result.bottom = endpoints[otherEndpoint];
    result.setupFace = selected.face;
    result.entryDistance = selected.distance;
    result.otherEndpointDistance = std::min(
        std::abs(otherCoordinate - bounds.minimum[dominantAxis]),
        std::abs(otherCoordinate - bounds.maximum[dominantAxis]));
    result.decision = !distanceTie
        ? MoldSetupFaceDecision::NearestDistance
        : usedTopology
            ? MoldSetupFaceDecision::TopologyTieBreak
            : MoldSetupFaceDecision::StableFaceOrder;
    return result;
}

// 已知装夹面时只决定孔段朝向:选离该装夹面包络更近的端点为入口,不重新推断面类别。
MoldSurfaceClassification orientHoleSegmentToMoldFace(
    const Bbox& worldBounds,
    const MoldCoordinateFrame& frame,
    MoldSetupFace setupFace,
    const gp_Pnt& first,
    const gp_Pnt& second,
    double axisToleranceDeg)
{
    MoldSurfaceClassification result;
    const LocalBounds bounds = localBounds(worldBounds, frame);
    int axisIndex = 0;
    bool positive = false;
    if (!bounds.valid || !faceAxis(setupFace, axisIndex, positive))
        return result;

    gp_Vec segment(first, second);
    if (segment.Magnitude() <= Precision::Confusion())
        return result;
    segment.Normalize();
    const std::array<gp_Vec, 3> axes{
        gp_Vec(frame.xAxis), gp_Vec(frame.yAxis), gp_Vec(frame.zAxis)};
    if (std::abs(segment.Dot(axes[axisIndex])) <
        minimumAxisDot(axisToleranceDeg))
        return result;

    const std::array<gp_Pnt, 2> endpoints{first, second};
    const std::array<gp_Pnt, 2> localEndpoints{
        frame.worldToLocal(first), frame.worldToLocal(second)};
    // 已知装夹面时只决定孔段朝向，不重新推断面类别。
    const double target = positive
        ? bounds.maximum[axisIndex]
        : bounds.minimum[axisIndex];
    const std::array<double, 2> targetDistances{
        std::abs(coordinate(localEndpoints[0], axisIndex) - target),
        std::abs(coordinate(localEndpoints[1], axisIndex) - target)};
    const int entryIndex = targetDistances[1] < targetDistances[0] ? 1 : 0;
    const int bottomIndex = 1 - entryIndex;
    const double bottomCoordinate = coordinate(
        localEndpoints[bottomIndex], axisIndex);

    result.valid = true;
    result.entry = endpoints[entryIndex];
    result.bottom = endpoints[bottomIndex];
    result.setupFace = setupFace;
    result.entryDistance = targetDistances[entryIndex];
    result.otherEndpointDistance = std::min(
        std::abs(bottomCoordinate - bounds.minimum[axisIndex]),
        std::abs(bottomCoordinate - bounds.maximum[axisIndex]));
    return result;
}

// 将任意外向向量(outwardAxis)映射到最近的模具装夹面;角度超容差返回 Unresolved。
MoldSetupFace classifyMoldSetupFace(
    const MoldCoordinateFrame& frame,
    const gp_Vec& outwardAxis,
    double axisToleranceDeg)
{
    if (!frame.isValid || outwardAxis.Magnitude() <= Precision::Confusion())
        return MoldSetupFace::Unresolved;

    gp_Vec axis = outwardAxis;
    axis.Normalize();
    const std::array<gp_Vec, 3> moldAxes{
        gp_Vec(frame.xAxis), gp_Vec(frame.yAxis), gp_Vec(frame.zAxis)};
    int dominantAxis = 0;
    double dominantDot = axis.Dot(moldAxes[0]);
    for (int candidate = 1; candidate < 3; ++candidate) {
        const double dot = axis.Dot(moldAxes[candidate]);
        if (std::abs(dot) > std::abs(dominantDot)) {
            dominantAxis = candidate;
            dominantDot = dot;
        }
    }
    if (std::abs(dominantDot) < minimumAxisDot(axisToleranceDeg))
        return MoldSetupFace::Unresolved;
    return dominantDot >= 0.0
        ? positiveFace(dominantAxis)
        : negativeFace(dominantAxis);
}

// 把 point 沿 outwardAxis 投影到指定装夹面的理想包络平面:轴向分量不足时返回
// false;distance<0(点已在包络内)按 0 处理仍返回 true。延长量写入 extensionLength。
bool projectPointToMoldSurface(
    const Bbox& worldBounds,
    const MoldCoordinateFrame& frame,
    MoldSetupFace face,
    const gp_Pnt& point,
    const gp_Vec& outwardAxis,
    gp_Pnt& projected,
    double& extensionLength,
    double axisToleranceDeg)
{
    projected = point;
    extensionLength = 0.0;
    const LocalBounds bounds = localBounds(worldBounds, frame);
    int faceAxisIndex = 0;
    bool positive = false;
    if (!bounds.valid || !faceAxis(face, faceAxisIndex, positive) ||
        outwardAxis.Magnitude() <= Precision::Confusion())
        return false;

    gp_Vec outward = outwardAxis;
    outward.Normalize();
    const std::array<gp_Vec, 3> moldAxes{
        gp_Vec(frame.xAxis), gp_Vec(frame.yAxis), gp_Vec(frame.zAxis)};
    const double component = outward.Dot(moldAxes[faceAxisIndex]);
    const double signedComponent = positive ? component : -component;
    if (signedComponent < minimumAxisDot(axisToleranceDeg))
        return false;

    const gp_Pnt local = frame.worldToLocal(point);
    const double target = positive
        ? bounds.maximum[faceAxisIndex]
        : bounds.minimum[faceAxisIndex];
    // 沿孔的外向轴投影到理想模具包络，而不是直接修改其他局部坐标分量。
    const double delta = target - coordinate(local, faceAxisIndex);
    const double distance = delta / component;
    const double span = bounds.maximum[faceAxisIndex] -
                        bounds.minimum[faceAxisIndex];
    const double tolerance = std::max(
        10.0 * Precision::Confusion(), std::abs(span) * 1.0e-9);
    if (!std::isfinite(distance) || distance < -tolerance)
        return false;

    extensionLength = std::max(0.0, distance);
    projected = point.Translated(outward.Multiplied(extensionLength));
    return true;
}

} // namespace oap
