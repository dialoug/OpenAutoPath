// 依据模具六个理想外包络面稳定确定孔段入口和装夹面的公共几何 API。
// 属于 oap_analysis 的"模具坐标系"部分:只做纯几何计算,不依赖 UI;
// 供孔识别与装夹规划两方共享,所有函数以 MoldCoordinateFrame 描述模具姿态。
#pragma once

#include "analysis/FeatureTypes.h"
#include "analysis/Model.h"
#include "analysis/MoldCoordinateFrame.h"

#include <gp_Vec.hxx>

namespace oap {

// 一个孔段相对模具六面的分类结果:入口/底端点、命中的装夹面、决策原因。
struct MoldSurfaceClassification {
    bool valid = false;
    gp_Pnt entry;
    gp_Pnt bottom;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    double entryDistance = -1.0;
    double otherEndpointDistance = -1.0;
    MoldSetupFaceDecision decision = MoldSetupFaceDecision::None;
};

// 对孔段两端点(first/second)在模具六面上做投影,确定入口端与底端并给出装夹面决策。
// worldBounds 为模具整体包围盒;topologyOpening 若给定则作为孔在拓扑上的开口参考;
// axisToleranceDeg 为轴向判定容差(度)。返回的分类含命中面与决策原因。
MoldSurfaceClassification classifyHoleSegmentByMoldSurface(
    const Bbox& worldBounds,
    const MoldCoordinateFrame& frame,
    const gp_Pnt& first,
    const gp_Pnt& second,
    const gp_Pnt* topologyOpening = nullptr,
    double axisToleranceDeg = 1.0);

// 按指定装夹面将孔段定向:返回入口/底端分类,入口端落在该面上。
MoldSurfaceClassification orientHoleSegmentToMoldFace(
    const Bbox& worldBounds,
    const MoldCoordinateFrame& frame,
    MoldSetupFace setupFace,
    const gp_Pnt& first,
    const gp_Pnt& second,
    double axisToleranceDeg = 1.0);

// 将给定外法向向量判定为六个模具面之一(在 axisToleranceDeg 容差内)。
MoldSetupFace classifyMoldSetupFace(
    const MoldCoordinateFrame& frame,
    const gp_Vec& outwardAxis,
    double axisToleranceDeg = 1.0);

// 将 point 沿 outwardAxis 投影到指定装夹面;成功时经 projected 输出投影点、
// 经 extensionLength 输出投影距离,返回 true;无法投影时返回 false。
bool projectPointToMoldSurface(
    const Bbox& worldBounds,
    const MoldCoordinateFrame& frame,
    MoldSetupFace face,
    const gp_Pnt& point,
    const gp_Vec& outwardAxis,
    gp_Pnt& projected,
    double& extensionLength,
    double axisToleranceDeg = 1.0);

} // namespace oap
