// 二面角、边方向和面法向相关的识别几何工具。
// 模块: oap_recognition。DihedralAngleChecker 在两面公共边处构造局部正交标架，
//       求带符号二面角并输出 6 态 FeatureAngleType。
// 消费方: Recognition.cpp 的 ClassifyEdge（判定孔壁端边凹凸），凹凸分析移植自
//       AnalysisSitus，被 oap_analysis 的曲面分类逻辑共享移植。
#pragma once

// 二面角分析 —— 移植自 AnalysisSitus 的 asiAlgo_CheckDihedralAngle /
// asiAlgo_FeatureAngleType。剥离了 ActAPI progress/plotter 等调试脚手架，
// 只保留纯几何算法 + 本地 face-edge 缓存。
//
// 源：
//   AnalysisSitus-master/src/asiAlgo/features/asiAlgo_CheckDihedralAngle.{h,cpp}
//   AnalysisSitus-master/src/asiAlgo/features/asiAlgo_FeatureAngleType.h

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

// ══════════════════════════════════════════════════════════════
//  二面角类型（6 态）
// ══════════════════════════════════════════════════════════════
// 数值取自 Joshi "Graph-based heuristics for recognition of machined
// features from a 3D solid model" (1988)，与 AnalysisSitus 保持一致。
enum FeatureAngleType
{
    FeatureAngleType_Undefined     = -1,
    FeatureAngleType_Concave       =  0,
    FeatureAngleType_Convex        =  1,
    FeatureAngleType_Smooth        =  2,
    FeatureAngleType_SmoothConcave =  3,
    FeatureAngleType_SmoothConvex  =  4,
    FeatureAngleType_NonManifold   =  5,
    //
    FeatureAngleType_LAST
};

// ══════════════════════════════════════════════════════════════
//  类型谓词工具
// ══════════════════════════════════════════════════════════════
namespace FeatureAngle
{
    // 非 Undefined 且非 NonManifold 时返回 true。
    inline bool IsDefinite(const FeatureAngleType type)
    {
        return type != FeatureAngleType_Undefined &&
               type != FeatureAngleType_NonManifold;
    }

    // 平滑角（Smooth / SmoothConcave / SmoothConvex）。
    inline bool IsSmooth(const FeatureAngleType type)
    {
        return type == FeatureAngleType_Smooth ||
               type == FeatureAngleType_SmoothConcave ||
               type == FeatureAngleType_SmoothConvex;
    }

    // 凸角（Convex / SmoothConvex）。
    inline bool IsConvex(const FeatureAngleType type)
    {
        return type == FeatureAngleType_Convex ||
               type == FeatureAngleType_SmoothConvex;
    }

    // 凹角（Concave / SmoothConcave）。
    inline bool IsConcave(const FeatureAngleType type)
    {
        return type == FeatureAngleType_Concave ||
               type == FeatureAngleType_SmoothConcave;
    }

    // 转字符串（调试用）。
    inline const char *ToString(const FeatureAngleType angle)
    {
        switch (angle)
        {
        case FeatureAngleType_Concave:       return "concave";
        case FeatureAngleType_Convex:        return "convex";
        case FeatureAngleType_Smooth:        return "smooth";
        case FeatureAngleType_SmoothConcave: return "smooth concave";
        case FeatureAngleType_SmoothConvex:  return "smooth convex";
        case FeatureAngleType_NonManifold:   return "non-manifold";
        case FeatureAngleType_Undefined:
        default:                             break;
        }
        return "undefined";
    }
} // namespace FeatureAngle

// ══════════════════════════════════════════════════════════════
//  二面角计算器
// ══════════════════════════════════════════════════════════════
// 对应 AnalysisSitus 的 asiAlgo_CheckDihedralAngle。在两面公共边处构建
// 局部正交标架，求带符号的真实二面角（弧度），并分类为 6 态。
class DihedralAngleChecker
{
public:
    DihedralAngleChecker() = default;

    // 计算两面间的二面角。用公共边中点附近取样查询法向，考虑面的朝向。
    //   F, G              两个面
    //   allowSmooth       是否将 C1 平滑接缝直接判为 Smooth（不再细分凹凸）
    //   smoothAngularTol  平滑角识别的角度容差（弧度），下限 3°
    //   commonEdges       [out] 公共边集合
    //   angleRad          [out] 带符号角度（弧度）
    //   FP, GP            [out] F / G 上的取样点
    //   FN, GN            [out] F / G 上的法向
    FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face &F,
                      const TopoDS_Face &G,
                      const bool allowSmooth,
                      const double smoothAngularTol,
                      TopTools_IndexedMapOfShape &commonEdges,
                      double &angleRad,
                      gp_Pnt &FP,
                      gp_Pnt &GP,
                      gp_Vec &FN,
                      gp_Vec &GN) const;

    FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face &F,
                      const TopoDS_Face &G,
                      const bool allowSmooth,
                      const double smoothAngularTol,
                      TopTools_IndexedMapOfShape &commonEdges,
                      double &angleRad) const;

    FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face &F,
                      const TopoDS_Face &G,
                      const bool allowSmooth,
                      const double smoothAngularTol,
                      TopTools_IndexedMapOfShape &commonEdges) const;

    FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face &F,
                      const TopoDS_Face &G,
                      const bool allowSmooth,
                      const double smoothAngularTol) const;

    FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face &F,
                      const TopoDS_Face &G,
                      const bool allowSmooth,
                      const double smoothAngularTol,
                      double &angleRad) const;

public:
    // 是否启用 face-edge 缓存。
    void SetUseCache(const bool on) { m_bUseCache = on; }

    // 设定先验已知的公共边（设定后跳过公共边搜索）。
    void SetCommonEdge(const TopoDS_Edge &commonEdge) { m_commonEdge = commonEdge; }

protected:
    // 当 face 为旋转面且在其 seam 边上做凹凸分析时，沿法向探针判断旋转
    // 直径沿法向是增大还是减小，从而定凹凸。
    //   返回 false 表示判定失败，调用方不应使用 isConcave。
    bool checkSeamVexity(const gp_Pnt &pt,
                         const gp_Vec &norm,
                         const TopoDS_Face &face,
                         bool &isConcave) const;

protected:
    TopoDS_Edge m_commonEdge; //!< 可选的先验公共边。

    //! face → edges 缓存。
    mutable TopTools_IndexedDataMapOfShapeListOfShape m_FECache;

    //! 是否使用缓存。
    bool m_bUseCache = true;
};
