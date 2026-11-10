// Portions derived from AnalysisSitus:
// Copyright (c) 2016-present, Sergey Slyadnev
// Copyright (c) 2017, Sergey Slyadnev
// Local modifications: Copyright (c) 2026 dialoug (MIT; see LICENSE).
// The following BSD-3-Clause terms continue to apply to upstream portions.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the copyright holder(s) nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// 二面角分析实现 —— 移植自 AnalysisSitus asiAlgo_CheckDihedralAngle.cpp，
// 剥离了 progress/plotter 调试可视化。算法逻辑与原版逐段对齐。
// 模块: oap_recognition。实现 AngleBetweenFaces 的公共边查找、same-parameter
//       取样、C1 平滑细分与旋转面 seam 边凹凸判定；公共边先验与 face->edge
//       缓存见 DihedralAngle.h。

#include "DihedralAngle.h"

#include <BRep_Tool.hxx>
#include <ElSLib.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Lin.hxx>

#include <optional>

namespace
{
    // 检查 face 的支撑面（或修剪面的基面）是否为 TSurf 类型，命中则填 basesurf。
    // 移植自 asiAlgo_Utils::IsTypeOf<TSurf>(face, basesurf)。
    template <typename TSurf>
    bool IsTypeOf(const TopoDS_Face &face, Handle(TSurf) &basesurf)
    {
        if (face.IsNull())
            return false;

        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        if (surf.IsNull())
            return false;

        // 直接命中支撑面。
        if (surf->IsInstance(STANDARD_TYPE(TSurf)))
        {
            basesurf = Handle(TSurf)::DownCast(surf);
            return true;
        }

        // 修剪面可能包裹目标面类型，解包基面再判。
        if (surf->IsInstance(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
        {
            Handle(Geom_RectangularTrimmedSurface) RT =
                Handle(Geom_RectangularTrimmedSurface)::DownCast(surf);
            if (RT.IsNull())
                return false;

            basesurf = Handle(TSurf)::DownCast(RT->BasisSurface());
            if (!basesurf.IsNull())
                return true;
        }

        return false;
    }
} // namespace

//-----------------------------------------------------------------------------

FeatureAngleType
DihedralAngleChecker::AngleBetweenFaces(const TopoDS_Face &F,
                                        const TopoDS_Face &G,
                                        const bool allowSmooth,
                                        const double smoothAngularTol,
                                        TopTools_IndexedMapOfShape &commonEdges,
                                        double &angleRad,
                                        gp_Pnt &FP,
                                        gp_Pnt &GP,
                                        gp_Vec &FN,
                                        gp_Vec &GN) const
{
    TopoDS_Edge commonEdge = m_commonEdge;
    bool isSeam = false;

    // 公共边可能先验已知。若未知则需要查找。
    if (commonEdge.IsNull())
    {
        if (F.IsSame(G))
        {
            // 尝试查找 seam 边。
            for (TopExp_Explorer exp(F, TopAbs_EDGE); exp.More(); exp.Next())
            {
                const TopoDS_Edge &edge = TopoDS::Edge(exp.Current());
                ShapeAnalysis_Edge sae;
                if (sae.IsSeam(edge, F))
                {
                    isSeam = true;
                    commonEdge = edge;
                    commonEdges.Add(edge);
                    break;
                }
            }
        }
        else
        {
            TopTools_IndexedMapOfShape EdgesF, EdgesG;

            // 使用 F 的边缓存。
            if (!m_bUseCache || !m_FECache.Contains(F))
            {
                TopExp::MapShapes(F, TopAbs_EDGE, EdgesF);
                if (m_bUseCache)
                {
                    TopTools_ListOfShape lst;
                    for (int i = 1; i <= EdgesF.Extent(); ++i)
                        lst.Append(EdgesF(i));
                    m_FECache.Add(F, lst);
                }
            }
            else
            {
                const TopTools_ListOfShape &lst = m_FECache.FindFromKey(F);
                for (TopTools_ListIteratorOfListOfShape it(lst); it.More(); it.Next())
                    EdgesF.Add(it.Value());
            }

            // 使用 G 的边缓存。
            if (!m_bUseCache || !m_FECache.Contains(G))
            {
                TopExp::MapShapes(G, TopAbs_EDGE, EdgesG);
                if (m_bUseCache)
                {
                    TopTools_ListOfShape lst;
                    for (int i = 1; i <= EdgesG.Extent(); ++i)
                        lst.Append(EdgesG(i));
                    m_FECache.Add(G, lst);
                }
            }
            else
            {
                const TopTools_ListOfShape &lst = m_FECache.FindFromKey(G);
                for (TopTools_ListIteratorOfListOfShape it(lst); it.More(); it.Next())
                    EdgesG.Add(it.Value());
            }

            // 收集公共边。
            for (int ef = 1; ef <= EdgesF.Extent(); ++ef)
            {
                for (int eg = 1; eg <= EdgesG.Extent(); ++eg)
                {
                    if (EdgesF(ef).IsSame(EdgesG(eg)))
                    {
                        commonEdges.Add(EdgesF(ef));
                        if (commonEdge.IsNull()) // 单条公共边即可分析。
                            commonEdge = TopoDS::Edge(EdgesF(ef));
                    }
                }
            }
        }
    }
    else
    {
        // 公共边已定，仅判断它是否为 seam 边——旋转面 seam 边有特殊规则。
        isSeam = ShapeAnalysis_Edge().IsSeam(commonEdge, F);
    }

    if (commonEdge.IsNull())
        return FeatureAngleType_Undefined;

    // 检查边是否具有 "same parameter" 属性。若有则可用其空间曲线配合
    // p-curve 在两面上取样；若无则方向估计不再合法，须解点反演（较重，
    // 仅对非法模型才会触发）。
    const bool isSameParam = BRep_Tool::SameParameter(commonEdge);

    // 取 co-edge 在各自宿主面上的朝向。
    TopAbs_Orientation CumulOriOnF = TopAbs_EXTERNAL, CumulOriOnG = TopAbs_EXTERNAL;
    //
    for (TopExp_Explorer exp(F, TopAbs_EDGE); exp.More(); exp.Next())
        if (exp.Current().IsSame(commonEdge))
        {
            CumulOriOnF = exp.Current().Orientation();
            break;
        }
    //
    if (!isSeam)
    {
        for (TopExp_Explorer exp(G, TopAbs_EDGE); exp.More(); exp.Next())
            if (exp.Current().IsSame(commonEdge))
            {
                CumulOriOnG = exp.Current().Orientation();
                break;
            }
    }
    else
        CumulOriOnG = ((CumulOriOnF == TopAbs_FORWARD) ? TopAbs_REVERSED : TopAbs_FORWARD);

    // 取宿主（共享）曲线。
    double f, l;
    Handle(Geom_Curve) probeCurve = BRep_Tool::Curve(commonEdge, f, l);
    if (probeCurve.IsNull())
        return FeatureAngleType_Undefined;

    // 在曲线上取两点。
    const double midParam = (f + l) * 0.5;
    const double paramStep = (l - f) * 0.01;
    const double A_param = midParam - paramStep;
    const double B_param = midParam + paramStep;
    //
    gp_Pnt A = probeCurve->Value(A_param);
    gp_Pnt B = probeCurve->Value(B_param);

    // 取宿主面。
    Handle(Geom_Surface) S1 = BRep_Tool::Surface(F),
                         S2 = isSeam ? S1 : BRep_Tool::Surface(G);
    Handle(Geom2d_Curve) c2d_F, c2d_G;

    gp_Pnt2d UV;
    gp_Vec TF, Ref; // F 的面内切向
    {
        // 主切向量 Vx。
        gp_Vec Vx = B.XYZ() - A.XYZ();
        if (Vx.Magnitude() < RealEpsilon())
            return FeatureAngleType_Undefined;
        //
        if (CumulOriOnF == TopAbs_REVERSED)
            Vx.Reverse();

        // (u,v) 为 S1 的参数空间，取点 A 在该面上的参数。
        if (isSameParam)
        {
            double cons_f, cons_l;
            try
            {
                c2d_F = BRep_Tool::CurveOnSurface(commonEdge, F, cons_f, cons_l);
            }
            catch (...)
            {
            }
            if (c2d_F.IsNull())
                return FeatureAngleType_Undefined; // 面非法
            UV = c2d_F->Value(A_param);
        }
        else
        {
            ShapeAnalysis_Surface SAS(S1);
            UV = SAS.ValueOfUV(A, 1.0e-1);
        }

        // 法向量 N（Vz）。
        gp_Pnt P;
        gp_Vec D1U, D1V;
        S1->D1(UV.X(), UV.Y(), P, D1U, D1V);
        //
        gp_Vec N = D1U.Crossed(D1V);
        if (N.Magnitude() < RealEpsilon())
            return FeatureAngleType_Undefined;
        //
        if (F.Orientation() == TopAbs_REVERSED)
            N.Reverse();

        // 取样属性。
        FP = P;
        FN = N;

        // 副切向量 Vy。
        gp_Vec Vy = N.Crossed(Vx);
        if (Vy.Magnitude() < RealEpsilon())
            return FeatureAngleType_Undefined;
        //
        TF = Vy.Normalized();

        // 参考方向 Ref。
        Ref = Vx.Normalized();
    }

    // 对规范旋转面上的 seam 边应用特殊规则：无需点反演，直接从法向场
    // 推出边的凹凸。
    if (isSeam)
    {
        bool isConcave = false;
        if (this->checkSeamVexity(FP, FN, F, isConcave))
        {
            angleRad = M_PI;
            return isConcave ? FeatureAngleType_SmoothConcave : FeatureAngleType_SmoothConvex;
        }
    }

    gp_Pnt2d ST;
    gp_Vec TG; // G 的面内切向
    {
        // 主切向量 Vx。
        gp_Vec Vx = B.XYZ() - A.XYZ();
        if (Vx.Magnitude() < RealEpsilon())
            return FeatureAngleType_Undefined;
        //
        if (CumulOriOnG == TopAbs_REVERSED)
            Vx.Reverse();

        // (s,t) 为 S2 的参数空间，取点 A 在该面上的参数。
        if (isSameParam)
        {
            double cons_f, cons_l;
            c2d_G = BRep_Tool::CurveOnSurface(commonEdge, G, cons_f, cons_l);
            if (c2d_G.IsNull())
                return FeatureAngleType_Undefined; // 面非法
            ST = c2d_G->Value(A_param);
        }
        else
        {
            ShapeAnalysis_Surface SAS(S2);
            ST = SAS.ValueOfUV(A, 1.0e-1);
        }

        // 法向量 N（Vz）。
        gp_Pnt P;
        gp_Vec D1S, D1T;
        S2->D1(ST.X(), ST.Y(), P, D1S, D1T);
        //
        gp_Vec N = D1S.Crossed(D1T);
        if (N.Magnitude() < RealEpsilon())
            return FeatureAngleType_Undefined;
        //
        if (G.Orientation() == TopAbs_REVERSED)
            N.Reverse();

        // 取样属性。
        GP = P;
        GN = N;

        // 副切向量 Vy。
        gp_Vec Vy = N.Crossed(Vx);
        if (Vy.Magnitude() < RealEpsilon())
            return FeatureAngleType_Undefined;
        //
        TG = Vy.Normalized();
    }

    // 计算二面角。
    angleRad = TF.AngleWithRef(TG, Ref);

    bool isSmooth = false;

    // 3° 是识别平滑角的默认角度容差。
    const double ang_tol = Max(smoothAngularTol, 3.0 / 180.0 * M_PI);
    //
    if (Abs(Abs(angleRad) - M_PI) < ang_tol)
    {
        isSmooth = true;

        if (allowSmooth)
            return FeatureAngleType_Smooth;

        // C1 接缝需要额外分析。沿面内向量做小偏移，在附加取样点查几何。
        // 这里用点反演而非 2D 行走，以保证向量共线（详见 AS 原注释）。
        gp_Pnt2d UV_shifted, ST_shifted;
        gp_Pnt S1_P, S2_P;
        gp_Vec TF_precised_vec, TG_precised_vec;
        gp_Dir TF_precised, TG_precised;

        int inSamples = 1;
        double inStep = 0.4;
        double avrAngleRad = 0.0;
        //
        for (int s = 1; s <= inSamples; ++s)
        {
            const double inDelta = inStep * s;

            S1_P = A.XYZ() + TF.XYZ() * inDelta;
            S2_P = A.XYZ() + TG.XYZ() * inDelta;

            ShapeAnalysis_Surface SAS1(S1), SAS2(S2);
            UV_shifted = SAS1.ValueOfUV(S1_P, 1.0e-1);
            ST_shifted = SAS2.ValueOfUV(S2_P, 1.0e-1);
            S1_P = SAS1.Value(UV_shifted);
            S2_P = SAS2.Value(ST_shifted);

            // 非法模型须防止零向量。
            TF_precised_vec = (S1_P.XYZ() - A.XYZ());
            TG_precised_vec = (S2_P.XYZ() - A.XYZ());
            //
            if (TF_precised_vec.Magnitude() < Precision::Confusion() ||
                TG_precised_vec.Magnitude() < Precision::Confusion())
                return FeatureAngleType_Undefined;
            //
            TF_precised = TF_precised_vec;
            TG_precised = TG_precised_vec;

            avrAngleRad += TF_precised.AngleWithRef(TG_precised, Ref);
        }
        //
        avrAngleRad /= inSamples;
        angleRad = avrAngleRad;
    }

    // 分类。
    FeatureAngleType angleType = FeatureAngleType_Undefined;
    if (angleRad < 0)
        angleType = isSmooth ? FeatureAngleType_SmoothConvex : FeatureAngleType_Convex;
    else
        angleType = isSmooth ? FeatureAngleType_SmoothConcave : FeatureAngleType_Concave;

    return angleType;
}

//-----------------------------------------------------------------------------

FeatureAngleType
DihedralAngleChecker::AngleBetweenFaces(const TopoDS_Face &F,
                                        const TopoDS_Face &G,
                                        const bool allowSmooth,
                                        const double smoothAngularTol,
                                        TopTools_IndexedMapOfShape &commonEdges,
                                        double &angleRad) const
{
    gp_Pnt FP, GP;
    gp_Vec FN, GN;
    return this->AngleBetweenFaces(F, G, allowSmooth, smoothAngularTol, commonEdges,
                                   angleRad, FP, GP, FN, GN);
}

//-----------------------------------------------------------------------------

FeatureAngleType
DihedralAngleChecker::AngleBetweenFaces(const TopoDS_Face &F,
                                        const TopoDS_Face &G,
                                        const bool allowSmooth,
                                        const double smoothAngularTol,
                                        TopTools_IndexedMapOfShape &commonEdges) const
{
    double angRad = 0.0;
    return this->AngleBetweenFaces(F, G, allowSmooth, smoothAngularTol, commonEdges, angRad);
}

//-----------------------------------------------------------------------------

FeatureAngleType
DihedralAngleChecker::AngleBetweenFaces(const TopoDS_Face &F,
                                        const TopoDS_Face &G,
                                        const bool allowSmooth,
                                        const double smoothAngularTol) const
{
    TopTools_IndexedMapOfShape commonEdges;
    double angRad = 0.0;
    return this->AngleBetweenFaces(F, G, allowSmooth, smoothAngularTol, commonEdges, angRad);
}

//-----------------------------------------------------------------------------

FeatureAngleType
DihedralAngleChecker::AngleBetweenFaces(const TopoDS_Face &F,
                                        const TopoDS_Face &G,
                                        const bool allowSmooth,
                                        const double smoothAngularTol,
                                        double &angleRad) const
{
    TopTools_IndexedMapOfShape commonEdges;
    return this->AngleBetweenFaces(F, G, allowSmooth, smoothAngularTol, commonEdges, angleRad);
}

//-----------------------------------------------------------------------------

bool DihedralAngleChecker::checkSeamVexity(const gp_Pnt &pt,
                                           const gp_Vec &norm,
                                           const TopoDS_Face &face,
                                           bool &isConcave) const
{
    std::optional<gp_Vec> axis;
    std::optional<gp_Pnt> origin;
    std::optional<double> diameter;
    bool isLabeled = false;

    /* 圆柱面 */
    {
        Handle(Geom_CylindricalSurface) surfCyl;
        if (IsTypeOf<Geom_CylindricalSurface>(face, surfCyl))
        {
            axis = surfCyl->Axis().Direction();
            origin = surfCyl->Axis().Location();
            diameter = 2 * surfCyl->Radius();
            isLabeled = true;
        }
    }

    /* 圆锥面 */
    if (!isLabeled)
    {
        Handle(Geom_ConicalSurface) coneSurf;
        if (IsTypeOf<Geom_ConicalSurface>(face, coneSurf))
        {
            gp_Cone cone = coneSurf->Cone();

            // 在锥面上反演点。
            double ucone, vcone;
            ElSLib::ConeParameters(cone.Position(),
                                   cone.RefRadius(),
                                   cone.SemiAngle(),
                                   pt,
                                   ucone, vcone);

            Handle(Geom_Curve) viso = coneSurf->VIso(vcone);
            if (!viso->IsInstance(STANDARD_TYPE(Geom_Circle)))
                return false;

            axis = coneSurf->Axis().Direction();
            origin = coneSurf->Axis().Location();
            diameter = 2 * Handle(Geom_Circle)::DownCast(viso)->Radius();
            isLabeled = true;
        }
    }

    /* 环面 */
    if (!isLabeled)
    {
        Handle(Geom_ToroidalSurface) torusSurf;
        if (IsTypeOf<Geom_ToroidalSurface>(face, torusSurf))
        {
            axis = torusSurf->Axis().Direction();
            origin = torusSurf->Axis().Location();
            diameter = 2 * (torusSurf->MajorRadius() + torusSurf->MinorRadius());
            isLabeled = true;
        }
    }

    /* 回转面 */
    if (!isLabeled)
    {
        Handle(Geom_SurfaceOfRevolution) revolSurf;
        if (IsTypeOf<Geom_SurfaceOfRevolution>(face, revolSurf))
        {
            axis = revolSurf->Axis().Direction();
            origin = revolSurf->Axis().Location();
            isLabeled = true;

            // 从母线极值推导直径。
            gp_Lin lin(*origin, *axis);
            Handle(Geom_Curve) gnx = revolSurf->BasisCurve();
            const double radii[2] = {lin.Distance(gnx->Value(gnx->FirstParameter())),
                                     lin.Distance(gnx->Value(gnx->LastParameter()))};
            diameter = 2 * Max(radii[0], radii[1]);
        }
    }

    /* 球面 */
    if (!isLabeled)
    {
        Handle(Geom_SphericalSurface) sphereSurf;
        if (IsTypeOf<Geom_SphericalSurface>(face, sphereSurf))
        {
            axis = sphereSurf->Axis().Direction();
            origin = sphereSurf->Axis().Location();
            diameter = 2 * sphereSurf->Radius();
            isLabeled = true;
        }
    }

    if (!isLabeled)
        return false;

    // 沿法向取探针点。
    gp_Pnt normProbe = pt.XYZ() + norm.Normalized().XYZ() * (*diameter) * 0.05;

    // 计算到轴的距离。
    gp_Lin axisLin(*origin, *axis);
    //
    const double probeDist = axisLin.Distance(normProbe);
    const double origDist = axisLin.Distance(pt);
    //
    isConcave = (probeDist < origDist);
    return true;
}
