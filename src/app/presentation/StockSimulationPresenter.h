// 孔毛坯与腔 IPW 叠加层的 OCCT 所有权和显示状态。
// 模块:oap_app_presentation。孔毛坯(HoleStock 覆盖组)与腔 IPW(PocketIpw 覆盖组)
// 的显示管理;PocketIpw 对象统一挂 m_pocketOwner 作为 Owner,便于一次性清理。
#pragma once

#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Triangulation.hxx>
#include <Standard_Transient.hxx>
#include <V3d_View.hxx>
#include <Poly_Triangulation.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

#include <string>

#include "app/view/IpwDisplayGeometry.h"

namespace oap::app::presentation {

class OverlayController;

// 仿真叠加呈现器:孔毛坯用 AIS_Shape 着色显示;腔 IPW 支持三种呈现形式
// (Poly_Triangulation 网格 / 中性图元 / BRep),三者互斥地替换 PocketIpw 组,
// 并由 m_pocketOwner 统一标记所有权,removePocketObjects() 按 Owner 清理
// context 中残留的对象。displayMode / 显示帧状态等 UI 参数也由本类维护。
class StockSimulationPresenter {
public:
    // 呈现结果:ok 表示成功;失败时 error 携带面向日志的错误信息。
    struct Result {
        bool ok = false;
        std::string error;
    };

    StockSimulationPresenter(
        OverlayController& overlays,
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view);

    // 重新绑定延迟创建的 OCCT context / view。
    void bindViewer(
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view);
    // 清空孔毛坯覆盖组。
    void clearHole();
    Handle(AIS_Shape) holePresentation() const;
    // 以 AIS_Shape 着色显示孔毛坯;已存在对象时复用句柄并 SetShape 换形,
    // 避免反复重建。matteMaterial 为 true 时应用哑光材质。
    Result presentHoleShape(
        const TopoDS_Shape& shape,
        const Quantity_Color& color,
        double transparency,
        int zLayer,
        const gp_Trsf& modelTransform,
        bool matteMaterial = false);

    // 清理全部腔 IPW 对象:覆盖组内 + 按 m_pocketOwner 在 context 中登记的
    // 对象一并 Deactivate/Erase/Remove,并刷新视图。
    void removePocketObjects();
    // 以 AIS_Triangulation 显示 IPW 网格,可配置边线/着色/线框模式。
    Result presentPocketTriangulation(
        const Handle(Poly_Triangulation)& mesh,
        const Quantity_Color& color,
        double transparency,
        bool topLayer,
        IpwOcctDisplayMode displayMode,
        const gp_Trsf& modelTransform);
    // 以自定义图元对象(MultiDexelPrimitivesObject)显示 IPW。
    Result presentPocketPrimitiveGeometry(
        const IpwDisplayPrimitiveGeometryPtr& geometry,
        const Quantity_Color& color,
        double transparency,
        bool topLayer,
        const gp_Trsf& modelTransform);
    // 以 AIS_Shape 着色显示 IPW 的 BRep 快照。
    Result presentPocketBRep(
        const TopoDS_Shape& shape,
        const Quantity_Color& color,
        double transparency,
        bool topLayer,
        const gp_Trsf& modelTransform);
    Handle(AIS_Triangulation) pocketTriangulation() const;
    const Handle(Standard_Transient)& pocketOwner() const noexcept
    {
        return m_pocketOwner;
    }
    Handle(AIS_InteractiveObject)& pocketObject() noexcept
    {
        return m_pocketObject;
    }
    const Handle(AIS_InteractiveObject)& pocketObject() const noexcept
    {
        return m_pocketObject;
    }

    int& displayQuality() noexcept { return m_displayQuality; }
    int displayQuality() const noexcept { return m_displayQuality; }
    bool& triangulationEnabled() noexcept { return m_triangulationEnabled; }
    bool triangulationEnabled() const noexcept { return m_triangulationEnabled; }
    IpwOcctDisplayMode& displayMode() noexcept { return m_displayMode; }
    IpwOcctDisplayMode displayMode() const noexcept { return m_displayMode; }

    int& displayedStepIndex() noexcept { return m_displayedStepIndex; }
    int displayedStepIndex() const noexcept { return m_displayedStepIndex; }
    int& displayedOperationIndex() noexcept { return m_displayedOperationIndex; }
    int displayedOperationIndex() const noexcept
    {
        return m_displayedOperationIndex;
    }
    int& displayedCompletedMoveCount() noexcept
    {
        return m_displayedCompletedMoveCount;
    }
    int displayedCompletedMoveCount() const noexcept
    {
        return m_displayedCompletedMoveCount;
    }
    bool& displayedFrameIsTransient() noexcept
    {
        return m_displayedFrameIsTransient;
    }
    bool displayedFrameIsTransient() const noexcept
    {
        return m_displayedFrameIsTransient;
    }
    void resetDisplayedFrame() noexcept;

private:
    OverlayController* m_overlays = nullptr;
    Handle(AIS_InteractiveContext) m_context;
    Handle(V3d_View) m_view;
    Handle(Standard_Transient) m_pocketOwner;
    Handle(AIS_InteractiveObject) m_pocketObject;
    int m_displayQuality = 10;
    bool m_triangulationEnabled = true;
    IpwOcctDisplayMode m_displayMode = IpwOcctDisplayMode::SurfaceWithEdges;
    int m_displayedStepIndex = -1;
    int m_displayedOperationIndex = -1;
    int m_displayedCompletedMoveCount = -1;
    bool m_displayedFrameIsTransient = false;
};

} // namespace oap::app::presentation
