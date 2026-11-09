// OverlayController 语义分组、替换、可见性和统一清理测试。
#include "app/presentation/OverlayController.h"
#include "app/presentation/HoleStageHighlightPresenter.h"
#include "app/presentation/PlaybackOverlayPolicy.h"
#include "app/presentation/PlaybackPresenter.h"
#include "app/presentation/StockSimulationPresenter.h"
#include "app/presentation/ToolpathPreviewPresenter.h"
#include "app/presentation/ToolpathSegmentsObject.h"

#include <AIS_Shape.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Prs3d_Drawer.hxx>

#include <iostream>
#include <memory>
#include <unordered_set>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

class RecordingSink final :
    public oap::app::presentation::PresentationSink {
public:
    explicit RecordingSink(bool initiallyBound = true)
        : bound(initiallyBound)
    {
    }
    void bindContext(const Handle(AIS_InteractiveContext)&) override
    {
        bound = true;
        ++bindCount;
    }
    void display(const Handle(AIS_InteractiveObject)& object) override
    {
        if (!bound)
            return;
        ++displayCount;
        displayed.insert(object.get());
    }
    void erase(const Handle(AIS_InteractiveObject)& object) override
    {
        ++eraseCount;
        displayed.erase(object.get());
    }
    void remove(const Handle(AIS_InteractiveObject)& object) override
    {
        ++removeCount;
        displayed.erase(object.get());
    }
    void redisplay(const Handle(AIS_InteractiveObject)&) override
    {
        ++redisplayCount;
    }
    void deactivate(const Handle(AIS_InteractiveObject)&) override
    {
        ++deactivateCount;
    }
    void setDisplayPriority(
        const Handle(AIS_InteractiveObject)&, int priority) override
    {
        lastPriority = priority;
    }
    bool isDisplayed(
        const Handle(AIS_InteractiveObject)& object) const override
    {
        return displayed.contains(object.get());
    }

    int displayCount = 0;
    int eraseCount = 0;
    int removeCount = 0;
    int redisplayCount = 0;
    int deactivateCount = 0;
    int lastPriority = -1;
    int bindCount = 0;
    bool bound = true;
    std::unordered_set<const AIS_InteractiveObject*> displayed;
};

Handle(AIS_InteractiveObject) box(double x)
{
    Handle(AIS_Shape) shape = new AIS_Shape(
        BRepPrimAPI_MakeBox(
            gp_Pnt(x, 0.0, 0.0), 1.0, 1.0, 1.0).Shape());
    return shape;
}

bool ownsAndReplacesSemanticGroups()
{
    using namespace oap::app::presentation;
    const auto sink = std::make_shared<RecordingSink>();
    OverlayController controller(sink);
    const Handle(AIS_InteractiveObject) first = box(0.0);
    const Handle(AIS_InteractiveObject) second = box(2.0);
    const Handle(AIS_InteractiveObject) third = box(4.0);

    OverlayDisplayOptions options;
    options.displayPriority = 9;
    controller.replace(OverlayGroup::Toolpath, {first}, true, options);
    if (!expect(controller.size(OverlayGroup::Toolpath) == 1 &&
                    controller.first(OverlayGroup::Toolpath) == first &&
                    sink->displayCount == 1 && sink->deactivateCount == 1 &&
                    sink->lastPriority == 9,
                "replace should register and display one owned object")) {
        return false;
    }

    controller.replace(OverlayGroup::Toolpath, {second}, true, options);
    controller.setDisplayPriority(OverlayGroup::Toolpath, 0, 7);
    if (!expect(sink->removeCount == 1 &&
                    controller.first(OverlayGroup::Toolpath) == second &&
                    sink->lastPriority == 7,
                "replace should remove the previous group before publishing")) {
        return false;
    }

    controller.setVisible(OverlayGroup::Toolpath, false);
    controller.setVisible(OverlayGroup::Toolpath, true);
    controller.redisplay(OverlayGroup::Toolpath);
    if (!expect(sink->eraseCount == 1 && sink->displayCount == 3 &&
                    sink->redisplayCount == 1,
                "visibility and redisplay should go through the sink")) {
        return false;
    }

    controller.append(OverlayGroup::TemporaryHighlight, third);
    if (!expect(controller.size(OverlayGroup::TemporaryHighlight) == 1 &&
                    controller.at(OverlayGroup::TemporaryHighlight, 0) == third &&
                    sink->displayed.contains(third.get()),
                "append should publish and retain one semantic object")) {
        return false;
    }

    controller.replace(
        OverlayGroup::PlaybackRemainingPath, {box(6.0)}, false);
    controller.clearAllTransient();
    return expect(controller.size(OverlayGroup::Toolpath) == 0 &&
                      controller.size(
                          OverlayGroup::PlaybackRemainingPath) == 0 &&
                      sink->removeCount == 4 && sink->displayed.empty(),
                  "clearAllTransient should remove every registered group");
}

bool republishesVisibleObjectsAfterLateContextBinding()
{
    using namespace oap::app::presentation;
    const auto sink = std::make_shared<RecordingSink>(false);
    OverlayController controller(sink);
    const Handle(AIS_InteractiveObject) toolpath = box(0.0);

    controller.replace(OverlayGroup::Toolpath, {toolpath});
    if (!expect(
            sink->displayed.empty() && controller.size(OverlayGroup::Toolpath) == 1,
            "an overlay created before viewer initialization must remain owned")) {
        return false;
    }

    controller.bindContext(Handle(AIS_InteractiveContext){});
    return expect(
        sink->bindCount == 1 && sink->displayed.contains(toolpath.get()) &&
            sink->displayCount == 1,
        "late context binding must publish previously registered visible overlays");
}

bool retainsPlaybackObjectsUntilLateContextBinding()
{
    using namespace oap::app::presentation;
    const auto sink = std::make_shared<RecordingSink>(false);
    OverlayController controller(sink);
    PlaybackPresenter presenter(
        controller, Handle(AIS_InteractiveContext){}, Handle(V3d_View){});

    PlaybackRemainingSegment segment;
    segment.from = gp_Pnt(0.0, 0.0, 0.0);
    segment.to = gp_Pnt(1.0, 0.0, 0.0);
    segment.color = Quantity_Color(Quantity_NOC_GREEN);
    presenter.updateRemainingPath(
        {segment}, 0, 1, 0.0, gp_Trsf{}, 0);
    if (!expect(
            controller.size(OverlayGroup::PlaybackRemainingPath) == 1,
            "remaining path must remain owned while the context is unavailable")) {
        return false;
    }
    presenter.updateTipMarker(
        gp_Pnt(0.5, 0.0, 0.0), 2.0, gp_Trsf{},
        Quantity_Color(Quantity_NOC_RED), 4.0);

    if (!expect(
            controller.size(OverlayGroup::PlaybackCursor) == 2,
            "cursor objects must remain owned while the context is unavailable")) {
        return false;
    }
    if (!expect(
            sink->displayed.empty(),
            "unbound sink must not display retained playback objects")) {
        return false;
    }

    controller.bindContext(Handle(AIS_InteractiveContext){});
    return expect(
        sink->displayed.size() == 3,
        "late context binding must publish retained remaining-path and cursor objects");
}

bool retainsPocketIpwUntilLateContextBinding()
{
    using namespace oap::app::presentation;
    const auto sink = std::make_shared<RecordingSink>(false);
    OverlayController controller(sink);
    StockSimulationPresenter presenter(
        controller, Handle(AIS_InteractiveContext){}, Handle(V3d_View){});

    const StockSimulationPresenter::Result result =
        presenter.presentPocketBRep(
            BRepPrimAPI_MakeBox(2.0, 2.0, 2.0).Shape(),
            Quantity_Color(Quantity_NOC_GREEN), 0.2, false, gp_Trsf{});
    if (!expect(
            result.ok && controller.size(OverlayGroup::PocketIpw) == 1 &&
                sink->displayed.empty(),
            "valid IPW geometry must be retained before viewer initialization")) {
        return false;
    }

    controller.bindContext(Handle(AIS_InteractiveContext){});
    return expect(
        sink->displayed.size() == 1,
        "late context binding must publish the retained IPW presentation");
}

bool playbackHidesEveryStaticToolpathLayer()
{
    using namespace oap::app::presentation;

    const PlaybackStaticOverlayVisibility start =
        playbackStaticOverlayVisibility(true, 0, 1.0, false);
    if (!expect(start.fullToolpath && !start.currentToolpath &&
                    start.holeStageHighlight,
                "unplayed program should show all applicable static layers")) {
        return false;
    }

    const PlaybackStaticOverlayVisibility playing =
        playbackStaticOverlayVisibility(true, 0, 0.25, false);
    if (!expect(!playing.fullToolpath && !playing.currentToolpath &&
                    !playing.holeStageHighlight,
                "playback should hide full, current-tool, and stage-highlight layers")) {
        return false;
    }

    const PlaybackStaticOverlayVisibility filteredStart =
        playbackStaticOverlayVisibility(true, 0, 1.0, true);
    return expect(!filteredStart.fullToolpath &&
                      filteredStart.currentToolpath &&
                      filteredStart.holeStageHighlight,
                  "current-tool mode should select only its static preview at reset");
}

bool playbackTransientClearPreservesRemainingPath()
{
    using namespace oap::app::presentation;
    const auto sink = std::make_shared<RecordingSink>();
    OverlayController controller(sink);
    PlaybackPresenter presenter(
        controller, Handle(AIS_InteractiveContext){}, Handle(V3d_View){});

    controller.replace(
        OverlayGroup::PlaybackRemainingPath, {box(0.0)});
    controller.replace(OverlayGroup::PlaybackTrail, {box(2.0)});
    controller.replace(OverlayGroup::PlaybackCursor, {box(4.0)});

    presenter.clearCursorAndTrail();
    if (!expect(
            controller.size(OverlayGroup::PlaybackRemainingPath) == 1 &&
                controller.size(OverlayGroup::PlaybackTrail) == 0 &&
                controller.size(OverlayGroup::PlaybackCursor) == 0,
            "rewind cleanup should preserve the remaining-path presentation")) {
        return false;
    }

    presenter.clear();
    return expect(
        controller.size(OverlayGroup::PlaybackRemainingPath) == 0,
        "full playback cleanup should remove the remaining path");
}

bool presentsSemanticToolpathLayersAsOneOwnedGroup()
{
    using namespace oap::app::presentation;
    const auto sink = std::make_shared<RecordingSink>();
    OverlayController controller(sink);
    ToolpathPreviewPresenter presenter(controller);

    std::vector<oap::appdetail::HolePreviewEdge> edges;
    edges.push_back({
        BRepBuilderAPI_MakeEdge(
            gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(1.0, 0.0, 0.0)).Edge(),
        oap::appdetail::HolePreviewSegmentKind::Cut});
    edges.push_back({
        BRepBuilderAPI_MakeEdge(
            gp_Pnt(1.0, 0.0, 0.0), gp_Pnt(2.0, 0.0, 0.0)).Edge(),
        oap::appdetail::HolePreviewSegmentKind::Rapid});
    edges.push_back({
        BRepBuilderAPI_MakeEdge(
            gp_Pnt(2.0, 0.0, 0.0), gp_Pnt(2.0, 0.0, 1.0)).Edge(),
        oap::appdetail::HolePreviewSegmentKind::Vertical});

    ToolpathPreviewStyle style;
    style.cutColor = Quantity_Color(Quantity_NOC_GREEN);
    style.rapidColor = Quantity_Color(Quantity_NOC_BLUE);
    style.verticalColor = Quantity_Color(Quantity_NOC_YELLOW);
    style.outlineColor = Quantity_Color(Quantity_NOC_BLACK);
    style.startColor = Quantity_Color(Quantity_NOC_RED);
    style.endColor = Quantity_Color(Quantity_NOC_MAGENTA1);
    const ToolpathPreviewPresentationStats stats = presenter.replace(
        OverlayGroup::Toolpath, edges,
        {{gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(2.0, 0.0, 1.0)}},
        gp_Trsf{}, style);
    presenter.setOperationFilter(OverlayGroup::Toolpath, {}, {});
    return expect(
        stats.cutEdges == 1 && stats.rapidEdges == 1 &&
            stats.verticalEdges == 1 && stats.objectCount == 1 &&
            controller.size(OverlayGroup::Toolpath) == 1 &&
            presenter.segmentCount(OverlayGroup::Toolpath) == 3 &&
            sink->displayCount == 1 && sink->redisplayCount == 1,
        "toolpath presenter should publish one selectable GPU segment object");
}

bool customToolpathHighlightContractDoesNotThrow()
{
    using namespace oap::app::presentation;

    ToolpathPreviewSegment segment;
    segment.from = gp_Pnt(0.0, 0.0, 0.0);
    segment.to = gp_Pnt(1.0, 0.0, 0.0);
    ToolpathPreviewStyle style;
    style.cutColor = Quantity_Color(Quantity_NOC_GREEN);
    style.rapidColor = Quantity_Color(Quantity_NOC_BLUE);
    style.verticalColor = Quantity_Color(Quantity_NOC_YELLOW);
    style.outlineColor = Quantity_Color(Quantity_NOC_BLACK);
    style.startColor = Quantity_Color(Quantity_NOC_RED);
    style.endColor = Quantity_Color(Quantity_NOC_MAGENTA1);

    Handle(ToolpathSegmentsObject) object =
        new ToolpathSegmentsObject({segment}, {}, style);
    Handle(SelectMgr_SelectableObject) selectable = object;
    Handle(ToolpathSegmentOwner) owner =
        new ToolpathSegmentOwner(selectable, 0);
    const Handle(Prs3d_Drawer) highlightStyle = new Prs3d_Drawer;

    try {
        owner->HilightWithColor(
            Handle(PrsMgr_PresentationManager){}, highlightStyle, 0);
        SelectMgr_SequenceOfOwner owners;
        owners.Append(owner);
        selectable->HilightSelected(
            Handle(PrsMgr_PresentationManager){}, owners);
        selectable->ClearDynamicHighlight(
            Handle(PrsMgr_PresentationManager){});
        selectable->ClearSelected();
    } catch (...) {
        return expect(false,
                      "custom toolpath hover/selection callbacks must not throw");
    }

    return expect(!selectable->IsAutoHilight(),
                  "toolpath object should retain per-segment custom highlighting");
}

bool isolatesToolpathHoverPresentation()
{
    using namespace oap::app::presentation;

    ToolpathPreviewSegment segment;
    segment.from = gp_Pnt(1.0, 2.0, 3.0);
    segment.to = gp_Pnt(4.0, 6.0, 3.0);
    ToolpathPreviewStyle style;
    style.directionColor = Quantity_Color(Quantity_NOC_ORANGE);
    style.directionWidth = 4.0;

    Handle(ToolpathSegmentsObject) toolpath =
        new ToolpathSegmentsObject({segment}, {}, style);
    Handle(ToolpathHoverObject) hover = new ToolpathHoverObject();
    hover->setSegment(segment, toolpath->style());

    const ToolpathPreviewSegment* hovered = hover->segment();
    if (!expect(
            hover->hasSegment() && hovered != nullptr &&
                hover->IsInfinite() &&
                hovered->from.Distance(segment.from) <= 1.0e-12 &&
                hovered->to.Distance(segment.to) <= 1.0e-12 &&
                toolpath->isSegmentVisible(0),
            "hover feedback should live in a separate lightweight object")) {
        return false;
    }

    hover->clearSegment();
    toolpath->setVisibleRange(1, 0);
    return expect(
        !hover->hasSegment() && !toolpath->isSegmentVisible(0),
        "hover feedback should clear without changing main toolpath geometry");
}

bool filtersToolpathSegmentsByOperation()
{
    using namespace oap::app::presentation;

    ToolpathPreviewSegment first;
    first.from = gp_Pnt(0.0, 0.0, 0.0);
    first.to = gp_Pnt(1.0, 0.0, 0.0);
    first.identity.operationId = "operation-a";
    ToolpathPreviewSegment second;
    second.from = gp_Pnt(1.0, 0.0, 0.0);
    second.to = gp_Pnt(2.0, 0.0, 0.0);
    second.identity.operationId = "operation-b";
    Handle(ToolpathSegmentsObject) toolpath =
        new ToolpathSegmentsObject({first, second}, {}, {});

    toolpath->setOperationFilter({"operation-a"}, {});
    if (!expect(
            !toolpath->isSegmentVisible(0) &&
                toolpath->isSegmentVisible(1),
            "hidden operations must be excluded from drawing and selection")) {
        return false;
    }

    toolpath->setOperationFilter({"operation-a"}, "operation-a");
    if (!expect(
            toolpath->isSegmentVisible(0) &&
                !toolpath->isSegmentVisible(1),
            "solo mode must override checkbox visibility for one operation")) {
        return false;
    }

    toolpath->setOperationFilter({}, {});
    return expect(
        toolpath->isSegmentVisible(0) && toolpath->isSegmentVisible(1),
        "clearing the operation filter must restore the full toolpath");
}

bool preservesPreviewIdentityAcrossOverlapClipping()
{
    using oap::appdetail::HolePreviewSegment;
    using oap::appdetail::HolePreviewSegmentKind;
    const std::vector<HolePreviewSegment> resolved =
        oap::appdetail::resolveHolePreviewColorOverlaps({
            {gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(10.0, 0.0, 0.0),
             HolePreviewSegmentKind::Rapid, 42},
            {gp_Pnt(4.0, 0.0, 0.0), gp_Pnt(6.0, 0.0, 0.0),
             HolePreviewSegmentKind::Cut, 7}});
    int rapidFragments = 0;
    int cutFragments = 0;
    for (const HolePreviewSegment& segment : resolved) {
        if (segment.kind == HolePreviewSegmentKind::Rapid) {
            ++rapidFragments;
            if (segment.sourceIndex != 42)
                return expect(false, "rapid fragments must retain source identity");
        } else if (segment.kind == HolePreviewSegmentKind::Cut) {
            ++cutFragments;
            if (segment.sourceIndex != 7)
                return expect(false, "cut fragments must retain source identity");
        }
    }
    return expect(
        rapidFragments == 2 && cutFragments == 1,
        "overlap clipping should split rapid display without losing identity");
}

bool appendsPocketLayersAndOwnsHoleStageHighlight()
{
    using namespace oap::app::presentation;
    const auto sink = std::make_shared<RecordingSink>();
    OverlayController controller(sink);
    ToolpathPreviewPresenter toolpaths(controller);

    std::vector<oap::appdetail::HolePreviewEdge> edges{{
        BRepBuilderAPI_MakeEdge(
            gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(1.0, 0.0, 0.0)).Edge(),
        oap::appdetail::HolePreviewSegmentKind::Cut}};
    ToolpathPreviewStyle previewStyle;
    previewStyle.cutColor = Quantity_Color(Quantity_NOC_GREEN);
    previewStyle.rapidColor = Quantity_Color(Quantity_NOC_BLUE);
    previewStyle.verticalColor = Quantity_Color(Quantity_NOC_YELLOW);
    previewStyle.outlineColor = Quantity_Color(Quantity_NOC_BLACK);
    previewStyle.startColor = Quantity_Color(Quantity_NOC_RED);
    previewStyle.endColor = Quantity_Color(Quantity_NOC_RED);
    previewStyle.showEndpoints = false;
    const auto first = toolpaths.append(
        OverlayGroup::Toolpath, edges, {}, gp_Trsf{}, previewStyle);
    const auto second = toolpaths.append(
        OverlayGroup::Toolpath, edges, {}, gp_Trsf{}, previewStyle);
    if (!expect(
            first.objectCount == 1 && second.objectCount == 1 &&
                controller.size(OverlayGroup::Toolpath) == 2 &&
                toolpaths.segmentCount(OverlayGroup::Toolpath) == 2,
            "incremental pocket previews should share one owned semantic group")) {
        return false;
    }

    HoleStageHighlightPresenter highlight(controller);
    HoleStageHighlightStyle highlightStyle;
    highlightStyle.wallColor = Quantity_Color(Quantity_NOC_YELLOW);
    highlightStyle.pathColor = Quantity_Color(Quantity_NOC_GREEN);
    highlightStyle.outlineColor = Quantity_Color(Quantity_NOC_BLACK);
    const TopoDS_Shape walls = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();
    const TopoDS_Shape path = BRepBuilderAPI_MakeEdge(
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0.0, 0.0, 1.0)).Edge();
    return expect(
        highlight.replace(
            walls, path, gp_Trsf{}, highlightStyle, true) &&
            controller.size(OverlayGroup::HoleStageHighlight) == 3,
        "hole-stage presenter should own wall, outline, and path objects");
}

} // namespace

int main()
{
    if (!ownsAndReplacesSemanticGroups() ||
        !republishesVisibleObjectsAfterLateContextBinding() ||
        !retainsPlaybackObjectsUntilLateContextBinding() ||
        !retainsPocketIpwUntilLateContextBinding() ||
        !playbackHidesEveryStaticToolpathLayer() ||
        !playbackTransientClearPreservesRemainingPath() ||
        !presentsSemanticToolpathLayersAsOneOwnedGroup() ||
        !customToolpathHighlightContractDoesNotThrow() ||
        !isolatesToolpathHoverPresentation() ||
        !filtersToolpathSegmentsByOperation() ||
        !preservesPreviewIdentityAcrossOverlapClipping() ||
        !appendsPocketLayersAndOwnsHoleStageHighlight())
        return 1;
    std::cout << "overlay controller smoke tests passed\n";
    return 0;
}
