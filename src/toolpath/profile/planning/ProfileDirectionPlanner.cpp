#include "toolpath/profile/planning/ProfileDirectionPlanner.h"

#include <algorithm>
#include <cmath>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopTools_ListOfShape.hxx>

namespace oap::toolpath {
namespace {

class PlanningProgress final : public Message_ProgressIndicator {
public:
    explicit PlanningProgress(const std::function<bool()>& callback) : m_callback(callback) {}
protected:
    Standard_Boolean UserBreak() override
    {
        try { return m_callback && m_callback(); }
        catch (...) { return Standard_True; }
    }
    void Show(const Message_ProgressScope&, Standard_Boolean) override {}
private:
    std::function<bool()> m_callback;
};

double area(const TopoDS_Shape& shape)
{
    GProp_GProps properties;
    BRepGProp::SurfaceProperties(shape, properties);
    return std::abs(properties.Mass());
}

double volume(const TopoDS_Shape& shape)
{
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return std::abs(properties.Mass());
}

bool cancelled(const ProfileDirectionPlanningOptions& options)
{
    return options.cancellationRequested && options.cancellationRequested();
}

bool difference(const TopoDS_Shape& left, const TopoDS_Shape& right,
                TopoDS_Shape& output, double tolerance,
                const ProfileDirectionPlanningOptions& options)
{
    BRepAlgoAPI_Cut cut;
    TopTools_ListOfShape arguments, tools;
    arguments.Append(left);
    tools.Append(right);
    cut.SetArguments(arguments);
    cut.SetTools(tools);
    cut.SetNonDestructive(true);
    cut.SetRunParallel(false);
    cut.SetFuzzyValue(tolerance);
    Handle(Message_ProgressIndicator) progress = new PlanningProgress(options.cancellationRequested);
    cut.Build(progress->Start());
    if (!cut.IsDone() || cut.HasErrors() || cut.HasWarnings() || cut.Shape().IsNull())
        return false;
    output = cut.Shape();
    return true;
}

TopoDS_Face footprint(const ProfileRegion& region, double offset)
{
    if (!region.boundary.closed || region.boundary.points.size() < 3)
        return {};
    BRepBuilderAPI_MakePolygon polygon;
    for (const auto& point : region.boundary.points) {
        polygon.Add(region.frame.Location().Translated(
            gp_Vec(region.frame.XDirection()) * point.X() +
            gp_Vec(region.frame.YDirection()) * point.Y() +
            gp_Vec(region.openDir) * offset));
    }
    polygon.Close();
    if (!polygon.IsDone())
        return {};
    BRepBuilderAPI_MakeFace face(polygon.Wire());
    return face.IsDone() ? face.Face() : TopoDS_Face{};
}

bool provesEnvelope(const std::vector<ProfileRegion>& regions,
                    const std::vector<std::size_t>& unit,
                    std::size_t selected,
                    const ProfileDirectionPlanningOptions& options)
{
    const auto& candidate = regions[selected];
    const auto& target = candidate.protectedMaterialShape;
    if (target.IsNull() || target.ShapeType() != TopAbs_SOLID ||
        !std::isfinite(candidate.depth) || candidate.depth <= 1.e-7 ||
        !std::isfinite(candidate.sectionTolerance) ||
        candidate.sectionTolerance < 0 || candidate.sectionTolerance > 1.e-4 ||
        !candidate.openDir.IsEqual(candidate.frame.Direction(), 1.e-8))
        return false;
    const double tolerance = std::max(1.e-7, candidate.sectionTolerance);
    const auto axis = gp_Vec(candidate.openDir);
    // An oblique setup through a world-aligned box can leave end wedges even
    // with zero explicit margin. Keep its other directions until stock-volume
    // coverage (rather than just external-surface coverage) is implemented.
    if (std::max({std::abs(axis.X()), std::abs(axis.Y()), std::abs(axis.Z())}) < 1.0 - 1.e-12 ||
        !std::isfinite(options.stockMargins.X()) || options.stockMargins.X() < 0 ||
        !std::isfinite(options.stockMargins.Y()) || options.stockMargins.Y() < 0 ||
        !std::isfinite(options.stockMargins.Z()) || options.stockMargins.Z() < 0)
        return false;
    const double capMargin = std::abs(axis.X()) * options.stockMargins.X() +
        std::abs(axis.Y()) * options.stockMargins.Y() +
        std::abs(axis.Z()) * options.stockMargins.Z();
    if (!std::isfinite(capMargin) || capMargin > tolerance || capMargin < 0)
        return false;
    if (!BRepCheck_Analyzer(target).IsValid() || cancelled(options))
        return false;
    const auto base = footprint(candidate, 0);
    if (base.IsNull())
        return false;
    BRepPrimAPI_MakePrism extrusion(base, axis * candidate.depth);
    if (!extrusion.IsDone() || !BRepCheck_Analyzer(extrusion.Shape()).IsValid())
        return false;
    const auto envelope = extrusion.Shape();
    TopoDS_Compound sides, boundary;
    BRep_Builder builder;
    builder.MakeCompound(sides);
    builder.MakeCompound(boundary);
    for (TopExp_Explorer faces(envelope, TopAbs_FACE); faces.More(); faces.Next()) {
        builder.Add(boundary, faces.Current());
        if (!faces.Current().IsSame(extrusion.FirstShape()) &&
            !faces.Current().IsSame(extrusion.LastShape()))
            builder.Add(sides, faces.Current());
    }
    const double surfaceArea = area(envelope);
    const double areaTolerance = std::max(1.e-9, surfaceArea * 1.e-10);
    const double volumeTolerance = std::max(1.e-9, surfaceArea * tolerance);
    if (area(sides) <= areaTolerance || volume(target) <= volumeTolerance)
        return false;
    TopoDS_Shape remainder;
    // The solid fits completely inside the proposed extrusion, and every
    // lateral face of that extrusion is present in the finished solid.
    // A side opening, external step or undercut therefore prevents reduction.
    if (cancelled(options) || !difference(target, envelope, remainder, tolerance, options) ||
        volume(remainder) > volumeTolerance || cancelled(options) ||
        !difference(sides, target, remainder, tolerance, options) || area(remainder) > areaTolerance)
        return false;
    // An internal root (e.g. a pocket floor) is a separate task, even when it
    // belongs to the same solid. Its filled root must lie on the envelope skin.
    for (const auto index : unit) {
        if (cancelled(options))
            return false;
        const auto root = footprint(regions[index], regions[index].depth);
        if (root.IsNull() || !difference(root, boundary, remainder, tolerance, options) ||
            area(remainder) > areaTolerance)
            return false;
    }
    return true;
}

} // namespace

ProfileDirectionPlan ProfileDirectionPlanner::plan(
    const std::vector<ProfileRegion>& regions,
    const ProfileDirectionPlanningOptions& options) const
{
    ProfileDirectionPlan result;
    std::vector<bool> assigned(regions.size(), false);
    for (std::size_t first = 0; first < regions.size(); ++first) {
        if (cancelled(options)) {
            result.cancelled = true;
            result.selectedRegionIndices.clear();
            result.coverage.clear();
            return result;
        }
        if (assigned[first]) continue;
        std::vector<std::size_t> unit;
        for (std::size_t index = first; index < regions.size(); ++index) {
            if (index == first || (!assigned[index] &&
                regions[index].source.unitId == regions[first].source.unitId &&
                !regions[first].protectedMaterialShape.IsNull() &&
                regions[index].protectedMaterialShape.IsSame(regions[first].protectedMaterialShape))) {
                unit.push_back(index);
                assigned[index] = true;
            }
        }
        auto candidates = unit;
        std::stable_sort(candidates.begin(), candidates.end(), [&](auto a, auto b) {
            const double left = std::isfinite(regions[a].depth) ? regions[a].depth : HUGE_VAL;
            const double right = std::isfinite(regions[b].depth) ? regions[b].depth : HUGE_VAL;
            return left < right;
        });
        bool reduced = false;
        if (options.allowReduction && unit.size() > 1) {
            for (const auto index : candidates) {
                if (cancelled(options)) break;
                if (options.hasCandidateTool && !options.hasCandidateTool(regions[index])) continue;
                bool proven = false;
                try {
                    proven = provesEnvelope(regions, unit, index, options);
                } catch (const Standard_Failure&) {
                    // An inconclusive Boolean never authorizes dropping tasks.
                }
                if (proven) {
                    result.selectedRegionIndices.push_back(index);
                    result.coverage.push_back({index, unit});
                    reduced = true;
                    break;
                }
            }
        }
        if (!reduced) {
            result.selectedRegionIndices.insert(result.selectedRegionIndices.end(), unit.begin(), unit.end());
            result.retainedUnitIds.push_back(regions[first].source.unitId);
        }
    }
    if (cancelled(options)) {
        result.cancelled = true;
        result.selectedRegionIndices.clear();
        result.coverage.clear();
        return result;
    }
    std::sort(result.selectedRegionIndices.begin(), result.selectedRegionIndices.end());
    result.complete = true;
    return result;
}

} // namespace oap::toolpath
