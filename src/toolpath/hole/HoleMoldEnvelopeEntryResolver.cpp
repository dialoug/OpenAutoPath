// 将被槽腔去料截短的孔入口恢复到理想模具六面包络，保持孔底和阶段关系不变。
// 属于 oap_toolpath_hole 模块内部实现：先按装夹面把孔段归组成物理孔，再取
// 每个物理孔最外侧段沿孔轴投影到对应模具外包络面。
#include "toolpath/hole/HoleMoldEnvelopeEntryResolver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

#include <Precision.hxx>
#include <gp_Vec.hxx>

#include "analysis/MoldSurfaceClassifier.h"
#include "toolpath/hole/HolePhysicalGrouping.h"

namespace oap::toolpath {
namespace {

int setupFaceIndex(MoldSetupFace face)
{
    const int index = static_cast<int>(face);
    return index >= 0 && index < 6 ? index : -1;
}

ToolpathDiagnostic projectionDiagnostic(std::size_t holeIndex,
                                        MoldSetupFace face)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = ToolpathDiagnosticCode::HoleMoldEnvelopeProjectionFailed;
    diagnostic.severity = ToolpathDiagnosticSeverity::Warning;
    diagnostic.sourceIndex = static_cast<int>(holeIndex);
    std::ostringstream message;
    message << "hole " << (holeIndex + 1)
            << " could not project its physical opening to mold face "
            << moldSetupFaceName(face);
    diagnostic.message = message.str();
    return diagnostic;
}

} // namespace

HoleMoldEnvelopeEntryResult HoleMoldEnvelopeEntryResolver::resolve(
    const std::vector<HoleMachiningInput>& holes,
    const Bbox& worldBounds,
    const MoldCoordinateFrame& frame,
    double linearTolerance,
    double axisToleranceDeg) const
{
    HoleMoldEnvelopeEntryResult result;
    result.holes = holes;
    if (!worldBounds.valid() || !frame.isValid)
        return result;

    const double tolerance = std::max(
        {1.0e-6, std::abs(linearTolerance), 10.0 * Precision::Confusion()});
    // 先把每个装夹面上的孔段收集起来，随后按面分别做物理孔归组。
    std::array<std::vector<HolePhysicalSegmentInput>, 6> setupSegments;
    for (std::size_t holeIndex = 0; holeIndex < holes.size(); ++holeIndex) {
        const HoleMachiningInput& hole = holes[holeIndex];
        const int faceIndex = setupFaceIndex(hole.setupFace);
        if (faceIndex < 0 || hole.entryResolution == HoleEntryResolution::Unresolved)
            continue;
        gp_Vec inward(hole.entry, hole.bottom);
        if (inward.Magnitude() <= tolerance)
            continue;
        inward.Normalize();
        setupSegments[static_cast<std::size_t>(faceIndex)].push_back(
            HolePhysicalSegmentInput{
                holeIndex, holeIndex, hole.entry, hole.bottom,
                inward, hole.radius});
    }

    std::size_t physicalGroupIndex = 0;
    // 对每个物理孔，用其最外侧（锚点）段的入口沿孔轴向外投影到外包络面；
    // 投影长度过短视为入口本就在包络面上，只锁定方向不修改入口。
    for (const auto& segments : setupSegments) {
        const std::vector<HolePhysicalGroup> groups =
            groupPhysicalHoleSegments(segments, tolerance);
        result.physicalGroupCount += static_cast<int>(groups.size());
        for (const HolePhysicalGroup& group : groups) {
            if (group.anchorSegmentIndex >= segments.size()) {
                ++physicalGroupIndex;
                continue;
            }
            const HolePhysicalSegmentInput& anchor =
                segments[group.anchorSegmentIndex];
            if (anchor.localIndex >= result.holes.size()) {
                ++physicalGroupIndex;
                continue;
            }

            HoleMachiningInput& hole = result.holes[anchor.localIndex];
            gp_Vec outward(hole.bottom, hole.entry);
            gp_Pnt projected;
            double extension = 0.0;
            if (!oap::projectPointToMoldSurface(
                    worldBounds, frame, hole.setupFace, hole.entry, outward,
                    projected, extension, axisToleranceDeg)) {
                result.diagnostics.push_back(
                    projectionDiagnostic(anchor.localIndex, hole.setupFace));
                ++physicalGroupIndex;
                continue;
            }
            if (extension <= tolerance) {
                hole.preserveEntrySide = true;
                ++physicalGroupIndex;
                continue;
            }

            const gp_Pnt originalEntry = hole.entry;
            hole.entry = projected;
            hole.entryResolution = HoleEntryResolution::MoldEnvelopeProjected;
            hole.preserveEntrySide = true;
            result.adjustments.push_back(HoleMoldEnvelopeEntryAdjustment{
                anchor.localIndex, physicalGroupIndex, hole.setupFace,
                originalEntry, projected, extension});
            ++physicalGroupIndex;
        }
    }
    return result;
}

} // namespace oap::toolpath
