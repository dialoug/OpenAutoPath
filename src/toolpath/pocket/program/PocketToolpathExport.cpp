// 槽腔工序导出适配层：构造中立程序后交由统一的事务式 G-code 导出流程。
// JSONL 路径直接序列化；G-code 路径统一委托 ToolpathProgramExport。
#include "toolpath/pocket/program/PocketToolpathExport.h"

#include "analysis/Utf8Path.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "toolpath/pocket/program/PocketProgramAdapter.h"
#include "toolpath/program/ToolpathProgramExport.h"

namespace oap::toolpath {

namespace {

const char* featureKindName(FeatureKind kind)
{
    switch (kind) {
        case FeatureKind::Slot: return "slot";
        case FeatureKind::Pocket: return "pocket";
        default: return "unknown";
    }
}

const char* moveTypeName(ToolpathMoveType type)
{
    switch (type) {
        case ToolpathMoveType::Rapid: return "rapid";
        case ToolpathMoveType::Plunge: return "plunge";
        case ToolpathMoveType::Cut: return "cut";
        case ToolpathMoveType::Retract: return "retract";
    }
    return "unknown";
}

const char* entryModeName(PocketEntryMode mode)
{
    switch (mode) {
        case PocketEntryMode::VerticalPlunge: return "vertical_plunge";
        case PocketEntryMode::RampOnShape: return "ramp_on_shape";
        case PocketEntryMode::LinearRamp: return "linear_ramp";
        case PocketEntryMode::HelicalRamp: return "helical_ramp";
        case PocketEntryMode::ZigZagRamp: return "zigzag_ramp";
    }
    return "unknown";
}

const char* rampStartReferenceName(PocketRampStartReference reference)
{
    switch (reference) {
        case PocketRampStartReference::PreviousLevel:
            return "previous_level";
        case PocketRampStartReference::OpeningPlane:
            return "opening_plane";
        case PocketRampStartReference::SafePlane:
            return "safe_plane";
    }
    return "unknown";
}

const char* expansionDirectionName(PocketClearingDirection direction)
{
    switch (direction) {
        case PocketClearingDirection::OutsideToInside:
            return "outside_to_inside";
        case PocketClearingDirection::InsideToOutside:
            return "inside_to_outside";
    }
    return "unknown";
}

const char* clearingPatternName(ClearingPattern pattern)
{
    switch (pattern) {
        case ClearingPattern::Offset: return "offset";
        case ClearingPattern::ZigZag: return "zigzag";
        case ClearingPattern::Spiral: return "spiral";
        case ClearingPattern::FreePath: return "adaptive_freepath";
    }
    return "unknown";
}

std::string jsonString(const std::string& value)
{
    std::ostringstream out;
    out << '"';
    for (char ch : value) {
        switch (ch) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\b': out << "\\b"; break;
            case '\f': out << "\\f"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20)
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(ch))
                        << std::dec << std::setfill(' ');
                else
                    out << ch;
                break;
        }
    }
    out << '"';
    return out.str();
}

void writeStringArray(std::ostream& os, const std::vector<std::string>& values)
{
    os << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            os << ',';
        os << jsonString(values[i]);
    }
    os << ']';
}

void writePointJson(std::ostream& os, const gp_Pnt& p)
{
    os << "{\"x\":" << p.X()
       << ",\"y\":" << p.Y()
       << ",\"z\":" << p.Z()
       << '}';
}

// 以二进制/截断模式打开输出文件；自动创建父目录（不报错），
// 打开失败时把可读错误写入 error 并返回 false。
bool openOutput(const std::string& outPath, std::ofstream& os, std::string& error)
{
    const std::filesystem::path path = oap::utf8Path(outPath);
    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            error = "cannot create output directory: " + ec.message();
            return false;
        }
    }

    os.open(path, std::ios::binary | std::ios::trunc);
    if (!os) {
        error = "cannot open output file: " + outPath;
        return false;
    }
    return true;
}

} // namespace

bool PocketToolpathExport::writeJsonl(
    const std::vector<PocketToolpathOperation>& operations,
    const ToolpathParams& params,
    const std::string& outPath,
    std::string& error) const
{
    std::ofstream os;
    if (!openOutput(outPath, os, error))
        return false;

    os << std::fixed << std::setprecision(6);
    for (const PocketToolpathOperation& op : operations) {
        for (const ToolpathMove& move : op.clearing.moves) {
            os << "{\"record_type\":\"pocket_toolpath_move\"";
            os << ",\"operation_id\":" << jsonString(op.operationId);
            os << ",\"algorithm_id\":" << jsonString(op.algorithmId);
            os << ",\"feature_kind\":" << jsonString(featureKindName(op.kind));
            os << ",\"floor_face_id\":" << jsonString(op.floorFaceId);
            os << ",\"source_face_ids\":";
            writeStringArray(os, op.sourceFaceIds);
            os << ",\"move_type\":" << jsonString(moveTypeName(move.type));
            os << ",\"loop_sequence\":" << move.loopSequence;
            os << ",\"depth_index\":" << move.depthIndex;
            os << ",\"ring\":" << move.ring;
            os << ",\"feed\":" << move.feed;
            os << ",\"from\":";
            writePointJson(os, move.from);
            os << ",\"to\":";
            writePointJson(os, move.to);
            os << "}\n";
        }
    }

    os << "{\"record_type\":\"pocket_toolpath_summary\"";
    os << ",\"operation_count\":" << operations.size();
    os << ",\"tool_diameter\":" << params.tool.diameter;
    os << ",\"horiz_feed\":" << params.tool.horizFeed;
    os << ",\"vert_feed\":" << params.tool.vertFeed;
    os << ",\"horiz_rapid\":" << params.tool.horizRapid;
    os << ",\"vert_rapid\":" << params.tool.vertRapid;
    os << ",\"spindle_speed\":" << params.tool.spindleSpeed;
    os << ",\"clearing_pattern\":"
       << jsonString(clearingPatternName(params.clearing.pattern));
    os << ",\"expansion_direction\":"
       << jsonString(expansionDirectionName(
              effectivePocketClearingDirection(
                  params.clearing.pattern,
                  params.clearing.expansionDirection)));
    os << ",\"entry_mode\":"
       << jsonString(entryModeName(params.clearing.entryMode));
    os << ",\"ramp_start_reference\":"
       << jsonString(rampStartReferenceName(
              params.clearing.rampStartReference));
    os << ",\"ramp_angle_deg\":" << params.clearing.rampAngleDeg;
    os << ",\"maximum_ramp_length\":"
       << params.clearing.maximumRampLength;
    os << ",\"helix_diameter\":"
       << params.clearing.helixDiameter;
    os << ",\"zigzag_ramp_width\":"
       << params.clearing.zigzagRampWidth;
    os << ",\"ramp_feed\":" << params.clearing.rampFeed;
    os << ",\"step_over_percent\":" << params.clearing.stepOverPercent;
    const AdaptiveClearingParams& adaptive = params.clearing.adaptive;
    os << ",\"adaptive_target_engagement\":"
       << adaptive.targetEngagement;
    os << ",\"adaptive_target_angle_deg\":"
       << adaptive.targetEngagementAngleDeg;
    os << ",\"adaptive_maximum_angle_deg\":"
       << adaptive.maximumEngagementAngleDeg;
    os << ",\"adaptive_angle_samples\":"
       << adaptive.engagementAngleSamples;
    os << ",\"adaptive_minimum_turn_radius_factor\":"
       << adaptive.minimumTurnRadiusFactor;
    os << ",\"adaptive_remaining_tolerance\":"
       << adaptive.remainingTolerance;
    os << ",\"adaptive_material_cell_size\":"
       << adaptive.materialCellSize;
    os << ",\"adaptive_maximum_passes\":"
       << adaptive.maximumAdaptivePasses;
    os << ",\"adaptive_lookahead_depth\":"
       << adaptive.lookaheadDepth;
    os << ",\"adaptive_beam_width\":" << adaptive.beamWidth;
    int adaptivePasses = 0;
    int adaptiveRemainingComponents = 0;
    int adaptiveOverloadSegments = 0;
    int adaptiveWavefrontLayers = 0;
    int adaptiveCentralSpirals = 0;
    int adaptiveRadialComponents = 0;
    int adaptiveRadialPasses = 0;
    double adaptiveRemainingFraction = 0.0;
    double adaptiveMaximumAngle = 0.0;
    for (const PocketToolpathOperation& operation : operations) {
        adaptivePasses += operation.clearing.adaptivePassCount;
        adaptiveRemainingComponents +=
            operation.clearing.adaptiveRemainingComponentCount;
        adaptiveOverloadSegments +=
            operation.clearing.adaptiveOverloadSegmentCount;
        adaptiveWavefrontLayers +=
            operation.clearing.adaptiveWavefrontLayerCount;
        adaptiveCentralSpirals +=
            operation.clearing.adaptiveCentralSpiralCount;
        adaptiveRadialComponents +=
            operation.clearing.adaptiveRadialComponentCount;
        adaptiveRadialPasses +=
            operation.clearing.adaptiveRadialPassCount;
        adaptiveRemainingFraction = std::max(
            adaptiveRemainingFraction,
            operation.clearing.adaptiveRemainingMaterialFraction);
        adaptiveMaximumAngle = std::max(
            adaptiveMaximumAngle,
            operation.clearing.adaptiveMaximumEngagementAngleDeg);
    }
    os << ",\"adaptive_pass_count\":" << adaptivePasses;
    os << ",\"adaptive_remaining_components\":"
       << adaptiveRemainingComponents;
    os << ",\"adaptive_overload_segments\":"
       << adaptiveOverloadSegments;
    os << ",\"adaptive_wavefront_layers\":"
       << adaptiveWavefrontLayers;
    os << ",\"adaptive_central_spirals\":"
       << adaptiveCentralSpirals;
    os << ",\"adaptive_radial_components\":"
       << adaptiveRadialComponents;
    os << ",\"adaptive_radial_passes\":"
       << adaptiveRadialPasses;
    os << ",\"adaptive_remaining_fraction\":"
       << adaptiveRemainingFraction;
    os << ",\"adaptive_maximum_observed_angle_deg\":"
       << adaptiveMaximumAngle;
    os << ",\"step_down\":" << params.depth.stepDown;
    os << ",\"finish_depth\":" << params.depth.finishDepth;
    os << ",\"safe_height\":" << params.depth.safeHeight;
    os << ",\"clearance_height\":" << params.depth.clearanceHeight;
    os << "}\n";

    return true;
}

bool PocketToolpathExport::writeGCode(
    const std::vector<PocketToolpathOperation>& operations,
    const ToolpathParams& params,
    const std::string& outPath,
    std::string& error) const
{
    MachiningSetup setup;
    GCodeProgramOptions options;
    return writeGCode(operations, params, setup, options, outPath, error);
}

bool PocketToolpathExport::writeGCode(
    const std::vector<PocketToolpathOperation>& operations,
    const ToolpathParams& params,
    const MachiningSetup& setup,
    const GCodeProgramOptions& options,
    const std::string& outPath,
    std::string& error,
    const GCodePostProcessor* postProcessor) const
{
    return ToolpathProgramExport().writeGCode(
        makeToolpathProgram(operations), params, setup, options, outPath,
        error, postProcessor);
}

} // namespace oap::toolpath
