// 六面装夹工件坐标系的构建实现（oap_toolpath_core/setup）。
// 输入是模具的世界包围盒与模具坐标系，输出六个确定性、右手系、
// 面内坐标非负的工件坐标系及配套的人工确认元数据。
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>

#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath {
namespace {

struct LocalBounds {
    std::array<double, 3> minimum{};
    std::array<double, 3> maximum{};
    bool valid = false;
};

double coordinate(const gp_Pnt& point, int axis)
{
    if (axis == 0) return point.X();
    if (axis == 1) return point.Y();
    return point.Z();
}

// 把世界 AABB 的 8 个角点变换到模具局部坐标后求各轴极值，得到
// 模具局部坐标下的毛坯包围盒；任一角点坐标非有限则整体无效。
LocalBounds localBounds(const Bbox& bounds,
                        const MoldCoordinateFrame& frame)
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

int faceIndex(MoldSetupFace face)
{
    const int index = static_cast<int>(face);
    return index >= 0 && index < 6 ? index : -1;
}

WorkCoordinateSystem workOffsetForIndex(int index)
{
    return static_cast<WorkCoordinateSystem>(
        static_cast<int>(WorkCoordinateSystem::G54) + index);
}

const char* workOffsetCode(WorkCoordinateSystem offset)
{
    return workCoordinateCode(offset);
}

gp_Trsf worldToSetupTransform(const gp_Ax3& frame)
{
    const gp_Dir x = frame.XDirection();
    const gp_Dir y = frame.YDirection();
    const gp_Dir z = frame.Direction();
    const gp_Pnt origin = frame.Location();
    gp_Trsf transform;
    transform.SetValues(
        x.X(), x.Y(), x.Z(),
        -(x.X() * origin.X() + x.Y() * origin.Y() + x.Z() * origin.Z()),
        y.X(), y.Y(), y.Z(),
        -(y.X() * origin.X() + y.Y() * origin.Y() + y.Z() * origin.Z()),
        z.X(), z.Y(), z.Z(),
        -(z.X() * origin.X() + z.Y() * origin.Y() + z.Z() * origin.Z()));
    return transform;
}

gp_Pnt localPoint(const LocalBounds& bounds,
                  bool maximumX, bool maximumY, bool maximumZ)
{
    return gp_Pnt(
        maximumX ? bounds.maximum[0] : bounds.minimum[0],
        maximumY ? bounds.maximum[1] : bounds.minimum[1],
        maximumZ ? bounds.maximum[2] : bounds.minimum[2]);
}

gp_Dir direction(const MoldCoordinateFrame& frame,
                 int axis, double sign)
{
    const gp_Dir base = axis == 0 ? frame.xAxis
        : axis == 1 ? frame.yAxis : frame.zAxis;
    return gp_Dir(gp_Vec(base).Multiplied(sign));
}

// 依据局部毛坯包围盒构建单个装夹面。definitions 表声明每个面取
// 包围盒的哪一端作为原点、面内 X 轴与法向如何从模具坐标轴映射；
// 非 PositiveZ 面必须物理翻面重夹，指令文本随之给出。
MoldFaceSetupWorkFrame makeFace(
    MoldSetupFace face,
    const MoldCoordinateFrame& frame,
    const LocalBounds& bounds)
{
    // 每个定义：originMax* 选择原点为包围盒最大端还是最小端；xAxis/xSign
    // 与 zAxis/zSign 把模具坐标轴映射为面内 X 与面法向（始终右手系）。
    struct Definition {
        MoldSetupFace face;
        bool originMaxX;
        bool originMaxY;
        bool originMaxZ;
        int xAxis;
        double xSign;
        int zAxis;
        double zSign;
        const char* mapping;
    };
    static constexpr std::array<Definition, 6> definitions{{
        {MoldSetupFace::PositiveZ, false, false, true,
         0, 1.0, 2, 1.0, "X=+moldX, Y=+moldY, Z=+moldZ"},
        {MoldSetupFace::PositiveX, true, false, false,
         1, 1.0, 0, 1.0, "X=+moldY, Y=+moldZ, Z=+moldX"},
        {MoldSetupFace::PositiveY, true, true, false,
         0, -1.0, 1, 1.0, "X=-moldX, Y=+moldZ, Z=+moldY"},
        {MoldSetupFace::NegativeX, false, true, false,
         1, -1.0, 0, -1.0, "X=-moldY, Y=+moldZ, Z=-moldX"},
        {MoldSetupFace::NegativeY, false, false, false,
         0, 1.0, 1, -1.0, "X=+moldX, Y=+moldZ, Z=-moldY"},
        {MoldSetupFace::NegativeZ, false, true, false,
         0, 1.0, 2, -1.0, "X=+moldX, Y=-moldY, Z=-moldZ"},
    }};

    MoldFaceSetupWorkFrame result;
    const auto found = std::find_if(
        definitions.begin(), definitions.end(),
        [face](const Definition& value) { return value.face == face; });
    const int index = faceIndex(face);
    if (found == definitions.end() || index < 0 || !bounds.valid)
        return result;

    result.face = face;
    const gp_Pnt origin = frame.localToWorld(localPoint(
        bounds, found->originMaxX, found->originMaxY,
        found->originMaxZ));
    result.setupWorkFrame = gp_Ax3(
        origin,
        direction(frame, found->zAxis, found->zSign),
        direction(frame, found->xAxis, found->xSign));
    result.worldToSetupTransform =
        worldToSetupTransform(result.setupWorkFrame);
    result.workOffset = workOffsetForIndex(index);
    result.valid = true;
    result.confirmed = false;
    result.requiresPhysicalReclamp =
        face != MoldSetupFace::PositiveZ;
    result.axisMapping = found->mapping;

    std::ostringstream instruction;
    instruction << "Set " << workOffsetCode(result.workOffset)
                << " origin to the displayed setup-frame corner and align machine axes as "
                << result.axisMapping;
    if (result.requiresPhysicalReclamp) {
        instruction << "; physically reclamp/rotate the mold so setup +Z points toward the spindle"
                    << "; no rotary-axis or G68 command is emitted";
    }
    result.operatorInstruction = instruction.str();
    return result;
}

} // namespace

MachiningSetup MoldFaceSetupWorkFrame::machiningSetup() const
{
    MachiningSetup result;
    result.workFrame = setupWorkFrame;
    result.workOffset = workOffset;
    result.useMoldFaceWorkFrame = true;
    result.setupFace = face;
    result.requiresManualConfirmation = true;
    result.workFrameConfirmed = confirmed;
    result.requiresPhysicalReclamp = requiresPhysicalReclamp;
    result.hasWorldToWorkTransform = true;
    result.worldToWorkTransform = worldToSetupTransform;
    result.axisMapping = axisMapping;
    result.operatorInstruction = operatorInstruction;
    return result;
}

MoldSetupWorkFrames MoldSetupWorkFrames::fromMold(
    const Bbox& worldBounds,
    const MoldCoordinateFrame& moldFrame)
{
    MoldSetupWorkFrames result;
    const LocalBounds bounds = localBounds(worldBounds, moldFrame);
    if (!bounds.valid)
        return result;
    for (int index = 0; index < 6; ++index) {
        const MoldSetupFace face = static_cast<MoldSetupFace>(index);
        result.m_faces[static_cast<std::size_t>(index)] =
            makeFace(face, moldFrame, bounds);
    }
    result.m_valid = std::all_of(
        result.m_faces.begin(), result.m_faces.end(),
        [](const MoldFaceSetupWorkFrame& item) { return item.valid; });
    return result;
}

const MoldFaceSetupWorkFrame* MoldSetupWorkFrames::forFace(
    MoldSetupFace face) const
{
    const int index = faceIndex(face);
    return index >= 0
        ? &m_faces[static_cast<std::size_t>(index)]
        : nullptr;
}

MoldFaceSetupWorkFrame* MoldSetupWorkFrames::forFace(MoldSetupFace face)
{
    const int index = faceIndex(face);
    return index >= 0
        ? &m_faces[static_cast<std::size_t>(index)]
        : nullptr;
}

bool MoldSetupWorkFrames::setConfirmed(
    MoldSetupFace face, bool confirmed)
{
    MoldFaceSetupWorkFrame* item = forFace(face);
    if (!item || !item->valid)
        return false;
    item->confirmed = confirmed;
    return true;
}

} // namespace oap::toolpath
