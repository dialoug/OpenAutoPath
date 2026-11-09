#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <utility>
#include <QOpenGLVersionFunctionsFactory>
#include <QSurfaceFormat>

namespace oap::app::pocket_cam_detail {

bool PocketCamOpenGLWidget::initializeGpuDexelPrograms()
{
    const QSurfaceFormat actualFormat = context()->format();
    if (actualFormat.majorVersion() < 4 ||
        (actualFormat.majorVersion() == 4 &&
         actualFormat.minorVersion() < 3)) {
        return false;
    }
    m_gl43 = QOpenGLVersionFunctionsFactory::get<
        QOpenGLFunctions_4_3_Core>(context());
    if (!m_gl43 || !m_gl43->initializeOpenGLFunctions())
        return false;
    GLint vertexStorage = 0, computeStorage = 0, storageBindings = 0;
    m_gl43->glGetIntegerv(GL_MAX_VERTEX_SHADER_STORAGE_BLOCKS, &vertexStorage);
    m_gl43->glGetIntegerv(GL_MAX_COMPUTE_SHADER_STORAGE_BLOCKS, &computeStorage);
    m_gl43->glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &storageBindings);
    if (vertexStorage < 6 || computeStorage < 6 || storageBindings < 8)
        return false;

    const char* computeShader = R"(
            #version 430 core
            layout(local_size_x=16, local_size_y=16) in;
            layout(std430, binding=0) buffer Intervals {
                vec2 intervalValues[];
            };
            layout(std430, binding=1) buffer Counts {
                uint intervalCounts[];
            };
            // 每个 Dexel 柱只保存边界附近的截断平面距离。负值表示位于
            // 已扫掠刀具体内，正值表示材料侧；Z 区间仍是材料真值。
            layout(std430, binding=5) buffer BoundaryDistance {
                float boundaryDistance[];
            };
            // 初始 IPW 的材料域。已经被切空的柱仍需继续累计解析距离，
            // 但毛坯原本不存在的柱不能被刀路污染。
            layout(std430, binding=6) readonly buffer StockDomain {
                uint stockDomain[];
            };
            uniform ivec2 uGridSize;
            uniform ivec2 uDispatchBase;
            uniform ivec2 uDispatchSize;
            uniform float uResolution;
            uniform vec2 uGridMinimum;
            uniform int uMaximumIntervals;
            uniform vec3 uMoveFrom;
            uniform vec3 uMoveTo;
            uniform int uProfileCount;
            uniform vec2 uProfile[32];
            uniform float uMaximumRadius;
            uniform float uNarrowBand;
            // 0/1 为普通线段，2 为固定轴恒深解析圆弧子段。
            uniform int uSweepKind;
            uniform vec2 uArcCenter;
            uniform float uArcSweepRadians;
            // 相对于 Dexel UV 右手平面的旋转方向：CCW=+1，CW=-1。
            uniform float uArcOrientation;

            float profileBottom(float radius) {
                float best = 3.402823e38;
                for (int index = 0; index < uProfileCount; ++index) {
                    if (uProfile[index].x + 1.0e-5 >= radius)
                        best = min(best, uProfile[index].y);
                }
                for (int index = 1; index < uProfileCount; ++index) {
                    vec2 first = uProfile[index - 1];
                    vec2 second = uProfile[index];
                    float low = min(first.x, second.x);
                    float high = max(first.x, second.x);
                    if (radius < low - 1.0e-5 ||
                        radius > high + 1.0e-5 ||
                        abs(second.x - first.x) <= 1.0e-7) {
                        continue;
                    }
                    float fraction = clamp(
                        (radius - first.x) /
                            (second.x - first.x),
                        0.0, 1.0);
                    best = min(
                        best, mix(first.y, second.y, fraction));
                }
                return best;
            }

            float candidateBottom(vec2 cell, float fraction) {
                vec3 tip = mix(uMoveFrom, uMoveTo, fraction);
                float radius = distance(cell, tip.xy);
                if (radius > uMaximumRadius + 1.0e-5)
                    return 3.402823e38;
                float offset = profileBottom(radius);
                return offset >= 3.0e38
                    ? 3.402823e38
                    : tip.z + offset;
            }

            vec2 rotate2d(vec2 value, float angle) {
                float c = cos(angle);
                float s = sin(angle);
                return vec2(
                    c * value.x - s * value.y,
                    s * value.x + c * value.y);
            }

            vec2 closestPathPoint(
                vec2 cell, out float pathFraction) {
                if (uSweepKind != 2 ||
                    uArcSweepRadians <= 1.0e-7) {
                    vec2 planar = uMoveTo.xy - uMoveFrom.xy;
                    float lengthSquared = dot(planar, planar);
                    pathFraction = lengthSquared <= 1.0e-12
                        ? 0.0
                        : clamp(
                            dot(cell - uMoveFrom.xy, planar) /
                                lengthSquared,
                            0.0, 1.0);
                    return mix(
                        uMoveFrom.xy, uMoveTo.xy, pathFraction);
                }
                vec2 start = uMoveFrom.xy - uArcCenter;
                vec2 relative = cell - uArcCenter;
                float startLength = length(start);
                float relativeLength = length(relative);
                if (startLength <= 1.0e-7 ||
                    relativeLength <= 1.0e-7) {
                    pathFraction = 0.0;
                    return uMoveFrom.xy;
                }
                float signedAngle = atan(
                    start.x * relative.y -
                        start.y * relative.x,
                    dot(start, relative));
                float directedAngle =
                    uArcOrientation * signedAngle;
                const float twoPi = 6.283185307179586;
                if (directedAngle < 0.0)
                    directedAngle += twoPi;
                if (directedAngle <=
                        uArcSweepRadians + 1.0e-6 ||
                    uArcSweepRadians >= twoPi - 1.0e-6) {
                    float angle = clamp(
                        directedAngle, 0.0,
                        uArcSweepRadians);
                    pathFraction = clamp(
                        angle / uArcSweepRadians, 0.0, 1.0);
                    return uArcCenter + rotate2d(
                        start,
                        uArcOrientation * angle);
                }
                float fromDistance = distance(
                    cell, uMoveFrom.xy);
                float toDistance = distance(
                    cell, uMoveTo.xy);
                pathFraction = fromDistance <= toDistance
                    ? 0.0 : 1.0;
                return pathFraction == 0.0
                    ? uMoveFrom.xy : uMoveTo.xy;
            }

            void main() {
                ivec2 localCell = ivec2(gl_GlobalInvocationID.xy);
                if (any(greaterThanEqual(localCell, uDispatchSize)))
                    return;
                ivec2 cellIndex = uDispatchBase + localCell;
                if (any(greaterThanEqual(cellIndex, uGridSize)))
                    return;
                vec2 cellPosition = uGridMinimum +
                    (vec2(cellIndex) + vec2(0.5)) * uResolution;
                float closest = 0.0;
                vec2 closestPoint = closestPathPoint(
                    cellPosition, closest);
                float bottom = 3.402823e38;
                if (uSweepKind == 2) {
                    float radius = distance(
                        cellPosition, closestPoint);
                    float offset = profileBottom(radius);
                    if (offset < 3.0e38) {
                        bottom = mix(
                            uMoveFrom.z, uMoveTo.z,
                            closest) + offset;
                    }
                } else {
                    bottom = min(
                        candidateBottom(cellPosition, 0.0),
                        candidateBottom(cellPosition, 1.0));
                    bottom = min(
                        bottom,
                        candidateBottom(cellPosition, closest));
                    bottom = min(
                        bottom,
                        candidateBottom(cellPosition, 0.25));
                    bottom = min(
                        bottom,
                        candidateBottom(cellPosition, 0.50));
                    bottom = min(
                        bottom,
                        candidateBottom(cellPosition, 0.75));
                }
                uint cellNumber = uint(
                    cellIndex.y * uGridSize.x + cellIndex.x);
                if (stockDomain[cellNumber] == 0u)
                    return;
                uint count = min(
                    intervalCounts[cellNumber],
                    uint(uMaximumIntervals));
                float sweptDistance =
                    distance(cellPosition, closestPoint) - uMaximumRadius;
                boundaryDistance[cellNumber] = clamp(
                    min(boundaryDistance[cellNumber], sweptDistance),
                    -uNarrowBand, uNarrowBand);
                if (count == 0u || bottom >= 3.0e38)
                    return;

                uint writeIndex = 0u;
                uint base = cellNumber * uint(uMaximumIntervals);
                for (uint readIndex = 0u; readIndex < count; ++readIndex) {
                    vec2 interval = intervalValues[base + readIndex];
                    if (interval.y <= bottom + 1.0e-5) {
                        intervalValues[base + writeIndex++] = interval;
                    } else if (interval.x < bottom - 1.0e-5) {
                        interval.y = bottom;
                        intervalValues[base + writeIndex++] = interval;
                    }
                }
                intervalCounts[cellNumber] = writeIndex;
            })";
    const char* surfaceShader = R"(
            #version 430 core
            layout(local_size_x=8, local_size_y=8, local_size_z=1) in;
            layout(std430, binding=0) readonly buffer Intervals {
                vec2 intervalValues[];
            };
            layout(std430, binding=1) readonly buffer Counts {
                uint intervalCounts[];
            };
            layout(std430, binding=2) writeonly buffer CoarseMask {
                uint coarseMask[];
            };
            layout(std430, binding=3) writeonly buffer FineEntries {
                uint fineEntries[];
            };
            layout(std430, binding=4) buffer FineDrawCommand {
                uint vertexCount;
                uint instanceCount;
                uint firstVertex;
                uint baseInstance;
                uint reservationCount;
                uint overflow;
                uint coarseVertexCount;
                uint coarseInstanceCount;
                uint coarseFirstVertex;
                uint coarseBaseInstance;
            };
            layout(std430, binding=7) writeonly buffer CoarseEntries {
                uint coarseEntries[];
            };
            uniform ivec2 uGridSize;
            uniform ivec2 uRenderGridSize;
            uniform int uRenderStride;
            uniform int uMaximumIntervals;
            uniform uint uFineCapacity;

            bool intervalAt(ivec2 coordinate, int slot, out vec2 value) {
                if (any(lessThan(coordinate, ivec2(0))) ||
                    any(greaterThanEqual(coordinate, uGridSize))) {
                    value = vec2(0.0);
                    return false;
                }
                int cell = coordinate.y * uGridSize.x + coordinate.x;
                uint count = min(
                    intervalCounts[cell], uint(uMaximumIntervals));
                if (slot >= int(count)) {
                    value = vec2(0.0);
                    return false;
                }
                value = intervalValues[cell * uMaximumIntervals + slot];
                return value.y > value.x + 1.0e-5;
            }

            bool sameInterval(bool valid, vec2 value,
                              bool referenceValid, vec2 reference) {
                if (valid != referenceValid)
                    return false;
                return !valid || all(lessThanEqual(
                    abs(value - reference), vec2(1.0e-5)));
            }

            void appendFine(ivec2 coordinate, int slot) {
                vec2 value;
                if (!intervalAt(coordinate, slot, value))
                    return;
                uint reservation = atomicAdd(reservationCount, 1u);
                if (reservation < uFineCapacity) {
                    uint cell = uint(
                        coordinate.y * uGridSize.x + coordinate.x);
                    fineEntries[reservation] =
                        cell * uint(uMaximumIntervals) + uint(slot);
                    atomicAdd(instanceCount, 1u);
                } else {
                    atomicExchange(overflow, 1u);
                }
            }

            void main() {
                ivec2 renderCell = ivec2(gl_GlobalInvocationID.xy);
                int slot = int(gl_GlobalInvocationID.z);
                if (any(greaterThanEqual(renderCell, uRenderGridSize)) ||
                    slot >= uMaximumIntervals) {
                    return;
                }
                ivec2 first = renderCell * uRenderStride;
                ivec2 last = min(
                    first + ivec2(uRenderStride), uGridSize);
                vec2 reference;
                bool referenceValid = intervalAt(first, slot, reference);
                bool homogeneous = referenceValid;
                for (int y = first.y; homogeneous && y < last.y; ++y) {
                    for (int x = first.x; x < last.x; ++x) {
                        vec2 value;
                        bool valid = intervalAt(ivec2(x, y), slot, value);
                        if (!sameInterval(
                                valid, value,
                                referenceValid, reference)) {
                            homogeneous = false;
                            break;
                        }
                    }
                }
                // 边界相邻粗块必须展开到原始 Dexel 分辨率，
                // 否则粗块依然会在转角或高度台阶处外伸。
                for (int x = first.x; homogeneous && x < last.x; ++x) {
                    vec2 value;
                    bool low = intervalAt(
                        ivec2(x, first.y - 1), slot, value);
                    homogeneous = sameInterval(
                        low, value, referenceValid, reference);
                    bool high = intervalAt(
                        ivec2(x, last.y), slot, value);
                    homogeneous = homogeneous && sameInterval(
                        high, value, referenceValid, reference);
                }
                for (int y = first.y; homogeneous && y < last.y; ++y) {
                    vec2 value;
                    bool low = intervalAt(
                        ivec2(first.x - 1, y), slot, value);
                    homogeneous = sameInterval(
                        low, value, referenceValid, reference);
                    bool high = intervalAt(
                        ivec2(last.x, y), slot, value);
                    homogeneous = homogeneous && sameInterval(
                        high, value, referenceValid, reference);
                }

                uint coarseEntry = uint(
                    (renderCell.y * uRenderGridSize.x + renderCell.x) *
                        uMaximumIntervals + slot);
                coarseMask[coarseEntry] = homogeneous ? 1u : 0u;
                if (homogeneous) {
                    coarseEntries[atomicAdd(coarseInstanceCount, 1u)] = coarseEntry;
                    return;
                }
                for (int y = first.y; y < last.y; ++y) {
                    for (int x = first.x; x < last.x; ++x)
                        appendFine(ivec2(x, y), slot);
                }
            })";
    const char* renderVertex = R"(
            #version 430 core
            layout(std430, binding=7) readonly buffer CoarseEntries {
                uint coarseEntries[];
            };
            layout(std430, binding=0) readonly buffer Intervals {
                vec2 intervalValues[];
            };
            layout(std430, binding=1) readonly buffer Counts {
                uint intervalCounts[];
            };
            layout(std430, binding=2) readonly buffer CoarseMask {
                uint coarseMask[];
            };
            layout(std430, binding=3) readonly buffer FineEntries {
                uint fineEntries[];
            };
            layout(std430, binding=5) readonly buffer BoundaryDistance {
                float boundaryDistance[];
            };
            out vec3 vNormal;
            uniform mat4 uModelViewProjection;
            uniform mat4 uSceneTransform;
            uniform vec3 uOrigin;
            uniform vec3 uUAxis;
            uniform vec3 uVAxis;
            uniform vec3 uWAxis;
            uniform ivec2 uGridSize;
            uniform ivec2 uRenderGridSize;
            uniform int uRenderStride;
            uniform int uMaximumIntervals;
            uniform float uResolution;
            uniform vec2 uGridMinimum;
            uniform vec2 uGridMaximum;
            uniform bool uFinePass;

            const vec2 quadVertices[6] = vec2[6](
                vec2(0.0, 0.0), vec2(1.0, 0.0),
                vec2(0.0, 1.0), vec2(0.0, 1.0),
                vec2(1.0, 0.0), vec2(1.0, 1.0));

            vec3 pointAt(float u, float v, float w) {
                return uOrigin + uUAxis * u +
                    uVAxis * v + uWAxis * w;
            }
            bool intervalAt(ivec2 coordinate, int slot, out vec2 value) {
                if (any(lessThan(coordinate, ivec2(0))) ||
                    any(greaterThanEqual(coordinate, uGridSize))) {
                    value = vec2(0.0);
                    return false;
                }
                int cell = coordinate.y * uGridSize.x + coordinate.x;
                uint count = min(
                    intervalCounts[cell], uint(uMaximumIntervals));
                if (slot >= int(count)) {
                    value = vec2(0.0);
                    return false;
                }
                value = intervalValues[cell * uMaximumIntervals + slot];
                return value.y > value.x + 1.0e-5;
            }
            float endpointAt(
                ivec2 coordinate, int slot,
                bool upper, float fallback) {
                vec2 value;
                if (!intervalAt(coordinate, slot, value))
                    return fallback;
                float candidate = upper ? value.y : value.x;
                return abs(candidate - fallback) <= 4.0 * uResolution
                    ? candidate : fallback;
            }
            vec3 surfaceNormal(
                ivec2 coordinate, int slot,
                bool upper, float center) {
                float left = endpointAt(
                    coordinate + ivec2(-1, 0), slot, upper, center);
                float right = endpointAt(
                    coordinate + ivec2(1, 0), slot, upper, center);
                float down = endpointAt(
                    coordinate + ivec2(0, -1), slot, upper, center);
                float up = endpointAt(
                    coordinate + ivec2(0, 1), slot, upper, center);
                float du = (right - left) / (2.0 * uResolution);
                float dv = (up - down) / (2.0 * uResolution);
                vec3 normal = normalize(
                    uWAxis - uUAxis * du - uVAxis * dv);
                return upper ? normal : -normal;
            }
            vec2 exposedSide(
                ivec2 neighbor, vec2 interval) {
                if (any(lessThan(neighbor, ivec2(0))) ||
                    any(greaterThanEqual(neighbor, uGridSize))) {
                    return interval;
                }
                int cell = neighbor.y * uGridSize.x + neighbor.x;
                uint count = min(
                    intervalCounts[cell], uint(uMaximumIntervals));
                float cursor = interval.x;
                vec2 best = vec2(interval.x);
                for (int slot = 0; slot < uMaximumIntervals; ++slot) {
                    if (slot >= int(count))
                        break;
                    vec2 covered = intervalValues[
                        cell * uMaximumIntervals + slot];
                    if (covered.y <= cursor + 1.0e-5)
                        continue;
                    if (covered.x >= interval.y - 1.0e-5)
                        break;
                    float gapHigh = min(covered.x, interval.y);
                    if (gapHigh - cursor > best.y - best.x)
                        best = vec2(cursor, gapHigh);
                    cursor = max(cursor, covered.y);
                    if (cursor >= interval.y - 1.0e-5)
                        break;
                }
                if (interval.y - cursor > best.y - best.x)
                    best = vec2(cursor, interval.y);
                return best;
            }

            // 在四个 Dexel 中心样本之间按 0.5 等值线重建平台边界。
            // 这里不修改区间材料，只把方格边界转换为 Marching Squares
            // 的亚单元显示轮廓；因此回放、体积和检查点语义仍由原始
            // intervalValues/intervalCounts 决定。
            bool topAtLevel(
                ivec2 coordinate, float lower, float level,
                out float top) {
                top = lower;
                if (any(lessThan(coordinate, ivec2(0))) ||
                    any(greaterThanEqual(coordinate, uGridSize))) {
                    return false;
                }
                int cell = coordinate.y * uGridSize.x + coordinate.x;
                uint count = min(
                    intervalCounts[cell], uint(uMaximumIntervals));
                float thickness = level - lower;
                float probe = thickness > 4.0e-5
                    ? level - 2.0e-5
                    : 0.5 * (lower + level);
                for (int slot = 0; slot < uMaximumIntervals; ++slot) {
                    if (slot >= int(count))
                        break;
                    vec2 value = intervalValues[
                        cell * uMaximumIntervals + slot];
                    if (value.x <= probe + 1.0e-5 &&
                        value.y >= probe - 1.0e-5) {
                        top = value.y;
                        return true;
                    }
                }
                return false;
            }
            float highestTopBelow(
                ivec2 coordinate, float lower, float level) {
                float result = lower;
                if (any(lessThan(coordinate, ivec2(0))) ||
                    any(greaterThanEqual(coordinate, uGridSize))) {
                    return result;
                }
                int cell = coordinate.y * uGridSize.x + coordinate.x;
                uint count = min(
                    intervalCounts[cell], uint(uMaximumIntervals));
                for (int slot = 0; slot < uMaximumIntervals; ++slot) {
                    if (slot >= int(count))
                        break;
                    vec2 value = intervalValues[
                        cell * uMaximumIntervals + slot];
                    if (value.y < level - 1.0e-5 &&
                        value.y > result &&
                        value.y > lower + 1.0e-5) {
                        result = value.y;
                    }
                }
                return result;
            }
            float sampleDistance(ivec2 coordinate, bool material) {
                if (any(lessThan(coordinate, ivec2(0))) ||
                    any(greaterThanEqual(coordinate, uGridSize))) {
                    return material ? uResolution : -uResolution;
                }
                int cell = coordinate.y * uGridSize.x + coordinate.x;
                float magnitude = clamp(
                    abs(boundaryDistance[cell]),
                    0.02 * uResolution, 2.0 * uResolution);
                // 区间决定符号，TSDF 只提供亚单元距离幅值。这样在恢复
                // 旧检查点（没有历史距离）时仍保持拓扑与权威区间一致。
                return material ? magnitude : -magnitude;
            }
            float edgeFraction(float first, float second) {
                float denominator = first - second;
                if (abs(denominator) <= 1.0e-7)
                    return 0.5;
                return clamp(first / denominator, 0.02, 0.98);
            }
            vec2 contourPoint(int index, vec4 field) {
                if (index == 0) return vec2(0.0, 0.0);
                if (index == 1) return vec2(1.0, 0.0);
                if (index == 2) return vec2(1.0, 1.0);
                if (index == 3) return vec2(0.0, 1.0);
                if (index == 4) return vec2(
                    edgeFraction(field.x, field.y), 0.0);
                if (index == 5) return vec2(
                    1.0, edgeFraction(field.y, field.z));
                if (index == 6) return vec2(
                    edgeFraction(field.w, field.z), 1.0);
                return vec2(
                    0.0, edgeFraction(field.x, field.w));
            }
            bool contourIndices(
                int code, int segment, out int a, out int b) {
                a = 0;
                b = 0;
                if (code == 1) { a = 7; b = 4; }
                else if (code == 2) { a = 4; b = 5; }
                else if (code == 3) { a = 7; b = 5; }
                else if (code == 4) { a = 5; b = 6; }
                else if (code == 5) {
                    if (segment == 0) { a = 7; b = 4; }
                    else { a = 5; b = 6; }
                }
                else if (code == 6) { a = 4; b = 6; }
                else if (code == 7) { a = 7; b = 6; }
                else if (code == 8) { a = 6; b = 7; }
                else if (code == 9) { a = 6; b = 4; }
                else if (code == 10) {
                    if (segment == 0) { a = 4; b = 5; }
                    else { a = 6; b = 7; }
                }
                else if (code == 11) { a = 6; b = 5; }
                else if (code == 12) { a = 5; b = 7; }
                else if (code == 13) { a = 5; b = 4; }
                else if (code == 14) { a = 4; b = 7; }
                else return false;
                if (segment > 0 && code != 5 && code != 10)
                    return false;
                return true;
            }
            bool contourSegment(
                int code, int segment, vec4 field,
                out vec2 first, out vec2 second) {
                int a;
                int b;
                if (!contourIndices(code, segment, a, b))
                    return false;
                first = contourPoint(a, field);
                second = contourPoint(b, field);
                return true;
            }
            // 双线性场的零等值线在单元内部通常不是直线。固定取参数中点
            // 并解另一个坐标，可得到严格落在同一零等值线上的细分点；与
            // 简单取弦中点相比，圆弧轮廓在放大时不会表现为连续钝角。
            vec2 contourCurvePoint(
                vec2 first, vec2 second, float fraction,
                vec4 field) {
                vec2 point = mix(first, second, fraction);
                if (abs(second.x - first.x) >=
                    abs(second.y - first.y)) {
                    float low = mix(field.x, field.y, point.x);
                    float high = mix(field.w, field.z, point.x);
                    float denominator = low - high;
                    if (abs(denominator) > 1.0e-7)
                        point.y = clamp(
                            low / denominator,
                            min(first.y, second.y),
                            max(first.y, second.y));
                } else {
                    float left = mix(field.x, field.w, point.y);
                    float right = mix(field.y, field.z, point.y);
                    float denominator = left - right;
                    if (abs(denominator) > 1.0e-7)
                        point.x = clamp(
                            left / denominator,
                            min(first.x, second.x),
                            max(first.x, second.x));
                }
                return point;
            }
            vec2 contourMidpoint(
                int code, int segment, vec4 field) {
                vec2 first;
                vec2 second;
                if (!contourSegment(
                        code, segment, field, first, second)) {
                    return vec2(0.5);
                }
                return contourCurvePoint(
                    first, second, 0.5, field);
            }
            int curvedRegionVertexCount(int code) {
                if (code == 0) return 0;
                if (code == 1 || code == 2 ||
                    code == 4 || code == 8 || code == 15) return 4;
                if (code == 3 || code == 6 || code == 9 ||
                    code == 12) return 5;
                if (code == 7 || code == 11 ||
                    code == 13 || code == 14) return 6;
                return 0;
            }
            int curvedRegionVertexIndex(int code, int index) {
                // 8 代表当前轮廓段的双线性零等值线中点。
                if (code == 1) { int v[4] = int[4](0, 4, 8, 7); return v[index]; }
                if (code == 2) { int v[4] = int[4](4, 1, 5, 8); return v[index]; }
                if (code == 3) { int v[5] = int[5](0, 1, 5, 8, 7); return v[index]; }
                if (code == 4) { int v[4] = int[4](5, 2, 6, 8); return v[index]; }
                if (code == 6) { int v[5] = int[5](4, 1, 2, 6, 8); return v[index]; }
                if (code == 7) { int v[6] = int[6](0, 1, 2, 6, 8, 7); return v[index]; }
                if (code == 8) { int v[4] = int[4](7, 8, 6, 3); return v[index]; }
                if (code == 9) { int v[5] = int[5](0, 4, 8, 6, 3); return v[index]; }
                if (code == 11) { int v[6] = int[6](0, 1, 5, 8, 6, 3); return v[index]; }
                if (code == 12) { int v[5] = int[5](7, 8, 5, 2, 3); return v[index]; }
                if (code == 13) { int v[6] = int[6](0, 4, 8, 5, 2, 3); return v[index]; }
                if (code == 14) { int v[6] = int[6](4, 1, 2, 3, 7, 8); return v[index]; }
                if (code == 15) { int v[4] = int[4](0, 1, 2, 3); return v[index]; }
                return 0;
            }
            vec2 curvedRegionPoint(
                int code, int index, vec4 field) {
                int pointIndex = curvedRegionVertexIndex(code, index);
                return pointIndex == 8
                    ? contourMidpoint(code, 0, field)
                    : contourPoint(pointIndex, field);
            }
            bool ambiguousTriangle(
                int code, int triangle, int vertex,
                vec4 field, out vec2 point) {
                if ((code != 5 && code != 10) || triangle > 3)
                    return false;
                int component = triangle / 2;
                int localTriangle = triangle % 2;
                int polygonVertex = localTriangle == 0
                    ? vertex : (vertex == 0 ? 0 : vertex + 1);
                int pointIndex = 0;
                if (code == 5 && component == 0) {
                    int v[4] = int[4](0, 4, 8, 7);
                    pointIndex = v[polygonVertex];
                } else if (code == 5) {
                    int v[4] = int[4](5, 2, 6, 8);
                    pointIndex = v[polygonVertex];
                } else if (component == 0) {
                    int v[4] = int[4](4, 1, 5, 8);
                    pointIndex = v[polygonVertex];
                } else {
                    int v[4] = int[4](7, 8, 6, 3);
                    pointIndex = v[polygonVertex];
                }
                point = pointIndex == 8
                    ? contourMidpoint(code, component, field)
                    : contourPoint(pointIndex, field);
                return true;
            }
            vec2 fieldGradient(vec4 field, vec2 point) {
                float du = mix(
                    field.y - field.x,
                    field.z - field.w, point.y);
                float dv = mix(
                    field.w - field.x,
                    field.z - field.y, point.x);
                return vec2(du, dv);
            }
            ivec2 quadrantSample(
                ivec2 current, int quadrant, int corner) {
                if (quadrant == 0) {
                    ivec2 d[4] = ivec2[4](
                        ivec2(0,0), ivec2(1,0),
                        ivec2(1,1), ivec2(0,1));
                    return current + d[corner];
                }
                if (quadrant == 1) {
                    ivec2 d[4] = ivec2[4](
                        ivec2(-1,0), ivec2(0,0),
                        ivec2(0,1), ivec2(-1,1));
                    return current + d[corner];
                }
                if (quadrant == 2) {
                    ivec2 d[4] = ivec2[4](
                        ivec2(-1,-1), ivec2(0,-1),
                        ivec2(0,0), ivec2(-1,0));
                    return current + d[corner];
                }
                ivec2 d[4] = ivec2[4](
                    ivec2(0,-1), ivec2(1,-1),
                    ivec2(1,0), ivec2(0,0));
                return current + d[corner];
            }
            void main() {
                int renderEntry = uFinePass ? gl_InstanceID : int(coarseEntries[gl_InstanceID]);
                int intervalSlot = 0;
                int i = 0;
                int j = 0;
                int cellStride = uRenderStride;
                if (uFinePass) {
                    int encoded = int(fineEntries[renderEntry]);
                    intervalSlot = encoded % uMaximumIntervals;
                    int cell = encoded / uMaximumIntervals;
                    i = cell % uGridSize.x;
                    j = cell / uGridSize.x;
                    cellStride = 1;
                } else {
                    if (coarseMask[renderEntry] == 0u) {
                        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                        return;
                    }
                    intervalSlot = renderEntry % uMaximumIntervals;
                    int renderCell = renderEntry / uMaximumIntervals;
                    int renderI = renderCell % uRenderGridSize.x;
                    int renderJ = renderCell / uRenderGridSize.x;
                    i = renderI * uRenderStride;
                    j = renderJ * uRenderStride;
                }
                if (i >= uGridSize.x || j >= uGridSize.y) {
                    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                    return;
                }
                int cell = j * uGridSize.x + i;
                uint count = min(
                    intervalCounts[cell], uint(uMaximumIntervals));
                if (intervalSlot >= int(count)) {
                    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                    return;
                }
                vec2 interval = intervalValues[
                    cell * uMaximumIntervals + intervalSlot];
                if (interval.y <= interval.x + 1.0e-5) {
                    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                    return;
                }

                if (uFinePass) {
                    // 每个细单元保留一个底面，并在其四个相邻双网格上为当前
                    // upper 高度生成最多 4 个顶面三角形；每条零等值线按
                    // 双线性场中点拆为两段，最多生成 4 个连续侧壁四边形。
                    // 6 + 4 * (12 + 24) = 150 个顶点。
                    int fineVertex = gl_VertexID;
                    if (fineVertex < 6) {
                        vec2 q = quadVertices[fineVertex];
                        float u0 = uGridMinimum.x + float(i) * uResolution;
                        float v0 = uGridMinimum.y + float(j) * uResolution;
                        float u1 = min(uGridMaximum.x, u0 + uResolution);
                        float v1 = min(uGridMaximum.y, v0 + uResolution);
                        float u = mix(u0, u1, q.x);
                        float v = mix(v0, v1, q.y);
                        vNormal = normalize(
                            mat3(uSceneTransform) * -uWAxis);
                        gl_Position = uModelViewProjection *
                            vec4(pointAt(u, v, interval.x), 1.0);
                        return;
                    }

                    int grouped = fineVertex - 6;
                    int quadrant = grouped / 36;
                    int localVertex = grouped % 36;
                    ivec2 samples[4];
                    bool inside[4];
                    float tops[4];
                    vec4 distanceField = vec4(0.0);
                    int code = 0;
                    vec2 insideCenter = vec2(0.0);
                    float insideCount = 0.0;
                    float lowerWall = interval.x;
                    for (int corner = 0; corner < 4; ++corner) {
                        samples[corner] = quadrantSample(
                            ivec2(i, j), quadrant, corner);
                        inside[corner] = topAtLevel(
                            samples[corner], interval.x,
                            interval.y, tops[corner]);
                        distanceField[corner] = sampleDistance(
                            samples[corner], inside[corner]);
                        if (inside[corner]) {
                            code |= 1 << corner;
                            insideCenter += contourPoint(
                                corner, distanceField);
                            insideCount += 1.0;
                        } else {
                            lowerWall = max(
                                lowerWall,
                                highestTopBelow(
                                    samples[corner], interval.x,
                                    interval.y));
                        }
                    }
                    // 相邻细单元可以重复提交同一个双网格块，重合片元由
                    // 深度测试去重。不能选唯一 owner：owner 可能落在相邻
                    // homogeneous 粗块中而没有进入 fineEntries，继而在
                    // 粗/细批次交界留下 L 形裂缝或孤立飞线。
                    if (code == 0) {
                        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                        return;
                    }

                    ivec2 squareFirst = samples[0];
                    vec2 squareMinimum = uGridMinimum +
                        (vec2(squareFirst) + vec2(0.5)) * uResolution;
                    vec2 squareMaximum = squareMinimum +
                        vec2(uResolution);
                    if (localVertex < 12) {
                        int triangle = localVertex / 3;
                        int triangleVertex = localVertex % 3;
                        vec2 localPoint;
                        bool validTriangle = false;
                        if (code == 5 || code == 10) {
                            validTriangle = ambiguousTriangle(
                                code, triangle, triangleVertex,
                                distanceField, localPoint);
                        } else {
                            int count = curvedRegionVertexCount(code);
                            if (triangle + 2 < count) {
                                int polygonIndex = triangleVertex == 0
                                    ? 0 : triangle + triangleVertex;
                                localPoint = curvedRegionPoint(
                                    code, polygonIndex,
                                    distanceField);
                                validTriangle = true;
                            }
                        }
                        if (!validTriangle) {
                            gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                            return;
                        }
                        vec2 planar = mix(
                            squareMinimum, squareMaximum,
                            localPoint);
                        vNormal = normalize(
                            mat3(uSceneTransform) * uWAxis);
                        gl_Position = uModelViewProjection *
                            vec4(pointAt(
                                planar.x, planar.y, interval.y), 1.0);
                        return;
                    }

                    int wallVertex = localVertex - 12;
                    int wallQuad = wallVertex / 6;
                    int segment = wallQuad / 2;
                    int curvePart = wallQuad % 2;
                    int quadVertex = wallVertex % 6;
                    vec2 first;
                    vec2 second;
                    if (!contourSegment(
                            code, segment, distanceField,
                            first, second) ||
                        lowerWall >= interval.y - 1.0e-5) {
                        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                        return;
                    }
                    vec2 middle = contourCurvePoint(
                        first, second, 0.5, distanceField);
                    vec2 curveFirst = curvePart == 0
                        ? first : middle;
                    vec2 curveSecond = curvePart == 0
                        ? middle : second;
                    vec2 q = quadVertices[quadVertex];
                    vec2 localPoint = contourCurvePoint(
                        curveFirst, curveSecond, q.x,
                        distanceField);
                    vec2 planar = mix(
                        squareMinimum, squareMaximum,
                        localPoint);
                    vec2 materialCenter = insideCenter /
                        max(insideCount, 1.0);
                    // 场值在材料侧为正，因此负梯度指向材料外部。它同时
                    // 是当前零交点的 Hermite 法线。必须在侧壁的每个顶点
                    // 重新求值，不能整段复用中心法线；否则每个 Dexel 段
                    // 都会形成一个独立的平面光照带，圆弧视觉上呈连续钝角。
                    // q.x 在同一竖边的上下顶点相同，因此侧壁仍保持竖直，
                    // 只让沿轮廓方向的法线连续插值。退化时再使用几何方向。
                    vec2 outward = -fieldGradient(
                        distanceField, localPoint);
                    if (dot(outward, outward) <= 1.0e-8) {
                        outward = localPoint - materialCenter;
                        if (dot(outward, outward) <= 1.0e-8) {
                            vec2 tangent = second - first;
                            outward = vec2(tangent.y, -tangent.x);
                        }
                    }
                    outward = normalize(outward);
                    vec3 normal = normalize(
                        uUAxis * outward.x +
                        uVAxis * outward.y);
                    float w = mix(lowerWall, interval.y, q.y);
                    vNormal = normalize(
                        mat3(uSceneTransform) * normal);
                    gl_Position = uModelViewProjection *
                        vec4(pointAt(planar.x, planar.y, w), 1.0);
                    return;
                }
                float u0 = uGridMinimum.x + float(i) * uResolution;
                float v0 = uGridMinimum.y + float(j) * uResolution;
                float u1 = min(uGridMaximum.x, uGridMinimum.x +
                    float(min(i + cellStride, uGridSize.x)) *
                    uResolution);
                float v1 = min(uGridMaximum.y, uGridMinimum.y +
                    float(min(j + cellStride, uGridSize.y)) *
                    uResolution);

                int face = gl_VertexID / 6;
                vec2 q = quadVertices[gl_VertexID % 6];
                float u = mix(u0, u1, q.x);
                float v = mix(v0, v1, q.y);
                float w = interval.y;
                vec3 normal = surfaceNormal(
                    ivec2(i, j), intervalSlot, true, interval.y);
                if (face == 1) {
                    w = interval.x;
                    normal = surfaceNormal(
                        ivec2(i, j), intervalSlot, false, interval.x);
                } else if (face == 2 || face == 3) {
                    if (!uFinePass) {
                        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                        return;
                    }
                    ivec2 neighbor = ivec2(
                        i, face == 2 ? j - 1 : j + 1);
                    vec2 side = exposedSide(neighbor, interval);
                    if (side.y <= side.x + 1.0e-5) {
                        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                        return;
                    }
                    v = face == 2 ? v0 : v1;
                    w = mix(side.x, side.y, q.y);
                    normal = face == 2 ? -uVAxis : uVAxis;
                } else if (face == 4 || face == 5) {
                    if (!uFinePass) {
                        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                        return;
                    }
                    ivec2 neighbor = ivec2(
                        face == 4 ? i - 1 : i + 1, j);
                    vec2 side = exposedSide(neighbor, interval);
                    if (side.y <= side.x + 1.0e-5) {
                        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                        return;
                    }
                    u = face == 4 ? u0 : u1;
                    v = mix(v0, v1, q.x);
                    w = mix(side.x, side.y, q.y);
                    normal = face == 4 ? -uUAxis : uUAxis;
                }
                vNormal = normalize(mat3(uSceneTransform) * normal);
                gl_Position = uModelViewProjection *
                    vec4(pointAt(u, v, w), 1.0);
            })";
    const char* renderFragment = R"(
            #version 430 core
            in vec3 vNormal;
            uniform vec4 uColor;
            out vec4 fragColor;
            void main() {
                vec3 normal = normalize(vNormal);
                vec3 light = normalize(vec3(0.35, 0.45, 1.0));
                float diffuse = max(dot(normal, light), 0.0);
                float shade = 0.30 + 0.70 * diffuse;
                fragColor = vec4(uColor.rgb * shade, uColor.a);
            })";

    const bool computeReady =
        m_dexelComputeProgram.addShaderFromSourceCode(
            QOpenGLShader::Compute, computeShader) &&
        m_dexelComputeProgram.link();
    const bool surfaceReady =
        m_dexelSurfaceProgram.addShaderFromSourceCode(
            QOpenGLShader::Compute, surfaceShader) &&
        m_dexelSurfaceProgram.link();
    const bool renderReady =
        m_dexelRenderProgram.addShaderFromSourceCode(
            QOpenGLShader::Vertex, renderVertex) &&
        m_dexelRenderProgram.addShaderFromSourceCode(
            QOpenGLShader::Fragment, renderFragment) &&
        m_dexelRenderProgram.link();
    m_gpuDexelSupported =
        computeReady && surfaceReady && renderReady;
    if (!m_gpuDexelSupported) {
        QStringList failures;
        if (!computeReady) {
            failures.push_back(
                QStringLiteral("减料 Compute：%1")
                    .arg(m_dexelComputeProgram.log()));
        }
        if (!surfaceReady) {
            failures.push_back(
                QStringLiteral("表面分类 Compute：%1")
                    .arg(m_dexelSurfaceProgram.log()));
        }
        if (!renderReady) {
            failures.push_back(
                QStringLiteral("光顺表面 Render：%1")
                    .arg(m_dexelRenderProgram.log()));
        }
        m_error = QStringLiteral(
            "GPU Multi-Dexel 着色器初始化失败：\n%1")
                      .arg(failures.join(QStringLiteral("\n")));
    }
    return m_gpuDexelSupported;
}

} // namespace oap::app::pocket_cam_detail
