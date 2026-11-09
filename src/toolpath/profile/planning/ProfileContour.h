// 外轮廓刀路规划核心（oap_toolpath_profile）。
// 对外轮廓/区域按层生成轮廓刀路：ProfileLoop 描述单条闭合走刀环，
// ProfileContourResult 汇总环、moves/primitives 与耗时统计，
// ProfileCandidateGeometryCache 缓存规划批次几何；由内置算法
// BuiltinProfileToolpathAlgorithm 调用。
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathMoves.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathPrimitives.h"
#include "toolpath/profile/geometry/ProfileRegion.h"

namespace oap::toolpath {

enum class ProfileContourFailureCode {
    None,
    InvalidRegion,
    InvalidParameters,
    OffsetFailed,
    SafetyValidationFailed,
    Cancelled,
};

enum class ProfilePassKind {
    Clearing,
    Finish,
};

using ProfileBoundary = std::vector<gp_Pnt2d>;
// Outer material components are filled: holes are not external-profile access.
using ProfileBoundaries = std::vector<ProfileBoundary>;
using SharedProfileBoundaries = std::shared_ptr<const ProfileBoundaries>;

// Single planning-batch cache. Entries use the stable addresses of the input
// ProfileRegion objects and must not outlive mutation/replacement of them.
class ProfileCandidateGeometryCache {
public:
    ProfileCandidateGeometryCache();
    ~ProfileCandidateGeometryCache();
    ProfileCandidateGeometryCache(
        const ProfileCandidateGeometryCache&) = delete;
    ProfileCandidateGeometryCache& operator=(
        const ProfileCandidateGeometryCache&) = delete;

    std::size_t hitCount() const;
    std::size_t missCount() const;
    std::size_t entryCount() const;
    std::size_t estimatedByteCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;

    friend class ProfileContour;
};

// 单条闭合走刀环：点、世界坐标、刀路类型与边界。
struct ProfileLoop {
    std::vector<gp_Pnt2d> points;
    std::vector<gp_Pnt> world;
    bool closed = true;
    ProfilePassKind passKind = ProfilePassKind::Finish;
    // 当前深度层最终实体的全部外边界，包括与根面分离的凸台。
    // 复杂降台轮廓不能只用根面 outer wire；普通直壁沿用 region.boundary。
    SharedProfileBoundaries materialBoundaries;
    // 同深度跳刀直连使用的累计轴向保护包络。它可能大于当前层真实截面，
    // 用于避免固定三轴刀具在低位横移时碰到上方尚未退让的台阶材料。
    SharedProfileBoundaries linkSafetyBoundaries;
    int depthIndex = 0;
    int radialPassIndex = 0; // 0=最远粗加工轮廓，末次=最终轮廓
    int sequence = 0;
    double depthOffset = 0.0;
    double offset = 0.0;
    double area = 0.0;
};

// 轮廓刀路结果：环、moves/primitives 与耗时统计。
struct ProfileContourResult {
    std::vector<ProfileLoop> loops;
    std::vector<ToolpathMove> moves;
    std::vector<ToolpathPrimitive> primitives;
    bool ok = false;
    bool cancelled = false;
    std::string warning;
    std::string error;
    ProfileContourFailureCode failureCode =
        ProfileContourFailureCode::None;
    int depthLayerCount = 0;
    int radialPassCount = 0;
    int clearingPassCount = 0;
    int finishingPassCount = 0;
    bool usedExternalClearing = false;
    bool restMachining = false;
    bool noRestMaterial = false;
    bool evaluationOnly = false;
    double restMachiningPreviousToolDiameter = 0.0;
    double toolRadius = 0.0;
    double finalOffset = 0.0;
    double radialStepOver = 0.0;
    double safeHeightOffset = 0.0;
    double clearanceHeightOffset = 0.0;
    double estimatedCuttingDistance = 0.0;
    int directCutLinkCount = 0;
    int retractEntryCount = 0;
    std::size_t travelReorderedLoopCount = 0;
    std::size_t travelAdjustedStartCount = 0;
    // Generation diagnostics. Section time includes both OCCT computation and
    // shared-cache lookup/copy time; cache counters expose which path was used.
    double sectionMilliseconds = 0.0;
    double offsetMilliseconds = 0.0;
    double patternMilliseconds = 0.0;
    double travelOptimizationMilliseconds = 0.0;
    double moveAssemblyMilliseconds = 0.0;
    // Time spent promoting an evaluation-only contour into executable
    // moves/primitives. Geometry generation and travel ordering are reused.
    double materializationMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
    std::size_t materialSectionCacheHitCount = 0u;
    std::size_t materialSectionCacheMissCount = 0u;
    std::size_t candidateGeometryCacheHitCount = 0u;
    std::size_t candidateGeometryCacheMissCount = 0u;
    int parallelLayerBatchCount = 0;
    int parallelLayerCount = 0;
    std::size_t layerPeakParallelism = 1u;
};

// 外轮廓刀路规划核心：分层、偏置与行程优化。
class ProfileContour {
public:
    ProfileContourResult generate(
        const ProfileRegion& region,
        const ToolpathParams& params,
        const ToolpathGenerationControl* control = nullptr) const;

    ProfileContourResult materializeEvaluation(
        const ProfileRegion& region,
        const ToolpathParams& params,
        ProfileContourResult evaluation,
        const ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::toolpath
