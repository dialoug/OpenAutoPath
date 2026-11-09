// ProfileContour 实现（oap_toolpath_profile）。
// 分层、偏置、清料图案与行程优化，见 ProfileContour.h。
#include "toolpath/profile/planning/ProfileContour.h"
#include "toolpath/profile/planning/detail/ProfileMotionPlanning.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <utility>

#include <gp_Vec.hxx>

#include "clipper2/clipper.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/pocket/planning/CutterCenterClearingPattern.h"
#include "toolpath/pocket/planning/PocketLayerSlicer.h"
#include "toolpath/core/ToolpathParallel.h"

namespace oap::toolpath {

namespace profile_candidate_cache_detail {

using Clipper2Lib::PathD;
using Clipper2Lib::PathsD;

std::uint64_t doubleBits(double value) noexcept
{
    if (value == 0.0)
        value = 0.0;
    return std::bit_cast<std::uint64_t>(value);
}

void mixHash(std::size_t& seed, std::size_t value) noexcept
{
    seed ^= value + 0x9e3779b97f4a7c15ull +
        (seed << 6u) + (seed >> 2u);
}

struct LayerKey {
    const ProfileRegion* region = nullptr;
    std::uint64_t depthOffset = 0u;
    std::uint64_t roughingAllowance = 0u;
    std::uint64_t arcTolerance = 0u;
    int precision = 0;

    bool operator==(const LayerKey&) const = default;
};

struct LayerKeyHash {
    std::size_t operator()(const LayerKey& key) const noexcept
    {
        std::size_t hash = std::hash<const ProfileRegion*>{}(key.region);
        mixHash(hash, std::hash<std::uint64_t>{}(key.depthOffset));
        mixHash(hash, std::hash<std::uint64_t>{}(key.roughingAllowance));
        mixHash(hash, std::hash<std::uint64_t>{}(key.arcTolerance));
        mixHash(hash, std::hash<int>{}(key.precision));
        return hash;
    }
};

struct ToolKey {
    LayerKey layer;
    std::uint64_t cutterRadius = 0u;
    std::uint64_t finalOffset = 0u;
    bool finishPass = false;

    bool operator==(const ToolKey&) const = default;
};

struct ToolKeyHash {
    std::size_t operator()(const ToolKey& key) const noexcept
    {
        std::size_t hash = LayerKeyHash{}(key.layer);
        mixHash(hash, std::hash<std::uint64_t>{}(key.cutterRadius));
        mixHash(hash, std::hash<std::uint64_t>{}(key.finalOffset));
        mixHash(hash, std::hash<bool>{}(key.finishPass));
        return hash;
    }
};

struct RestKey {
    LayerKey layer;
    std::uint64_t currentRadius = 0u;
    std::uint64_t previousRadius = 0u;

    bool operator==(const RestKey&) const = default;
};

struct RestKeyHash {
    std::size_t operator()(const RestKey& key) const noexcept
    {
        std::size_t hash = LayerKeyHash{}(key.layer);
        mixHash(hash, std::hash<std::uint64_t>{}(key.currentRadius));
        mixHash(hash, std::hash<std::uint64_t>{}(key.previousRadius));
        return hash;
    }
};

struct LayerGeometry {
    PathsD layerBase;
    PathsD protectedMaterialBase;
    SharedProfileBoundaries linkProtectedBoundary;
    SharedProfileBoundaries protectedMaterialBoundary;
    PathsD materialArea;
    bool ok = false;
    std::string error;
};

struct ToolGeometry {
    PathsD cumulativeCenters;
    PathsD safeCenters;
    PathsD cumulativeSweep;
    PathsD finishPaths;
};

struct RestGeometry {
    PathsD centers;
};

std::size_t pathBytes(const PathD& path) noexcept
{
    return path.capacity() * sizeof(Clipper2Lib::PointD);
}

std::size_t pathsBytes(const PathsD& paths) noexcept
{
    std::size_t result = paths.capacity() * sizeof(PathD);
    for (const PathD& path : paths)
        result += pathBytes(path);
    return result;
}

template <typename Value>
struct CacheEntry {
    std::shared_ptr<const Value> value;
    std::exception_ptr exception;
    std::condition_variable readyCondition;
    bool ready = false;
};

} // namespace profile_candidate_cache_detail

struct ProfileCandidateGeometryCache::Impl {
    using LayerEntry =
        profile_candidate_cache_detail::CacheEntry<
            profile_candidate_cache_detail::LayerGeometry>;
    using ToolEntry =
        profile_candidate_cache_detail::CacheEntry<
            profile_candidate_cache_detail::ToolGeometry>;
    using RestEntry =
        profile_candidate_cache_detail::CacheEntry<
            profile_candidate_cache_detail::RestGeometry>;

    mutable std::mutex mutex;
    std::unordered_map<
        profile_candidate_cache_detail::LayerKey,
        std::shared_ptr<LayerEntry>,
        profile_candidate_cache_detail::LayerKeyHash>
        layers;
    std::unordered_map<
        profile_candidate_cache_detail::ToolKey,
        std::shared_ptr<ToolEntry>,
        profile_candidate_cache_detail::ToolKeyHash>
        tools;
    std::unordered_map<
        profile_candidate_cache_detail::RestKey,
        std::shared_ptr<RestEntry>,
        profile_candidate_cache_detail::RestKeyHash>
        rests;
    std::size_t hits = 0u;
    std::size_t misses = 0u;

    template <typename Key, typename Value, typename Map, typename Compute>
    std::shared_ptr<const Value> getOrCompute(
        Map& entries, const Key& key, Compute&& compute)
    {
        using Entry =
            profile_candidate_cache_detail::CacheEntry<Value>;
        std::shared_ptr<Entry> entry;
        {
            std::unique_lock<std::mutex> lock(mutex);
            const auto found = entries.find(key);
            if (found != entries.end()) {
                ++hits;
                entry = found->second;
                entry->readyCondition.wait(
                    lock, [&entry]() { return entry->ready; });
                if (entry->exception)
                    std::rethrow_exception(entry->exception);
                return entry->value;
            }
            ++misses;
            entry = std::make_shared<Entry>();
            entries.emplace(key, entry);
        }

        std::shared_ptr<const Value> value;
        std::exception_ptr exception;
        try {
            value = std::make_shared<const Value>(compute());
        } catch (...) {
            exception = std::current_exception();
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            entry->value = std::move(value);
            entry->exception = exception;
            entry->ready = true;
        }
        entry->readyCondition.notify_all();
        if (exception)
            std::rethrow_exception(exception);
        return entry->value;
    }

    template <typename Compute>
    std::shared_ptr<const profile_candidate_cache_detail::LayerGeometry>
    getLayer(
        const profile_candidate_cache_detail::LayerKey& key,
        Compute&& compute)
    {
        return getOrCompute<
            profile_candidate_cache_detail::LayerKey,
            profile_candidate_cache_detail::LayerGeometry>(
                layers, key, std::forward<Compute>(compute));
    }

    template <typename Compute>
    std::shared_ptr<const profile_candidate_cache_detail::ToolGeometry>
    getTool(
        const profile_candidate_cache_detail::ToolKey& key,
        Compute&& compute)
    {
        return getOrCompute<
            profile_candidate_cache_detail::ToolKey,
            profile_candidate_cache_detail::ToolGeometry>(
                tools, key, std::forward<Compute>(compute));
    }

    template <typename Compute>
    std::shared_ptr<const profile_candidate_cache_detail::RestGeometry>
    getRest(
        const profile_candidate_cache_detail::RestKey& key,
        Compute&& compute)
    {
        return getOrCompute<
            profile_candidate_cache_detail::RestKey,
            profile_candidate_cache_detail::RestGeometry>(
                rests, key, std::forward<Compute>(compute));
    }
};

ProfileCandidateGeometryCache::ProfileCandidateGeometryCache()
    : m_impl(std::make_unique<Impl>())
{
}

ProfileCandidateGeometryCache::~ProfileCandidateGeometryCache() = default;

std::size_t ProfileCandidateGeometryCache::hitCount() const
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->hits;
}

std::size_t ProfileCandidateGeometryCache::missCount() const
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->misses;
}

std::size_t ProfileCandidateGeometryCache::entryCount() const
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->layers.size() + m_impl->tools.size() +
        m_impl->rests.size();
}

std::size_t ProfileCandidateGeometryCache::estimatedByteCount() const
{
    using namespace profile_candidate_cache_detail;
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    std::size_t result = sizeof(*m_impl);
    result += m_impl->layers.size() *
        (sizeof(LayerKey) + sizeof(std::shared_ptr<Impl::LayerEntry>));
    result += m_impl->tools.size() *
        (sizeof(ToolKey) + sizeof(std::shared_ptr<Impl::ToolEntry>));
    result += m_impl->rests.size() *
        (sizeof(RestKey) + sizeof(std::shared_ptr<Impl::RestEntry>));
    for (const auto& [key, entry] : m_impl->layers) {
        static_cast<void>(key);
        result += sizeof(*entry);
        if (!entry->ready || !entry->value)
            continue;
        const LayerGeometry& geometry = *entry->value;
        result += sizeof(geometry) + pathsBytes(geometry.layerBase) +
            pathsBytes(geometry.protectedMaterialBase) +
            pathsBytes(geometry.materialArea) +
            geometry.error.capacity() + 1u;
        if (geometry.linkProtectedBoundary) {
            result += geometry.linkProtectedBoundary->capacity() * sizeof(ProfileBoundary);
            for (const auto& boundary : *geometry.linkProtectedBoundary)
                result += boundary.capacity() * sizeof(gp_Pnt2d);
        }
        if (geometry.protectedMaterialBoundary &&
            geometry.protectedMaterialBoundary !=
                geometry.linkProtectedBoundary) {
            result += geometry.protectedMaterialBoundary->capacity() * sizeof(ProfileBoundary);
            for (const auto& boundary : *geometry.protectedMaterialBoundary)
                result += boundary.capacity() * sizeof(gp_Pnt2d);
        }
    }
    for (const auto& [key, entry] : m_impl->tools) {
        static_cast<void>(key);
        result += sizeof(*entry);
        if (!entry->ready || !entry->value)
            continue;
        const ToolGeometry& geometry = *entry->value;
        result += sizeof(geometry) +
            pathsBytes(geometry.cumulativeCenters) +
            pathsBytes(geometry.safeCenters) +
            pathsBytes(geometry.cumulativeSweep) +
            pathsBytes(geometry.finishPaths);
    }
    for (const auto& [key, entry] : m_impl->rests) {
        static_cast<void>(key);
        result += sizeof(*entry);
        if (entry->ready && entry->value) {
            result += sizeof(RestGeometry) +
                pathsBytes(entry->value->centers);
        }
    }
    return result;
}

namespace {

constexpr double kTolerance = 1.0e-9;
using Clipper2Lib::PathD;
using Clipper2Lib::PathsD;
using Clipper2Lib::PointD;
using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(const Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - start)
        .count();
}

class ScopedMilliseconds {
public:
    explicit ScopedMilliseconds(double& accumulator)
        : m_accumulator(accumulator), m_start(Clock::now())
    {
    }

    ~ScopedMilliseconds()
    {
        m_accumulator += elapsedMilliseconds(m_start);
    }

private:
    double& m_accumulator;
    Clock::time_point m_start;
};

struct ProfilePathInfo {
    PathD path;
    double area = 0.0;
    double absArea = 0.0;
    int nesting = 0;
    double minX = 0.0;
    double minY = 0.0;
};

double signedArea(const PathD& path)
{
    return Clipper2Lib::Area(path);
}

double gridSize(int precision)
{
    return std::pow(10.0, -precision);
}

bool nearSame(const PointD& a, const PointD& b, double tolerance)
{
    return std::hypot(a.x - b.x, a.y - b.y) <= tolerance;
}

std::pair<double, double> minPoint(const PathD& path)
{
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    for (const PointD& point : path) {
        minX = std::min(minX, point.x);
        minY = std::min(minY, point.y);
    }
    return {minX, minY};
}

PathsD cleanPaths(const PathsD& paths, int precision)
{
    const double epsilon = std::max(1.0e-9, gridSize(precision) * 0.5);
    const double minArea = std::max(1.0e-9, epsilon * epsilon * 4.0);
    PathsD simplified = Clipper2Lib::SimplifyPaths(paths, epsilon, true);
    PathsD cleaned;
    cleaned.reserve(simplified.size());
    for (PathD path : simplified) {
        while (path.size() > 1 && nearSame(path.front(), path.back(), epsilon))
            path.pop_back();
        if (path.size() < 3 || std::abs(signedArea(path)) <= minArea)
            continue;
        cleaned.push_back(std::move(path));
    }
    return cleaned;
}

PathsD offsetArea(
    const PathsD& paths,
    double delta,
    int precision,
    double arcTolerance)
{
    if (paths.empty() || !std::isfinite(delta))
        return {};
    if (std::abs(delta) <= 1.0e-12) {
        return cleanPaths(
            Clipper2Lib::Union(
                paths, Clipper2Lib::FillRule::NonZero, precision),
            precision);
    }
    PathsD offset = Clipper2Lib::InflatePaths(
        paths, delta,
        Clipper2Lib::JoinType::Round,
        Clipper2Lib::EndType::Polygon,
        2.0, precision, arcTolerance);
    offset = cleanPaths(offset, precision);
    return offset.empty()
        ? offset
        : cleanPaths(
              Clipper2Lib::Union(
                  offset, Clipper2Lib::FillRule::NonZero, precision),
              precision);
}

PathsD differenceArea(
    const PathsD& subject,
    const PathsD& clip,
    int precision)
{
    if (subject.empty())
        return {};
    Clipper2Lib::ClipperD clipper(precision);
    clipper.AddSubject(subject);
    if (!clip.empty())
        clipper.AddClip(clip);
    PathsD solution;
    if (!clipper.Execute(
            Clipper2Lib::ClipType::Difference,
            Clipper2Lib::FillRule::NonZero,
            solution)) {
        return {};
    }
    return cleanPaths(solution, precision);
}

PathsD intersectArea(
    const PathsD& subject,
    const PathsD& clip,
    int precision)
{
    if (subject.empty() || clip.empty())
        return {};
    return cleanPaths(
        Clipper2Lib::Intersect(
            subject, clip,
            Clipper2Lib::FillRule::NonZero,
            precision),
        precision);
}

double profileValidationGuard(int precision)
{
    const double quantization = std::pow(
        10.0, -std::clamp(precision, 0, 8));
    // ProfileSafetyValidator 和生成器各自对成品曲线做离散。把两倍验证
    // 容差直接加到成品保护偏置上，覆盖两条离散链路的最坏方向叠加；
    // 不对复合环整体做负偏置，避免内孔绕向影响保护方向。
    return 2.0 * std::max(0.02, 4.0 * quantization);
}

PathsD externalCenterArea(
    const PathD& stock,
    const PathsD& protectedBoundary,
    double cutterRadius,
    double roughingAllowance,
    int precision,
    double arcTolerance)
{
    return differenceArea(
        offsetArea(
            PathsD{stock}, cutterRadius,
            precision, arcTolerance),
        offsetArea(
            protectedBoundary,
            cutterRadius + roughingAllowance +
                profileValidationGuard(precision),
            precision, arcTolerance),
        precision);
}

// 外部清料余料区域：分别计算当前刀和前序大刀的完整扫掠材料，取差后
// 扩回当前刀心域。与槽腔余料计算相同，这里保留一个小的数值内缩，避免
// 多次圆角偏置后的量化边界被安全验证误判为侵入成品。
PathsD externalRestCenterArea(
    const PathsD& currentCenters,
    const PathsD& previousCenters,
    const PathsD& currentSweep,
    const PathsD& previousSweep,
    double currentRadius,
    double previousRadius,
    int precision,
    double arcTolerance)
{
    if (previousRadius <= currentRadius + kTolerance)
        return {};
    if (currentCenters.empty())
        return {};
    if (previousCenters.empty())
        return currentCenters;
    if (currentSweep.empty())
        return {};
    if (previousSweep.empty())
        return currentCenters;

    const PathsD incrementalMaterial = differenceArea(
        currentSweep, previousSweep, precision);
    if (incrementalMaterial.empty())
        return {};
    PathsD relevantCenters = offsetArea(
        incrementalMaterial, currentRadius,
        precision, arcTolerance);
    relevantCenters = intersectArea(
        currentCenters, relevantCenters, precision);
    if (relevantCenters.empty())
        return {};

    return intersectArea(
        currentCenters, relevantCenters, precision);
}

std::vector<ProfilePathInfo> sortedPathInfos(const PathsD& paths)
{
    std::vector<ProfilePathInfo> infos;
    infos.reserve(paths.size());
    for (const PathD& path : paths) {
        const double area = signedArea(path);
        auto [minX, minY] = minPoint(path);
        infos.push_back({path, area, std::abs(area), 0, minX, minY});
    }
    for (std::size_t i = 0; i < infos.size(); ++i) {
        if (infos[i].path.empty())
            continue;
        const PointD probe = infos[i].path.front();
        for (std::size_t j = 0; j < infos.size(); ++j) {
            if (i == j || infos[j].absArea <= infos[i].absArea)
                continue;
            if (Clipper2Lib::PointInPolygon(probe, infos[j].path) ==
                Clipper2Lib::PointInPolygonResult::IsInside) {
                ++infos[i].nesting;
            }
        }
    }
    std::sort(
        infos.begin(), infos.end(),
        [](const ProfilePathInfo& a, const ProfilePathInfo& b) {
            if (a.nesting != b.nesting)
                return a.nesting < b.nesting;
            if ((a.area < 0.0) != (b.area < 0.0))
                return a.area > b.area;
            if (std::abs(a.absArea - b.absArea) > 1.0e-9)
                return a.absArea > b.absArea;
            if (std::abs(a.minY - b.minY) > 1.0e-9)
                return a.minY < b.minY;
            return a.minX < b.minX;
        });
    return infos;
}

// 偏置图案按圈次发布多分量结果（A0、B0、A1、B1……）。外轮廓清料
// 更适合在同一深度内按连通分量执行（A0、A1……、B0、B1……），避免
// 刀具在彼此分离的余料区域之间反复抬刀横移。
void groupClearingLoopsByConnectedArea(
    const PathsD& centerArea,
    std::vector<ClearingLoop>& loops)
{
    if (centerArea.size() < 2u || loops.size() < 2u)
        return;

    std::vector<PathD> components;
    for (ProfilePathInfo& info : sortedPathInfos(centerArea)) {
        if (info.nesting == 0 && info.area > kTolerance)
            components.push_back(std::move(info.path));
    }
    if (components.size() < 2u)
        return;

    std::vector<int> assignments(loops.size(), -1);
    std::vector<int> componentOrder;
    componentOrder.reserve(components.size());
    for (std::size_t loopIndex = 0u;
         loopIndex < loops.size(); ++loopIndex) {
        const ClearingLoop& loop = loops[loopIndex];
        for (const gp_Pnt2d& point : loop.points) {
            const PointD probe(point.X(), point.Y());
            for (std::size_t componentIndex = 0u;
                 componentIndex < components.size(); ++componentIndex) {
                if (Clipper2Lib::PointInPolygon(
                        probe, components[componentIndex]) ==
                    Clipper2Lib::PointInPolygonResult::IsOutside) {
                    continue;
                }
                assignments[loopIndex] =
                    static_cast<int>(componentIndex);
                if (std::find(
                        componentOrder.begin(), componentOrder.end(),
                        assignments[loopIndex]) == componentOrder.end()) {
                    componentOrder.push_back(assignments[loopIndex]);
                }
                break;
            }
            if (assignments[loopIndex] >= 0)
                break;
        }
    }
    if (componentOrder.size() < 2u)
        return;

    std::vector<ClearingLoop> grouped;
    grouped.reserve(loops.size());
    std::vector<unsigned char> moved(loops.size(), 0u);
    for (const int componentIndex : componentOrder) {
        for (std::size_t loopIndex = 0u;
             loopIndex < loops.size(); ++loopIndex) {
            if (assignments[loopIndex] != componentIndex)
                continue;
            grouped.push_back(std::move(loops[loopIndex]));
            moved[loopIndex] = 1u;
        }
    }
    for (std::size_t loopIndex = 0u;
         loopIndex < loops.size(); ++loopIndex) {
        if (moved[loopIndex] == 0u)
            grouped.push_back(std::move(loops[loopIndex]));
    }
    loops = std::move(grouped);
}

// An offset region can begin as one connected outline and split into several
// independent lobes after a few inward rings. Ring-major publication would
// alternate between those lobes. Build the containment tree of positive
// offset loops and publish it depth-first so a split branch is completed
// before its sibling branch starts.
void groupOffsetLoopsByBranch(std::vector<ClearingLoop>& loops)
{
    struct BranchNode {
        std::size_t loopIndex = 0u;
        PathD polygon;
        int parent = -1;
        std::vector<int> children;
        std::vector<std::size_t> holes;
    };

    if (loops.size() < 2u)
        return;

    std::vector<BranchNode> nodes;
    nodes.reserve(loops.size());
    for (std::size_t loopIndex = 0u;
         loopIndex < loops.size(); ++loopIndex) {
        const ClearingLoop& loop = loops[loopIndex];
        if (!loop.closed || loop.isHole || loop.points.size() < 3u)
            continue;
        BranchNode node;
        node.loopIndex = loopIndex;
        node.polygon.reserve(loop.points.size());
        for (const gp_Pnt2d& point : loop.points)
            node.polygon.emplace_back(point.X(), point.Y());
        nodes.push_back(std::move(node));
    }
    if (nodes.size() < 2u)
        return;

    const auto containsPoint = [&](const BranchNode& outer,
                                   const gp_Pnt2d& point) {
        return Clipper2Lib::PointInPolygon(
                   PointD(point.X(), point.Y()), outer.polygon) !=
            Clipper2Lib::PointInPolygonResult::IsOutside;
    };

    for (std::size_t childIndex = 0u;
         childIndex < nodes.size(); ++childIndex) {
        const ClearingLoop& child = loops[nodes[childIndex].loopIndex];
        int selectedParent = -1;
        int selectedRing = std::numeric_limits<int>::min();
        double selectedArea = std::numeric_limits<double>::max();
        for (std::size_t candidateIndex = 0u;
             candidateIndex < nodes.size(); ++candidateIndex) {
            if (candidateIndex == childIndex)
                continue;
            const ClearingLoop& candidate =
                loops[nodes[candidateIndex].loopIndex];
            if (candidate.ring >= child.ring ||
                candidate.ring < selectedRing ||
                !containsPoint(
                    nodes[candidateIndex], child.points.front())) {
                continue;
            }
            const double candidateArea = std::abs(candidate.area);
            if (candidate.ring > selectedRing ||
                candidateArea < selectedArea) {
                selectedParent = static_cast<int>(candidateIndex);
                selectedRing = candidate.ring;
                selectedArea = candidateArea;
            }
        }
        nodes[childIndex].parent = selectedParent;
        if (selectedParent >= 0) {
            nodes[static_cast<std::size_t>(selectedParent)]
                .children.push_back(static_cast<int>(childIndex));
        }
    }

    for (std::size_t loopIndex = 0u;
         loopIndex < loops.size(); ++loopIndex) {
        const ClearingLoop& hole = loops[loopIndex];
        if (!hole.closed || !hole.isHole || hole.points.size() < 3u)
            continue;
        int selectedNode = -1;
        double selectedArea = std::numeric_limits<double>::max();
        for (std::size_t nodeIndex = 0u;
             nodeIndex < nodes.size(); ++nodeIndex) {
            const ClearingLoop& candidate =
                loops[nodes[nodeIndex].loopIndex];
            if (candidate.ring != hole.ring ||
                !containsPoint(nodes[nodeIndex], hole.points.front())) {
                continue;
            }
            const double candidateArea = std::abs(candidate.area);
            if (candidateArea < selectedArea) {
                selectedNode = static_cast<int>(nodeIndex);
                selectedArea = candidateArea;
            }
        }
        if (selectedNode >= 0) {
            nodes[static_cast<std::size_t>(selectedNode)]
                .holes.push_back(loopIndex);
        }
    }

    std::vector<int> roots;
    roots.reserve(nodes.size());
    bool hasSplit = false;
    for (std::size_t nodeIndex = 0u;
         nodeIndex < nodes.size(); ++nodeIndex) {
        if (nodes[nodeIndex].parent < 0)
            roots.push_back(static_cast<int>(nodeIndex));
        if (nodes[nodeIndex].children.size() > 1u)
            hasSplit = true;
    }
    if (roots.size() < 2u && !hasSplit)
        return;

    std::vector<ClearingLoop> grouped;
    grouped.reserve(loops.size());
    std::vector<unsigned char> moved(loops.size(), 0u);
    std::vector<int> pending;
    pending.reserve(nodes.size());
    for (auto root = roots.rbegin(); root != roots.rend(); ++root)
        pending.push_back(*root);
    while (!pending.empty()) {
        const int nodeIndex = pending.back();
        pending.pop_back();
        BranchNode& node = nodes[static_cast<std::size_t>(nodeIndex)];
        grouped.push_back(std::move(loops[node.loopIndex]));
        moved[node.loopIndex] = 1u;
        for (const std::size_t holeIndex : node.holes) {
            grouped.push_back(std::move(loops[holeIndex]));
            moved[holeIndex] = 1u;
        }
        for (auto child = node.children.rbegin();
             child != node.children.rend(); ++child) {
            pending.push_back(*child);
        }
    }
    for (std::size_t loopIndex = 0u;
         loopIndex < loops.size(); ++loopIndex) {
        if (moved[loopIndex] == 0u)
            grouped.push_back(std::move(loops[loopIndex]));
    }
    loops = std::move(grouped);
}

struct DepthLayer {
    int index = 0;
    double offset = 0.0;
};

std::vector<DepthLayer> buildDepthLayers(
    double recognizedDepth, const DepthParams& depth)
{
    const double start = depth.useFeatureDepth
        ? std::max(0.0, recognizedDepth) : depth.startDepth;
    const double final = depth.useFeatureDepth ? 0.0 : depth.finalDepth;
    if (!std::isfinite(start) || !std::isfinite(final))
        return {};
    double stepDown = depth.stepDown;
    if (!std::isfinite(stepDown) || stepDown <= kTolerance)
        stepDown = std::max(1.0, std::abs(start - final));
    if (std::abs(start - final) <= kTolerance)
        return {{0, final}};

    const double direction = final < start ? -1.0 : 1.0;
    const int limit = std::max(
        1, static_cast<int>(std::ceil(std::abs(start - final) / stepDown)) + 2);
    std::vector<DepthLayer> layers;
    double current = start;
    for (int index = 0; index < limit; ++index) {
        const double remaining = std::abs(final - current);
        if (remaining <= kTolerance)
            break;
        const double delta = std::min(stepDown, remaining);
        current += direction * delta;
        if (std::abs(current - final) <= kTolerance)
            current = final;
        layers.push_back({static_cast<int>(layers.size()), current});
    }
    return layers;
}

PathD boundaryPath(const Loop2d& boundary)
{
    PathD path;
    path.reserve(boundary.points.size());
    for (const gp_Pnt2d& point : boundary.points)
        path.emplace_back(point.X(), point.Y());
    if (signedArea(path) < 0.0)
        std::reverse(path.begin(), path.end());
    return path;
}

SharedProfileBoundaries sharedBoundary(const PathsD& paths)
{
    auto boundaries = std::make_shared<ProfileBoundaries>();
    boundaries->reserve(paths.size());
    for (const PathD& path : paths) {
        ProfileBoundary boundary;
        boundary.reserve(path.size());
        for (const PointD& point : path)
            boundary.emplace_back(point.x, point.y);
        boundaries->push_back(std::move(boundary));
    }
    return boundaries;
}

// 把超过最大长度的线段均匀细分，避免长直线段导致偏置/仿真误差。
void subdivideLongSegments(PathD& path, double maximumLength)
{
    if (path.size() < 2 || maximumLength <= kTolerance)
        return;
    PathD subdivided;
    subdivided.reserve(path.size());
    for (std::size_t index = 0; index < path.size(); ++index) {
        const PointD& from = path[index];
        const PointD& to = path[(index + 1) % path.size()];
        subdivided.push_back(from);
        const double length = std::hypot(to.x - from.x, to.y - from.y);
        const int pieces = std::max(
            1, static_cast<int>(std::ceil(length / maximumLength)));
        for (int piece = 1; piece < pieces; ++piece) {
            const double t = static_cast<double>(piece) /
                static_cast<double>(pieces);
            subdivided.emplace_back(
                from.x + t * (to.x - from.x),
                from.y + t * (to.y - from.y));
        }
    }
    path = std::move(subdivided);
}

// Keep every disconnected outer component. Fill interior holes deliberately:
// external-profile clearing must not enter holes belonging to other operations.
PathsD accumulatedOuterEnvelope(
    const PathsD& previous, const PathsD& current, int precision)
{
    PathsD paths = previous;
    paths.insert(paths.end(), current.begin(), current.end());
    const auto infos = sortedPathInfos(cleanPaths(
        Clipper2Lib::Union(paths, Clipper2Lib::FillRule::NonZero, precision),
        precision));
    PathsD outer;
    for (const ProfilePathInfo& info : infos) {
        if (info.path.size() >= 3 && info.area > kTolerance && info.nesting == 0)
            outer.push_back(info.path);
    }
    return outer;
}

bool materialBoundariesAtDepth(
    const ProfileRegion& region,
    double depthOffset,
    const ToolpathGenerationControl* control,
    PathsD& protectedMaterialBoundary,
    std::string& error)
{
    PocketRegion slicingRegion;
    slicingRegion.floorPlane = gp_Pln(region.frame);
    slicingRegion.openDir = region.openDir;
    slicingRegion.frame = region.frame;
    slicingRegion.depth = region.depth;
    slicingRegion.protectedMaterialShape = region.protectedMaterialShape;
    slicingRegion.sectionTolerance = region.sectionTolerance;
    slicingRegion.sectionDeflection = region.sectionDeflection;
    slicingRegion.boundary = region.boundary;

    PocketLayerSlice slice;
    const bool sliced = control &&
            control->pocketMaterialSectionCache
        ? control->pocketMaterialSectionCache->slice(
              slicingRegion, depthOffset, slice, error, control)
        : PocketLayerSlicer().sliceProtectedMaterial(
              slicingRegion, depthOffset, slice, error, control);
    if (!sliced) {
        return false;
    }
    protectedMaterialBoundary.clear();
    for (const Loop2d& contour : slice.protectedMaterialContours) {
        if (contour.closed && contour.points.size() >= 3 &&
            contour.signedArea > kTolerance) {
            protectedMaterialBoundary.push_back(boundaryPath(contour));
        }
    }
    if (protectedMaterialBoundary.empty()) {
        error = "profile material section has no outer boundary";
        return false;
    }
    return true;
}

gp_Pnt worldPoint(
    const gp_Pnt2d& point, const ProfileRegion& region, double depthOffset)
{
    gp_Pnt world = OcctAreaAdapter::pointFrom2d(point, region.frame);
    world.Translate(gp_Vec(region.openDir) * depthOffset);
    return world;
}

void appendProfileLoop(
    ProfileContourResult& result,
    const ClearingLoop& source,
    const SharedProfileBoundaries& protectedMaterialBoundary,
    const SharedProfileBoundaries& linkProtectedBoundary,
    const ProfileRegion& region,
    const DepthLayer& layer,
    ProfilePassKind passKind,
    int& sequence)
{
    const std::size_t minimumPoints = source.closed ? 3u : 2u;
    if (source.points.size() < minimumPoints)
        return;
    ProfileLoop loop;
    loop.closed = source.closed;
    loop.passKind = passKind;
    loop.depthIndex = layer.index;
    loop.radialPassIndex = source.ring;
    loop.sequence = sequence++;
    loop.depthOffset = layer.offset;
    loop.offset = source.offset;
    loop.area = std::abs(source.area);
    loop.materialBoundaries = protectedMaterialBoundary;
    loop.linkSafetyBoundaries = linkProtectedBoundary;
    loop.points = source.points;
    loop.world.reserve(loop.points.size());
    for (const gp_Pnt2d& point : loop.points)
        loop.world.push_back(worldPoint(point, region, layer.offset));
    result.loops.push_back(std::move(loop));
}

} // namespace

ProfileContourResult ProfileContour::generate(
    const ProfileRegion& region,
    const ToolpathParams& params,
    const ToolpathGenerationControl* control) const
{
    ToolpathGenerationControl boundedControl =
        control ? *control : ToolpathGenerationControl{};
    ensureToolpathParallelBudget(boundedControl);
    ToolpathParallelScope parallelScope(&boundedControl);
    control = &boundedControl;
    ProfileContourResult result;
    const Clock::time_point totalStart = Clock::now();
    PocketMaterialSectionCache* sectionCache = control &&
            control->pocketMaterialSectionCache
        ? control->pocketMaterialSectionCache.get()
        : nullptr;
    ProfileCandidateGeometryCache* candidateGeometryCache = control &&
            control->profileCandidateGeometryCache
        ? control->profileCandidateGeometryCache.get()
        : nullptr;
    const std::size_t cacheHitsBefore = sectionCache
        ? sectionCache->hitCount() : 0u;
    const std::size_t cacheMissesBefore = sectionCache
        ? sectionCache->missCount() : 0u;
    const std::size_t geometryHitsBefore = candidateGeometryCache
        ? candidateGeometryCache->hitCount() : 0u;
    const std::size_t geometryMissesBefore = candidateGeometryCache
        ? candidateGeometryCache->missCount() : 0u;
    const auto finalize = [&]() -> ProfileContourResult {
        if (sectionCache) {
            result.materialSectionCacheHitCount =
                sectionCache->hitCount() - cacheHitsBefore;
            result.materialSectionCacheMissCount =
                sectionCache->missCount() - cacheMissesBefore;
        }
        if (candidateGeometryCache) {
            result.candidateGeometryCacheHitCount =
                candidateGeometryCache->hitCount() - geometryHitsBefore;
            result.candidateGeometryCacheMissCount =
                candidateGeometryCache->missCount() -
                geometryMissesBefore;
        }
        result.totalMilliseconds = elapsedMilliseconds(totalStart);
        return std::move(result);
    };
    if (region.boundary.points.size() < 3 || !region.boundary.closed ||
        region.depth <= kTolerance) {
        result.failureCode = ProfileContourFailureCode::InvalidRegion;
        result.error = "profile region requires a closed boundary and positive depth";
        return finalize();
    }
    if (!std::isfinite(params.tool.diameter) ||
        params.tool.diameter <= kTolerance) {
        result.failureCode = ProfileContourFailureCode::InvalidParameters;
        result.error = "profile tool diameter must be positive";
        return finalize();
    }

    result.toolRadius = params.tool.radius();
    result.restMachiningPreviousToolDiameter =
        params.clearing.restMachiningPreviousToolDiameter;
    const double previousToolRadius =
        0.5 * result.restMachiningPreviousToolDiameter;
    result.restMachining =
        std::isfinite(previousToolRadius) &&
        previousToolRadius > result.toolRadius +
            std::max(1.0e-7, gridSize(params.profile.precision));
    result.finalOffset = result.toolRadius +
        std::max(0.0, params.profile.sideAllowance);
    const bool useExternalClearing =
        params.profile.machiningMode ==
            ProfileMachiningMode::ExternalClearing &&
        region.stockBoundary.closed &&
        region.stockBoundary.points.size() >= 3;
    result.usedExternalClearing = useExternalClearing;
    if (params.profile.machiningMode ==
            ProfileMachiningMode::ExternalClearing &&
        !useExternalClearing) {
        result.warning =
            "profile stock boundary is unavailable; contour-only compatibility mode was used";
    }
    result.radialPassCount = useExternalClearing
        ? 0 : std::max(1, params.profile.radialPassCount);
    result.radialStepOver = params.profile.radialStepOver;
    if (!std::isfinite(result.radialStepOver) ||
        result.radialStepOver <= kTolerance) {
        const double percent = std::clamp(
            params.clearing.stepOverPercent, 1.0, 100.0);
        result.radialStepOver = params.tool.diameter * percent / 100.0;
    }
    result.safeHeightOffset = region.depth +
        std::max(0.0, params.depth.safeHeight);
    result.clearanceHeightOffset = region.depth +
        std::max(params.depth.safeHeight, params.depth.clearanceHeight);

    const std::vector<DepthLayer> layers =
        buildDepthLayers(region.depth, params.depth);
    if (layers.empty()) {
        result.failureCode = ProfileContourFailureCode::InvalidParameters;
        result.error = "profile depth parameters are invalid";
        return finalize();
    }
    result.depthLayerCount = static_cast<int>(layers.size());

    const PathD recognizedBase = boundaryPath(region.boundary);
    if (recognizedBase.size() < 3) {
        result.failureCode = ProfileContourFailureCode::InvalidRegion;
        result.error = "profile boundary cannot be converted to an area";
        return finalize();
    }
    const PathD stockBase = useExternalClearing
        ? boundaryPath(region.stockBoundary) : PathD{};

    int sequence = 0;
    const int precision = std::clamp(params.profile.precision, 0, 8);
    const double arcTolerance = params.profile.arcTolerance > 0.0
        ? params.profile.arcTolerance
        : gridSize(precision);
    const double roughingAllowance = std::max(
        std::max(0.0, params.profile.sideAllowance),
        std::max(0.0, params.profile.roughingAllowance));
    if (region.useMaterialSectionBoundaryAtEveryDepth &&
        (region.sectionShape.IsNull() ||
         region.protectedMaterialShape.IsNull())) {
        result.failureCode = ProfileContourFailureCode::InvalidRegion;
        result.error =
            "profile layered boundary requires source faces and the final solid";
        return finalize();
    }
    using profile_candidate_cache_detail::LayerGeometry;
    using profile_candidate_cache_detail::LayerKey;
    struct PreparedLayer {
        DepthLayer layer;
        LayerKey key;
        std::shared_ptr<const LayerGeometry> geometry;
    };
    PathsD reachableEnvelope{recognizedBase};
    std::vector<PreparedLayer> preparedLayers;
    preparedLayers.reserve(layers.size());
    // Prefix phase: material protection can only grow with depth, so this
    // pass remains ordered. The expensive cutter geometry and clearing
    // patterns below depend only on the completed immutable layer snapshot.
    for (const DepthLayer& layer : layers) {
        const LayerKey layerKey{
            &region,
            profile_candidate_cache_detail::doubleBits(layer.offset),
            profile_candidate_cache_detail::doubleBits(
                roughingAllowance),
            profile_candidate_cache_detail::doubleBits(arcTolerance),
            precision};
        const auto buildLayerGeometry = [&]() {
            LayerGeometry geometry;
            geometry.layerBase = PathsD{recognizedBase};
            if (region.useMaterialSectionBoundaryAtEveryDepth) {
                std::string sectionError;
                bool sectionOk = false;
                {
                    ScopedMilliseconds measure(
                        result.sectionMilliseconds);
                    sectionOk = materialBoundariesAtDepth(
                        region, layer.offset, control,
                        geometry.protectedMaterialBase,
                        sectionError);
                }
                if (!sectionOk) {
                    geometry.error =
                        "profile material section failed at depth offset " +
                        std::to_string(layer.offset) + ": " +
                        sectionError;
                    return geometry;
                }
                // The axial protection envelope can only grow. Cache it once
                // per layer so every cutter transition sees identical input.
                {
                    ScopedMilliseconds measure(result.offsetMilliseconds);
                    geometry.layerBase = accumulatedOuterEnvelope(
                        reachableEnvelope,
                        geometry.protectedMaterialBase,
                        precision);
                }
                if (geometry.layerBase.empty()) {
                    geometry.error =
                        "profile cumulative material envelope could not be formed";
                    return geometry;
                }
            }
            if (geometry.protectedMaterialBase.empty())
                geometry.protectedMaterialBase = geometry.layerBase;
            geometry.linkProtectedBoundary =
                sharedBoundary(geometry.layerBase);
            geometry.protectedMaterialBoundary =
                region.useMaterialSectionBoundaryAtEveryDepth
                ? sharedBoundary(geometry.protectedMaterialBase)
                : geometry.linkProtectedBoundary;
            if (useExternalClearing &&
                (candidateGeometryCache || result.restMachining)) {
                ScopedMilliseconds measure(result.offsetMilliseconds);
                const PathsD protectedArea = offsetArea(
                    geometry.layerBase, roughingAllowance,
                    precision, arcTolerance);
                geometry.materialArea = differenceArea(
                    PathsD{stockBase}, protectedArea, precision);
            }
            geometry.ok = true;
            return geometry;
        };
        const std::shared_ptr<const LayerGeometry> layerGeometry =
            candidateGeometryCache
            ? candidateGeometryCache->m_impl->getLayer(
                  layerKey, buildLayerGeometry)
            : std::make_shared<const LayerGeometry>(
                  buildLayerGeometry());
        if (!layerGeometry->ok) {
            result.failureCode = ProfileContourFailureCode::OffsetFailed;
            result.error = layerGeometry->error;
            return finalize();
        }
        reachableEnvelope = layerGeometry->layerBase;
        preparedLayers.push_back(
            PreparedLayer{layer, layerKey, layerGeometry});
    }

    ProfileContourResult layerTemplate;
    layerTemplate.toolRadius = result.toolRadius;
    layerTemplate.restMachiningPreviousToolDiameter =
        result.restMachiningPreviousToolDiameter;
    layerTemplate.restMachining = result.restMachining;
    layerTemplate.finalOffset = result.finalOffset;
    layerTemplate.radialPassCount = result.radialPassCount;
    layerTemplate.radialStepOver = result.radialStepOver;
    layerTemplate.safeHeightOffset = result.safeHeightOffset;
    layerTemplate.clearanceHeightOffset = result.clearanceHeightOffset;
    layerTemplate.usedExternalClearing = result.usedExternalClearing;
    std::vector<ProfileContourResult> generatedLayers(
        preparedLayers.size());
    const std::size_t desiredLayerParallelism = toolpathParallelismFor(preparedLayers.size());
    ToolpathParallelLease layerLease(
        control,
        desiredLayerParallelism > 0u
            ? desiredLayerParallelism - 1u : 0u,
        desiredLayerParallelism > 0u
            ? desiredLayerParallelism - 1u : 0u);
    const std::size_t layerWorkerCount = std::min(
        desiredLayerParallelism, layerLease.parallelism());
    result.layerPeakParallelism = std::max(
        result.layerPeakParallelism, layerWorkerCount);
    if (layerWorkerCount > 1u) {
        ++result.parallelLayerBatchCount;
        result.parallelLayerCount +=
            static_cast<int>(preparedLayers.size());
    }
    ToolpathGenerationControl parallelLayerControl =
        control ? *control : ToolpathGenerationControl{};
    parallelLayerControl.reportProgress = {};
    parallelLayerControl.reportStatus = {};
    parallelLayerControl.tryAcquireParallelWorkers =
        [](std::size_t) { return 0u; };
    parallelLayerControl.releaseParallelWorkers =
        [](std::size_t) {};
    const ToolpathGenerationControl* layerControl =
        layerWorkerCount > 1u ? &parallelLayerControl : control;

    toolpathParallelForWorkers(
        preparedLayers.size(), layerWorkerCount,
        [&](std::size_t preparedIndex) {
        ToolpathParallelScope layerScope(layerControl);
        const PreparedLayer& prepared = preparedLayers[preparedIndex];
        ProfileContourResult result = layerTemplate;
        int sequence = 0;
        const ToolpathGenerationControl* control = layerControl;
        const DepthLayer& layer = prepared.layer;
        const LayerKey& layerKey = prepared.key;
        const std::shared_ptr<const LayerGeometry>& layerGeometry =
            prepared.geometry;
        const PathsD& layerBase = layerGeometry->layerBase;
        const PathsD& protectedMaterialBase =
            layerGeometry->protectedMaterialBase;
        const SharedProfileBoundaries& linkProtectedBoundary =
            layerGeometry->linkProtectedBoundary;
        const SharedProfileBoundaries& protectedMaterialBoundary =
            layerGeometry->protectedMaterialBoundary;

        if (useExternalClearing) {
            // 刀具中心可达区 = 毛坯截面外扩刀具半径 -
            // （累计成品包络外扩刀具半径和粗加工余量）。直接覆盖这个
            // 中心区，开放槽和外缘凹口就会得到多刀去料路径。
            using profile_candidate_cache_detail::ToolGeometry;
            using profile_candidate_cache_detail::ToolKey;
            using profile_candidate_cache_detail::RestGeometry;
            using profile_candidate_cache_detail::RestKey;
            const auto fetchToolGeometry =
                [&](double cutterRadius,
                    double finishOffset,
                    bool finishPass) {
                ToolKey toolKey{
                    layerKey,
                    profile_candidate_cache_detail::doubleBits(
                        cutterRadius),
                    profile_candidate_cache_detail::doubleBits(
                        finishPass ? finishOffset : 0.0),
                    finishPass};
                const auto compute = [&]() {
                    ToolGeometry geometry;
                    geometry.cumulativeCenters = externalCenterArea(
                        stockBase, layerBase, cutterRadius,
                        roughingAllowance, precision, arcTolerance);
                    geometry.safeCenters = geometry.cumulativeCenters;
                    if (region.useMaterialSectionBoundaryAtEveryDepth) {
                        geometry.safeCenters = intersectArea(
                            geometry.cumulativeCenters,
                            externalCenterArea(
                                stockBase, protectedMaterialBase,
                                cutterRadius, roughingAllowance,
                                precision, arcTolerance),
                            precision);
                    }
                    if (!geometry.cumulativeCenters.empty() &&
                        !layerGeometry->materialArea.empty()) {
                        geometry.cumulativeSweep = intersectArea(
                            offsetArea(
                                geometry.cumulativeCenters,
                                cutterRadius,
                                precision, arcTolerance),
                            layerGeometry->materialArea, precision);
                    }
                    if (finishPass) {
                        geometry.finishPaths = offsetArea(
                            layerBase, finishOffset,
                            precision, arcTolerance);
                    }
                    return geometry;
                };
                return candidateGeometryCache
                    ? candidateGeometryCache->m_impl->getTool(
                          toolKey, compute)
                    : std::make_shared<const ToolGeometry>(compute());
            };

            std::shared_ptr<const ToolGeometry> currentGeometry;
            std::shared_ptr<const ToolGeometry> previousGeometry;
            std::shared_ptr<const RestGeometry> restGeometry;
            const PathsD* clearableCenterArea = nullptr;
            {
                ScopedMilliseconds measure(result.offsetMilliseconds);
                currentGeometry = fetchToolGeometry(
                    result.toolRadius, result.finalOffset,
                    params.profile.finishPass);
                if (result.restMachining) {
                    previousGeometry = fetchToolGeometry(
                        previousToolRadius,
                        previousToolRadius +
                            std::max(0.0, params.profile.sideAllowance),
                        false);
                    const RestKey restKey{
                        layerKey,
                        profile_candidate_cache_detail::doubleBits(
                            result.toolRadius),
                        profile_candidate_cache_detail::doubleBits(
                            previousToolRadius)};
                    const auto computeRest = [&]() {
                        RestGeometry geometry;
                        geometry.centers = externalRestCenterArea(
                            currentGeometry->cumulativeCenters,
                            previousGeometry->cumulativeCenters,
                            currentGeometry->cumulativeSweep,
                            previousGeometry->cumulativeSweep,
                            result.toolRadius, previousToolRadius,
                            precision, arcTolerance);
                        if (region.useMaterialSectionBoundaryAtEveryDepth) {
                            geometry.centers = intersectArea(
                                geometry.centers,
                                currentGeometry->safeCenters,
                                precision);
                        }
                        return geometry;
                    };
                    restGeometry = candidateGeometryCache
                        ? candidateGeometryCache->m_impl->getRest(
                              restKey, computeRest)
                        : std::make_shared<const RestGeometry>(
                              computeRest());
                    clearableCenterArea = &restGeometry->centers;
                } else {
                    clearableCenterArea =
                        &currentGeometry->safeCenters;
                }
            }
            int layerClearingPassCount = 0;
            CutterCenterClearingPatternResult clearing;
            {
                ScopedMilliseconds measure(result.patternMilliseconds);
                clearing = generateCutterCenterClearingPattern(
                    *clearableCenterArea, params.clearing.pattern,
                    params.profile.cutMode, result.radialStepOver,
                    2.0 * result.toolRadius, params.clearing.adaptive,
                    params.clearing.zigzagAngle,
                    params.clearing.perimeterPass,
                    precision, arcTolerance, control);
                groupClearingLoopsByConnectedArea(
                    *clearableCenterArea, clearing.loops);
                if (params.clearing.pattern !=
                        ClearingPattern::ZigZag &&
                    params.clearing.pattern !=
                        ClearingPattern::FreePath) {
                    groupOffsetLoopsByBranch(clearing.loops);
                }
            }
            layerClearingPassCount = clearing.passCount;
            if (clearing.spiralFallback && result.warning.empty()) {
                result.warning =
                    "profile spiral clearing was not applicable; offset clearing was used";
            }
            result.clearingPassCount = std::max(
                result.clearingPassCount, layerClearingPassCount);
            for (const ClearingLoop& loop : clearing.loops) {
                appendProfileLoop(
                    result, loop, protectedMaterialBoundary,
                    linkProtectedBoundary,
                    region, layer, ProfilePassKind::Clearing, sequence);
            }

            if (params.profile.finishPass) {
                std::vector<ProfilePathInfo> finishInfos;
                {
                    ScopedMilliseconds measure(result.offsetMilliseconds);
                    finishInfos = sortedPathInfos(
                        currentGeometry->finishPaths);
                }
                for (ProfilePathInfo& info : finishInfos) {
                    if (info.path.size() < 3 || info.area <= kTolerance)
                        continue;
                    if (params.profile.cutMode == CutMode::Conventional)
                        std::reverse(info.path.begin(), info.path.end());
                    if (region.useMaterialSectionBoundaryAtEveryDepth) {
                        subdivideLongSegments(
                            info.path,
                            std::max(1.0, params.tool.diameter));
                    }
                    ClearingLoop finish;
                    finish.closed = true;
                    finish.ring = layerClearingPassCount;
                    finish.offset = result.finalOffset;
                    finish.area = info.area;
                    finish.points.reserve(info.path.size());
                    for (const PointD& point : info.path)
                        finish.points.emplace_back(point.x, point.y);
                    appendProfileLoop(
                        result, finish, protectedMaterialBoundary,
                        linkProtectedBoundary,
                        region, layer, ProfilePassKind::Finish, sequence);
                    ++result.finishingPassCount;
                }
            }
            result.radialPassCount = std::max(
                result.radialPassCount,
                layerClearingPassCount +
                    (params.profile.finishPass ? 1 : 0));
        }

        if (!useExternalClearing) {
            for (int radialIndex = 0;
                 radialIndex < result.radialPassCount; ++radialIndex) {
                if (control && control->cancelled()) {
                    result.cancelled = true;
                    result.failureCode = ProfileContourFailureCode::Cancelled;
                    result.error = "profile generation cancelled";
                    result.loops.clear();
                    result.moves.clear();
                    result.primitives.clear();
                    generatedLayers[preparedIndex] = std::move(result);
                    return;
                }
            // FreeCAD Profile 的 MultiDepth/stepover 顺序：先加工离成品面最远的
            // 粗轮廓，再向最终 tool-radius + allowance 偏置收敛。
            const double offset = result.finalOffset +
                (result.radialPassCount - radialIndex - 1) *
                    result.radialStepOver;
                PathsD paths;
                std::vector<ProfilePathInfo> infos;
                {
                    ScopedMilliseconds measure(result.offsetMilliseconds);
                    paths = offsetArea(
                        layerBase, offset, precision,
                        arcTolerance);
                    infos = sortedPathInfos(paths);
                }
                for (ProfilePathInfo& info : infos) {
                    if (info.path.size() < 3 || info.area <= kTolerance)
                        continue;
                    if (params.profile.cutMode == CutMode::Conventional)
                        std::reverse(info.path.begin(), info.path.end());
                // 动态截面补齐后可能包含模型外轮廓上的合法长直边。等距细分
                // 不改变几何，只保证预览、播放和导出的 G1 序列连续可见。
                    if (region.useMaterialSectionBoundaryAtEveryDepth) {
                        subdivideLongSegments(
                            info.path,
                            std::max(1.0, params.tool.diameter));
                    }
                    ProfileLoop loop;
                    loop.depthIndex = layer.index;
                    loop.radialPassIndex = radialIndex;
                    loop.sequence = sequence++;
                    loop.depthOffset = layer.offset;
                    loop.offset = offset;
                    loop.area = std::abs(info.area);
                    if (region.useMaterialSectionBoundaryAtEveryDepth)
                        loop.materialBoundaries = protectedMaterialBoundary;
                    loop.linkSafetyBoundaries = linkProtectedBoundary;
                    loop.points.reserve(info.path.size());
                    loop.world.reserve(info.path.size());
                    for (const PointD& point : info.path) {
                        loop.points.emplace_back(point.x, point.y);
                        loop.world.push_back(worldPoint(
                            loop.points.back(), region, layer.offset));
                    }
                    result.loops.push_back(std::move(loop));
                    ++result.finishingPassCount;
                }
            }
        }
        generatedLayers[preparedIndex] = std::move(result);
    });

    int generatedDepthLayerCount = 0;
    for (std::size_t layerIndex = 0u;
         layerIndex < generatedLayers.size(); ++layerIndex) {
        ProfileContourResult& layerResult = generatedLayers[layerIndex];
        if (layerResult.cancelled || (control && control->cancelled())) {
            result.cancelled = true;
            result.failureCode = ProfileContourFailureCode::Cancelled;
            result.error = "profile generation cancelled";
            result.loops.clear();
            result.moves.clear();
            result.primitives.clear();
            return finalize();
        }
        if (!layerResult.warning.empty() &&
            result.warning.find(layerResult.warning) ==
                std::string::npos) {
            if (!result.warning.empty())
                result.warning += "; ";
            result.warning += layerResult.warning;
        }
        result.offsetMilliseconds += layerResult.offsetMilliseconds;
        result.patternMilliseconds += layerResult.patternMilliseconds;
        result.clearingPassCount = std::max(
            result.clearingPassCount,
            layerResult.clearingPassCount);
        result.finishingPassCount +=
            layerResult.finishingPassCount;
        result.radialPassCount = std::max(
            result.radialPassCount, layerResult.radialPassCount);
        if (!layerResult.loops.empty())
            ++generatedDepthLayerCount;
        for (ProfileLoop& loop : layerResult.loops) {
            loop.sequence = sequence++;
            result.loops.push_back(std::move(loop));
        }
        if (control)
            control->progress(layerIndex + 1u, generatedLayers.size());
    }
    if (result.loops.empty()) {
        if (result.restMachining) {
            result.noRestMaterial = true;
            result.ok = true;
            result.warning =
                "previous profile cutter left no reachable rest material for this tool";
            return finalize();
        }
        result.failureCode = ProfileContourFailureCode::OffsetFailed;
        result.error = "profile exterior offset produced no closed contour";
        return finalize();
    }
    result.depthLayerCount = generatedDepthLayerCount;
    {
        ScopedMilliseconds measure(
            result.travelOptimizationMilliseconds);
        profile_motion_detail::optimizeRestMachiningTravel(region, params, result);
    }
    if (control && control->profileEvaluationOnly) {
        result.evaluationOnly = true;
        {
            ScopedMilliseconds measure(result.moveAssemblyMilliseconds);
            result.estimatedCuttingDistance =
                profile_motion_detail::estimateCuttingDistance(result, region, params);
        }
        result.ok = !result.loops.empty();
        return finalize();
    }
    {
        ScopedMilliseconds measure(result.moveAssemblyMilliseconds);
        profile_motion_detail::assembleMoves(result, region, params);
    }
    result.ok = !result.moves.empty();
    if (!result.ok) {
        result.failureCode = ProfileContourFailureCode::OffsetFailed;
        result.error = "profile contour produced no executable moves";
    }
    return finalize();
}

ProfileContourResult ProfileContour::materializeEvaluation(
    const ProfileRegion& region,
    const ToolpathParams& params,
    ProfileContourResult evaluation,
    const ToolpathGenerationControl* control) const
{
    ToolpathParallelScope parallelScope(control);
    const Clock::time_point totalStart = Clock::now();
    const auto finish = [&]() -> ProfileContourResult {
        const double milliseconds = elapsedMilliseconds(totalStart);
        evaluation.materializationMilliseconds += milliseconds;
        evaluation.totalMilliseconds += milliseconds;
        return std::move(evaluation);
    };
    if (control && control->cancelled()) {
        evaluation.ok = false;
        evaluation.cancelled = true;
        evaluation.failureCode = ProfileContourFailureCode::Cancelled;
        evaluation.error = "profile materialization cancelled";
        evaluation.loops.clear();
        evaluation.moves.clear();
        evaluation.primitives.clear();
        return finish();
    }
    if (!evaluation.evaluationOnly) {
        evaluation.ok = false;
        evaluation.failureCode =
            ProfileContourFailureCode::InvalidParameters;
        evaluation.error =
            "profile materialization requires an evaluation-only contour";
        return finish();
    }
    if (!std::isfinite(params.tool.diameter) ||
        params.tool.diameter <= kTolerance ||
        std::abs(evaluation.toolRadius - params.tool.radius()) >
            kTolerance) {
        evaluation.ok = false;
        evaluation.failureCode =
            ProfileContourFailureCode::InvalidParameters;
        evaluation.error =
            "profile materialization tool does not match the evaluated contour";
        return finish();
    }

    evaluation.evaluationOnly = false;
    if (evaluation.restMachining && evaluation.noRestMaterial)
        return finish();
    if (!evaluation.ok || evaluation.loops.empty()) {
        evaluation.ok = false;
        if (evaluation.failureCode == ProfileContourFailureCode::None)
            evaluation.failureCode =
                ProfileContourFailureCode::OffsetFailed;
        if (evaluation.error.empty()) {
            evaluation.error =
                "profile evaluation contains no contour loops to materialize";
        }
        return finish();
    }

    evaluation.moves.clear();
    evaluation.primitives.clear();
    // Evaluation computes these counters while estimating cutting distance;
    // assembly computes the same values again for the executable program.
    evaluation.directCutLinkCount = 0;
    evaluation.retractEntryCount = 0;
    const Clock::time_point assemblyStart = Clock::now();
    profile_motion_detail::assembleMoves(evaluation, region, params);
    evaluation.moveAssemblyMilliseconds +=
        elapsedMilliseconds(assemblyStart);
    evaluation.ok = !evaluation.moves.empty();
    if (!evaluation.ok) {
        evaluation.failureCode = ProfileContourFailureCode::OffsetFailed;
        evaluation.error =
            "profile contour produced no executable moves";
    }
    return finish();
}

} // namespace oap::toolpath
