// IpwDisplayArtifactCache 实现：质量档位解析、LRU 插入/命中与按预算淘汰。
#include "app/workflows/simulation/detail/IpwDisplayArtifactCache.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <utility>

namespace oap::app {
namespace {

// 组合哈希：boost 风格混合，避免简单异或的分布缺陷。
void combineHash(
    std::size_t& seed,
    std::size_t value) noexcept
{
    seed ^= value +
            static_cast<std::size_t>(
                0x9e3779b97f4a7c15ull) +
            (seed << 6u) + (seed >> 2u);
}

} // namespace

// 1..10 级显示质量 → 网格轴向量化（按材料分辨率比例）与
// 平面单元/三角形预算；材料真值与比较结果不受这些选项影响。
IpwDisplayQualityProfile ipwDisplayQualityProfile(
    int level,
    double gridResolution)
{
    IpwDisplayQualityProfile profile;
    const int clamped = std::clamp(level, 1, 10);
    const double resolution =
        std::isfinite(gridResolution) &&
                gridResolution > 0.0
        ? gridResolution
        : 1.0;
    if (clamped <= 3) {
        profile.quality =
            IpwDisplayQuality::Interactive;
        profile.meshOptions.axialQuantization =
            0.5 * resolution;
        profile.meshOptions.maximumPlanarCellCount =
            125'000u;
        profile.meshOptions.maximumTriangleCount =
            1'000'000u;
    } else if (clamped <= 7) {
        profile.quality =
            IpwDisplayQuality::Balanced;
        profile.meshOptions.axialQuantization =
            0.25 * resolution;
        profile.meshOptions.maximumPlanarCellCount =
            250'000u;
        profile.meshOptions.maximumTriangleCount =
            2'000'000u;
    } else {
        profile.quality =
            IpwDisplayQuality::Detailed;
        // 高分辨率材料网格上的自由曲面可能让每个 XY 单元都产生独立
        // 水平片。保留 1/8 单元的轴向显示量化，使详细档仍能在 OCCT
        // 三角形预算内发布；材料真值、体积和比较结果不受影响。
        profile.meshOptions.axialQuantization =
            0.125 * resolution;
        profile.meshOptions.maximumPlanarCellCount =
            500'000u;
        profile.meshOptions.maximumTriangleCount =
            4'000'000u;
    }
    return profile;
}

// 1..10 级仿真质量 → 每刀径单元数与最大单元数，
// 由此推导请求的 XY 采样分辨率与平面误差上界。
IpwSimulationQualityProfile ipwSimulationQualityProfile(
    int level,
    double referenceCutterDiameter)
{
    IpwSimulationQualityProfile profile;
    const int clamped = std::clamp(level, 1, 10);
    const double cutterDiameter =
        std::isfinite(referenceCutterDiameter) &&
                referenceCutterDiameter > 0.0
        ? referenceCutterDiameter
        : 1.0;
    if (clamped <= 3) {
        profile.quality =
            IpwDisplayQuality::Interactive;
        profile.cellsPerCutterDiameter = 20.0;
        profile.maximumCellCount = 1'000'000;
    } else if (clamped <= 7) {
        profile.quality =
            IpwDisplayQuality::Balanced;
        profile.cellsPerCutterDiameter = 40.0;
        profile.maximumCellCount = 4'000'000;
    } else {
        profile.quality =
            IpwDisplayQuality::Detailed;
        profile.cellsPerCutterDiameter = 80.0;
        profile.maximumCellCount = 8'000'000;
    }
    profile.requestedResolution =
        std::max(
            0.01,
            cutterDiameter /
                profile.cellsPerCutterDiameter);
    profile.requestedPlanarErrorUpperBound =
        profile.requestedResolution /
        std::sqrt(2.0);
    return profile;
}

const char* ipwDisplayQualityName(
    IpwDisplayQuality quality) noexcept
{
    switch (quality) {
        case IpwDisplayQuality::Interactive:
            return "interactive";
        case IpwDisplayQuality::Balanced:
            return "balanced";
        case IpwDisplayQuality::Detailed:
            return "detailed";
    }
    return "unknown";
}

IpwDisplayArtifactCache::IpwDisplayArtifactCache(
    std::size_t budgetBytes)
    : m_budgetBytes(budgetBytes)
{
}

std::size_t IpwDisplayArtifactCache::KeyHash::operator()(
    const IpwDisplayArtifactKey& key) const noexcept
{
    std::size_t seed =
        std::hash<std::string>{}(
            key.contentFingerprint);
    combineHash(
        seed,
        std::hash<std::string>{}(key.variant));
    combineHash(
        seed,
        static_cast<std::size_t>(key.kind));
    combineHash(
        seed,
        static_cast<std::size_t>(key.quality));
    return seed;
}

Handle(Poly_Triangulation)
IpwDisplayArtifactCache::find(
    const IpwDisplayArtifactKey& key)
{
    return findArtifact(key).mesh;
}

// 命中时提升 LRU 位置并返回产物（含诊断元数据）。
IpwDisplayArtifact
IpwDisplayArtifactCache::findArtifact(
    const IpwDisplayArtifactKey& key)
{
    std::lock_guard lock(m_mutex);
    const auto found = m_entries.find(key);
    if (found == m_entries.end()) {
        ++m_missCount;
        return {};
    }
    ++m_hitCount;
    m_lru.splice(
        m_lru.end(),
        m_lru,
        found->second.lruPosition);
    return found->second.artifact;
}

// 插入产物：单件超过预算直接拒绝；键已存在则先替换旧条目，
// 再按预算从 LRU 头部淘汰。
bool IpwDisplayArtifactCache::insert(
    IpwDisplayArtifactKey key,
    const Handle(Poly_Triangulation)& mesh)
{
    IpwDisplayArtifact artifact;
    artifact.mesh = mesh;
    return insert(std::move(key), std::move(artifact));
}

bool IpwDisplayArtifactCache::insert(
    IpwDisplayArtifactKey key,
    IpwDisplayArtifact artifact)
{
    const std::size_t bytes = estimateBytes(artifact);
    if (!artifact.valid() || bytes == 0u)
        return false;
    std::lock_guard lock(m_mutex);
    if (bytes > m_budgetBytes) {
        ++m_rejectedArtifactCount;
        return false;
    }
    const auto existing = m_entries.find(key);
    if (existing != m_entries.end()) {
        m_residentBytes -= existing->second.bytes;
        m_lru.erase(existing->second.lruPosition);
        m_entries.erase(existing);
    }
    m_lru.push_back(key);
    Entry entry;
    entry.artifact = std::move(artifact);
    entry.bytes = bytes;
    entry.lruPosition = std::prev(m_lru.end());
    m_residentBytes += bytes;
    m_entries.emplace(std::move(key), std::move(entry));
    evictToBudgetLocked();
    return true;
}

void IpwDisplayArtifactCache::clear()
{
    std::lock_guard lock(m_mutex);
    m_entries.clear();
    m_lru.clear();
    m_residentBytes = 0u;
    m_hitCount = 0u;
    m_missCount = 0u;
    m_evictionCount = 0u;
    m_rejectedArtifactCount = 0u;
}

void IpwDisplayArtifactCache::setBudgetBytes(
    std::size_t budgetBytes)
{
    std::lock_guard lock(m_mutex);
    m_budgetBytes = budgetBytes;
    evictToBudgetLocked();
}

std::size_t
IpwDisplayArtifactCache::budgetBytes() const
{
    std::lock_guard lock(m_mutex);
    return m_budgetBytes;
}

IpwDisplayCacheStats
IpwDisplayArtifactCache::stats() const
{
    std::lock_guard lock(m_mutex);
    IpwDisplayCacheStats result;
    result.budgetBytes = m_budgetBytes;
    result.residentBytes = m_residentBytes;
    result.residentArtifactCount =
        m_entries.size();
    result.hitCount = m_hitCount;
    result.missCount = m_missCount;
    result.evictionCount = m_evictionCount;
    result.rejectedArtifactCount =
        m_rejectedArtifactCount;
    return result;
}

// 估算网格占用：节点 6 个 double + 三角形 3 个整数，带溢出保护。
std::size_t IpwDisplayArtifactCache::estimateBytes(
    const Handle(Poly_Triangulation)& mesh) noexcept
{
    if (mesh.IsNull() ||
        mesh->NbNodes() <= 0 ||
        mesh->NbTriangles() <= 0) {
        return 0u;
    }
    constexpr std::size_t nodeBytes =
        6u * sizeof(double);
    constexpr std::size_t triangleBytes =
        3u * sizeof(Standard_Integer);
    const std::size_t nodes =
        static_cast<std::size_t>(mesh->NbNodes());
    const std::size_t triangles =
        static_cast<std::size_t>(
            mesh->NbTriangles());
    if (nodes >
            std::numeric_limits<std::size_t>::max() /
                nodeBytes ||
        triangles >
            std::numeric_limits<std::size_t>::max() /
                triangleBytes) {
        return std::numeric_limits<std::size_t>::max();
    }
    const std::size_t nodeTotal =
        nodes * nodeBytes;
    const std::size_t triangleTotal =
        triangles * triangleBytes;
    if (nodeTotal >
        std::numeric_limits<std::size_t>::max() -
            triangleTotal) {
        return std::numeric_limits<std::size_t>::max();
    }
    return nodeTotal + triangleTotal;
}

std::size_t IpwDisplayArtifactCache::estimateBytes(
    const IpwDisplayArtifact& artifact) noexcept
{
    const std::size_t meshBytes =
        estimateBytes(artifact.mesh);
    if (meshBytes == 0u)
        return 0u;
    const std::size_t fingerprintBytes =
        artifact.metadata.meshContentFingerprint.size();
    const std::size_t metadataBytes = fingerprintBytes;
    if (meshBytes >
        std::numeric_limits<std::size_t>::max() -
            metadataBytes) {
        return std::numeric_limits<std::size_t>::max();
    }
    return meshBytes + metadataBytes;
}

// 从 LRU 头部逐项淘汰，直到驻留字节不超预算（调用方必须持锁）。
void IpwDisplayArtifactCache::evictToBudgetLocked()
{
    while (m_residentBytes > m_budgetBytes &&
           !m_lru.empty()) {
        const auto found =
            m_entries.find(m_lru.front());
        if (found != m_entries.end()) {
            m_residentBytes -=
                found->second.bytes;
            m_entries.erase(found);
            ++m_evictionCount;
        }
        m_lru.pop_front();
    }
}

} // namespace oap::app
