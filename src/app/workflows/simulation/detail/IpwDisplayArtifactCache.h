// IPW 派生显示产物的进程内 LRU 缓存。
//
// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 由 StockSimulationCoordinator 创建并持有；后台/UI 线程均经互斥锁访问。
// 线程安全：所有公开方法都在内部加锁，可在任意线程安全调用。
// 缓存键显式包含材料/目标内容指纹、产物种类、变体和质量档位。缓存只保存
// 可重建的 Poly_Triangulation；淘汰或生成失败不会影响 IPW 材料检查点。
#pragma once

#include <cstddef>
#include <cstdint>
#include <list>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Poly_Triangulation.hxx>

#include "toolpath/pocket/simulation/DexelDisplayMesh.h"

namespace oap::app {

// 可缓存的 IPW 派生显示产物种类。
enum class IpwDisplayArtifactKind {
    StockCheckpoint,
    InitialMissing,
    RemainingAllowance,
    FeatureAllowance,
    Overcut,
    OperationOvercut,
};

// 显示质量档位：影响三角网格与图元的生成预算。
enum class IpwDisplayQuality {
    Interactive,
    Balanced,
    Detailed,
};

// 缓存键：内容指纹 + 变体 + 产物种类 + 质量档位；
// 指纹变化（材料/目标变化）即视为新产物。
struct IpwDisplayArtifactKey {
    std::string contentFingerprint;
    std::string variant;
    IpwDisplayArtifactKind kind =
        IpwDisplayArtifactKind::StockCheckpoint;
    IpwDisplayQuality quality =
        IpwDisplayQuality::Detailed;

    bool operator==(
        const IpwDisplayArtifactKey&) const = default;
};

// 缓存统计：预算/驻留字节、命中、未命中、淘汰与单件超预算拒绝计数。
struct IpwDisplayCacheStats {
    std::size_t budgetBytes = 0u;
    std::size_t residentBytes = 0u;
    std::size_t residentArtifactCount = 0u;
    std::uint64_t hitCount = 0u;
    std::uint64_t missCount = 0u;
    std::uint64_t evictionCount = 0u;
    std::uint64_t rejectedArtifactCount = 0u;
};

// 可缓存的诊断信息随网格一起存放，使基于状态键的缓存命中能恢复
// 产物创建时发布的那一份体积审计、语义覆盖与构建指标。
struct IpwDisplayArtifactMetadata {
    toolpath::DexelDisplayMeshMetrics dexelMetrics;
    std::string meshContentFingerprint;
};

// 可缓存的产物：三角网格 + 可随缓存命中恢复的诊断元数据。
struct IpwDisplayArtifact {
    Handle(Poly_Triangulation) mesh;
    IpwDisplayArtifactMetadata metadata;

    bool valid() const noexcept
    {
        return !mesh.IsNull();
    }
};

struct IpwDisplayQualityProfile {
    IpwDisplayQuality quality =
        IpwDisplayQuality::Detailed;
    toolpath::DexelDisplayMeshOptions meshOptions;
};

// 同一 1～10 级质量同时约束 Multi-Dexel 材料真值的 XY 采样密度。
// requestedResolution 是期望值；MultiDexelStock 仍可为满足 maximumCellCount
// 而采用更粗的实际分辨率，调用方必须显示最终实际值而不是只显示请求值。
struct IpwSimulationQualityProfile {
    IpwDisplayQuality quality =
        IpwDisplayQuality::Detailed;
    double cellsPerCutterDiameter = 80.0;
    double requestedResolution = 0.0;
    double requestedPlanarErrorUpperBound = 0.0;
    int maximumCellCount = 8'000'000;
};

IpwDisplayQualityProfile ipwDisplayQualityProfile(
    int level,
    double gridResolution);

IpwSimulationQualityProfile ipwSimulationQualityProfile(
    int level,
    double referenceCutterDiameter);

const char* ipwDisplayQualityName(
    IpwDisplayQuality quality) noexcept;

// 线程安全 LRU 缓存：只保存可重建的显示产物；单件超过预算时拒绝插入。
class IpwDisplayArtifactCache {
public:
    static constexpr std::size_t DefaultBudgetBytes =
        384u * 1024u * 1024u;

    explicit IpwDisplayArtifactCache(
        std::size_t budgetBytes =
            DefaultBudgetBytes);

    Handle(Poly_Triangulation) find(
        const IpwDisplayArtifactKey& key);
    IpwDisplayArtifact findArtifact(
        const IpwDisplayArtifactKey& key);
    bool insert(
        IpwDisplayArtifactKey key,
        const Handle(Poly_Triangulation)& mesh);
    bool insert(
        IpwDisplayArtifactKey key,
        IpwDisplayArtifact artifact);

    void clear();
    void setBudgetBytes(std::size_t budgetBytes);
    std::size_t budgetBytes() const;
    IpwDisplayCacheStats stats() const;

    static std::size_t estimateBytes(
        const Handle(Poly_Triangulation)& mesh) noexcept;
    static std::size_t estimateBytes(
        const IpwDisplayArtifact& artifact) noexcept;

private:
    struct KeyHash {
        std::size_t operator()(
            const IpwDisplayArtifactKey& key) const noexcept;
    };

    struct Entry {
        IpwDisplayArtifact artifact;
        std::size_t bytes = 0u;
        std::list<IpwDisplayArtifactKey>::iterator
            lruPosition;
    };

    void evictToBudgetLocked();

    mutable std::mutex m_mutex;
    std::size_t m_budgetBytes = 0u;
    std::size_t m_residentBytes = 0u;
    std::uint64_t m_hitCount = 0u;
    std::uint64_t m_missCount = 0u;
    std::uint64_t m_evictionCount = 0u;
    std::uint64_t m_rejectedArtifactCount = 0u;
    std::list<IpwDisplayArtifactKey> m_lru;
    std::unordered_map<
        IpwDisplayArtifactKey,
        Entry,
        KeyHash>
        m_entries;
};

} // namespace oap::app
