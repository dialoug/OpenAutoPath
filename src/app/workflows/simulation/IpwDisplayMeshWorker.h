// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 职责：在后台线程从不可变 Multi-Dexel/Z-map 显示源构建显示网格/BRep/图元。
// 线程：由 IpwDisplayMeshController 通过队列槽驱动，绝不触碰 AIS/查看器。
// 协作：StockSimulationCoordinator 持有缓存，evaluateDisplayResult 决定是否发布。
// IPW 派生显示网格后台生成器。
//
// Worker 只读取不可变 Multi-Dexel/Z-map 显示源。逐移动请求使用同一
// Multi-Dexel 材料内核派生不持久化的前缀状态；AIS/OCCT 查看器仍只在
// UI 线程消费最终 Poly_Triangulation。
#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <QObject>
#include <QtGlobal>

#include <Poly_Triangulation.hxx>
#include <TopoDS_Shape.hxx>

#include "app/workflows/simulation/detail/IpwDisplayArtifactCache.h"
#include "app/view/IpwDisplayGeometry.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/pocket/simulation/PocketZMapComparison.h"

namespace oap::app {

// 逐移动临时帧的来源：从不可变父检查点重放前 completedMoveCount 个移动，
// 不写入检查点存储、不进入显示产物 LRU。
struct IpwTransientFrameSource {
    std::shared_ptr<const toolpath::MultiDexelStock>
        parentStock;
    std::shared_ptr<const std::vector<
        toolpath::PocketToolpathOperation>>
        operations;
    // 非空时必须与 operations 一一对应；用于保留球刀/圆鼻刀轮廓。
    std::vector<toolpath::DexelToolProfileDefinition> toolProfiles;
    std::vector<toolpath::ToolpathParams> operationParameters;
    toolpath::ToolpathParams params;
    toolpath::StockSimulationOptions options;
    std::size_t completedMoveCount = 0u;
    // Fraction of the following original move, used only for transient playback.
    // The completed prefix remains a separate immutable checkpoint.
    double nextMoveFraction = 0.0;
    // parentStock may already include this prefix during forward animation.
    // Zero retains the checkpoint-based replay used by seeking and audits.
    std::size_t firstMoveCount = 0u;
    int sequenceIndex = 0;
};

// Resolve unknown cutting length against the complete original operation before
// slicing it into incremental/partial ranges. Callers may cache this in the source.
toolpath::ToolpathParams ipwReplayParameters(const IpwTransientFrameSource& source, std::size_t operationIndex);

// 显示网格构建请求。三类来源互斥：stockSource（完整检查点）、
// differenceSource（Z-map 差异视图）或 transientSource（逐移动临时帧）。
struct IpwDisplayMeshRequest {
    quint64 generation = 0u;
    quint64 requestId = 0u;
    IpwDisplayArtifactKey key;
    std::shared_ptr<const toolpath::MultiDexelStock>
        stockSource;
    std::shared_ptr<const toolpath::PocketZMapDisplaySource>
        differenceSource;
    // 临时帧从不可变父检查点重放移动前缀。它不会发布到
    // IpwCheckpointStore，也不会进入显示产物 LRU。
    std::shared_ptr<const IpwTransientFrameSource>
        transientSource;
    toolpath::PocketZMapDifferenceView differenceView =
        toolpath::PocketZMapDifferenceView::InitialMissing;
    std::string operationId;
    toolpath::DexelDisplayMeshOptions meshOptions;
    IpwOcctDisplayMode displayMode =
        IpwOcctDisplayMode::SurfaceWithEdges;
    // MidSection 使用材料整体 W 范围中的比例位置。
    double sectionFraction = 0.5;
    std::shared_ptr<std::atomic_bool> cancellation;
};

// 构建结果：ok() 依据显示模式校验有效几何；materialStock 供
// OpenGL Multi-Dexel 播放器直接上传区间数据，避免从网格反推拓扑。
struct IpwDisplayMeshResult {
    quint64 generation = 0u;
    quint64 requestId = 0u;
    IpwDisplayArtifactKey key;
    IpwOcctDisplayMode displayMode =
        IpwOcctDisplayMode::SurfaceWithEdges;
    Handle(Poly_Triangulation) mesh;
    // 可选的不可变材料状态。OpenGL GPU Multi-Dexel 播放器可直接上传
    // 区间数据，避免从显示三角网格反推材料拓扑。
    std::shared_ptr<const toolpath::MultiDexelStock> materialStock;
    std::shared_ptr<const toolpath::MultiDexelStock> completedPrefixStock;
    IpwDisplayPrimitiveGeometryPtr primitiveGeometry;
    TopoDS_Shape shape;
    toolpath::DexelDisplayMeshMetrics metrics;
    std::string meshContentFingerprint;
    bool transientFrame = false;
    std::size_t completedMoveCount = 0u;
    std::size_t totalMoveCount = 0u;
    int cuttingMoveCount = 0;
    int touchedTileCount = 0;
    int changedCellCount = 0;
    double materialVolume = 0.0;
    double removedVolume = 0.0;
    double materialUpdateMilliseconds = 0.0;
    // GPU-only transient frames use generation/request IDs for publication and
    // leave this empty. Formal checkpoint digests remain available on demand.
    std::string materialContentFingerprint;
    std::string error;
    bool cancelled = false;

    bool ok() const noexcept
    {
        if (cancelled || !error.empty())
            return false;
        if (isIpwSurfaceDisplayMode(displayMode))
            return !mesh.IsNull();
        if (displayMode == IpwOcctDisplayMode::BRepSnapshot)
            return !shape.IsNull();
        if (displayMode == IpwOcctDisplayMode::Off)
            return true;
        return static_cast<bool>(primitiveGeometry);
    }
};

// 把结果打包为可缓存产物（网格 + 诊断元数据）。
IpwDisplayArtifact displayArtifactFromResult(
    const IpwDisplayMeshResult& result);

// IPW 显示网格工作线程：后台派生 IPW 显示网格。
class IpwDisplayMeshWorker final : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;

public slots:
    void build(IpwDisplayMeshRequest request);

signals:
    void resultReady(const oap::app::IpwDisplayMeshResult& result);
};

} // namespace oap::app

Q_DECLARE_METATYPE(oap::app::IpwDisplayMeshResult)
