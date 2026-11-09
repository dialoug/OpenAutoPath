// PocketLayerSlicer —— 使用与底面平行的平面截取已识别腔体面组，
// 并将结果转换为归一化二维环。
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "toolpath/pocket/geometry/PocketRegionBuilder.h"

namespace oap::toolpath {

struct ToolpathGenerationControl;

// 切片失败码：区分无效深度、OCCT 截面失败、无边/无线框、开放轮廓等，
// 由 PocketClearing 映射为 PocketClearingFailureCode。
enum class PocketLayerSliceFailureCode {
    None,
    InvalidDepth,
    MissingSectionShape,
    SectionOperationFailed,
    NoEdges,
    NoWires,
    OpenContour,
    NoValidContour
};

// 槽腔分层切片：单层加工深度与截面环。
struct PocketLayerSlice {
    // 在 PocketRegion::frame 中，外轮廓为逆时针，岛轮廓为顺时针。
    std::vector<Loop2d> contours;
    // 同一层稍向开口侧取样得到的最终工件材料截面。轮廓按嵌套奇偶
    // 归一化，供清腔区域在通槽扩展后执行材料差集。
    std::vector<Loop2d> protectedMaterialContours;
    PocketLayerSliceFailureCode failureCode = PocketLayerSliceFailureCode::None;
    double connectionTolerance = 0.0;
    int connectionAttemptCount = 0;
    int sectionEdgeCount = 0;
    int wireCount = 0;
    int virtualOpenBoundaryCount = 0;
    std::size_t wireParallelBatchCount = 0u;
    std::size_t wireParallelTaskCount = 0u;
    std::size_t wirePeakParallelism = 1u;
    std::size_t occtInternalParallelCount = 0u;
    std::size_t occtSerialFallbackCount = 0u;
};

// 槽腔截面缓存：以“区域形状 + 采样平面 + 坐标系”为键，同时缓存完整
// 区域切片与保护材料切片。内部使用 single-flight，同一键并发首次请求只
// 计算一次，其余调用等待并复用结果。
class PocketMaterialSectionCache {
public:
    PocketMaterialSectionCache();
    ~PocketMaterialSectionCache();
    PocketMaterialSectionCache(
        const PocketMaterialSectionCache&) = delete;
    PocketMaterialSectionCache& operator=(
        const PocketMaterialSectionCache&) = delete;

    bool slice(const PocketRegion& region,
               double depthOffset,
               PocketLayerSlice& result,
               std::string& error,
               const ToolpathGenerationControl* control = nullptr);
    bool sliceRegion(const PocketRegion& region,
                     double depthOffset,
                     PocketLayerSlice& result,
                     std::string& error,
                     const ToolpathGenerationControl* control = nullptr);
    std::size_t hitCount() const;
    std::size_t missCount() const;
    std::size_t entryCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// 逐深度层截面切片器：用与底面平行的平面截取已识别腔体面组（或仅截取
// 最终工件材料），将离散交线连接为归一化二维轮廓。无状态，每次 slice()
// 独立；内部对象仅供本模块使用，不经由共享接口暴露。
class PocketLayerSlicer {
public:
    // 仅截取最终工件材料，不依赖已识别的槽腔 sectionShape。
    // 固定三轴清腔会把开口层及更浅加工层的这些截面累积为轴向材料阴影。
    bool sliceProtectedMaterial(const PocketRegion& region,
                                double depthOffset,
                                PocketLayerSlice& result,
                                std::string& error,
                                const ToolpathGenerationControl* control =
                                    nullptr) const;

    // depthOffset 从最深层沿 region.openDir 朝开口方向测量。普通区域的明显开放
    // 线框属于硬错误；显式标记的腔—通槽联合区域会在出口添加虚拟封口，
    // 并在 Loop2d::virtualOpenBoundary 中保留该语义。
    bool slice(const PocketRegion& region,
               double depthOffset,
               PocketLayerSlice& result,
               std::string& error,
               PocketMaterialSectionCache* materialCache =
                   nullptr,
               const ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::toolpath
