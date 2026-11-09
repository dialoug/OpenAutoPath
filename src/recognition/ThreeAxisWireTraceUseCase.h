// 三轴通结构/内外环识别用例。同步执行纯领域计算，不创建线程、Qt 控件或 AIS。
#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

#include "analysis/FeatureTypes.h"
#include "recognition/AxisWireTraceTracker.h"
#include "recognition/RecognitionGeometryCache.h"

namespace oap {

class ModelIndex;
class StepIndexer;
struct Model;

// 执行控制回调：isCancellationRequested 轮询取消，reportProgress 上报进度。
// 两者均为可空回调，缺省时相关调用直接跳过。
struct ThreeAxisWireTraceControl {
    std::function<bool()> isCancellationRequested;
    std::function<void(std::size_t completed, std::size_t total)>
        reportProgress;
    // Synchronous, joined range execution. Empty means serial recognition.
    RecognitionRangeExecutor executeRange;

    bool cancelled() const
    {
        return isCancellationRequested && isCancellationRequested();
    }

    void progress(std::size_t completed, std::size_t total) const
    {
        if (reportProgress)
            reportProgress(completed, total);
    }
};

// 一次三轴识别的输入：模型/索引/拓扑映射指针加已识别孔面集合。
// 指针由调用方持有且需在 execute() 返回前保持有效；recognizedHoleFaceIds
// 中的面在三轴扫描中被排除，避免与孔识别结果重叠。
struct ThreeAxisWireTraceRequest {
    const Model* model = nullptr;
    const ModelIndex* modelIndex = nullptr;
    const StepIndexer* stepIndexer = nullptr;
    std::unordered_set<std::string> recognizedHoleFaceIds;
};

// 单个轴向的累计统计与命中面集合，供 UI 展示与调试。
struct ThreeAxisTraceStats {
    std::string axisLabel;
    int rootFaceCount = 0;
    int activePathCount = 0;
    int blindSlotPathCount = 0;
    int blindPocketPathCount = 0;
    int terminalFaceCount = 0;
    int skippedInvalidFaceCount = 0;
    std::unordered_set<std::string> planeFaceIds;
    std::unordered_set<std::string> traceFaceIds;
};

// UI 面板可直接消费的中立分组；不含颜色、控件或场景状态。
struct ThreeAxisFeatureSubset {
    std::string axisLabel;
    std::string rootFaceId;
    std::string rootWireId;
    bool fromInnerWire = true;
    bool rootIsMainOuterFace = false;
    AxisWireTraceFeatureKind featureKind =
        AxisWireTraceFeatureKind::BlindPocket;
    std::vector<std::string> faceIds;
    bool rootCanConnectBlindPockets = false;
    std::vector<std::string> terminalFaceIds;
    std::size_t mergedSourceCount = 1;
};

// recognition 不依赖 toolpath；应用层把该值映射为 RootInnerWirePocketSource。
struct ThreeAxisPocketSource {
    FeatureKind kind = FeatureKind::Pocket;
    std::vector<std::string> sourceFaceIds;
    std::string rootFaceId;
    std::string rootWireId;
    bool allowOpenSectionContours = false;
    int openBoundaryFaceCount = 0;
};

// 自动外轮廓加工源。根面的 outer wire 定义零件外边界，sourceFaceIds
// 保存沿加工轴追踪到的侧壁/终端面，供应用层构建 ProfileRegion。
struct ThreeAxisProfileSource {
    std::string unitId;
    std::string axisLabel;
    int normalDirection = 1;
    std::string rootFaceId;
    std::string rootWireId;
    std::vector<std::string> sourceFaceIds;
    std::vector<std::string> machiningFaceIds;
    std::vector<std::string> terminalFaceIds;
    // 同一实体的其它基础主外表面只用于闭合块体，不属于当前根外环上的
    // 局部轮廓特征；下游计算加工深度时必须排除。
    std::vector<std::string> mainOuterFaceIds;
};

// execute() 的汇总结果：三个轴向的统计、合并后的特征子集与盲腔/通结构
// 增强后的孔源；failedUnitCount 记录单 Unit 失败的次数，ok() 反映整体成败。
struct ThreeAxisWireTraceUseCaseResult {
    std::unordered_set<std::string> combinedTraceFaceIds;
    std::unordered_set<std::string> throughStructureFaceIds;
    std::unordered_set<std::string> axisParallelPlaneFaceIds;
    std::array<ThreeAxisTraceStats, 3> axisStats;
    std::vector<ThreeAxisFeatureSubset> featureSubsets;
    std::vector<ThreeAxisPocketSource> pocketSources;
    std::vector<ThreeAxisProfileSource> profileSources;

    int failedUnitCount = 0;
    int verticalSurfaceCount = 0;
    int excludedHoleFaceCount = 0;
    int excludedMainOuterFaceCount = 0;
    int rejectedWithoutNonZNeighborCount = 0;
    int rejectedNonConvexCount = 0;
    int overwrittenFaceCount = 0;
    int absorbedBlindPocketSubsetCount = 0;
    bool cancelled = false;
    std::string error;

    bool ok() const noexcept
    {
        return !cancelled && error.empty();
    }
};

// 三轴识别用例：在 oap_analysis 的分析结果之上同步执行 Z/Y/X 三个轴向的
// 通结构识别、平行面筛选与内外环追踪，产出供 UI 直接消费的特征子集。
class ThreeAxisWireTraceUseCase {
public:
    // 用例唯一公开入口，同步执行。
    // 前置条件：request.model/modelIndex/stepIndexer 非空且模型已完成分析。
    ThreeAxisWireTraceUseCaseResult execute(
        const ThreeAxisWireTraceRequest& request,
        const ThreeAxisWireTraceControl* control = nullptr) const;
};

} // namespace oap
