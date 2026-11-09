// OCCT BRep 布尔差实现的在制毛坯状态；失败时保持上一个有效快照不变。
#include "toolpath/core/simulation/InProcessStock.h"
#include "toolpath/core/ToolpathOcctParallel.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

namespace oap::toolpath {
namespace {

using SteadyClock = std::chrono::steady_clock;
constexpr int kMaximumBooleanToolsPerBatch = 512;
constexpr int kDirectBooleanToolThreshold = 64;
constexpr std::size_t kExactSpatialBatchingToolLimit = 4096u;
constexpr int kIntermediateValidationBatchInterval = 32;
constexpr std::size_t kMinimumParallelValidationCount = 4u;

struct BooleanTool {
    TopoDS_Shape shape;
    Bnd_Box bounds;
    std::array<double, 3> minimum{};
    std::array<double, 3> maximum{};
    bool hasFiniteBounds = false;
};

struct BooleanBatch {
    TopTools_ListOfShape tools;
    std::vector<std::size_t> toolIndices;
};

class CancellationProgressIndicator final
    : public Message_ProgressIndicator {
public:
    explicit CancellationProgressIndicator(
        std::function<bool()> cancellationRequested)
        : m_cancellationRequested(
              std::move(cancellationRequested))
    {
    }

protected:
    Standard_Boolean UserBreak() override
    {
        try {
            return m_cancellationRequested &&
                   m_cancellationRequested();
        } catch (...) {
            return Standard_True;
        }
    }

    void Show(const Message_ProgressScope&,
              const Standard_Boolean) override
    {
    }

private:
    std::function<bool()> m_cancellationRequested;
};

void buildCut(BRepAlgoAPI_Cut& cut,
              const std::function<bool()>& cancellationRequested)
{
    if (!cancellationRequested) {
        cut.Build();
        return;
    }
    const Handle(Message_ProgressIndicator) progress =
        new CancellationProgressIndicator(cancellationRequested);
    cut.Build(progress->Start());
}

double elapsedMilliseconds(const SteadyClock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               SteadyClock::now() - start)
        .count();
}

bool shapeVolume(const TopoDS_Shape& shape, double& volume)
{
    if (shape.IsNull()) {
        volume = 0.0;
        return true;
    }
    try {
        GProp_GProps properties;
        BRepGProp::VolumeProperties(shape, properties);
        volume = std::abs(properties.Mass());
        return std::isfinite(volume);
    } catch (const Standard_Failure&) {
        return false;
    } catch (...) {
        return false;
    }
}

bool validShape(const TopoDS_Shape& shape, bool runParallel)
{
    if (shape.IsNull())
        return true; // 空形状表示毛坯被完全去除。
    try {
        ToolpathOcctParallelLease lease(runParallel);
        return BRepCheck_Analyzer(
                   shape, Standard_True, lease.enabled())
            .IsValid();
    } catch (const Standard_Failure&) {
        return false;
    } catch (...) {
        return false;
    }
}

// 把每个独立实体并行做拓扑有效性/体积校验，全部通过才返回；
// 任一无效应拒绝整个减料请求。外层并行，内层关闭并行避免线程池嵌套。
bool validIndependentSolids(
    const TopoDS_Shape& shape,
    bool runParallel,
    double minimumVolume,
    double& totalVolume)
{
    totalVolume = 0.0;
    std::vector<TopoDS_Shape> solids;
    try {
        for (TopExp_Explorer explorer(
                 shape, TopAbs_SOLID);
             explorer.More();
             explorer.Next()) {
            solids.push_back(explorer.Current());
        }
    } catch (const Standard_Failure&) {
        return false;
    } catch (...) {
        return false;
    }
    if (solids.empty())
        return false;

    std::vector<double> volumes(solids.size(), 0.0);
    std::vector<unsigned char> valid(solids.size(), 0u);
    const auto validateSolid = [&](std::size_t index) {
        try {
            double solidVolume = 0.0;
            // 独立实体在外层并行；关闭单实体内部并行，避免线程池嵌套。
            if (validShape(solids[index], false) &&
                shapeVolume(
                    solids[index], solidVolume) &&
                solidVolume > minimumVolume) {
                volumes[index] = solidVolume;
                valid[index] = 1u;
            }
        } catch (...) {
            valid[index] = 0u;
        }
    };
    if (runParallel &&
        solids.size() >=
            kMinimumParallelValidationCount) {
        toolpathParallelForBudgeted(solids.size(), validateSolid);
    } else {
        for (std::size_t index = 0;
             index < solids.size(); ++index) {
            validateSolid(index);
        }
    }
    if (std::find(
            valid.begin(), valid.end(), 0u) !=
        valid.end()) {
        return false;
    }
    totalVolume = std::accumulate(
        volumes.begin(), volumes.end(), 0.0);
    return std::isfinite(totalVolume) &&
           totalVolume > minimumVolume;
}

BooleanTool makeBooleanTool(const TopoDS_Shape& shape,
                            double tolerance)
{
    BooleanTool tool;
    tool.shape = shape;
    try {
        BRepBndLib::AddOptimal(
            shape, tool.bounds,
            Standard_False, Standard_False);
        if (!tool.bounds.IsVoid()) {
            tool.bounds.SetGap(tolerance);
            Standard_Real xmin = 0.0;
            Standard_Real ymin = 0.0;
            Standard_Real zmin = 0.0;
            Standard_Real xmax = 0.0;
            Standard_Real ymax = 0.0;
            Standard_Real zmax = 0.0;
            tool.bounds.Get(
                xmin, ymin, zmin, xmax, ymax, zmax);
            tool.minimum = {xmin, ymin, zmin};
            tool.maximum = {xmax, ymax, zmax};
            tool.hasFiniteBounds =
                std::isfinite(xmin) &&
                std::isfinite(ymin) &&
                std::isfinite(zmin) &&
                std::isfinite(xmax) &&
                std::isfinite(ymax) &&
                std::isfinite(zmax);
        }
    } catch (const Standard_Failure&) {
        tool.bounds.SetWhole();
    } catch (...) {
        tool.bounds.SetWhole();
    }
    return tool;
}

// 计算一组工具在某坐标轴上的最大同时重叠数（区间着色），
// 用于超大集合选择重叠最少的轴做分批。
int intervalOverlapPeak(const std::vector<BooleanTool>& tools,
                        int axis,
                        double tolerance)
{
    std::vector<std::size_t> order;
    order.reserve(tools.size());
    for (std::size_t index = 0;
         index < tools.size(); ++index) {
        if (tools[index].hasFiniteBounds)
            order.push_back(index);
    }
    std::stable_sort(
        order.begin(), order.end(),
        [&tools, axis](std::size_t first,
                       std::size_t second) {
            return tools[first].minimum[axis] <
                   tools[second].minimum[axis];
        });

    std::priority_queue<
        double, std::vector<double>,
        std::greater<double>> activeEnds;
    int peak = 0;
    for (const std::size_t index : order) {
        const double start =
            tools[index].minimum[axis];
        while (!activeEnds.empty() &&
               activeEnds.top() <=
                   start + 2.0 * tolerance) {
            activeEnds.pop();
        }
        activeEnds.push(tools[index].maximum[axis]);
        peak = std::max(
            peak,
            static_cast<int>(activeEnds.size()));
    }
    return peak;
}

bool boundsHaveInteriorOverlap(
    const BooleanTool& first,
    const BooleanTool& second,
    double tolerance)
{
    if (!first.hasFiniteBounds ||
        !second.hasFiniteBounds) {
        return true;
    }
    for (int axis = 0; axis < 3; ++axis) {
        // makeBooleanTool() 已经向两端各加入 tolerance；这里补回
        // 2*tolerance，使只共享边界面或边的实体仍可进入同一批。
        if (first.maximum[axis] <=
                second.minimum[axis] +
                    2.0 * tolerance ||
            second.maximum[axis] <=
                first.minimum[axis] +
                    2.0 * tolerance) {
            return false;
        }
    }
    return true;
}

std::vector<BooleanBatch> exactSpatialBooleanBatches(
    const std::vector<BooleanTool>& tools,
    double tolerance)
{
    std::vector<BooleanBatch> batches;
    batches.reserve(
        (tools.size() +
         kMaximumBooleanToolsPerBatch - 1u) /
        kMaximumBooleanToolsPerBatch);
    for (std::size_t toolIndex = 0;
         toolIndex < tools.size(); ++toolIndex) {
        std::size_t selectedBatch = batches.size();
        if (tools[toolIndex].hasFiniteBounds) {
            for (std::size_t batchIndex = 0;
                 batchIndex < batches.size();
                 ++batchIndex) {
                const BooleanBatch& batch =
                    batches[batchIndex];
                if (batch.tools.Extent() >=
                    kMaximumBooleanToolsPerBatch) {
                    continue;
                }
                const bool conflicts =
                    std::any_of(
                        batch.toolIndices.begin(),
                        batch.toolIndices.end(),
                        [&](std::size_t existingIndex) {
                            return boundsHaveInteriorOverlap(
                                tools[toolIndex],
                                tools[existingIndex],
                                tolerance);
                        });
                if (!conflicts) {
                    selectedBatch = batchIndex;
                    break;
                }
            }
        }
        if (selectedBatch == batches.size())
            batches.emplace_back();
        BooleanBatch& batch = batches[selectedBatch];
        batch.tools.Append(tools[toolIndex].shape);
        batch.toolIndices.push_back(toolIndex);
    }
    return batches;
}

std::vector<BooleanBatch> spatialBooleanBatches(
    const std::vector<BooleanTool>& boundedTools,
    double tolerance)
{
    // 固定轴分层构建器已经在每个轴向区间内对二维覆盖区域做过并集，
    // 常见的数百个输出实体在三维中互不重叠，但它们在任意单一坐标轴上
    // 仍可能有大量投影重叠。对这类中等规模集合做完整三维包围盒冲突
    // 检查，可把确认不相交的实体放进同一个 OCCT 多工具 Cut，显著减少
    // 必须顺序更新在制毛坯的布尔批次数。超大集合仍使用 O(n log n)
    // 单轴规划，避免二次复杂度失控。
    if (boundedTools.size() <=
        kExactSpatialBatchingToolLimit) {
        return exactSpatialBooleanBatches(
            boundedTools, tolerance);
    }

    std::array<int, 3> overlapPeaks{
        intervalOverlapPeak(
            boundedTools, 0, tolerance),
        intervalOverlapPeak(
            boundedTools, 1, tolerance),
        intervalOverlapPeak(
            boundedTools, 2, tolerance)};
    const int axis = static_cast<int>(
        std::distance(
            overlapPeaks.begin(),
            std::min_element(
                overlapPeaks.begin(),
                overlapPeaks.end())));

    std::vector<std::size_t> order(
        boundedTools.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(
        order.begin(), order.end(),
        [&boundedTools, axis](
            std::size_t first, std::size_t second) {
            const BooleanTool& firstTool =
                boundedTools[first];
            const BooleanTool& secondTool =
                boundedTools[second];
            if (firstTool.hasFiniteBounds !=
                secondTool.hasFiniteBounds) {
                return firstTool.hasFiniteBounds;
            }
            if (!firstTool.hasFiniteBounds)
                return first < second;
            return firstTool.minimum[axis] <
                   secondTool.minimum[axis];
        });

    std::vector<BooleanBatch> batches;
    using ActiveBatch =
        std::pair<double, std::size_t>;
    std::priority_queue<
        ActiveBatch,
        std::vector<ActiveBatch>,
        std::greater<ActiveBatch>> activeBatches;
    for (const std::size_t toolIndex : order) {
        const BooleanTool& tool =
            boundedTools[toolIndex];
        if (!tool.hasFiniteBounds) {
            batches.emplace_back();
            batches.back().tools.Append(tool.shape);
            batches.back().toolIndices.push_back(toolIndex);
            continue;
        }

        while (!activeBatches.empty() &&
               batches[activeBatches.top().second]
                       .tools.Extent() >=
                   kMaximumBooleanToolsPerBatch) {
            activeBatches.pop();
        }

        std::size_t batchIndex = batches.size();
        if (!activeBatches.empty() &&
            activeBatches.top().first <=
                tool.minimum[axis] +
                    2.0 * tolerance) {
            batchIndex =
                activeBatches.top().second;
            activeBatches.pop();
        } else {
            batches.emplace_back();
        }

        BooleanBatch& batch = batches[batchIndex];
        batch.tools.Append(tool.shape);
        batch.toolIndices.push_back(toolIndex);
        if (batch.tools.Extent() <
            kMaximumBooleanToolsPerBatch) {
            activeBatches.emplace(
                tool.maximum[axis], batchIndex);
        }
    }
    return batches;
}

// 核心减法：把 removal 从 stock 中减去，逐批执行并做体积守恒校验。
// independentSolids=true 时把 removal 展开为独立实体逐个批量相减；
// 只有全部批次成功才提交结果，中途失败保持原毛坯。
TopoDS_Shape subtractShape(const TopoDS_Shape& stock,
                           const TopoDS_Shape& removal,
                           double tolerance,
                           bool independentSolids,
                           bool runParallel,
                           const std::function<bool()>& cancellationRequested,
                           int& batchCount,
                           bool& cancelled,
                           bool& done)
{
    batchCount = 0;
    cancelled = cancellationRequested &&
                cancellationRequested();
    if (cancelled) {
        done = false;
        return {};
    }
    if (!independentSolids) {
        ToolpathOcctParallelLease lease(runParallel);
        // The two-shape constructor already builds. Configure an empty builder
        // first so tolerance, admission and cancellation govern the only build.
        BRepAlgoAPI_Cut cut;
        TopTools_ListOfShape arguments, tools;
        arguments.Append(stock);
        tools.Append(removal);
        cut.SetArguments(arguments);
        cut.SetTools(tools);
        cut.SetFuzzyValue(tolerance);
        cut.SetRunParallel(lease.enabled());
        buildCut(cut, cancellationRequested);
        batchCount = 1;
        cancelled = cancellationRequested &&
                    cancellationRequested();
        if (cancelled) {
            done = false;
            return {};
        }
        done = cut.IsDone();
        return done ? cut.Shape() : TopoDS_Shape{};
    }
    std::vector<BooleanTool> tools;
    if (removal.ShapeType() == TopAbs_SOLID) {
        tools.push_back(
            makeBooleanTool(removal, tolerance));
    } else {
        for (TopExp_Explorer explorer(removal, TopAbs_SOLID);
             explorer.More(); explorer.Next()) {
            tools.push_back(
                makeBooleanTool(
                    explorer.Current(), tolerance));
        }
    }
    if (tools.empty()) {
        done = false;
        return {};
    }

    // 包围盒没有体积交叠的工具不可能去除材料。固定轴后续工序中常有
    // 已被前序工序清空、或完全落在毛坯外的回退扫掠体；在进入 OCCT
    // PaveFiller 前剔除这些明确空切，既避免无意义的相交检查，也不会
    // 把仅接触毛坯边界的零体积接触误报为实际减料。
    const BooleanTool stockBounds =
        makeBooleanTool(stock, tolerance);
    if (stockBounds.hasFiniteBounds) {
        tools.erase(
            std::remove_if(
                tools.begin(), tools.end(),
                [&](const BooleanTool& tool) {
                    return tool.hasFiniteBounds &&
                           !boundsHaveInteriorOverlap(
                               stockBounds,
                               tool,
                               tolerance);
                }),
            tools.end());
    }
    if (tools.empty()) {
        done = true;
        return stock;
    }

    // 中小规模集合一次提交给 OCCT，避免把同一道工序拆成多次连续差集；
    // 大规模集合中的平面覆盖棱柱和斜坡回退扫掠可能大量互相重叠，若仍按
    // 512 个固定上限一次提交，OCCT 会先分割工具之间的交叠，复杂度远高于
    // 逐批切毛坯。
    // 选取包围盒重叠峰值最低的坐标轴并做区间着色；同批工具在该轴上的
    // 体积区间分离，允许相邻轴向层只共享边界面。包围盒已向两侧各扩张
    // tolerance，因此比较时补回 2*tolerance 不会把真实体积重叠的工具合批。
    // 该规划为 O(n log n)，可直接处理数万
    // 个回退实体，同时仍把空间分散的工具聚合到最多 512 个一批。
    // candidate 只在全部批次成功后提交，因此中途失败仍保持工序前毛坯。
    std::vector<BooleanBatch> batches;
    if (tools.size() <=
        kDirectBooleanToolThreshold) {
        batches.emplace_back();
        for (std::size_t toolIndex = 0;
             toolIndex < tools.size(); ++toolIndex) {
            batches.back().tools.Append(
                tools[toolIndex].shape);
            batches.back().toolIndices.push_back(
                toolIndex);
        }
    } else {
        batches =
            spatialBooleanBatches(
                tools, tolerance);
    }
    TopoDS_Shape candidate = stock;
    const auto retryBatchSequentially =
        [&](const TopoDS_Shape& input,
            const BooleanBatch& batch,
            TopoDS_Shape& output) {
            TopoDS_Shape retryCandidate = input;
            for (TopTools_ListIteratorOfListOfShape iterator(
                     batch.tools);
                 iterator.More(); iterator.Next()) {
                if (cancellationRequested &&
                    cancellationRequested()) {
                    cancelled = true;
                    return false;
                }
                double volumeBeforeRetry = 0.0;
                double retryToolVolume = 0.0;
                if (!shapeVolume(
                        retryCandidate, volumeBeforeRetry) ||
                    !shapeVolume(
                        iterator.Value(), retryToolVolume)) {
                    return false;
                }
                BRepAlgoAPI_Cut retryCut;
                TopTools_ListOfShape retryArguments, retryTools;
                retryArguments.Append(retryCandidate);
                retryTools.Append(iterator.Value());
                retryCut.SetArguments(retryArguments);
                retryCut.SetTools(retryTools);
                retryCut.SetFuzzyValue(tolerance);
                // 重试是多工具操作失败后的稳定路径。保持单线程运行,
                // 使回退不会重复同样的 OCCT 调度失败。
                retryCut.SetRunParallel(false);
                ++batchCount;
                buildCut(retryCut, cancellationRequested);
                if (!retryCut.IsDone())
                    return false;
                retryCandidate = retryCut.Shape();
                double volumeAfterRetry = 0.0;
                const double conservationTolerance = std::max(
                    tolerance * tolerance * tolerance,
                    1.0e-9 * std::max(
                        1.0, volumeBeforeRetry));
                if (retryCandidate.IsNull() ||
                    !validShape(retryCandidate, false) ||
                    !shapeVolume(
                        retryCandidate, volumeAfterRetry) ||
                    volumeBeforeRetry - volumeAfterRetry >
                        retryToolVolume + conservationTolerance) {
                    return false;
                }
            }
            output = retryCandidate;
            return true;
        };
    for (const BooleanBatch& batch : batches) {
        cancelled = cancellationRequested &&
                    cancellationRequested();
        if (cancelled) {
            done = false;
            return {};
        }

        const TopoDS_Shape batchInput = candidate;
        TopTools_ListOfShape arguments;
        arguments.Append(candidate);
        ToolpathOcctParallelLease lease(runParallel);
        BRepAlgoAPI_Cut cut;
        cut.SetArguments(arguments);
        cut.SetTools(batch.tools);
        cut.SetFuzzyValue(tolerance);
        cut.SetRunParallel(lease.enabled());
        ++batchCount;
        buildCut(cut, cancellationRequested);
        cancelled = cancellationRequested &&
                    cancellationRequested();
        if (cancelled) {
            done = false;
            return {};
        }
        if (!cut.IsDone()) {
            if (!retryBatchSequentially(
                    batchInput, batch, candidate)) {
                done = false;
                return {};
            }
        } else {
            candidate = cut.Shape();
        }
        if (candidate.IsNull()) {
            // 多批相减中途出现空形状更常见于上一个无效中间拓扑被
            // OCCT 塌缩，不能把它当作“毛坯确已完全去除”提交。
            done = batches.size() == 1;
            if (!retryBatchSequentially(
                    batchInput, batch, candidate) ||
                candidate.IsNull()) {
                done = false;
                return {};
            }
        }
        double candidateVolume = 0.0;
        const double minimumCandidateVolume =
            tolerance * tolerance * tolerance;
        if (!shapeVolume(candidate, candidateVolume) ||
            candidateVolume <= minimumCandidateVolume) {
            if (!retryBatchSequentially(
                    batchInput, batch, candidate) ||
                candidate.IsNull() ||
                !shapeVolume(candidate, candidateVolume) ||
                candidateVolume <= minimumCandidateVolume) {
                done = false;
                return {};
            }
        }
        double batchInputVolume = 0.0;
        double batchToolVolume = 0.0;
        bool measuredBatchTools =
            shapeVolume(batchInput, batchInputVolume);
        for (TopTools_ListIteratorOfListOfShape iterator(
                 batch.tools);
             measuredBatchTools && iterator.More();
             iterator.Next()) {
            double toolVolume = 0.0;
            measuredBatchTools =
                shapeVolume(iterator.Value(), toolVolume);
            batchToolVolume += toolVolume;
        }
        const double conservationTolerance = std::max(
            tolerance * tolerance * tolerance,
            1.0e-9 * std::max(1.0, batchInputVolume));
        if (!measuredBatchTools ||
            batchInputVolume - candidateVolume >
                batchToolVolume + conservationTolerance) {
            if (!retryBatchSequentially(
                    batchInput, batch, candidate) ||
                candidate.IsNull() ||
                !shapeVolume(candidate, candidateVolume) ||
                batchInputVolume - candidateVolume >
                    batchToolVolume + conservationTolerance) {
                done = false;
                return {};
            }
        }
        // 连续小批次布尔可能返回 IsDone() 但产生已损坏的中间拓扑。
        // 每 32 批及末批检查一次，在及时阻断无效形状传播的同时，避免
        // 数千个斜坡批次逐批执行昂贵的全拓扑校验。
        const bool validateIntermediate =
            (batchCount %
                 kIntermediateValidationBatchInterval) == 0 ||
            batchCount ==
                static_cast<int>(batches.size());
        if (validateIntermediate &&
            !validShape(candidate, runParallel)) {
            if (!retryBatchSequentially(
                    batchInput, batch, candidate)) {
                done = false;
                return {};
            }
        }
    }
    done = true;
    return candidate;
}

} // namespace

const char* stockRemovalStatusName(StockRemovalStatus status) noexcept
{
    switch (status) {
        case StockRemovalStatus::Applied: return "applied";
        case StockRemovalStatus::NoMaterialRemoved: return "no_material_removed";
        case StockRemovalStatus::InvalidStock: return "invalid_stock";
        case StockRemovalStatus::InvalidRemovalVolume: return "invalid_removal_volume";
        case StockRemovalStatus::BooleanFailed: return "boolean_failed";
        case StockRemovalStatus::InvalidResult: return "invalid_result";
        case StockRemovalStatus::Cancelled: return "cancelled";
    }
    return "unknown";
}

InProcessStock::InProcessStock(const TopoDS_Shape& initialStock,
                               double geometricTolerance,
                               bool runParallel,
                               std::function<bool()> cancellationRequested)
    : m_initial(initialStock),
      m_current(initialStock),
      m_runParallel(runParallel),
      m_cancellationRequested(
          std::move(cancellationRequested))
{
    m_tolerance = std::isfinite(geometricTolerance)
        ? std::max(geometricTolerance, Precision::Confusion())
        : std::max(1.0e-5, Precision::Confusion());
    m_valid = !initialStock.IsNull() &&
              validShape(initialStock, m_runParallel) &&
              shapeVolume(initialStock, m_initialVolume) &&
              m_initialVolume > m_tolerance * m_tolerance * m_tolerance;
    m_currentVolume = m_initialVolume;
}

StockRemovalRecord InProcessStock::applyRemoval(
    const TopoDS_Shape& removalVolume,
    std::string operationId,
    int sequenceIndex,
    StockRemovalPhaseCallback phaseChanged)
{
    return applyRemovalImpl(
        removalVolume, std::move(operationId),
        sequenceIndex, false, std::move(phaseChanged));
}

StockRemovalRecord InProcessStock::applyRemovalWithIndependentSolids(
    const TopoDS_Shape& removalVolume,
    std::string operationId,
    int sequenceIndex,
    StockRemovalPhaseCallback phaseChanged)
{
    return applyRemovalImpl(
        removalVolume, std::move(operationId),
        sequenceIndex, true, std::move(phaseChanged));
}

StockRemovalRecord InProcessStock::applyRemovalImpl(
    const TopoDS_Shape& removalVolume,
    std::string operationId,
    int sequenceIndex,
    bool independentSolids,
    StockRemovalPhaseCallback phaseChanged)
{
    StockRemovalRecord record;
    record.operationId = std::move(operationId);
    record.sequenceIndex = sequenceIndex;
    record.volumeBefore = m_currentVolume;
    record.volumeAfter = m_currentVolume;
    record.stockAfter = m_current;

    if (m_cancellationRequested &&
        m_cancellationRequested()) {
        record.status = StockRemovalStatus::Cancelled;
        record.message = "毛坯布尔减料已取消";
        m_history.push_back(record);
        return record;
    }
    if (!m_valid) {
        record.status = StockRemovalStatus::InvalidStock;
        record.message = "在制毛坯不是有效的正体积实体";
        m_history.push_back(record);
        return record;
    }
    double removalMeasure = 0.0;
    if (phaseChanged) {
        phaseChanged(
            StockRemovalPhase::ValidatingRemovalVolume);
    }
    const SteadyClock::time_point inputValidationStart =
        SteadyClock::now();
    const double minimumVolume =
        m_tolerance * m_tolerance * m_tolerance;
    const bool removalValid =
        !removalVolume.IsNull() &&
        (independentSolids
             ? validIndependentSolids(
                   removalVolume,
                   m_runParallel,
                   minimumVolume,
                   removalMeasure)
             : validShape(
                   removalVolume,
                   m_runParallel) &&
                   shapeVolume(
                       removalVolume,
                       removalMeasure) &&
                   removalMeasure >
                       minimumVolume);
    record.inputValidationMilliseconds =
        elapsedMilliseconds(inputValidationStart);
    if (!removalValid) {
        record.status = StockRemovalStatus::InvalidRemovalVolume;
        record.message = "切削扫掠体不是有效的正体积形体";
        m_history.push_back(record);
        return record;
    }

    if (phaseChanged) {
        phaseChanged(
            StockRemovalPhase::ApplyingBooleanRemoval);
    }
    const SteadyClock::time_point booleanStart =
        SteadyClock::now();
    try {
        bool cutDone = false;
        bool cutCancelled = false;
        const TopoDS_Shape candidate = subtractShape(
            m_current, removalVolume, m_tolerance,
            independentSolids, m_runParallel,
            m_cancellationRequested,
            record.booleanBatchCount,
            cutCancelled,
            cutDone);
        record.booleanMilliseconds =
            elapsedMilliseconds(booleanStart);
        if (cutCancelled) {
            record.status = StockRemovalStatus::Cancelled;
            record.message = "毛坯布尔减料已取消";
            m_history.push_back(record);
            return record;
        }
        if (!cutDone) {
            record.status = StockRemovalStatus::BooleanFailed;
            record.message = "OCCT 毛坯布尔减料未能完成";
            m_history.push_back(record);
            return record;
        }
        double candidateVolume = 0.0;
        if (phaseChanged) {
            phaseChanged(
                StockRemovalPhase::ValidatingRemovalResult);
        }
        const SteadyClock::time_point resultValidationStart =
            SteadyClock::now();
        const bool candidateValid =
            validShape(candidate, m_runParallel) &&
            shapeVolume(candidate, candidateVolume);
        record.resultValidationMilliseconds =
            elapsedMilliseconds(resultValidationStart);
        if (!candidateValid) {
            record.status = StockRemovalStatus::InvalidResult;
            record.message = "毛坯布尔减料生成了无效形体";
            m_history.push_back(record);
            return record;
        }
        const double volumeTolerance = std::max(
            m_tolerance * m_tolerance * m_tolerance,
            1.0e-9 * std::max(1.0, m_currentVolume));
        if (candidateVolume > m_currentVolume + volumeTolerance) {
            record.status = StockRemovalStatus::InvalidResult;
            record.message = "布尔减料后的毛坯体积异常增大";
            m_history.push_back(record);
            return record;
        }
        const double removed = std::max(0.0, m_currentVolume - candidateVolume);
        if (removed <= volumeTolerance) {
            record.status = StockRemovalStatus::NoMaterialRemoved;
            record.message = "切削扫掠体未与当前毛坯相交";
            m_history.push_back(record);
            return record;
        }

        m_current = candidate;
        m_currentVolume = candidateVolume;
        record.status = StockRemovalStatus::Applied;
        record.volumeAfter = candidateVolume;
        record.removedVolume = removed;
        record.message = "毛坯减料已应用";
        record.stockAfter = m_current;
        m_history.push_back(record);
        return record;
    } catch (const Standard_Failure&) {
        record.booleanMilliseconds =
            elapsedMilliseconds(booleanStart);
        if (m_cancellationRequested &&
            m_cancellationRequested()) {
            record.status = StockRemovalStatus::Cancelled;
            record.message = "毛坯布尔减料已取消";
        } else {
            record.status = StockRemovalStatus::BooleanFailed;
            record.message =
                "OCCT 在执行毛坯布尔减料时发生异常";
        }
    } catch (...) {
        record.booleanMilliseconds =
            elapsedMilliseconds(booleanStart);
        if (m_cancellationRequested &&
            m_cancellationRequested()) {
            record.status = StockRemovalStatus::Cancelled;
            record.message = "毛坯布尔减料已取消";
        } else {
            record.status = StockRemovalStatus::BooleanFailed;
            record.message =
                "执行毛坯布尔减料时发生未知异常";
        }
    }
    m_history.push_back(record);
    return record;
}

void InProcessStock::reset()
{
    m_current = m_initial;
    m_currentVolume = m_initialVolume;
    m_history.clear();
}

} // namespace oap::toolpath
