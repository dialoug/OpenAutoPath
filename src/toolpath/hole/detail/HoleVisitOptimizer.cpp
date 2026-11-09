// 孔访问顺序优化：最近邻初始化后依次执行 2-opt、重定位、候选 3-opt 和避碰修复。
// 属于 oap_toolpath_hole 模块 detail 私有实现，由 HoleMachining 在装夹分组后调用。
#include "toolpath/hole/detail/HoleVisitOptimizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "HoleRapidPlanner.h"

namespace oap::toolpath::detail {
namespace {

constexpr double kTolerance = 1.0e-7;
constexpr std::size_t kThreeOptCandidateCount = 20;
constexpr int kThreeOptMaxPasses = 12;
constexpr std::size_t kThreeOptFullSearchLimit = 200;
constexpr int kTwoOptMaxPasses = 32;
constexpr std::size_t kLocalOptimizationHoleLimit = 500;
constexpr std::size_t kDistanceMatrixHoleLimit = 2000;
constexpr std::size_t kRelocationFullSearchLimit = 200;
constexpr std::size_t kRelocationCandidateCount = 20;
constexpr int kRelocationMaxImprovements = 12;
constexpr std::size_t kRapidRepairCandidateCount = 20;
constexpr std::size_t kRapidRepairHoleLimit = 500;

gp_Pnt selectedCorner(const MoldCoordinateFrame& frame, HoleStartCorner corner)
{
    switch (corner) {
        case HoleStartCorner::LowerRight:
            return gp_Pnt(frame.length, 0.0, 0.0);
        case HoleStartCorner::UpperLeft:
            return gp_Pnt(0.0, frame.width, 0.0);
        case HoleStartCorner::UpperRight:
            return gp_Pnt(frame.length, frame.width, 0.0);
        case HoleStartCorner::LowerLeft:
        default:
            return gp_Pnt(0.0, 0.0, 0.0);
    }
}

bool cancellationRequested(const ToolpathGenerationControl* control)
{
    return control && control->cancelled();
}

} // namespace

std::vector<std::size_t> optimizeHoleVisitOrder(
    const std::vector<HoleVisitInput>& inputs,
    const MoldCoordinateFrame& moldFrame,
    HoleStartCorner startCorner,
    HoleMachiningResult& result,
    const ToolpathGenerationControl* control)
{
    const std::size_t count = inputs.size();
    std::vector<std::size_t> order;
    order.reserve(count);
    if (count == 0)
        return order;
    if (count == 1) {
        order.push_back(0);
        return order;
    }

    // 起点由用户选择的模具局部角点决定；相同距离用原始索引保证结果可复现。
    const gp_Pnt corner = selectedCorner(moldFrame, startCorner);
    std::size_t first = 0;
    double bestCornerDistance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < count; ++i) {
        if (cancellationRequested(control))
            return order;
        const gp_Pnt local = moldFrame.worldToLocal(inputs[i].entry);
        const double dx = local.X() - corner.X();
        const double dy = local.Y() - corner.Y();
        const double squared = dx * dx + dy * dy;
        if (squared < bestCornerDistance ||
            (squared == bestCornerDistance &&
             inputs[i].originalIndex < inputs[first].originalIndex)) {
            first = i;
            bestCornerDistance = squared;
        }
    }

    // 极端孔数下不构造 O(n^2) 矩阵。固定首孔后保持识别顺序，优先保证
    // 任务有界、可取消和可生成；常规孔组仍进入完整优化。
    if (count > kDistanceMatrixHoleLimit) {
        order.push_back(first);
        for (std::size_t index = 0; index < count; ++index) {
            if (cancellationRequested(control))
                return order;
            if (index != first)
                order.push_back(index);
        }
        double length = 0.0;
        for (std::size_t index = 1; index < order.size(); ++index) {
            length += inputs[order[index - 1]].clearance.Distance(
                inputs[order[index]].clearance);
        }
        result.nearestNeighborRouteLength += length;
        result.twoOptRouteLength += length;
        result.relocationRouteLength += length;
        result.optimizedRouteLength += length;
        result.warnings.push_back(
            "hole visit optimization preserved stable input order because the setup exceeds 2000 holes");
        return order;
    }

    // 逐行分配孔间代价，使大矩阵构造过程也可响应取消。
    std::vector<std::vector<double>> cost(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (cancellationRequested(control))
            return order;
        cost[i].resize(count, 0.0);
    }
    for (std::size_t i = 0; i < count; ++i) {
        if (cancellationRequested(control))
            return order;
        for (std::size_t j = i + 1; j < count; ++j) {
            const double value = inputs[i].clearance.Distance(inputs[j].clearance);
            cost[i][j] = cost[j][i] = value;
        }
    }

    // 最近邻提供低成本初解，后面的局部优化都保持首孔不变。
    std::vector<bool> used(count, false);
    order.push_back(first);
    used[first] = true;
    while (order.size() < count) {
        if (cancellationRequested(control))
            return order;
        const std::size_t current = order.back();
        std::size_t next = count;
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t candidate = 0; candidate < count; ++candidate) {
            if (used[candidate])
                continue;
            if (cost[current][candidate] < best ||
                (cost[current][candidate] == best &&
                 (next == count || inputs[candidate].originalIndex <
                                   inputs[next].originalIndex))) {
                next = candidate;
                best = cost[current][candidate];
            }
        }
        used[next] = true;
        order.push_back(next);
    }

    auto orderedLength = [&](const std::vector<std::size_t>& route) {
        double length = 0.0;
        for (std::size_t i = 1; i < route.size(); ++i)
            length += cost[route[i - 1]][route[i]];
        return length;
    };
    result.nearestNeighborRouteLength += orderedLength(order);

    if (count > kLocalOptimizationHoleLimit) {
        const double length = orderedLength(order);
        result.twoOptRouteLength += length;
        result.relocationRouteLength += length;
        result.optimizedRouteLength += length;
        result.warnings.push_back(
            "hole visit optimization used bounded nearest-neighbor routing because the setup exceeds 500 holes");
        return order;
    }

    // 候选边限制大规模重定位和 3-opt 的搜索空间，同时保留小规模全搜索效果。
    std::vector<std::vector<unsigned char>> candidateEdge(
        count, std::vector<unsigned char>(count, 0));
    const std::size_t candidateCount =
        std::min(std::max(kThreeOptCandidateCount,
                          kRelocationCandidateCount),
                 count - 1);
    for (std::size_t node = 0; node < count; ++node) {
        if (cancellationRequested(control))
            return order;
        std::vector<std::size_t> neighbors;
        neighbors.reserve(count - 1);
        for (std::size_t other = 0; other < count; ++other)
            if (other != node)
                neighbors.push_back(other);
        std::stable_sort(neighbors.begin(), neighbors.end(),
                         [&](std::size_t a, std::size_t b) {
            if (cost[node][a] != cost[node][b])
                return cost[node][a] < cost[node][b];
            return inputs[a].originalIndex < inputs[b].originalIndex;
        });
        for (std::size_t rank = 0; rank < candidateCount; ++rank) {
            candidateEdge[node][neighbors[rank]] = 1;
            candidateEdge[neighbors[rank]][node] = 1;
        }
    }

    // 2-opt 消除明显交叉；每次接受首个改进，直至达到局部最优。
    auto runTwoOpt = [&]() {
        bool anyImprovement = false;
        bool improved = true;
        int pass = 0;
        while (improved && pass++ < kTwoOptMaxPasses &&
               !cancellationRequested(control)) {
            improved = false;
            for (std::size_t begin = 1; begin + 1 < count && !improved; ++begin) {
                for (std::size_t end = begin + 1; end < count; ++end) {
                    if (cancellationRequested(control))
                        return anyImprovement;
                    double oldBoundary = cost[order[begin - 1]][order[begin]];
                    double newBoundary = cost[order[begin - 1]][order[end]];
                    if (end + 1 < count) {
                        oldBoundary += cost[order[end]][order[end + 1]];
                        newBoundary += cost[order[begin]][order[end + 1]];
                    }
                    if (newBoundary + kTolerance < oldBoundary) {
                        std::reverse(order.begin() + static_cast<std::ptrdiff_t>(begin),
                                     order.begin() + static_cast<std::ptrdiff_t>(end + 1));
                        ++result.twoOptImprovementCount;
                        improved = true;
                        anyImprovement = true;
                        break;
                    }
                }
            }
        }
        return anyImprovement;
    };

    // 单点重定位可修复 2-opt 无法处理的“孔位插错区段”情况。
    auto runRelocation = [&]() {
        bool anyImprovement = false;
        for (int pass = 0; pass < kRelocationMaxImprovements; ++pass) {
            if (cancellationRequested(control))
                return anyImprovement;
            const double currentLength = orderedLength(order);
            double bestLength = currentLength;
            std::vector<std::size_t> bestOrder;
            std::size_t bestOriginalIndex = std::numeric_limits<std::size_t>::max();
            std::size_t bestInsertPosition = std::numeric_limits<std::size_t>::max();

            for (std::size_t from = 1; from < count; ++from) {
                const std::size_t node = order[from];
                std::vector<std::size_t> without = order;
                without.erase(without.begin() + static_cast<std::ptrdiff_t>(from));
                for (std::size_t insert = 1; insert <= without.size(); ++insert) {
                    if (cancellationRequested(control))
                        return anyImprovement;
                    if (insert == from)
                        continue;
                    if (count > kRelocationFullSearchLimit) {
                        const std::size_t previous = without[insert - 1];
                        const bool nearPrevious = candidateEdge[node][previous] != 0;
                        const bool nearNext = insert < without.size() &&
                            candidateEdge[node][without[insert]] != 0;
                        if (!nearPrevious && !nearNext)
                            continue;
                    }
                    std::vector<std::size_t> trial = without;
                    trial.insert(trial.begin() + static_cast<std::ptrdiff_t>(insert), node);
                    const double trialLength = orderedLength(trial);
                    const std::size_t originalIndex = inputs[node].originalIndex;
                    const bool shorter = trialLength + kTolerance < bestLength;
                    const bool stableTie =
                        std::abs(trialLength - bestLength) <= kTolerance &&
                        (originalIndex < bestOriginalIndex ||
                         (originalIndex == bestOriginalIndex &&
                          insert < bestInsertPosition));
                    if (shorter ||
                        (bestLength + kTolerance < currentLength && stableTie)) {
                        bestLength = trialLength;
                        bestOrder = std::move(trial);
                        bestOriginalIndex = originalIndex;
                        bestInsertPosition = insert;
                    }
                }
            }
            if (bestOrder.empty() || bestLength + kTolerance >= currentLength)
                break;
            order = std::move(bestOrder);
            ++result.relocationImprovementCount;
            anyImprovement = true;
        }
        return anyImprovement;
    };

    runTwoOpt();
    result.twoOptRouteLength += orderedLength(order);
    runRelocation();
    result.relocationRouteLength += orderedLength(order);
    runTwoOpt();

    // 候选 3-opt 同时替换三条边，用于跨越更深的局部最优。
    if (count >= 4 && count <= kThreeOptFullSearchLimit) {
        auto allCandidateEdges = [&](std::size_t a, std::size_t b,
                                     std::size_t c, std::size_t d,
                                     std::size_t e, std::size_t f) {
            return candidateEdge[a][b] && candidateEdge[c][d] && candidateEdge[e][f];
        };
        enum class ThreeOptPattern {
            ReverseBoth,
            Swap,
            ReverseSecondThenFirst,
            SecondThenReverseFirst
        };
        auto rebuildThreeOpt = [&](std::size_t firstCut,
                                   std::size_t secondCut,
                                   std::size_t thirdCut,
                                   ThreeOptPattern pattern) {
            std::vector<std::size_t> replacement;
            replacement.reserve(count);
            replacement.insert(replacement.end(), order.begin(),
                               order.begin() + static_cast<std::ptrdiff_t>(firstCut + 1));
            auto appendForward = [&](std::size_t begin, std::size_t end) {
                replacement.insert(
                    replacement.end(),
                    order.begin() + static_cast<std::ptrdiff_t>(begin),
                    order.begin() + static_cast<std::ptrdiff_t>(end + 1));
            };
            auto appendReverse = [&](std::size_t begin, std::size_t end) {
                for (std::size_t pos = end + 1; pos-- > begin;)
                    replacement.push_back(order[pos]);
            };
            const std::size_t firstBegin = firstCut + 1;
            const std::size_t secondBegin = secondCut + 1;
            switch (pattern) {
                case ThreeOptPattern::ReverseBoth:
                    appendReverse(firstBegin, secondCut);
                    appendReverse(secondBegin, thirdCut);
                    break;
                case ThreeOptPattern::Swap:
                    appendForward(secondBegin, thirdCut);
                    appendForward(firstBegin, secondCut);
                    break;
                case ThreeOptPattern::ReverseSecondThenFirst:
                    appendReverse(secondBegin, thirdCut);
                    appendForward(firstBegin, secondCut);
                    break;
                case ThreeOptPattern::SecondThenReverseFirst:
                    appendForward(secondBegin, thirdCut);
                    appendReverse(firstBegin, secondCut);
                    break;
            }
            replacement.insert(
                replacement.end(),
                order.begin() + static_cast<std::ptrdiff_t>(thirdCut + 1), order.end());
            order = std::move(replacement);
        };

        bool improved = true;
        int pass = 0;
        while (improved && pass++ < kThreeOptMaxPasses &&
               !cancellationRequested(control)) {
            improved = false;
            for (std::size_t i = 0; i + 3 < count && !improved; ++i) {
                for (std::size_t j = i + 1; j + 2 < count && !improved; ++j) {
                    for (std::size_t k = j + 1; k + 1 < count; ++k) {
                        if (cancellationRequested(control))
                            return order;
                        const std::size_t a = order[i];
                        const std::size_t b = order[i + 1];
                        const std::size_t c = order[j];
                        const std::size_t d = order[j + 1];
                        const std::size_t e = order[k];
                        const std::size_t f = order[k + 1];
                        const double oldBoundary = cost[a][b] + cost[c][d] + cost[e][f];
                        struct PatternCost {
                            ThreeOptPattern pattern;
                            double value;
                            bool candidates;
                        };
                        const std::array<PatternCost, 4> patterns{{
                            {ThreeOptPattern::ReverseBoth,
                             cost[a][c] + cost[b][e] + cost[d][f],
                             allCandidateEdges(a, c, b, e, d, f)},
                            {ThreeOptPattern::Swap,
                             cost[a][d] + cost[e][b] + cost[c][f],
                             allCandidateEdges(a, d, e, b, c, f)},
                            {ThreeOptPattern::ReverseSecondThenFirst,
                             cost[a][e] + cost[d][b] + cost[c][f],
                             allCandidateEdges(a, e, d, b, c, f)},
                            {ThreeOptPattern::SecondThenReverseFirst,
                             cost[a][d] + cost[e][c] + cost[b][f],
                             allCandidateEdges(a, d, e, c, b, f)}
                        }};
                        for (const PatternCost& proposal : patterns) {
                            if (!proposal.candidates ||
                                proposal.value + kTolerance >= oldBoundary)
                                continue;
                            rebuildThreeOpt(i, j, k, proposal.pattern);
                            ++result.threeOptImprovementCount;
                            improved = true;
                            break;
                        }
                        if (improved)
                            break;
                    }
                }
            }
        }
    }

    runTwoOpt();
    runRelocation();
    runTwoOpt();
    result.optimizedRouteLength += orderedLength(order);
    return order;
}

std::vector<std::size_t> repairUnsafeHoleVisitOrder(
    const std::vector<HoleVisitInput>& inputs,
    HoleRapidPlanner& rapidPlanner,
    HoleMachiningResult& result,
    const ToolpathGenerationControl* control)
{
    const std::size_t count = inputs.size();
    std::vector<std::size_t> order(count);
    for (std::size_t i = 0; i < count; ++i)
        order[i] = i;
    if (count <= 2 || !rapidPlanner.canPlanLinks())
        return order;
    if (count > kRapidRepairHoleLimit) {
        result.warnings.push_back(
            "hole rapid-link route repair was skipped because the setup exceeds 500 holes");
        return order;
    }

    // 避碰修复不重新求最短路，只在危险连接附近移动节点，并优先减少不安全边数。
    std::vector<std::vector<unsigned char>> candidate(
        count, std::vector<unsigned char>(count, 0));
    const std::size_t candidateCount =
        std::min(kRapidRepairCandidateCount, count - 1);
    for (std::size_t node = 0; node < count; ++node) {
        std::vector<std::size_t> neighbors;
        neighbors.reserve(count - 1);
        for (std::size_t other = 0; other < count; ++other)
            if (other != node)
                neighbors.push_back(other);
        std::stable_sort(neighbors.begin(), neighbors.end(),
                         [&](std::size_t a, std::size_t b) {
            const double da = inputs[node].clearance.SquareDistance(inputs[a].clearance);
            const double db = inputs[node].clearance.SquareDistance(inputs[b].clearance);
            if (da != db)
                return da < db;
            return inputs[a].originalIndex < inputs[b].originalIndex;
        });
        for (std::size_t rank = 0; rank < candidateCount; ++rank) {
            candidate[node][neighbors[rank]] = 1;
            candidate[neighbors[rank]][node] = 1;
        }
    }

    struct RepairScore {
        int unsafe = 0;
        double length = 0.0;
    };
    auto scoreRoute = [&](const std::vector<std::size_t>& route) {
        RepairScore score;
        for (std::size_t i = 1; i < route.size(); ++i) {
            if (cancellationRequested(control)) {
                score.unsafe = std::numeric_limits<int>::max();
                return score;
            }
            const RapidLinkPlan plan = rapidPlanner.evaluateLink(
                inputs[route[i - 1]].clearance, inputs[route[i]].clearance);
            if (plan.safe)
                score.length += plan.length;
            else
                ++score.unsafe;
        }
        return score;
    };
    auto scoreBetter = [](const RepairScore& a, const RepairScore& b) {
        return a.unsafe < b.unsafe ||
               (a.unsafe == b.unsafe && a.length + kTolerance < b.length);
    };

    RepairScore currentScore = scoreRoute(order);
    const int repairLimit = static_cast<int>(2 * count);
    for (int repair = 0;
         currentScore.unsafe > 0 && repair < repairLimit;
         ++repair) {
        if (cancellationRequested(control))
            return order;
        // 只标记危险边两端（首节点固定），避免安全区段被无谓打乱。
        std::vector<bool> movable(count, false);
        for (std::size_t i = 1; i < count; ++i) {
            if (cancellationRequested(control))
                return order;
            if (!rapidPlanner.evaluateLink(inputs[order[i - 1]].clearance,
                                           inputs[order[i]].clearance).safe) {
                movable[i] = true;
                if (i - 1 > 0)
                    movable[i - 1] = true;
            }
        }

        RepairScore bestScore = currentScore;
        std::vector<std::size_t> bestOrder;
        std::size_t bestOriginalIndex = std::numeric_limits<std::size_t>::max();
        std::size_t bestInsertPosition = std::numeric_limits<std::size_t>::max();
        for (std::size_t from = 1; from < count; ++from) {
            if (cancellationRequested(control))
                return order;
            if (!movable[from])
                continue;
            const std::size_t moved = order[from];
            std::vector<std::size_t> without = order;
            without.erase(without.begin() + static_cast<std::ptrdiff_t>(from));
            for (std::size_t insert = 1; insert <= without.size(); ++insert) {
                if (cancellationRequested(control))
                    return order;
                if (insert == from)
                    continue;
                const std::size_t previous = without[insert - 1];
                const bool nearPrevious = candidate[moved][previous] != 0;
                const bool nearNext = insert < without.size() &&
                    candidate[moved][without[insert]] != 0;
                if (!nearPrevious && !nearNext)
                    continue;
                std::vector<std::size_t> trial = without;
                trial.insert(trial.begin() + static_cast<std::ptrdiff_t>(insert), moved);
                const RepairScore trialScore = scoreRoute(trial);
                const bool better = scoreBetter(trialScore, bestScore);
                const bool stableTie =
                    trialScore.unsafe == bestScore.unsafe &&
                    std::abs(trialScore.length - bestScore.length) <= kTolerance &&
                    (inputs[moved].originalIndex < bestOriginalIndex ||
                     (inputs[moved].originalIndex == bestOriginalIndex &&
                      insert < bestInsertPosition));
                if (better || (scoreBetter(bestScore, currentScore) && stableTie)) {
                    bestScore = trialScore;
                    bestOrder = std::move(trial);
                    bestOriginalIndex = inputs[moved].originalIndex;
                    bestInsertPosition = insert;
                }
            }
        }
        if (bestOrder.empty() || !scoreBetter(bestScore, currentScore))
            break;
        result.rapidRepairRecoveredLinkCount += currentScore.unsafe - bestScore.unsafe;
        ++result.rapidRepairRelocationCount;
        order = std::move(bestOrder);
        currentScore = bestScore;
    }
    return order;
}

} // namespace oap::toolpath::detail
