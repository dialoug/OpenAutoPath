#include "toolpath/pocket/simulation/DexelIntervalOps.h"
#include "toolpath/pocket/simulation/detail/DexelCellIntervals.h"

#include <array>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

using oap::toolpath::DexelInterval;
using oap::toolpath::subtractDexelInterval;

bool expect(bool condition, const std::string& message)
{
    if (!condition)
        std::cerr << "FAILED: " << message << '\n';
    return condition;
}

bool expectIntervals(
    const std::vector<DexelInterval>& actual,
    const std::vector<DexelInterval>& expected,
    const std::string& message)
{
    return expect(actual == expected, message);
}

bool splitsInteriorRemoval()
{
    std::vector<DexelInterval> material{{0.0, 10.0}};
    return expect(
               subtractDexelInterval(
                   material, {4.0, 6.0}, 1.0e-6),
               "an interior removal should report a material change") &&
           expectIntervals(
               material, {{0.0, 4.0}, {6.0, 10.0}},
               "an interior removal should split one material interval");
}

bool subtractsAcrossSeparatedMaterial()
{
    std::vector<DexelInterval> material{
        {0.0, 2.0}, {4.0, 6.0}, {8.0, 10.0}};
    return expect(
               subtractDexelInterval(
                   material, {1.0, 9.0}, 1.0e-6),
               "a removal spanning gaps should report a material change") &&
           expectIntervals(
               material, {{0.0, 1.0}, {9.0, 10.0}},
               "subtraction should preserve only material outside the removal");
}

bool trimsAndClearsMaterial()
{
    std::vector<DexelInterval> lowerTrim{{0.0, 10.0}};
    std::vector<DexelInterval> upperTrim{{0.0, 10.0}};
    const DexelInterval* const lowerStorage =
        lowerTrim.data();
    const DexelInterval* const upperStorage =
        upperTrim.data();
    std::vector<DexelInterval> cleared{
        {0.0, 2.0}, {4.0, 6.0}};
    return expect(
               subtractDexelInterval(
                   lowerTrim, {-1.0, 3.0}, 1.0e-6),
               "a lower-bound overlap should report a material change") &&
           expectIntervals(
               lowerTrim, {{3.0, 10.0}},
               "a lower-bound overlap should trim the material start") &&
           expect(
               subtractDexelInterval(
                   upperTrim, {7.0, 11.0}, 1.0e-6),
               "an upper-bound overlap should report a material change") &&
           expectIntervals(
               upperTrim, {{0.0, 7.0}},
               "an upper-bound overlap should trim the material end") &&
           expect(
               lowerTrim.data() == lowerStorage &&
                   upperTrim.data() == upperStorage,
               "single-interval boundary trims should update in place") &&
           expect(
               subtractDexelInterval(
                   cleared, {-1.0, 7.0}, 1.0e-6),
               "a covering removal should report a material change") &&
           expect(
               cleared.empty(),
               "a covering removal should clear all material intervals");
}

bool preservesNoOpInputs()
{
    constexpr double tolerance = 1.0e-6;
    const std::vector<DexelInterval> original{
        {0.0, 2.0}, {4.0, 6.0}};

    std::vector<DexelInterval> outside = original;
    std::vector<DexelInterval> thin = original;
    std::vector<DexelInterval> nearGap = original;
    std::vector<DexelInterval> reversed = original;
    std::vector<DexelInterval> notFinite = original;
    std::vector<DexelInterval> negativeTolerance =
        original;
    std::vector<DexelInterval> infiniteTolerance =
        original;

    return expect(
               !subtractDexelInterval(
                   outside, {7.0, 8.0}, tolerance),
               "a disjoint removal should be a no-op") &&
           expectIntervals(
               outside, original,
               "a disjoint removal must preserve the source intervals") &&
           expect(
               !subtractDexelInterval(
                   thin, {1.0, 1.0 + tolerance},
                   tolerance),
               "a tolerance-thick removal should be a no-op") &&
           expectIntervals(
               thin, original,
               "a tolerance-thick removal must preserve the source intervals") &&
           expect(
               !subtractDexelInterval(
                   nearGap,
                   {2.0 + 0.25 * tolerance,
                    4.0 - 0.25 * tolerance},
                   tolerance),
               "a removal confined to a material gap should be a no-op") &&
           expectIntervals(
               nearGap, original,
               "a gap-only removal must preserve the source intervals") &&
           expect(
               !subtractDexelInterval(
                   reversed, {3.0, 2.0}, tolerance),
               "a reversed removal should be rejected") &&
           expectIntervals(
               reversed, original,
               "a reversed removal must preserve the source intervals") &&
           expect(
               !subtractDexelInterval(
                   notFinite,
                   {std::numeric_limits<double>::quiet_NaN(),
                    5.0},
                   tolerance),
               "a non-finite removal should be rejected") &&
           expectIntervals(
               notFinite, original,
               "a non-finite removal must preserve the source intervals") &&
           expect(
               !subtractDexelInterval(
                   negativeTolerance, {0.5, 1.5},
                   -tolerance),
               "a negative tolerance should be rejected") &&
           expectIntervals(
               negativeTolerance, original,
               "a negative tolerance must preserve the source intervals") &&
           expect(
               !subtractDexelInterval(
                   infiniteTolerance, {0.5, 1.5},
                   std::numeric_limits<double>::infinity()),
               "a non-finite tolerance should be rejected") &&
           expectIntervals(
               infiniteTolerance, original,
               "a non-finite tolerance must preserve the source intervals");
}

bool cellStorageOwnershipAndTransitions()
{
    using oap::toolpath::multi_dexel_detail::DexelCellIntervals;
    DexelCellIntervals cell;
    cell.push_back({0, 10});
    if (!expect(!cell.usesHeap(), "one interval must remain inline")) return false;
    auto copy = cell;
    copy.front().upper = 8;
    if (!expect(cell.front().upper == 10, "inline copies must be independent")) return false;
    if (!expect(subtractDexelInterval(cell, {4, 6}, 0) && cell.usesHeap() && cell.size() == 2,
                "splitting a cell must spill both intervals to owned storage")) return false;
    copy = cell;
    copy.front().upper = 3;
    if (!expect(cell.front().upper == 4 && copy.data() != cell.data(),
                "heap copies must not mutate their source")) return false;
    auto moved = std::move(copy);
    copy.push_back({20, 30});
    if (!expect(moved.size() == 2 && !copy.usesHeap(), "moved-from cells must be reusable")) return false;
    cell = cell;
    cell = std::move(cell);
    if (!expect(cell.size() == 2 && cell.front().upper == 4, "self assignment must preserve the cell")) return false;
    if (!expect(subtractDexelInterval(cell, {5, 11}, 0) && !cell.usesHeap() && cell.size() == 1,
                "removing the second interval must reclaim inline storage")) return false;
    cell.push_back(cell.front());
    if (!expect(cell.size() == 2 && cell.begin()[1] == cell.front(),
                "an aliased append must survive reallocation")) return false;
    cell.assign(std::span<const DexelInterval>(cell.data() + 1, 1));
    if (!expect(!cell.usesHeap() && cell.front() == DexelInterval{0, 4},
                "assigning a view into a cell must be safe")) return false;
    const auto original = cell;
    try {
        cell.reserve(std::numeric_limits<std::size_t>::max());
        return expect(false, "oversized allocation should be rejected");
    } catch (const std::length_error&) {
        if (!expect(cell.front() == original.front() && !cell.usesHeap(),
                    "failed growth must preserve the cell")) return false;
    }
    moved = std::move(cell);
    if (!expect(!moved.usesHeap() && moved.size() == 1 && cell.empty(),
                "inline move assignment must release previous heap ownership")) return false;
    moved.clear();
    return expect(moved.empty() && !moved.usesHeap(), "clear must release storage");
}

bool cellSubtractionMatchesDiscreteMaterial()
{
    using oap::toolpath::multi_dexel_detail::DexelCellIntervals;
    std::mt19937 random(71923);
    for (int run = 0; run < 200; ++run) {
        std::array<bool, 64> occupied;
        occupied.fill(true);
        DexelCellIntervals cell;
        cell.push_back({0, 64});
        std::vector<DexelInterval> legacy{{0, 64}};
        for (int cut = 0; cut < 40; ++cut) {
            const int lower = static_cast<int>(random() % 72) - 4;
            const int upper = lower + static_cast<int>(random() % 12);
            bool changed = false;
            for (int i = 0; i < 64; ++i) {
                if (i >= lower && i < upper) {
                    changed = changed || occupied[i];
                    occupied[i] = false;
                }
            }
            const DexelInterval removal{static_cast<double>(lower), static_cast<double>(upper)};
            const bool actualChange = subtractDexelInterval(cell, removal, 0);
            const bool legacyChange = subtractDexelInterval(legacy, removal, 0);
            std::vector<DexelInterval> expected;
            for (int i = 0; i < 64;) {
                if (!occupied[i]) { ++i; continue; }
                const int begin = i;
                while (i < 64 && occupied[i]) ++i;
                expected.push_back({static_cast<double>(begin), static_cast<double>(i)});
            }
            if (!expect(actualChange == changed && legacyChange == changed && legacy == expected &&
                            std::vector<DexelInterval>(cell.begin(), cell.end()) == expected,
                        "both interval stores must match an independent discrete-material oracle")) return false;
            if (!expect(cell.usesHeap() == (cell.size() > 1),
                        "subtraction must return empty/single cells to inline storage")) return false;
            const auto saved = cell;
            if (!expect(!subtractDexelInterval(cell, {0, 20}, -1) &&
                            std::equal(cell.begin(), cell.end(), saved.begin(), saved.end()),
                        "invalid input must preserve inline and heap cells")) return false;
        }
    }
    return true;
}

} // namespace

int main()
{
    if (!cellStorageOwnershipAndTransitions() || !cellSubtractionMatchesDiscreteMaterial())
        return 1;
    if (!splitsInteriorRemoval())
        return 1;
    if (!subtractsAcrossSeparatedMaterial())
        return 1;
    if (!trimsAndClearsMaterial())
        return 1;
    if (!preservesNoOpInputs())
        return 1;

    std::cout << "Dexel interval smoke checks passed\n";
    return 0;
}
