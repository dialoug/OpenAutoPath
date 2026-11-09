// Private PocketZMapMultiDexelSupport contract; not part of the module public API.
#pragma once

#include "toolpath/pocket/simulation/detail/PocketZMapCoreSupport.h"

namespace oap::toolpath::zmap_detail {

struct MultiDexelDifferenceSet {
    DexelDisplayGrid initialMissing;
    DexelDisplayGrid remainingAllowance;
    DexelDisplayGrid overcut;
    double initialMissingVolume = 0.0;
    double remainingAllowanceVolume = 0.0;
    double overcutVolume = 0.0;
    double certainOvercutVolume = 0.0;
};

bool buildMultiDexelDifferences(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target,
    double tolerance,
    const ToolpathGenerationControl* control,
    MultiDexelDifferenceSet& output,
    std::string& error);

void buildMultiDexelFeatureAllowances(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target,
    double tolerance,
    const ToolpathGenerationControl* control,
    std::vector<StockFeatureAllowance>& output);

bool buildMultiDexelFeatureAllowanceGrid(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target,
    const std::string& featureId,
    double tolerance,
    std::function<bool()> cancellationRequested,
    DexelDisplayGrid& output,
    std::string& error);

void buildZMapFeatureAllowances(
    const PocketZMapSimulationData& data,
    const TargetField& target,
    double tolerance,
    const ToolpathGenerationControl* control,
    std::vector<StockFeatureAllowance>& output);

bool buildMultiDexelOperationOvercut(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target,
    const MultiDexelOperationState& operation,
    double tolerance,
    std::function<bool()> cancellationRequested,
    DexelDisplayGrid& output,
    double& volume,
    double& certainVolume,
    std::string& error);

double displayGridVolume(
    const DexelDisplayGrid& grid);

std::string multiDexelDisplaySourceFingerprint(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target);

void reportComparisonFailure(
    StockComparisonResult& result,
    const ToolpathGenerationControl* control,
    std::string message);

} // namespace oap::toolpath::zmap_detail
