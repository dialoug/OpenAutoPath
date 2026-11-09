// 轮廓区域构建器（oap_toolpath_profile）。
// 从 StepIndexer 索引的模型面识别外轮廓源线框、加工平面与区域面，产出
// ProfileRegion 列表（或 SkippedProfileRegion 跳过原因），并支持按毛坯余量
// 更新 stockBoundary；是识别结果到轮廓刀路规划的桥梁。
#pragma once

#include <optional>
#include <vector>

#include "toolpath/profile/geometry/ProfileRegion.h"

namespace oap {
class StepIndexer;
}

namespace oap::toolpath {

// 轮廓区域构建选项：离散偏转、容差与毛坯余量。
struct ProfileRegionBuildOptions {
    double discretizationDeflection = 0.01;
    double tolerance = 1.0e-7;
    double stockMarginX = 0.0;
    double stockMarginY = 0.0;
    double stockMarginZ = 0.0;
};

// 轮廓区域构建器：从 StepIndexer 识别外轮廓源线框与区域面。
class ProfileRegionBuilder {
public:
    static void updateStockBoundary(
        ProfileRegion& region,
        const ProfileRegionBuildOptions& options = {});

    std::vector<ProfileRegion> build(
        const std::vector<RootOuterWireProfileSource>& sources,
        const StepIndexer& indexer,
        const ProfileRegionBuildOptions& options = {},
        std::vector<SkippedProfileRegion>* skipped = nullptr) const;

    std::optional<ProfileRegion> buildOne(
        const RootOuterWireProfileSource& source,
        const StepIndexer& indexer,
        const ProfileRegionBuildOptions& options = {},
        std::string* error = nullptr) const;
};

} // namespace oap::toolpath
