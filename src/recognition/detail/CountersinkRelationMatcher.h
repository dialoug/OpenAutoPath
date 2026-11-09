// 解析沉头孔同轴台阶与平底偏心子孔关系。
// oap_recognition 模块私有实现细节，不对外暴露；供孔特征父子关系解析内部调用。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gp_Pnt.hxx>

#include "analysis/FeatureTypes.h"

class AAG;

namespace oap::recognition::detail {

// 只携带沉头段匹配需要的稳定几何和拓扑索引，避免匹配器依赖刀路构建过程。
struct CountersinkStageMatchInput {
    FeatureKind kind = FeatureKind::ThroughHole;
    double radius = 0.0;
    gp_Pnt first;
    gp_Pnt second;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    std::vector<int> cylinderFaceIds;
    HoleBottomProfile bottomProfile = HoleBottomProfile::Unknown;
    bool largestCountersinkStage = false;
    std::vector<int> planarBoundaryFaceIds;
};

// 平底子孔的匹配结果：ownerIndex 为父沉孔在 inputs 中的下标，
// floorFaceId 为共享的平底面，valid 表示匹配成功。
struct CounterboreFloorOwner {
    std::size_t ownerIndex = 0;
    int floorFaceId = -1;
    bool valid = false;
};

// 返回 candidate 应继承的最大直径沉头段索引；未找到时返回 inputs.size()。
// 必须同时满足同轴以及直接共享台阶肩面，或存在严格的
// “共轴锥/环面 + 轴向环形肩面”拓扑链；不能仅凭几何位置合并。
std::size_t findCountersinkStageOwner(
    const std::vector<CountersinkStageMatchInput>& inputs,
    std::size_t candidateIndex,
    const AAG& graph,
    double linearTolerance);

// 偏心通孔/盲孔若与最大直径沉孔的平底面直接共享孔口边，则保持独立孔，
// 但继承父沉孔的装夹面。没有直接拓扑关系时严格不猜测。
CounterboreFloorOwner findCounterboreFloorOwner(
    const std::vector<CountersinkStageMatchInput>& inputs,
    std::size_t candidateIndex,
    const AAG& graph,
    double linearTolerance);

} // namespace oap::recognition::detail
