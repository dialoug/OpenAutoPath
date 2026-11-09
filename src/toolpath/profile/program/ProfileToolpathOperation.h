// 外轮廓工序数据模型（oap_toolpath_profile）。
// ProfileToolpathOperation 描述单条外轮廓工序：区域索引、刀具上下文、
// 算法标识、装夹刀轴、多刀余量加工与精加工策略，并携带生成的
// ProfileContourResult；由 ProfileToolpathService 产出，经 ProfileProgramAdapter
// 转为中立程序，供校验与导出。
#pragma once

#include <string>
#include <vector>

#include "analysis/FeatureTypes.h"
#include "toolpath/core/ToolpathToolContext.h"
#include "toolpath/profile/planning/ProfileContour.h"

namespace oap::toolpath {

// 外轮廓工序：区域、刀具、算法与生成的轮廓刀路。
struct ProfileToolpathOperation {
    std::string operationId;
    std::string algorithmId = "builtin.profile.freecad";
    int sourceRegionIndex = -1;
    int setupIndex = 0;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    ToolpathToolContext tool;
    std::vector<std::string> sourceFaceIds;
    std::string rootFaceId;
    std::string rootWireId;
    gp_Vec outwardToolAxis;
    // 自动多刀外轮廓阶段：0 表示全量外部清料；正值表示只加工该前序
    // 大刀留下且当前刀可达的剩余材料。
    double restMachiningPreviousToolDiameter = 0.0;
    // 保存本工序生成时的精加工圈策略，确保 Job 重建不会让每个粗加工
    // 阶段都重复最终轮廓。
    bool finishPass = true;
    ProfileContourResult contour;
};

} // namespace oap::toolpath
