// HoleMoldEnvelopeEntryResolver —— 将物理孔最外侧段恢复到开槽前模具外包络面。
// 属于 oap_toolpath_hole 模块；消费 HolePhysicalGrouping 的物理孔归组结果，
// 由自动规划链路在生成刀路前调用，保持孔底和阶段关系不变。
#pragma once

#include "toolpath/hole/HoleMachiningInput.h"

#include <cstddef>
#include <vector>

#include "analysis/FeatureTypes.h"
#include "analysis/Model.h"
#include "analysis/MoldCoordinateFrame.h"
#include "toolpath/core/ToolpathDiagnostics.h"

namespace oap::toolpath {

// 单个入口调整记录：记录原始入口、恢复后的外包络入口及沿轴延伸长度。
struct HoleMoldEnvelopeEntryAdjustment {
    std::size_t holeIndex = 0;         // 原输入向量中的孔段下标。
    std::size_t physicalGroupIndex = 0; // 所属物理孔的全局归组序号。
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    gp_Pnt originalEntry;
    gp_Pnt envelopeEntry;
    double extensionLength = 0.0;      // 入口沿孔轴向外延伸的长度（mm）。
};

// 解析结果：修正后的孔列表 + 调整记录 + 投影失败诊断。
struct HoleMoldEnvelopeEntryResult {
    std::vector<HoleMachiningInput> holes;
    std::vector<HoleMoldEnvelopeEntryAdjustment> adjustments;
    std::vector<ToolpathDiagnostic> diagnostics;
    int physicalGroupCount = 0;
};

// 孔模架包络入口解析器：为孔加工确定模架包络上的入口位置。
class HoleMoldEnvelopeEntryResolver {
public:
    // 把每个物理孔的最外侧段入口沿孔轴投影到对应模具外包络面。
    // 返回修正后的孔副本；无法投影时写入 diagnostics 并保持原入口。
    HoleMoldEnvelopeEntryResult resolve(
        const std::vector<HoleMachiningInput>& holes,
        const Bbox& worldBounds,             // 模具世界包围盒，外包络投影的依据。
        const MoldCoordinateFrame& frame,
        double linearTolerance = 1.0e-3,
        double axisToleranceDeg = 1.0) const;
};

} // namespace oap::toolpath
