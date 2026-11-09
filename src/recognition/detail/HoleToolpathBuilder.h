// 从已识别圆柱孔结果提取稳定的孔加工几何。
#pragma once

#include <vector>

#include "analysis/HoleGeometry.h"

namespace oap {
struct Model;
class StepIndexer;
}

class AAG;
struct HoleResult;

namespace oap::recognition::detail {

// 模块私有实现细节，不对外暴露；由 recognition 孔识别结果向后续应用层
// 提供稳定加工几何。holes 为识别结果，graph 提供邻接拓扑，model/indexer
// 提供模具坐标系与 STEP 面 id。返回按入口/孔底定向的加工几何列表。
std::vector<RecognizedHoleGeometry> buildHoleGeometries(
    const std::vector<HoleResult>& holes,
    const AAG& graph,
    const Model& model,
    const StepIndexer& indexer);

} // namespace oap::recognition::detail
