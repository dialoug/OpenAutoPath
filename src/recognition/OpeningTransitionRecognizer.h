// 孔口扩径过渡识别接口：从孔壁凸邻接面中提取非解析沉头几何。
// 模块: oap_recognition（detail 层），仅被 Recognition.cpp 的孔识别调用。
#pragma once

#include "Recognition.h"

namespace oap::recognition::detail {

// 识别与圆柱孔壁相连、向外扩径并连接外部平面的旋转过渡。
// 参数: graph 为复合体 AAG；wallFaceId 为孔壁圆柱面索引；axis / radius 为
//       孔轴与孔半径；convexFaceIds 为孔壁的凸接邻面候选。
// 返回: HoleOpeningTransition，其 faceId < 1 表示未识别到过渡。
HoleOpeningTransition recognizeOpeningTransition(
    const AAG& graph,
    int wallFaceId,
    const gp_Ax1& axis,
    double wallRadius,
    const std::vector<int>& convexFaceIds);

} // namespace oap::recognition::detail
