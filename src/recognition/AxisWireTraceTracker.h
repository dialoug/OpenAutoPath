// 从各个轴向平面的内环或外环按有向面高度追踪连接面。
// 模块: oap_recognition。输入为复合体 + AAG 邻接图 + 几何缓存；以轴向平面为
//       根，沿内环/外环向实体内部做高度单调扩散，把走过的面按边连通分量拆
//       成分支并分类为盲槽/盲腔。
// 消费方: ThreeAxisWireTraceUseCase 的三轴联合识别；*OCCT 结尾的 Z 轴入口
//       保留给原有单轴流程。
#pragma once

#include <unordered_set>
#include <vector>

class AAG;
class TopoDS_Shape;
class gp_Dir;

namespace oap {

class RecognitionGeometryCache;

// 追踪分支最终特征类别：盲槽（末级邻接面的全部有效法向均垂直根面法向）
// 与盲腔（其余情况）。命名沿用三轴结构识别的槽/腔语义。
enum class AxisWireTraceFeatureKind {
    BlindSlot,
    BlindPocket
};

// 轴向线环追踪的一条分支：从第一级邻接面扩散到最远层，分类为盲槽/盲腔。
struct AxisWireTraceBranch {
    // 当前线环实际进入高度泛洪的第一级邻接面。
    std::vector<int> firstLevelFaceIndices;
    // 在当前边连通分支中，从第一级面进行多源拓扑扩散后距离最远的一层。
    std::vector<int> lastLevelFaceIndices;
    // 终止平面不属于走过面。
    std::vector<int> traversedFaceIndices;
    // 当前分支直接到达但不染为路径面的轴向终止平面。
    std::vector<int> terminalFaceIndices;
    // 最后一级邻接面非空且每张面的全部有效法向均垂直根面法向时为盲槽；
    // 无最后一级邻接面、存在非垂直法向或无法可靠验证时均按盲腔处理。
    AxisWireTraceFeatureKind featureKind =
        AxisWireTraceFeatureKind::BlindPocket;
};

// 单个根线环的追踪路径：内外环拆分后的分支与走过面/终止面并集。
struct AxisWireTracePath {
    int rootFaceIndex = 0;
    double rootHeight = 0.0;
    // 根平面定向法向相对当前高度轴的正负方向。
    int normalDirection = 1;
    // 同一线环扩散得到的走过面按共享拓扑边拆成连通分量；每个分量分别
    // 形成一条分支并独立分类。innerTraces 汇总所有内环的拆分分量。
    std::vector<AxisWireTraceBranch> outerTraces;
    std::vector<AxisWireTraceBranch> innerTraces;
    std::vector<int> terminalFaceIndices;
    // 内外环走过面的并集，用于跨轴排除和总量统计。
    std::vector<int> traversedFaceIndices;
};

// 轴向线环追踪的汇总结果：所有路径、统计计数与内外环走过面并集。
struct AxisWireTraceResult {
    // paths 按每种法向的相对高度从低到高排列；每个根平面保留独立路径。
    std::vector<AxisWireTracePath> paths;
    // 下列索引是所有独立路径的并集，用于统计；终止平面不属于染色路径。
    std::vector<int> terminalFaceIndices;
    std::vector<int> traversedFaceIndices;
    int mainOuterFaceCount = 0;
    int rootFaceCount = 0;
    int seedWireCount = 0;
    int excludedFaceCount = 0;
    int traversedFaceCount = 0;
    int directionalHeightStepCount = 0;
    int stoppedOppositeHeightCount = 0;
    int stoppedWithoutDirectionalNeighborCount = 0;
    int skippedInvalidFaceCount = 0;
};

using InnerWireShrinkPath = AxisWireTracePath;
using InnerWireShrinkTraceResult = AxisWireTraceResult;
using OuterWireSpreadPath = AxisWireTracePath;
using OuterWireSpreadTraceResult = AxisWireTraceResult;
using CombinedWireTracePath = AxisWireTracePath;
using CombinedWireTraceResult = AxisWireTraceResult;

// 通用轴向入口。根面高度取紧致包围盒中心在 axisDirection 上的投影；
// 候选扩散面逐条计算非退化边线的线质心投影，并取相对高度最低的边线。
// 仅当这条最低边线不低于根面高度时进入。
InnerWireShrinkTraceResult TraceInnerWireShrinkAlongAxisOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices = {});

OuterWireSpreadTraceResult TraceOuterWireSpreadAlongAxisOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices = {});

CombinedWireTraceResult TraceCombinedWireLoopsAlongAxisOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices = {});

// Z/Y/X 联合追踪使用：主外表面、紧致包围盒和边—面映射均复用同一 Unit 缓存。
// geometryCache 为可变引用：本入口不重新构建缓存，只复用；它必须与 graph
// 来自同一次 AAG 构建。
CombinedWireTraceResult TraceCombinedWireLoopsAlongAxisOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices,
    RecognitionGeometryCache& geometryCache);

// 内环收缩：对每个未排除且法向平行 Z 轴的平面：
// 1. 枚举其内环，并找到通过内环边直接相连的面；
// 2. 根面高度取紧致包围盒中心 Z 值；扩散面取相对根法向最低的边线质心；
// 3. 该最低边线必须不低于根面高度；
// 4. 每个新面继续检查全部相邻面，形成向内坍缩式蔓延；
// 5. 扩散面最低边线低于根面且超过容差时不进入，等高面继续；遇到 Z 向平面时停止；
// 6. 终止平面不属于追踪路径。
// 根平面按相对高度从低到高返回；同一方向中 +Z 按 Z 升序、-Z 按 Z 降序。
// excludedFaceIndices 在任何面筛选前设为不可进入；主外表面不可作为路径面。
// 以下三个接口保留为 Z 轴兼容入口。
InnerWireShrinkTraceResult TraceInnerWireShrinkOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const std::unordered_set<int>& excludedFaceIndices = {});

// 外环扩散：根平面、排除、排序、高度方向与终止规则同内环收缩，
// 仅将种子线环改为外环；同样要求每张扩散面的相对最低边线不低于根面。
OuterWireSpreadTraceResult TraceOuterWireSpreadOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const std::unordered_set<int>& excludedFaceIndices = {});

// 联合追踪：同一个 Z 向根面的外环和全部内环分别追踪；每条线环的结果
// 再按走过面的边连通分量拆分、分别分类和染色，同时在根 path 中保留
// 全部分量走过面的并集。
CombinedWireTraceResult TraceCombinedWireLoopsOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const std::unordered_set<int>& excludedFaceIndices = {});

} // namespace oap
