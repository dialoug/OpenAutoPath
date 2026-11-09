// PocketClearing —— 根据归一化 PocketRegion 生成 2.5D 刀具中心路径。
// Clipper2 负责平面可达区域计算；对于侧壁随高度变化的腔体，OCCT 可提供
// 各深度层截面。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

#include "toolpath/pocket/geometry/OcctAreaAdapter.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathMoves.h"
#include "toolpath/core/ToolpathPrimitives.h"

namespace oap::toolpath {

// 清腔生成的失败码，供调用方分类处理（如 UI 展示具体原因、测试断言）。
enum class PocketClearingFailureCode {
    None,
    AlgorithmUnavailable,
    AlgorithmExecutionFailed,
    InvalidRegion,
    LayerSectionFailed,
    LayerNoEdges,
    LayerOpenContour,
    LayerNoValidArea,
    ToolDoesNotFit,
    NoClearableArea
};

// 单条闭合刀路环(单个深度层、单次偏置的结果)。points 为局部 frame 下的 2D 点,
// world 为同序的 3D 点(投影回工件坐标)。环隐式闭合(首尾不重复)。
struct ClearingLoop {
    std::vector<gp_Pnt2d> points;
    std::vector<gp_Pnt> world;
    int ring = 0;          // 图案执行层级；Offset 中 0=最外环，FreePath 按中心向外
    int depthIndex = 0;    // 深度层级:0=第一刀层,向最终深度递增
    bool closed = true;    // true=闭环(Offset 同心环);false=开放折线(ZigZag/Spiral 的一趟连续走刀)
    bool isHole = false;   // true=该环来自岛屿外扩后形成的内边界(绕岛走刀)
    double offset = 0.0;    // 本环刀心相对原始边界/岛的偏置距离(mm)
    double depthOffset = 0.0; // 相对槽/腔底面的局部偏移(mm),0=底面,正值朝开口方向
    double area = 0.0;      // 有符号面积,用于调试与排序;负值通常为岛周边环
    int sequence = 0;       // 生成后的稳定顺序,便于 UI/G-code 阶段复用
    // FreePath 规划器为当前 run 选择的同深连接。首点应等于上一 run 终点，
    // 末点应等于当前 run 起点；公共装配器会重新做连续几何安全验证，失败
    // 时仍按标准退刀/重进策略恢复。
    std::vector<gp_Pnt2d> preferredLinkFromPrevious;
    std::vector<gp_Pnt> preferredWorldLinkFromPrevious;
};

// 挖槽清腔刀路。带逐层几何或实体材料保护的区域按每个目标深度独立生成，
// 其余简单区域可复用固定底面截面；随后统一生成快速横移、进刀、切削和退刀移动。
struct ProtectedMaterialLayer {
    // 当前刀尖层到开口之间的实体截面并集，用于固定三轴刀具轴向避障。
    int depthIndex = 0;
    double depthOffset = 0.0;
    std::vector<Loop2d> contours;
};

// 槽腔清料结果：清料环与统计。
struct PocketClearingResult {
    std::vector<ClearingLoop> loops; // 按深度层和所选图案的真实执行顺序排列
    std::vector<ToolpathMove> moves;  // 按执行顺序排列的移动段
    std::vector<ToolpathPrimitive> primitives; // 与 moves 等价的有序语义流
    std::string warning;             // 非致命提示(如图案回退、区域过小)
    bool ok = false;                 // false=未能生成任何环(error 给出原因)
    bool cancelled = false;          // true=协作取消；不得发布任何局部刀路
    std::string error;
    PocketClearingFailureCode failureCode = PocketClearingFailureCode::None;
    bool usedPerLayerSections = false;
    bool usedProtectedMaterialSections = false;
    bool restMachining = false;
    bool noRestMaterial = false;
    bool firstLayerPreflightRejected = false;
    int sectionLayerCount = 0;
    std::vector<ProtectedMaterialLayer> protectedMaterialLayers;
    int openSectionLayerCount = 0;
    int virtualOpenBoundaryCount = 0;
    int failedDepthIndex = -1;
    double failedDepthOffset = 0.0;
    int ringCount = 0;                // 各深度层实际偏置层数的最大值
    int adaptivePassCount = 0;
    int adaptiveRemainingComponentCount = 0;
    int adaptiveOverloadSegmentCount = 0;
    int adaptiveWavefrontLayerCount = 0;
    int adaptiveCentralSpiralCount = 0;
    int adaptiveRadialComponentCount = 0;
    int adaptiveRadialPassCount = 0;
    double adaptiveRemainingMaterialFraction = 0.0;
    double adaptiveMaximumEngagementAngleDeg = 0.0;
    int depthLayerCount = 0;          // 实际生成的深度层数
    double toolRadius = 0.0;
    double stepOver = 0.0;
    double firstOffset = 0.0;
    double startDepthOffset = 0.0;    // 实际起始偏移(mm),0=底面,正值朝开口方向
    double finalDepthOffset = 0.0;    // 实际最终偏移(mm)
    double stepDown = 0.0;            // 实际使用的最大层深(mm)
    double finishDepth = 0.0;         // 实际精修留量(mm),0=不留;贴底由单独一刀切除
    // 下刀面偏移换算后，以腔底为 0 的实际局部坐标。
    double safeHeightOffset = 0.0;
    double clearanceHeightOffset = 0.0;
    int verticalEntryCount = 0;
    int rampOnShapeEntryCount = 0;
    int linearRampEntryCount = 0;
    int zigzagRampEntryCount = 0;
    int helicalRampEntryCount = 0;
    int entryFallbackCount = 0;
    bool usedParallelLayerProcessing = false;
    bool usedParallelPatternProcessing = false;
    std::size_t patternParallelBatchCount = 0u;
    std::size_t patternParallelTaskCount = 0u;
    std::size_t patternPeakParallelism = 1u;
    double sectionMilliseconds = 0.0; // 全部深度截面的并行墙钟耗时
    double patternMilliseconds = 0.0; // 轮廓归一化、偏置/扫描及三维映射耗时
    double moveAssemblyMilliseconds = 0.0;
    double movePreparationMilliseconds = 0.0;
    double moveLinkingMilliseconds = 0.0;
    std::size_t collinearMergedMoveCount = 0;
    // 余料加工路径会在不改变深度层和径向清料顺序的前提下，按安全直连
    // 优先、否则最短跳刀的规则重排同组路径。以下计数用于诊断优化效果。
    std::size_t travelReorderedLoopCount = 0;
    std::size_t travelAdjustedStartCount = 0;
    std::size_t travelChoiceParallelBatchCount = 0u;
    std::size_t travelChoiceParallelTaskCount = 0u;
    std::size_t travelChoicePeakParallelism = 1u;
    std::size_t sectionWireParallelBatchCount = 0u;
    std::size_t sectionWireParallelTaskCount = 0u;
    std::size_t sectionWirePeakParallelism = 1u;
    std::size_t occtInternalParallelCount = 0u;
    std::size_t occtSerialFallbackCount = 0u;
    double totalMilliseconds = 0.0;
};

// 槽腔清料规划器：对可达区生成清料图案。
class PocketClearing {
public:
    // 对单个区域生成分层清腔环。region 须已由 PocketRegionBuilder 归一化
    // (外环 CCW、岛 CW)。失败时 result.ok=false 并填 error/failureCode；
    // 不返回半成品移动，调用方可以安全地跳过该区域。
    PocketClearingResult generate(const PocketRegion& region,
                                  const ToolpathParams& params,
                                  const ToolpathGenerationControl* control =
                                      nullptr) const;
};

} // namespace oap::toolpath
