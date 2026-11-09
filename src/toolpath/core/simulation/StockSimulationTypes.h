// 顺序材料去除仿真的公共参数与结果。孔模拟器构造精确切削体，腔模拟器
// 使用分块 Z-map；两者共用快照、体积和诊断语义。
#pragma once

#include <memory>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/core/simulation/InProcessStock.h"

namespace oap::toolpath {

struct PocketZMapSimulationData;

enum class StockSimulationMethod {
    HeightField,
    FixedAxisLayeredBRep,
};

// 毛坯仿真选项：仿真方法、几何/弦高/斜坡/刀具轮廓容差。
struct StockSimulationOptions {
    StockSimulationMethod method =
        StockSimulationMethod::HeightField;
    double geometricTolerance = 1.0e-5;
    double arcChordTolerance = 0.02;
    // 固定轴 BRep 对线性/螺旋斜坡按刀轴分层时允许的最大轴向欠切误差。
    // 非正值表示自动采用不小于 0.05 mm 的弦高容差。
    double rampAxialTolerance = 0.0;
    // 圆角刀/球刀旋转轮廓离散为内接圆柱带时允许的最大轴向欠切误差。
    // 非正值表示自动采用刀角半径的 5%，并限制在 0.05～2.5 mm；
    // 若弦高容差更大则沿用弦高容差。
    double toolProfileAxialTolerance = 0.0;
    // 圆角刀/球刀内接带允许的最大径向欠切误差。非正值时采用不劣于
    // 旧固定轴向带的径向跳变量，保持兼容性能；轴向和径向约束同时生效。
    double toolProfileRadialTolerance = 0.0;
    int maxSweepSegments = 20000;
    // 防止异常斜坡或错误坐标产生无界数量的轴向分层。
    int maxRampSweepSlices = 100000;
    // 入口组数乘刀具轮廓带数超过该值时，跳过昂贵的二维包含预检，
    // 直接生成安全的内接斜坡分层；非正值表示始终跳过。
    int maxRampContainmentChecks = 2048;
    int maxToolProfileBands = 512;
    // 异轴旋转刀具使用内接正多边形参与 Dexel 射线求交。该上限约束
    // 径向分面数量；实际误差会记录到 maximumToolProfileRadialError。
    int maxOrientedToolFacets = 256;
    bool stopOnFailure = true;
    // 固定轴 BRep 的二维轮廓、轴向层、独立实体构建/校验及 OCCT 布尔，
    // 以及 Z-map 分块均允许并行；false 用于确定性串行诊断。
    bool runParallel = true;
    // 腔毛坯仿真使用分块 Z-map。非正值表示采用平衡档（约每刀径 40 个
    // XY 单元），再根据毛坯投影范围和总单元预算安全放粗。显式设置时
    // 单位为 mm，仍不得突破 zMapMaxCellCount。
    double zMapResolution = 0.0;
    // 自动提高分辨率下限，使高度场不会因超大毛坯占用无界内存。
    int zMapMaxCellCount = 2000000;
    // 每个分块由一个工作线程独占更新，因此不同分块之间无需原子写高度。
    int zMapTileSize = 32;
};

// 毛坯仿真结果：初始/最终形状、去料步骤、诊断与体积。
struct StockSimulationResult {
    TopoDS_Shape initialStock;
    TopoDS_Shape finalStock;
    std::vector<StockRemovalRecord> steps;
    std::vector<ToolpathDiagnostic> diagnostics;
    double initialVolume = 0.0;
    double finalVolume = 0.0;
    double removedVolume = 0.0;
    // 由折线恢复圆弧语义时，fallback 采样点相对解析圆的最大径向残差。
    double maximumCurveFitError = 0.0;
    // 仅在解析平面后端回退到折线扫掠时，圆弧 fallback 弦线相对圆弧的
    // 最大弓高；解析圆弧后端不计入该项。
    double maximumChordError = 0.0;
    double sweepBuildMilliseconds = 0.0;
    double inputValidationMilliseconds = 0.0;
    double booleanMilliseconds = 0.0;
    double resultValidationMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
    int sweepSegmentCount = 0;
    // 孔仿真表示提交给布尔运算的切削实体数；Z-map 腔仿真表示每道工序
    // 的逻辑更新批次数。保留同一字段以兼容现有摘要和持久化调用方。
    int removalSolidCount = 0;
    int planarSweepLayerCount = 0;
    int skippedInvalidRemovalSolidCount = 0;
    double planarFootprintArea = 0.0;
    double maximumPlanarFootprintArea = 0.0;
    int entrySweepGroupCount = 0;
    int containedEntrySweepGroupCount = 0;
    int containedEntrySweepSegmentCount = 0;
    int fallbackSweepSegmentCount = 0;
    int linearRampMoveCount = 0;
    int helicalRampMoveCount = 0;
    int rampSweepSliceCount = 0;
    double maximumRampAxialError = 0.0;
    int toolProfileBandCount = 0;
    double maximumToolProfileAxialError = 0.0;
    double maximumToolProfileRadialError = 0.0;
    double sweepInflationMilliseconds = 0.0;
    int analyticPlanarFallbackOperationCount = 0;
    int analyticPlanarPrimitiveCount = 0;
    int analyticPlanarArcCount = 0;
    int analyticPlanarBooleanGroupCount = 0;
    double analyticPlanarMilliseconds = 0.0;
    bool inferredCuttingEdgeLength = false;
    double effectiveCuttingEdgeLength = 0.0;
    int booleanBatchCount = 0;
    bool parallelProcessingEnabled = false;
    // Configured worker ceiling, not an observed OS-thread peak. Shared-budget
    // contention and backend admission may reduce actual parallel work.
    int parallelWorkerCount = 1;
    bool heightFieldSimulation = false;
    double zMapResolution = 0.0;
    int zMapCellCount = 0;
    int zMapActiveCellCount = 0;
    int zMapTileCount = 0;
    double zMapUpdateMilliseconds = 0.0;
    // 仅腔 Z-map 仿真填充。比较阶段直接复用该网格，避免对棱柱快照
    // 再执行精确 OCCT 布尔；孔仿真保持为空。
    std::shared_ptr<const PocketZMapSimulationData> pocketZMapData;
    int processedOperationCount = 0;
    int appliedOperationCount = 0;
    int noMaterialOperationCount = 0;
    bool cancelled = false;
    bool complete = false;
};

} // namespace oap::toolpath
