// 孔、槽腔与外轮廓的通用工艺参数模型；各加工类型可持有独立实例。
#pragma once

// 刀路工艺参数 —— 把 PocketRegion(纯 2D 区域)转成真实刀路所需的机加工信息。
// 对标 FreeCAD CAM 的三组来源:ToolController(刀具/进给)、AREA_PARAMS_POCKET(清腔)、
// PathUtils.depth_params(深度分层)。当前用于槽/腔清腔与孔钻削的几何、移动段和
// 基础 G-code 导出。

namespace oap::toolpath {

// 清腔填充图案。Offset 为回字形等距环,ZigZag 为平行往复,Spiral 为连续螺旋。
// Spiral 遇到多岛或复杂几何时会保守回退到 Offset 并在结果里标注。
enum class ClearingPattern {
    Offset,  // 回字形:沿边界向内等距偏置成一组同心闭合环
    ZigZag,  // Z字形:在刀心可达区内生成平行往复开放折线
    Spiral,  // 螺旋形:把偏置环 morph 成连续开放螺旋
    FreePath, // 自适应清腔:与 pyocc-exp 对齐的残料/包角反馈路径
};

// FreePath 的二维自适应规划参数。默认值与 pyocc-exp 原型保持一致；
// 这些参数只改变平面清料策略，不改变公共分层、进退刀和安全验证语义。
struct AdaptiveClearingParams {
    double targetEngagement = 0.10;          // 刀具圆盘材料占比目标，[0, 1]
    double targetEngagementAngleDeg = 70.0;  // 目标最大连续包角
    double maximumEngagementAngleDeg = 150.0;// 允许的最大连续包角
    int engagementAngleSamples = 72;         // 刀具圆周离散采样数
    double minimumTurnRadiusFactor = 0.60;    // 最小转弯半径/刀具直径
    double remainingTolerance = 0.015;        // 允许的最终栅格残料比例
    double materialCellSize = 0.0;            // 0=按步距和刀径自动选择
    int maximumAdaptivePasses = 60;
    int lookaheadDepth = 3;                   // 1=关闭多步前瞻
    int beamWidth = 4;
};

// 回字形/螺旋形清料路径的径向执行顺序。它独立于 CutMode。
// Adaptive FreePath 与 pyocc-exp 一致，始终按中心向外的规划顺序执行，
// 不消费这个公共兼容参数。
enum class PocketClearingDirection {
    OutsideToInside,
    InsideToOutside,
};

// 把公共参数转换为图案真实支持的语义。旧项目中的 FreePath 可能仍保存
// OutsideToInside；加载、生成和再次保存时一律迁移为 Python 的中心向外顺序。
constexpr PocketClearingDirection effectivePocketClearingDirection(
    ClearingPattern pattern,
    PocketClearingDirection requested) noexcept
{
    return pattern == ClearingPattern::FreePath
        ? PocketClearingDirection::InsideToOutside
        : requested;
}

// 顺铣/逆铣。决定偏置环的走刀方向(对标 FreeCAD orientation Normal/Reversed)。
// 仅影响每个环的点序方向,不影响环本身的几何。
enum class CutMode {
    Climb,        // 顺铣
    Conventional, // 逆铣
};

// 外轮廓加工策略。ExternalClearing 会先清除毛坯外框与成品包络之间的材料，
// 再按最终侧壁余量走精加工轮廓；ContourOnly 保留旧版固定径向圈语义。
enum class ProfileMachiningMode {
    ExternalClearing = 0,
    ContourOnly = 1,
};

// 槽/腔进入切削层的方式。斜坡角均相对局部水平面定义，角度越小，
// 为达到相同切深所需的水平路径越长。
enum class PocketEntryMode {
    VerticalPlunge = 0,
    // 数值 1 保持旧项目兼容：旧“线性斜坡”实际是沿清料轮廓斜坡。
    RampOnShape = 1,
    HelicalRamp = 2,
    LinearRamp = 3,
    ZigZagRamp = 4,
};

enum class PocketRampStartReference {
    PreviousLevel = 0,
    OpeningPlane = 1,
    SafePlane = 2,
};

// 模具局部坐标中的孔访问起始角，仅影响同一装夹内的访问顺序。
enum class HoleStartCorner {
    LowerLeft,
    LowerRight,
    UpperLeft,
    UpperRight,
};

// 中立钻削循环偏好；后处理可映射为 G81/G83/G73，或展开为显式移动。
enum class HoleDrillCycleMode {
    Auto,
    Direct,
    Peck,
    ChipBreak,
};

// 固定循环完成后的返回高度。InitialPlane 对应 G98，RetractPlane 对应 G99。
enum class HoleDrillRetractMode {
    InitialPlane,
    RetractPlane,
};

// 孔间快移碰撞包络。CutterOnly 保留旧版刀具半径检查；ToolAssembly
// 同时检查已知的切削部、刀杆、刀柄/夹头和主轴头轴向分段包络。
// 未知装配段会产生警告并从包络中省略，不阻止刀路或 G-code 生成。
enum class HoleCollisionEnvelopeMode {
    CutterOnly,
    ToolAssembly,
    ToolAssemblySolidSweep,
};

// 刀具定义。对标 FreeCAD ToolController + Tool；直径和刀角半径进入刀具
// 快照及 NC 注释，进给/转速字段供生成、仿真与后处理共同使用。
struct ToolDef {
    double diameter = 6.0;      // 刀具直径 mm
    double cornerRadius = 0.0;  // 刀尖圆角半径 mm
    double horizFeed = 600.0;   // 水平进给 mm/min
    double vertFeed = 200.0;    // 垂直(下刀)进给 mm/min
    double horizRapid = 0.0;    // 水平快速移动 mm/min(0=机床默认 G0)
    double vertRapid = 0.0;     // 垂直快速移动 mm/min
    double spindleSpeed = 0.0;  // 主轴转速 rpm(0=不输出 S 指令)
    double cuttingEdgeLength = 0.0; // 刀尖起算的切削刃长度；0 表示未知。
    double overallLength = 0.0;     // 刀尖到刀柄端面的伸出长度；0 表示未知。
    double shankDiameter = 0.0;     // 刀杆直径；0 时按刀具直径处理。
    double holderDiameter = 0.0;    // 刀柄/夹头碰撞直径；0 表示未知。
    double holderLength = 0.0;      // 刀柄从总长端面向外的建模长度。
    double spindleEnvelopeDiameter = 0.0; // 主轴头/鼻端保守碰撞直径。
    double spindleEnvelopeLength = 0.0;   // 从刀柄外端继续向主轴方向的长度。
    // 钻尖沿刀轴方向的直接补偿；大于 0 时优先于按刀径和钻尖角计算。
    double tipCompensation = 0.0;

    double radius() const { return 0.5 * diameter; }
};

// 深度/高度模型。对标 FreeCAD depth_params + ClearanceHeight/SafeHeight。
// 清腔深度以 PocketRegion 底面为 0,沿 region.openDir 朝开口方向为正；
// 安全/清空高度的输入值则以下刀开口面（毛坯顶面）为 0。
struct DepthParams {
    // 安全/清空高度为下刀面外侧距离；孔间避障还会按模型包络自动抬高。
    double clearanceHeight = 10.0; // 下刀面到清空面的距离
    double safeHeight = 5.0;       // 下刀面到进退刀安全面的距离
    bool useFeatureDepth = true;   // true=从识别到的槽/腔深度分层到底面
    double startDepth = 0.0;       // 手动起始偏移(mm),0=底面,正值朝开口方向
    double finalDepth = 0.0;       // 手动最终偏移(mm),通常为 0(腔底)
    double stepDown = 2.0;         // 每层最大切深;小孔一次到底时不使用
    double finishDepth = 0.0;      // 精修留量(0=不留)
};

// 清腔参数。对标 AREA_PARAMS_POCKET。
struct ClearingParams {
    ClearingPattern pattern = ClearingPattern::Offset;
    PocketClearingDirection expansionDirection =
        PocketClearingDirection::OutsideToInside;
    CutMode cutMode = CutMode::Climb;
    PocketEntryMode entryMode = PocketEntryMode::VerticalPlunge;
    PocketRampStartReference rampStartReference =
        PocketRampStartReference::PreviousLevel;

    // 线性/螺旋斜坡相对局部水平面的最大下刀角度。
    double rampAngleDeg = 3.0;
    // 允许用于一次斜坡下刀的最大刀心路径长度；0 表示仅受内部点数预算限制。
    double maximumRampLength = 0.0;
    // 螺旋下刀的刀心轨迹直径；0 表示按刀具直径自动选择。
    double helixDiameter = 0.0;
    // 往复斜坡单程允许使用的最大宽度；0 表示按刀具直径自动选择。
    double zigzagRampWidth = 0.0;
    // 斜坡进给；0 表示沿用刀具的垂直进给。
    double rampFeed = 0.0;

    // 步距:相邻偏置环的间距,按刀具直径的百分比表示(FreeCAD StepOver%)。
    // 实际间距 = diameter * stepOverPercent/100。0 或负值时回退为刀具半径(50%)。
    double stepOverPercent = 50.0;

    // 额外偏置:在刀具半径基础上再向内让刀(精修留量/避让公差),mm。
    double extraOffset = 0.0;

    // 自动多刀开粗内部参数。大于当前刀径时，只生成上一把较大平底刀
    // 加工后剩余、且当前刀能够触达的增量刀路；普通单刀生成保持 0。
    double restMachiningPreviousToolDiameter = 0.0;

    // 偏置离散化:Clipper 用整数运算,内部按此精度缩放(小数位数)。
    // 2 = 0.01mm 量化,对机加工足够;增大可提精度但放大坐标值。
    int precision = 3;

    // 圆角偏置离散公差(mm)。0 表示让 Clipper2 使用与坐标精度匹配的默认值。
    double arcTolerance = 0.0;

    // ZigZag 平行往复扫描线角度(度),0=沿局部 X 轴。仅 pattern==ZigZag 时生效。
    double zigzagAngle = 0.0;

    // 可选的 ZigZag 贴壁精修环。默认关闭,使 Z 字形只包含平行往复扫描线;
    // 仅在显式启用时才先走一圈闭合外轮廓。
    bool perimeterPass = false;

    AdaptiveClearingParams adaptive;
};

// 外轮廓/侧壁轮廓参数。语义参考 FreeCAD Profile：刀具中心位于零件
// outer wire 外侧；多次径向走刀从较远偏置逐步收敛到最终侧壁余量。
struct ProfileParams {
    ProfileMachiningMode machiningMode =
        ProfileMachiningMode::ExternalClearing;
    CutMode cutMode = CutMode::Climb;
    double sideAllowance = 0.0; // 最终侧壁余量，正值保留材料，mm
    double roughingAllowance = 0.2; // 外部清料后、精加工前保留的侧壁材料，mm
    bool finishPass = true;     // 外部清料完成后走最终轮廓
    int radialPassCount = 1;    // 1=仅最终轮廓；大于 1 时先远后近
    double radialStepOver = 0.0; // 相邻径向走刀间距；0=按 clearing.stepOverPercent
    double leadInLength = 0.0;  // 沿首段切向进入长度；0=关闭
    double leadOutLength = 0.0; // 沿末段切向退出长度；0=关闭
    int precision = 3;
    double arcTolerance = 0.0;
};

// 孔加工参数。小孔仍使用中心线钻削;中孔/大孔会按孔径自动切换为螺旋铣。
struct HoleMachiningParams {
    HoleStartCorner startCorner = HoleStartCorner::LowerLeft;
    double allowance = 0.0;       // 侧向余量(mm),负值按 0 处理
    // 孔径比刀具直径最多大多少仍按中心钻削处理，用于吸收尺寸取整与配合间隙。
    double smallHoleDiameterTolerance = 0.05;
    // 自动规划是否创建中心钻定位阶段；单孔段人工计划明确设置时优先。
    bool useCenterSpot = true;
    // 自动多刀工作流中的中心定位深度；仅在启用中心钻定位时生效。
    double centerSpotDepth = 1.0;
    // 通孔完整刀径越过出口面后的附加距离；实际钻尖目标还会叠加钻尖长度。
    double throughHoleBreakthrough = 0.5;
    double mediumHoleRadiusToolDiameterRatio = 0.95;
    HoleDrillCycleMode drillCycleMode = HoleDrillCycleMode::Auto;
    HoleDrillRetractMode drillRetractMode =
        HoleDrillRetractMode::InitialPlane;
    double peckDepth = 0.0;       // G83/G73 每次进给深度；0 表示禁用/自动。
    double dwellSeconds = 0.0;    // 普通钻削时大于 0 映射为 G82。
    double drillTipAngleDeg = 0.0; // 钻尖包含角；0 表示未知，常见值 118/135。
    // Auto 模式按 CAD 名义深度/刀径选择 G81、G73、G83。阈值相等可禁用 G73。
    double g81MaxDepthDiameterRatio = 2.0;
    double g73MaxDepthDiameterRatio = 4.0;
    double g73PeckDepth = 7.0;
    double g83PeckDepth = 20.0;
    double g73ChipBreakRetract = 0.25;
    double g83ReentryClearance = 0.01;
    // Operation 可强制保持 G98；允许 G99 时仍须通过相邻孔直连避障验证。
    bool allowG99RetractOptimization = true;
    // 刀具包络之外额外保留的物理碰撞距离，区别于 OCCT 数值容差。
    double collisionClearance = 0.0;
    HoleCollisionEnvelopeMode collisionEnvelopeMode =
        HoleCollisionEnvelopeMode::CutterOnly;
    double collisionEnvelopeSampleStep = 2.0; // 刀轴方向包络采样上限，mm。
    double helixPitch = 0.0;      // 中孔螺距(mm/rev),<=0 时回退 depth.stepDown/孔深
    double radialStep = 0.0;      // 大孔径向步距(mm/rev),<=0 时回退刀具步距百分比
    double sampleAngleDeg = 10.0; // 螺旋采样角度,会被夹在 min/max 之间
    double helixChordTolerance = 0.0; // mm，固定半径螺旋的弦高上限；0 保持旧角度采样行为。
    double minSampleAngleDeg = 3.0;
    double maxSampleAngleDeg = 20.0;
    int maxSamplePoints = 50000;  // 单个孔最多采样点预算,防止预览线段爆炸
};

// 单类加工使用的完整工艺输入（不包含具体特征几何）。
struct ToolpathParams {
    ToolDef tool;
    DepthParams depth;
    ClearingParams clearing;
    ProfileParams profile;
    HoleMachiningParams hole;
};

} // namespace oap::toolpath
