// 中立 ToolpathProgram 的参考 NC 后处理实现；只解释程序模型，
// 不感知孔或槽腔生成器的内部结果类型。
//
// 模块归属:oap_toolpath_program(GCode 后处理与程序导出)。
// 职责:把中立的 ToolpathProgram 渲染为与仓库 G-code 参考文件一致的
// 轻量 NC 方言,包括注释清洗、世界→工件坐标变换、钻削固定循环
// (G81/G82/G83/G73/G84)与圆弧(G2/G3)的状态机输出,以及尾部
// 刀具/最低点/时间摘要。本文件不持有任何状态,write() 为纯函数式
// 调用,可在任意线程中安全执行。
#include "GCodePostProcessor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>

#include <gp_Vec.hxx>

namespace oap::toolpath {

namespace {


// 把任意文本清洗为可安全放入 G-code 括号注释的内容:括号会导致
// 注释提前闭合,统一替换为方括号;回车/换行会破坏单行注释结构,
// 替换为空格。按值传参以便就地修改副本。
std::string safeComment(std::string text)
{
    std::replace(text.begin(), text.end(), '(', '[');
    std::replace(text.begin(), text.end(), ')', ']');
    std::replace(text.begin(), text.end(), '\r', ' ');
    std::replace(text.begin(), text.end(), '\n', ' ');
    return text;
}

// 把世界坐标点投影到工件坐标系:以 frame 原点为原点,对三根坐标轴
// 求点积得到各分量(要求 frame 为右手正交坐标系)。
gp_Pnt toWorkCoordinates(const gp_Pnt& point, const gp_Ax3& frame)
{
    const gp_Vec offset(frame.Location(), point);
    return {offset.Dot(gp_Vec(frame.XDirection())),
            offset.Dot(gp_Vec(frame.YDirection())),
            offset.Dot(gp_Vec(frame.Direction()))};
}

// 按装夹配置把世界坐标点转换到工件坐标:若显式提供了世界→工件变换
// 则直接变换,否则退化为按 workFrame 的三轴投影。
gp_Pnt toWorkCoordinates(
    const gp_Pnt& point,
    const MachiningSetup& setup)
{
    if (setup.hasWorldToWorkTransform)
        return point.Transformed(setup.worldToWorkTransform);
    return toWorkCoordinates(point, setup.workFrame);
}

// 把世界坐标向量投影到工件坐标(与 toWorkCoordinates 的三轴点积
// 规则一致),用于方向类数据(如钻削轴、圆弧轴向)。
gp_Vec toWorkVector(const gp_Vec& vector, const gp_Ax3& frame)
{
    return {vector.Dot(gp_Vec(frame.XDirection())),
            vector.Dot(gp_Vec(frame.YDirection())),
            vector.Dot(gp_Vec(frame.Direction()))};
}

// 以下为后处理输出前的一致性谓词:有限点/向量、沿工件 Z 轴的向量、
// 正的有限值。G-code 要求所有坐标可打印且方向可判读,非有限或
// 近乎退化的数据在输出前必须被拒绝。
bool finitePoint(const gp_Pnt& point)
{
    return std::isfinite(point.X()) && std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

bool finiteVector(const gp_Vec& vector)
{
    return std::isfinite(vector.X()) && std::isfinite(vector.Y()) &&
           std::isfinite(vector.Z());
}

bool isWorkZAxis(const gp_Vec& vector)
{
    if (!finiteVector(vector) || vector.SquareMagnitude() <= 1.0e-16)
        return false;
    gp_Vec normalized = vector;
    normalized.Normalize();
    return std::abs(normalized.X()) <= 1.0e-7 &&
           std::abs(normalized.Y()) <= 1.0e-7 &&
           std::abs(std::abs(normalized.Z()) - 1.0) <= 1.0e-7;
}

bool positiveFinite(double value)
{
    return std::isfinite(value) && value > 0.0;
}

// 逐字段比较两次刀具快照是否完全一致。参考 NC 输出只允许一个刀具,
// 当程序按操作绑定刀具(per-operation tool)时用它校验所有操作共享
// 同一把刀具。
bool sameReferenceToolSnapshot(const ToolpathToolContext& first,
                               const ToolpathToolContext& second)
{
    return first.toolId == second.toolId &&
           first.toolName == second.toolName &&
           first.toolType == second.toolType &&
           first.diameter == second.diameter &&
           first.cornerRadius == second.cornerRadius &&
           first.spindleSpeed == second.spindleSpeed &&
           first.horizontalFeed == second.horizontalFeed &&
           first.verticalFeed == second.verticalFeed &&
           first.horizontalRapid == second.horizontalRapid &&
           first.verticalRapid == second.verticalRapid &&
           first.tipAngle == second.tipAngle &&
           first.tipCompensation == second.tipCompensation &&
           first.peckDepth == second.peckDepth &&
           first.drillCycle == second.drillCycle &&
           first.coolant == second.coolant;
}

} // namespace



namespace {

// 世界→工件坐标的向量版本:优先使用显式变换矩阵,否则按 workFrame
// 三轴投影(与上文的坐标点版本对称)。
gp_Vec toWorkVector(const gp_Vec& vector, const MachiningSetup& setup)
{
    if (!setup.hasWorldToWorkTransform)
        return toWorkVector(vector, setup.workFrame);
    gp_Vec result = vector;
    result.Transform(setup.worldToWorkTransform);
    return result;
}

// 定点格式输出数值:先把绝对值小于半格(即会被舍入为 0)的微小值
// 归零,避免输出 -0.000 之类的外观噪声。
std::string referenceFixed(double value, int precision)
{
    const int clampedPrecision = std::max(0, precision);
    if (std::abs(value) < 0.5 * std::pow(10.0, -clampedPrecision))
        value = 0.0;
    std::ostringstream text;
    text << std::fixed << std::setprecision(clampedPrecision) << value;
    return text.str();
}

// 参考 NC 的数字排版:定点格式后去掉末尾多余的 0;若没有小数点则补
// 一个(多数控制器把 X1 与 X1.0 视为不同词法)。
std::string referenceNumber(double value, int precision)
{
    std::string text = referenceFixed(value, precision);
    const std::size_t decimal = text.find('.');
    if (decimal == std::string::npos)
        return text + '.';
    while (text.size() > decimal + 1 && text.back() == '0')
        text.pop_back();
    return text;
}

// 生成文件头注释用的当前日期(YYYY-MM-DD),按本地时区格式化。
std::string referenceDate()
{
    const std::time_t now = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    std::ostringstream value;
    value << std::put_time(&local, "%Y-%m-%d");
    return value.str();
}

// 把中立的钻削循环类型映射为参考 NC 的固定循环 G 代码。
const char* referenceCycleCode(ToolpathDrillCycleType type)
{
    switch (type) {
        case ToolpathDrillCycleType::Drilling: return "G81";
        case ToolpathDrillCycleType::Dwell: return "G82";
        case ToolpathDrillCycleType::Peck: return "G83";
        case ToolpathDrillCycleType::ChipBreak: return "G73";
        case ToolpathDrillCycleType::Tapping: return "G84";
    }
    return "G81";
}

// 参考 NC 输出所需的「单刀具摘要」:从操作绑定刀具快照或导出选项/参数
// 回退构造,后续所有进给、转速与刀具注释都取自这里。
struct ReferenceTool {
    std::string id;
    std::string name;
    double diameter = 0.0;
    double cornerRadius = 0.0;
    double spindleSpeed = 0.0;
    double horizontalFeed = 0.0;
    double verticalFeed = 0.0;
    double horizontalRapid = 0.0;
    double verticalRapid = 0.0;
    ToolpathCoolantMode coolant = ToolpathCoolantMode::Off;
};

// 构造单刀具摘要:perOperationTools 为真时取第一个操作绑定的刀具快照
// (调用方已保证各操作一致),否则回退到 options 的刀具标识与
// params.tool 的工艺参数;刀具名缺失时用 id 或 "ManualTool" 兜底。
ReferenceTool referenceTool(
    const ToolpathProgram& program,
    const ToolpathParams& params,
    const GCodeProgramOptions& options,
    bool perOperationTools)
{
    ReferenceTool result;
    if (perOperationTools && !program.operations.empty()) {
        const ToolpathToolContext& source = program.operations.front().tool;
        result.id = source.toolId;
        result.name = source.toolName;
        result.diameter = source.diameter;
        result.cornerRadius = source.cornerRadius;
        result.spindleSpeed = source.spindleSpeed;
        result.horizontalFeed = source.horizontalFeed;
        result.verticalFeed = source.verticalFeed;
        result.horizontalRapid = source.horizontalRapid;
        result.verticalRapid = source.verticalRapid;
        result.coolant = source.coolant;
    } else {
        result.id = options.toolId;
        result.name = options.toolName;
        result.diameter = params.tool.diameter;
        result.cornerRadius = params.tool.cornerRadius;
        result.spindleSpeed = params.tool.spindleSpeed;
        result.horizontalFeed = params.tool.horizFeed;
        result.verticalFeed = params.tool.vertFeed;
        result.horizontalRapid = params.tool.horizRapid;
        result.verticalRapid = params.tool.vertRapid;
        result.coolant = options.coolant ? ToolpathCoolantMode::Flood
                                         : ToolpathCoolantMode::Off;
    }
    if (result.name.empty())
        result.name = !result.id.empty() ? result.id : "ManualTool";
    return result;
}

// 判断一个操作能否紧凑地表达为单个固定循环:仅当操作内至多一个
// DrillCycle 原语、且其前只允许规划器生成的直线安全连接时返回该循环。
const ToolpathDrillCycle* compactReferenceCycle(
    const ToolpathOperation& operation)
{
    const ToolpathDrillCycle* result = nullptr;
    bool foundCycle = false;
    for (const ToolpathPrimitive& primitive : operation.primitives) {
        if (primitive.type == ToolpathPrimitiveType::DrillCycle) {
            if (foundCycle)
                return nullptr;
            foundCycle = true;
            result = &primitive.drillCycle;
        } else if (foundCycle ||
                   primitive.type != ToolpathPrimitiveType::Linear) {
            // 循环前只允许规划器生成的线性安全连接，循环后不能丢弃动作。
            return nullptr;
        }
    }
    return result;
}

// 解析单个移动的进给:移动自带有效进给时优先采用,否则按移动类型
// (垂直方向用垂直进给,其余用水平进给)从刀具摘要回退。
double referenceFeed(const ToolpathMove& move, const ReferenceTool& tool)
{
    if (positiveFinite(move.feed))
        return move.feed;
    return move.type == ToolpathMoveType::Plunge ||
           move.type == ToolpathMoveType::Retract
        ? tool.verticalFeed : tool.horizontalFeed;
}

// 粗略估算整个程序的加工时长(秒):按每段移动的位移/进给累加,快速
// 移动使用刀具摘要或 7000 mm/min 兜底;另外累加 Dwell 循环的停顿秒数。
// 结果只用于输出文件尾部的 (Total time) 摘要,不用于任何工艺决策。
double referenceEstimatedSeconds(
    const ToolpathProgram& program,
    const ReferenceTool& tool)
{
    double fallbackRapid = 7000.0;
    if (positiveFinite(tool.horizontalRapid) &&
        positiveFinite(tool.verticalRapid)) {
        fallbackRapid = std::min(tool.horizontalRapid,
                                 tool.verticalRapid);
    } else if (positiveFinite(tool.horizontalRapid)) {
        fallbackRapid = tool.horizontalRapid;
    } else if (positiveFinite(tool.verticalRapid)) {
        fallbackRapid = tool.verticalRapid;
    }
    double seconds = 0.0;
    for (const ToolpathOperation& operation : program.operations) {
        for (const ToolpathMove& move : operation.moves) {
            double feed = referenceFeed(move, tool);
            if (move.type == ToolpathMoveType::Rapid) {
                feed = positiveFinite(move.feed) ? move.feed : fallbackRapid;
            }
            if (positiveFinite(feed))
                seconds += 60.0 * move.from.Distance(move.to) / feed;
        }
        for (const ToolpathPrimitive& primitive : operation.primitives) {
            if (primitive.type == ToolpathPrimitiveType::DrillCycle &&
                primitive.drillCycle.type == ToolpathDrillCycleType::Dwell &&
                primitive.drillCycle.dwellSeconds > 0.0) {
                seconds += primitive.drillCycle.dwellSeconds;
            }
        }
    }
    return seconds;
}

} // namespace


bool ReferenceNcGCodePostProcessor::write(
    std::ostream& output,
    const ToolpathProgram& program,
    const ToolpathParams& params,
    const MachiningSetup& setup,
    const GCodeProgramOptions& options,
    std::string& error) const
{
    error.clear();
    if (!output) {
        error = "G-code output stream is not writable";
        return false;
    }
    if (program.operations.empty()) {
        error = "reference NC export requires at least one operation";
        return false;
    }
    if (setup.workOffset == WorkCoordinateSystem::None) {
        error = "reference NC export requires a G54-G59 work offset";
        return false;
    }
    // 前置校验:参考 NC 输出要求整个程序只含一个装夹、一个孔加工阶段
    // 与一把刀具(刀具一致性在下文按工具模式分别校验),否则拒绝导出。
    const int setupIndex = program.operations.front().setupIndex;
    const MoldSetupFace setupFace = program.operations.front().setupFace;
    const HoleMachiningPhase phase = program.operations.front().holePhase;
    for (const ToolpathOperation& operation : program.operations) {
        const bool conflictingFace =
            setupFace != MoldSetupFace::Unresolved &&
            operation.setupFace != MoldSetupFace::Unresolved &&
            operation.setupFace != setupFace;
        if (operation.setupIndex != setupIndex || conflictingFace) {
            error = "reference NC output must contain exactly one setup";
            return false;
        }
        if (operation.holePhase != phase) {
            error = "reference NC output must contain exactly one hole-machining phase";
            return false;
        }
    }
    if (setup.setupFace != MoldSetupFace::Unresolved &&
        setupFace != MoldSetupFace::Unresolved &&
        setup.setupFace != setupFace) {
        error = "selected setup work frame does not match the reference NC program";
        return false;
    }

    // 工具模式决定刀具来源:PerOperation 时取操作绑定的刀具快照并校验
    // 全部一致;单刀具(全局)模式则从 options/params 回退。
    const OperationToolContextMode toolMode = operationToolContextMode(program);
    if (toolMode == OperationToolContextMode::Mixed) {
        error = "reference NC export rejects programs mixing bound and unbound operation tools";
        return false;
    }
    const bool perOperationTools =
        toolMode == OperationToolContextMode::PerOperation;
    if (perOperationTools) {
        const ToolpathToolContext& first = program.operations.front().tool;
        for (const ToolpathOperation& operation : program.operations) {
            if (!sameReferenceToolSnapshot(first, operation.tool)) {
                error = "reference NC output must contain one consistent tool snapshot";
                return false;
            }
        }
    }
    const ReferenceTool tool = referenceTool(
        program, params, options, perOperationTools);
    if (!positiveFinite(tool.diameter)) {
        error = "reference NC requires a positive tool diameter";
        return false;
    }
    if (!std::isfinite(tool.spindleSpeed) || tool.spindleSpeed < 0.0) {
        error = "reference NC requires a finite non-negative spindle speed";
        return false;
    }
    const bool outputSpindle = positiveFinite(tool.spindleSpeed);
    const bool validCornerRadius = std::isfinite(tool.cornerRadius) &&
        tool.cornerRadius >= 0.0 &&
        tool.cornerRadius <= 0.5 * tool.diameter + 1.0e-6;
    const double outputCornerRadius = validCornerRadius
        ? tool.cornerRadius : 0.0;
    const bool coolant = options.coolant ||
        tool.coolant != ToolpathCoolantMode::Off;

    constexpr int precision = 3;
    const auto number = [precision](double value) {
        return referenceNumber(value, precision);
    };
    // 预扫描一遍全部移动:转换到工件坐标后校验有限性,并累计最低点,
    // 供文件尾部的 (Zmin) 摘要使用。
    double minimumZ = std::numeric_limits<double>::infinity();
    for (const ToolpathOperation& operation : program.operations) {
        for (const ToolpathMove& move : operation.moves) {
            const gp_Pnt from = toWorkCoordinates(move.from, setup);
            const gp_Pnt to = toWorkCoordinates(move.to, setup);
            if (!finitePoint(from) || !finitePoint(to)) {
                error = "toolpath contains a non-finite point after work-coordinate conversion";
                return false;
            }
            minimumZ = std::min({minimumZ, from.Z(), to.Z()});
        }
    }

    // 文件头:百分比号分隔 + 日期/程序名/警告注释。警告明确标注这是
    // 参考输出,不可直接上机,并提示工件坐标系确认状态。
    output << "%\n"
           << "(Date=" << referenceDate() << ")\n"
           << "(NAME: " << safeComment(options.programName) << ")\n";
    output << "(WARNING: REFERENCE NC - NOT FOR MACHINE EXECUTION)\n"
           << "(WARNING: production controller and commissioning checks are not included in this reference output)\n";
    if (setup.requiresManualConfirmation && !setup.workFrameConfirmed) {
        output << "(WARNING: setup work frame has not been manually confirmed)\n";
    }
    if (!validCornerRadius) {
        output << "(WARNING: invalid tool corner radius omitted from reference comment)\n";
    }
    output << "(Manual tool load and length-offset verification required: no T/M6/G43)\n"
           << "G40 G17 G49 G80 G90\n"
           << "G21\n";
    output << "G91 G28 Z0.\n"
           << "(ToolName=" << safeComment(tool.name)
           << " D.=" << referenceFixed(tool.diameter, 2)
           << " R.=" << referenceFixed(outputCornerRadius, 2) << ")\n"
           << "G00 G90 " << workCoordinateCode(setup.workOffset)
           << " X0. Y0.\n";
    if (outputSpindle) {
        output << 'S' << referenceFixed(tool.spindleSpeed, 0) << " M03\n";
    } else {
        output << "(WARNING: spindle speed is unspecified; "
                  "S/M03/M05 are omitted)\n";
    }
    if (coolant)
        output << "M07\n";

    // —— 输出状态机 ——
    // hasCurrent/current 跟踪当前刀尖位置,决定是否需要先快速移动;
    // cycleActive 等记录上一个固定循环的激活状态与返回平面,以便用
    // G99(返回 R 平面)连续加工多个同型孔而避免反复取消/重建循环。
    bool hasCurrent = false;
    gp_Pnt current;
    bool cycleActive = false;
    bool previousCycleReturnedG99 = false;
    ToolpathDrillCycleType activeCycleType =
        ToolpathDrillCycleType::Drilling;
    const auto cancelCycle = [&] {
        if (cycleActive)
            output << "G80\n";
        cycleActive = false;
        previousCycleReturnedG99 = false;
    };
    const auto writeXYZ = [&](const gp_Pnt& point) {
        output << " X" << number(point.X())
               << " Y" << number(point.Y())
               << " Z" << number(point.Z());
    };
    // 输出单段直线移动:先按需快速移动到位,再按移动类型决定 G1/G0;
    // 进给仅在切削/下刀/带进给的退刀时输出。
    const auto emitMove = [&](const ToolpathMove& move) -> bool {
        const gp_Pnt from = toWorkCoordinates(move.from, setup);
        const gp_Pnt to = toWorkCoordinates(move.to, setup);
        if (!finitePoint(from) || !finitePoint(to)) {
            error = "toolpath contains a non-finite point after work-coordinate conversion";
            return false;
        }
        if (!hasCurrent || current.Distance(from) > 1.0e-6) {
            output << "G0";
            writeXYZ(from);
            output << '\n';
        }
        const bool feedMove = move.type == ToolpathMoveType::Cut ||
            move.type == ToolpathMoveType::Plunge ||
            (move.type == ToolpathMoveType::Retract && move.feed > 0.0);
        output << (feedMove ? "G1" : "G0");
        writeXYZ(to);
        const double feed = referenceFeed(move, tool);
        if (feedMove) {
            if (!positiveFinite(feed)) {
                error = "reference NC cutting moves require a positive feed";
                return false;
            }
            output << " F" << number(feed);
        }
        output << '\n';
        current = to;
        hasCurrent = true;
        return true;
    };

    for (const ToolpathOperation& operation : program.operations) {
        // 优先路径:操作被识别为单个紧凑固定循环时,走固定循环分支
        // (钻孔/锪孔/攻丝等),否则退回逐原语输出。
        const ToolpathDrillCycle* cycle =
            compactReferenceCycle(operation);
        if (cycle) {
            // 循环坐标轴全部换算到工件坐标后校验:轴必须平行于负工件
            // Z,且高度满足 clearance >= R >= entry > bottom。
            const gp_Pnt clearance = toWorkCoordinates(cycle->clearance, setup);
            const gp_Pnt retract = toWorkCoordinates(cycle->retract, setup);
            const gp_Pnt entry = toWorkCoordinates(cycle->entry, setup);
            const gp_Pnt bottom = toWorkCoordinates(cycle->bottom, setup);
            const gp_Vec axis = toWorkVector(cycle->axis, setup);
            if (!finitePoint(clearance) || !finitePoint(retract) ||
                !finitePoint(entry) || !finitePoint(bottom) ||
                !finiteVector(axis) ||
                !isWorkZAxis(axis) || axis.Z() >= 0.0) {
                error = "reference NC drill cycles must be finite and aligned with negative work Z";
                return false;
            }
            if (clearance.Z() + 1.0e-6 < retract.Z() ||
                retract.Z() + 1.0e-6 < entry.Z() ||
                entry.Z() <= bottom.Z() + 1.0e-6) {
                error = "reference NC requires planned clearance >= R >= entry > bottom";
                return false;
            }
            if ((cycle->type == ToolpathDrillCycleType::Peck ||
                 cycle->type == ToolpathDrillCycleType::ChipBreak) &&
                !positiveFinite(cycle->peckDepth)) {
                error = "reference NC peck cycles require a positive Q depth";
                return false;
            }
            if (!positiveFinite(cycle->feed)) {
                error = "reference NC drill cycles require a positive feed";
                return false;
            }

            // 若上一个循环以 G99 结束且类型相同,可直接续写新循环而
            // 无需取消;否则先取消已激活循环、输出循环前的安全连接并
            // 移动到 clearance 平面。
            const bool continueFromG99 =
                previousCycleReturnedG99 && cycleActive &&
                activeCycleType == cycle->type;
            if (previousCycleReturnedG99 && !continueFromG99)
                cancelCycle();

            if (!continueFromG99) {
                cancelCycle();
                for (const ToolpathPrimitive& primitive :
                     operation.primitives) {
                    if (primitive.type ==
                        ToolpathPrimitiveType::DrillCycle) {
                        break;
                    }
                    if (!emitMove(primitive.linear))
                        return false;
                }
                if (!hasCurrent || current.Distance(clearance) > 1.0e-6) {
                    output << "G0";
                    writeXYZ(clearance);
                    output << '\n';
                    current = clearance;
                    hasCurrent = true;
                }
            }

            // 按返回平面选择 G98(回初始平面)或 G99(回 R 平面),输出
            // 固定循环并附 Q(啄钻步进)/P(停顿秒数)。循环结束后更新
            // 状态:当前点按返回平面取值;若返回 R 平面则保持循环激活以
            // 支持下一个孔的连续加工。
            const bool returnToInitialPlane =
                cycle->retractMode ==
                    ToolpathDrillRetractMode::InitialPlane;
            output << (returnToInitialPlane ? "G98 " : "G99 ")
                   << referenceCycleCode(cycle->type)
                   << " X" << number(bottom.X())
                   << " Y" << number(bottom.Y())
                   << " Z" << number(bottom.Z())
                   << " R" << number(retract.Z())
                   << " F" << number(cycle->feed);
            if (cycle->type == ToolpathDrillCycleType::Peck ||
                cycle->type == ToolpathDrillCycleType::ChipBreak)
                output << " Q" << number(cycle->peckDepth);
            if (cycle->type == ToolpathDrillCycleType::Dwell &&
                cycle->dwellSeconds > 0.0)
                output << " P" << number(cycle->dwellSeconds);
            output << '\n';

            cycleActive = true;
            activeCycleType = cycle->type;
            previousCycleReturnedG99 = !returnToInitialPlane;
            current = returnToInitialPlane ? clearance : retract;
            hasCurrent = true;
            minimumZ = std::min(minimumZ, bottom.Z());
            if (!previousCycleReturnedG99)
                cancelCycle();
            continue;
        }

        // 非固定循环操作:先取消激活的循环,再逐原语输出;纯移动操作
        // 直接输出全部 moves。
        cancelCycle();
        if (operation.primitives.empty()) {
            for (const ToolpathMove& move : operation.moves) {
                if (!emitMove(move)) return false;
            }
            continue;
        }
        for (const ToolpathPrimitive& primitive : operation.primitives) {
            if (primitive.type == ToolpathPrimitiveType::Linear) {
                if (!emitMove(primitive.linear)) return false;
                continue;
            }
            // 圆弧原语:换算到工件坐标并校验轴向平行于工件 Z;输出
            // G2/G3 与圆心增量 I/J。轴向为负 Z 时顺时针/逆时针互换
            //(参考 NC 的顺时针约定面向正 Z 观测)。
            if (primitive.type == ToolpathPrimitiveType::CircularArc) {
                const ToolpathCircularArc& arc = primitive.arc;
                const gp_Pnt from = toWorkCoordinates(arc.from, setup);
                const gp_Pnt to = toWorkCoordinates(arc.to, setup);
                const gp_Pnt center = toWorkCoordinates(arc.center, setup);
                const gp_Vec axis = toWorkVector(arc.axis, setup);
                if (!finitePoint(from) || !finitePoint(to) ||
                    !finitePoint(center) || !isWorkZAxis(axis)) {
                    error = "reference NC arcs must be finite and parallel to work Z";
                    return false;
                }
                if (!hasCurrent || current.Distance(from) > 1.0e-6) {
                    output << "G0";
                    writeXYZ(from);
                    output << '\n';
                }
                bool clockwise =
                    arc.direction == ToolpathArcDirection::Clockwise;
                if (axis.Z() < 0.0) clockwise = !clockwise;
                output << (clockwise ? "G2" : "G3");
                writeXYZ(to);
                output << " I" << number(center.X() - from.X())
                       << " J" << number(center.Y() - from.Y());
                if (positiveFinite(arc.feed))
                    output << " F" << number(arc.feed);
                output << '\n';
                current = to;
                hasCurrent = true;
                continue;
            }
            for (const ToolpathMove& move : primitive.fallbackMoves) {
                if (!emitMove(move)) return false;
            }
            // 无法紧凑表达为固定循环的 Dwell 原语,退化为显式暂停 G4。
            if (primitive.type == ToolpathPrimitiveType::DrillCycle &&
                primitive.drillCycle.type == ToolpathDrillCycleType::Dwell &&
                primitive.drillCycle.dwellSeconds > 0.0) {
                output << "G4 P" << number(
                    primitive.drillCycle.dwellSeconds) << '\n';
            }
        }
    }

    // 结尾:取消固定循环,停主轴/关冷却,回到参考原点,收尾 M30。
    // 之后追加刀具名/最低点/预估时间的摘要注释。
    cancelCycle();
    if (outputSpindle)
        output << "M05\n";
    if (coolant)
        output << "M09\n";
    output << "G91 G28 Z0.0\n"
           << "G28 Y0.0\n"
           << "M30\n"
           << "%\n";
    if (!std::isfinite(minimumZ)) minimumZ = 0.0;
    const double seconds = referenceEstimatedSeconds(program, tool);
    const long long minutes = seconds > 0.0
        ? static_cast<long long>(std::ceil(seconds / 60.0)) : 0;
    output << "(Tool name: " << safeComment(tool.name) << ")\n"
           << "(Zmin: " << referenceFixed(minimumZ, 2) << ")\n"
           << "(Total time : " << minutes << ")\n";
    if (!output) {
        error = "failed while writing reference NC output";
        return false;
    }
    return true;
}

} // namespace oap::toolpath
