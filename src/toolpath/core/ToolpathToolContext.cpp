// 工序级刀具冷却与加工阶段枚举的稳定名称映射。
#include "ToolpathToolContext.h"

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace oap::toolpath {

const char* holeMachiningPhaseName(HoleMachiningPhase phase) noexcept
{
    switch (phase) {
        case HoleMachiningPhase::Standard: return "standard";
        case HoleMachiningPhase::CenterSpot: return "center_spot";
        case HoleMachiningPhase::Drill: return "drill";
        case HoleMachiningPhase::FlatEndMill: return "flat_end_mill";
    }
    return "standard";
}

namespace {

void appendNumber(std::ostringstream& output, double value)
{
    output << '|';
    if (std::isnan(value)) {
        output << "nan";
    } else if (std::isinf(value)) {
        output << (value < 0.0 ? "-inf" : "+inf");
    } else if (value == 0.0) {
        output << '0';
    } else {
        output << std::setprecision(std::numeric_limits<double>::max_digits10)
               << value;
    }
}

} // namespace

std::string toolpathToolContextKey(const ToolpathToolContext& tool)
{
    // 快照键：按固定顺序串联全部字段，字符串先写长度再写内容以消除
    // 分隔符歧义；浮点使用 max_digits10 保证同一 double 稳定往返。
    // 该值仅供哈希/去重使用，不属于持久化格式，版本号 v1 允许演进。
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "tool-context-v1|" << tool.valid
           << '|' << tool.toolNumber
           << '|' << tool.toolId.size() << ':' << tool.toolId
           << '|' << tool.toolName.size() << ':' << tool.toolName
           << '|' << tool.toolType.size() << ':' << tool.toolType;
    appendNumber(output, tool.diameter);
    appendNumber(output, tool.cornerRadius);
    appendNumber(output, tool.spindleSpeed);
    appendNumber(output, tool.horizontalFeed);
    appendNumber(output, tool.verticalFeed);
    appendNumber(output, tool.horizontalRapid);
    appendNumber(output, tool.verticalRapid);
    appendNumber(output, tool.cuttingEdgeLength);
    appendNumber(output, tool.overallLength);
    appendNumber(output, tool.shankDiameter);
    appendNumber(output, tool.holderDiameter);
    appendNumber(output, tool.holderLength);
    appendNumber(output, tool.spindleEnvelopeDiameter);
    appendNumber(output, tool.spindleEnvelopeLength);
    appendNumber(output, tool.tipAngle);
    appendNumber(output, tool.tipCompensation);
    appendNumber(output, tool.peckDepth);
    output << '|' << static_cast<int>(tool.drillCycle)
           << '|' << static_cast<int>(tool.coolant);
    return output.str();
}

} // namespace oap::toolpath
