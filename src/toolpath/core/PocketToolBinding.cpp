// 槽腔工序刀具快照到通用刀路参数的绑定实现（oap_toolpath_core）。
// 只做字段复制与正数过滤，不推导任何工艺值；参数归一化由
// ToolpathParamNormalizer 在绑定之后统一完成。
#include "toolpath/core/PocketToolBinding.h"

#include <cmath>

namespace oap::toolpath {
namespace {

// 仅当源值为有限正数时覆盖目标；零、负数或 NaN 保留基准值，避免把
// 无效输入直接写进后续归一化之外的有效参数。
void assignPositive(double source, double& destination)
{
    if (std::isfinite(source) && source > 0.0)
        destination = source;
}

} // namespace

ToolpathParams bindPocketToolParameters(
    const ToolpathParams& base,
    const ToolpathToolContext& tool)
{
    ToolpathParams result = base;
    if (!tool.valid)
        return result;

    // 直径与圆角半径无条件覆盖：它们是几何可行性的前提，快照已保证有效。
    // 进给/转速等工艺字段只在正数时覆盖；刀尖补偿在此清零，交由孔策略
    // 按钻尖角和深度几何另行推导。
    result.tool.diameter = tool.diameter;
    result.tool.cornerRadius = tool.cornerRadius;
    assignPositive(tool.horizontalFeed, result.tool.horizFeed);
    assignPositive(tool.verticalFeed, result.tool.vertFeed);
    assignPositive(tool.horizontalRapid, result.tool.horizRapid);
    assignPositive(tool.verticalRapid, result.tool.vertRapid);
    assignPositive(tool.spindleSpeed, result.tool.spindleSpeed);
    result.tool.cuttingEdgeLength = tool.cuttingEdgeLength;
    result.tool.overallLength = tool.overallLength;
    result.tool.shankDiameter = tool.shankDiameter;
    result.tool.holderDiameter = tool.holderDiameter;
    result.tool.holderLength = tool.holderLength;
    result.tool.spindleEnvelopeDiameter =
        tool.spindleEnvelopeDiameter;
    result.tool.spindleEnvelopeLength =
        tool.spindleEnvelopeLength;
    result.tool.tipCompensation = 0.0;
    return result;
}

} // namespace oap::toolpath
