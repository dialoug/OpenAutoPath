// 刀库领域模型实现：维护刀具唯一标识、类型字段和增删改查的一致性约束。
#include "support/Text.h"
#include <format>
#include <map>
#include <vector>
#include <string>
#include "tooling/library/ToolLibrary.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace oap::tooling {
namespace {

// 归一化字符串键：去首尾空白并转小写，用于 ID 容错解析与比较。
std::string normalizedId(std::string value)
{
    return oap::lowerAscii(oap::trim(value));
}

// 几何/切削参数校验规则：必须是有限且非负的数值。
bool finiteNonNegative(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

} // namespace

std::string toolTypeId(ToolType type)
{
    switch (type) {
        case ToolType::CenterDrill: return std::string("center_drill");
        case ToolType::Drill: return std::string("drill");
        case ToolType::IndexableDrill: return std::string("indexable_drill");
        case ToolType::EndMill: return std::string("end_mill");
        case ToolType::BallEndMill: return std::string("ball_end_mill");
        case ToolType::BullNose: return std::string("bull_nose");
        case ToolType::FaceMill: return std::string("face_mill");
        case ToolType::ChamferMill: return std::string("chamfer_mill");
        case ToolType::BoringBar: return std::string("boring_bar");
        case ToolType::Reamer: return std::string("reamer");
        case ToolType::Tap: return std::string("tap");
        case ToolType::Probe: return std::string("probe");
        case ToolType::Unknown: return std::string("unknown");
    }
    return std::string("unknown");
}

std::string toolTypeDisplayName(ToolType type)
{
    switch (type) {
        case ToolType::CenterDrill: return std::string("中心钻");
        case ToolType::Drill: return std::string("钻头");
        case ToolType::IndexableDrill: return std::string("U钻");
        case ToolType::EndMill: return std::string("平底刀");
        case ToolType::BallEndMill: return std::string("球刀");
        case ToolType::BullNose: return std::string("圆鼻刀");
        case ToolType::FaceMill: return std::string("飞刀");
        case ToolType::ChamferMill: return std::string("倒角刀");
        case ToolType::BoringBar: return std::string("镗刀");
        case ToolType::Reamer: return std::string("铰刀");
        case ToolType::Tap: return std::string("丝锥");
        case ToolType::Probe: return std::string("分中棒");
        case ToolType::Unknown: return std::string("其他");
    }
    return std::string("其他");
}

// 反向解析：与 toolTypeId 保持一一对应，未知 ID 一律回退为 Unknown。
ToolType toolTypeFromId(const std::string& id)
{
    const std::string key = normalizedId(id);
    for (ToolType type : {ToolType::CenterDrill, ToolType::Drill,
                          ToolType::IndexableDrill, ToolType::EndMill,
                          ToolType::BallEndMill, ToolType::BullNose,
                          ToolType::FaceMill, ToolType::ChamferMill,
                          ToolType::BoringBar, ToolType::Reamer,
                          ToolType::Tap, ToolType::Probe}) {
        if (toolTypeId(type) == key)
            return type;
    }
    return ToolType::Unknown;
}

// 按刀具类型返回参数编辑 schema：字段可见性、标签与默认顶尖角。
ToolTypeParameterSchema toolTypeParameterSchema(ToolType type)
{
    ToolTypeParameterSchema schema;
    schema.diameterLabel = std::string("刀具直径");
    schema.cuttingEdgeLengthLabel = std::string("刀刃长度");
    schema.fluteCountLabel = std::string("刃数");
    schema.tipAngleLabel = std::string("刀尖角");

    switch (type) {
        case ToolType::CenterDrill:
            schema.diameterLabel = std::string("切削直径");
            schema.tipAngleVisible = true;
            schema.tipCompensationVisible = true;
            schema.drillingDefaultsVisible = true;
            schema.defaultTipAngle = 60.0;
            break;
        case ToolType::Drill:
        case ToolType::IndexableDrill:
            schema.tipAngleVisible = true;
            schema.tipCompensationVisible = true;
            schema.drillingDefaultsVisible = true;
            schema.defaultTipAngle = 118.0;
            break;
        case ToolType::EndMill:
            break;
        case ToolType::BallEndMill:
            schema.cornerRadiusVisible = true;
            schema.cornerRadiusDerivedFromDiameter = true;
            break;
        case ToolType::BullNose:
            schema.cornerRadiusVisible = true;
            break;
        case ToolType::FaceMill:
            schema.diameterLabel = std::string("切削直径");
            schema.fluteCountLabel = std::string("刀片数");
            break;
        case ToolType::ChamferMill:
            schema.diameterLabel = std::string("最大切削直径");
            schema.tipAngleLabel = std::string("包含角");
            schema.tipAngleVisible = true;
            schema.defaultTipAngle = 90.0;
            break;
        case ToolType::BoringBar:
            schema.diameterLabel = std::string("镗削直径");
            schema.cuttingEdgeLengthLabel = std::string("最大镗深");
            schema.fluteCountVisible = false;
            break;
        case ToolType::Reamer:
            schema.diameterLabel = std::string("铰削直径");
            break;
        case ToolType::Tap:
            schema.diameterLabel = std::string("公称直径");
            schema.fluteCountLabel = std::string("槽数");
            schema.tipAngleVisible = true;
            schema.defaultTipAngle = 90.0;
            schema.threadSpecificationVisible = true;
            break;
        case ToolType::Probe:
            schema.diameterLabel = std::string("测球直径");
            schema.cuttingEdgeLengthLabel = std::string("测杆长度");
            schema.diameterRequired = false;
            schema.fluteCountVisible = false;
            break;
        case ToolType::Unknown:
            // 其他刀型保留全部可选几何字段，避免无法编辑旧数据。
            schema.diameterRequired = false;
            schema.fluteCountVisible = false;
            schema.cornerRadiusVisible = true;
            schema.tipAngleVisible = true;
            schema.tipCompensationVisible = true;
            break;
    }
    return schema;
}

ToolGeometry normalizedToolGeometry(ToolType type,
                                    ToolGeometry geometry) noexcept
{
    const ToolTypeParameterSchema schema = toolTypeParameterSchema(type);
    if (schema.cornerRadiusDerivedFromDiameter)
        geometry.cornerRadius = std::max(0.0, geometry.diameter * 0.5);
    else if (!schema.cornerRadiusVisible)
        geometry.cornerRadius = 0.0;
    if (!schema.tipAngleVisible)
        geometry.tipAngle = 0.0;
    if (!schema.tipCompensationVisible)
        geometry.tipCompensation = 0.0;
    return geometry;
}

std::vector<std::string> toolGeometryValidationErrors(ToolType type,
                                         const ToolGeometry& g)
{
    std::vector<std::string> errors;
    if (!finiteNonNegative(g.diameter) ||
        !finiteNonNegative(g.cuttingEdgeLength) ||
        !finiteNonNegative(g.overallLength) ||
        !finiteNonNegative(g.cornerRadius) ||
        !finiteNonNegative(g.tipAngle) ||
        !finiteNonNegative(g.tipCompensation) ||
        !finiteNonNegative(g.shankDiameter) ||
        !finiteNonNegative(g.holderDiameter) ||
        !finiteNonNegative(g.holderLength) ||
        !finiteNonNegative(g.spindleEnvelopeDiameter) ||
        !finiteNonNegative(g.spindleEnvelopeLength)) {
        errors.push_back(std::string("刀具几何参数必须为有限非负数"));
        return errors;
    }

    const ToolTypeParameterSchema schema = toolTypeParameterSchema(type);
    if (schema.diameterRequired && g.diameter <= 0.0)
        errors.push_back(std::string("该刀型的直径必须大于 0"));
    if (g.cornerRadius > 0.5 * g.diameter && g.diameter > 0.0)
        errors.push_back(std::string("刀尖圆角不能大于刀具半径"));
    if (schema.cornerRadiusDerivedFromDiameter && g.diameter > 0.0 &&
        g.cornerRadius > 0.0 &&
        std::abs(g.cornerRadius - 0.5 * g.diameter) > 1.0e-6) {
        errors.push_back(std::string("球刀圆角半径必须等于刀具半径"));
    }
    if (schema.tipAngleVisible && g.tipAngle >= 180.0)
        errors.push_back(std::string("刀尖角必须小于 180°"));
    if (g.overallLength > 0.0 && g.cuttingEdgeLength > g.overallLength)
        errors.push_back(std::string("刀刃长度不能大于刀具总长"));
    if ((g.holderDiameter > 0.0) != (g.holderLength > 0.0))
        errors.push_back(std::string("刀柄直径和刀柄长度必须同时填写"));
    if (g.holderDiameter > 0.0 && g.overallLength <= 0.0)
        errors.push_back(std::string("建立刀柄包络前必须填写刀具总长"));
    if ((g.spindleEnvelopeDiameter > 0.0) !=
        (g.spindleEnvelopeLength > 0.0)) {
        errors.push_back(std::string("主轴头包络直径和长度必须同时填写"));
    }
    if (g.spindleEnvelopeDiameter > 0.0 && g.holderDiameter <= 0.0)
        errors.push_back(std::string("建立主轴头包络前必须填写刀柄包络"));
    return errors;
}

std::string coolantModeId(CoolantMode mode)
{
    switch (mode) {
        case CoolantMode::Flood: return std::string("flood");
        case CoolantMode::ThroughTool: return std::string("through_tool");
        case CoolantMode::Off: return std::string("off");
    }
    return std::string("off");
}

std::string coolantModeDisplayName(CoolantMode mode)
{
    switch (mode) {
        case CoolantMode::Flood: return std::string("外冷");
        case CoolantMode::ThroughTool: return std::string("中心出水");
        case CoolantMode::Off: return std::string("关闭");
    }
    return std::string("关闭");
}

CoolantMode coolantModeFromId(const std::string& id)
{
    const std::string key = normalizedId(id);
    if (key == std::string("flood")) return CoolantMode::Flood;
    if (key == std::string("through_tool")) return CoolantMode::ThroughTool;
    return CoolantMode::Off;
}

std::string drillCycleId(DrillCycle cycle)
{
    switch (cycle) {
        case DrillCycle::G73Peck: return std::string("g73");
        case DrillCycle::G83Peck: return std::string("g83");
        case DrillCycle::G81Direct: return std::string("g81");
        case DrillCycle::Auto: return std::string("auto");
    }
    return std::string("auto");
}

std::string drillCycleDisplayName(DrillCycle cycle)
{
    switch (cycle) {
        case DrillCycle::G73Peck: return std::string("G73 断屑钻");
        case DrillCycle::G83Peck: return std::string("G83 深孔啄钻");
        case DrillCycle::G81Direct: return std::string("G81 直钻");
        case DrillCycle::Auto: return std::string("自动");
    }
    return std::string("自动");
}

DrillCycle drillCycleFromId(const std::string& id)
{
    const std::string key = normalizedId(id);
    if (key == std::string("g73")) return DrillCycle::G73Peck;
    if (key == std::string("g83")) return DrillCycle::G83Peck;
    if (key == std::string("g81")) return DrillCycle::G81Direct;
    return DrillCycle::Auto;
}

ToolLibrary::ToolLibrary(std::string id, std::string name, std::string machine)
    : m_id(std::move(id)), m_name(std::move(name)), m_machine(std::move(machine))
{
}

const ToolBit* ToolLibrary::toolById(const std::string& id) const
{
    const auto it = m_tools.find(id);
    return it == m_tools.cend() ? nullptr : &it->second;
}

ToolBit* ToolLibrary::toolById(const std::string& id)
{
    auto it = m_tools.find(id);
    return it == m_tools.end() ? nullptr : &it->second;
}

const LibraryToolSlot* ToolLibrary::slotByNumber(int toolNumber) const
{
    const auto it = m_slots.find(toolNumber);
    return it == m_slots.cend() ? nullptr : &it->second;
}

const ToolBit* ToolLibrary::toolByNumber(int toolNumber) const
{
    const LibraryToolSlot* slot = slotByNumber(toolNumber);
    return slot ? toolById(slot->toolId) : nullptr;
}

const LibraryToolSlot* ToolLibrary::slotForTool(const std::string& toolId) const
{
    for (auto it = m_slots.cbegin(); it != m_slots.cend(); ++it) {
        if (it->second.toolId == toolId)
            return &it->second;
    }
    return nullptr;
}

int ToolLibrary::toolNumberFor(const std::string& toolId) const
{
    const LibraryToolSlot* slot = slotForTool(toolId);
    return slot ? slot->toolNumber : 0;
}

std::vector<AssignedTool> ToolLibrary::assignedTools() const
{
    std::vector<AssignedTool> result;
    result.reserve(m_slots.size());
    // std::map 按 T 号升序迭代，保证后续选刀与日志结果可重复。
    for (auto it = m_slots.cbegin(); it != m_slots.cend(); ++it) {
        const ToolBit* tool = toolById(it->second.toolId);
        if (tool)
            result.push_back(AssignedTool{*tool, it->second});
    }
    return result;
}

bool ToolLibrary::upsertTool(ToolBit tool, std::string* error)
{
    tool.id = oap::trim(tool.id);
    tool.name = oap::trim(tool.name);
    if (tool.id.empty()) {
        if (error) *error = std::string("刀具 ID 不能为空");
        return false;
    }
    if (tool.name.empty()) {
        if (error) *error = std::string("刀具名称不能为空");
        return false;
    }
    m_tools.insert_or_assign(tool.id, std::move(tool));
    return true;
}

bool ToolLibrary::removeTool(const std::string& toolId, std::string* error)
{
    if (!m_tools.contains(toolId)) {
        if (error) *error = std::format("刀具不存在：{0}", toolId);
        return false;
    }
    unassignToolSlot(toolId);
    m_tools.erase(toolId);
    return true;
}

bool ToolLibrary::assignToolToSlot(LibraryToolSlot slot, std::string* error)
{
    slot.toolId = oap::trim(slot.toolId);
    if (!m_tools.contains(slot.toolId)) {
        if (error) *error = std::format("刀号引用了不存在的刀具：{0}", slot.toolId);
        return false;
    }
    if (slot.toolNumber <= 0) {
        if (error) *error = std::string("刀号必须大于 0");
        return false;
    }

    // 同一刀具最多占一个 T 号；重新分配时先移除旧映射。
    const int oldNumber = toolNumberFor(slot.toolId);
    if (oldNumber == slot.toolNumber) {
        m_slots[slot.toolNumber] = std::move(slot);
        return true;
    }
    if (oldNumber > 0)
        m_slots.erase(oldNumber);

    // 目标 T 号已占用时，不丢弃原刀具，而是把它顺延到下一个空闲号。
    LibraryToolSlot displaced;
    bool hasDisplaced = false;
    const auto occupied = m_slots.find(slot.toolNumber);
    if (occupied != m_slots.end()) {
        displaced = occupied->second;
        hasDisplaced = true;
        m_slots.erase(occupied);
    }
    m_slots.insert_or_assign(slot.toolNumber, std::move(slot));
    if (hasDisplaced) {
        displaced.toolNumber = nextAvailableToolNumber();
        m_slots.insert_or_assign(displaced.toolNumber, std::move(displaced));
    }
    return true;
}

bool ToolLibrary::unassignToolSlot(const std::string& toolId)
{
    const int number = toolNumberFor(toolId);
    return number > 0 && m_slots.erase(number) > 0;
}

int ToolLibrary::nextAvailableToolNumber() const
{
    return m_slots.empty() ? 1 : (m_slots.rbegin()->first + 1);
}

std::vector<std::string> ToolLibrary::validationErrors() const
{
    // 一次收集全部结构问题，便于 UI/导入流程向用户完整展示而非逐条失败。
    std::vector<std::string> errors;
    if (oap::trim(m_id).empty()) errors.push_back(std::string("刀库 ID 不能为空"));
    if (oap::trim(m_name).empty()) errors.push_back(std::string("刀库名称不能为空"));

    for (auto it = m_tools.cbegin(); it != m_tools.cend(); ++it) {
        const ToolBit& tool = it->second;
        const std::string label = tool.name.empty() ? it->first : tool.name;
        if (it->first != tool.id)
            errors.push_back(std::format("刀具映射键与 ID 不一致：{0}", label));
        if (oap::trim(tool.id).empty())
            errors.push_back(std::string("存在空刀具 ID"));
        if (oap::trim(tool.name).empty())
            errors.push_back(std::format("刀具名称不能为空：{0}", tool.id));
        const std::vector<std::string> geometryErrors =
            toolGeometryValidationErrors(tool.type, tool.geometry);
        for (const std::string& geometryError : geometryErrors)
            errors.push_back(std::format("{0}：{1}", geometryError, label));
    }

    std::map<std::string, int> assigned;
    for (auto it = m_slots.cbegin(); it != m_slots.cend(); ++it) {
        const LibraryToolSlot& slot = it->second;
        if (it->first <= 0 || slot.toolNumber != it->first)
            errors.push_back(std::format("刀号映射无效：{0}", it->first));
        if (!m_tools.contains(slot.toolId))
            errors.push_back(std::format("刀号 {0} 引用了不存在的刀具 {1}", it->first, slot.toolId));
        if (assigned.contains(slot.toolId))
            errors.push_back(std::format("刀具 {0} 被重复分配到刀号 {1} 和 {2}", slot.toolId, assigned.at(slot.toolId), it->first));
        assigned.insert_or_assign(slot.toolId, it->first);

        const CuttingParameters& c = slot.cuttingDefaults;
        if (!finiteNonNegative(c.spindleSpeed) || !finiteNonNegative(c.horizontalFeed) ||
            !finiteNonNegative(c.verticalFeed) || !finiteNonNegative(c.horizontalRapid) ||
            !finiteNonNegative(c.verticalRapid) || !finiteNonNegative(c.peckDepth)) {
            errors.push_back(std::format("刀号 {0} 的切削参数必须为有限非负数", it->first));
        }
    }
    return errors;
}

bool ToolLibrary::isValid(std::string* error) const
{
    const std::vector<std::string> errors = validationErrors();
    if (error) *error = oap::joinLines(errors);
    return errors.empty();
}

} // namespace oap::tooling
