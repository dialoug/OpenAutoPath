// 刀库领域模型实现：维护刀具唯一标识、类型字段和增删改查的一致性约束。
#include "tooling/library/ToolLibrary.h"
#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <utility>

namespace oap::tooling {
namespace {

// 归一化字符串键：去首尾空白并转小写，用于 ID 容错解析与比较。
QString normalizedId(QString value)
{
    return value.trimmed().toLower();
}

// 几何/切削参数校验规则：必须是有限且非负的数值。
bool finiteNonNegative(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

} // namespace

QString toolTypeId(ToolType type)
{
    switch (type) {
        case ToolType::CenterDrill: return QStringLiteral("center_drill");
        case ToolType::Drill: return QStringLiteral("drill");
        case ToolType::IndexableDrill: return QStringLiteral("indexable_drill");
        case ToolType::EndMill: return QStringLiteral("end_mill");
        case ToolType::BallEndMill: return QStringLiteral("ball_end_mill");
        case ToolType::BullNose: return QStringLiteral("bull_nose");
        case ToolType::FaceMill: return QStringLiteral("face_mill");
        case ToolType::ChamferMill: return QStringLiteral("chamfer_mill");
        case ToolType::BoringBar: return QStringLiteral("boring_bar");
        case ToolType::Reamer: return QStringLiteral("reamer");
        case ToolType::Tap: return QStringLiteral("tap");
        case ToolType::Probe: return QStringLiteral("probe");
        case ToolType::Unknown: return QStringLiteral("unknown");
    }
    return QStringLiteral("unknown");
}

QString toolTypeDisplayName(ToolType type)
{
    switch (type) {
        case ToolType::CenterDrill: return QCoreApplication::translate("OpenAutoPath", "中心钻");
        case ToolType::Drill: return QCoreApplication::translate("OpenAutoPath", "钻头");
        case ToolType::IndexableDrill: return QCoreApplication::translate("OpenAutoPath", "U钻");
        case ToolType::EndMill: return QCoreApplication::translate("OpenAutoPath", "平底刀");
        case ToolType::BallEndMill: return QCoreApplication::translate("OpenAutoPath", "球刀");
        case ToolType::BullNose: return QCoreApplication::translate("OpenAutoPath", "圆鼻刀");
        case ToolType::FaceMill: return QCoreApplication::translate("OpenAutoPath", "飞刀");
        case ToolType::ChamferMill: return QCoreApplication::translate("OpenAutoPath", "倒角刀");
        case ToolType::BoringBar: return QCoreApplication::translate("OpenAutoPath", "镗刀");
        case ToolType::Reamer: return QCoreApplication::translate("OpenAutoPath", "铰刀");
        case ToolType::Tap: return QCoreApplication::translate("OpenAutoPath", "丝锥");
        case ToolType::Probe: return QCoreApplication::translate("OpenAutoPath", "分中棒");
        case ToolType::Unknown: return QCoreApplication::translate("OpenAutoPath", "其他");
    }
    return QCoreApplication::translate("OpenAutoPath", "其他");
}

// 反向解析：与 toolTypeId 保持一一对应，未知 ID 一律回退为 Unknown。
ToolType toolTypeFromId(const QString& id)
{
    const QString key = normalizedId(id);
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
    schema.diameterLabel = QCoreApplication::translate("OpenAutoPath", "刀具直径");
    schema.cuttingEdgeLengthLabel = QCoreApplication::translate("OpenAutoPath", "刀刃长度");
    schema.fluteCountLabel = QCoreApplication::translate("OpenAutoPath", "刃数");
    schema.tipAngleLabel = QCoreApplication::translate("OpenAutoPath", "刀尖角");

    switch (type) {
        case ToolType::CenterDrill:
            schema.diameterLabel = QCoreApplication::translate("OpenAutoPath", "切削直径");
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
            schema.diameterLabel = QCoreApplication::translate("OpenAutoPath", "切削直径");
            schema.fluteCountLabel = QCoreApplication::translate("OpenAutoPath", "刀片数");
            break;
        case ToolType::ChamferMill:
            schema.diameterLabel = QCoreApplication::translate("OpenAutoPath", "最大切削直径");
            schema.tipAngleLabel = QCoreApplication::translate("OpenAutoPath", "包含角");
            schema.tipAngleVisible = true;
            schema.defaultTipAngle = 90.0;
            break;
        case ToolType::BoringBar:
            schema.diameterLabel = QCoreApplication::translate("OpenAutoPath", "镗削直径");
            schema.cuttingEdgeLengthLabel = QCoreApplication::translate("OpenAutoPath", "最大镗深");
            schema.fluteCountVisible = false;
            break;
        case ToolType::Reamer:
            schema.diameterLabel = QCoreApplication::translate("OpenAutoPath", "铰削直径");
            break;
        case ToolType::Tap:
            schema.diameterLabel = QCoreApplication::translate("OpenAutoPath", "公称直径");
            schema.fluteCountLabel = QCoreApplication::translate("OpenAutoPath", "槽数");
            schema.tipAngleVisible = true;
            schema.defaultTipAngle = 90.0;
            schema.threadSpecificationVisible = true;
            break;
        case ToolType::Probe:
            schema.diameterLabel = QCoreApplication::translate("OpenAutoPath", "测球直径");
            schema.cuttingEdgeLengthLabel = QCoreApplication::translate("OpenAutoPath", "测杆长度");
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

QStringList toolGeometryValidationErrors(ToolType type,
                                         const ToolGeometry& g)
{
    QStringList errors;
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
        errors << QCoreApplication::translate("OpenAutoPath", "刀具几何参数必须为有限非负数");
        return errors;
    }

    const ToolTypeParameterSchema schema = toolTypeParameterSchema(type);
    if (schema.diameterRequired && g.diameter <= 0.0)
        errors << QCoreApplication::translate("OpenAutoPath", "该刀型的直径必须大于 0");
    if (g.cornerRadius > 0.5 * g.diameter && g.diameter > 0.0)
        errors << QCoreApplication::translate("OpenAutoPath", "刀尖圆角不能大于刀具半径");
    if (schema.cornerRadiusDerivedFromDiameter && g.diameter > 0.0 &&
        g.cornerRadius > 0.0 &&
        std::abs(g.cornerRadius - 0.5 * g.diameter) > 1.0e-6) {
        errors << QCoreApplication::translate("OpenAutoPath", "球刀圆角半径必须等于刀具半径");
    }
    if (schema.tipAngleVisible && g.tipAngle >= 180.0)
        errors << QCoreApplication::translate("OpenAutoPath", "刀尖角必须小于 180°");
    if (g.overallLength > 0.0 && g.cuttingEdgeLength > g.overallLength)
        errors << QCoreApplication::translate("OpenAutoPath", "刀刃长度不能大于刀具总长");
    if ((g.holderDiameter > 0.0) != (g.holderLength > 0.0))
        errors << QCoreApplication::translate("OpenAutoPath", "刀柄直径和刀柄长度必须同时填写");
    if (g.holderDiameter > 0.0 && g.overallLength <= 0.0)
        errors << QCoreApplication::translate("OpenAutoPath", "建立刀柄包络前必须填写刀具总长");
    if ((g.spindleEnvelopeDiameter > 0.0) !=
        (g.spindleEnvelopeLength > 0.0)) {
        errors << QCoreApplication::translate("OpenAutoPath", "主轴头包络直径和长度必须同时填写");
    }
    if (g.spindleEnvelopeDiameter > 0.0 && g.holderDiameter <= 0.0)
        errors << QCoreApplication::translate("OpenAutoPath", "建立主轴头包络前必须填写刀柄包络");
    return errors;
}

QString coolantModeId(CoolantMode mode)
{
    switch (mode) {
        case CoolantMode::Flood: return QStringLiteral("flood");
        case CoolantMode::ThroughTool: return QStringLiteral("through_tool");
        case CoolantMode::Off: return QStringLiteral("off");
    }
    return QStringLiteral("off");
}

QString coolantModeDisplayName(CoolantMode mode)
{
    switch (mode) {
        case CoolantMode::Flood: return QCoreApplication::translate("OpenAutoPath", "外冷");
        case CoolantMode::ThroughTool: return QCoreApplication::translate("OpenAutoPath", "中心出水");
        case CoolantMode::Off: return QCoreApplication::translate("OpenAutoPath", "关闭");
    }
    return QCoreApplication::translate("OpenAutoPath", "关闭");
}

CoolantMode coolantModeFromId(const QString& id)
{
    const QString key = normalizedId(id);
    if (key == QStringLiteral("flood")) return CoolantMode::Flood;
    if (key == QStringLiteral("through_tool")) return CoolantMode::ThroughTool;
    return CoolantMode::Off;
}

QString drillCycleId(DrillCycle cycle)
{
    switch (cycle) {
        case DrillCycle::G73Peck: return QStringLiteral("g73");
        case DrillCycle::G83Peck: return QStringLiteral("g83");
        case DrillCycle::G81Direct: return QStringLiteral("g81");
        case DrillCycle::Auto: return QStringLiteral("auto");
    }
    return QStringLiteral("auto");
}

QString drillCycleDisplayName(DrillCycle cycle)
{
    switch (cycle) {
        case DrillCycle::G73Peck: return QCoreApplication::translate("OpenAutoPath", "G73 断屑钻");
        case DrillCycle::G83Peck: return QCoreApplication::translate("OpenAutoPath", "G83 深孔啄钻");
        case DrillCycle::G81Direct: return QCoreApplication::translate("OpenAutoPath", "G81 直钻");
        case DrillCycle::Auto: return QCoreApplication::translate("OpenAutoPath", "自动");
    }
    return QCoreApplication::translate("OpenAutoPath", "自动");
}

DrillCycle drillCycleFromId(const QString& id)
{
    const QString key = normalizedId(id);
    if (key == QStringLiteral("g73")) return DrillCycle::G73Peck;
    if (key == QStringLiteral("g83")) return DrillCycle::G83Peck;
    if (key == QStringLiteral("g81")) return DrillCycle::G81Direct;
    return DrillCycle::Auto;
}

ToolLibrary::ToolLibrary(QString id, QString name, QString machine)
    : m_id(std::move(id)), m_name(std::move(name)), m_machine(std::move(machine))
{
}

const ToolBit* ToolLibrary::toolById(const QString& id) const
{
    const auto it = m_tools.constFind(id);
    return it == m_tools.cend() ? nullptr : &it.value();
}

ToolBit* ToolLibrary::toolById(const QString& id)
{
    auto it = m_tools.find(id);
    return it == m_tools.end() ? nullptr : &it.value();
}

const LibraryToolSlot* ToolLibrary::slotByNumber(int toolNumber) const
{
    const auto it = m_slots.constFind(toolNumber);
    return it == m_slots.cend() ? nullptr : &it.value();
}

const ToolBit* ToolLibrary::toolByNumber(int toolNumber) const
{
    const LibraryToolSlot* slot = slotByNumber(toolNumber);
    return slot ? toolById(slot->toolId) : nullptr;
}

const LibraryToolSlot* ToolLibrary::slotForTool(const QString& toolId) const
{
    for (auto it = m_slots.cbegin(); it != m_slots.cend(); ++it) {
        if (it.value().toolId == toolId)
            return &it.value();
    }
    return nullptr;
}

int ToolLibrary::toolNumberFor(const QString& toolId) const
{
    const LibraryToolSlot* slot = slotForTool(toolId);
    return slot ? slot->toolNumber : 0;
}

QVector<AssignedTool> ToolLibrary::assignedTools() const
{
    QVector<AssignedTool> result;
    result.reserve(m_slots.size());
    // QMap 按 T 号升序迭代，保证后续选刀与日志结果可重复。
    for (auto it = m_slots.cbegin(); it != m_slots.cend(); ++it) {
        const ToolBit* tool = toolById(it.value().toolId);
        if (tool)
            result.push_back(AssignedTool{*tool, it.value()});
    }
    return result;
}

bool ToolLibrary::upsertTool(ToolBit tool, QString* error)
{
    tool.id = tool.id.trimmed();
    tool.name = tool.name.trimmed();
    if (tool.id.isEmpty()) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀具 ID 不能为空");
        return false;
    }
    if (tool.name.isEmpty()) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀具名称不能为空");
        return false;
    }
    m_tools.insert(tool.id, std::move(tool));
    return true;
}

bool ToolLibrary::removeTool(const QString& toolId, QString* error)
{
    if (!m_tools.contains(toolId)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀具不存在：%1").arg(toolId);
        return false;
    }
    unassignToolSlot(toolId);
    m_tools.remove(toolId);
    return true;
}

bool ToolLibrary::assignToolToSlot(LibraryToolSlot slot, QString* error)
{
    slot.toolId = slot.toolId.trimmed();
    if (!m_tools.contains(slot.toolId)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀号引用了不存在的刀具：%1").arg(slot.toolId);
        return false;
    }
    if (slot.toolNumber <= 0) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀号必须大于 0");
        return false;
    }

    // 同一刀具最多占一个 T 号；重新分配时先移除旧映射。
    const int oldNumber = toolNumberFor(slot.toolId);
    if (oldNumber == slot.toolNumber) {
        m_slots[slot.toolNumber] = std::move(slot);
        return true;
    }
    if (oldNumber > 0)
        m_slots.remove(oldNumber);

    // 目标 T 号已占用时，不丢弃原刀具，而是把它顺延到下一个空闲号。
    LibraryToolSlot displaced;
    bool hasDisplaced = false;
    const auto occupied = m_slots.find(slot.toolNumber);
    if (occupied != m_slots.end()) {
        displaced = occupied.value();
        hasDisplaced = true;
        m_slots.erase(occupied);
    }
    m_slots.insert(slot.toolNumber, std::move(slot));
    if (hasDisplaced) {
        displaced.toolNumber = nextAvailableToolNumber();
        m_slots.insert(displaced.toolNumber, std::move(displaced));
    }
    return true;
}

bool ToolLibrary::unassignToolSlot(const QString& toolId)
{
    const int number = toolNumberFor(toolId);
    return number > 0 && m_slots.remove(number) > 0;
}

int ToolLibrary::nextAvailableToolNumber() const
{
    return m_slots.isEmpty() ? 1 : (m_slots.lastKey() + 1);
}

QStringList ToolLibrary::validationErrors() const
{
    // 一次收集全部结构问题，便于 UI/导入流程向用户完整展示而非逐条失败。
    QStringList errors;
    if (m_id.trimmed().isEmpty()) errors << QCoreApplication::translate("OpenAutoPath", "刀库 ID 不能为空");
    if (m_name.trimmed().isEmpty()) errors << QCoreApplication::translate("OpenAutoPath", "刀库名称不能为空");

    for (auto it = m_tools.cbegin(); it != m_tools.cend(); ++it) {
        const ToolBit& tool = it.value();
        const QString label = tool.name.isEmpty() ? it.key() : tool.name;
        if (it.key() != tool.id)
            errors << QCoreApplication::translate("OpenAutoPath", "刀具映射键与 ID 不一致：%1").arg(label);
        if (tool.id.trimmed().isEmpty())
            errors << QCoreApplication::translate("OpenAutoPath", "存在空刀具 ID");
        if (tool.name.trimmed().isEmpty())
            errors << QCoreApplication::translate("OpenAutoPath", "刀具名称不能为空：%1").arg(tool.id);
        const QStringList geometryErrors =
            toolGeometryValidationErrors(tool.type, tool.geometry);
        for (const QString& geometryError : geometryErrors)
            errors << QStringLiteral("%1：%2").arg(geometryError, label);
    }

    QMap<QString, int> assigned;
    for (auto it = m_slots.cbegin(); it != m_slots.cend(); ++it) {
        const LibraryToolSlot& slot = it.value();
        if (it.key() <= 0 || slot.toolNumber != it.key())
            errors << QCoreApplication::translate("OpenAutoPath", "刀号映射无效：%1").arg(it.key());
        if (!m_tools.contains(slot.toolId))
            errors << QCoreApplication::translate("OpenAutoPath", "刀号 %1 引用了不存在的刀具 %2")
                          .arg(it.key()).arg(slot.toolId);
        if (assigned.contains(slot.toolId))
            errors << QCoreApplication::translate("OpenAutoPath", "刀具 %1 被重复分配到刀号 %2 和 %3")
                          .arg(slot.toolId).arg(assigned.value(slot.toolId)).arg(it.key());
        assigned.insert(slot.toolId, it.key());

        const CuttingParameters& c = slot.cuttingDefaults;
        if (!finiteNonNegative(c.spindleSpeed) || !finiteNonNegative(c.horizontalFeed) ||
            !finiteNonNegative(c.verticalFeed) || !finiteNonNegative(c.horizontalRapid) ||
            !finiteNonNegative(c.verticalRapid) || !finiteNonNegative(c.peckDepth)) {
            errors << QCoreApplication::translate("OpenAutoPath", "刀号 %1 的切削参数必须为有限非负数").arg(it.key());
        }
    }
    return errors;
}

bool ToolLibrary::isValid(QString* error) const
{
    const QStringList errors = validationErrors();
    if (error) *error = errors.join(QLatin1Char('\n'));
    return errors.isEmpty();
}

} // namespace oap::tooling
