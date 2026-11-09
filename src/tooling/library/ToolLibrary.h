// C++ 刀具与刀库领域模型。
//
// 设计参考 FreeCAD CAM 的 ToolBit / Library / Job ToolController 分层：
// - ToolBit 描述可复用的刀具几何与扩展属性；
// - ToolLibrary 维护刀具集合和“刀号 -> 刀具”映射；
// - LibraryToolSlot 只表达刀库中的 T 刀位和默认切削值；
// - 实际 Job ToolController 由适配层创建为不可变快照，不回指本对象。
//
// 模块归属：oap_toollibrary（刀具数据与刀库领域模型，不依赖 CAM）。
// 持久化由 ToolLibraryStorage 负责：.oaptl 自包含 JSON 与目录 catalog.json。
#pragma once

#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>

#include <utility>

namespace oap::tooling {

// 刀具类型：驱动显示名、几何校验规则与 CAM 刀具建模类型选择。
enum class ToolType {
    Unknown,
    CenterDrill,
    Drill,
    IndexableDrill,
    EndMill,
    BallEndMill,
    BullNose,
    FaceMill,
    ChamferMill,
    BoringBar,
    Reamer,
    Tap,
    Probe,
};

// 冷却方式：与持久化 coolant 字段一致（off/flood/through_tool）。
enum class CoolantMode {
    Off,
    Flood,
    ThroughTool,
};

// 钻孔循环：G73 断屑 / G83 深孔啄钻 / G81 直钻；Auto 交由刀路或工艺决策。
enum class DrillCycle {
    Auto,
    G73Peck,
    G83Peck,
    G81Direct,
};

// 枚举与稳定字符串 ID 的双向转换；ID 用于 JSON 持久化，显示名用于界面。
QString toolTypeId(ToolType type);
QString toolTypeDisplayName(ToolType type);
ToolType toolTypeFromId(const QString& id);

// 刀型参数规则：描述编辑器与校验器应如何解释通用 ToolGeometry 字段。
// 刀柄/夹头和主轴包络始终属于装配参数，不随刀型隐藏。
struct ToolTypeParameterSchema {
    QString diameterLabel;
    QString cuttingEdgeLengthLabel;
    QString fluteCountLabel;
    QString tipAngleLabel;
    bool diameterRequired = true;
    bool cuttingEdgeLengthVisible = true;
    bool fluteCountVisible = true;
    bool cornerRadiusVisible = false;
    bool cornerRadiusDerivedFromDiameter = false;
    bool tipAngleVisible = false;
    bool tipCompensationVisible = false;
    bool drillingDefaultsVisible = false;
    bool threadSpecificationVisible = false;
    double defaultTipAngle = 0.0;
};

ToolTypeParameterSchema toolTypeParameterSchema(ToolType type);

QString coolantModeId(CoolantMode mode);
QString coolantModeDisplayName(CoolantMode mode);
CoolantMode coolantModeFromId(const QString& id);

QString drillCycleId(DrillCycle cycle);
QString drillCycleDisplayName(DrillCycle cycle);
DrillCycle drillCycleFromId(const QString& id);

// 刀具几何参数（单位见各成员行内注释）；0 约定为“未知/未填写”，持久化原样保存。
struct ToolGeometry {
    double diameter = 0.0;          // mm；分中棒等非切削工具可为 0。
    double cuttingEdgeLength = 0.0; // mm；0 表示未知。
    double overallLength = 0.0;     // mm；0 表示未知。
    double cornerRadius = 0.0;      // mm。
    double tipAngle = 0.0;          // degree；钻头未填写时为 0。
    // 钻尖沿刀轴方向的实测/厂家补偿值；大于 0 时优先于角度公式。
    double tipCompensation = 0.0;   // mm；0 表示未填写。
    double shankDiameter = 0.0;     // mm；0 表示按刀具直径或未知处理。
    double holderDiameter = 0.0;    // mm；夹头/刀柄最大碰撞直径。
    double holderLength = 0.0;      // mm；从刀具总长端面向主轴方向的建模长度。
    double spindleEnvelopeDiameter = 0.0; // mm；主轴头/鼻端保守直径。
    double spindleEnvelopeLength = 0.0;   // mm；刀柄之后的主轴头建模长度。
};

ToolGeometry normalizedToolGeometry(ToolType type,
                                    ToolGeometry geometry) noexcept;
QStringList toolGeometryValidationErrors(ToolType type,
                                         const ToolGeometry& geometry);

// 一把可复用刀具的完整定义；进入 ToolAssetCatalog 后 id 全局唯一，attributes
// 保存牙规、材料、来源及旧目录迁移信息等开放属性。
struct ToolBit {
    QString id;                     // 在刀库内稳定且唯一。
    QString name;
    ToolType type = ToolType::Unknown;
    ToolGeometry geometry;
    QString note;
    bool throughCoolant = false;
    QMap<QString, QString> attributes; // 牙规、材料、来源等开放属性。
};

// 切削参数；0 表示“沿用当前/机床默认”，以便导入时区分未知与用户维护值。
struct CuttingParameters {
    double spindleSpeed = 0.0;   // rpm；0 表示沿用当前/机床默认。
    double horizontalFeed = 0.0; // mm/min；0 表示沿用当前值。
    double verticalFeed = 0.0;   // mm/min；0 表示沿用当前值。
    double horizontalRapid = 0.0;
    double verticalRapid = 0.0;
    double peckDepth = 0.0;      // G73/G83 每次钻深 Q，mm；0=使用刀路默认。
    DrillCycle drillCycle = DrillCycle::Auto;
};

// 刀库中的 T 刀位：刀号、所引用刀具 id 以及创建 Job 控制器时使用的
// 默认切削/冷却值。它不是 Job ToolController，不能被工序直接引用。
struct LibraryToolSlot {
    int toolNumber = 0;
    QString toolId;
    CuttingParameters cuttingDefaults;
    CoolantMode coolantDefault = CoolantMode::Off;
};

// 一个已经占用机床 T 刀位的完整刀具快照。后续孔、槽腔选刀器只依赖此
// 集合，不应遍历未分配刀号的备用目录项。
struct AssignedTool {
    ToolBit tool;
    LibraryToolSlot slot;
};

// 一个刀具库：按 id 索引 ToolBit 物化快照，按 T 号索引 LibraryToolSlot。
// 全局目录中的权威定义由 ToolAssetCatalog 持有；这里保留快照以兼容单文件 .oaptl
// 和现有 CAM 自动规划器。
// 不变量：ToolBit.id 唯一且稳定；刀位经 toolId 引用刀具；同一刀具最多占一个 T 号。
// 由调用方（导入器/目录/界面）持有，本类非线程安全，多线程访问需外部同步。
class ToolLibrary {
public:
    ToolLibrary() = default;
    ToolLibrary(QString id, QString name, QString machine = {});

    // 只读访问器：返回内部成员引用，调用方不应持有超过刀库生命周期的引用。
    const QString& id() const noexcept { return m_id; }
    const QString& name() const noexcept { return m_name; }
    const QString& machine() const noexcept { return m_machine; }
    const QString& description() const noexcept { return m_description; }
    const QMap<QString, QString>& attributes() const noexcept { return m_attributes; }

    void setId(QString id) { m_id = std::move(id); }
    void setName(QString name) { m_name = std::move(name); }
    void setMachine(QString machine) { m_machine = std::move(machine); }
    void setDescription(QString description) { m_description = std::move(description); }
    void setAttribute(QString key, QString value)
    {
        m_attributes.insert(std::move(key), std::move(value));
    }

    const QMap<QString, ToolBit>& tools() const noexcept { return m_tools; }
    const QMap<int, LibraryToolSlot>& toolSlots() const noexcept { return m_slots; }

    // 查询接口：未命中返回 nullptr；返回指针指向内部容器元素，任何修改操作后失效。
    const ToolBit* toolById(const QString& id) const;
    ToolBit* toolById(const QString& id);
    const ToolBit* toolByNumber(int toolNumber) const;
    const LibraryToolSlot* slotByNumber(int toolNumber) const;
    const LibraryToolSlot* slotForTool(const QString& toolId) const;
    int toolNumberFor(const QString& toolId) const;
    QVector<AssignedTool> assignedTools() const;

    // 新增或替换刀具定义。替换时保留已有刀位映射。
    bool upsertTool(ToolBit tool, QString* error = nullptr);
    // 删除刀具定义，并顺带解除其已占用的刀位。
    bool removeTool(const QString& toolId, QString* error = nullptr);

    // 分配刀号时，若目标刀号已占用，会把原刀具移动到末尾的新刀号；
    // 该行为与 FreeCAD Library::assign_new_bit_no 一致。
    bool assignToolToSlot(LibraryToolSlot slot, QString* error = nullptr);
    // 解除刀具占用的刀位（仅删除控制器，保留刀具定义）；返回是否真的解除了分配。
    bool unassignToolSlot(const QString& toolId);
    // 返回当前最大 T 号 + 1；无任何刀位时返回 1（与 FreeCAD 分配行为一致）。
    int nextAvailableToolNumber() const;

    // 收集全部结构/几何/刀位一致性错误；空列表表示刀库有效。isValid 返回其合并文本。
    QStringList validationErrors() const;
    bool isValid(QString* error = nullptr) const;

private:
    // 刀具定义与刀位控制器分开存放；控制器只通过 toolId 引用刀具。
    QString m_id;
    QString m_name;
    QString m_machine;
    QString m_description;
    QMap<QString, QString> m_attributes;
    QMap<QString, ToolBit> m_tools;
    QMap<int, LibraryToolSlot> m_slots;
};

} // namespace oap::tooling
