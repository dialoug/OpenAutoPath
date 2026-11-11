// 刀库 JSON 持久化边界：负责版本/字段校验、兼容导入和安全覆盖写入。
#include "tooling/library/ToolLibraryStorage.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QRegularExpression>
#include <QSaveFile>
#include <QStandardPaths>

#include <cmath>

namespace oap::tooling {
namespace {

// 单刀库 schema 版本为 1；目录 schema v3 将 ToolBit 提升为全局资产。
// v1/v2 仅迁移资产存储结构，保留用户参数。
constexpr int kLibrarySchemaVersion = 1;
constexpr int kCatalogSchemaVersion = 3;
constexpr int kOldestSupportedCatalogVersion = 1;
// 单文件大小上限 16 MiB，防止读取超大文件。
constexpr qint64 kMaxToolLibraryBytes = 16ll * 1024ll * 1024ll;

// 旧标识仅用于显式导入兼容；写入始终使用 OAP 标识。
bool isLibrarySchema(const QString& schema)
{
    return schema == QStringLiteral("oap-tool-library") ||
           schema == QStringLiteral("faceface-tool-library");
}

// 受大小限制地读取整个文件；超限或读取出错时报错并返回 false。
bool readLimitedJson(QFile& file, QByteArray& data, QString* error)
{
    const qint64 size = file.size();
    if (size < 0 || size > kMaxToolLibraryBytes) {
        if (error) {
            *error = QCoreApplication::translate("OpenAutoPath", "刀库文件过大（上限 %1 MiB）")
                         .arg(kMaxToolLibraryBytes / (1024 * 1024));
        }
        return false;
    }
    data = file.read(kMaxToolLibraryBytes + 1);
    if (data.size() > kMaxToolLibraryBytes) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀库文件超过允许大小");
        return false;
    }
    if (file.error() != QFileDevice::NoError) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "读取刀库失败：%1").arg(file.errorString());
        return false;
    }
    return true;
}

// 路径安全校验：candidate 必须是 root 目录内的真实文件，拒绝 ../ 逃逸。
bool isContainedFile(const QString& rootPath, const QString& candidatePath)
{
    const QString root = QFileInfo(rootPath).canonicalFilePath();
    const QFileInfo candidateInfo(candidatePath);
    const QString candidate = candidateInfo.canonicalFilePath();
    if (root.isEmpty() || candidate.isEmpty() || !candidateInfo.isFile())
        return false;
    const QString relative = QDir(root).relativeFilePath(candidate);
    return !QDir::isAbsolutePath(relative) && relative != QStringLiteral("..") &&
           !relative.startsWith(QStringLiteral("../")) &&
           !relative.startsWith(QStringLiteral("..\\"));
}

// ToolGeometry ↔ JSON 的字段映射，字段名即持久化格式的一部分，勿随意改名。
QJsonObject geometryToJson(const ToolGeometry& geometry)
{
    return {
        {QStringLiteral("diameter"), geometry.diameter},
        {QStringLiteral("cutting_edge_length"), geometry.cuttingEdgeLength},
        {QStringLiteral("overall_length"), geometry.overallLength},
        {QStringLiteral("corner_radius"), geometry.cornerRadius},
        {QStringLiteral("tip_angle"), geometry.tipAngle},
        {QStringLiteral("tip_compensation"), geometry.tipCompensation},
        {QStringLiteral("shank_diameter"), geometry.shankDiameter},
        {QStringLiteral("holder_diameter"), geometry.holderDiameter},
        {QStringLiteral("holder_length"), geometry.holderLength},
        {QStringLiteral("spindle_envelope_diameter"),
         geometry.spindleEnvelopeDiameter},
        {QStringLiteral("spindle_envelope_length"),
         geometry.spindleEnvelopeLength},
    };
}

// 反序列化几何：缺失字段按 0（未知）处理，语义与领域模型约定一致。
ToolGeometry geometryFromJson(const QJsonObject& object)
{
    ToolGeometry geometry;
    geometry.diameter = object.value(QStringLiteral("diameter")).toDouble();
    geometry.cuttingEdgeLength =
        object.value(QStringLiteral("cutting_edge_length")).toDouble();
    geometry.overallLength = object.value(QStringLiteral("overall_length")).toDouble();
    geometry.cornerRadius = object.value(QStringLiteral("corner_radius")).toDouble();
    geometry.tipAngle = object.value(QStringLiteral("tip_angle")).toDouble();
    geometry.tipCompensation =
        object.value(QStringLiteral("tip_compensation")).toDouble();
    geometry.shankDiameter =
        object.value(QStringLiteral("shank_diameter")).toDouble();
    geometry.holderDiameter =
        object.value(QStringLiteral("holder_diameter")).toDouble();
    geometry.holderLength =
        object.value(QStringLiteral("holder_length")).toDouble();
    geometry.spindleEnvelopeDiameter = object.value(
        QStringLiteral("spindle_envelope_diameter")).toDouble();
    geometry.spindleEnvelopeLength = object.value(
        QStringLiteral("spindle_envelope_length")).toDouble();
    return geometry;
}

// 切削参数 → JSON 的字段映射（含钻循环字符串 ID）。
QJsonObject cuttingToJson(const CuttingParameters& cutting)
{
    return {
        {QStringLiteral("spindle_speed"), cutting.spindleSpeed},
        {QStringLiteral("horizontal_feed"), cutting.horizontalFeed},
        {QStringLiteral("vertical_feed"), cutting.verticalFeed},
        {QStringLiteral("horizontal_rapid"), cutting.horizontalRapid},
        {QStringLiteral("vertical_rapid"), cutting.verticalRapid},
        {QStringLiteral("peck_depth"), cutting.peckDepth},
        {QStringLiteral("drill_cycle"), drillCycleId(cutting.drillCycle)},
    };
}

QJsonObject toolToJson(const ToolBit& tool)
{
    QJsonObject attributes;
    for (auto it = tool.attributes.cbegin(); it != tool.attributes.cend(); ++it)
        attributes.insert(it.key(), it.value());
    return {
        {QStringLiteral("id"), tool.id},
        {QStringLiteral("name"), tool.name},
        {QStringLiteral("type"), toolTypeId(tool.type)},
        {QStringLiteral("geometry"), geometryToJson(tool.geometry)},
        {QStringLiteral("note"), tool.note},
        {QStringLiteral("through_coolant"), tool.throughCoolant},
        {QStringLiteral("attributes"), attributes},
    };
}

bool toolFromJson(const QJsonObject& item, ToolBit& tool, QString* error)
{
    ToolBit parsed;
    parsed.id = item.value(QStringLiteral("id")).toString();
    parsed.name = item.value(QStringLiteral("name")).toString();
    parsed.type = toolTypeFromId(item.value(QStringLiteral("type")).toString());
    parsed.geometry = geometryFromJson(
        item.value(QStringLiteral("geometry")).toObject());
    parsed.note = item.value(QStringLiteral("note")).toString();
    parsed.throughCoolant =
        item.value(QStringLiteral("through_coolant")).toBool();
    const QJsonObject attributes =
        item.value(QStringLiteral("attributes")).toObject();
    for (auto it = attributes.begin(); it != attributes.end(); ++it)
        parsed.attributes.insert(it.key(), it.value().toVariant().toString());

    ToolLibrary validator(QStringLiteral("asset-validator"),
                          QStringLiteral("asset-validator"));
    QString validationError;
    if (!validator.upsertTool(parsed, &validationError) ||
        !validator.isValid(&validationError)) {
        if (error)
            *error = validationError;
        return false;
    }
    tool = std::move(parsed);
    return true;
}

// 反序列化切削参数：缺失字段按 0（沿用默认）处理。
CuttingParameters cuttingFromJson(const QJsonObject& object)
{
    CuttingParameters cutting;
    cutting.spindleSpeed = object.value(QStringLiteral("spindle_speed")).toDouble();
    cutting.horizontalFeed = object.value(QStringLiteral("horizontal_feed")).toDouble();
    cutting.verticalFeed = object.value(QStringLiteral("vertical_feed")).toDouble();
    cutting.horizontalRapid = object.value(QStringLiteral("horizontal_rapid")).toDouble();
    cutting.verticalRapid = object.value(QStringLiteral("vertical_rapid")).toDouble();
    cutting.peckDepth = object.value(QStringLiteral("peck_depth")).toDouble();
    cutting.drillCycle =
        drillCycleFromId(object.value(QStringLiteral("drill_cycle")).toString());
    return cutting;
}

QJsonObject libraryToJson(const ToolLibrary& library)
{
    // 刀具定义与刀位控制器分开存储；控制器只通过稳定 tool_id 引用刀具。
    QJsonObject libraryAttributes;
    for (auto it = library.attributes().cbegin(); it != library.attributes().cend(); ++it)
        libraryAttributes.insert(it.key(), it.value());

    QJsonArray tools;
    for (const ToolBit& tool : library.tools())
        tools.append(toolToJson(tool));

    QJsonArray controllers;
    // Wire keys stay unchanged for backward compatibility with existing v1
    // .oaptl files and catalog.json. In the domain these are library defaults,
    // not Job ToolController instances.
    for (const LibraryToolSlot& slot : library.toolSlots()) {
        controllers.append(QJsonObject{
            {QStringLiteral("tool_number"), slot.toolNumber},
            {QStringLiteral("tool_id"), slot.toolId},
            {QStringLiteral("coolant"), coolantModeId(slot.coolantDefault)},
            {QStringLiteral("cutting"), cuttingToJson(slot.cuttingDefaults)},
        });
    }

    return {
        {QStringLiteral("id"), library.id()},
        {QStringLiteral("name"), library.name()},
        {QStringLiteral("machine"), library.machine()},
        {QStringLiteral("description"), library.description()},
        {QStringLiteral("attributes"), libraryAttributes},
        {QStringLiteral("tools"), tools},
        {QStringLiteral("controllers"), controllers},
    };
}

// 反序列化单刀库：临时对象全量解析 + 领域校验，成功才替换输出参数。
bool libraryFromJson(const QJsonObject& object, ToolLibrary& library, QString* error)
{
    // 先在临时对象中完整解析并走领域校验，失败时不污染调用方已有刀库。
    ToolLibrary parsed(object.value(QStringLiteral("id")).toString(),
                       object.value(QStringLiteral("name")).toString(),
                       object.value(QStringLiteral("machine")).toString());
    parsed.setDescription(object.value(QStringLiteral("description")).toString());
    const QJsonObject libraryAttributes =
        object.value(QStringLiteral("attributes")).toObject();
    for (auto it = libraryAttributes.begin(); it != libraryAttributes.end(); ++it)
        parsed.setAttribute(it.key(), it.value().toVariant().toString());

    const QJsonValue toolsValue = object.value(QStringLiteral("tools"));
    if (!toolsValue.isArray()) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀库缺少 tools 数组");
        return false;
    }
    for (const QJsonValue& value : toolsValue.toArray()) {
        if (!value.isObject()) continue;
        ToolBit tool;
        QString itemError;
        if (!toolFromJson(value.toObject(), tool, &itemError)) {
            if (error) *error = itemError;
            return false;
        }
        if (!parsed.upsertTool(std::move(tool), &itemError)) {
            if (error) *error = itemError;
            return false;
        }
    }

    const QJsonValue controllersValue = object.value(QStringLiteral("controllers"));
    if (!controllersValue.isArray()) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀库缺少 controllers 数组");
        return false;
    }
    for (const QJsonValue& value : controllersValue.toArray()) {
        if (!value.isObject()) continue;
        const QJsonObject item = value.toObject();
        LibraryToolSlot slot;
        slot.toolNumber = item.value(QStringLiteral("tool_number")).toInt();
        slot.toolId = item.value(QStringLiteral("tool_id")).toString();
        slot.coolantDefault = coolantModeFromId(
            item.value(QStringLiteral("coolant")).toString());
        slot.cuttingDefaults = cuttingFromJson(
            item.value(QStringLiteral("cutting")).toObject());
        QString itemError;
        if (!parsed.assignToolToSlot(std::move(slot), &itemError)) {
            if (error) *error = itemError;
            return false;
        }
    }

    QString validationError;
    if (!parsed.isValid(&validationError)) {
        if (error) *error = validationError;
        return false;
    }
    library = std::move(parsed);
    return true;
}

QJsonObject libraryReferenceToJson(const ToolLibrary& library)
{
    QJsonObject object = libraryToJson(library);
    object.remove(QStringLiteral("tools"));
    QJsonArray references;
    for (const ToolBit& tool : library.tools())
        references.append(tool.id);
    object.insert(QStringLiteral("tool_refs"), references);
    return object;
}

bool libraryFromReferencesJson(const QJsonObject& object,
                               const ToolAssetCatalog& assets,
                               ToolLibrary& library,
                               QString* error)
{
    const QJsonValue referencesValue = object.value(QStringLiteral("tool_refs"));
    if (!referencesValue.isArray()) {
        if (error)
            *error = QCoreApplication::translate("OpenAutoPath", "刀库缺少 tool_refs 数组");
        return false;
    }
    QJsonArray tools;
    for (const QJsonValue& value : referencesValue.toArray()) {
        const QString assetId = value.toString();
        const ToolBit* asset = assets.assetById(assetId);
        if (!asset) {
            if (error) {
                *error = QCoreApplication::translate("OpenAutoPath", "刀库引用了不存在的全局刀具资产：%1")
                             .arg(assetId);
            }
            return false;
        }
        tools.append(toolToJson(*asset));
    }
    QJsonObject materialized = object;
    materialized.insert(QStringLiteral("tools"), tools);
    return libraryFromJson(materialized, library, error);
}

// 解析 JSON 并确认顶层为对象；失败时报解析错误与偏移。
bool parseDocument(const QByteArray& data, QJsonDocument& document, QString* error)
{
    QJsonParseError parseError;
    document = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        if (error) {
            *error = QCoreApplication::translate("OpenAutoPath", "JSON 解析失败：%1（偏移 %2）")
                         .arg(parseError.errorString()).arg(parseError.offset);
        }
        return false;
    }
    return true;
}

bool writeAtomically(const QString& filePath, const QByteArray& data, QString* error)
{
    // QSaveFile 通过临时文件提交，写入失败时保留原有刀库文件。
    const QFileInfo info(filePath);
    if (!QDir().mkpath(info.absolutePath())) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "无法创建目录：%1").arg(info.absolutePath());
        return false;
    }
    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "无法写入文件：%1").arg(file.errorString());
        return false;
    }
    if (file.write(data) != data.size() || !file.commit()) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "保存刀库失败：%1").arg(file.errorString());
        return false;
    }
    return true;
}

double freeCadQuantity(const QJsonObject& parameters, const QString& key)
{
    // FreeCAD 数量字段可能是数值，也可能是带单位字符串；本层只读取前导数值。
    const QJsonValue value = parameters.value(key);
    if (value.isDouble()) return value.toDouble();
    const QString text = value.toString().trimmed();
    static const QRegularExpression number(
        QStringLiteral(R"(^\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?))"));
    const QRegularExpressionMatch match = number.match(text);
    return match.hasMatch() ? match.captured(1).toDouble() : 0.0;
}

// 按 FreeCAD 的 shape-type + shape 关键字推断刀具类型；无法识别回退 Unknown。
ToolType freeCadToolType(const QString& shapeType, const QString& shape)
{
    const QString key = (shapeType + QLatin1Char(' ') + shape).toLower();
    if (key.contains(QStringLiteral("drill"))) return ToolType::Drill;
    if (key.contains(QStringLiteral("ball"))) return ToolType::BallEndMill;
    if (key.contains(QStringLiteral("bull")) || key.contains(QStringLiteral("radius")))
        return ToolType::BullNose;
    if (key.contains(QStringLiteral("chamfer"))) return ToolType::ChamferMill;
    if (key.contains(QStringLiteral("boring"))) return ToolType::BoringBar;
    if (key.contains(QStringLiteral("ream"))) return ToolType::Reamer;
    if (key.contains(QStringLiteral("tap"))) return ToolType::Tap;
    if (key.contains(QStringLiteral("probe"))) return ToolType::Probe;
    if (key.contains(QStringLiteral("endmill")) || key.contains(QStringLiteral("end_mill")))
        return ToolType::EndMill;
    return ToolType::Unknown;
}

// 解析 FreeCAD 的 .fctb（Bit/*.fctb）刀具文件；id 取文件主名。
bool loadFreeCadToolBit(const QString& filePath, ToolBit& tool, QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "无法读取 FreeCAD 刀具 %1：%2")
                                .arg(filePath, file.errorString());
        return false;
    }
    QByteArray data;
    if (!readLimitedJson(file, data, error)) return false;
    QJsonDocument document;
    if (!parseDocument(data, document, error)) return false;
    const QJsonObject object = document.object();
    const QFileInfo info(filePath);
    tool.id = info.completeBaseName();
    tool.name = object.value(QStringLiteral("name")).toString(tool.id);
    const QString shape = object.value(QStringLiteral("shape")).toString();
    const QString shapeType = object.value(QStringLiteral("shape-type")).toString();
    tool.type = freeCadToolType(shapeType, shape);
    const QJsonObject parameters = object.value(QStringLiteral("parameter")).toObject();
    tool.geometry.diameter = freeCadQuantity(parameters, QStringLiteral("Diameter"));
    tool.geometry.overallLength = freeCadQuantity(parameters, QStringLiteral("Length"));
    tool.geometry.cuttingEdgeLength =
        freeCadQuantity(parameters, QStringLiteral("CuttingEdgeHeight"));
    tool.geometry.cornerRadius = freeCadQuantity(parameters, QStringLiteral("CornerRadius"));
    tool.geometry.tipAngle = freeCadQuantity(parameters, QStringLiteral("TipAngle"));
    if (tool.geometry.tipAngle <= 0.0) {
        tool.geometry.tipAngle = freeCadQuantity(
            parameters, QStringLiteral("CuttingEdgeAngle"));
    }
    tool.geometry.shankDiameter =
        freeCadQuantity(parameters, QStringLiteral("ShankDiameter"));
    const QJsonObject attributes = object.value(QStringLiteral("attribute")).toObject();
    for (auto it = attributes.begin(); it != attributes.end(); ++it)
        tool.attributes.insert(it.key(), it.value().toVariant().toString());
    const int fluteCount = static_cast<int>(std::lround(
        freeCadQuantity(parameters, QStringLiteral("Flutes"))));
    if (fluteCount > 0)
        tool.attributes.insert(QStringLiteral("flutes"),
                               QString::number(fluteCount));
    tool.attributes.insert(QStringLiteral("freecad_shape"), shape);
    return true;
}

// 解析 FreeCAD 的 .fctl 刀库：逐条装载 ToolBit 依赖并建立刀号控制器。
bool loadFreeCadLibrary(const QString& filePath,
                        const QJsonObject& object,
                        ToolLibrary& library,
                        QString* error)
{
    // .fctl 只保存刀号和 ToolBit 路径，需同时兼容库旁路径与 FreeCAD ../Bit 布局。
    const QFileInfo libraryFile(filePath);
    ToolLibrary parsed(libraryFile.completeBaseName(),
                       object.value(QStringLiteral("label")).toString(
                           libraryFile.completeBaseName()));
    parsed.setDescription(QCoreApplication::translate("OpenAutoPath", "从 FreeCAD .fctl 导入"));
    const QJsonArray entries = object.value(QStringLiteral("tools")).toArray();
    for (const QJsonValue& value : entries) {
        const QJsonObject entry = value.toObject();
        const int number = entry.value(QStringLiteral("nr")).toInt();
        const QString relative = entry.value(QStringLiteral("path")).toString();
        const QString fileName = QFileInfo(relative).fileName();
        const QString libraryRoot = libraryFile.dir().absolutePath();
        const QString bitRoot =
            QDir(libraryFile.dir().filePath(QStringLiteral("../Bit"))).absolutePath();
        const QStringList roots{libraryRoot, bitRoot};
        const QStringList candidates{
            QDir(libraryRoot).filePath(relative),
            QDir(bitRoot).filePath(fileName),
        };
        QString bitPath;
        for (qsizetype index = 0; index < candidates.size(); ++index) {
            const QString& candidate = candidates[index];
            if (isContainedFile(roots[index], candidate)) {
                bitPath = candidate;
                break;
            }
        }

        ToolBit tool;
        QString bitError;
        if (bitPath.isEmpty() || !loadFreeCadToolBit(bitPath, tool, &bitError)) {
            tool.id = QFileInfo(fileName).completeBaseName();
            tool.name = QCoreApplication::translate("OpenAutoPath", "缺失刀具：%1").arg(fileName);
            tool.type = ToolType::Unknown;
            tool.note = bitPath.isEmpty()
                ? QCoreApplication::translate("OpenAutoPath", "未找到 FreeCAD 刀具依赖")
                : bitError;
        }
        QString itemError;
        if (!parsed.upsertTool(tool, &itemError)) {
            if (error) *error = itemError;
            return false;
        }
        LibraryToolSlot slot;
        slot.toolNumber = number;
        slot.toolId = tool.id;
        if (!parsed.assignToolToSlot(slot, &itemError)) {
            if (error) *error = itemError;
            return false;
        }
    }
    library = std::move(parsed);
    return true;
}

} // namespace

// 单刀库序列化：根对象带 schema 标识与版本号，供 deserializeLibrary 识别。
QByteArray ToolLibraryStorage::serializeLibrary(const ToolLibrary& library)
{
    const QJsonObject root{
        {QStringLiteral("schema"), QStringLiteral("oap-tool-library")},
        {QStringLiteral("version"), kLibrarySchemaVersion},
        {QStringLiteral("library"), libraryToJson(library)},
    };
    return QJsonDocument(root).toJson(QJsonDocument::Indented);
}

// 单刀库反序列化：校验 schema 与版本，再走领域校验；失败不污染 library。
bool ToolLibraryStorage::deserializeLibrary(const QByteArray& data,
                                            ToolLibrary& library,
                                            QString* error)
{
    QJsonDocument document;
    if (!parseDocument(data, document, error)) return false;
    const QJsonObject root = document.object();
    if (!isLibrarySchema(root.value(QStringLiteral("schema")).toString())) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "不是 OpenAutoPath 刀库文件");
        return false;
    }
    if (root.value(QStringLiteral("version")).toInt() !=
        kLibrarySchemaVersion) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "不支持的刀库版本：%1")
                                .arg(root.value(QStringLiteral("version")).toInt());
        return false;
    }
    return libraryFromJson(root.value(QStringLiteral("library")).toObject(),
                           library, error);
}

// 保存单刀库：先校验，再原子写盘；不合法时不落盘。
bool ToolLibraryStorage::saveLibrary(const QString& filePath,
                                     const ToolLibrary& library,
                                     QString* error)
{
    QString validationError;
    if (!library.isValid(&validationError)) {
        if (error) *error = validationError;
        return false;
    }
    return writeAtomically(filePath, serializeLibrary(library), error);
}

// 加载单刀库或 FreeCAD .fctl：按根字段自动判别格式。
bool ToolLibraryStorage::loadLibrary(const QString& filePath,
                                     ToolLibrary& library,
                                     QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "无法读取刀库：%1").arg(file.errorString());
        return false;
    }
    QByteArray data;
    if (!readLimitedJson(file, data, error)) return false;
    QJsonDocument document;
    if (!parseDocument(data, document, error)) return false;
    const QJsonObject root = document.object();
    // 通过根字段自动区分本项目格式和 FreeCAD .fctl，调用方无需预先指定来源。
    if (isLibrarySchema(root.value(QStringLiteral("schema")).toString())) {
        return deserializeLibrary(data, library, error);
    }
    if (root.value(QStringLiteral("tools")).isArray())
        return loadFreeCadLibrary(filePath, root, library, error);
    if (error) *error = QCoreApplication::translate("OpenAutoPath", "未知刀库格式");
    return false;
}

// 保存整个刀库目录：逐个校验后合并为 catalog.json 原子写入。
bool ToolLibraryStorage::saveCatalog(const QString& filePath,
                                     const ToolAssetCatalog& catalog,
                                     QString* error)
{
    QString validationError;
    if (!catalog.isValid(&validationError)) {
        if (error)
            *error = QCoreApplication::translate("OpenAutoPath", "刀具资产目录无效：\n%1")
                         .arg(validationError);
        return false;
    }

    QJsonArray assets;
    for (const ToolBit& tool : catalog.assets())
        assets.append(toolToJson(tool));
    QJsonArray libraries;
    for (const ToolLibrary& library : catalog.libraries())
        libraries.append(libraryReferenceToJson(library));
    const QJsonObject root{
        {QStringLiteral("schema"), QStringLiteral("oap-tool-library-catalog")},
        {QStringLiteral("version"), kCatalogSchemaVersion},
        {QStringLiteral("tool_assets"), assets},
        {QStringLiteral("libraries"), libraries},
    };
    return writeAtomically(filePath,
                           QJsonDocument(root).toJson(QJsonDocument::Indented), error);
}

bool ToolLibraryStorage::saveCatalog(const QString& filePath,
                                     const QVector<ToolLibrary>& libraries,
                                     QString* error)
{
    ToolAssetCatalog catalog;
    if (!ToolAssetCatalog::fromLibraries(libraries, catalog, error))
        return false;
    return saveCatalog(filePath, catalog, error);
}

// 加载刀库目录；旧版本仅迁移存储结构，保留用户提供的参数。
bool ToolLibraryStorage::loadCatalog(const QString& filePath,
                                     ToolAssetCatalog& catalog,
                                     QString* error,
                                     bool* requiresCatalogRewrite)
{
    if (requiresCatalogRewrite)
        *requiresCatalogRewrite = false;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "无法读取刀库目录：%1").arg(file.errorString());
        return false;
    }
    QByteArray data;
    if (!readLimitedJson(file, data, error)) return false;
    QJsonDocument document;
    if (!parseDocument(data, document, error)) return false;
    const QJsonObject root = document.object();
    const int version =
        root.value(QStringLiteral("version")).toInt();
    const QString schema = root.value(QStringLiteral("schema")).toString();
    if ((schema != QStringLiteral("oap-tool-library-catalog") &&
         schema != QStringLiteral("faceface-tool-library-catalog")) ||
        version < kOldestSupportedCatalogVersion ||
        version > kCatalogSchemaVersion) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "刀库目录格式或版本不受支持");
        return false;
    }

    ToolAssetCatalog parsed;
    bool identifiersMigrated = false;
    const bool usesGlobalAssets =
        root.value(QStringLiteral("tool_assets")).isArray();
    if (usesGlobalAssets) {
        for (const QJsonValue& value :
             root.value(QStringLiteral("tool_assets")).toArray()) {
            if (!value.isObject())
                continue;
            ToolBit tool;
            QString itemError;
            if (!toolFromJson(value.toObject(), tool, &itemError)) {
                if (error)
                    *error = itemError;
                return false;
            }
            if (parsed.assetById(tool.id)) {
                if (error)
                    *error = QCoreApplication::translate("OpenAutoPath", "全局刀具资产 ID 重复：%1").arg(tool.id);
                return false;
            }
            if (!parsed.upsertAsset(std::move(tool), &itemError)) {
                if (error)
                    *error = itemError;
                return false;
            }
        }
        for (const QJsonValue& value :
             root.value(QStringLiteral("libraries")).toArray()) {
            ToolLibrary library;
            QString itemError;
            if (!libraryFromReferencesJson(value.toObject(), parsed,
                                           library, &itemError) ||
                !parsed.addLibrary(std::move(library), &itemError)) {
                if (error)
                    *error = itemError;
                return false;
            }
        }
    } else {
        QVector<ToolLibrary> legacyLibraries;
        for (const QJsonValue& value :
             root.value(QStringLiteral("libraries")).toArray()) {
            ToolLibrary library;
            QString itemError;
            if (!libraryFromJson(value.toObject(), library, &itemError)) {
                if (error)
                    *error = itemError;
                return false;
            }
            legacyLibraries.push_back(std::move(library));
        }
        if (!ToolAssetCatalog::fromLibraries(
                std::move(legacyLibraries), parsed, error,
                &identifiersMigrated)) {
            return false;
        }
    }
    catalog = std::move(parsed);
    if (requiresCatalogRewrite)
        *requiresCatalogRewrite =
            schema != QStringLiteral("oap-tool-library-catalog") || !usesGlobalAssets ||
            version < kCatalogSchemaVersion || identifiersMigrated;
    return true;
}

bool ToolLibraryStorage::loadCatalog(const QString& filePath,
                                     QVector<ToolLibrary>& libraries,
                                     QString* error)
{
    ToolAssetCatalog catalog;
    if (!loadCatalog(filePath, catalog, error)) {
        return false;
    }
    libraries = catalog.libraries();
    return true;
}

// 默认目录路径：AppConfigLocation/tool-libraries/catalog.json，兜底为程序目录 config。
QString ToolLibraryStorage::defaultCatalogPath()
{
    QString base = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (base.isEmpty())
        base = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config"));
    return QDir(base).filePath(QStringLiteral("tool-libraries/catalog.json"));
}

} // namespace oap::tooling
