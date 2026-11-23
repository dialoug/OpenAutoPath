// 刀库 JSON 持久化边界：负责版本/字段校验、兼容导入和安全覆盖写入。
#include "support/Files.h"
#include "support/Utf8Path.h"
#include <regex>
#include "support/Json.h"
#include "support/Text.h"
#include <format>
#include <vector>
#include <string>
#include "tooling/library/ToolLibraryStorage.h"


#include <cmath>

namespace oap::tooling {
namespace {

// 单刀库 schema 版本为 1；目录 schema v3 将 ToolBit 提升为全局资产。
// v1/v2 仅迁移资产存储结构，保留用户参数。
constexpr int kLibrarySchemaVersion = 1;
constexpr int kCatalogSchemaVersion = 3;
constexpr int kOldestSupportedCatalogVersion = 1;
// 单文件大小上限 16 MiB，防止读取超大文件。
constexpr std::int64_t kMaxToolLibraryBytes = 16ll * 1024ll * 1024ll;

// 旧标识仅用于显式导入兼容；写入始终使用 OAP 标识。
bool isLibrarySchema(const std::string& schema)
{
    return schema == std::string("oap-tool-library") ||
           schema == std::string("faceface-tool-library");
}

// 受大小限制地读取整个文件；超限或读取出错时报错并返回 false。
bool isContainedFile(const std::string& rootPath, const std::string& candidatePath)
{
    std::error_code error;
    const auto root = std::filesystem::canonical(oap::utf8Path(rootPath), error);
    if (error) return false;
    const auto candidate = std::filesystem::canonical(oap::utf8Path(candidatePath), error);
    if (error || !std::filesystem::is_regular_file(candidate, error)) return false;
    const auto relative = candidate.lexically_relative(root);
    return !relative.empty() && !relative.is_absolute() && *relative.begin() != "..";
}

// ToolGeometry ↔ JSON 的字段映射，字段名即持久化格式的一部分，勿随意改名。
oap::JsonObject geometryToJson(const ToolGeometry& geometry)
{
    return {
        {std::string("diameter"), geometry.diameter},
        {std::string("cutting_edge_length"), geometry.cuttingEdgeLength},
        {std::string("overall_length"), geometry.overallLength},
        {std::string("corner_radius"), geometry.cornerRadius},
        {std::string("tip_angle"), geometry.tipAngle},
        {std::string("tip_compensation"), geometry.tipCompensation},
        {std::string("shank_diameter"), geometry.shankDiameter},
        {std::string("holder_diameter"), geometry.holderDiameter},
        {std::string("holder_length"), geometry.holderLength},
        {std::string("spindle_envelope_diameter"),
         geometry.spindleEnvelopeDiameter},
        {std::string("spindle_envelope_length"),
         geometry.spindleEnvelopeLength},
    };
}

// 反序列化几何：缺失字段按 0（未知）处理，语义与领域模型约定一致。
ToolGeometry geometryFromJson(const oap::JsonObject& object)
{
    ToolGeometry geometry;
    geometry.diameter = object.value(std::string("diameter")).toDouble();
    geometry.cuttingEdgeLength =
        object.value(std::string("cutting_edge_length")).toDouble();
    geometry.overallLength = object.value(std::string("overall_length")).toDouble();
    geometry.cornerRadius = object.value(std::string("corner_radius")).toDouble();
    geometry.tipAngle = object.value(std::string("tip_angle")).toDouble();
    geometry.tipCompensation =
        object.value(std::string("tip_compensation")).toDouble();
    geometry.shankDiameter =
        object.value(std::string("shank_diameter")).toDouble();
    geometry.holderDiameter =
        object.value(std::string("holder_diameter")).toDouble();
    geometry.holderLength =
        object.value(std::string("holder_length")).toDouble();
    geometry.spindleEnvelopeDiameter = object.value(
        std::string("spindle_envelope_diameter")).toDouble();
    geometry.spindleEnvelopeLength = object.value(
        std::string("spindle_envelope_length")).toDouble();
    return geometry;
}

// 切削参数 → JSON 的字段映射（含钻循环字符串 ID）。
oap::JsonObject cuttingToJson(const CuttingParameters& cutting)
{
    return {
        {std::string("spindle_speed"), cutting.spindleSpeed},
        {std::string("horizontal_feed"), cutting.horizontalFeed},
        {std::string("vertical_feed"), cutting.verticalFeed},
        {std::string("horizontal_rapid"), cutting.horizontalRapid},
        {std::string("vertical_rapid"), cutting.verticalRapid},
        {std::string("peck_depth"), cutting.peckDepth},
        {std::string("drill_cycle"), drillCycleId(cutting.drillCycle)},
    };
}

oap::JsonObject toolToJson(const ToolBit& tool)
{
    oap::JsonObject attributes;
    for (auto it = tool.attributes.cbegin(); it != tool.attributes.cend(); ++it)
        attributes.insert(it->first, it->second);
    return {
        {std::string("id"), tool.id},
        {std::string("name"), tool.name},
        {std::string("type"), toolTypeId(tool.type)},
        {std::string("geometry"), geometryToJson(tool.geometry)},
        {std::string("note"), tool.note},
        {std::string("through_coolant"), tool.throughCoolant},
        {std::string("attributes"), attributes},
    };
}

bool toolFromJson(const oap::JsonObject& item, ToolBit& tool, std::string* error)
{
    ToolBit parsed;
    parsed.id = item.value(std::string("id")).toString();
    parsed.name = item.value(std::string("name")).toString();
    parsed.type = toolTypeFromId(item.value(std::string("type")).toString());
    parsed.geometry = geometryFromJson(
        item.value(std::string("geometry")).toObject());
    parsed.note = item.value(std::string("note")).toString();
    parsed.throughCoolant =
        item.value(std::string("through_coolant")).toBool();
    const oap::JsonObject attributes =
        item.value(std::string("attributes")).toObject();
    for (auto it = attributes.begin(); it != attributes.end(); ++it)
        parsed.attributes.insert_or_assign(it.key(), it.value().scalarText());

    ToolLibrary validator(std::string("asset-validator"),
                          std::string("asset-validator"));
    std::string validationError;
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
CuttingParameters cuttingFromJson(const oap::JsonObject& object)
{
    CuttingParameters cutting;
    cutting.spindleSpeed = object.value(std::string("spindle_speed")).toDouble();
    cutting.horizontalFeed = object.value(std::string("horizontal_feed")).toDouble();
    cutting.verticalFeed = object.value(std::string("vertical_feed")).toDouble();
    cutting.horizontalRapid = object.value(std::string("horizontal_rapid")).toDouble();
    cutting.verticalRapid = object.value(std::string("vertical_rapid")).toDouble();
    cutting.peckDepth = object.value(std::string("peck_depth")).toDouble();
    cutting.drillCycle =
        drillCycleFromId(object.value(std::string("drill_cycle")).toString());
    return cutting;
}

oap::JsonObject libraryToJson(const ToolLibrary& library)
{
    // 刀具定义与刀位控制器分开存储；控制器只通过稳定 tool_id 引用刀具。
    oap::JsonObject libraryAttributes;
    for (auto it = library.attributes().cbegin(); it != library.attributes().cend(); ++it)
        libraryAttributes.insert(it->first, it->second);

    oap::JsonArray tools;
    for (const auto& [key, tool] : library.tools())
        tools.append(toolToJson(tool));

    oap::JsonArray controllers;
    // Wire keys stay unchanged for backward compatibility with existing v1
    // .oaptl files and catalog.json. In the domain these are library defaults,
    // not Job ToolController instances.
    for (const auto& [key, slot] : library.toolSlots()) {
        controllers.append(oap::JsonObject{
            {std::string("tool_number"), slot.toolNumber},
            {std::string("tool_id"), slot.toolId},
            {std::string("coolant"), coolantModeId(slot.coolantDefault)},
            {std::string("cutting"), cuttingToJson(slot.cuttingDefaults)},
        });
    }

    return {
        {std::string("id"), library.id()},
        {std::string("name"), library.name()},
        {std::string("machine"), library.machine()},
        {std::string("description"), library.description()},
        {std::string("attributes"), libraryAttributes},
        {std::string("tools"), tools},
        {std::string("controllers"), controllers},
    };
}

// 反序列化单刀库：临时对象全量解析 + 领域校验，成功才替换输出参数。
bool libraryFromJson(const oap::JsonObject& object, ToolLibrary& library, std::string* error)
{
    // 先在临时对象中完整解析并走领域校验，失败时不污染调用方已有刀库。
    ToolLibrary parsed(object.value(std::string("id")).toString(),
                       object.value(std::string("name")).toString(),
                       object.value(std::string("machine")).toString());
    parsed.setDescription(object.value(std::string("description")).toString());
    const oap::JsonObject libraryAttributes =
        object.value(std::string("attributes")).toObject();
    for (auto it = libraryAttributes.begin(); it != libraryAttributes.end(); ++it)
        parsed.setAttribute(it.key(), it.value().scalarText());

    const oap::JsonValue toolsValue = object.value(std::string("tools"));
    if (!toolsValue.isArray()) {
        if (error) *error = std::string("刀库缺少 tools 数组");
        return false;
    }
    for (const oap::JsonValue& value : toolsValue.toArray()) {
        if (!value.isObject()) continue;
        ToolBit tool;
        std::string itemError;
        if (!toolFromJson(value.toObject(), tool, &itemError)) {
            if (error) *error = itemError;
            return false;
        }
        if (!parsed.upsertTool(std::move(tool), &itemError)) {
            if (error) *error = itemError;
            return false;
        }
    }

    const oap::JsonValue controllersValue = object.value(std::string("controllers"));
    if (!controllersValue.isArray()) {
        if (error) *error = std::string("刀库缺少 controllers 数组");
        return false;
    }
    for (const oap::JsonValue& value : controllersValue.toArray()) {
        if (!value.isObject()) continue;
        const oap::JsonObject item = value.toObject();
        LibraryToolSlot slot;
        slot.toolNumber = item.value(std::string("tool_number")).toInt();
        slot.toolId = item.value(std::string("tool_id")).toString();
        slot.coolantDefault = coolantModeFromId(
            item.value(std::string("coolant")).toString());
        slot.cuttingDefaults = cuttingFromJson(
            item.value(std::string("cutting")).toObject());
        std::string itemError;
        if (!parsed.assignToolToSlot(std::move(slot), &itemError)) {
            if (error) *error = itemError;
            return false;
        }
    }

    std::string validationError;
    if (!parsed.isValid(&validationError)) {
        if (error) *error = validationError;
        return false;
    }
    library = std::move(parsed);
    return true;
}

oap::JsonObject libraryReferenceToJson(const ToolLibrary& library)
{
    oap::JsonObject object = libraryToJson(library);
    object.remove(std::string("tools"));
    oap::JsonArray references;
    for (const auto& [key, tool] : library.tools())
        references.append(tool.id);
    object.insert(std::string("tool_refs"), references);
    return object;
}

bool libraryFromReferencesJson(const oap::JsonObject& object,
                               const ToolAssetCatalog& assets,
                               ToolLibrary& library,
                               std::string* error)
{
    const oap::JsonValue referencesValue = object.value(std::string("tool_refs"));
    if (!referencesValue.isArray()) {
        if (error)
            *error = std::string("刀库缺少 tool_refs 数组");
        return false;
    }
    oap::JsonArray tools;
    for (const oap::JsonValue& value : referencesValue.toArray()) {
        const std::string assetId = value.toString();
        const ToolBit* asset = assets.assetById(assetId);
        if (!asset) {
            if (error) {
                *error = std::format("刀库引用了不存在的全局刀具资产：{0}", assetId);
            }
            return false;
        }
        tools.append(toolToJson(*asset));
    }
    oap::JsonObject materialized = object;
    materialized.insert(std::string("tools"), tools);
    return libraryFromJson(materialized, library, error);
}

// 解析 JSON 并确认顶层为对象；失败时报解析错误与偏移。
bool parseDocument(const std::string& data, oap::JsonDocument& document, std::string* error)
{
    oap::JsonParseError parseError;
    document = oap::JsonDocument::fromJson(data, &parseError);
    if (parseError || !document.isObject()) {
        if (error) {
            *error = std::format("JSON 解析失败：{0}（偏移 {1}）", parseError.message, parseError.offset);
        }
        return false;
    }
    return true;
}

bool writeAtomically(const std::string& filePath, const std::string& data, std::string* error)
{
    const auto parent = oap::utf8Path(filePath).parent_path();
    std::error_code failure;
    if (!parent.empty()) std::filesystem::create_directories(parent, failure);
    if (failure) {
        if (error) *error = failure.message();
        return false;
    }
    return oap::writeFileAtomically(filePath, data, error);
}

double freeCadQuantity(const oap::JsonObject& parameters, const std::string& key)
{
    // FreeCAD 数量字段可能是数值，也可能是带单位字符串；本层只读取前导数值。
    const oap::JsonValue value = parameters.value(key);
    if (value.isDouble()) return value.toDouble();
    const std::string text = oap::trim(value.toString());
    static const std::regex number(R"(^([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?))");
    std::smatch match;
    if (!std::regex_search(text, match, number)) return 0.0;
    try {
        return std::stod(match[1].str());
    } catch (const std::out_of_range&) {
        return 0.0;
    }
}

// 按 FreeCAD 的 shape-type + shape 关键字推断刀具类型；无法识别回退 Unknown。
ToolType freeCadToolType(const std::string& shapeType, const std::string& shape)
{
    const std::string key = oap::lowerAscii(shapeType + ' ' + shape);
    if ((key.find("drill") != std::string::npos)) return ToolType::Drill;
    if ((key.find("ball") != std::string::npos)) return ToolType::BallEndMill;
    if ((key.find("bull") != std::string::npos) || (key.find("radius") != std::string::npos))
        return ToolType::BullNose;
    if ((key.find("chamfer") != std::string::npos)) return ToolType::ChamferMill;
    if ((key.find("boring") != std::string::npos)) return ToolType::BoringBar;
    if ((key.find("ream") != std::string::npos)) return ToolType::Reamer;
    if ((key.find("tap") != std::string::npos)) return ToolType::Tap;
    if ((key.find("probe") != std::string::npos)) return ToolType::Probe;
    if ((key.find("endmill") != std::string::npos) || (key.find("end_mill") != std::string::npos))
        return ToolType::EndMill;
    return ToolType::Unknown;
}

// 解析 FreeCAD 的 .fctb（Bit/*.fctb）刀具文件；id 取文件主名。
bool loadFreeCadToolBit(const std::string& filePath, ToolBit& tool, std::string* error)
{
    std::string data;
    if (!oap::readFile(filePath, data, error, kMaxToolLibraryBytes)) return false;
    oap::JsonDocument document;
    if (!parseDocument(data, document, error)) return false;
    const oap::JsonObject object = document.object();
    tool.id = oap::pathUtf8(oap::utf8Path(filePath).stem());
    tool.name = object.value(std::string("name")).toString(tool.id);
    const std::string shape = object.value(std::string("shape")).toString();
    const std::string shapeType = object.value(std::string("shape-type")).toString();
    tool.type = freeCadToolType(shapeType, shape);
    const oap::JsonObject parameters = object.value(std::string("parameter")).toObject();
    tool.geometry.diameter = freeCadQuantity(parameters, std::string("Diameter"));
    tool.geometry.overallLength = freeCadQuantity(parameters, std::string("Length"));
    tool.geometry.cuttingEdgeLength =
        freeCadQuantity(parameters, std::string("CuttingEdgeHeight"));
    tool.geometry.cornerRadius = freeCadQuantity(parameters, std::string("CornerRadius"));
    tool.geometry.tipAngle = freeCadQuantity(parameters, std::string("TipAngle"));
    if (tool.geometry.tipAngle <= 0.0) {
        tool.geometry.tipAngle = freeCadQuantity(
            parameters, std::string("CuttingEdgeAngle"));
    }
    tool.geometry.shankDiameter =
        freeCadQuantity(parameters, std::string("ShankDiameter"));
    const oap::JsonObject attributes = object.value(std::string("attribute")).toObject();
    for (auto it = attributes.begin(); it != attributes.end(); ++it)
        tool.attributes.insert_or_assign(it.key(), it.value().scalarText());
    const int fluteCount = static_cast<int>(std::lround(
        freeCadQuantity(parameters, std::string("Flutes"))));
    if (fluteCount > 0)
        tool.attributes.insert_or_assign(std::string("flutes"),
                               std::to_string(fluteCount));
    tool.attributes.insert_or_assign(std::string("freecad_shape"), shape);
    return true;
}

// 解析 FreeCAD 的 .fctl 刀库：逐条装载 ToolBit 依赖并建立刀号控制器。
bool loadFreeCadLibrary(const std::string& filePath,
                        const oap::JsonObject& object,
                        ToolLibrary& library,
                        std::string* error)
{
    // .fctl 只保存刀号和 ToolBit 路径，需同时兼容库旁路径与 FreeCAD ../Bit 布局。
    const auto libraryFile = oap::utf8Path(filePath);
    const auto libraryName = oap::pathUtf8(libraryFile.stem());
    ToolLibrary parsed(libraryName,
                       object.value(std::string("label")).toString(
                           libraryName));
    parsed.setDescription(std::string("从 FreeCAD .fctl 导入"));
    const oap::JsonArray entries = object.value(std::string("tools")).toArray();
    for (const oap::JsonValue& value : entries) {
        const oap::JsonObject entry = value.toObject();
        const int number = entry.value(std::string("nr")).toInt();
        const std::string relative = entry.value(std::string("path")).toString();
        const std::string fileName = oap::pathUtf8(oap::utf8Path(relative).filename());
        const std::string libraryRoot = oap::pathUtf8(std::filesystem::absolute(libraryFile).parent_path());
        const std::string bitRoot =
            oap::pathUtf8(oap::utf8Path(libraryRoot).parent_path() / "Bit");
        const std::vector<std::string> roots{libraryRoot, bitRoot};
        const std::vector<std::string> candidates{
            oap::pathUtf8(oap::utf8Path(libraryRoot) / oap::utf8Path(relative)),
            oap::pathUtf8(oap::utf8Path(bitRoot) / oap::utf8Path(fileName)),
        };
        std::string bitPath;
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            const std::string& candidate = candidates[index];
            if (isContainedFile(roots[index], candidate)) {
                bitPath = candidate;
                break;
            }
        }

        ToolBit tool;
        std::string bitError;
        if (bitPath.empty() || !loadFreeCadToolBit(bitPath, tool, &bitError)) {
            tool.id = oap::pathUtf8(oap::utf8Path(fileName).stem());
            tool.name = std::format("缺失刀具：{0}", fileName);
            tool.type = ToolType::Unknown;
            tool.note = bitPath.empty()
                ? std::string("未找到 FreeCAD 刀具依赖")
                : bitError;
        }
        std::string itemError;
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
std::string ToolLibraryStorage::serializeLibrary(const ToolLibrary& library)
{
    const oap::JsonObject root{
        {std::string("schema"), std::string("oap-tool-library")},
        {std::string("version"), kLibrarySchemaVersion},
        {std::string("library"), libraryToJson(library)},
    };
    return oap::JsonDocument(root).toJson(oap::JsonDocument::Indented);
}

// 单刀库反序列化：校验 schema 与版本，再走领域校验；失败不污染 library。
bool ToolLibraryStorage::deserializeLibrary(const std::string& data,
                                            ToolLibrary& library,
                                            std::string* error)
{
    oap::JsonDocument document;
    if (!parseDocument(data, document, error)) return false;
    const oap::JsonObject root = document.object();
    if (!isLibrarySchema(root.value(std::string("schema")).toString())) {
        if (error) *error = std::string("不是 OpenAutoPath 刀库文件");
        return false;
    }
    if (root.value(std::string("version")).toInt() !=
        kLibrarySchemaVersion) {
        if (error) *error = std::format("不支持的刀库版本：{0}", root.value(std::string("version")).toInt());
        return false;
    }
    return libraryFromJson(root.value(std::string("library")).toObject(),
                           library, error);
}

// 保存单刀库：先校验，再原子写盘；不合法时不落盘。
bool ToolLibraryStorage::saveLibrary(const std::string& filePath,
                                     const ToolLibrary& library,
                                     std::string* error)
{
    std::string validationError;
    if (!library.isValid(&validationError)) {
        if (error) *error = validationError;
        return false;
    }
    return writeAtomically(filePath, serializeLibrary(library), error);
}

// 加载单刀库或 FreeCAD .fctl：按根字段自动判别格式。
bool ToolLibraryStorage::loadLibrary(const std::string& filePath,
                                     ToolLibrary& library,
                                     std::string* error)
{
    std::string data;
    if (!oap::readFile(filePath, data, error, kMaxToolLibraryBytes)) return false;
    oap::JsonDocument document;
    if (!parseDocument(data, document, error)) return false;
    const oap::JsonObject root = document.object();
    // 通过根字段自动区分本项目格式和 FreeCAD .fctl，调用方无需预先指定来源。
    if (isLibrarySchema(root.value(std::string("schema")).toString())) {
        return deserializeLibrary(data, library, error);
    }
    if (root.value(std::string("tools")).isArray())
        return loadFreeCadLibrary(filePath, root, library, error);
    if (error) *error = std::string("未知刀库格式");
    return false;
}

// 保存整个刀库目录：逐个校验后合并为 catalog.json 原子写入。
bool ToolLibraryStorage::saveCatalog(const std::string& filePath,
                                     const ToolAssetCatalog& catalog,
                                     std::string* error)
{
    std::string validationError;
    if (!catalog.isValid(&validationError)) {
        if (error)
            *error = std::format("刀具资产目录无效：\n{0}", validationError);
        return false;
    }

    oap::JsonArray assets;
    for (const auto& [key, tool] : catalog.assets())
        assets.append(toolToJson(tool));
    oap::JsonArray libraries;
    for (const ToolLibrary& library : catalog.libraries())
        libraries.append(libraryReferenceToJson(library));
    const oap::JsonObject root{
        {std::string("schema"), std::string("oap-tool-library-catalog")},
        {std::string("version"), kCatalogSchemaVersion},
        {std::string("tool_assets"), assets},
        {std::string("libraries"), libraries},
    };
    return writeAtomically(filePath,
                           oap::JsonDocument(root).toJson(oap::JsonDocument::Indented), error);
}

bool ToolLibraryStorage::saveCatalog(const std::string& filePath,
                                     const std::vector<ToolLibrary>& libraries,
                                     std::string* error)
{
    ToolAssetCatalog catalog;
    if (!ToolAssetCatalog::fromLibraries(libraries, catalog, error))
        return false;
    return saveCatalog(filePath, catalog, error);
}

// 加载刀库目录；旧版本仅迁移存储结构，保留用户提供的参数。
bool ToolLibraryStorage::loadCatalog(const std::string& filePath,
                                     ToolAssetCatalog& catalog,
                                     std::string* error,
                                     bool* requiresCatalogRewrite)
{
    if (requiresCatalogRewrite)
        *requiresCatalogRewrite = false;
    std::string data;
    if (!oap::readFile(filePath, data, error, kMaxToolLibraryBytes)) return false;
    oap::JsonDocument document;
    if (!parseDocument(data, document, error)) return false;
    const oap::JsonObject root = document.object();
    const int version =
        root.value(std::string("version")).toInt();
    const std::string schema = root.value(std::string("schema")).toString();
    if ((schema != std::string("oap-tool-library-catalog") &&
         schema != std::string("faceface-tool-library-catalog")) ||
        version < kOldestSupportedCatalogVersion ||
        version > kCatalogSchemaVersion) {
        if (error) *error = std::string("刀库目录格式或版本不受支持");
        return false;
    }

    ToolAssetCatalog parsed;
    bool identifiersMigrated = false;
    const bool usesGlobalAssets =
        root.value(std::string("tool_assets")).isArray();
    if (usesGlobalAssets) {
        for (const oap::JsonValue& value :
             root.value(std::string("tool_assets")).toArray()) {
            if (!value.isObject())
                continue;
            ToolBit tool;
            std::string itemError;
            if (!toolFromJson(value.toObject(), tool, &itemError)) {
                if (error)
                    *error = itemError;
                return false;
            }
            if (parsed.assetById(tool.id)) {
                if (error)
                    *error = std::format("全局刀具资产 ID 重复：{0}", tool.id);
                return false;
            }
            if (!parsed.upsertAsset(std::move(tool), &itemError)) {
                if (error)
                    *error = itemError;
                return false;
            }
        }
        for (const oap::JsonValue& value :
             root.value(std::string("libraries")).toArray()) {
            ToolLibrary library;
            std::string itemError;
            if (!libraryFromReferencesJson(value.toObject(), parsed,
                                           library, &itemError) ||
                !parsed.addLibrary(std::move(library), &itemError)) {
                if (error)
                    *error = itemError;
                return false;
            }
        }
    } else {
        std::vector<ToolLibrary> legacyLibraries;
        for (const oap::JsonValue& value :
             root.value(std::string("libraries")).toArray()) {
            ToolLibrary library;
            std::string itemError;
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
            schema != std::string("oap-tool-library-catalog") || !usesGlobalAssets ||
            version < kCatalogSchemaVersion || identifiersMigrated;
    return true;
}

bool ToolLibraryStorage::loadCatalog(const std::string& filePath,
                                     std::vector<ToolLibrary>& libraries,
                                     std::string* error)
{
    ToolAssetCatalog catalog;
    if (!loadCatalog(filePath, catalog, error)) {
        return false;
    }
    libraries = catalog.libraries();
    return true;
}

} // namespace oap::tooling
