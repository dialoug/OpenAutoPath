// 体名称解析 — STEP Unicode 解码 + prefix/family/role_hint 映射 (docs/08_name_rule_table.md)。
// 名称规则仅供参考:它们填充提示,几何仅做校验。冲突会被记录,而非覆盖。
// 属于 oap_analysis 的模型 IO 链路:解析结果为纯值,不持有外部资源。
#pragma once

#include <string>

namespace oap {

// 单个实体名称的解析结果;confidence 为 family 映射的置信度(doc 08 §4)。
struct NameInfo {
    std::string decoded_name;     // 经 \X2\..\X0\ 解码 + 去空白后
    std::string name_prefix;      // 例如 "UP"、"MI_LP"、"BODY_GENERIC"
    std::string name_family;      // 例如 "upper_plate_family"
    std::string unit_role_hint;   // 例如 "plate_layer_candidate"
    double confidence = 0.20;     // 来自 doc 08 §4 的 family 置信度
};

// 将 STEP \X2\<hex...>\X0\ 与 \X\<hex>\ 转义解码为 UTF-8,然后去除首尾空白。
std::string decodeStepName(const std::string& raw);

// 完整流水线:解码 -> prefix -> family -> role_hint (doc 08 §2)。raw 可为空。
NameInfo parseUnitName(const std::string& raw);

} // namespace oap
