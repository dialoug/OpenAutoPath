// 体名称解析实现 — 参见 Naming.h 与 docs/08_name_rule_table.md。
#include "Naming.h"

#include <algorithm>
#include <cctype>
#include <cstdint>

namespace oap {
namespace {

// 未知/默认 family 的置信度（doc 08 §4）。
constexpr double kDefaultConfidence = 0.20;
// 十六进制字母相对 'A'/'a' 的偏移基数。
constexpr int kHexLetterBase = 10;

// 将一个 Unicode 码点追加到 UTF-8 字符串。
void appendUtf8(std::string& out, uint32_t cp) {
    if (cp <= 0x7F) {
        out.push_back(char(cp));
    } else if (cp <= 0x7FF) {
        out.push_back(char(0xC0 | (cp >> 6)));
        out.push_back(char(0x80 | (cp & 0x3F)));
    } else if (cp <= 0xFFFF) {
        out.push_back(char(0xE0 | (cp >> 12)));
        out.push_back(char(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(char(0x80 | (cp & 0x3F)));
    } else {
        out.push_back(char(0xF0 | (cp >> 18)));
        out.push_back(char(0x80 | ((cp >> 12) & 0x3F)));
        out.push_back(char(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(char(0x80 | (cp & 0x3F)));
    }
}

// 十六进制字符 -> 数值;非十六进制字符返回 -1。
int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + kHexLetterBase;
    if (c >= 'a' && c <= 'f') return c - 'a' + kHexLetterBase;
    return -1;
}

// 已知的复合前缀(在通用字母串规则之前检查)— doc 08 §3。
struct PrefixRule { const char* token; };
const PrefixRule kCompoundPrefixes[] = {
    {"MI_MSW"}, {"MI_LP"}, {"MI_WR"}, {"MI_MYKP"}, {"MI_GSH"}, {"MI_SWF"},
};

// name_prefix 到 {name_family, confidence} 的映射 —— 文档 08 §4。
struct FamilyRule { const char* prefix; const char* family; double confidence; };
const FamilyRule kFamilyTable[] = {
    {"UP", "upper_plate_family", 0.90},
    {"UB", "upper_block_family", 0.80},
    {"LP", "lower_plate_family", 0.90},
    {"LB", "lower_block_family", 0.80},
    {"DIE", "die_insert_family", 0.75},
    {"PH", "punch_holder_family", 0.70},
    {"PU", "punch_family", 0.70},
    {"PS", "plate_or_pad_family", 0.55},
    {"U2", "upper_aux_family", 0.55},
    {"B2", "block_aux_family", 0.55},
    {"MI_LP", "insert_plate_family", 0.70},
    {"MI_MSW", "insert_or_screw_family", 0.60},
    {"MI_WR", "insert_wear_family", 0.60},
    {"MI_MYKP", "insert_key_plate_family", 0.60},
    {"MI_GSH", "guide_or_holder_family", 0.55},
    {"MI_SWF", "insert_fastener_family", 0.55},
    {"\xE8\x9E\xBA\xE4\xB8\x9D", "fastener_family", 0.90},        // 螺丝
    {"\xE5\xB7\xB2\xE6\x8B\xBC\xE6\x96\x99", "scrap_or_aux_family", 0.70}, // 已拼料
    {"BODY_GENERIC", "unknown_family", 0.20},
};

// name_family 到 unit_role_hint 的映射 —— 文档 08 §5。
struct RoleRule { const char* family; const char* role; };
const RoleRule kRoleTable[] = {
    {"upper_plate_family", "plate_layer_candidate"},
    {"lower_plate_family", "plate_layer_candidate"},
    {"upper_block_family", "block_layer_candidate"},
    {"lower_block_family", "block_layer_candidate"},
    {"die_insert_family", "insert_candidate"},
    {"punch_holder_family", "insert_candidate"},
    {"punch_family", "insert_candidate"},
    {"plate_or_pad_family", "plate_layer_candidate"},
    {"upper_aux_family", "support_column_candidate"},
    {"block_aux_family", "block_like_candidate"},
    {"insert_plate_family", "insert_candidate"},
    {"insert_or_screw_family", "insert_candidate"},
    {"insert_wear_family", "insert_candidate"},
    {"insert_key_plate_family", "insert_candidate"},
    {"guide_or_holder_family", "support_column_candidate"},
    {"insert_fastener_family", "fastener_candidate"},
    {"fastener_family", "fastener_candidate"},
    {"scrap_or_aux_family", "scrap_or_aux_candidate"},
    {"unknown_family", "unknown_role"},
};

} // namespace


std::string decodeStepName(const std::string& raw) {
    std::string out;
    out.reserve(raw.size());
    size_t i = 0;
    const size_t n = raw.size();
    while (i < n) {
        // STEP \X2\ ... \X0\ 块:每个 UTF-16 码元 4 个十六进制位。
        if (i + 3 < n && raw[i] == '\\' && (raw[i + 1] == 'X' || raw[i + 1] == 'x') &&
            raw[i + 2] == '2' && raw[i + 3] == '\\') {
            i += 4;
            while (i + 3 < n) {
                int h0 = hexVal(raw[i]), h1 = hexVal(raw[i + 1]),
                    h2 = hexVal(raw[i + 2]), h3 = hexVal(raw[i + 3]);
                if (h0 < 0 || h1 < 0 || h2 < 0 || h3 < 0) break;
                uint32_t unit = (h0 << 12) | (h1 << 8) | (h2 << 4) | h3;
                i += 4;
                // 若下一个码元是低位代理项,则处理 UTF-16 代理对。
                if (unit >= 0xD800 && unit <= 0xDBFF && i + 3 < n) {
                    int l0 = hexVal(raw[i]), l1 = hexVal(raw[i + 1]),
                        l2 = hexVal(raw[i + 2]), l3 = hexVal(raw[i + 3]);
                    if (l0 >= 0 && l1 >= 0 && l2 >= 0 && l3 >= 0) {
                        uint32_t lo = (l0 << 12) | (l1 << 8) | (l2 << 4) | l3;
                        if (lo >= 0xDC00 && lo <= 0xDFFF) {
                            uint32_t cp = 0x10000 + ((unit - 0xD800) << 10) + (lo - 0xDC00);
                            appendUtf8(out, cp);
                            i += 4;
                            continue;
                        }
                    }
                }
                appendUtf8(out, unit);
            }
            // 跳过结尾的 \X0\(若存在)。
            if (i + 3 < n && raw[i] == '\\' && (raw[i + 1] == 'X' || raw[i + 1] == 'x') &&
                raw[i + 2] == '0' && raw[i + 3] == '\\')
                i += 4;
            continue;
        }
        // \X\<2-hex> 单字节转义。
        if (i + 4 < n && raw[i] == '\\' && (raw[i + 1] == 'X' || raw[i + 1] == 'x') &&
            raw[i + 2] == '\\') {
            int h0 = hexVal(raw[i + 3]), h1 = hexVal(raw[i + 4]);
            if (h0 >= 0 && h1 >= 0) {
                appendUtf8(out, uint32_t((h0 << 4) | h1));
                i += 5;
                continue;
            }
        }
        out.push_back(raw[i++]);
    }
    // 去除首尾空白。
    size_t b = out.find_first_not_of(" \t\r\n");
    size_t e = out.find_last_not_of(" \t\r\n");
    if (b == std::string::npos) return std::string();
    return out.substr(b, e - b + 1);
}

namespace {

// 按 doc 08 §3 从解码后的名称中提取前缀:
//  - 优先匹配复合 MI_* 标记
//  - "Body (...)" / "体 ..." -> BODY_GENERIC
//  - 否则取 '-' 或首个数字之前的起始字母串(例如 "DIE2-06" -> "DIE")
std::string extractPrefix(const std::string& name) {
    if (name.empty()) return "BODY_GENERIC";

    for (const auto& cp : kCompoundPrefixes) {
        const std::string tok = cp.token;
        if (name.compare(0, tok.size(), tok) == 0) return tok;
    }

    // 通用 "Body" 与中文 体 归并为 BODY_GENERIC。
    if (name.starts_with("Body") || name.starts_with("body"))
        return "BODY_GENERIC";
    if (name.starts_with("\xE4\xBD\x93")) // 体
        return "BODY_GENERIC";

    // 起始的 ASCII 字母串,遇到首个数字、'-' 或 '_' 即停止。
    std::string p;
    for (char c : name) {
        if (std::isalpha((unsigned char)c)) p.push_back(char(std::toupper((unsigned char)c)));
        else break;
    }
    if (p.empty()) return "BODY_GENERIC";
    return p;
}

} // namespace

NameInfo parseUnitName(const std::string& raw) {
    NameInfo info;
    info.decoded_name = decodeStepName(raw);
    info.name_prefix = extractPrefix(info.decoded_name);

    // 映射 prefix -> family(默认 unknown_family)。
    info.name_family = "unknown_family";
    info.confidence = 0.20;
    for (const auto& f : kFamilyTable) {
        if (info.name_prefix == f.prefix) {
            info.name_family = f.family;
            info.confidence = f.confidence;
            break;
        }
    }

    // 映射 family -> role_hint(默认 unknown_role)。
    info.unit_role_hint = "unknown_role";
    for (const auto& r : kRoleTable) {
        if (info.name_family == r.family) {
            info.unit_role_hint = r.role;
            break;
        }
    }
    return info;
}

} // namespace oap
