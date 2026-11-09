// 极简 JSON 序列化器实现 — 参见 Json.h。
// 不依赖外部 JSON 库；所有值在渲染时直接序列化为字符串，不保留中间解析结果。
#include "Json.h"

#include <cmath>
#include <cstdio>

namespace oap {

namespace {
// 最低可打印 ASCII：低于此码位的字符需转义为 \u00XX。
constexpr unsigned char kFirstPrintableAscii = 0x20;
// 数值格式化缓冲区大小。
constexpr int kNumberBufSize = 32;
// JSON 数值输出的有效位数。
constexpr const char* kNumberFormat = "%.6g";
} // namespace

std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < kFirstPrintableAscii) { // 其他控制字符 -> \u00XX
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out.push_back((char)c); // 有效的 UTF-8 字节原样透传
                }
        }
    }
    return out;
}

// 有限数值渲染为 JSON;非有限值(nan/inf)渲染为 null 以保证输出有效。
static std::string numLit(double v) {
    if (!std::isfinite(v)) return "null";
    char buf[kNumberBufSize];
    std::snprintf(buf, sizeof(buf), kNumberFormat, v);
    return std::string(buf);
}

std::string jsonArray(const std::array<double, 3>& v) {
    return "[" + numLit(v[0]) + "," + numLit(v[1]) + "," + numLit(v[2]) + "]";
}

std::string jsonArray(const std::vector<double>& v) {
    std::string out = "[";
    for (size_t i = 0; i < v.size(); ++i) { if (i) out += ","; out += numLit(v[i]); }
    return out + "]";
}

void JsonObject::sep() {
    if (!m_first) m_buf += ",";
    m_first = false;
}

JsonObject& JsonObject::str(const std::string& key, const std::string& v) {
    sep(); m_buf += "\""; m_buf += key; m_buf += "\":\""; m_buf += jsonEscape(v); m_buf += "\""; return *this;
}

JsonObject& JsonObject::strOrNull(const std::string& key, const std::string& v) {
    sep();
    if (v.empty()) { m_buf += "\""; m_buf += key; m_buf += "\":null"; }
    else { m_buf += "\""; m_buf += key; m_buf += "\":\""; m_buf += jsonEscape(v); m_buf += "\""; }
    return *this;
}

JsonObject& JsonObject::num(const std::string& key, double v) {
    sep(); m_buf += "\""; m_buf += key; m_buf += "\":"; m_buf += numLit(v); return *this;
}

JsonObject& JsonObject::numOrNull(const std::string& key, bool has, double v) {
    sep(); m_buf += "\""; m_buf += key; m_buf += "\":"; m_buf += (has ? numLit(v) : "null"); return *this;
}

JsonObject& JsonObject::integer(const std::string& key, long long v) {
    sep(); m_buf += "\""; m_buf += key; m_buf += "\":"; m_buf += std::to_string(v); return *this;
}

JsonObject& JsonObject::boolean(const std::string& key, bool v) {
    sep(); m_buf += "\""; m_buf += key; m_buf += "\":"; m_buf += (v ? "true" : "false"); return *this;
}

JsonObject& JsonObject::raw(const std::string& key, const std::string& jsonLiteral) {
    sep(); m_buf += "\""; m_buf += key; m_buf += "\":"; m_buf += jsonLiteral; return *this;
}

JsonObject& JsonObject::strArray(const std::string& key, const std::vector<std::string>& v) {
    sep(); m_buf += "\""; m_buf += key; m_buf += "\":[";
    for (size_t i = 0; i < v.size(); ++i) { if (i) m_buf += ","; m_buf += "\""; m_buf += jsonEscape(v[i]); m_buf += "\""; }
    m_buf += "]"; return *this;
}

JsonObject& JsonObject::numArray(const std::string& key, const std::vector<double>& v) {
    sep(); m_buf += "\""; m_buf += key; m_buf += "\":"; m_buf += jsonArray(v); return *this;
}

std::string JsonObject::str() const {
    return "{" + m_buf + "}";
}

} // namespace oap
