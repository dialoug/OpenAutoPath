// 无外部依赖的 JSON 写出工具，供刀具库与加工 Job 等持久化使用。
// 未知数值输出为 null，未知数组为 []，未知对象为 {}；不支持解析。
#pragma once

#include <array>
#include <string>
#include <vector>

namespace oap {

// 转义 UTF-8 字符串以便嵌入 JSON 字符串字面量。字节原样透传(有效的 UTF-8
// 仍然有效);仅转义 JSON 语义相关的控制字符。
std::string jsonEscape(const std::string& s);

// 按插入顺序输出 "key":value 键值对的轻量对象构造器。
// 每个链式调用向内部缓冲区追加一段，末尾由 str() 包上花括号；
// 临时对象即可完成一条记录的拼装，无需管理所有权。
class JsonObject {
public:
    JsonObject& str(const std::string& key, const std::string& v);
    JsonObject& strOrNull(const std::string& key, const std::string& v); // 空字符串 -> null
    JsonObject& num(const std::string& key, double v);                          // 数值字段
    JsonObject& numOrNull(const std::string& key, bool has, double v);          // has=false 时输出 null
    JsonObject& integer(const std::string& key, long long v);                   // 整数字段
    JsonObject& boolean(const std::string& key, bool v);                        // 布尔字段
    JsonObject& raw(const std::string& key, const std::string& jsonLiteral); // 预先格式化的值
    JsonObject& strArray(const std::string& key, const std::vector<std::string>& v); // 字符串数组
    JsonObject& numArray(const std::string& key, const std::vector<double>& v);      // 数值数组

    // 渲染为单行 JSON 对象字符串。
    std::string str() const;

private:
    void sep();
    std::string m_buf;
    bool m_first = true;
};

// 将 bbox/point 数组转为 JSON 字面量 "[...]" 的辅助函数。
std::string jsonArray(const std::array<double, 3>& v);
std::string jsonArray(const std::vector<double>& v);

} // namespace oap
