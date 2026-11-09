// UTF-8 路径与 std::filesystem 原生路径之间的无损转换。
// oap_analysis 内部使用的路径编码工具:跨模块窄字符串路径统一为 UTF-8,与
// std::filesystem 原生路径无损互转(Windows 下避免按活动代码页解码导致乱码)。
#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace oap {

// 跨模块的窄字符串路径统一使用 UTF-8。std::filesystem::path 在 Windows
// 上直接接收 std::string 时会按活动代码页解码，因此必须显式走 char8_t。
inline std::filesystem::path utf8Path(std::string_view value)
{
#if defined(__cpp_char8_t)
    std::u8string encoded;
    encoded.reserve(value.size());
    for (const unsigned char byte : value)
        encoded.push_back(static_cast<char8_t>(byte));
    return std::filesystem::path(encoded);
#else
    return std::filesystem::u8path(value.begin(), value.end());
#endif
}

// 返回 UTF-8，而不是 Windows 活动代码页编码的 path::string()。
inline std::string pathUtf8(const std::filesystem::path& value)
{
    const auto encoded = value.u8string();
    std::string result;
    result.reserve(encoded.size());
    for (const auto byte : encoded)
        result.push_back(static_cast<char>(byte));
    return result;
}

} // namespace oap
