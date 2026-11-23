#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>
#include <vector>

namespace oap {

inline std::string trim(std::string_view text)
{
    constexpr std::string_view whitespace = " \t\r\n\f\v";
    const auto begin = text.find_first_not_of(whitespace);
    return begin == std::string_view::npos
        ? std::string{} : std::string(text.substr(begin, text.find_last_not_of(whitespace) - begin + 1));
}

inline std::string lowerAscii(std::string text)
{
    std::ranges::transform(text, text.begin(), [](unsigned char value) {
        return static_cast<char>(value >= 'A' && value <= 'Z' ? value + ('a' - 'A') : value);
    });
    return text;
}

inline std::string joinLines(const std::vector<std::string>& lines)
{
    std::string result;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (i != 0) result += '\n';
        result += lines[i];
    }
    return result;
}

} // namespace oap
