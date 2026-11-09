// 桌面面板共享的单元分组与稳定 id 排序规则。
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace oap {
struct Unit;
}

namespace oap::app {

// 字母/复合前缀直接作为分组；BODY_GENERIC 再按显示名归入
// “数字 / 未命名 / 其他”。
std::string unitDisplayGroupKey(
    const Unit& unit);

// 普通前缀组排在前三个通用组之前。
int unitDisplayGroupRank(
    std::string_view groupKey) noexcept;

// 从 F123/U12 等稳定 id 提取数字后缀；超长数字饱和到 uint64 最大值。
std::uint64_t stableIdNumber(
    std::string_view id) noexcept;

// 先按数字后缀、再按完整字符串排序，保证严格弱序。
bool stableIdLess(
    const std::string& first,
    const std::string& second) noexcept;

} // namespace oap::app
