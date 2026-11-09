// UnitDisplayGrouping 实现（oap_app_workflow_base）。
// 桌面面板共享的单元分组与稳定 id 排序规则，见 UnitDisplayGrouping.h。
#include "UnitDisplayGrouping.h"

#include "analysis/Model.h"

#include <cctype>
#include <limits>

namespace oap::app {
namespace {

constexpr std::string_view kGenericUnitPrefix =
    "BODY_GENERIC";
constexpr std::string_view kNumericGroup =
    "(数字)";
constexpr std::string_view kUnnamedGroup =
    "(未命名)";
constexpr std::string_view kOtherGroup =
    "(其他)";

} // namespace

// 分组键:优先用字母/复合前缀;BODY_GENERIC 需按显示名归类,
// 纯数字(可含 . - _ 空格)归"数字",空名归"未命名",其余归"其他"。
std::string unitDisplayGroupKey(
    const Unit& unit)
{
    if (!unit.name_prefix.empty() &&
        unit.name_prefix !=
            kGenericUnitPrefix) {
        return unit.name_prefix;
    }
    if (unit.decoded_name.empty())
        return std::string(kUnnamedGroup);

    bool hasDigit = false;
    bool containsOnlyDigitsOrSeparators =
        true;
    for (const unsigned char character :
         unit.decoded_name) {
        if (std::isdigit(character)) {
            hasDigit = true;
            continue;
        }
        if (character == '.' ||
            character == '-' ||
            character == '_' ||
            std::isspace(character)) {
            continue;
        }
        containsOnlyDigitsOrSeparators =
            false;
        break;
    }
    return hasDigit &&
                   containsOnlyDigitsOrSeparators
        ? std::string(kNumericGroup)
        : std::string(kOtherGroup);
}

int unitDisplayGroupRank(
    std::string_view groupKey) noexcept
{
    if (groupKey == kNumericGroup)
        return 1;
    if (groupKey == kUnnamedGroup)
        return 2;
    if (groupKey == kOtherGroup)
        return 3;
    return 0;
}

// 跳过前缀字母后解析连续数字;逐位累加前先做溢出检查,
// 超长数字饱和到 uint64 最大值,避免无符号回绕破坏排序。
std::uint64_t stableIdNumber(
    std::string_view id) noexcept
{
    std::size_t index = 0;
    while (index < id.size() &&
           !std::isdigit(
               static_cast<unsigned char>(
                   id[index]))) {
        ++index;
    }

    std::uint64_t value = 0;
    for (; index < id.size() &&
           std::isdigit(
               static_cast<unsigned char>(
                   id[index]));
         ++index) {
        const std::uint64_t digit =
            static_cast<std::uint64_t>(
                id[index] - '0');
        constexpr std::uint64_t maximum =
            std::numeric_limits<
                std::uint64_t>::max();
        if (value >
            (maximum - digit) / 10) {
            return maximum;
        }
        value = value * 10 + digit;
    }
    return value;
}

bool stableIdLess(
    const std::string& first,
    const std::string& second) noexcept
{
    const std::uint64_t firstNumber =
        stableIdNumber(first);
    const std::uint64_t secondNumber =
        stableIdNumber(second);
    if (firstNumber != secondNumber)
        return firstNumber < secondNumber;
    return first < second;
}

} // namespace oap::app
