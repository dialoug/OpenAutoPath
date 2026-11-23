#include <random>
#include <string_view>
#include "support/Identity.h"

#include <array>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <locale>
#include <sstream>

namespace oap {
namespace {

template<typename Buffer>
std::string hex(const Buffer& bytes)
{
    constexpr std::string_view digits = "0123456789abcdef";
    std::string result;
    result.reserve(bytes.size() * 2);
    for (const auto byte : bytes) {
        result += digits[byte >> 4];
        result += digits[byte & 15];
    }
    return result;
}

} // namespace

std::string randomUuid()
{
    std::array<unsigned char, 16> bytes;
    std::random_device random;
    for (auto& byte : bytes) byte = static_cast<unsigned char>(random());
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    auto result = hex(bytes);
    for (const auto index : {20, 16, 12, 8}) result.insert(index, 1, '-');
    return result;
}

std::string utcTimestamp(bool milliseconds)
{
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::floor<std::chrono::seconds>(now);
    const auto time = std::chrono::system_clock::to_time_t(seconds);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &time);
#else
    gmtime_r(&time, &utc);
#endif
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S");
    if (milliseconds)
        text << '.' << std::setfill('0') << std::setw(3)
             << std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
    text << 'Z';
    return text.str();
}

} // namespace oap
