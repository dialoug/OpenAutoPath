// ContentHash 实现：自包含的 SHA-256 内容指纹。
#include "ContentHash.h"
#include "Utf8Path.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>

namespace oap {
namespace {

inline uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }

const uint32_t K[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};

// 自包含的 SHA-256 实现(不依赖外部加密库):支持流式 update(),finish() 输出
// 64 位小写十六进制摘要。内部维护 64 字节块缓冲与 8 个状态寄存器,
// 供 sha256Hex / sha256FileHex 复用。
class Sha256 {
public:
    // 追加输入数据:先填满缓冲再整块 transform(),不足一块的余量存入缓冲。
    // 溢出检测(累计字节数回绕或非法指针)置 m_overflow,finish() 随即返回空串。
    void update(const unsigned char* data, size_t len)
    {
        if (m_overflow || (!data && len != 0) ||
            len > std::numeric_limits<uint64_t>::max() - m_totalBytes) {
            m_overflow = true;
            return;
        }
        m_totalBytes += static_cast<uint64_t>(len);

        if (m_bufferSize != 0) {
            const size_t copied = std::min(len, m_buffer.size() - m_bufferSize);
            std::copy_n(data, copied, m_buffer.data() + m_bufferSize);
            m_bufferSize += copied;
            data += copied;
            len -= copied;
            if (m_bufferSize == m_buffer.size()) {
                transform(m_buffer.data());
                m_bufferSize = 0;
            }
        }

        while (len >= m_buffer.size()) {
            transform(data);
            data += m_buffer.size();
            len -= m_buffer.size();
        }
        if (len != 0) {
            std::copy_n(data, len, m_buffer.data());
            m_bufferSize = len;
        }
    }

    // 结束输入:按 SHA-256 规范补 0x80 与 64 位比特长度,做最后一次 transform(),
    // 再以 "%08x" 逐字输出状态寄存器为小写十六进制。发生溢出时返回空串。
    std::string finish()
    {
        if (m_overflow || m_totalBytes > std::numeric_limits<uint64_t>::max() / 8)
            return {};
        const uint64_t bitLength = m_totalBytes * 8;
        m_buffer[m_bufferSize++] = 0x80;
        if (m_bufferSize > 56) {
            std::fill(m_buffer.begin() + static_cast<std::ptrdiff_t>(m_bufferSize),
                      m_buffer.end(), 0);
            transform(m_buffer.data());
            m_bufferSize = 0;
        }
        std::fill(m_buffer.begin() + static_cast<std::ptrdiff_t>(m_bufferSize),
                  m_buffer.begin() + 56, 0);
        for (int index = 0; index < 8; ++index)
            m_buffer[56 + index] = static_cast<unsigned char>(
                bitLength >> (56 - index * 8));
        transform(m_buffer.data());

        char out[65]{};
        for (size_t index = 0; index < m_state.size(); ++index)
            std::snprintf(out + index * 8, 9, "%08x", m_state[index]);
        return std::string(out, 64);
    }

private:
    // 处理一个 64 字节块:先由 16 个字扩展出 64 个字(消息调度),再执行 64 轮
    // 压缩(Σ / Ch / Maj 轮函数),最后把轮结果累加进 m_state。
    void transform(const unsigned char* block)
    {
        uint32_t words[64]{};
        for (int index = 0; index < 16; ++index) {
            words[index] =
                (static_cast<uint32_t>(block[index * 4]) << 24) |
                (static_cast<uint32_t>(block[index * 4 + 1]) << 16) |
                (static_cast<uint32_t>(block[index * 4 + 2]) << 8) |
                static_cast<uint32_t>(block[index * 4 + 3]);
        }
        for (int index = 16; index < 64; ++index) {
            const uint32_t s0 = rotr(words[index - 15], 7) ^
                                rotr(words[index - 15], 18) ^
                                (words[index - 15] >> 3);
            const uint32_t s1 = rotr(words[index - 2], 17) ^
                                rotr(words[index - 2], 19) ^
                                (words[index - 2] >> 10);
            words[index] = words[index - 16] + s0 + words[index - 7] + s1;
        }

        uint32_t a = m_state[0], b = m_state[1], c = m_state[2], d = m_state[3];
        uint32_t e = m_state[4], f = m_state[5], g = m_state[6], h = m_state[7];
        for (int index = 0; index < 64; ++index) {
            const uint32_t sum1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const uint32_t choose = (e & f) ^ (~e & g);
            const uint32_t temp1 = h + sum1 + choose + K[index] + words[index];
            const uint32_t sum0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t temp2 = sum0 + majority;
            h = g; g = f; f = e; e = d + temp1;
            d = c; c = b; b = a; a = temp1 + temp2;
        }
        m_state[0] += a; m_state[1] += b; m_state[2] += c; m_state[3] += d;
        m_state[4] += e; m_state[5] += f; m_state[6] += g; m_state[7] += h;
    }

    std::array<uint32_t, 8> m_state{
        0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
        0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    std::array<unsigned char, 64> m_buffer{};
    size_t m_bufferSize = 0;
    uint64_t m_totalBytes = 0;
    bool m_overflow = false;
};

} // namespace

std::string sha256Hex(const unsigned char* data, size_t len) {
    Sha256 hash;
    hash.update(data, len);
    return hash.finish();
}

bool sha256FileHex(const std::string& path, std::string& outHex)
{
    std::ifstream in(utf8Path(path), std::ios::binary);
    if (!in) return false;
    Sha256 hash;
    // Windows GUI 线程的默认栈通常只有 1 MiB，块缓冲保持在 64 KiB，避免
    // 仅为计算指纹就在栈上触发 guard page/stack overflow。
    std::array<unsigned char, 64u * 1024u> bytes{};
    while (in) {
        in.read(reinterpret_cast<char*>(bytes.data()),
                static_cast<std::streamsize>(bytes.size()));
        const std::streamsize count = in.gcount();
        if (count > 0)
            hash.update(bytes.data(), static_cast<size_t>(count));
    }
    if (in.bad()) return false;
    outHex = hash.finish();
    return !outHex.empty();
}

} // namespace oap
