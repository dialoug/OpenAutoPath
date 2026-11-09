// 模型缓存使用的 SHA-256 内容指纹。
#pragma once

#include <cstddef>
#include <string>

namespace oap {

// 对任意字节缓冲区计算 SHA-256，返回小写十六进制。
std::string sha256Hex(const unsigned char* data, size_t len);

// 流式计算 UTF-8 路径文件的 SHA-256，读取失败返回 false。
bool sha256FileHex(const std::string& path, std::string& outHex);

} // namespace oap
