// ModelIO — oap::Model 的紧凑二进制序列化(缓存用,UI 无关、不依赖 Qt)。
// 格式自带魔数 + 版本号;版本不符则 readModel 返回 false,使旧缓存自动失效。
// 仅序列化模型拓扑(units/faces/edges/wires),不含 TopoDS 几何
// (几何由 BinXCAF 的 .xbf 单独缓存)。读回后由 StepIndexer::restoreMaps 重建
// id<->shape 映射。
#pragma once

#include "Model.h"
#include <string>

namespace oap {

// 缓存二进制格式版本。改动 Model 结构或本文件的写出布局时必须 +1,
// 以令磁盘上的旧缓存在 readModel 阶段被判为不兼容而作废。
// V3 guarantees geometry-based bounds, including models loaded with a mesh.
constexpr unsigned kModelIOFormatVersion = 3;

// path 采用 UTF-8。将 model 写入 path(覆盖)。成功返回 true;打开/写入失败返回 false。
bool writeModel(const std::string& path, const Model& model);

// path 采用 UTF-8。从 path 读取到 out。魔数/版本不符或读取出错返回 false(out 内容未定义,调用方应丢弃)。
bool readModel(const std::string& path, Model& out);

} // namespace oap
