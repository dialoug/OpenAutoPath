// 加工前毛坯模型加载器。只负责把 STEP 根形状转换为一个可供在制毛坯
// 状态使用的 TopoDS_Shape；正体积和拓扑有效性由 InProcessStock 统一验证。
#pragma once

#include <string>

#include <TopoDS_Shape.hxx>

#include "toolpath/core/ToolpathExecution.h"

namespace oap::toolpath {

enum class StockModelLoadStatus {
    Ok,
    Cancelled,
    EmptyPath,
    ReadFailed,
    TransferFailed,
    EmptyShape,
};

// 毛坯模型加载结果：状态、形状与消息。
struct StockModelLoadResult {
    StockModelLoadStatus status = StockModelLoadStatus::ReadFailed;
    TopoDS_Shape shape;
    std::string message;

    bool ok() const noexcept { return status == StockModelLoadStatus::Ok; }
};

// 毛坯模型加载器：从 STEP 路径加载毛坯形状。
class StockModelLoader {
public:
    // sourcePath 采用 UTF-8；OCCT 在 Windows 内部将其转换为 UTF-16。
    // 全程不访问 XCAF 文档或查看器状态，可安全放入后台任务；
    // 可通过 control 查询取消。
    StockModelLoadResult loadStep(
        const std::string& sourcePath,
        const ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::toolpath
