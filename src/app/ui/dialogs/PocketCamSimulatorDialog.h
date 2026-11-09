// 独立加工仿真窗口：Vulkan 网格帧与 OpenGL GPU Multi-Dexel 双显示后端。
#pragma once

#include <QString>

#include "app/workflows/pocket/PocketCamSimulatorTypes.h"

class QWidget;

namespace oap::app {

// 仅当场景无法准备或对话框无法启动时才返回 false。
// Vulkan 初始化/渲染失败会重建 OpenGL 窗口；后续诊断显示在模拟器内部。
bool showPocketCamSimulatorDialog(
    QWidget* parent,
    PocketCamSimulatorInput input,
    QString* errorMessage = nullptr);

} // namespace oap::app
