// 将已验证/已重新生成的项目原子地安装到桌面会话中。
#pragma once

#include <string>
#include <vector>

#include "app/workflows/project/ToolLibraryController.h"
#include "app/workflows/project/MachiningConfigurationSession.h"
#include "app/workflows/project/MachiningProjectController.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::app {

// 安装结果的统计摘要:工序/工位数量、警告列表,以及安装后的
// 毛坯定义与工位确认状态,供 UI 展示安装效果。
struct MachiningProjectInstallResult {
    bool hasHoleDefinitions = false;
    bool hasPocketDefinitions = false;
    bool hasProfileDefinitions = false;
    bool hasFinishingDefinitions = false;
    int holeOperationCount = 0;
    int pocketOperationCount = 0;
    int profileOperationCount = 0;
    int finishingOperationCount = 0;
    int setupCount = 0;
    std::vector<std::string> warnings;
    manufacturing::StockDefinition stockDefinition;
    toolpath::MoldSetupWorkFrames setupWorkFrames;
};

// 原子安装器:把校验/重建完成的 LoadResult 一次性写入
// workspace/configuration/toolLibraries 三个会话对象,不写盘、
// 不带 UI 依赖;调用方在确认成功后才触发持久化。
class MachiningProjectInstaller {
public:
    // 安装入口:迁移 Job 所有权、填充预览参数与识别源、
    // 恢复刀具库,最后返回安装摘要。
    static MachiningProjectInstallResult install(
        MachiningProjectLoadResult loaded,
        MachiningWorkspaceSession& workspace,
        MachiningConfigurationSession& configuration,
        ToolLibraryController& toolLibraries,
        const toolpath::MoldSetupWorkFrames& currentSetupFrames);
};

} // namespace oap::app
