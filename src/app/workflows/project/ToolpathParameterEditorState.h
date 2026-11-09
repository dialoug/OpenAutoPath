// 加工配置与桌面编辑器之间交换的值快照。
#pragma once

#include <string>
#include <vector>

#include "app/workflows/engraving/EngravingSettings.h"

#include "toolpath/core/MachiningSetup.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::app {

// 孔、槽腔和外轮廓共享的刀具来源：活动刀库自动选刀或直接填写单刀。
enum class ToolSelectionMode {
    AutomaticLibrary = 1,
    SingleTool = 2, // 0 remains the removed historical manual-planning mode.
};

// Family aliases keep call sites expressive while enforcing one value model.
using HoleToolSelectionMode = ToolSelectionMode;
using PocketToolSelectionMode = ToolSelectionMode;
using ProfileToolSelectionMode = ToolSelectionMode;

struct EngravingLibraryChoice {
    std::string id;
    std::string name;
};

// 编辑器与配置会话之间的值快照:孔/槽腔/外轮廓参数、工装设定、
// 工位坐标系与 G 代码程序选项整体打包,便于在 UI 中编辑后一次性提交。
struct ToolpathParameterEditorState {
    oap::toolpath::ToolpathParams holeParams;
    oap::toolpath::ToolpathParams pocketParams;
    oap::toolpath::ToolpathParams profileParams;
    toolpath::ToolpathToolContext holeSingleTool;
    toolpath::ToolpathToolContext pocketSingleTool;
    toolpath::ToolpathToolContext profileSingleTool;
    EngravingSettings engraving;
    std::vector<EngravingLibraryChoice> engravingLibraries;
    std::string activeToolLibraryId;
    HoleToolSelectionMode holeToolSelectionMode =
        HoleToolSelectionMode::AutomaticLibrary;
    PocketToolSelectionMode pocketToolSelectionMode =
        PocketToolSelectionMode::AutomaticLibrary;
    ProfileToolSelectionMode profileToolSelectionMode =
        ProfileToolSelectionMode::AutomaticLibrary;
    oap::toolpath::MachiningSetup machiningSetup;
    oap::toolpath::MoldSetupWorkFrames moldSetupWorkFrames;
    oap::toolpath::GCodeProgramOptions gcodeProgramOptions;
    double moldLength = 0.0;
    double moldWidth = 0.0;
};

} // namespace oap::app
