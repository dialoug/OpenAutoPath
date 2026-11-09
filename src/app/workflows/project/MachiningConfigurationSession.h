// 桌面加工参数与程序级配置。
#pragma once

#include <QString>

#include "app/workflows/project/ToolpathParameterEditorState.h"

namespace oap::app {

// apply() 的变更摘要:逐项指出哪些配置发生变化,供调用方决定
// 是否重建刀路/清空刀具身份;persistenceError 携带本地持久化失败信息。
struct MachiningConfigurationApplyResult {
    bool holeParametersChanged = false;
    bool pocketParametersChanged = false;
    bool profileParametersChanged = false;
    bool engravingParametersChanged = false;
    bool holeToolSelectionModeChanged = false;
    bool pocketToolSelectionModeChanged = false;
    bool profileToolSelectionModeChanged = false;
    bool toolIdentityCleared = false;
    QString persistenceError;
    toolpath::MoldSetupWorkFrames setupWorkFrames;
};

// 会话级加工配置:持有孔/型腔刀路参数、模具装夹设定与 G 代码程序选项。
// 由 MachiningProjectController 拥有,生命周期与工程会话一致;
// 公开访问器直接暴露内部可变引用,apply() 是唯一的整体写入入口。
class MachiningConfigurationSession {
public:
    // 当持久化值无法被完整恢复时返回一条警告。
    QString loadDefaults();

    // 生成编辑器的值快照:把当前内部配置与传入的工位坐标系/模具尺寸
    // 打包为 ToolpathParameterEditorState,供 UI 编辑后原样交回 apply()。
    ToolpathParameterEditorState editorState(
        const toolpath::MoldSetupWorkFrames& workFrames,
        double moldLength,
        double moldWidth) const;
    // 应用编辑后的参数:先比较变更、再接管(move)编辑值,
    // 若刀具身份发生变更则清空 G 代码中的 toolId/toolName,
    // 最后把默认配置持久化到本地存储并返回变更摘要。
    MachiningConfigurationApplyResult apply(
        ToolpathParameterEditorState edited);

    // 逐字段比较两组刀路参数(含刀具几何、深度、清角、孔策略),
    // 仅用于判断"是否发生实质变更",不等价于内存级比较。
    static bool parametersEqual(
        const toolpath::ToolpathParams& first,
        const toolpath::ToolpathParams& second) noexcept;

    toolpath::ToolpathParams& holeParams() noexcept { return m_holeParams; }
    EngravingSettings& engravingSettings() noexcept { return m_engraving; }
    const EngravingSettings& engravingSettings() const noexcept { return m_engraving; }
    const toolpath::ToolpathParams& holeParams() const noexcept
    {
        return m_holeParams;
    }
    toolpath::ToolpathParams& pocketParams() noexcept
    {
        return m_pocketParams;
    }
    const toolpath::ToolpathParams& pocketParams() const noexcept
    {
        return m_pocketParams;
    }
    toolpath::ToolpathParams& profileParams() noexcept
    {
        return m_profileParams;
    }
    const toolpath::ToolpathParams& profileParams() const noexcept
    {
        return m_profileParams;
    }
    HoleToolSelectionMode holeToolSelectionMode() const noexcept
    {
        return m_holeToolSelectionMode;
    }
    PocketToolSelectionMode pocketToolSelectionMode() const noexcept
    {
        return m_pocketToolSelectionMode;
    }
    ProfileToolSelectionMode profileToolSelectionMode() const noexcept
    {
        return m_profileToolSelectionMode;
    }
    toolpath::ToolpathToolContext& holeSingleTool() noexcept { return m_holeSingleTool; }
    const toolpath::ToolpathToolContext& holeSingleTool() const noexcept { return m_holeSingleTool; }
    void setHoleToolSelectionMode(ToolSelectionMode value) noexcept { m_holeToolSelectionMode = value; }
    toolpath::ToolpathToolContext& pocketSingleTool() noexcept { return m_pocketSingleTool; }
    const toolpath::ToolpathToolContext& pocketSingleTool() const noexcept { return m_pocketSingleTool; }
    void setPocketToolSelectionMode(ToolSelectionMode value) noexcept { m_pocketToolSelectionMode = value; }
    toolpath::ToolpathToolContext& profileSingleTool() noexcept { return m_profileSingleTool; }
    const toolpath::ToolpathToolContext& profileSingleTool() const noexcept { return m_profileSingleTool; }
    void setProfileToolSelectionMode(ToolSelectionMode value) noexcept { m_profileToolSelectionMode = value; }
    toolpath::MachiningSetup& machiningSetup() noexcept
    {
        return m_machiningSetup;
    }
    const toolpath::MachiningSetup& machiningSetup() const noexcept
    {
        return m_machiningSetup;
    }
    toolpath::GCodeProgramOptions& gcodeOptions() noexcept
    {
        return m_gcodeOptions;
    }
    const toolpath::GCodeProgramOptions& gcodeOptions() const noexcept
    {
        return m_gcodeOptions;
    }

private:
    // 判断刀具切削特征(直径、圆角、补偿、转速、进给)是否变化,
    // 用于决定是否清除 G 代码中的刀具身份标识。
    static bool toolChanged(
        const toolpath::ToolDef& first,
        const toolpath::ToolDef& second) noexcept;

    toolpath::ToolpathParams m_holeParams;
    toolpath::ToolpathParams m_pocketParams;
    toolpath::ToolpathParams m_profileParams;
    EngravingSettings m_engraving;
    toolpath::ToolpathToolContext m_holeSingleTool;
    toolpath::ToolpathToolContext m_pocketSingleTool;
    toolpath::ToolpathToolContext m_profileSingleTool;
    HoleToolSelectionMode m_holeToolSelectionMode =
        HoleToolSelectionMode::AutomaticLibrary;
    PocketToolSelectionMode m_pocketToolSelectionMode =
        PocketToolSelectionMode::AutomaticLibrary;
    ProfileToolSelectionMode m_profileToolSelectionMode =
        ProfileToolSelectionMode::AutomaticLibrary;
    toolpath::MachiningSetup m_machiningSetup;
    toolpath::GCodeProgramOptions m_gcodeOptions;
};

} // namespace oap::app
