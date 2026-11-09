// 当前刀路工艺参数的桌面端持久化。工件原点、装夹面确认等零件相关状态不在此处保存。
#pragma once

#include <QString>

#include "app/workflows/project/ToolpathParameterEditorState.h"

class QSettings;

namespace oap::app {

// 刀路工艺参数在 QSettings 中的序列化存储;只保存工艺参数,
// 零件相关的原点/装夹面确认等状态不在此存储(见文件头)。
class ToolpathParameterSettingsStorage {
public:
    // 接受外部 QSettings 便于在临时 INI 中做无副作用的回归测试。
    // 事务式读取:整体失败时调用方参数保持原值;
    // 单个字段无效则回退该字段到程序默认并给出警告。
    static bool load(QSettings& settings,
                     oap::toolpath::ToolpathParams& holeParams,
                     oap::toolpath::ToolpathParams& pocketParams,
                     oap::toolpath::ToolpathParams& profileParams,
                     HoleToolSelectionMode& holeToolSelectionMode,
                     PocketToolSelectionMode& pocketToolSelectionMode,
                     ProfileToolSelectionMode& profileToolSelectionMode,
                     bool& coolant,
                     QString* warningOrError = nullptr);
    // 把全部工艺参数写回 QSettings,带 schema 版本号,
    // 保存失败时通过 error 返回原因。
    static bool save(QSettings& settings,
                     const oap::toolpath::ToolpathParams& holeParams,
                     const oap::toolpath::ToolpathParams& pocketParams,
                     const oap::toolpath::ToolpathParams& profileParams,
                     HoleToolSelectionMode holeToolSelectionMode,
                     PocketToolSelectionMode pocketToolSelectionMode,
                     ProfileToolSelectionMode profileToolSelectionMode,
                     bool coolant,
                     QString* error = nullptr);

    // 在用户级默认存储上加载/保存工艺参数(load/save 的便捷封装)。
    static bool loadDefault(oap::toolpath::ToolpathParams& holeParams,
                            oap::toolpath::ToolpathParams& pocketParams,
                            oap::toolpath::ToolpathParams& profileParams,
                            HoleToolSelectionMode& holeToolSelectionMode,
                            PocketToolSelectionMode& pocketToolSelectionMode,
                            ProfileToolSelectionMode& profileToolSelectionMode,
                            bool& coolant,
                            QString* warningOrError = nullptr);
    static bool saveDefault(const oap::toolpath::ToolpathParams& holeParams,
                            const oap::toolpath::ToolpathParams& pocketParams,
                            const oap::toolpath::ToolpathParams& profileParams,
                            HoleToolSelectionMode holeToolSelectionMode,
                            PocketToolSelectionMode pocketToolSelectionMode,
                            ProfileToolSelectionMode profileToolSelectionMode,
                            bool coolant,
                            QString* error = nullptr);
};

} // namespace oap::app
