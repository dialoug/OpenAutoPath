// 孔刀路命令策略（oap_app_workflow_cam）。
// 依据「是否已安装程序 / 是否已识别来源」解析孔刀路命令（重建已安装程序
// 或从识别重新生成），并给出孔/槽/腔刀路生成可用性判断。
#pragma once

namespace oap::app {

enum class HoleToolpathCommandKind {
    GenerateFromRecognition,
    RebuildInstalledProgram,
};

constexpr HoleToolpathCommandKind resolveHoleToolpathCommand(
    bool hasInstalledProgram,
    bool hasRecognizedSources) noexcept
{
    return hasInstalledProgram && hasRecognizedSources
        ? HoleToolpathCommandKind::RebuildInstalledProgram
        : HoleToolpathCommandKind::GenerateFromRecognition;
}

constexpr bool toolpathGenerationAvailable(
    bool recognitionReady,
    bool activeLibraryHasAssignedTools,
    bool busy) noexcept
{
    return recognitionReady &&
        activeLibraryHasAssignedTools && !busy;
}

constexpr bool holeToolpathGenerationAvailable(
    bool recognitionReady,
    bool activeLibraryHasAssignedTools,
    bool busy) noexcept
{
    return toolpathGenerationAvailable(
        recognitionReady,
        activeLibraryHasAssignedTools, busy);
}

constexpr bool pocketToolpathGenerationAvailable(
    bool recognitionReady,
    bool activeLibraryHasAssignedTools,
    bool busy) noexcept
{
    return toolpathGenerationAvailable(
        recognitionReady,
        activeLibraryHasAssignedTools, busy);
}

} // namespace oap::app
