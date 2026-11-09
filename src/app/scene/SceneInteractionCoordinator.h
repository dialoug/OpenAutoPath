// OCCT 拾取、与面板无关的选择及可见性命令的应用边界。
// 职责:把 OCCT 拾取到的形状映射为分析实体(面/边/线),维护与面板
//   无关的全局选择状态,并执行可见性命令与撤销事务。
// 模块归属:src/app(scene 聚焦工作区协调器),由 MainWindow 持有。
// 协作:SelectionCoordinator 管理选择源与快照;SceneController 管理
//   场景呈现与可见性;CommandHistory 记录可见性命令以供撤销。
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <AIS_InteractiveContext.hxx>
#include <TopoDS_Shape.hxx>

#include "app/presentation/SceneController.h"
#include "app/workflows/shared/SelectionCoordinator.h"
#include "app/workflows/shared/UndoHistory.h"

namespace oap {
class ModelIndex;
class StepIndexer;
}

namespace oap::app {

// 拾取动作:None 无动作、SelectUnit 选择单元、SelectFace 单选面、
// ToggleFace 增选模式下切换面、SelectFaces 整特征选择、
// ShowEdge/ShowWire 显示边/线信息。
enum class ScenePickAction {
    None,
    SelectUnit,
    SelectFace,
    ToggleFace,
    SelectFaces,
    ShowEdge,
    ShowWire,
};

// 单点拾取结果:动作、实体/单元 id、子类型、相关面列表、邻接面数、
// 是否命中特征与是否请求孔阶段高亮。
struct ScenePickResult {
    ScenePickAction action = ScenePickAction::None;
    std::string entityId;
    std::string unitId;
    std::string entitySubtype;
    std::vector<std::string> faceIds;
    int relatedFaceCount = 0;
    bool matchedFeature = false;
    bool requestHoleStageHighlight = false;

    bool accepted() const noexcept
    {
        return action != ScenePickAction::None;
    }
};

// 框选结果:命中的面/单元 id 集合、实体数、形状类型与是否按单元选择。
struct SceneBoxSelectionResult {
    std::vector<std::string> faceIds;
    std::vector<std::string> unitIds;
    int entityCount = 0;
    int shapeType = 0;
    bool selectUnits = false;
};

// 可见性命令:隐藏选中/仅显示选中/显示选中/反相/全部显示。
enum class SceneVisibilityCommand {
    HideSelected,
    OnlySelected,
    ShowSelected,
    Invert,
    ShowAll,
};

// 撤销结果:是否已恢复可见性,以及因类型不匹配而推迟的记录。
struct SceneUndoResult {
    bool restoredVisibility = false;
    std::optional<UndoRecord> deferredRecord;
};

// 场景交互协调器:持有全局选择协调器与场景控制器,是 UI 事件
// (拾取/框选/可见性命令/撤销)到场景状态变更的翻译边界。
class SceneInteractionCoordinator final {
public:
    // 特征面解析回调:由调用方提供(如把面解析为所属孔/特征的全部面)。
    using FeatureFacesResolver =
        std::function<std::vector<std::string>(const std::string&)>;

    // 场景控制器访问(呈现/可见性)。
    SceneController& scene() noexcept { return m_scene; }
    const SceneController& scene() const noexcept { return m_scene; }

    // 选择源策略:设置/查询当前选择源,重置选择,以及按快照求当前选择。
    void setSelectionSource(SelectionSource source) noexcept;
    SelectionSource selectionSource() const noexcept;
    void resetSelection() noexcept;
    std::vector<std::string> currentSelection(
        SelectionSnapshot snapshot) const;

    // 把单次 OCCT 拾取映射为分析实体选择结果;内部按形状类型分派,
    // 并受单元选择模式/增选/整特征开关影响。返回的 action 非 None
    // 表示成功,同时会把选择源切到 Analysis。
    ScenePickResult mapPick(
        const TopoDS_Shape& shape,
        int shapeType,
        bool unitSelectionMode,
        bool additive,
        bool wholeFeature,
        std::vector<std::string> currentAnalysisSelection,
        const StepIndexer& stepIndexer,
        const ModelIndex& modelIndex,
        const FeatureFacesResolver& featureFaces) noexcept;

    // 把框选命中的形状集合映射为面/单元选择(面直接计入,边/线取其
    // 邻接面),结果含去重后的面与单元 id。
    SceneBoxSelectionResult mapBoxPick(
        const std::vector<TopoDS_Shape>& shapes,
        int shapeType,
        bool unitSelectionMode,
        const StepIndexer& stepIndexer,
        const ModelIndex& modelIndex) noexcept;

    // 在交互上下文中选中指定面(跳过隐藏面),返回实际选中的数量。
    int selectVisibleFaces(
        const Handle(AIS_InteractiveContext)& context,
        const std::vector<std::string>& faceIds) const;

    // 执行可见性命令:先校验前置条件,把当前可见性快照压入历史,
    // 再执行 beforeMutation 回调与命令本身,返回是否执行。
    bool applyVisibility(
        SceneVisibilityCommand command,
        SelectionSnapshot snapshot,
        CommandHistory& history,
        const std::function<void()>& beforeMutation = {});

    // 取历史最后一条记录执行撤销:可见性记录直接恢复并返回;
    // 非可见性记录原样退回(由调用方处理)。
    SceneUndoResult takeUndo(
        CommandHistory& history,
        const std::function<void()>& beforeVisibilityRestore = {});

private:
    SelectionCoordinator m_selection;   // 全局选择状态(选择源/快照)。
    SceneController m_scene;            // 场景呈现与可见性。
};

} // namespace oap::app
