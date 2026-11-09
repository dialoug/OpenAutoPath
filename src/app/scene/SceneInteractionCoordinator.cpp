// SceneInteractionCoordinator 实现（oap_app_scene）。
// OCCT 拾取、与面板无关的选择及可见性命令的应用边界，见 SceneInteractionCoordinator.h。
#include "SceneInteractionCoordinator.h"

#include <algorithm>
#include <unordered_set>
#include <utility>

#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>

#include "analysis/Model.h"
#include "analysis/ModelIndex.h"
#include "analysis/StepIndexer.h"

namespace oap::app {
namespace {

// 去重追加:仅在值非空且未出现过时追加到目标列表。
void appendUnique(
    std::vector<std::string>& target,
    std::unordered_set<std::string>& seen,
    const std::string& value)
{
    if (!value.empty() && seen.insert(value).second)
        target.push_back(value);
}

} // namespace

void SceneInteractionCoordinator::setSelectionSource(
    SelectionSource source) noexcept
{
    m_selection.setSource(source);
}

SelectionSource SceneInteractionCoordinator::selectionSource() const noexcept
{
    return m_selection.source();
}

void SceneInteractionCoordinator::resetSelection() noexcept
{
    m_selection.reset();
}

std::vector<std::string> SceneInteractionCoordinator::currentSelection(
    SelectionSnapshot snapshot) const
{
    return m_selection.current(std::move(snapshot));
}

// 拾取映射:把 OCCT 形状按类型翻译为分析实体选择。面拾取受
// 单元模式/整特征/增选三种开关影响:单元模式选单元,整特征解析
// 特征面集合(失败回退单面),增选切换单面,否则单选;边/线则映射
// 为信息展示动作。成功命中时把选择源切到 Analysis。
ScenePickResult SceneInteractionCoordinator::mapPick(
    const TopoDS_Shape& shape,
    int shapeType,
    bool unitSelectionMode,
    bool additive,
    bool wholeFeature,
    std::vector<std::string> currentAnalysisSelection,
    const StepIndexer& stepIndexer,
    const ModelIndex& modelIndex,
    const FeatureFacesResolver& featureFaces) noexcept
{
    ScenePickResult result;
    if (shape.IsNull())
        return result;

    try {
        const TopAbs_ShapeEnum type =
            static_cast<TopAbs_ShapeEnum>(shapeType);
        // 面拾取:先经步骤索引器取稳定面 id,再按开关决定动作。
        if (type == TopAbs_FACE && shape.ShapeType() == TopAbs_FACE) {
            const std::string faceId =
                stepIndexer.idForFace(TopoDS::Face(shape));
            if (faceId.empty())
                return result;

            result.entityId = faceId;
            result.requestHoleStageHighlight = !unitSelectionMode;
            if (unitSelectionMode) {
                if (const Face* face = modelIndex.face(faceId)) {
                    result.action = ScenePickAction::SelectUnit;
                    result.unitId = face->unit_id;
                }
            } else if (wholeFeature) {
                std::vector<std::string> target = featureFaces
                    ? featureFaces(faceId) : std::vector<std::string>{};
                result.matchedFeature = !target.empty();
                if (target.empty())
                    target.push_back(faceId);
                std::unordered_set<std::string> seen(
                    currentAnalysisSelection.begin(),
                    currentAnalysisSelection.end());
                for (const std::string& id : target)
                    appendUnique(currentAnalysisSelection, seen, id);
                result.action = ScenePickAction::SelectFaces;
                result.faceIds = std::move(currentAnalysisSelection);
            } else if (additive) {
                result.action = ScenePickAction::ToggleFace;
                result.faceIds = {faceId};
            } else {
                result.action = ScenePickAction::SelectFace;
                result.faceIds = {faceId};
            }
        } else if (type == TopAbs_EDGE && shape.ShapeType() == TopAbs_EDGE) {
            const std::string edgeId =
                stepIndexer.idForEdge(TopoDS::Edge(shape));
            if (const Edge* edge = modelIndex.edge(edgeId)) {
                result.action = ScenePickAction::ShowEdge;
                result.entityId = edgeId;
                result.entitySubtype = edge->curve_type;
                result.relatedFaceCount =
                    static_cast<int>(edge->face_ids.size());
                if (!edge->face_ids.empty())
                    result.faceIds = {edge->face_ids.front()};
            }
        } else if (type == TopAbs_WIRE && shape.ShapeType() == TopAbs_WIRE) {
            const std::string wireId =
                stepIndexer.idForWire(TopoDS::Wire(shape));
            if (const Wire* wire = modelIndex.wire(wireId)) {
                result.action = ScenePickAction::ShowWire;
                result.entityId = wireId;
                result.entitySubtype = wire->wire_role;
                result.faceIds = {wire->face_id};
            }
        }
    } catch (...) {
        return {};
    }

    // 拾取被接受后同步选择源,使后续命令的“当前选择”与该动作一致。
    if (result.accepted())
        m_selection.setSource(SelectionSource::Analysis);
    return result;
}

// 框选映射:对每个命中形状按类型取面(面直接取,边取其邻接面,线取
// 其所在面),经去重后汇总面与单元 id;异常或空命中时返回空结果。
SceneBoxSelectionResult SceneInteractionCoordinator::mapBoxPick(
    const std::vector<TopoDS_Shape>& shapes,
    int shapeType,
    bool unitSelectionMode,
    const StepIndexer& stepIndexer,
    const ModelIndex& modelIndex) noexcept
{
    SceneBoxSelectionResult result;
    result.shapeType = shapeType;
    std::unordered_set<std::string> seenFaces;
    std::unordered_set<std::string> seenUnits;
    const auto addFace = [&](const std::string& faceId) {
        appendUnique(result.faceIds, seenFaces, faceId);
        if (const Face* face = modelIndex.face(faceId))
            appendUnique(result.unitIds, seenUnits, face->unit_id);
    };

    try {
        const TopAbs_ShapeEnum type =
            static_cast<TopAbs_ShapeEnum>(shapeType);
        for (const TopoDS_Shape& shape : shapes) {
            if (shape.IsNull())
                continue;
            if (type == TopAbs_FACE && shape.ShapeType() == TopAbs_FACE) {
                addFace(stepIndexer.idForFace(TopoDS::Face(shape)));
                ++result.entityCount;
            } else if (type == TopAbs_EDGE &&
                       shape.ShapeType() == TopAbs_EDGE) {
                const std::string edgeId =
                    stepIndexer.idForEdge(TopoDS::Edge(shape));
                if (const Edge* edge = modelIndex.edge(edgeId)) {
                    for (const std::string& faceId : edge->face_ids)
                        addFace(faceId);
                }
                ++result.entityCount;
            } else if (type == TopAbs_WIRE &&
                       shape.ShapeType() == TopAbs_WIRE) {
                const std::string wireId =
                    stepIndexer.idForWire(TopoDS::Wire(shape));
                if (const Wire* wire = modelIndex.wire(wireId))
                    addFace(wire->face_id);
                ++result.entityCount;
            }
        }
        result.selectUnits = unitSelectionMode && type == TopAbs_FACE;
    } catch (...) {
        return {};
    }

    m_selection.setSource(SelectionSource::Analysis);
    return result;
}

// 选中可见面:清空既有选中后逐面判断可见性与 AIS 对象存在性,
// 以累加方式加入选中;返回实际选中的数量。
int SceneInteractionCoordinator::selectVisibleFaces(
    const Handle(AIS_InteractiveContext)& context,
    const std::vector<std::string>& faceIds) const
{
    if (context.IsNull())
        return 0;
    context->ClearSelected(Standard_False);
    int selected = 0;
    for (const std::string& faceId : faceIds) {
        if (m_scene.isHidden(faceId))
            continue;
        const Handle(AIS_Shape)& ais = m_scene.aisForFace(faceId);
        if (!ais.IsNull()) {
            context->AddOrRemoveSelected(ais, Standard_False);
            ++selected;
        }
    }
    return selected;
}

// 可见性命令事务:先按命令校验前置条件(依赖选择的命令需有选中、
// 反相需有面、其余需有隐藏项),再把当前可见性快照压入历史作为
// 撤销依据,执行 beforeMutation 回调后应用命令本身。
bool SceneInteractionCoordinator::applyVisibility(
    SceneVisibilityCommand command,
    SelectionSnapshot snapshot,
    CommandHistory& history,
    const std::function<void()>& beforeMutation)
{
    std::vector<std::string> selected;
    if (command == SceneVisibilityCommand::HideSelected ||
        command == SceneVisibilityCommand::OnlySelected ||
        command == SceneVisibilityCommand::ShowSelected) {
        selected = m_selection.current(std::move(snapshot));
        if (selected.empty())
            return false;
    } else if (command == SceneVisibilityCommand::Invert) {
        if (m_scene.faceCount() == 0)
            return false;
    } else if (m_scene.hiddenCount() == 0) {
        return false;
    }

    history.pushVisibility(m_scene.hiddenSnapshot());
    if (beforeMutation)
        beforeMutation();
    switch (command) {
        case SceneVisibilityCommand::HideSelected:
            m_scene.hideFaces(selected);
            break;
        case SceneVisibilityCommand::OnlySelected:
            m_scene.onlyFaces(selected);
            break;
        case SceneVisibilityCommand::ShowSelected:
            m_scene.showFaces(selected);
            break;
        case SceneVisibilityCommand::Invert:
            m_scene.invertVisibility();
            break;
        case SceneVisibilityCommand::ShowAll:
            m_scene.showAll();
            break;
    }
    return true;
}

// 撤销事务:取出历史末条记录,仅处理可见性记录(先执行前置回调,
// 再恢复隐藏面快照);其它类型记录原样退回给调用方处理。
SceneUndoResult SceneInteractionCoordinator::takeUndo(
    CommandHistory& history,
    const std::function<void()>& beforeVisibilityRestore)
{
    SceneUndoResult result;
    std::optional<UndoRecord> record = history.takeLast();
    if (!record)
        return result;
    if (record->kind != UndoRecord::Kind::Visibility) {
        result.deferredRecord = std::move(record);
        return result;
    }
    if (beforeVisibilityRestore)
        beforeVisibilityRestore();
    m_scene.restoreHidden(record->hiddenFaces);
    result.restoredVisibility = true;
    return result;
}

} // namespace oap::app
