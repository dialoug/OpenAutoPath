// Stable hole feature identities, resource definitions and input fingerprints.
#include "manufacturing/hole/HoleMachiningJobService.h"
#include <algorithm>
#include <cctype>
#include <locale>
#include <set>
#include <sstream>
#include <utility>
#include <QCryptographicHash>
#include "manufacturing/job/MachiningJobStorage.h"

#include "manufacturing/hole/detail/HoleJobTypes.h"
#include "manufacturing/hole/detail/HoleJobFoundation.h"

namespace oap::manufacturing::hole_job_detail {

// 稳定 featureId/geometryFingerprint、Setup/ToolController ID 构造、
// 旧孔资源清理与操作指纹计算，均为纯函数。

std::string sha256(const std::string& value)
{
    return QCryptographicHash::hash(
               QByteArray::fromStdString(value), QCryptographicHash::Sha256)
        .toHex().toStdString();
}

std::string pointToken(const gp_Pnt& point)
{
    std::ostringstream stream;
    stream.precision(17);
    stream << point.X() << ',' << point.Y() << ',' << point.Z();
    return stream.str();
}

// 孔稳定特性 ID：源面集合排序后哈希；无源面时回退到几何（出入口+半径）。
std::string featureId(const toolpath::HoleMachiningInput& hole)
{
    std::vector<std::string> faces = hole.sourceFaceIds;
    std::sort(faces.begin(), faces.end());
    std::ostringstream source;
    for (const std::string& face : faces)
        source << face << '\n';
    if (faces.empty()) {
        source << pointToken(hole.entry) << '|' << pointToken(hole.bottom)
               << '|' << hole.radius;
    }
    return "hole:" + sha256(source.str());
}

// 孔几何指纹：种类、出入口、半径、轮廓/入口解析、装夹面、父级关系与源面；
// 重识别后按 featureId 匹配失败时用它做容差重绑定。
std::string geometryFingerprint(const toolpath::HoleMachiningInput& hole)
{
    std::vector<std::string> faces = hole.sourceFaceIds;
    std::sort(faces.begin(), faces.end());
    std::ostringstream source;
    source.precision(17);
    source << static_cast<int>(hole.kind) << '|'
           << pointToken(hole.entry) << '|' << pointToken(hole.bottom) << '|'
           << hole.radius << '|' << static_cast<int>(hole.bottomProfile) << '|'
           << static_cast<int>(hole.entryResolution) << '|'
           << static_cast<int>(hole.setupFace) << '|'
           << static_cast<int>(hole.parentRelation) << '|'
           << hole.parentCounterboreFaceId << '|'
           << hole.parentCounterboreFloorFaceId << '|'
           << hole.parentCounterboreRadius;
    for (const std::string& face : faces)
        source << '|' << face;
    return sha256(source.str());
}

// 装夹 ID：setup-<index>-<face>（正负号转 pos/neg，其余小写），跨保存稳定。
std::string setupId(int setupIndex, MoldSetupFace face)
{
    std::ostringstream value;
    value << "setup-" << setupIndex + 1 << '-';
    for (const char* p = moldSetupFaceName(face); *p != '\0'; ++p) {
        if (*p == '+') value << "pos";
        else if (*p == '-') value << "neg";
        else value << static_cast<char>(std::tolower(
            static_cast<unsigned char>(*p)));
    }
    return value.str();
}

// 刀具控制器 ID：t<刀号>（无刀号用回退序号）+ 参数指纹前 12 位，
// 同一刀具参数跨捕获保持同一 ID。
std::string controllerId(const toolpath::ToolpathToolContext& tool,
                         std::size_t fallbackIndex,
                         const std::string& snapshotFingerprint)
{
    std::ostringstream value;
    value << "tool-controller-";
    if (tool.toolNumber > 0)
        value << 't' << tool.toolNumber;
    else
        value << fallbackIndex + 1;
    const std::string fingerprint = snapshotFingerprint.empty()
        ? sha256(toolpath::toolpathToolContextKey(tool))
        : snapshotFingerprint;
    value << '-' << fingerprint.substr(0, 12);
    return value.str();
}

std::string controllerKey(
    const toolpath::ToolpathToolContext& tool)
{
    return sha256(toolpath::toolpathToolContextKey(tool));
}

// 手工孔段占位刀具控制器：仅取参数集中的直径/转速/进给等，valid=false 表示未入刀库。


// 构造一个装夹定义：优先从六面工作框取坐标与确认；确认时写入输入指纹。
SetupDefinition makeSetup(
    int index, MoldSetupFace face,
    const toolpath::MoldSetupWorkFrames* frames,
    const std::string& partFingerprint)
{
    SetupDefinition result;
    result.id = setupId(index, face);
    result.setupIndex = index;
    result.face = face;
    const toolpath::MoldFaceSetupWorkFrame* frame =
        frames ? frames->forFace(face) : nullptr;
    if (frame && frame->valid) {
        result.workOffset = frame->workOffset;
        result.origin = toJobVector(frame->setupWorkFrame.Location());
        result.xAxis = toJobVector(frame->setupWorkFrame.XDirection());
        result.yAxis = toJobVector(frame->setupWorkFrame.YDirection());
        result.zAxis = toJobVector(frame->setupWorkFrame.Direction());
        result.valid = true;
        result.requiresPhysicalReclamp = frame->requiresPhysicalReclamp;
        result.axisMapping = frame->axisMapping;
        result.operatorInstruction = frame->operatorInstruction;
        result.confirmation.confirmed = frame->confirmed;
    }
    if (result.confirmation.confirmed) {
        PartDefinition part;
        part.sourceFingerprint = partFingerprint;
        result.confirmation.confirmedInputFingerprint =
            MachiningJobStorage::setupInputFingerprint(part, result)
                .toStdString();
    }
    return result;
}

// 保证定义中存在指定面的装夹：已有则复用；否则分配未占用的 0..5 索引新建。
std::string ensureSetup(
    MachiningJobDefinition& definition,
    int preferredIndex,
    MoldSetupFace face,
    const toolpath::MoldSetupWorkFrames* frames)
{
    const auto sameFace = std::find_if(
        definition.setups.begin(), definition.setups.end(),
        [face](const SetupDefinition& setup) {
            return setup.face == face;
        });
    if (sameFace != definition.setups.end())
        return sameFace->id;

    std::set<int> used;
    for (const SetupDefinition& setup : definition.setups)
        used.insert(setup.setupIndex);
    int setupIndex = preferredIndex;
    if (setupIndex < 0 || setupIndex >= 6 ||
        used.contains(setupIndex)) {
        setupIndex = -1;
        for (int candidate = 0; candidate < 6; ++candidate) {
            if (!used.contains(candidate)) {
                setupIndex = candidate;
                break;
            }
        }
    }
    if (setupIndex < 0)
        return {};
    SetupDefinition setup = makeSetup(
        setupIndex, face, frames,
        definition.part.sourceFingerprint);
    const std::string id = setup.id;
    definition.setups.push_back(std::move(setup));
    return id;
}

// 丢弃旧孔资源：清空孔工序，并剔除未被槽腔工序引用的 Setup/刀具控制器；
// 保证整段孔捕获替换后 Job 定义不残留孤立资源。
void discardOldHoleResources(
    MachiningJobDefinition& definition)
{
    std::set<std::string> retainedSetups;
    std::set<std::string> retainedControllers;
    for (const PocketOperationDefinition& operation :
         definition.pocketOperations) {
        retainedSetups.insert(operation.setupId);
        retainedControllers.insert(operation.toolControllerId);
    }
    for (const ProfileOperationDefinition& operation :
         definition.profileOperations) {
        retainedSetups.insert(operation.setupId);
        retainedControllers.insert(operation.toolControllerId);
    }
    for (const auto& operation : definition.finishingOperations) {
        retainedSetups.insert(operation.setupId);
        retainedControllers.insert(operation.toolControllerId);
    }
    definition.holeOperations.clear();
    std::erase_if(
        definition.setups,
        [&retainedSetups](const SetupDefinition& setup) {
            return !retainedSetups.contains(setup.id);
        });
    std::erase_if(
        definition.toolControllers,
        [&retainedControllers](
            const JobToolController& controller) {
            return !retainedControllers.contains(controller.id);
        });
}

// 构造孔目标引用：填充稳定 featureId/几何指纹与来源/实体孔索引；
// coveredSourceHoleIndices 缺省时以自身来源索引兜底。
HoleTargetReference makeTarget(
    const toolpath::HoleMachiningInput& hole, int sourceIndex, int physicalIndex,
    toolpath::HoleMachiningStrategy strategy,
    const std::vector<std::size_t>& coveredSourceIndices)
{
    HoleTargetReference result;
    result.featureId = featureId(hole);
    result.geometryFingerprint = geometryFingerprint(hole);
    result.sourceHoleIndex = sourceIndex;
    result.coveredSourceHoleIndices.reserve(coveredSourceIndices.size());
    for (const std::size_t coveredSourceIndex : coveredSourceIndices) {
        result.coveredSourceHoleIndices.push_back(
            static_cast<int>(coveredSourceIndex));
    }
    if (result.coveredSourceHoleIndices.empty())
        result.coveredSourceHoleIndices.push_back(sourceIndex);
    result.physicalHoleIndex = physicalIndex;
    result.hole = hole;
    result.strategy = strategy;
    return result;
}

// 工序输入指纹：Setup 指纹、刀具上下文键、阶段/启用/局部批次与覆盖键、
// 各目标的几何指纹+策略+覆盖来源；用于运行期结果失效判定。
std::string operationFingerprint(
    const MachiningJobDefinition& definition,
    const HoleOperationDefinition& operation)
{
    std::ostringstream input;
    const auto setup = std::find_if(
        definition.setups.begin(), definition.setups.end(),
        [&](const SetupDefinition& value) {
            return value.id == operation.setupId;
        });
    const auto controller = std::find_if(
        definition.toolControllers.begin(), definition.toolControllers.end(),
        [&](const JobToolController& value) {
            return value.id == operation.toolControllerId;
        });
    input << operation.setupId << '|'
          << (setup != definition.setups.end()
                  ? MachiningJobStorage::setupInputFingerprint(
                        definition.part, *setup).toStdString()
                  : std::string{}) << '|'
          << operation.toolControllerId << '|'
          << (controller != definition.toolControllers.end()
                  ? toolpath::toolpathToolContextKey(controller->tool)
                  : std::string{}) << '|'
          << static_cast<int>(operation.phase) << '|'
          << operation.enabled << '|'
          << operation.sequence;
    // Sampling precision affects the move stream used by IPW.
    if (definition.holeParameters.hole.helixChordTolerance != 0.0) {
        std::ostringstream precision;
        precision.imbue(std::locale::classic());
        precision.precision(17);
        precision << definition.holeParameters.hole.helixChordTolerance;
        input << "|helix-chord-mm:" << precision.str();
    }
    for (const HoleTargetReference& target : operation.targets) {
        input << '|' << target.geometryFingerprint << ':'
              << static_cast<int>(target.strategy);
        for (const int sourceIndex : target.coveredSourceHoleIndices)
            input << ':' << sourceIndex;
    }
    return sha256(input.str());
}

} // namespace oap::manufacturing::hole_job_detail
