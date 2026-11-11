// RecognitionWorkspaceCoordinator 实现（oap_app_recognition）。
// 识别状态、任务与 OCCT 呈现的应用边界，见 RecognitionWorkspaceCoordinator.h。
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include <QCoreApplication>

#include "analysis/MoldCoordinateFrame.h"
#include "analysis/Model.h"
#include "analysis/AabbBoundaryFace.h"
#include "analysis/StepIndexer.h"
#include "app/presentation/SceneController.h"
#include "app/workflows/shared/UnitDisplayGrouping.h"
#include "app/presentation/InspectionOverlayPresenter.h"
#include "app/presentation/RecognitionPresentationCoordinator.h"
#include "app/workflows/hole/HoleJobSourceSynchronizer.h"
#include "tooling/adapter/hole/HoleMachiningInputMapper.h"
#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"

#include <algorithm>
#include <set>
#include <unordered_set>
#include <utility>

#include <BRepBndLib.hxx>
#include <Quantity_Color.hxx>

namespace oap::app {
namespace {

// 通结构面在场景中的着色颜色(浅蓝)。
const Quantity_Color kThroughStructureColor(
    0.53, 0.81, 0.98, Quantity_TOC_sRGB);

// 是否为孔类特征(通孔/盲孔/沉头孔)。
bool isHoleFeature(FeatureKind kind) noexcept
{
    return kind == FeatureKind::ThroughHole ||
        kind == FeatureKind::BlindHole ||
        kind == FeatureKind::CountersinkHole;
}

// 从识别结果的所有实例中移除指定面:先从各实例的面列表里剔除,
// 再删除面列表清空或底面也被移除的实例;返回移除的面总数。
int removeFacesFromInstances(
    RecognitionResult& recognition,
    const std::set<std::string>& faceIds)
{
    int removed = 0;
    for (RecognitionResult::Instance& instance : recognition.instances) {
        std::vector<std::string>& faces = instance.faceIds;
        const std::size_t before = faces.size();
        faces.erase(
            std::remove_if(
                faces.begin(), faces.end(),
                [&faceIds](const std::string& faceId) {
                    return faceIds.contains(faceId);
                }),
            faces.end());
        removed += static_cast<int>(before - faces.size());
    }
    recognition.instances.erase(
        std::remove_if(
            recognition.instances.begin(), recognition.instances.end(),
            [&faceIds](const RecognitionResult::Instance& instance) {
                return instance.faceIds.empty() ||
                    (!instance.floorFaceId.empty() &&
                     faceIds.contains(instance.floorFaceId));
            }),
        recognition.instances.end());
    return removed;
}

// 由面集合生成实例键:排序去重后用 '|' 连接,使同一组面无论顺序
// 如何都对应同一实例(用于去重与冲突检测)。
std::string instanceKey(std::vector<std::string> faceIds)
{
    std::sort(faceIds.begin(), faceIds.end());
    faceIds.erase(
        std::unique(faceIds.begin(), faceIds.end()),
        faceIds.end());
    std::string key;
    for (const std::string& faceId : faceIds) {
        if (!key.empty())
            key += '|';
        key += faceId;
    }
    return key;
}

// 构造识别成功的汇总消息:涵盖孔/通结构、三向(Z→Y→X)线追踪统计、
// 盲槽/盲腔连通、无效面跳过与耗时等可读性信息(供 UI 直接展示)。
QString buildSuccessMessage(
    const RecognitionSession& session,
    const ThreeAxisWireTraceUseCaseResult& useCaseResult,
    int recognizedHoleFaceCount,
    int visiblePlaneFaceCount,
    int visibleTraceFaceCount,
    qint64 elapsedMillis)
{
    QString message = QCoreApplication::translate("OpenAutoPath", "内外环结构识别完成：识别孔 %1 个、孔面 %2 个，通结构面 %3 个；"
        "三向执行顺序 Z→Y→X，孔、通结构及前序方向的根平面和路径"
        "均在后续追踪入口排除。")
        .arg(session.result().holeCount)
        .arg(recognizedHoleFaceCount)
        .arg(session.throughStructureFaces().size());

    int skippedInvalidFaceCount = 0;
    int finalBlindSlotSubsetCount = 0;
    int finalBlindPocketSubsetCount = 0;
    for (const ThreeAxisFeatureSubset& subset :
         session.wireTraceFeatureSubsets()) {
        if (subset.featureKind == AxisWireTraceFeatureKind::BlindSlot)
            ++finalBlindSlotSubsetCount;
        else
            ++finalBlindPocketSubsetCount;
    }
    for (const ThreeAxisTraceStats& stats : useCaseResult.axisStats) {
        skippedInvalidFaceCount += stats.skippedInvalidFaceCount;
        message += QCoreApplication::translate("OpenAutoPath", " %1向：内部平面 %2 个、追踪根平面 %3 个、有效分支 %4 个"
            "（盲槽 %5、盲腔 %6），唯一特征面 %7 个，终止平面 %8 个。")
            .arg(QString::fromStdString(stats.axisLabel))
            .arg(stats.planeFaceIds.size())
            .arg(stats.rootFaceCount)
            .arg(stats.activePathCount)
            .arg(stats.blindSlotPathCount)
            .arg(stats.blindPocketPathCount)
            .arg(stats.traceFaceIds.size())
            .arg(stats.terminalFaceCount);
    }
    message += QCoreApplication::translate("OpenAutoPath", " 三向内部平面共 %1 个（X 红、Y 绿、Z 蓝，当前显示 %2 个）；"
        "内外环共形成 %3 个最终连通结构，唯一走过 %4 个面"
        "（盲槽深红 %5 个、盲腔深青 %6 个；"
        "盲腔连通合并吸收 %7 个原始子集，当前显示 %8 个面），"
        "同轴较高根平面覆盖较低路径的面归属 %9 次。"
        "非主外表面来源根面可作为盲腔连通桥，其外环盲腔还可经终止面"
        "接入已有盲腔，但所有连接面均不加入特征子集；"
        "各最终结构以白色外轮廓分隔，终止平面不属于走过面；"
        "已知孔结构面 %10 个、通结构面 %11 个均在入口排除。")
        .arg(session.axisParallelPlaneFaces().size())
        .arg(visiblePlaneFaceCount)
        .arg(session.wireTraceFeatureSubsets().size())
        .arg(session.combinedWireTraceFaces().size())
        .arg(finalBlindSlotSubsetCount)
        .arg(finalBlindPocketSubsetCount)
        .arg(useCaseResult.absorbedBlindPocketSubsetCount)
        .arg(visibleTraceFaceCount)
        .arg(useCaseResult.overwrittenFaceCount)
        .arg(recognizedHoleFaceCount)
        .arg(session.throughStructureFaces().size());
    if (skippedInvalidFaceCount > 0) {
        message += QCoreApplication::translate("OpenAutoPath", " 跳过 %1 个几何无效面。")
            .arg(skippedInvalidFaceCount);
    }
    if (useCaseResult.failedUnitCount > 0) {
        message += QCoreApplication::translate("OpenAutoPath", " 另有 %1 个单元因拓扑异常跳过。")
            .arg(useCaseResult.failedUnitCount);
    }
    message += QCoreApplication::translate("OpenAutoPath", " 通结构筛选检查法向垂直 Z 的面 %1 个，排除孔面 %2 个、"
        "主外表面 %3 个；无非 Z 向共享边 %4 个，"
        "存在非凸非 Z 向边 %5 个。"
        " 通结构与 XYZ 追踪共享逐体拓扑缓存，本阶段耗时 %6 ms。")
        .arg(useCaseResult.verticalSurfaceCount)
        .arg(useCaseResult.excludedHoleFaceCount)
        .arg(useCaseResult.excludedMainOuterFaceCount)
        .arg(useCaseResult.rejectedWithoutNonZNeighborCount)
        .arg(useCaseResult.rejectedNonConvexCount)
        .arg(elapsedMillis);
    return message;
}

} // namespace

// 构造函数:携带外部 runner(可注入测试替身)创建任务控制器,
// 并创建识别呈现协调器与叠加层检查呈现器。
RecognitionWorkspaceCoordinator::RecognitionWorkspaceCoordinator(
    presentation::OverlayController& overlays,
    ThreeAxisRecognitionController::Runner runner)
    : m_controller(std::move(runner)),
      m_presentation(std::make_unique<
          presentation::RecognitionPresentationCoordinator>()),
      m_inspectionPresenter(std::make_unique<
          presentation::InspectionOverlayPresenter>(overlays))
{
}

// 析构:同步取消并等待后台识别任务,避免任务访问已销毁的会话。
RecognitionWorkspaceCoordinator::~RecognitionWorkspaceCoordinator()
{
    cancelAndWait();
}

std::uint64_t RecognitionWorkspaceCoordinator::start(
    ThreeAxisRecognitionRequest request)
{
    return m_controller.start(request);
}

void RecognitionWorkspaceCoordinator::cancel(std::uint64_t requestId)
{
    m_controller.cancel(requestId);
}

void RecognitionWorkspaceCoordinator::cancelAndWait()
{
    m_controller.cancelAndWait();
}

ThreeAxisRecognitionProgress RecognitionWorkspaceCoordinator::progress(
    std::uint64_t requestId) const
{
    return m_controller.progress(requestId);
}

bool RecognitionWorkspaceCoordinator::ready(
    std::uint64_t requestId) const
{
    return m_controller.ready(requestId);
}

std::optional<ThreeAxisRecognitionTaskResult>
RecognitionWorkspaceCoordinator::take(std::uint64_t requestId)
{
    return m_controller.take(requestId);
}

// 模型切换时的级联重置:先终止后台任务,再重置会话与识别呈现,
// 最后清空叠加层,保证旧模型的识别状态完全不残留。
void RecognitionWorkspaceCoordinator::resetForModel()
{
    cancelAndWait();
    m_session.reset();
    if (m_presentation)
        m_presentation->reset();
    clearInspection();
}

// 安装孔几何识别结果:先由模型单元包围盒合并出整体包围盒,再把识别
// 几何按包围盒与模具坐标系映射为孔加工输入并写回会话。
const std::vector<toolpath::HoleMachiningInput>&
RecognitionWorkspaceCoordinator::installHoleGeometryRecognition(
    HoleGeometryRecognitionResult recognition,
    const Model& model,
    const MoldCoordinateFrame& moldFrame)
{
    Bbox bounds;
    for (const Unit& unit : model.units)
        bounds.merge(unit.bbox);
    m_session.holeGeometryProfile() = std::move(recognition);
    m_session.holeToolpathInputs() =
        tooling::HoleMachiningInputMapper().resolve(
            m_session.holeGeometryProfile().geometries,
            bounds, moldFrame);
    m_session.hasHoleToolpathInputs() = true;
    return m_session.holeToolpathInputs();
}

// 槽区域缓存懒构建:仅首次调用时由槽源构建根内线槽区域(可含跳过
// 项),之后直接返回缓存,避免重复计算。
const toolpath::PocketRegionWorkflowResult&
RecognitionWorkspaceCoordinator::ensurePocketRegionCache(
    const StepIndexer& stepIndexer)
{
    if (!m_session.hasPocketRegionCache()) {
        m_session.pocketRegionCache() = {};
        m_session.pocketRegionCache().regions =
            toolpath::PocketRegionBuilder().buildRootInnerWireRegions(
                m_session.pocketSources(), stepIndexer, {},
                &m_session.pocketRegionCache().skippedRegions);
        m_session.hasPocketRegionCache() = true;
    }
    return m_session.pocketRegionCache();
}

const std::vector<toolpath::ProfileRegion>&
RecognitionWorkspaceCoordinator::ensureProfileRegionCache(
    const StepIndexer& stepIndexer)
{
    if (!m_session.hasProfileRegionCache()) {
        m_session.profileRegionCache() =
            toolpath::ProfileRegionBuilder().build(
                m_session.profileSources(), stepIndexer);
        m_session.hasProfileRegionCache() = true;
    }
    return m_session.profileRegionCache();
}

void RecognitionWorkspaceCoordinator::synchronizeLoadedProjectHoles(
    const std::vector<toolpath::HoleMachiningInput>& recognizedHoles)
{
    synchronizeLoadedJobHoleSources(
        recognizedHoles, m_session.result(),
        m_session.holeToolpathInputs(),
        m_session.hasHoleToolpathInputs());
}

RecognitionInstallResult
RecognitionWorkspaceCoordinator::installThreeAxisResult(
    const RecognitionResult& recognition,
    ThreeAxisWireTraceUseCaseResult useCaseResult,
    const Model& model,
    const MoldCoordinateFrame& moldFrame,
    SceneController& scene,
    qint64 elapsedMillis,
    const std::function<void()>& beforeCommit)
{
    RecognitionInstallResult result;
    // 安装门禁:无模型 / 已取消 / 失败均直接返回,不动会话。
    if (model.faces.empty() || model.units.empty()) {
        result.status = RecognitionInstallStatus::NoModel;
        result.message = QCoreApplication::translate("OpenAutoPath", "请先打开并分析一个 STEP 文件。");
        return result;
    }
    if (useCaseResult.cancelled) {
        result.status = RecognitionInstallStatus::Cancelled;
        result.message = QCoreApplication::translate("OpenAutoPath", "结构识别已取消，上一次有效结果保持不变。");
        return result;
    }
    if (!useCaseResult.error.empty()) {
        result.status = RecognitionInstallStatus::Failed;
        result.message = QCoreApplication::translate("OpenAutoPath", "结构识别失败：%1")
            .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(useCaseResult.error).toUtf8().constData()));
        return result;
    }

    // 先在候选会话上装配全部派生状态(孔输入/通结构面/线追踪子集/
    // 槽源等),确保只有完整构建成功才替换主会话;beforeCommit 供
    // 调用方在替换前同步清理,随后原子替换并重置场景颜色、重建呈现。
    RecognitionSession candidate;
    candidate.result() = recognition;
    candidate.hasResult() = true;
    Bbox bounds;
    for (const Unit& unit : model.units)
        bounds.merge(unit.bbox);
    candidate.holeToolpathInputs() =
        tooling::HoleMachiningInputMapper().resolve(
            recognition.holeGeometries, bounds, moldFrame);
    candidate.hasHoleToolpathInputs() = true;
    candidate.combinedWireTraceFaces() =
        std::move(useCaseResult.combinedTraceFaceIds);
    candidate.throughStructureFaces() =
        std::move(useCaseResult.throughStructureFaceIds);
    candidate.axisParallelPlaneFaces() =
        std::move(useCaseResult.axisParallelPlaneFaceIds);
    candidate.wireTraceFeatureSubsets() =
        std::move(useCaseResult.featureSubsets);
    candidate.pocketSources().reserve(useCaseResult.pocketSources.size());
    for (ThreeAxisPocketSource& value : useCaseResult.pocketSources) {
        toolpath::RootInnerWirePocketSource source;
        source.kind = value.kind;
        source.sourceFaceIds = std::move(value.sourceFaceIds);
        source.rootFaceId = std::move(value.rootFaceId);
        source.rootWireId = std::move(value.rootWireId);
        source.allowOpenSectionContours = value.allowOpenSectionContours;
        source.openBoundaryFaceCount = value.openBoundaryFaceCount;
        candidate.pocketSources().push_back(std::move(source));
    }
    candidate.profileSources().reserve(useCaseResult.profileSources.size());
    for (ThreeAxisProfileSource& value : useCaseResult.profileSources) {
        toolpath::RootOuterWireProfileSource source;
        source.unitId = std::move(value.unitId);
        source.axisLabel = std::move(value.axisLabel);
        source.normalDirection = value.normalDirection;
        source.rootFaceId = std::move(value.rootFaceId);
        source.rootWireId = std::move(value.rootWireId);
        source.sourceFaceIds = std::move(value.sourceFaceIds);
        source.machiningFaceIds = std::move(value.machiningFaceIds);
        source.terminalFaceIds = std::move(value.terminalFaceIds);
        source.mainOuterFaceIds = std::move(value.mainOuterFaceIds);
        candidate.profileSources().push_back(std::move(source));
    }
    candidate.pocketRegionCache() = {};
    candidate.hasPocketRegionCache() = false;
    candidate.profileRegionCache().clear();
    candidate.hasProfileRegionCache() = false;

    int recognizedHoleFaceCount = 0;
    for (const auto& [faceId, kind] : recognition.faceKind) {
        (void)faceId;
        if (isHoleFeature(kind))
            ++recognizedHoleFaceCount;
    }

    if (beforeCommit)
        beforeCommit();
    // 原子替换会话后重建识别呈现并应用特征着色。
    m_session = std::move(candidate);
    scene.resetColors();
    const presentation::RecognitionPresentationStats presentationStats =
        m_presentation->rebuild(
            useCaseResult.axisStats,
            m_session.wireTraceFeatureSubsets(), scene);
    applyRecognitionColors(scene);

    result.status = RecognitionInstallStatus::Installed;
    result.hasCombinedPresentation =
        !m_session.result().faceKind.empty() ||
        !m_session.throughStructureFaces().empty() ||
        !m_presentation->empty();
    result.message = buildSuccessMessage(
        m_session, useCaseResult, recognizedHoleFaceCount,
        presentationStats.visiblePlaneFaces,
        presentationStats.visibleTraceFaces,
        elapsedMillis);
    return result;
}

// 手动特征编辑:把选中面归属的实例改成指定特征(或清除型编辑槽/腔)。
// 思路:在候选副本上操作,先校验(空选择/单底面/底面有效/底面不歧义),
// 再匹配实例并改 kind,清除型编辑还会为未识别面扩展种子底面并新建实例
// (带冲突检测与警告),最后重建派生索引并原子写回会话。
ManualRecognitionEditResult
RecognitionWorkspaceCoordinator::applyManualFeatureEdit(
    FeatureKind kind,
    const std::vector<std::string>& selectedFaceIds,
    const Model& model)
{
    ManualRecognitionEditResult result;
    // 前置校验:空选择直接返回;槽/腔要求恰好一个底面,且底面必须是
    // 平面且不被多个实例同时引用。
    if (selectedFaceIds.empty()) {
        result.status = ManualRecognitionEditStatus::EmptySelection;
        return result;
    }
    result.clearingKind =
        kind == FeatureKind::Slot || kind == FeatureKind::Pocket;
    if (result.clearingKind && selectedFaceIds.size() != 1) {
        result.status = ManualRecognitionEditStatus::RequiresSingleFloor;
        return result;
    }
    if (result.clearingKind) {
        result.floorFaceId = selectedFaceIds.front();
        const auto floor = std::find_if(
            model.faces.begin(), model.faces.end(),
            [&result](const Face& face) {
                return face.face_id == result.floorFaceId;
            });
        if (floor == model.faces.end() || floor->surface_type != "plane") {
            result.status = ManualRecognitionEditStatus::InvalidFloor;
            return result;
        }
        const int containingInstances = static_cast<int>(std::count_if(
            m_session.result().instances.begin(),
            m_session.result().instances.end(),
            [&result](const RecognitionResult::Instance& instance) {
                return std::find(
                    instance.faceIds.begin(), instance.faceIds.end(),
                    result.floorFaceId) != instance.faceIds.end();
            }));
        if (containingInstances > 1) {
            result.status = ManualRecognitionEditStatus::AmbiguousFloor;
            return result;
        }
    }

    RecognitionResult candidate = m_session.result();
    // 匹配选中面命中的实例:记录下标、收集涉及面,并生成修改后的副本。
    const std::set<std::string> selected(
        selectedFaceIds.begin(), selectedFaceIds.end());
    std::vector<std::size_t> matchedIndices;
    std::set<std::string> facesInMatchedInstances;
    std::vector<RecognitionResult::Instance> affectedInstances;
    for (std::size_t index = 0;
         index < candidate.instances.size(); ++index) {
        const RecognitionResult::Instance& instance =
            candidate.instances[index];
        const bool matched = std::any_of(
            instance.faceIds.begin(), instance.faceIds.end(),
            [&selected](const std::string& faceId) {
                return selected.contains(faceId);
            });
        if (!matched)
            continue;
        matchedIndices.push_back(index);
        facesInMatchedInstances.insert(
            instance.faceIds.begin(), instance.faceIds.end());
        RecognitionResult::Instance changed = instance;
        changed.kind = kind;
        changed.manual = true;
        changed.floorFaceId = result.clearingKind
            ? result.floorFaceId : std::string{};
        affectedInstances.push_back(std::move(changed));
    }
    for (auto iterator = matchedIndices.rbegin();
         iterator != matchedIndices.rend(); ++iterator) {
        candidate.instances.erase(
            candidate.instances.begin() + *iterator);
    }

    std::vector<std::string> unrecognized;
    for (const std::string& faceId : selectedFaceIds) {
        if (!facesInMatchedInstances.contains(faceId))
            unrecognized.push_back(faceId);
    }

    if (result.clearingKind) {
        std::set<std::string> createdKeys;
        for (const std::string& seedFaceId : unrecognized) {
            toolpath::ToolpathDiagnostic diagnostic;
            std::vector<std::string> expanded =
                toolpath::PocketRegionBuilder::expandManualFloorSeedDetailed(
                    seedFaceId, model, &diagnostic);
            if (expanded.empty()) {
                expanded.push_back(seedFaceId);
                result.warnings.push_back({
                    ManualRecognitionWarningKind::FloorExpansionFailed,
                    seedFaceId, {},
                    diagnostic.message.empty()
                        ? QCoreApplication::translate("OpenAutoPath",
                              toolpath::toolpathDiagnosticDefaultMessageZh(
                                  diagnostic.code)).toStdString()
                        : diagnostic.message});
            }
            if (!createdKeys.insert(instanceKey(expanded)).second)
                continue;
            const std::set<std::string> expandedSet(
                expanded.begin(), expanded.end());
            const auto conflict = std::find_if(
                candidate.instances.begin(), candidate.instances.end(),
                [&expandedSet](
                    const RecognitionResult::Instance& instance) {
                    return !instance.floorFaceId.empty() &&
                        expandedSet.contains(instance.floorFaceId);
                });
            if (conflict != candidate.instances.end()) {
                result.warnings.push_back({
                    ManualRecognitionWarningKind::ConflictingFloor,
                    seedFaceId, conflict->floorFaceId, {}});
                continue;
            }
            removeFacesFromInstances(candidate, expandedSet);
            RecognitionResult::Instance created;
            created.kind = kind;
            created.faceIds = std::move(expanded);
            created.manual = true;
            created.floorFaceId = seedFaceId;
            affectedInstances.push_back(std::move(created));
        }
    } else if (!unrecognized.empty()) {
        RecognitionResult::Instance created;
        created.kind = kind;
        created.faceIds = std::move(unrecognized);
        created.manual = true;
        affectedInstances.push_back(std::move(created));
    }

    if (affectedInstances.empty()) {
        result.status = ManualRecognitionEditStatus::NoChange;
        return result;
    }
    for (const RecognitionResult::Instance& instance : affectedInstances) {
        result.affectedFaceCount +=
            static_cast<int>(instance.faceIds.size());
        candidate.instances.push_back(instance);
    }
    candidate.rebuildDerived();
    m_session.result() = std::move(candidate);
    m_session.hasResult() = true;
    invalidateManualEditDerivatives();
    result.affectedInstanceCount =
        static_cast<int>(affectedInstances.size());
    result.status = ManualRecognitionEditStatus::Updated;
    return result;
}

// 清除特征:删除所有命中选中面的识别实例(候选副本操作),
// 重建派生索引后原子写回,并失效依赖识别结果的派生状态。
ManualRecognitionEditResult
RecognitionWorkspaceCoordinator::clearFeatureSelection(
    const std::vector<std::string>& selectedFaceIds)
{
    ManualRecognitionEditResult result;
    if (selectedFaceIds.empty()) {
        result.status = ManualRecognitionEditStatus::EmptySelection;
        return result;
    }
    const std::set<std::string> selected(
        selectedFaceIds.begin(), selectedFaceIds.end());
    RecognitionResult candidate = m_session.result();
    std::set<std::string> clearedFaces;
    candidate.instances.erase(
        std::remove_if(
            candidate.instances.begin(), candidate.instances.end(),
            [&selected, &clearedFaces, &result](
                const RecognitionResult::Instance& instance) {
                const bool matched = std::any_of(
                    instance.faceIds.begin(), instance.faceIds.end(),
                    [&selected](const std::string& faceId) {
                        return selected.contains(faceId);
                    });
                if (!matched)
                    return false;
                ++result.affectedInstanceCount;
                clearedFaces.insert(
                    instance.faceIds.begin(), instance.faceIds.end());
                return true;
            }),
        candidate.instances.end());
    if (clearedFaces.empty()) {
        result.status = ManualRecognitionEditStatus::NoChange;
        return result;
    }
    candidate.rebuildDerived();
    m_session.result() = std::move(candidate);
    invalidateManualEditDerivatives();
    result.affectedFaceCount = static_cast<int>(clearedFaces.size());
    result.status = ManualRecognitionEditStatus::Updated;
    return result;
}

// 基准面识别:对每个单元求轴向包围盒,把位于单元外侧(在包围盒边界)
// 的面记为基准面,结果写入会话。
BaseFaceRecognitionResult
RecognitionWorkspaceCoordinator::recognizeBaseFaces(
    const Model& model,
    const StepIndexer& stepIndexer)
{
    BaseFaceRecognitionResult result;
    if (model.faces.empty() || model.units.empty())
        return result;
    std::unordered_set<std::string> baseFaces;
    for (const Unit& unit : model.units) {
        const TopoDS_Shape* shape =
            stepIndexer.shapeForUnit(unit.unit_id);
        if (!shape || shape->IsNull())
            continue;
        Bnd_Box unitBox = buildAxisAlignedPlanarBox(*shape);
        if (unitBox.IsVoid()) {
            BRepBndLib::AddOptimal(
                *shape, unitBox, Standard_False, Standard_False);
        }
        if (unitBox.IsVoid())
            continue;
        for (const std::string& faceId : unit.face_ids) {
            const TopoDS_Face* face = stepIndexer.faceForId(faceId);
            if (face && !face->IsNull() && isOuterFace(*face, unitBox))
                baseFaces.insert(faceId);
        }
    }
    m_session.baseFaces() = std::move(baseFaces);
    result.baseFaceCount =
        static_cast<int>(m_session.baseFaces().size());
    result.installed = true;
    return result;
}

// 失效手动编辑影响到的派生状态:孔加工输入、孔几何档案与槽区域缓存
// 都依赖识别结果,编辑后必须清空以免用过期的派生数据。
void RecognitionWorkspaceCoordinator::invalidateManualEditDerivatives()
{
    m_session.holeToolpathInputs().clear();
    m_session.hasHoleToolpathInputs() = false;
    m_session.holeGeometryProfile() = HoleGeometryRecognitionResult{};
    m_session.pocketRegionCache() = toolpath::PocketRegionWorkflowResult{};
    m_session.hasPocketRegionCache() = false;
}

// 把识别结果的特征颜色应用到场景(通过呈现协调器)。
void RecognitionWorkspaceCoordinator::applyRecognitionColors(
    SceneController& scene) const
{
    if (!m_presentation)
        return;
    m_presentation->apply(
        scene, m_session.result(),
        m_session.throughStructureFaces(),
        kThroughStructureColor);
}

bool RecognitionWorkspaceCoordinator::hasRecognitionPresentation() const noexcept
{
    return m_presentation && !m_presentation->empty();
}

// 叠加层检查图元:清空或追加面/线/包围盒,全部转发给检查呈现器。
void RecognitionWorkspaceCoordinator::clearInspection()
{
    if (m_inspectionPresenter)
        m_inspectionPresenter->clear();
}

void RecognitionWorkspaceCoordinator::appendInspectionFace(
    const TopoDS_Face& face,
    const Quantity_Color& color,
    const gp_Trsf& modelTransform)
{
    if (m_inspectionPresenter) {
        m_inspectionPresenter->appendFace(
            face, color, modelTransform);
    }
}

bool RecognitionWorkspaceCoordinator::appendWitnessLine(
    const gp_Pnt& first,
    const gp_Pnt& second,
    const Quantity_Color& color,
    double width,
    double coincidenceTolerance,
    const gp_Trsf& modelTransform)
{
    return m_inspectionPresenter &&
        m_inspectionPresenter->appendWitnessLine(
            first, second, color, width,
            coincidenceTolerance, modelTransform);
}

bool RecognitionWorkspaceCoordinator::appendInspectionBoundingBox(
    const Bnd_Box& bounds,
    const Quantity_Color& color,
    double width,
    const gp_Trsf& modelTransform)
{
    return m_inspectionPresenter &&
        m_inspectionPresenter->appendBoundingBox(
            bounds, color, width, modelTransform);
}

} // namespace oap::app
