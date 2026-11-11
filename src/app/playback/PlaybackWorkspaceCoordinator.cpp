// PlaybackWorkspaceCoordinator 实现（oap_app_playback）。
// 应用级播放边界：拥有会话、worker 与 OCCT 播放呈现，见 PlaybackWorkspaceCoordinator.h。
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include <QCoreApplication>

#include "analysis/StepIndexer.h"
#include "app/presentation/HoleStageHighlightPresenter.h"
#include "app/presentation/PlaybackPresenter.h"
#include "app/presentation/ToolpathPreviewPresenter.h"
#include "app/view/HolePreviewGeometryBuilder.h"
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "app/workflows/playback/PlaybackProgramAssembler.h"
#include "app/workflows/playback/ToolpathPlaybackController.h"
#include "toolpath/hole/HoleStageWallHighlightResolver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_set>
#include <utility>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <QDebug>
#include <QTimer>

namespace oap::app {
namespace {

// 播放轨迹分页常量:每页移动数(帧派生按页推进)、每批大小(合并阈值)、
// 重绘最小间隔与预览段预算。
constexpr int kPlaybackTrailPageMoveCount = 256;
constexpr int kPlaybackTrailBatchSize = 1;
constexpr qint64 kPlaybackRedrawMinIntervalMs = 33;
constexpr int kPocketPreviewCutMoveBudget = 50000;
constexpr std::size_t kHolePreviewSegmentBudget = 50000;
// 呈现线宽/点大小/透明度与容差常量。
constexpr double kToolpathCutLineWidth = 1.5;
constexpr double kToolpathOutlineWidth = 1.6;
constexpr double kToolpathRapidLineWidth = 1.0;
constexpr double kToolpathEntryRetractLineWidth = 1.0;
constexpr double kHolePathEndpointPointSize = 6.0;
constexpr double kHoleWallHighlightTransparency = 0.35;
constexpr double kHolePathHighlightWidth = 4.5;
constexpr double kHolePathHighlightOutlineWidth = 8.0;
constexpr double kDisplaySegmentTolerance = 1.0e-9;

// 播放/预览各段类型的呈现颜色:快速蓝、进退刀黄、槽切削绿、
// 轮廓黑、孔路径绿、孔路径起/终点红/紫、孔阶段高亮黄。
const Quantity_Color kColorToolpathRapid(
    0.15, 0.45, 1.0, Quantity_TOC_sRGB);
const Quantity_Color kColorToolpathEntryRetract(
    1.0, 0.85, 0.15, Quantity_TOC_sRGB);
const Quantity_Color kColorPocketPreviewCut(
    0.0, 0.85, 0.20, Quantity_TOC_sRGB);
const Quantity_Color kColorPocketPreviewOutline(
    0.0, 0.0, 0.0, Quantity_TOC_sRGB);
const Quantity_Color kColorHoleRoute(
    0.0, 0.85, 0.20, Quantity_TOC_sRGB);
const Quantity_Color kColorHolePathStart(
    1.0, 0.05, 0.05, Quantity_TOC_sRGB);
const Quantity_Color kColorHolePathEnd(
    0.65, 0.12, 0.90, Quantity_TOC_sRGB);
const Quantity_Color kColorHolePathHighlight(
    1.0, 0.95, 0.10, Quantity_TOC_sRGB);

// 是否为孔类特征。
bool isHoleKind(FeatureKind kind) noexcept
{
    return kind == FeatureKind::ThroughHole ||
        kind == FeatureKind::BlindHole ||
        kind == FeatureKind::CountersinkHole;
}

// 移动类型 → 预览段类型:进刀/退刀归为竖直段,其余直译。
appdetail::HolePreviewSegmentKind previewSegmentKind(
    toolpath::ToolpathMoveType type) noexcept
{
    switch (type) {
        case toolpath::ToolpathMoveType::Rapid:
            return appdetail::HolePreviewSegmentKind::Rapid;
        case toolpath::ToolpathMoveType::Plunge:
        case toolpath::ToolpathMoveType::Retract:
            return appdetail::HolePreviewSegmentKind::Vertical;
        case toolpath::ToolpathMoveType::Cut:
            return appdetail::HolePreviewSegmentKind::Cut;
    }
    return appdetail::HolePreviewSegmentKind::Cut;
}

std::string previewFeatureName(FeatureKind kind)
{
    switch (kind) {
        case FeatureKind::ThroughHole: return QCoreApplication::translate("OpenAutoPath", "通孔").toStdString();
        case FeatureKind::BlindHole: return QCoreApplication::translate("OpenAutoPath", "盲孔").toStdString();
        case FeatureKind::CountersinkHole: return QCoreApplication::translate("OpenAutoPath", "沉孔").toStdString();
        case FeatureKind::Slot: return QCoreApplication::translate("OpenAutoPath", "槽").toStdString();
        case FeatureKind::Pocket: return QCoreApplication::translate("OpenAutoPath", "腔").toStdString();
    }
    return QCoreApplication::translate("OpenAutoPath", "特征").toStdString();
}

bool appendPreviewSegment(
    std::vector<presentation::ToolpathPreviewSegment>& segments,
    const toolpath::ToolpathMove& move,
    presentation::ToolpathSegmentIdentity identity,
    int& failureCount)
{
    if (!std::isfinite(move.from.X()) || !std::isfinite(move.from.Y()) ||
        !std::isfinite(move.from.Z()) || !std::isfinite(move.to.X()) ||
        !std::isfinite(move.to.Y()) || !std::isfinite(move.to.Z()) ||
        move.from.Distance(move.to) < kDisplaySegmentTolerance) {
        ++failureCount;
        return false;
    }
    identity.loopSequence = move.loopSequence;
    identity.depthIndex = move.depthIndex;
    identity.ring = move.ring;
    identity.feed = move.feed;
    if (identity.commandName.empty()) {
        identity.commandName =
            move.type == toolpath::ToolpathMoveType::Rapid ? "G0" : "G1";
    }
    segments.push_back({
        move.from, move.to, previewSegmentKind(move.type),
        std::move(identity)});
    return true;
}

void applyPrimitiveIdentity(
    presentation::ToolpathSegmentIdentity& identity,
    const toolpath::ToolpathMoveSemantic& semantic)
{
    identity.primitiveIndex = semantic.primitiveIndex;
    identity.primitiveMoveIndex = semantic.primitiveMoveIndex;
    identity.primitiveMoveCount = semantic.primitiveMoveCount;
    identity.commandName = semantic.commandName;
    identity.commandSummary = semantic.commandSummary;
}

void applyToolIdentity(
    presentation::ToolpathSegmentIdentity& identity,
    const toolpath::ToolpathToolContext& tool)
{
    if (!tool.valid)
        return;
    identity.toolNumber = tool.toolNumber;
    identity.toolName = tool.toolName;
    identity.toolType = tool.toolType;
}

std::vector<presentation::ToolpathPreviewSegment>
resolvePreviewOverlaps(
    const std::vector<presentation::ToolpathPreviewSegment>& source)
{
    std::vector<appdetail::HolePreviewSegment> geometry;
    geometry.reserve(source.size());
    for (std::size_t index = 0; index < source.size(); ++index) {
        const auto& segment = source[index];
        geometry.push_back({
            segment.from, segment.to, segment.kind, index});
    }
    geometry = appdetail::resolveHolePreviewColorOverlaps(geometry);

    std::vector<presentation::ToolpathPreviewSegment> resolved;
    resolved.reserve(geometry.size());
    for (const appdetail::HolePreviewSegment& fragment : geometry) {
        if (fragment.sourceIndex >= source.size())
            continue;
        presentation::ToolpathPreviewSegment segment =
            source[fragment.sourceIndex];
        segment.from = fragment.from;
        segment.to = fragment.to;
        segment.kind = fragment.kind;
        resolved.push_back(std::move(segment));
    }
    return resolved;
}

// 构建刀路预览样式:孔预览开启起点/终点标记与外框化快速线,并使用
// 指定的 OCCT Z 图层。
presentation::ToolpathPreviewStyle previewStyle(
    bool holePreview, int zLayer)
{
    presentation::ToolpathPreviewStyle style;
    style.cutColor = kColorPocketPreviewCut;
    style.rapidColor = holePreview
        ? kColorHoleRoute : kColorToolpathRapid;
    style.verticalColor = kColorToolpathEntryRetract;
    style.outlineColor = kColorPocketPreviewOutline;
    style.startColor = kColorHolePathStart;
    style.endColor = kColorHolePathEnd;
    style.cutWidth = kToolpathCutLineWidth;
    style.rapidWidth = kToolpathRapidLineWidth;
    style.verticalWidth = kToolpathEntryRetractLineWidth;
    style.outlineWidth = kToolpathOutlineWidth;
    style.endpointSize = kHolePathEndpointPointSize;
    style.zLayer = zLayer;
    style.outlineRapid = holePreview;
    style.showEndpoints = holePreview;
    return style;
}

// 把单个移动追加为复合形状中的边:先做有限性与最短长度门禁,
// 再构造边,失败时返回 false 并跳过。
bool addMoveToCompound(
    const toolpath::ToolpathMove& move,
    BRep_Builder& builder,
    TopoDS_Compound& compound)
{
    const gp_Pnt& from = move.from;
    const gp_Pnt& to = move.to;
    if (!std::isfinite(from.X()) || !std::isfinite(from.Y()) ||
        !std::isfinite(from.Z()) || !std::isfinite(to.X()) ||
        !std::isfinite(to.Y()) || !std::isfinite(to.Z()) ||
        from.Distance(to) < kDisplaySegmentTolerance) {
        return false;
    }
    try {
        BRepBuilderAPI_MakeEdge edge(from, to);
        if (!edge.IsDone())
            return false;
        builder.Add(compound, edge.Edge());
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

} // namespace

// 构造函数:创建后台播放控制器与各呈现器,并把后台帧信号以队列方式
// 接回主线程统一处理。
PlaybackWorkspaceCoordinator::PlaybackWorkspaceCoordinator(
    presentation::OverlayController& overlays,
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view,
    QObject* parent)
    : QObject(parent),
      m_overlays(&overlays),
      m_controller(std::make_unique<ToolpathPlaybackController>()),
      m_presenter(std::make_unique<presentation::PlaybackPresenter>(
          overlays, context, view)),
      m_previewPresenter(
          std::make_unique<presentation::ToolpathPreviewPresenter>(overlays)),
      m_holeStageHighlightPresenter(
          std::make_unique<presentation::HoleStageHighlightPresenter>(
              overlays)),
      m_view(view)
{
    connect(
        m_controller.get(), &ToolpathPlaybackController::frameReady,
        this, &PlaybackWorkspaceCoordinator::handleWorkerFrame,
        Qt::QueuedConnection);
}

// 析构:同步取消并等待后台播放任务。
PlaybackWorkspaceCoordinator::~PlaybackWorkspaceCoordinator()
{
    cancelAndWait();
}

void PlaybackWorkspaceCoordinator::bindViewer(
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view)
{
    m_view = view;
    if (m_presenter)
        m_presenter->bindViewer(context, view);
}

std::pair<int, int> PlaybackWorkspaceCoordinator::toolBatchRange(
    int toolBatchIndex) const
{
    return m_session.toolBatchRange(toolBatchIndex);
}

// 替换播放程序:先清空呈现与当前刀具预览,由装配器把程序写入会话,
// 再把播放移动集交给后台控制器(替换其数据源)。
bool PlaybackWorkspaceCoordinator::replaceProgram(
    const toolpath::ToolpathProgram& program,
    const PlaybackProgramBuildOptions& options)
{
    m_presenter->clear();
    clearCurrentToolpathPreview();
    m_hiddenOperationIds.clear();
    m_soloOperationId.clear();
    PlaybackProgramAssembler::replaceProgram(m_session, program, options);

    std::vector<toolpath::ToolpathMove> moves;
    moves.reserve(m_session.moves().size());
    for (const PlaybackMove& playback : m_session.moves())
        moves.push_back(playback.move);
    return m_controller && m_controller->setMoves(std::move(moves));
}

// 重载版本：以通用口袋显示预算和默认配色调用主版本。
bool PlaybackWorkspaceCoordinator::replaceProgram(
    const toolpath::ToolpathProgram& program,
    bool limitPocketCutMoves)
{
    PlaybackProgramBuildOptions options;
    options.limitPocketCutMoves = limitPocketCutMoves;
    options.previewCutMoveBudget = kPocketPreviewCutMoveBudget;
    options.pocketRapidColor = kColorToolpathRapid;
    options.holeRapidColor = kColorHoleRoute;
    options.verticalColor = kColorToolpathEntryRetract;
    return replaceProgram(program, options);
}

// 清除程序:清空呈现与静态刀路预览,重置会话程序与后台控制器。
void PlaybackWorkspaceCoordinator::clearProgram()
{
    m_presenter->clear();
    clearStaticToolpathPresentation();
    m_session.clearProgram();
    m_hiddenOperationIds.clear();
    m_soloOperationId.clear();
    if (m_controller)
        m_controller->clear();
}

// 清除播放呈现:清空叠加层并作废后台请求,会话请求状态复位。
void PlaybackWorkspaceCoordinator::clearPresentation()
{
    m_presenter->clear();
    if (m_controller)
        m_controller->invalidateRequests();
    m_session.resetRequestState();
}

// 清除完整/当前刀路预览与孔阶段高亮(按叠加层组分别清空)。
void PlaybackWorkspaceCoordinator::clearFullToolpathPreview()
{
    if (m_previewPresenter) {
        m_previewPresenter->clear(
            presentation::OverlayGroup::Toolpath);
    }
    m_staticToolpathFirstIndex = 0;
    m_staticToolpathCount = -1;
}

void PlaybackWorkspaceCoordinator::clearCurrentToolpathPreview()
{
    if (m_previewPresenter) {
        m_previewPresenter->clear(
            presentation::OverlayGroup::CurrentToolpath);
    }
}

void PlaybackWorkspaceCoordinator::clearHoleStageHighlight()
{
    if (m_holeStageHighlightPresenter)
        m_holeStageHighlightPresenter->clear();
}

// 清除全部静态刀路呈现并复位当前刀具批次。
void PlaybackWorkspaceCoordinator::clearStaticToolpathPresentation()
{
    clearFullToolpathPreview();
    clearCurrentToolpathPreview();
    clearHoleStageHighlight();
    resetCurrentToolBatch();
}

// 追加槽/腔预览段：切削移动超预算时按跨步抽样，再把带工序身份的
// 轻量线段追加到完整刀路预览组。
ToolpathPreviewBuildStats
PlaybackWorkspaceCoordinator::appendPocketPreview(
    const PocketPreviewEntry& entry,
    const gp_Trsf& modelTransform)
{
    ToolpathPreviewBuildStats stats;
    const auto& moves = entry.operation.clearing.moves;
    stats.sourceSegmentCount = moves.size();

    int totalCutMoves = 0;
    for (const toolpath::ToolpathMove& move : moves) {
        if (move.type == toolpath::ToolpathMoveType::Cut)
            ++totalCutMoves;
    }
    // 任意口袋模式超预算时，按 ceil(总切削/预算) 作为抽样跨步。
    const int cutStride = totalCutMoves > kPocketPreviewCutMoveBudget
        ? std::max(2, static_cast<int>(std::ceil(
              static_cast<double>(totalCutMoves) /
              kPocketPreviewCutMoveBudget)))
        : 1;
    const std::vector<toolpath::ToolpathMoveSemantic> semantics =
        cutStride > 1
            ? std::vector<toolpath::ToolpathMoveSemantic>{}
            : PlaybackProgramAssembler::mapMoveSemantics(
                  moves, entry.operation.clearing.primitives);

    std::vector<presentation::ToolpathPreviewSegment> segments;
    const std::size_t nonCutMoveCount =
        moves.size() - static_cast<std::size_t>(totalCutMoves);
    const std::size_t sampledCutMoveCount = cutStride > 1
        ? (static_cast<std::size_t>(totalCutMoves) +
           static_cast<std::size_t>(cutStride) - 1u) /
              static_cast<std::size_t>(cutStride)
        : static_cast<std::size_t>(totalCutMoves);
    segments.reserve(nonCutMoveCount + sampledCutMoveCount);
    int cutOrdinal = 0;
    const int displayBase = m_previewPresenter
        ? m_previewPresenter->segmentCount(
              presentation::OverlayGroup::Toolpath)
        : 0;
    for (int moveIndex = 0;
         moveIndex < static_cast<int>(moves.size()); ++moveIndex) {
        const toolpath::ToolpathMove& move =
            moves[static_cast<std::size_t>(moveIndex)];
        if (move.type == toolpath::ToolpathMoveType::Cut &&
            (cutOrdinal++ % cutStride) != 0) {
            continue;
        }
        presentation::ToolpathSegmentIdentity identity;
        identity.moveIndex = displayBase + static_cast<int>(segments.size());
        identity.operationMoveIndex = moveIndex;
        identity.sourceIndex = entry.operation.sourceRegionIndex;
        identity.setupIndex = entry.operation.setupIndex;
        identity.operationId = entry.operation.operationId;
        identity.featureName = previewFeatureName(entry.operation.kind);
        if (!semantics.empty()) {
            applyPrimitiveIdentity(
                identity,
                semantics[static_cast<std::size_t>(moveIndex)]);
        } else {
            identity.commandName =
                move.type == toolpath::ToolpathMoveType::Rapid
                    ? "G0" : "G1";
        }
        applyToolIdentity(identity, entry.operation.tool);
        appendPreviewSegment(
            segments, move, std::move(identity),
            stats.failedSegmentCount);
    }

    stats.displayedSegmentCount = segments.size();
    if (segments.size() < moves.size()) {
        qWarning().noquote()
            << "[OpenAutoPath] decimated pocket preview segments:"
            << moves.size() << "->" << segments.size();
    }
    if (stats.failedSegmentCount > 0) {
        qWarning().noquote()
            << "[OpenAutoPath] skipped invalid pocket preview segments:"
            << stats.failedSegmentCount;
    }
    if (segments.empty() || !m_previewPresenter)
        return stats;

    presentation::ToolpathPreviewStyle style =
        previewStyle(false, displayZLayer());
    style.startColor = kColorPocketPreviewCut;
    style.endColor = kColorPocketPreviewCut;
    style.outlineRapid = false;
    style.showEndpoints = false;
    stats.objectCount = m_previewPresenter->append(
        presentation::OverlayGroup::Toolpath,
        segments, {}, modelTransform, style).objectCount;
    m_previewPresenter->setDisplayRange(
        presentation::OverlayGroup::Toolpath,
        m_staticToolpathFirstIndex, m_staticToolpathCount,
        m_staticToolpathShowNodes);
    m_previewPresenter->setOperationFilter(
        presentation::OverlayGroup::Toolpath,
        m_hiddenOperationIds, m_soloOperationId);
    return stats;
}

ToolpathPreviewBuildStats
PlaybackWorkspaceCoordinator::replaceProgramPreview(
    const toolpath::ToolpathProgram& program,
    const gp_Trsf& modelTransform)
{
    ToolpathPreviewBuildStats stats;
    clearFullToolpathPreview();
    std::vector<presentation::ToolpathPreviewSegment> segments;
    segments.reserve(program.moveCount());
    int globalMoveIndex = 0;
    for (int operationIndex = 0;
         operationIndex < static_cast<int>(program.operations.size());
         ++operationIndex) {
        const toolpath::ToolpathOperation& operation =
            program.operations[static_cast<std::size_t>(operationIndex)];
        const auto semantics = PlaybackProgramAssembler::mapMoveSemantics(
            operation.moves, operation.primitives);
        stats.sourceSegmentCount += operation.moves.size();
        for (int moveIndex = 0;
             moveIndex < static_cast<int>(operation.moves.size());
             ++moveIndex, ++globalMoveIndex) {
            presentation::ToolpathSegmentIdentity identity;
            identity.moveIndex = globalMoveIndex;
            identity.operationMoveIndex = moveIndex;
            identity.sourceIndex = operation.sourceIndex >= 0
                ? operation.sourceIndex : operationIndex;
            identity.setupIndex = operation.setupIndex;
            identity.operationId = operation.operationId;
            if (operation.type ==
                toolpath::ToolpathOperationType::ProfileContour) {
                identity.featureName = "Profile";
            } else if (operation.type == toolpath::ToolpathOperationType::SurfaceFinishing) {
                identity.featureName = "Engraving";
            } else {
                identity.featureName =
                    previewFeatureName(operation.featureKind);
            }
            applyPrimitiveIdentity(
                identity, semantics[static_cast<std::size_t>(moveIndex)]);
            applyToolIdentity(identity, operation.tool);
            appendPreviewSegment(
                segments,
                operation.moves[static_cast<std::size_t>(moveIndex)],
                std::move(identity), stats.failedSegmentCount);
        }
    }
    stats.displayedSegmentCount = segments.size();
    if (segments.empty() || !m_previewPresenter)
        return stats;
    presentation::ToolpathPreviewStyle style =
        previewStyle(false, displayZLayer());
    style.outlineRapid = false;
    style.showEndpoints = false;
    stats.objectCount = m_previewPresenter->replace(
        presentation::OverlayGroup::Toolpath,
        segments, {}, modelTransform, style).objectCount;
    m_previewPresenter->setDisplayRange(
        presentation::OverlayGroup::Toolpath,
        m_staticToolpathFirstIndex, m_staticToolpathCount,
        m_staticToolpathShowNodes);
    return stats;
}

// 整体替换孔预览:清空后按预算采样削减全部孔段，裁剪颜色重叠且保留
// 原移动身份，并记录各装夹的入口端点。
ToolpathPreviewBuildStats
PlaybackWorkspaceCoordinator::replaceHolePreview(
    std::span<const HolePreviewEntry> entries,
    const gp_Trsf& modelTransform)
{
    ToolpathPreviewBuildStats stats;
    clearFullToolpathPreview();

    for (const HolePreviewEntry& entry : entries)
        stats.sourceSegmentCount += entry.operation.moves.size();
    // 全局采样索引表:把总段数压到预算内,同时跨装夹保留均匀覆盖。
    const std::vector<std::size_t> sampleIndices =
        appdetail::holePreviewSampleIndices(
            stats.sourceSegmentCount, kHolePreviewSegmentBudget);
    std::vector<presentation::ToolpathPreviewSegment> segments;
    segments.reserve(sampleIndices.size());
    std::size_t moveIndex = 0;
    std::size_t sampleIndex = 0;
    std::map<int, std::pair<const HolePreviewEntry*, const HolePreviewEntry*>>
        setupEnds;

    for (const HolePreviewEntry& entry : entries) {
        const std::vector<toolpath::ToolpathMoveSemantic> semantics =
            PlaybackProgramAssembler::mapMoveSemantics(
                entry.operation.moves, entry.operation.primitives);
        auto [setupIt, inserted] = setupEnds.emplace(
            entry.operation.setupIndex, std::make_pair(&entry, &entry));
        if (!inserted)
            setupIt->second.second = &entry;
        for (int operationMoveIndex = 0;
             operationMoveIndex <
                 static_cast<int>(entry.operation.moves.size());
             ++operationMoveIndex) {
            const toolpath::ToolpathMove& move =
                entry.operation.moves[
                    static_cast<std::size_t>(operationMoveIndex)];
            const bool include = sampleIndex < sampleIndices.size() &&
                moveIndex == sampleIndices[sampleIndex];
            const int originalMoveIndex = static_cast<int>(moveIndex);
            ++moveIndex;
            if (!include)
                continue;
            ++sampleIndex;
            presentation::ToolpathSegmentIdentity identity;
            identity.moveIndex = originalMoveIndex;
            identity.operationMoveIndex = operationMoveIndex;
            identity.sourceIndex = entry.sourceIndex >= 0
                ? entry.sourceIndex : entry.operation.sourceHoleIndex;
            identity.setupIndex = entry.operation.setupIndex;
            identity.operationId = entry.operation.operationId;
            identity.featureName = previewFeatureName(entry.operation.kind);
            applyPrimitiveIdentity(
                identity,
                semantics[static_cast<std::size_t>(operationMoveIndex)]);
            applyToolIdentity(identity, entry.operation.tool);
            appendPreviewSegment(
                segments, move, std::move(identity),
                stats.failedSegmentCount);
        }
    }

    segments = resolvePreviewOverlaps(segments);
    stats.displayedSegmentCount = segments.size();
    if (segments.size() < stats.sourceSegmentCount) {
        qWarning().noquote()
            << "[OpenAutoPath] decimated hole preview segments:"
            << stats.sourceSegmentCount << "->" << segments.size();
    }
    if (stats.failedSegmentCount > 0) {
        qWarning().noquote()
            << "[OpenAutoPath] skipped invalid hole preview segments:"
            << stats.failedSegmentCount;
    }

    std::vector<presentation::ToolpathPreviewEndpointPair> endpoints;
    endpoints.reserve(setupEnds.size());
    for (const auto& [setupIndex, setupEndpoints] : setupEnds) {
        (void)setupIndex;
        endpoints.push_back({
            setupEndpoints.first->operation.entry,
            setupEndpoints.second->operation.entry});
    }
    if (m_previewPresenter) {
        stats.objectCount = m_previewPresenter->replace(
            presentation::OverlayGroup::Toolpath,
            segments, endpoints, modelTransform,
            previewStyle(true, displayZLayer())).objectCount;
        m_previewPresenter->setDisplayRange(
            presentation::OverlayGroup::Toolpath,
            m_staticToolpathFirstIndex, m_staticToolpathCount,
            m_staticToolpathShowNodes);
        m_previewPresenter->setOperationFilter(
            presentation::OverlayGroup::Toolpath,
            m_hiddenOperationIds, m_soloOperationId);
    }
    return stats;
}

// 重建当前刀具批次预览:仅取当前批次范围并同样采样削减,
// 孔预览额外给出批次首尾端点。
ToolpathPreviewBuildStats
PlaybackWorkspaceCoordinator::replaceCurrentToolPreview(
    bool holePreview,
    const gp_Trsf& modelTransform)
{
    ToolpathPreviewBuildStats stats;
    clearCurrentToolpathPreview();
    const auto [start, end] = toolBatchRange(currentToolBatchIndex());
    if (start >= end)
        return stats;

    stats.sourceSegmentCount = static_cast<std::size_t>(end - start);
    const std::vector<std::size_t> sampleIndices =
        appdetail::holePreviewSampleIndices(
            stats.sourceSegmentCount, kHolePreviewSegmentBudget);
    std::vector<presentation::ToolpathPreviewSegment> segments;
    segments.reserve(sampleIndices.size());
    std::size_t sampleIndex = 0;
    for (int index = start; index < end; ++index) {
        const std::size_t sourceIndex =
            static_cast<std::size_t>(index - start);
        if (sampleIndex >= sampleIndices.size() ||
            sourceIndex != sampleIndices[sampleIndex]) {
            continue;
        }
        ++sampleIndex;
        const PlaybackMove& playback = m_session.moves()[index];
        presentation::ToolpathSegmentIdentity identity;
        identity.moveIndex = index;
        identity.operationMoveIndex = playback.operationMoveIndex;
        identity.sourceIndex = playback.sourceIndex;
        identity.setupIndex = playback.setupIndex;
        identity.toolBatchIndex = playback.toolBatchIndex;
        identity.operationId = playback.operationId;
        identity.featureName = previewFeatureName(playback.kind);
        applyPrimitiveIdentity(identity, playback.semantic);
        applyToolIdentity(identity, playback.tool);
        appendPreviewSegment(
            segments, playback.move, std::move(identity),
            stats.failedSegmentCount);
    }

    segments = resolvePreviewOverlaps(segments);
    stats.displayedSegmentCount = segments.size();
    if (segments.size() < stats.sourceSegmentCount) {
        qWarning().noquote()
            << "[OpenAutoPath] decimated current-tool preview segments:"
            << stats.sourceSegmentCount << "->" << segments.size();
    }
    if (stats.failedSegmentCount > 0) {
        qWarning().noquote()
            << "[OpenAutoPath] skipped invalid current-tool preview segments:"
            << stats.failedSegmentCount;
    }
    std::vector<presentation::ToolpathPreviewEndpointPair> endpoints;
    if (holePreview) {
        endpoints.push_back({
            m_session.moves()[start].move.from,
            m_session.moves()[end - 1].move.to});
    }
    if (m_previewPresenter) {
        stats.objectCount = m_previewPresenter->replace(
            presentation::OverlayGroup::CurrentToolpath,
            segments, endpoints, modelTransform,
            previewStyle(holePreview, displayZLayer())).objectCount;
        m_previewPresenter->setDisplayRange(
            presentation::OverlayGroup::CurrentToolpath,
            0, -1, m_staticToolpathShowNodes);
        m_previewPresenter->setOperationFilter(
            presentation::OverlayGroup::CurrentToolpath,
            m_hiddenOperationIds, m_soloOperationId);
    }
    return stats;
}

int PlaybackWorkspaceCoordinator::staticToolpathSegmentCount() const
{
    return m_previewPresenter
        ? m_previewPresenter->segmentCount(
              presentation::OverlayGroup::Toolpath)
        : 0;
}

void PlaybackWorkspaceCoordinator::setStaticToolpathDisplay(
    int firstIndex, int count, bool showNodes)
{
    m_staticToolpathFirstIndex = std::max(0, firstIndex);
    m_staticToolpathCount = count;
    m_staticToolpathShowNodes = showNodes;
    if (!m_previewPresenter)
        return;
    m_previewPresenter->setDisplayRange(
        presentation::OverlayGroup::Toolpath,
        m_staticToolpathFirstIndex, m_staticToolpathCount,
        m_staticToolpathShowNodes);
    m_previewPresenter->setDisplayRange(
        presentation::OverlayGroup::CurrentToolpath,
        0, -1, m_staticToolpathShowNodes);
}

void PlaybackWorkspaceCoordinator::setOperationVisible(
    const std::string& operationId, bool visible)
{
    if (operationId.empty())
        return;
    if (visible)
        m_hiddenOperationIds.erase(operationId);
    else
        m_hiddenOperationIds.insert(operationId);
    refreshRemainingPathOperationFilter();
    if (!m_previewPresenter)
        return;
    for (const presentation::OverlayGroup group : {
             presentation::OverlayGroup::Toolpath,
             presentation::OverlayGroup::CurrentToolpath}) {
        m_previewPresenter->setOperationFilter(
            group, m_hiddenOperationIds, m_soloOperationId);
    }
}

void PlaybackWorkspaceCoordinator::setSoloOperation(
    const std::string& operationId)
{
    m_soloOperationId = operationId;
    refreshRemainingPathOperationFilter();
    if (!m_previewPresenter)
        return;
    for (const presentation::OverlayGroup group : {
             presentation::OverlayGroup::Toolpath,
             presentation::OverlayGroup::CurrentToolpath}) {
        m_previewPresenter->setOperationFilter(
            group, m_hiddenOperationIds, m_soloOperationId);
    }
}

bool PlaybackWorkspaceCoordinator::remainingSegmentVisible(int moveIndex) const
{
    return m_presenter && m_presenter->remainingSegmentVisible(moveIndex);
}

void PlaybackWorkspaceCoordinator::refreshRemainingPathOperationFilter()
{
    if (!m_presenter || !m_presenter->hasRemainingPath())
        return;
    std::vector<bool> visibility;
    visibility.reserve(m_session.moves().size());
    for (const PlaybackMove& playback : m_session.moves())
        visibility.push_back(operationVisible(playback.operationId));
    m_presenter->setRemainingSegmentVisibility(visibility);
}

bool PlaybackWorkspaceCoordinator::operationVisible(
    const std::string& operationId) const noexcept
{
    if (!m_soloOperationId.empty())
        return operationId == m_soloOperationId;
    return operationId.empty() ||
        m_hiddenOperationIds.find(operationId) ==
            m_hiddenOperationIds.end();
}

// 面→孔源映射:在孔源中查找含该墙面的源;命中多个时视为歧义返回 -1。
int PlaybackWorkspaceCoordinator::holeStageIndexForWallFace(
    const std::string& faceId,
    std::span<const RecognizedHoleSource> sources) const
{
    if (faceId.empty())
        return -1;
    int matchedIndex = -1;
    for (int index = 0; index < static_cast<int>(sources.size()); ++index) {
        const RecognizedHoleSource& source =
            sources[static_cast<std::size_t>(index)];
        // 优先用阶段墙面列表,未解析时回退到源面列表。
        const auto& walls = source.hole.stageWallFaceIds.empty()
            ? source.sourceFaceIds : source.hole.stageWallFaceIds;
        if (std::find(walls.begin(), walls.end(), faceId) == walls.end())
            continue;
        if (matchedIndex >= 0)
            return -1;
        matchedIndex = index;
    }
    return matchedIndex;
}

// 判断某孔源是否存在可高亮路径:存在覆盖该源的孔预览条目、满足当前
// 刀具过滤,且条目内含有切削/进刀移动。
bool PlaybackWorkspaceCoordinator::hasHighlightableHoleStagePath(
    int sourceIndex,
    std::span<const HolePreviewEntry> entries) const
{
    const auto coversSource = [sourceIndex](const HolePreviewEntry& entry) {
        return entry.sourceIndex == sourceIndex ||
            std::find(
                entry.operation.coveredSourceHoleIndices.begin(),
                entry.operation.coveredSourceHoleIndices.end(),
                sourceIndex) != entry.operation.coveredSourceHoleIndices.end();
    };
    return std::any_of(
        entries.begin(), entries.end(),
        [this, &coversSource](const HolePreviewEntry& entry) {
            if (!coversSource(entry) ||
                (currentToolOnly() && currentToolBatchIndex() >= 0 &&
                 !HoleToolpathWorkbenchCoordinator::
                      previewEntryBelongsToToolBatch(
                          entry, m_session.moves(),
                          m_session.toolBatchStarts(),
                          currentToolBatchIndex()))) {
                return false;
            }
            return std::any_of(
                entry.operation.moves.begin(), entry.operation.moves.end(),
                [](const toolpath::ToolpathMove& move) {
                    return move.type == toolpath::ToolpathMoveType::Cut ||
                        move.type == toolpath::ToolpathMoveType::Plunge;
                });
        });
}

// 重建孔阶段高亮:把选中孔源对应的切削/进刀移动并入路径复合体,
// 解析出可见墙面的孔阶段墙,一并交给高亮呈现器;返回墙/工序/移动计数。
HoleStageHighlightStats
PlaybackWorkspaceCoordinator::replaceHoleStageHighlight(
    std::span<const HolePreviewEntry> entries,
    std::span<const RecognizedHoleSource> sources,
    const std::unordered_set<int>& selectedSources,
    const oap::StepIndexer& indexer,
    const std::function<bool(const std::string&)>& isFaceHidden,
    const gp_Trsf& modelTransform,
    bool visible)
{
    HoleStageHighlightStats stats;
    clearHoleStageHighlight();
    if (selectedSources.empty())
        return stats;

    try {
        BRep_Builder builder;
        TopoDS_Compound path;
        builder.MakeCompound(path);
        std::unordered_set<int> visibleSources;
        // 收集覆盖选中源且满足刀具过滤的条目的路径段。
        for (const HolePreviewEntry& entry : entries) {
            if (currentToolOnly() && currentToolBatchIndex() >= 0 &&
                !HoleToolpathWorkbenchCoordinator::
                     previewEntryBelongsToToolBatch(
                         entry, m_session.moves(),
                         m_session.toolBatchStarts(),
                         currentToolBatchIndex())) {
                continue;
            }
            std::vector<int> selectedEntrySources;
            for (const int sourceIndex : selectedSources) {
                if (entry.sourceIndex == sourceIndex ||
                    std::find(
                        entry.operation.coveredSourceHoleIndices.begin(),
                        entry.operation.coveredSourceHoleIndices.end(),
                        sourceIndex) !=
                        entry.operation.coveredSourceHoleIndices.end()) {
                    selectedEntrySources.push_back(sourceIndex);
                }
            }
            if (selectedEntrySources.empty())
                continue;

            bool operationHighlighted = false;
            for (const toolpath::ToolpathMove& move : entry.operation.moves) {
                if (move.type != toolpath::ToolpathMoveType::Cut &&
                    move.type != toolpath::ToolpathMoveType::Plunge) {
                    continue;
                }
                if (addMoveToCompound(move, builder, path)) {
                    ++stats.moveCount;
                    operationHighlighted = true;
                }
            }
            if (!operationHighlighted)
                continue;
            ++stats.operationCount;
            visibleSources.insert(
                selectedEntrySources.begin(), selectedEntrySources.end());
        }
        if (stats.moveCount == 0)
            return stats;

        std::vector<std::size_t> visibleSourceIndices;
        visibleSourceIndices.reserve(visibleSources.size());
        for (int sourceIndex = 0;
             sourceIndex < static_cast<int>(sources.size()); ++sourceIndex) {
            if (visibleSources.contains(sourceIndex)) {
                visibleSourceIndices.push_back(
                    static_cast<std::size_t>(sourceIndex));
            }
        }
        std::vector<toolpath::HoleMachiningInput> holes;
        holes.reserve(sources.size());
        for (const RecognizedHoleSource& source : sources)
            holes.push_back(source.hole);
        // 按可见孔源解析应高亮的孔阶段墙面,并过滤隐藏/缺失的面。
        const std::vector<std::string> wallFaceIds =
            toolpath::resolveHoleStageWallHighlightFaceIds(
                holes, visibleSourceIndices);

        BRep_Builder wallBuilder;
        TopoDS_Compound walls;
        wallBuilder.MakeCompound(walls);
        for (const std::string& faceId : wallFaceIds) {
            if (isFaceHidden && isFaceHidden(faceId))
                continue;
            const TopoDS_Face* face = indexer.faceForId(faceId);
            if (!face || face->IsNull())
                continue;
            wallBuilder.Add(walls, *face);
            ++stats.wallCount;
        }

        // 统一以高亮黄呈现路径与墙,透明墙体 + 描边路径。
        presentation::HoleStageHighlightStyle style;
        style.wallColor = kColorHolePathHighlight;
        style.pathColor = kColorHolePathHighlight;
        style.outlineColor = kColorPocketPreviewOutline;
        style.wallTransparency = kHoleWallHighlightTransparency;
        style.pathWidth = kHolePathHighlightWidth;
        style.outlineWidth = kHolePathHighlightOutlineWidth;
        style.zLayer = displayZLayer();
        const TopoDS_Shape noWalls;
        if (!m_holeStageHighlightPresenter ||
            !m_holeStageHighlightPresenter->replace(
                stats.wallCount > 0
                    ? static_cast<const TopoDS_Shape&>(walls) : noWalls,
                path, modelTransform, style, visible)) {
            clearHoleStageHighlight();
            return {};
        }
    } catch (const Standard_Failure& error) {
        clearHoleStageHighlight();
        qWarning().noquote()
            << "[OpenAutoPath] failed to rebuild hole-stage highlight:"
            << error.GetMessageString();
        return {};
    }
    return stats;
}

// 重置播放:清空呈现与请求状态,复位当前刀具批次,并恢复起始叠加层。
void PlaybackWorkspaceCoordinator::resetPlayback()
{
    clearPresentation();
    m_session.currentToolBatchIndex() = -1;
    clearCurrentToolpathPreview();
    applyStaticOverlayVisibility(0, 1.0);
}

void PlaybackWorkspaceCoordinator::requestCancel() noexcept
{
    if (m_controller)
        m_controller->requestCancel();
}

void PlaybackWorkspaceCoordinator::cancelAndWait()
{
    if (m_controller)
        m_controller->cancelAndWait();
}

bool PlaybackWorkspaceCoordinator::running() const noexcept
{
    return m_controller && m_controller->running();
}

void PlaybackWorkspaceCoordinator::setDisplayLayer(int layer) noexcept
{
    m_session.displayLayer() = std::clamp(layer, 0, 2);
}

// 会话内显示层(0..2)换算为 OCCT Z 图层。
int PlaybackWorkspaceCoordinator::displayZLayer() const noexcept
{
    switch (m_session.displayLayer()) {
        case 1:
            return Graphic3d_ZLayerId_Top;
        case 2:
            return Graphic3d_ZLayerId_Topmost;
        default:
            return Graphic3d_ZLayerId_Default;
    }
}

// 把显示层应用到播放呈现(刀路图层)。
void PlaybackWorkspaceCoordinator::applyDisplayLayer()
{
    m_presenter->applyToolpathDisplayLayer(displayZLayer());
}

void PlaybackWorkspaceCoordinator::applyModelTransform(
    const gp_Trsf& transform)
{
    m_presenter->applyModelTransform(transform);
}

// 更新当前刀具过滤器:关闭过滤时清除批次并重置轨迹范围;
// 开启时按移动所在批次更新批次号(force 可强制重建),并返回变化类型。
PlaybackToolFilterChange
PlaybackWorkspaceCoordinator::updateCurrentToolFilter(
    int moveIndex, bool force)
{
    if (!m_session.currentToolOnly()) {
        if (!force && m_session.currentToolBatchIndex() < 0)
            return {};
        m_session.currentToolBatchIndex() = -1;
        resetTrailRange(0);
        return {PlaybackToolFilterChangeKind::Cleared, -1};
    }

    if (moveIndex < 0 || moveIndex >= moveCount())
        return {};
    const int batchIndex = m_session.moves()[moveIndex].toolBatchIndex;
    if (batchIndex < 0 ||
        (!force && batchIndex == m_session.currentToolBatchIndex())) {
        return {};
    }

    m_session.currentToolBatchIndex() = batchIndex;
    resetTrailRange(toolBatchRange(batchIndex).first);
    return {PlaybackToolFilterChangeKind::RebuildBatch, batchIndex};
}

// 更新过滤器并同步呈现:清除时清空当前刀具预览并显示完整路径;
// 重建时重新生成批次预览,失败时回退为显示完整路径。
PlaybackToolFilterChange
PlaybackWorkspaceCoordinator::updateCurrentToolFilterPresentation(
    int moveIndex,
    bool force,
    bool holePreview,
    const gp_Trsf& modelTransform)
{
    const PlaybackToolFilterChange change =
        updateCurrentToolFilter(moveIndex, force);
    switch (change.kind) {
        case PlaybackToolFilterChangeKind::None:
            break;
        case PlaybackToolFilterChangeKind::Cleared:
            clearCurrentToolpathPreview();
            if (m_overlays) {
                m_overlays->setVisible(
                    presentation::OverlayGroup::Toolpath, true);
            }
            break;
        case PlaybackToolFilterChangeKind::RebuildBatch:
            {
                const ToolpathPreviewBuildStats stats =
                    replaceCurrentToolPreview(
                        holePreview, modelTransform);
                if (m_overlays) {
                    m_overlays->setVisible(
                        presentation::OverlayGroup::Toolpath,
                        stats.objectCount == 0);
                }
            }
            break;
    }
    return change;
}

// 计算给定播放位置的叠加层可见性:静态叠加层由会话规则决定;
// 当前刀具预览若失败或为空则回退到完整路径(避免起始帧空白),
// 剩余路径按当前/批次范围计算。
PlaybackOverlayVisibilityState
PlaybackWorkspaceCoordinator::visibilityForPosition(
    int index, double fraction) const noexcept
{
    PlaybackOverlayVisibilityState result;
    result.staticOverlays =
        presentation::playbackStaticOverlayVisibility(
            !m_session.moves().empty(), index, fraction,
            m_session.currentToolOnly());
    if (result.staticOverlays.currentToolpath &&
        (!m_overlays ||
         m_overlays->size(
             presentation::OverlayGroup::CurrentToolpath) == 0)) {
        // 失败或为空的当前批次预览绝不能使起始帧空白。在播放具备可显示的
        // 剩余路径呈现之前,回退到完整的静态路径。
        result.staticOverlays.currentToolpath = false;
        result.staticOverlays.fullToolpath = true;
    }

    const int total = moveCount();
    const bool atUnplayedStart =
        index <= 0 && fraction >= 1.0;
    if (m_session.moves().empty() || atUnplayedStart)
        return result;

    // 剩余路径范围:默认从当前步到末尾;当前刀具过滤时限制在本批次内。
    int first = std::clamp(index, 0, total);
    int end = total;
    if (m_session.currentToolOnly() && first < total) {
        const int batchIndex = m_session.moves()[first].toolBatchIndex;
        const auto [batchStart, batchEnd] = toolBatchRange(batchIndex);
        first = std::max(first, batchStart);
        end = batchEnd;
    }
    result.remainingPath = first < end;
    return result;
}

bool PlaybackWorkspaceCoordinator::staticOverlaysVisible(
    int index, double fraction) const noexcept
{
    return visibilityForPosition(index, fraction)
        .staticOverlays.holeStageHighlight;
}

void PlaybackWorkspaceCoordinator::applyStaticOverlayVisibility(
    int index, double fraction)
{
    m_presenter->setStaticOverlayVisibility(
        visibilityForPosition(index, fraction).staticOverlays);
}

// 发布播放位置:应用静态叠加层可见性,更新剩余路径,并按需请求光标帧。
void PlaybackWorkspaceCoordinator::publishPosition(
    int index, double fraction,
    const gp_Trsf& modelTransform,
    bool requestCursorFrame)
{
    const PlaybackOverlayVisibilityState visibility =
        visibilityForPosition(index, fraction);
    m_presenter->setStaticOverlayVisibility(visibility.staticOverlays);
    updateRemainingPath(
        index, fraction, modelTransform, visibility.remainingPath);
    if (requestCursorFrame)
        requestFrame(index, fraction);
}

bool PlaybackWorkspaceCoordinator::shouldRenderFrame(
    int index, bool panelPlaying,
    bool modalOrPopup, bool mouseDragging)
{
    return m_session.shouldRenderFrame(
        index, panelPlaying, modalOrPopup, mouseDragging);
}

bool PlaybackWorkspaceCoordinator::shouldFollowCamera(
    bool panelPlaying, double speedMultiplier)
{
    return m_session.shouldFollowCamera(panelPlaying, speedMultiplier);
}

// 合并重绘请求:会话按最小间隔决定延迟,到时后完成请求并触发重绘。
void PlaybackWorkspaceCoordinator::scheduleRedraw()
{
    if (m_view.IsNull())
        return;
    const int delayMs = m_session.beginRedrawRequest(
        kPlaybackRedrawMinIntervalMs);
    if (delayMs < 0)
        return;
    QTimer::singleShot(delayMs, this, [this]() {
        m_session.completeRedrawRequest();
        if (!m_view.IsNull())
            m_view->Redraw();
    });
}

void PlaybackWorkspaceCoordinator::updateTipMarker(
    const gp_Pnt& tip,
    double toolDiameter,
    const gp_Trsf& modelTransform,
    const Quantity_Color& color,
    double pointSize)
{
    m_presenter->updateTipMarker(
        tip, toolDiameter, modelTransform, color, pointSize);
}

void PlaybackWorkspaceCoordinator::clearCursor()
{
    m_presenter->clearCursor();
}

// 更新剩余路径:不可见或空程序时隐藏;否则懒构建全量剩余段列表,
// 再按当前/批次范围与进度比例裁剪后交给呈现器。
void PlaybackWorkspaceCoordinator::updateRemainingPath(
    int index, double fraction,
    const gp_Trsf& modelTransform,
    bool visible)
{
    if (!visible || m_session.moves().empty()) {
        m_presenter->setRemainingPathVisible(false);
        return;
    }

    std::vector<presentation::PlaybackRemainingSegment> segments;
    // 段列表只在首次显示时构建一次(颜色/样式按移动类型固定),之后复用。
    if (!m_presenter->hasRemainingPath()) {
        segments.reserve(m_session.moves().size());
        for (const PlaybackMove& playback : m_session.moves()) {
            presentation::PlaybackRemainingSegment segment;
            segment.from = playback.move.from;
            segment.to = playback.move.to;
            segment.color = playback.color;
            segment.visible = operationVisible(playback.operationId);
            switch (playback.move.type) {
                case toolpath::ToolpathMoveType::Cut:
                    segment.style =
                        presentation::PlaybackRemainingLineStyle::Cut;
                    segment.outlined = true;
                    break;
                case toolpath::ToolpathMoveType::Rapid:
                    segment.style =
                        presentation::PlaybackRemainingLineStyle::Rapid;
                    segment.outlined = isHoleKind(playback.kind);
                    break;
                case toolpath::ToolpathMoveType::Plunge:
                case toolpath::ToolpathMoveType::Retract:
                    segment.style = presentation::
                        PlaybackRemainingLineStyle::EntryRetract;
                    break;
            }
            segments.push_back(std::move(segment));
        }
    }

    const int total = moveCount();
    // 当前刀具过滤时,剩余范围限制在本批次内。
    int first = std::clamp(index, 0, total);
    int end = total;
    if (m_session.currentToolOnly() && first < total) {
        const int batchIndex = m_session.moves()[first].toolBatchIndex;
        const auto [batchStart, batchEnd] = toolBatchRange(batchIndex);
        first = std::max(first, batchStart);
        end = batchEnd;
    }
    m_presenter->updateRemainingPath(
        std::move(segments), first, end, fraction,
        modelTransform, displayZLayer());
}

// 重置轨迹范围:只清理临时播放对象并作废后台请求(静态路径可能仍
// 隐藏,剩余路径呈现予以保留),会话请求状态从 startIndex 重新开始。
void PlaybackWorkspaceCoordinator::resetTrailRange(int startIndex)
{
    // 静态路径可能已被隐藏,因此只重置临时播放对象,
    // 保留剩余路径的呈现。
    m_presenter->clearCursorAndTrail();
    if (m_controller)
        m_controller->invalidateRequests();
    m_session.resetRequestState(startIndex);
}

// 请求单帧光标:会话先做去重/倒带决策;倒带时清理轨迹并作废后台
// 请求,可派发时转给后台控制器。
void PlaybackWorkspaceCoordinator::requestFrame(
    int index, double fraction)
{
    if (!m_controller || m_session.moves().empty())
        return;

    const PlaybackSession::CursorDecision decision =
        m_session.requestCursor(index, fraction);
    if (decision.rewound) {
        m_presenter->clearCursorAndTrail();
        m_controller->invalidateRequests();
    }
    if (decision.dispatch) {
        dispatchCursorRequest(
            decision.dispatch->index, decision.dispatch->fraction);
    }
}

// 轨迹帧分页推进:把目标步按页(256 步)切分,支持合并与挂起语义——
// 增量不足一批或已有在飞请求时记入 pending,待上一页完成后继续;
// force 忽略批量合并直接派发下一页。
void PlaybackWorkspaceCoordinator::requestTrailFlush(
    int toIndex, bool force)
{
    const int total = moveCount();
    const int desiredTarget = std::clamp(
        std::max(toIndex, m_session.trailPendingTo()), 0, total);
    if (desiredTarget <= m_session.trailFlushIndex())
        return;

    if (!force &&
        desiredTarget - m_session.trailFlushIndex() <
            kPlaybackTrailBatchSize) {
        m_session.trailPending() = true;
        m_session.trailPendingTo() = std::max(
            m_session.trailPendingTo(), desiredTarget);
        return;
    }

    if (m_session.trailInFlight()) {
        m_session.trailPending() = true;
        m_session.trailPendingForce() =
            m_session.trailPendingForce() || force;
        m_session.trailPendingTo() = std::max(
            m_session.trailPendingTo(), desiredTarget);
        return;
    }

    const int pageStart = m_session.trailFlushIndex();
    const int pageLimit = std::min(
        total, pageStart + kPlaybackTrailPageMoveCount);
    const int dispatchTarget = std::min(desiredTarget, pageLimit);
    if (dispatchTarget <= m_session.trailFlushIndex())
        return;

    // 本页未覆盖目标时挂起剩余部分,否则视为全部完成。
    if (dispatchTarget < desiredTarget) {
        m_session.trailPending() = true;
        m_session.trailPendingForce() = force;
        m_session.trailPendingTo() = desiredTarget;
    } else {
        m_session.trailPending() = false;
        m_session.trailPendingForce() = false;
        m_session.trailPendingTo() = 0;
    }
    dispatchTrailRequest(m_session.trailFlushIndex(), dispatchTarget);
}

// 派发光标请求:失败时在会话侧标记派发失败。
void PlaybackWorkspaceCoordinator::dispatchCursorRequest(
    int index, double fraction)
{
    if (!m_controller ||
        !m_controller->requestCursor(index, fraction)) {
        m_session.cursorDispatchFailed();
    }
}

// 派发轨迹请求:先置在飞标记,提交失败则复位。
void PlaybackWorkspaceCoordinator::dispatchTrailRequest(
    int fromIndex, int toIndex)
{
    if (!m_controller || toIndex <= fromIndex)
        return;
    m_session.trailInFlight() = true;
    if (!m_controller->requestTrail(fromIndex, toIndex))
        m_session.trailInFlight() = false;
}

// 消费后台帧:光标帧转发光标并继续下一个排队的请求;轨迹帧推进
// 刷指针并处理挂起的下一页(形成分页流水线)。
void PlaybackWorkspaceCoordinator::handleWorkerFrame(
    const ToolpathPlaybackFrame& frame)
{
    if (frame.kind == ToolpathPlaybackFrameKind::Cursor) {
        emit frameReady(frame);
        const auto next = m_session.completeCursorRequest();
        if (next)
            dispatchCursorRequest(next->index, next->fraction);
        return;
    }

    m_session.trailInFlight() = false;
    m_session.trailFlushIndex() = std::max(
        m_session.trailFlushIndex(), frame.toIndex);
    emit frameReady(frame);
    if (!m_session.trailPending())
        return;

    const int target = m_session.trailPendingTo();
    const bool force = m_session.trailPendingForce();
    m_session.trailPending() = false;
    m_session.trailPendingForce() = false;
    m_session.trailPendingTo() = 0;
    requestTrailFlush(target, force);
}

} // namespace oap::app
