#include "app/ui/dialogs/detail/PocketCamVulkanWidget.h"
#include <QCoreApplication>
#include <algorithm>
#include <cmath>
#include <limits>

namespace oap::app::pocket_cam_detail {
PocketCamVulkanWidget::PocketCamVulkanWidget(toolpath::PocketCamSimulationScene scene,
    RenderMesh target, PocketCamInteractionOptions interaction,
    std::shared_ptr<const toolpath::MultiDexelStock> initial,
    std::shared_ptr<const toolpath::MultiDexelStock> finalStock,
    std::vector<std::shared_ptr<const IpwTransientFrameSource>> sources,
    QVector3D minimum, QVector3D maximum, bool boundsValid, QWidget* parent)
    : QRhiWidget(nullptr), PocketCamView(interaction), m_scene(std::move(scene)),
      m_target(std::move(target)), m_initial(std::move(initial)), m_final(std::move(finalStock)),
      m_sources(std::move(sources)), m_controller(std::make_unique<IpwDisplayMeshController>())
{
    // Configure before joining a native widget hierarchy. An OCCT parent already
    // has a HWND; attaching first can create the top-level surface for Qt's
    // default API (D3D11 on Windows), before setApi(Vulkan) takes effect.
    setApi(QRhiWidget::Api::Vulkan);
    setParent(parent);
    setMinimumSize(240, 160);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);
    connect(this, &QRhiWidget::renderFailed, this, [this] {
        m_renderReady = false;
        m_error = QCoreApplication::translate("OpenAutoPath", "Vulkan 设备或窗口初始化失败，请使用 OpenGL 兼容显示。");
    });
    connect(m_controller.get(), &IpwDisplayMeshController::resultReady,
        this, [this](const IpwDisplayMeshResult& result) { acceptFrame(result); });
    if (!boundsValid) {
        minimum = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        maximum = -minimum;
        for (const auto& motion : m_scene.motions) {
            for (const auto& point : {vector3(motion.move.from), vector3(motion.move.to)}) {
                for (int axis = 0; axis < 3; ++axis) {
                    minimum[axis] = std::min(minimum[axis], point[axis]);
                    maximum[axis] = std::max(maximum[axis], point[axis]);
                }
            }
        }
        if (m_scene.motions.empty()) { minimum = {-1, -1, -1}; maximum = {1, 1, 1}; }
    }
    m_center = (minimum + maximum) * 0.5f;
    m_radius = std::max(0.001f, (maximum - minimum).length() * 0.5f);
    resetCamera();
    // Resolve original paths once. A simplified display motion can cover many
    // source moves; distance-based lookup preserves the source cutting geometry.
    m_sourcePaths.resize(m_sources.size());
    for (std::size_t i = 0; i < m_sources.size(); ++i) {
        const auto& source = m_sources[i];
        if (!source || !source->operations || !source->parentStock || !source->parentStock->valid()) continue;
        auto offset = source->completedMoveCount;
        for (const auto& operation : *source->operations) {
            if (offset >= operation.clearing.moves.size()) {
                offset -= operation.clearing.moves.size();
                continue;
            }
            if (offset != 0) break; // Sources must start at an operation boundary.
            auto& path = m_sourcePaths[i];
            path.operation = &operation;
            const auto operationIndex = static_cast<std::size_t>(&operation - source->operations->data());
            auto preparedSource = std::make_shared<IpwTransientFrameSource>(*source);
            const auto params = ipwReplayParameters(*source, operationIndex);
            if (preparedSource->operationParameters.size() != preparedSource->operations->size())
                preparedSource->operationParameters.assign(preparedSource->operations->size(), preparedSource->params);
            preparedSource->operationParameters[operationIndex] = params;
            m_sources[i] = std::move(preparedSource);
            path.distances.push_back(0.0);
            for (const auto& move : operation.clearing.moves)
                path.distances.push_back(path.distances.back() + move.from.Distance(move.to));
            break;
        }
    }
    const int slices = m_scene.quality < 3.0 ? 12 : (m_scene.quality < 7.0 ? 32 : 64);
    for (const auto& tool : m_scene.tools) m_tools.push_back(makeRevolvedTool(tool.profile, slices));
    for (const auto& motion : m_scene.motions) {
        m_motionEnds.push_back(motion.firstSimulationStep + static_cast<std::size_t>(motion.simulationStepCount));
        const float rapid = motion.move.type == toolpath::ToolpathMoveType::Rapid ? 1.0f : 0.0f;
        // Path attributes share the mesh vertex layout: step and rapid replace normals.
        m_path.vertices.push_back({vector3(motion.move.from), {static_cast<float>(motion.firstSimulationStep), rapid, 0}});
        m_path.indices.push_back(static_cast<std::uint32_t>(m_path.indices.size()));
        m_path.vertices.push_back({vector3(motion.move.to), {static_cast<float>(m_motionEnds.back()), rapid, 0}});
        m_path.indices.push_back(static_cast<std::uint32_t>(m_path.indices.size()));
    }
    if (!m_scene.motions.empty()) m_tip = m_scene.motions.front().move.from;
    updateTool();
    m_gpuPlayback = !m_scene.authoritativeIpwPlayback && m_initial && m_initial->valid()
        && m_initial->axisAligned(m_scene.outwardToolAxis) && !m_scene.tools.empty();
    for (const auto& motion : m_scene.motions) {
        const auto axis = motion.outwardToolAxis.SquareMagnitude() > 1.0e-12 ? motion.outwardToolAxis : m_scene.outwardToolAxis;
        if (!m_initial || !m_initial->axisAligned(axis) || motion.toolIndex < 0 ||
            static_cast<std::size_t>(motion.toolIndex) >= m_scene.tools.size()) m_gpuPlayback = false;
    }
    if (m_gpuPlayback) m_materialReady = true;
    else requestFrame(0);
}

void PocketCamVulkanWidget::setCurrentStep(std::size_t step)
{
    step = std::min(step, totalSteps());
    if (m_gpuPlayback) {
        m_step = step;
        if (!m_scene.motions.empty()) {
            const auto end = std::lower_bound(m_motionEnds.begin(), m_motionEnds.end(), step);
            const auto index = std::min(static_cast<std::size_t>(end - m_motionEnds.begin()), m_scene.motions.size() - 1);
            const auto& motion = m_scene.motions[index];
            m_tip = motion.playbackPointAt(double(step - motion.firstSimulationStep) / motion.simulationStepCount);
        }
        updateTool();
        update();
        return;
    }
    if (m_pending && step == m_requestedStep) return;
    if (m_materialReady && step == m_step) {
        m_controller->cancel(true);
        m_pending = false;
        return;
    }
    requestFrame(step);
}

void PocketCamVulkanWidget::requestFrame(std::size_t step)
{
    m_controller->cancel(true);
    m_pending = false;
    if (!m_initial || !m_initial->valid()) {
        m_error = QCoreApplication::translate("OpenAutoPath", "Vulkan 仿真缺少有效的初始 IPW。");
        return;
    }
    IpwDisplayMeshRequest request;
    request.displayMode = m_gpuSurface ? IpwOcctDisplayMode::Off : IpwOcctDisplayMode::SurfaceShaded;
    m_requestedOperation = -1;
    m_requestedMoves = 0;
    m_requestedTip = m_scene.motions.empty() ? gp_Pnt() : m_scene.motions.front().move.from;
    if (step == 0) request.stockSource = m_initial;
    else if (step == totalSteps() && m_final && m_final->valid()) {
        request.stockSource = m_final;
        if (!m_scene.motions.empty()) m_requestedTip = m_scene.motions.back().move.to;
    }
    else {
        const auto end = std::lower_bound(m_motionEnds.begin(), m_motionEnds.end(), step);
        if (end == m_motionEnds.end()) { m_error = QCoreApplication::translate("OpenAutoPath", "刀路播放索引无效。"); return; }
        const auto index = static_cast<std::size_t>(end - m_motionEnds.begin());
        const auto& motion = m_scene.motions[index];
        if (motion.operationIndex < 0 || static_cast<std::size_t>(motion.operationIndex) >= m_sources.size() ||
            !m_sources[motion.operationIndex]) {
            m_error = QCoreApplication::translate("OpenAutoPath", "此工序缺少原始刀路 IPW 来源，无法生成 Vulkan 材料帧。");
            return;
        }
        auto source = std::make_shared<IpwTransientFrameSource>(*m_sources[motion.operationIndex]);
        const auto& path = m_sourcePaths[motion.operationIndex];
        if (!path.operation || motion.sourceMoveEndCount == 0 || motion.sourceMoveEndCount >= path.distances.size()) {
            m_error = QCoreApplication::translate("OpenAutoPath", "原始刀路与播放索引不一致。");
            return;
        }
        std::size_t completed = motion.sourceMoveEndCount;
        m_requestedTip = path.operation->clearing.moves[completed - 1].to;
        if (step < *end) {
            const std::size_t first = index > 0 && m_scene.motions[index - 1].operationIndex == motion.operationIndex
                ? m_scene.motions[index - 1].sourceMoveEndCount : 0;
            const double fraction = static_cast<double>(step - motion.firstSimulationStep) / motion.simulationStepCount;
            const double distance = path.distances[first] + fraction * (path.distances[completed] - path.distances[first]);
            const auto next = std::upper_bound(path.distances.begin() + first, path.distances.begin() + completed + 1, distance);
            completed = std::min(static_cast<std::size_t>(next - path.distances.begin() - 1), completed - 1);
            const double length = path.distances[completed + 1] - path.distances[completed];
            source->nextMoveFraction = length > 0 ? std::clamp((distance - path.distances[completed]) / length, 0.0, 1.0) : 0.0;
            const auto& move = path.operation->clearing.moves[completed];
            m_requestedTip = interpolate(move.from, move.to, source->nextMoveFraction);
            if (source->nextMoveFraction >= 1.0) { ++completed; source->nextMoveFraction = 0.0; }
        }
        source->completedMoveCount += completed;
        if (m_completedStock && m_operation == motion.operationIndex && source->completedMoveCount >= m_materialMoves) {
            source->parentStock = m_completedStock;
            source->firstMoveCount = m_materialMoves;
        }
        m_requestedOperation = motion.operationIndex;
        m_requestedMoves = source->completedMoveCount;
        request.transientSource = std::move(source);
    }
    const auto stock = request.stockSource ? request.stockSource : request.transientSource->parentStock;
    if (!stock || !stock->valid()) { m_error = QCoreApplication::translate("OpenAutoPath", "工序 IPW 检查点无效。"); return; }
    request.meshOptions = IpwDisplayMeshController::meshOptionsForQuality(
        static_cast<int>(std::clamp(m_scene.quality, 1.0, 10.0)), stock->resolution());
    m_requestedStep = step;
    m_pending = m_controller->submit(std::move(request));
    if (!m_pending) m_error = QCoreApplication::translate("OpenAutoPath", "无法启动 Vulkan IPW 材料帧更新。");
}

void PocketCamVulkanWidget::acceptFrame(const IpwDisplayMeshResult& result)
{
    m_pending = false;
    if (result.cancelled) return;
    const bool empty = result.materialStock && result.materialStock->valid() && result.materialStock->intervalCount() == 0;
    if ((!result.ok() && !empty) || !result.materialStock || !result.materialStock->valid()) {
        m_error = QCoreApplication::translate("OpenAutoPath", "IPW 材料帧生成失败：%1").arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(result.error).toUtf8().constData()));
        return;
    }
    QString error;
    RenderMesh material;
    if (!m_gpuSurface && !empty) material = meshTriangulation(result.mesh, error);
    if (!error.isEmpty() || (!m_gpuSurface && !empty && material.empty())) {
        m_error = QCoreApplication::translate("OpenAutoPath", "IPW 网格转换失败：%1").arg(error);
        return;
    }
    // Publish tool, progress and material atomically; stale requests are rejected by the controller.
    m_material = std::move(material);
    m_stock = result.materialStock;
    m_completedStock = result.completedPrefixStock;
    m_tip = m_requestedTip;
    m_step = m_requestedStep;
    m_operation = m_requestedOperation;
    m_materialMoves = m_requestedMoves;
    m_materialReady = true;
    m_materialDirty = true;
    updateTool();
    update();
}

void PocketCamVulkanWidget::updateTool()
{
    if (m_scene.motions.empty()) { m_tool = -1; return; }
    // At an exact endpoint keep the cutter from the completed move, including tool changes.
    const auto end = std::lower_bound(m_motionEnds.begin(), m_motionEnds.end(), m_step);
    const auto index = std::min(static_cast<std::size_t>(end - m_motionEnds.begin()), m_scene.motions.size() - 1);
    const auto& motion = m_scene.motions[index];
    m_tool = motion.toolIndex;
    const auto axis = motion.outwardToolAxis.SquareMagnitude() > 1.0e-12 ? motion.outwardToolAxis : m_scene.outwardToolAxis;
    m_toolTransform = toolTransform(makeToolFrame(axis), m_tip);
}

QString PocketCamVulkanWidget::materialStatusString() const
{
    if (m_gpuPlayback) return QCoreApplication::translate("OpenAutoPath", "Vulkan GPU Multi-Dexel / TSDF｜增量去料与间接表面绘制");
    if (m_gpuSurface) return QCoreApplication::translate("OpenAutoPath", "Vulkan GPU Multi-Dexel / TSDF｜权威 IPW 材料｜%1")
        .arg(playbackBusy() ? QCoreApplication::translate("OpenAutoPath", "材料帧准备中") : QCoreApplication::translate("OpenAutoPath", "材料帧已就绪"));
    return QCoreApplication::translate("OpenAutoPath", "Vulkan｜%1｜材料由原始刀路和 IPW 检查点计算，显示采用有界网格")
        .arg(!m_error.isEmpty() ? QCoreApplication::translate("OpenAutoPath", "显示失败") : playbackBusy() ? QCoreApplication::translate("OpenAutoPath", "材料帧准备中") : QCoreApplication::translate("OpenAutoPath", "材料帧已就绪"))
        + (m_gpuFallbackReason.isEmpty() ? QString() : QCoreApplication::translate("OpenAutoPath", "｜GPU 回退：") + m_gpuFallbackReason);
}
} // namespace oap::app::pocket_cam_detail
