#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <utility>
#include <span>

namespace oap::app::pocket_cam_detail {
QString PocketCamOpenGLWidget::gpuDeviceName()
{
    if (!m_gpuDexelReady) return {};
    makeCurrent();
    const auto name = QString::fromLatin1(reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
    doneCurrent();
    return name;
}

double PocketCamOpenGLWidget::displayedGpuVolume()
{
    if (!m_gpuDexelReady) return 0.0;
    makeCurrent();
    const auto cells = static_cast<std::size_t>(m_dexelNx) * m_dexelNy;
    std::vector<QVector2D> intervals(cells * m_dexelMaximumIntervals);
    std::vector<GLuint> counts(cells);
    m_gl43->glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    m_gl43->glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_dexelIntervalsBuffer);
    m_gl43->glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, intervals.size() * sizeof(QVector2D), intervals.data());
    m_gl43->glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_dexelCountsBuffer);
    m_gl43->glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, counts.size() * sizeof(GLuint), counts.data());
    m_gl43->glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    doneCurrent();
    double volume = 0;
    for (int j = 0; j < m_dexelNy; ++j) for (int i = 0; i < m_dexelNx; ++i) {
        const auto cell = static_cast<std::size_t>(j) * m_dexelNx + i;
        const double du = std::min(double(m_dexelResolution), double(m_dexelGridMaximum.x()) - m_dexelGridMinimum.x() - i * double(m_dexelResolution));
        const double dv = std::min(double(m_dexelResolution), double(m_dexelGridMaximum.y()) - m_dexelGridMinimum.y() - j * double(m_dexelResolution));
        for (GLuint k = 0; k < std::min(counts[cell], GLuint(m_dexelMaximumIntervals)); ++k) {
            const auto interval = intervals[cell * m_dexelMaximumIntervals + k];
            volume += std::max(0.0, double(interval.y() - interval.x())) * du * dv;
        }
    }
    return volume;
}

void PocketCamOpenGLWidget::destroyAnalyticBoundarySnapshots()
{
    if (m_gl43) {
        for (const AnalyticBoundarySnapshot& snapshot :
             m_analyticBoundarySnapshots) {
            if (snapshot.buffer != 0u)
                m_gl43->glDeleteBuffers(1, &snapshot.buffer);
        }
    }
    m_analyticBoundarySnapshots.clear();
    m_analyticBoundarySnapshotNx = 0;
    m_analyticBoundarySnapshotNy = 0;
}

bool PocketCamOpenGLWidget::cacheAnalyticBoundarySnapshot(int operationIndex)
{
    if (!m_gl43 || operationIndex < 0 ||
        m_dexelBoundaryDistanceBuffer == 0u ||
        m_dexelNx <= 0 || m_dexelNy <= 0) {
        return false;
    }
    const GLsizeiptr bytes = static_cast<GLsizeiptr>(
        static_cast<std::size_t>(m_dexelNx) *
        static_cast<std::size_t>(m_dexelNy) *
        sizeof(float));
    auto existing = std::find_if(
        m_analyticBoundarySnapshots.begin(),
        m_analyticBoundarySnapshots.end(),
        [&](const AnalyticBoundarySnapshot& snapshot) {
            return snapshot.operationIndex == operationIndex;
        });
    if (existing == m_analyticBoundarySnapshots.end()) {
        m_analyticBoundarySnapshots.push_back(
            {operationIndex, 0u});
        existing = std::prev(
            m_analyticBoundarySnapshots.end());
        m_gl43->glGenBuffers(1, &existing->buffer);
    }
    m_gl43->glBindBuffer(
        GL_COPY_WRITE_BUFFER, existing->buffer);
    m_gl43->glBufferData(
        GL_COPY_WRITE_BUFFER, bytes,
        nullptr, GL_DYNAMIC_COPY);
    m_gl43->glBindBuffer(
        GL_COPY_READ_BUFFER,
        m_dexelBoundaryDistanceBuffer);
    m_gl43->glCopyBufferSubData(
        GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER,
        0, 0, bytes);
    m_gl43->glBindBuffer(GL_COPY_READ_BUFFER, 0u);
    m_gl43->glBindBuffer(GL_COPY_WRITE_BUFFER, 0u);
    m_analyticBoundarySnapshotNx = m_dexelNx;
    m_analyticBoundarySnapshotNy = m_dexelNy;

    // 每份 800 万单元缓存约 32 MiB；保留最近三个工序边界，
    // 其余回退仍可安全使用 CPU Multi-Dexel 基线。
    constexpr std::size_t maximumSnapshots = 3u;
    while (m_analyticBoundarySnapshots.size() >
           maximumSnapshots) {
        const GLuint buffer =
            m_analyticBoundarySnapshots.front().buffer;
        if (buffer != 0u)
            m_gl43->glDeleteBuffers(1, &buffer);
        m_analyticBoundarySnapshots.erase(
            m_analyticBoundarySnapshots.begin());
    }
    return checkOpenGlStage(
        QStringLiteral("解析边界工序检查点缓存"));
}

bool PocketCamOpenGLWidget::restoreAnalyticBoundarySnapshot(int operationIndex)
{
    if (!m_gl43 || operationIndex < 0 ||
        m_dexelBoundaryDistanceBuffer == 0u ||
        m_analyticBoundarySnapshotNx != m_dexelNx ||
        m_analyticBoundarySnapshotNy != m_dexelNy) {
        return false;
    }
    const auto snapshot = std::find_if(
        m_analyticBoundarySnapshots.begin(),
        m_analyticBoundarySnapshots.end(),
        [&](const AnalyticBoundarySnapshot& candidate) {
            return candidate.operationIndex == operationIndex;
        });
    if (snapshot == m_analyticBoundarySnapshots.end() ||
        snapshot->buffer == 0u) {
        return false;
    }
    const GLsizeiptr bytes = static_cast<GLsizeiptr>(
        static_cast<std::size_t>(m_dexelNx) *
        static_cast<std::size_t>(m_dexelNy) *
        sizeof(float));
    m_gl43->glBindBuffer(
        GL_COPY_READ_BUFFER, snapshot->buffer);
    m_gl43->glBindBuffer(
        GL_COPY_WRITE_BUFFER,
        m_dexelBoundaryDistanceBuffer);
    m_gl43->glCopyBufferSubData(
        GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER,
        0, 0, bytes);
    m_gl43->glBindBuffer(GL_COPY_READ_BUFFER, 0u);
    m_gl43->glBindBuffer(GL_COPY_WRITE_BUFFER, 0u);
    m_gl43->glMemoryBarrier(
        GL_SHADER_STORAGE_BARRIER_BIT);
    return checkOpenGlStage(
        QStringLiteral("解析边界工序检查点恢复"));
}

void PocketCamOpenGLWidget::destroyGpuDexelState()
{
    if (m_gl43) {
        if (m_dexelIntervalsBuffer != 0u)
            m_gl43->glDeleteBuffers(1, &m_dexelIntervalsBuffer);
        if (m_dexelCountsBuffer != 0u)
            m_gl43->glDeleteBuffers(1, &m_dexelCountsBuffer);
        if (m_dexelBoundaryDistanceBuffer != 0u) {
            m_gl43->glDeleteBuffers(
                1, &m_dexelBoundaryDistanceBuffer);
        }
        if (m_dexelStockDomainBuffer != 0u) {
            m_gl43->glDeleteBuffers(
                1, &m_dexelStockDomainBuffer);
        }
        if (m_dexelCoarseMaskBuffer != 0u)
            m_gl43->glDeleteBuffers(1, &m_dexelCoarseMaskBuffer);
        if (m_dexelCoarseEntriesBuffer != 0u)
            m_gl43->glDeleteBuffers(1, &m_dexelCoarseEntriesBuffer);
        if (m_dexelFineEntriesBuffer != 0u)
            m_gl43->glDeleteBuffers(1, &m_dexelFineEntriesBuffer);
        if (m_dexelFineDrawCommandBuffer != 0u) {
            m_gl43->glDeleteBuffers(
                1, &m_dexelFineDrawCommandBuffer);
        }
        if (m_dexelVao != 0u)
            m_gl43->glDeleteVertexArrays(1, &m_dexelVao);
    }
    m_dexelIntervalsBuffer = 0u;
    m_dexelCountsBuffer = 0u;
    m_dexelBoundaryDistanceBuffer = 0u;
    m_dexelStockDomainBuffer = 0u;
    m_dexelCoarseMaskBuffer = 0u;
    m_dexelCoarseEntriesBuffer = 0u;
    m_dexelFineEntriesBuffer = 0u;
    m_dexelFineDrawCommandBuffer = 0u;
    m_dexelBufferBytes.fill(0);
    m_dexelVao = 0u;
    m_dexelFineCapacity = 0u;
    m_gpuDexelSurfaceDirty = false;
    m_gpuDexelReady = false;
    m_gpuDexelAppliedStep = 0u;
    m_gpuDexelSourceIntervalCount = 0u;
    m_gpuDexelOccupiedCellCount = 0u;
}

bool PocketCamOpenGLWidget::uploadPlaybackStock(
    const toolpath::MultiDexelStock& stock, std::size_t appliedStep,
    const Handle(Poly_Triangulation)& preparedMesh)
{
    if (!m_usePlaybackMesh && uploadGpuDexelStock(stock, appliedStep))
        return true;
    if (!m_scene.authoritativeIpwPlayback || !stock.valid())
        return false;

    if (!m_usePlaybackMesh)
        m_playbackDisplayWarning = m_error;
    destroyGpuDexelState();
    m_error.clear();
    // Meshes are display artifacts only. Removal, audit and subsequent frames
    // continue to use the original full-resolution immutable checkpoint.
    Handle(Poly_Triangulation) triangulation = preparedMesh;
    std::string error;
    const auto meshOptions = IpwDisplayMeshController::meshOptionsForQuality(1, stock.resolution());
    if (triangulation.IsNull() && stock.intervalCount() > 0 && !stock.buildDisplayMesh(
            triangulation, nullptr, &error, {}, meshOptions)) {
        m_error = QStringLiteral("IPW 兼容显示网格生成失败：%1（GPU：%2）")
            .arg(QString::fromStdString(error), m_playbackDisplayWarning);
        return false;
    }
    RenderMesh mesh;
    if (!triangulation.IsNull()) {
        mesh = meshTriangulation(triangulation, m_error);
        if (!m_error.isEmpty()) return false;
    }
    beginOpenGlStage(QStringLiteral("IPW 兼容网格上传"));
    GpuMesh replacement;
    uploadMesh(mesh, replacement);
    if (!checkOpenGlStage(QStringLiteral("IPW 兼容网格上传"))) {
        destroyGpuMesh(replacement);
        return false;
    }
    destroyGpuMesh(m_playbackStockGpu);
    m_playbackStockGpu = replacement;
    m_usePlaybackMesh = true;
    return true;
}

bool PocketCamOpenGLWidget::uploadGpuDexelStock(
    const toolpath::MultiDexelStock& stock,
    std::size_t appliedStep)
{
    if (!m_gpuDexelSupported || !m_gl43 ||
        !stock.valid() ||
        (!m_scene.authoritativeIpwPlayback && !stock.axisAligned(m_scene.outwardToolAxis))) {
        m_error = QStringLiteral("GPU 不支持、材料无效或材料轴与播放轴不一致");
        return false;
    }
    const int nx = stock.nx();
    const int ny = stock.ny();
    if (nx <= 0 || ny <= 0) {
        m_error = QStringLiteral("IPW 网格尺寸无效：%1 × %2").arg(nx).arg(ny);
        return false;
    }
    const std::size_t cellCount =
        static_cast<std::size_t>(nx) *
        static_cast<std::size_t>(ny);
    if (cellCount == 0u || cellCount > 8'000'000u) {
        m_error = QStringLiteral("IPW 网格 %1 × %2 超过 GPU 的 800 万单元预算").arg(nx).arg(ny);
        return false;
    }
    if (!m_analyticBoundarySnapshots.empty() &&
        (m_analyticBoundarySnapshotNx != nx ||
         m_analyticBoundarySnapshotNy != ny)) {
        destroyAnalyticBoundarySnapshots();
    }

    beginOpenGlStage(
        QStringLiteral("GPU Multi-Dexel SSBO 上传"));

    int maximumIntervals = 0;
    std::size_t sourceIntervalCount = 0u;
    std::size_t occupiedCellCount = 0u;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const std::size_t intervalCount =
                stock.cellIntervalsView(i, j).size();
            maximumIntervals = std::max(
                maximumIntervals,
                static_cast<int>(intervalCount));
            sourceIntervalCount += intervalCount;
            occupiedCellCount += intervalCount > 0u ? 1u : 0u;
        }
    }
    // An empty checkpoint is valid (all material removed). Keep one unused
    // slot per cell so that zero counts render an empty material surface.
    maximumIntervals = std::max(1, maximumIntervals);
    constexpr std::size_t maximumIntervalSlots = 64'000'000u;
    if (cellCount > maximumIntervalSlots /
            static_cast<std::size_t>(maximumIntervals)) {
        m_error = QStringLiteral("IPW 网格 %1 × %2、每单元最多 %3 个区间，超过 GPU 区间预算")
            .arg(nx).arg(ny).arg(maximumIntervals);
        return false;
    }

    GLint64 maximumBlockBytes = 0;
    m_gl43->glGetInteger64v(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &maximumBlockBytes);
    const std::size_t intervalBytes = cellCount *
        static_cast<std::size_t>(maximumIntervals) * sizeof(QVector2D);
    if (maximumBlockBytes <= 0 || intervalBytes > static_cast<std::size_t>(maximumBlockBytes)) {
        m_error = QStringLiteral("IPW 区间缓冲需要 %1 MiB，显卡单个 SSBO 上限为 %2 MiB")
            .arg(intervalBytes / (1024.0 * 1024.0), 0, 'f', 1)
            .arg(maximumBlockBytes / (1024.0 * 1024.0), 0, 'f', 1);
        return false;
    }

    std::vector<QVector2D> intervals(
        cellCount * static_cast<std::size_t>(maximumIntervals));
    std::vector<std::uint32_t> counts(cellCount, 0u);
    std::vector<float> boundaryDistance(
        cellCount, static_cast<float>(stock.resolution()));
    std::vector<std::uint32_t> stockDomain(
        cellCount, 0u);
    const toolpath::MultiDexelStock* domainStock = &stock;
    if (m_initialGpuStock &&
        m_initialGpuStock->nx() == nx &&
        m_initialGpuStock->ny() == ny &&
        std::abs(
            m_initialGpuStock->resolution() -
            stock.resolution()) <= 1.0e-9) {
        domainStock = m_initialGpuStock.get();
    }
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const std::size_t cell =
                static_cast<std::size_t>(j) *
                    static_cast<std::size_t>(nx) +
                static_cast<std::size_t>(i);
            const std::span<const toolpath::DexelInterval> source =
                stock.cellIntervalsView(i, j);
            stockDomain[cell] =
                domainStock->cellIntervalsView(i, j).empty()
                    ? 0u : 1u;
            counts[cell] = static_cast<std::uint32_t>(source.size());
            if (source.empty()) {
                boundaryDistance[cell] =
                    -static_cast<float>(stock.resolution());
            }
            for (std::size_t intervalIndex = 0u;
                 intervalIndex < source.size(); ++intervalIndex) {
                intervals[
                    cell * static_cast<std::size_t>(maximumIntervals) +
                    intervalIndex] = QVector2D(
                        static_cast<float>(source[intervalIndex].lower),
                        static_cast<float>(source[intervalIndex].upper));
            }
        }
    }

    // 为导入毛坯和工序检查点重建一个两单元宽的初始窄带。
    // 区间仍决定材料拓扑；这里的距离只决定零等值线在相邻
    // Dexel 中心之间的位置，因此不会改变体积或审计结果。
    const float dexelResolution =
        static_cast<float>(stock.resolution());
    const float narrowBand = 2.0f * dexelResolution;
    auto occupiedAt = [&](int i, int j) {
        if (i < 0 || j < 0 || i >= nx || j >= ny)
            return false;
        const std::size_t cell =
            static_cast<std::size_t>(j) *
                static_cast<std::size_t>(nx) +
            static_cast<std::size_t>(i);
        return counts[cell] > 0u;
    };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const std::size_t cell =
                static_cast<std::size_t>(j) *
                    static_cast<std::size_t>(nx) +
                static_cast<std::size_t>(i);
            const bool material = counts[cell] > 0u;
            float closestBoundary = narrowBand;
            for (int dj = -2; dj <= 2; ++dj) {
                for (int di = -2; di <= 2; ++di) {
                    if (di == 0 && dj == 0)
                        continue;
                    if (occupiedAt(i + di, j + dj) == material)
                        continue;
                    const float centerDistance = std::sqrt(
                        static_cast<float>(di * di + dj * dj));
                    const float edgeDistance =
                        std::max(0.5f, centerDistance - 0.5f) *
                        dexelResolution;
                    closestBoundary = std::min(
                        closestBoundary, edgeDistance);
                }
            }
            boundaryDistance[cell] = material
                ? closestBoundary : -closestBoundary;
        }
    }

    // Reuse storage for same-size authoritative frames and rewind uploads.
    // Allocation changes are limited to actual grid/capacity changes.
    m_gpuDexelReady = false;
    m_gl43->glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    const auto upload = [&](GLuint& buffer, int index, GLsizeiptr bytes, const void* contents) {
        if (!buffer) m_gl43->glGenBuffers(1, &buffer);
        m_gl43->glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer);
        if (m_dexelBufferBytes[index] != bytes) {
            m_gl43->glBufferData(GL_SHADER_STORAGE_BUFFER, bytes, contents, GL_DYNAMIC_DRAW);
            m_dexelBufferBytes[index] = bytes;
        } else if (contents) {
            m_gl43->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, bytes, contents);
        }
    };
    upload(m_dexelIntervalsBuffer, 0, static_cast<GLsizeiptr>(intervals.size()*sizeof(QVector2D)), intervals.data());
    upload(m_dexelCountsBuffer, 1, static_cast<GLsizeiptr>(counts.size()*sizeof(std::uint32_t)), counts.data());
    upload(m_dexelBoundaryDistanceBuffer, 2, static_cast<GLsizeiptr>(boundaryDistance.size()*sizeof(float)), boundaryDistance.data());
    upload(m_dexelStockDomainBuffer, 3, static_cast<GLsizeiptr>(stockDomain.size()*sizeof(std::uint32_t)), stockDomain.data());
    if (!m_dexelVao) m_gl43->glGenVertexArrays(1, &m_dexelVao);

    m_dexelNx = nx;
    m_dexelNy = ny;
    m_dexelMaximumIntervals = maximumIntervals;
    m_dexelResolution = static_cast<float>(stock.resolution());
    m_dexelGridMinimum = QVector2D(
        static_cast<float>(stock.uMin()),
        static_cast<float>(stock.vMin()));
    m_dexelGridMaximum = QVector2D(
        static_cast<float>(stock.uMax()),
        static_cast<float>(stock.vMax()));
    m_dexelOriginPoint = stock.origin();
    m_dexelUAxisVector = stock.uAxis();
    m_dexelVAxisVector = stock.vAxis();
    m_dexelWAxisVector = stock.wAxis();
    m_dexelOrigin = vector3(stock.origin());
    m_dexelUAxis = vector3(stock.uAxis());
    m_dexelVAxis = vector3(stock.vAxis());
    m_dexelWAxis = vector3(stock.wAxis());

    m_dexelRenderStride = 1;
    constexpr std::size_t maximumRenderedEntries = 500000u;
    if (static_cast<std::size_t>(maximumIntervals) >
        maximumRenderedEntries) {
        return false;
    }
    while (
        static_cast<std::size_t>(
            (nx + m_dexelRenderStride - 1) /
            m_dexelRenderStride) *
            static_cast<std::size_t>(
                (ny + m_dexelRenderStride - 1) /
                m_dexelRenderStride) *
            static_cast<std::size_t>(maximumIntervals) >
        maximumRenderedEntries) {
        ++m_dexelRenderStride;
    }
    m_dexelRenderNx =
        (nx + m_dexelRenderStride - 1) /
        m_dexelRenderStride;
    m_dexelRenderNy =
        (ny + m_dexelRenderStride - 1) /
        m_dexelRenderStride;
    const std::size_t coarseEntryCount =
        static_cast<std::size_t>(m_dexelRenderNx) *
        static_cast<std::size_t>(m_dexelRenderNy) *
        static_cast<std::size_t>(maximumIntervals);
    constexpr std::size_t maximumFineEntries = 16'000'000u;
    const std::size_t fineCapacity = std::min(
        cellCount * static_cast<std::size_t>(maximumIntervals),
        maximumFineEntries);
    if (coarseEntryCount == 0u || fineCapacity == 0u ||
        coarseEntryCount > static_cast<std::size_t>(
            std::numeric_limits<GLsizei>::max())) {
        destroyGpuDexelState();
        return false;
    }
    upload(m_dexelCoarseMaskBuffer, 4, static_cast<GLsizeiptr>(coarseEntryCount*sizeof(std::uint32_t)), nullptr);
    upload(m_dexelFineEntriesBuffer, 5, static_cast<GLsizeiptr>(fineCapacity*sizeof(std::uint32_t)), nullptr);
    // Fine indirect command + diagnostics, followed by the coarse command.
    const std::array<std::uint32_t, 10> drawCommand{
        150u, 0u, 0u, 0u, 0u, 0u, 12u, 0u, 0u, 0u};
    upload(m_dexelFineDrawCommandBuffer, 6, sizeof(drawCommand), drawCommand.data());
    upload(m_dexelCoarseEntriesBuffer, 7, static_cast<GLsizeiptr>(coarseEntryCount*sizeof(std::uint32_t)), nullptr);
    m_gl43->glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0u);
    m_dexelFineCapacity =
        static_cast<std::uint32_t>(fineCapacity);
    if (!checkOpenGlStage(
            QStringLiteral("GPU Multi-Dexel SSBO 上传"))) {
        destroyGpuDexelState();
        return false;
    }
    m_gpuDexelAppliedStep = std::min(
        appliedStep, m_scene.totalSimulationSteps);
    m_gpuDexelSourceIntervalCount = sourceIntervalCount;
    m_gpuDexelOccupiedCellCount = occupiedCellCount;
    m_gpuDexelSurfaceDirty = true;
    m_dexelComputeUniformsDirty = true;
    m_dexelSurfaceUniformsDirty = true;
    m_dexelRenderUniformsDirty = true;
    m_dexelComputeTool = -1;
    m_gpuDexelReady = true;
    return true;
}

QVector3D PocketCamOpenGLWidget::gpuDexelCoordinates(const gp_Pnt& point) const
{
    const gp_Vec relative(m_dexelOriginPoint, point);
    return QVector3D(
        static_cast<float>(relative.Dot(m_dexelUAxisVector)),
        static_cast<float>(relative.Dot(m_dexelVAxisVector)),
        static_cast<float>(relative.Dot(m_dexelWAxisVector)));
}

} // namespace oap::app::pocket_cam_detail
