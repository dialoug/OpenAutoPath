#include "app/ui/dialogs/detail/PocketCamVulkanWidget.h"
#include "app/ui/dialogs/detail/PocketCamVulkanDexel.h"
#include <rhi/qrhi.h>
#include <QFile>
#include <QVector4D>
#include <cstring>
#include <limits>
#include <array>

static void initializeCamShaderResources()
{
    Q_INIT_RESOURCE(cam_scene_shaders);
}

namespace oap::app::pocket_cam_detail {
namespace {
constexpr int uniformBytes = 160;
QShader shader(const char* name)
{
    QFile file(QString::fromLatin1(name));
    return file.open(QIODevice::ReadOnly) ? QShader::fromSerialized(file.readAll()) : QShader{};
}
struct MeshBuffers {
    std::unique_ptr<QRhiBuffer> vertices, indices;
    quint32 count = 0;
};
bool upload(QRhi* rhi, QRhiResourceUpdateBatch* updates, const RenderMesh& mesh, MeshBuffers& target)
{
    target = {};
    if (mesh.empty()) return true;
    if (mesh.vertices.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) / sizeof(RenderVertex) ||
        mesh.indices.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) / sizeof(std::uint32_t)) return false;
    const int vertexBytes = static_cast<int>(mesh.vertices.size() * sizeof(RenderVertex));
    const int indexBytes = static_cast<int>(mesh.indices.size() * sizeof(std::uint32_t));
    target.vertices.reset(rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, vertexBytes));
    target.indices.reset(rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer, indexBytes));
    if (!target.vertices->create() || !target.indices->create()) { target = {}; return false; }
    updates->uploadStaticBuffer(target.vertices.get(), mesh.vertices.data());
    updates->uploadStaticBuffer(target.indices.get(), mesh.indices.data());
    target.count = static_cast<quint32>(mesh.indices.size());
    return true;
}
}

struct PocketCamVulkanWidget::Renderer {
    QRhi* rhi = nullptr;
    QRhiRenderPassDescriptor* pass = nullptr;
    std::unique_ptr<QRhiTexture> color, depth;
    std::unique_ptr<QRhiRenderPassDescriptor> scenePass;
    std::unique_ptr<QRhiTextureRenderTarget> sceneTarget;
    std::unique_ptr<QRhiSampler> colorSampler, depthSampler;
    std::unique_ptr<QRhiBuffer> postUniform;
    std::unique_ptr<QRhiShaderResourceBindings> postBindings;
    std::array<std::unique_ptr<QRhiBuffer>, 5> uniforms;
    std::array<std::unique_ptr<QRhiShaderResourceBindings>, 5> bindings;
    std::unique_ptr<QRhiGraphicsPipeline> solid, transparent, lines, hiddenLines, post;
    MeshBuffers target, material, path;
    std::vector<MeshBuffers> tools;
    bool uploadStatic = true;
    std::unique_ptr<PocketCamVulkanDexel> dexel;
};

void PocketCamVulkanWidget::RendererDeleter::operator()(Renderer* renderer) const { delete renderer; }
PocketCamVulkanWidget::~PocketCamVulkanWidget()
{
    m_controller->cancelAndWait();
    releaseResources();
}
bool PocketCamVulkanWidget::usingVulkan() const { return rhi() && rhi()->backend() == QRhi::Vulkan && m_renderReady; }
bool PocketCamVulkanWidget::usingGpuMaterial() const { return m_renderer && bool(m_renderer->dexel); }
std::size_t PocketCamVulkanWidget::gpuDispatchCount() const { return usingGpuMaterial() ? m_renderer->dexel->dispatchCount() : 0; }
QString PocketCamVulkanWidget::gpuDeviceName() const { return usingGpuMaterial() ? m_renderer->dexel->deviceName() : QString(); }
double PocketCamVulkanWidget::displayedVolume() const { return m_gpuPlayback && usingGpuMaterial() ? m_renderer->dexel->volume() : m_stock ? m_stock->volume() : 0.0; }
void PocketCamVulkanWidget::releaseResources()
{
    m_renderer.reset();
    m_renderReady = false;
    m_materialDirty = true;
}

void PocketCamVulkanWidget::initialize(QRhiCommandBuffer*)
{
    static const bool resourcesInitialized = [] { initializeCamShaderResources(); return true; }();
    Q_UNUSED(resourcesInitialized);
    // QRhiWidget can recreate its resources when reparented or its backing RHI changes.
    releaseResources();
    if (!rhi() || rhi()->backend() != QRhi::Vulkan || !renderTarget()) {
        m_error = QStringLiteral("未能创建 Vulkan 渲染设备。");
        return;
    }
    m_renderer.reset(new Renderer);
    auto& renderer = *m_renderer;
    renderer.rhi = rhi();
    renderer.pass = renderTarget()->renderPassDescriptor();
    const auto size = renderTarget()->pixelSize();
    renderer.color.reset(rhi()->newTexture(QRhiTexture::RGBA8, size, 1, QRhiTexture::RenderTarget));
    renderer.depth.reset(rhi()->newTexture(QRhiTexture::D32F, size, 1, QRhiTexture::RenderTarget));
    if (!renderer.color->create() || !renderer.depth->create()) {
        m_error = QStringLiteral("Vulkan 后处理纹理创建失败。"); return;
    }
    renderer.sceneTarget.reset(rhi()->newTextureRenderTarget(
        QRhiTextureRenderTargetDescription(QRhiColorAttachment(renderer.color.get()), renderer.depth.get())));
    renderer.scenePass.reset(renderer.sceneTarget->newCompatibleRenderPassDescriptor());
    renderer.sceneTarget->setRenderPassDescriptor(renderer.scenePass.get());
    if (!renderer.sceneTarget->create()) { m_error = QStringLiteral("Vulkan 离屏目标创建失败。"); return; }
    for (int slot = 0; slot < 5; ++slot) {
        renderer.uniforms[slot].reset(rhi()->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, uniformBytes));
        if (!renderer.uniforms[slot]->create()) { m_error = QStringLiteral("Vulkan 常量缓冲创建失败。"); return; }
        renderer.bindings[slot].reset(rhi()->newShaderResourceBindings());
        renderer.bindings[slot]->setBindings({QRhiShaderResourceBinding::uniformBuffer(
            0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage,
            renderer.uniforms[slot].get())});
        if (!renderer.bindings[slot]->create()) { m_error = QStringLiteral("Vulkan 着色器资源绑定失败。"); return; }
    }
    const auto vertex = shader(":/cam/scene.vert.qsb");
    const auto fragment = shader(":/cam/scene.frag.qsb");
    if (!vertex.isValid() || !fragment.isValid()) { m_error = QStringLiteral("Vulkan 着色器资源缺失或无效。"); return; }
    const auto pipeline = [&](bool lines, bool blended, bool hidden = false) {
        std::unique_ptr<QRhiGraphicsPipeline> result(rhi()->newGraphicsPipeline());
        result->setShaderStages({{QRhiShaderStage::Vertex, vertex}, {QRhiShaderStage::Fragment, fragment}});
        QRhiVertexInputLayout layout;
        layout.setBindings({QRhiVertexInputBinding(sizeof(RenderVertex))});
        layout.setAttributes({{0, 0, QRhiVertexInputAttribute::Float3, offsetof(RenderVertex, position)},
                              {0, 1, QRhiVertexInputAttribute::Float3, offsetof(RenderVertex, normal)}});
        result->setVertexInputLayout(layout);
        result->setTopology(lines ? QRhiGraphicsPipeline::Lines : QRhiGraphicsPipeline::Triangles);
        result->setDepthTest(true);
        result->setDepthWrite(!blended);
        result->setDepthOp(hidden ? QRhiGraphicsPipeline::Greater : QRhiGraphicsPipeline::LessOrEqual);
        result->setCullMode(lines ? QRhiGraphicsPipeline::None : QRhiGraphicsPipeline::Back);
        if (lines && rhi()->isFeatureSupported(QRhi::WideLines)) result->setLineWidth(2.0f);
        if (blended) {
            QRhiGraphicsPipeline::TargetBlend blend;
            blend.enable = true;
            blend.srcColor = QRhiGraphicsPipeline::SrcAlpha;
            blend.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
            blend.srcAlpha = QRhiGraphicsPipeline::One;
            blend.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
            result->setTargetBlends({blend});
        }
        result->setSampleCount(1);
        result->setShaderResourceBindings(renderer.bindings[0].get());
        result->setRenderPassDescriptor(renderer.scenePass.get());
        if (!result->create()) result.reset();
        return result;
    };
    renderer.solid = pipeline(false, false);
    renderer.transparent = pipeline(false, true);
    renderer.lines = pipeline(true, true);
    renderer.hiddenLines = pipeline(true, true, true);
    renderer.colorSampler.reset(rhi()->newSampler(QRhiSampler::Linear, QRhiSampler::Linear,
        QRhiSampler::None, QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
    renderer.depthSampler.reset(rhi()->newSampler(QRhiSampler::Nearest, QRhiSampler::Nearest,
        QRhiSampler::None, QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
    renderer.postUniform.reset(rhi()->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, 32));
    if (!renderer.colorSampler->create() || !renderer.depthSampler->create() || !renderer.postUniform->create()) {
        m_error = QStringLiteral("Vulkan 后处理资源创建失败。"); return;
    }
    renderer.postBindings.reset(rhi()->newShaderResourceBindings());
    renderer.postBindings->setBindings({
        QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::FragmentStage, renderer.postUniform.get()),
        QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, renderer.color.get(), renderer.colorSampler.get()),
        QRhiShaderResourceBinding::sampledTexture(2, QRhiShaderResourceBinding::FragmentStage, renderer.depth.get(), renderer.depthSampler.get())});
    if (!renderer.postBindings->create()) { m_error = QStringLiteral("Vulkan 后处理绑定失败。"); return; }
    const auto postVertex = shader(":/cam/post.vert.qsb"), postFragment = shader(":/cam/post.frag.qsb");
    if (!postVertex.isValid() || !postFragment.isValid()) {
        m_error = QStringLiteral("Vulkan 后处理着色器无效。"); return;
    }
    renderer.post.reset(rhi()->newGraphicsPipeline());
    renderer.post->setShaderStages({{QRhiShaderStage::Vertex, postVertex}, {QRhiShaderStage::Fragment, postFragment}});
    renderer.post->setShaderResourceBindings(renderer.postBindings.get());
    renderer.post->setRenderPassDescriptor(renderer.pass);
    renderer.post->setSampleCount(renderTarget()->sampleCount());
    m_renderReady = renderer.solid && renderer.transparent && renderer.lines && renderer.hiddenLines && renderer.post->create();
    if (!m_renderReady) m_error = QStringLiteral("Vulkan 图形管线创建失败。");
    if (m_renderReady && m_gpuSurface && m_initial && m_initial->valid()) {
        renderer.dexel = std::make_unique<PocketCamVulkanDexel>();
        const auto stock = m_gpuPlayback || !m_stock ? m_initial : m_stock;
        if (!renderer.dexel->initialize(rhi(), renderer.scenePass.get(), *stock, m_scene, m_gpuFallbackReason)) {
            renderer.dexel.reset();
            m_gpuPlayback = false;
            m_gpuSurface = false;
            m_materialReady = false;
            requestFrame(m_step);
        } else m_materialDirty = false;
    }
}

void PocketCamVulkanWidget::render(QRhiCommandBuffer* commands)
{
    if (!m_renderReady || !m_renderer) return;
    if (m_renderer->color->pixelSize() != renderTarget()->pixelSize()) {
        initialize(commands);
        if (!m_renderReady) return;
    }
    auto& renderer = *m_renderer;
    auto* updates = rhi()->nextResourceUpdateBatch();
    bool uploaded = true;
    if (renderer.uploadStatic) {
        uploaded &= upload(rhi(), updates, m_target, renderer.target);
        uploaded &= upload(rhi(), updates, m_path, renderer.path);
        renderer.tools.resize(m_tools.size());
        for (std::size_t i = 0; i < m_tools.size(); ++i) uploaded &= upload(rhi(), updates, m_tools[i], renderer.tools[i]);
        renderer.uploadStatic = false;
    }
    if (m_materialDirty && renderer.dexel && !m_gpuPlayback && m_stock) {
        if (!renderer.dexel->uploadStock(rhi(), commands, *m_stock)) {
            renderer.dexel.reset();
            renderer.dexel = std::make_unique<PocketCamVulkanDexel>();
            if (!renderer.dexel->initialize(rhi(), renderer.scenePass.get(), *m_stock, m_scene, m_gpuFallbackReason)) {
                renderer.dexel.reset();
                m_gpuSurface = false;
                m_materialReady = false;
                requestFrame(m_step);
            }
        }
        m_materialDirty = false;
    } else if (m_materialDirty) {
        uploaded &= upload(rhi(), updates, m_material, renderer.material);
        m_materialDirty = false;
    }
    if (!uploaded) {
        updates->release();
        m_renderReady = false;
        m_error = QStringLiteral("Vulkan 网格上传失败，可能超过显存或缓冲预算。");
        return;
    }
    const QMatrix4x4 viewProjection = rhi()->clipSpaceCorrMatrix() * cameraMatrix();
    const auto uniform = [&](int slot, const QMatrix4x4& local, const QVector4D& color, bool lines, float alpha = 1.0f) {
        const QMatrix4x4 model = m_sceneTransform * local;
        float data[40]{};
        std::memcpy(data, viewProjection.constData(), 64);
        std::memcpy(data + 16, model.constData(), 64);
        data[32] = color.x(); data[33] = color.y(); data[34] = color.z(); data[35] = color.w();
        data[36] = lines ? 1.0f : 0.0f; data[37] = static_cast<float>(m_step);
        data[38] = alpha;
        updates->updateDynamicBuffer(renderer.uniforms[slot].get(), 0, uniformBytes, data);
    };
    uniform(0, {}, {102/255.0f, 208/255.0f, 173/255.0f, 1.0f}, false);
    uniform(1, {}, m_modelMode == 2 ? QVector4D(214/255.0f, 158/255.0f, 92/255.0f, 105/255.0f)
                                  : QVector4D(189/255.0f, 142/255.0f, 91/255.0f, 1.0f), false);
    uniform(2, m_toolTransform, {173/255.0f, 118/255.0f, 76/255.0f, 1.0f}, false);
    uniform(3, {}, {}, true);
    uniform(4, {}, {}, true, 0.1f);
    const auto size = renderTarget()->pixelSize();
    const float postData[8]{1.0f / size.width(), 1.0f / size.height(), cameraNearPlane(), cameraFarPlane(),
        m_orthographicProjection ? 1.0f : 0.0f, m_ambientOcclusionEnabled ? 1.0f : 0.0f, 0, 0};
    updates->updateDynamicBuffer(renderer.postUniform.get(), 0, 32, postData);
    if (renderer.dexel) renderer.dexel->advance(commands, m_scene, m_gpuPlayback ? m_step : 0);
    commands->beginPass(renderer.sceneTarget.get(), QColor::fromRgbF(0.075f, 0.09f, 0.12f), {1.0f, 0}, updates,
        renderer.dexel ? QRhiCommandBuffer::ExternalContent : QRhiCommandBuffer::BeginPassFlags{});
    const auto draw = [&](const MeshBuffers& mesh, QRhiGraphicsPipeline* pipeline, int slot) {
        if (!mesh.count) return;
        commands->setGraphicsPipeline(pipeline);
        // QRhi derives the default scissor from the current pipeline's viewport.
        // Bind the pipeline first, including when switching triangles to lines.
        commands->setViewport({0, 0, static_cast<float>(size.width()), static_cast<float>(size.height())});
        commands->setShaderResources(renderer.bindings[slot].get());
        const QRhiCommandBuffer::VertexInput input{mesh.vertices.get(), 0};
        commands->setVertexInput(0, 1, &input, mesh.indices.get(), 0, QRhiCommandBuffer::IndexUInt32);
        commands->drawIndexed(mesh.count);
    };
    if (m_modelMode != 1) {
        if (renderer.dexel) renderer.dexel->draw(commands, size, viewProjection, m_sceneTransform);
        else draw(renderer.material, renderer.solid.get(), 0);
    }
    if (m_modelMode != 0) draw(renderer.target, m_modelMode == 2 ? renderer.transparent.get() : renderer.solid.get(), 1);
    if (m_showPath) {
        draw(renderer.path, renderer.lines.get(), 3);
        draw(renderer.path, renderer.hiddenLines.get(), 4);
    }
    if (m_showCurrentTool && m_tool >= 0 && static_cast<std::size_t>(m_tool) < renderer.tools.size())
        draw(renderer.tools[m_tool], renderer.solid.get(), 2);
    commands->endPass();
    commands->beginPass(renderTarget(), Qt::black, {1.0f, 0});
    commands->setGraphicsPipeline(renderer.post.get());
    commands->setViewport({0, 0, static_cast<float>(size.width()), static_cast<float>(size.height())});
    commands->setShaderResources(renderer.postBindings.get());
    commands->draw(3);
    commands->endPass();
}
} // namespace oap::app::pocket_cam_detail
