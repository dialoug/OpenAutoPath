#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <utility>
#include <QVector4D>

namespace oap::app::pocket_cam_detail {

void PocketCamOpenGLWidget::initializeGL()
{
    if (!context() || !context()->isValid() ||
        !initializeOpenGLFunctions()) {
        m_error = QStringLiteral(
            "OpenGL 3.3 Core 上下文初始化失败");
        m_gpuReady = false;
        return;
    }
    const char* plainVertex = R"(
            #version 330 core
            layout(location=0) in vec3 aPosition;
            layout(location=1) in vec3 aNormal;
            uniform mat4 uViewProjection;
            uniform mat4 uSceneTransform;
            uniform mat4 uModel;
            out vec3 vNormal;
            void main() {
                mat4 sceneModel = uSceneTransform * uModel;
                vec4 world = sceneModel * vec4(aPosition, 1.0);
                vNormal = normalize(transpose(inverse(mat3(sceneModel))) * aNormal);
                gl_Position = uViewProjection * world;
            })";
    const char* fragment = R"(
            #version 330 core
            in vec3 vNormal;
            uniform vec4 uColor;
            out vec4 fragColor;
            void main() {
                vec3 normal = normalize(vNormal);
                vec3 light = normalize(vec3(0.35, 0.45, 1.0));
                float diffuse = max(dot(normal, light), 0.0);
                float shade = 0.30 + 0.70 * diffuse;
                fragColor = vec4(uColor.rgb * shade, uColor.a);
            })";
    const char* lineVertex = R"(
            #version 330 core
            layout(location=0) in vec3 aPosition;
            layout(location=1) in float aSimulationStep;
            layout(location=2) in float aRapid;
            uniform mat4 uViewProjection;
            uniform mat4 uSceneTransform;
            out float vSimulationStep;
            flat out float vRapid;
            void main() {
                vSimulationStep = aSimulationStep;
                vRapid = aRapid;
                gl_Position = uViewProjection * uSceneTransform * vec4(aPosition, 1.0);
            })";
    const char* lineFragment = R"(
            #version 330 core
            in float vSimulationStep;
            flat in float vRapid;
            uniform float uCurrentStep;
            uniform vec4 uFeedColor;
            uniform vec4 uRapidColor;
            uniform vec4 uPendingColor;
            out vec4 fragColor;
            void main() {
                vec4 motionColor = mix(
                    uFeedColor, uRapidColor,
                    step(0.5, vRapid));
                float pending = step(
                    uCurrentStep + 0.5,
                    vSimulationStep);
                fragColor = mix(
                    motionColor, uPendingColor,
                    pending);
            })";
    const char* postVertex = R"(
            #version 330 core
            out vec2 vUv;
            void main() {
                vec2 position = vec2(
                    (gl_VertexID << 1) & 2,
                    gl_VertexID & 2);
                vUv = position;
                gl_Position = vec4(
                    position * 2.0 - 1.0,
                    0.0, 1.0);
            })";
    const char* postFragment = R"(
            #version 330 core
            in vec2 vUv;
            uniform sampler2D uColorTexture;
            uniform sampler2D uDepthTexture;
            uniform vec2 uTexelSize;
            uniform vec2 uClipPlanes;
            uniform bool uOrthographic;
            uniform bool uAmbientOcclusion;
            out vec4 fragColor;

            float linearDepth(float depth) {
                float nearPlane = uClipPlanes.x;
                float farPlane = uClipPlanes.y;
                if (uOrthographic) {
                    return mix(
                        nearPlane, farPlane, depth);
                }
                float ndc = depth * 2.0 - 1.0;
                return
                    (2.0 * nearPlane * farPlane) /
                    (farPlane + nearPlane -
                     ndc * (farPlane - nearPlane));
            }

            float luminance(vec3 color) {
                return dot(color, vec3(0.299, 0.587, 0.114));
            }

            vec4 fxaa(vec2 uv) {
                vec3 rgbNorthWest = texture(
                    uColorTexture,
                    clamp(uv + vec2(-1.0, -1.0) * uTexelSize,
                          vec2(0.0), vec2(1.0))).rgb;
                vec3 rgbNorthEast = texture(
                    uColorTexture,
                    clamp(uv + vec2(1.0, -1.0) * uTexelSize,
                          vec2(0.0), vec2(1.0))).rgb;
                vec3 rgbSouthWest = texture(
                    uColorTexture,
                    clamp(uv + vec2(-1.0, 1.0) * uTexelSize,
                          vec2(0.0), vec2(1.0))).rgb;
                vec3 rgbSouthEast = texture(
                    uColorTexture,
                    clamp(uv + vec2(1.0, 1.0) * uTexelSize,
                          vec2(0.0), vec2(1.0))).rgb;
                vec4 center = texture(uColorTexture, uv);
                float lumaNorthWest = luminance(rgbNorthWest);
                float lumaNorthEast = luminance(rgbNorthEast);
                float lumaSouthWest = luminance(rgbSouthWest);
                float lumaSouthEast = luminance(rgbSouthEast);
                float lumaCenter = luminance(center.rgb);
                float lumaMinimum = min(
                    lumaCenter,
                    min(min(lumaNorthWest, lumaNorthEast),
                        min(lumaSouthWest, lumaSouthEast)));
                float lumaMaximum = max(
                    lumaCenter,
                    max(max(lumaNorthWest, lumaNorthEast),
                        max(lumaSouthWest, lumaSouthEast)));
                vec2 direction = vec2(
                    -((lumaNorthWest + lumaNorthEast) -
                      (lumaSouthWest + lumaSouthEast)),
                    (lumaNorthWest + lumaSouthWest) -
                      (lumaNorthEast + lumaSouthEast));
                float directionReduce = max(
                    0.0078125,
                    0.03125 * 0.25 *
                        (lumaNorthWest + lumaNorthEast +
                         lumaSouthWest + lumaSouthEast));
                float reciprocalMinimum = 1.0 /
                    (min(abs(direction.x), abs(direction.y)) +
                     directionReduce);
                direction = clamp(
                    direction * reciprocalMinimum,
                    vec2(-8.0), vec2(8.0)) * uTexelSize;
                vec3 rgbA = 0.5 * (
                    texture(uColorTexture, clamp(
                        uv + direction * (1.0 / 3.0 - 0.5),
                        vec2(0.0), vec2(1.0))).rgb +
                    texture(uColorTexture, clamp(
                        uv + direction * (2.0 / 3.0 - 0.5),
                        vec2(0.0), vec2(1.0))).rgb);
                vec3 rgbB = rgbA * 0.5 + 0.25 * (
                    texture(uColorTexture, clamp(
                        uv + direction * -0.5,
                        vec2(0.0), vec2(1.0))).rgb +
                    texture(uColorTexture, clamp(
                        uv + direction * 0.5,
                        vec2(0.0), vec2(1.0))).rgb);
                float lumaB = luminance(rgbB);
                return vec4(
                    lumaB < lumaMinimum || lumaB > lumaMaximum
                        ? rgbA : rgbB,
                    center.a);
            }

            void main() {
                vec4 color = fxaa(vUv);
                if (!uAmbientOcclusion) {
                    fragColor = color;
                    return;
                }
                float rawDepth =
                    texture(uDepthTexture, vUv).r;
                if (rawDepth >= 0.999999) {
                    fragColor = color;
                    return;
                }

                float centerDepth =
                    linearDepth(rawDepth);
                vec2 directions[8] = vec2[](
                    vec2(1.0, 0.0),
                    vec2(-1.0, 0.0),
                    vec2(0.0, 1.0),
                    vec2(0.0, -1.0),
                    vec2(0.707, 0.707),
                    vec2(-0.707, 0.707),
                    vec2(0.707, -0.707),
                    vec2(-0.707, -0.707));
                float occlusion = 0.0;
                for (int radiusIndex = 0;
                     radiusIndex < 3; ++radiusIndex) {
                    float radius =
                        1.5 + float(radiusIndex) * 2.0;
                    for (int index = 0;
                         index < 8; ++index) {
                        vec2 sampleUv = clamp(
                            vUv +
                                directions[index] *
                                uTexelSize * radius,
                            vec2(0.0), vec2(1.0));
                        float sampleDepth = linearDepth(
                            texture(
                                uDepthTexture,
                                sampleUv).r);
                        float gap =
                            centerDepth - sampleDepth;
                        float bias =
                            max(0.0005,
                                centerDepth * 0.0008);
                        float rangeWeight =
                            1.0 - smoothstep(
                                centerDepth * 0.02,
                                centerDepth * 0.16,
                                abs(gap));
                        occlusion +=
                            step(bias, gap) *
                            rangeWeight;
                    }
                }
                float ambient = clamp(
                    1.0 - 0.55 *
                        occlusion / 24.0,
                    0.45, 1.0);
                fragColor = vec4(
                    color.rgb * ambient,
                    color.a);
            })";

    if (!compileProgram(
            m_plainProgram, plainVertex, fragment,
            QStringLiteral("普通网格")) ||
        !compileProgram(
            m_lineProgram, lineVertex, lineFragment,
            QStringLiteral("刀路线")) ||
        !compileProgram(
            m_postProgram, postVertex, postFragment,
            QStringLiteral("SSAO 后处理"))) {
        return;
    }
    if (!initializeGpuDexelPrograms()) {
        if (m_error.isEmpty()) {
            m_error = QStringLiteral(
                "GPU Multi-Dexel 初始化失败：需要 OpenGL 4.3、Compute Shader 和 SSBO 支持");
        }
        m_gpuReady = false;
        return;
    }

    uploadMesh(m_finalStockCpu, m_finalStockGpu);
    uploadMesh(m_targetCpu, m_targetGpu);
    m_toolGpu.resize(m_toolCpu.size());
    for (std::size_t index = 0u;
         index < m_toolCpu.size(); ++index) {
        uploadMesh(m_toolCpu[index], m_toolGpu[index]);
    }
    glGenVertexArrays(1, &m_screenVao);
    uploadPathLines();
    m_gpuReady = m_gpuDexelSupported &&
        !m_toolGpu.empty() &&
        m_toolGpu.size() == m_scene.tools.size();
    for (std::size_t index = 0u;
        m_gpuReady && index < m_toolGpu.size(); ++index) {
        m_gpuReady =
            m_toolGpu[index].indexCount > 0;
    }
    if (m_gpuReady &&
        (!m_initialGpuStock ||
         !uploadPlaybackStock(*m_initialGpuStock))) {
        m_error = QStringLiteral("初始 IPW 材料无法上传到 GPU Multi-Dexel：%1")
            .arg(m_error.isEmpty() ? QStringLiteral("缺少有效材料或网格超过上传预算") : m_error);
        m_gpuReady = false;
    }
}

void PocketCamOpenGLWidget::resizeGL(int width, int height)
{
    glViewport(
        0, 0,
        static_cast<GLsizei>(width * devicePixelRatioF()),
        static_cast<GLsizei>(height * devicePixelRatioF()));
}

void PocketCamOpenGLWidget::paintGL()
{
    if (!m_gpuReady || !m_error.isEmpty()) {
        glBindFramebuffer(
            GL_FRAMEBUFFER,
            defaultFramebufferObject());
        glClearColor(0.075f, 0.09f, 0.12f, 1.0f);
        glClear(
            GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT);
        return;
    }
    beginOpenGlStage(QStringLiteral("OpenGL 帧"));
    if (!m_gpuDexelReady &&
        m_waitingForOperationBaseline) {
        glBindFramebuffer(
            GL_FRAMEBUFFER,
            defaultFramebufferObject());
        glClearColor(0.075f, 0.09f, 0.12f, 1.0f);
        glClear(
            GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT);
        return;
    }
    if (!m_gpuDexelReady && !m_usePlaybackMesh &&
        (!m_initialGpuStock ||
         !uploadPlaybackStock(*m_initialGpuStock))) {
        m_error = QStringLiteral(
            "回退播放时无法恢复 GPU Multi-Dexel 初始材料");
        glBindFramebuffer(
            GL_FRAMEBUFFER,
            defaultFramebufferObject());
        glClearColor(0.075f, 0.09f, 0.12f, 1.0f);
        glClear(
            GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT);
        update();
        return;
    }
    // 完成帧继续保留同一 GPU 增量状态，避免重新上传 CPU 检查点时
    // 丢失逐原语累计的解析距离缓存。CPU finalIpwStock 仍独立用于
    // OCCT、体积和余量审计，不作为解析显示边界的替换来源。
    if (!m_scene.authoritativeIpwPlayback) syncGpuDexelToCurrentStep();

    const int pixelWidth = std::max(
        1, static_cast<int>(std::lround(
               width() * devicePixelRatioF())));
    const int pixelHeight = std::max(
        1, static_cast<int>(std::lround(
               height() * devicePixelRatioF())));
    const int scenePixelWidth = pixelWidth;
    const int scenePixelHeight = pixelHeight;
    // 后处理始终启用，以便 FXAA 在 SSAO 关闭时仍然生效。
    const bool wantsOffscreen = true;
    const bool useOffscreen =
        wantsOffscreen &&
        ensurePostFramebuffer(
            scenePixelWidth,
            scenePixelHeight);
    const QMatrix4x4 viewProjection = cameraMatrix();
    const GLuint renderFramebuffer = useOffscreen
        ? m_sceneFramebuffer
        : defaultFramebufferObject();
    glBindFramebuffer(
        GL_FRAMEBUFFER, renderFramebuffer);
    glViewport(0, 0, pixelWidth, pixelHeight);
    clearSceneFramebuffer();
    if (!checkOpenGlStage(
            QStringLiteral("OpenGL 帧缓冲设置与清屏"))) {
        return;
    }
    renderScene(viewProjection);

    if (useOffscreen) {
        glBindFramebuffer(
            GL_FRAMEBUFFER,
            defaultFramebufferObject());
        glViewport(0, 0, pixelWidth, pixelHeight);
        glClearColor(0.075f, 0.09f, 0.12f, 1.0f);
        glClear(
            GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT);
        renderPostProcess(
            scenePixelWidth,
            scenePixelHeight);
    }
}

void PocketCamOpenGLWidget::renderScene(const QMatrix4x4& viewProjection)
{
    renderStockLayer(viewProjection);
    renderSceneOverlays(viewProjection);
}

void PocketCamOpenGLWidget::clearSceneFramebuffer()
{
    glClearColor(0.075f, 0.09f, 0.12f, 1.0f);
    glClear(
        GL_COLOR_BUFFER_BIT |
        GL_DEPTH_BUFFER_BIT);
}

void PocketCamOpenGLWidget::renderStockLayer(const QMatrix4x4& viewProjection)
{
    if (m_modelMode != 1) {
        // 播放时使用 GPU Multi-Dexel 表面提供逐步动画；完成帧优先显示
        // 与 OCCT/审计结果一致的权威网格，避免显示近似被误判为过切。
        if (m_usePlaybackMesh) {
            glDisable(GL_BLEND);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glEnable(GL_CULL_FACE);
            glEnable(GL_DEPTH_TEST);
            glCullFace(GL_BACK);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            bindPlain(viewProjection, QColor(102, 208, 173));
            drawPlain(m_playbackStockGpu);
        } else if (m_currentStep >= totalSteps() &&
            m_finalStockGpu.indexCount > 0) {
            renderAuthoritativeFinalStock(
                viewProjection);
        } else if (m_gpuDexelReady) {
            renderGpuDexel(viewProjection);
        }
    }
}

void PocketCamOpenGLWidget::renderSceneOverlays(
    const QMatrix4x4& viewProjection)
{
    if (m_modelMode != 0 && m_targetGpu.indexCount > 0)
        renderTarget(viewProjection);
    if (m_showPath)
        renderPath(viewProjection);
    if (m_showCurrentTool)
        renderCurrentTool(viewProjection);
}

void PocketCamOpenGLWidget::destroyPostFramebuffer()
{
    if (m_sceneColorTexture != 0u)
        glDeleteTextures(1, &m_sceneColorTexture);
    if (m_sceneDepthTexture != 0u)
        glDeleteTextures(
            1, &m_sceneDepthTexture);
    if (m_sceneFramebuffer != 0u)
        glDeleteFramebuffers(
            1, &m_sceneFramebuffer);
    m_sceneColorTexture = 0u;
    m_sceneDepthTexture = 0u;
    m_sceneFramebuffer = 0u;
    m_sceneFramebufferWidth = 0;
    m_sceneFramebufferHeight = 0;
}

bool PocketCamOpenGLWidget::ensurePostFramebuffer(
    int pixelWidth,
    int pixelHeight)
{
    if (m_sceneFramebuffer != 0u &&
        m_sceneFramebufferWidth == pixelWidth &&
        m_sceneFramebufferHeight == pixelHeight) {
        return true;
    }

    destroyPostFramebuffer();
    glGenFramebuffers(1, &m_sceneFramebuffer);
    glBindFramebuffer(
        GL_FRAMEBUFFER, m_sceneFramebuffer);

    auto configureTexture = [&](GLuint& texture) {
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(
            GL_TEXTURE_2D,
            GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(
            GL_TEXTURE_2D,
            GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(
            GL_TEXTURE_2D,
            GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(
            GL_TEXTURE_2D,
            GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    };

    configureTexture(m_sceneColorTexture);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA8,
        pixelWidth, pixelHeight, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D,
        m_sceneColorTexture, 0);

    configureTexture(m_sceneDepthTexture);
    glTexParameteri(
        GL_TEXTURE_2D,
        GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(
        GL_TEXTURE_2D,
        GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
        pixelWidth, pixelHeight, 0,
        GL_DEPTH_COMPONENT,
        GL_UNSIGNED_INT, nullptr);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_DEPTH_ATTACHMENT,
        GL_TEXTURE_2D,
        m_sceneDepthTexture, 0);

    const GLenum drawBuffer =
        GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &drawBuffer);
    const bool complete =
        glCheckFramebufferStatus(
            GL_FRAMEBUFFER) ==
        GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(
        GL_FRAMEBUFFER,
        defaultFramebufferObject());
    if (!complete) {
        destroyPostFramebuffer();
        return false;
    }
    m_sceneFramebufferWidth = pixelWidth;
    m_sceneFramebufferHeight = pixelHeight;
    return true;
}

void PocketCamOpenGLWidget::renderPostProcess(
    int pixelWidth,
    int pixelHeight)
{
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    m_postProgram.bind();
    m_postProgram.setUniformValue(
        "uColorTexture", 0);
    m_postProgram.setUniformValue(
        "uDepthTexture", 1);
    m_postProgram.setUniformValue(
        "uTexelSize",
        QVector2D(
            1.0f / static_cast<float>(pixelWidth),
            1.0f / static_cast<float>(pixelHeight)));
    m_postProgram.setUniformValue(
        "uClipPlanes",
        QVector2D(
            cameraNearPlane(),
            cameraFarPlane()));
    m_postProgram.setUniformValue(
        "uOrthographic",
        m_orthographicProjection);
    m_postProgram.setUniformValue(
        "uAmbientOcclusion",
        m_ambientOcclusionEnabled);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(
        GL_TEXTURE_2D,
        m_sceneColorTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(
        GL_TEXTURE_2D,
        m_sceneDepthTexture);
    glBindVertexArray(m_screenVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDepthMask(GL_TRUE);
}

bool PocketCamOpenGLWidget::compileProgram(
    QOpenGLShaderProgram& program,
    const char* vertex,
    const char* fragment,
    const QString& label)
{
    if (!program.addShaderFromSourceCode(
            QOpenGLShader::Vertex, vertex) ||
        !program.addShaderFromSourceCode(
            QOpenGLShader::Fragment, fragment) ||
        !program.link()) {
        m_error = QStringLiteral("%1着色器编译失败：%2")
            .arg(label, program.log());
        return false;
    }
    return true;
}

void PocketCamOpenGLWidget::uploadMesh(const RenderMesh& source, GpuMesh& target)
{
    if (source.empty())
        return;
    glGenVertexArrays(1, &target.vao);
    glGenBuffers(1, &target.vertexBuffer);
    glGenBuffers(1, &target.indexBuffer);
    glBindVertexArray(target.vao);
    glBindBuffer(GL_ARRAY_BUFFER, target.vertexBuffer);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(
            source.vertices.size() * sizeof(RenderVertex)),
        source.vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, target.indexBuffer);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(
            source.indices.size() * sizeof(std::uint32_t)),
        source.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE, sizeof(RenderVertex),
        reinterpret_cast<const void*>(
            offsetof(RenderVertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1, 3, GL_FLOAT, GL_FALSE, sizeof(RenderVertex),
        reinterpret_cast<const void*>(
            offsetof(RenderVertex, normal)));
    target.indexCount =
        static_cast<GLsizei>(source.indices.size());
    glBindVertexArray(0);
}

void PocketCamOpenGLWidget::destroyGpuMesh(GpuMesh& mesh)
{
    if (mesh.vao != 0u)
        glDeleteVertexArrays(1, &mesh.vao);
    if (mesh.vertexBuffer != 0u)
        glDeleteBuffers(1, &mesh.vertexBuffer);
    if (mesh.indexBuffer != 0u)
        glDeleteBuffers(1, &mesh.indexBuffer);
    mesh = {};
}

void PocketCamOpenGLWidget::uploadPathLines()
{
    std::vector<PathRenderVertex> vertices;
    vertices.reserve(m_scene.motions.size() * 2u);
    for (const toolpath::PocketCamMotion& motion :
         m_scene.motions) {
        const bool rapid =
            motion.move.type ==
                toolpath::ToolpathMoveType::Rapid ||
            motion.move.type ==
                toolpath::ToolpathMoveType::Retract;
        const float firstStep = static_cast<float>(
            motion.firstSimulationStep);
        const float finalStep = static_cast<float>(
            motion.firstSimulationStep +
            static_cast<std::size_t>(
                motion.simulationStepCount));

        // Draw the exact same controller-independent fallback segment
        // used by OCCT, validation and NC export.
        vertices.push_back({
            vector3(motion.move.from),
            firstStep,
            rapid ? 1.0f : 0.0f});
        vertices.push_back({
            vector3(motion.move.to),
            finalStep,
            rapid ? 1.0f : 0.0f});
    }
    glGenVertexArrays(1, &m_lineVao);
    glGenBuffers(1, &m_lineVbo);
    glBindVertexArray(m_lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(
            vertices.size() * sizeof(PathRenderVertex)),
        vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE,
        sizeof(PathRenderVertex),
        reinterpret_cast<const void*>(
            offsetof(PathRenderVertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1, 1, GL_FLOAT, GL_FALSE,
        sizeof(PathRenderVertex),
        reinterpret_cast<const void*>(
            offsetof(
                PathRenderVertex,
                simulationStep)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
        2, 1, GL_FLOAT, GL_FALSE,
        sizeof(PathRenderVertex),
        reinterpret_cast<const void*>(
            offsetof(PathRenderVertex, rapid)));
    glBindVertexArray(0);
    m_lineVertexCount =
        static_cast<GLsizei>(vertices.size());
}

void PocketCamOpenGLWidget::bindPlain(
    const QMatrix4x4& viewProjection,
    const QColor& color,
    const QMatrix4x4& model)
{
    m_plainProgram.bind();
    m_plainProgram.setUniformValue(
        "uViewProjection", viewProjection);
    m_plainProgram.setUniformValue(
        "uSceneTransform", m_sceneTransform);
    m_plainProgram.setUniformValue("uModel", model);
    m_plainProgram.setUniformValue(
        "uColor", QVector4D(
            color.redF(), color.greenF(),
            color.blueF(), color.alphaF()));
}

void PocketCamOpenGLWidget::drawPlain(const GpuMesh& mesh)
{
    if (mesh.indexCount <= 0)
        return;
    glBindVertexArray(mesh.vao);
    glDrawElements(
        GL_TRIANGLES, mesh.indexCount,
        GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void PocketCamOpenGLWidget::renderAuthoritativeFinalStock(
    const QMatrix4x4& viewProjection)
{
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glColorMask(
        GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glCullFace(GL_BACK);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    bindPlain(
        viewProjection,
        QColor(102, 208, 173));
    drawPlain(m_finalStockGpu);
}

void PocketCamOpenGLWidget::renderTarget(const QMatrix4x4& viewProjection)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glDepthFunc(GL_LEQUAL);
    const bool overlay = m_modelMode == 2;
    if (overlay) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }
    QMatrix4x4 identity;
    bindPlain(
        viewProjection,
        overlay ? QColor(214, 158, 92, 105)
                : QColor(189, 142, 91),
        identity);
    drawPlain(m_targetGpu);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void PocketCamOpenGLWidget::renderPath(const QMatrix4x4& viewProjection)
{
    if (m_lineVertexCount <= 0)
        return;
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(2.0f);
    m_lineProgram.bind();
    m_lineProgram.setUniformValue(
        "uViewProjection", viewProjection);
    m_lineProgram.setUniformValue(
        "uSceneTransform", m_sceneTransform);
    m_lineProgram.setUniformValue(
        "uCurrentStep",
        static_cast<float>(m_currentStep));
    glBindVertexArray(m_lineVao);
    auto drawPass = [&](GLenum depthFunction,
                        float alphaScale) {
        glDepthFunc(depthFunction);
        m_lineProgram.setUniformValue(
            "uFeedColor",
            QVector4D(
                0.97f, 0.82f, 0.18f,
                0.90f * alphaScale));
        m_lineProgram.setUniformValue(
            "uRapidColor",
            QVector4D(
                0.25f, 0.72f, 1.0f,
                0.82f * alphaScale));
        m_lineProgram.setUniformValue(
            "uPendingColor",
            QVector4D(
                0.55f, 0.59f, 0.66f,
                0.30f * alphaScale));
        glDrawArrays(
            GL_LINES, 0, m_lineVertexCount);
    };
    drawPass(GL_LEQUAL, 1.0f);
    // 匹配 FreeCAD 的第二遍微弱通道,用于隐藏在模拟毛坯
    // 后面的刀路部分。
    drawPass(GL_GREATER, 0.1f);
    glBindVertexArray(0);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);
}

void PocketCamOpenGLWidget::renderCurrentTool(
    const QMatrix4x4& viewProjection)
{
    if (m_currentToolIndex < 0 ||
        static_cast<std::size_t>(m_currentToolIndex) >=
            m_toolGpu.size()) {
        return;
    }
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    bindPlain(
        viewProjection, QColor(173, 118, 76),
        m_currentToolTransform);
    drawPlain(m_toolGpu[
        static_cast<std::size_t>(m_currentToolIndex)]);
}

} // namespace oap::app::pocket_cam_detail
