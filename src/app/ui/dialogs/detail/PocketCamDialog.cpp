#include "app/ui/dialogs/detail/PocketCamDialog.h"
#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"
#include "app/ui/dialogs/detail/PocketCamVulkanWidget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>
#include <gp_XYZ.hxx>

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QKeySequence>
#include <QLabel>
#include <QPushButton>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector3D>

namespace oap::app::pocket_cam_detail {

PocketCamDialog::PocketCamDialog(
    toolpath::PocketCamSimulationScene scene,
    std::vector<std::string> warnings,
    RenderMesh finalStock,
    RenderMesh target,
    PocketCamInteractionOptions interaction,
    std::shared_ptr<const toolpath::MultiDexelStock>
        initialIpwStock,
    std::shared_ptr<const toolpath::MultiDexelStock>
        finalIpwStock,
    std::vector<std::shared_ptr<const IpwTransientFrameSource>>
        operationInitialIpwSources,
    QVector3D stockBoundsMinimum,
    QVector3D stockBoundsMaximum,
    bool stockBoundsValid,
    std::string occtMeshFingerprint,
    std::string openGlMeshFingerprint,
    bool sharedMeshAccepted,
    std::size_t finalStockTriangleCount,
    QString visualAcceptanceDirectory,
    PocketCamVisualAcceptanceView
        visualAcceptanceView,
    QWidget* parent, PocketCamBackend backend)
    : QDialog(parent),
      m_motionResolution(scene.motionResolution),
      m_displayMotionCount(scene.motions.size()),
      m_sourceMotionCount(scene.sourceMotionCount),
      m_totalSimulationSteps(scene.totalSimulationSteps), m_backend(backend)
{
    const bool projectPlayback = scene.authoritativeIpwPlayback;
    const bool vulkan = backend == PocketCamBackend::Vulkan;
    if (vulkan) {
        m_view = new PocketCamVulkanWidget(std::move(scene), std::move(target), interaction,
            std::move(initialIpwStock), std::move(finalIpwStock), std::move(operationInitialIpwSources),
            stockBoundsMinimum, stockBoundsMaximum, stockBoundsValid, this);
    } else {
        m_view = new PocketCamOpenGLWidget(std::move(scene), std::move(finalStock), std::move(target), interaction,
            std::move(initialIpwStock), std::move(finalIpwStock), std::move(operationInitialIpwSources),
            stockBoundsMinimum, stockBoundsMaximum, stockBoundsValid, this);
    }
    setWindowTitle(QStringLiteral("加工 IPW 仿真 — %1").arg(vulkan ? "Vulkan" : "OpenGL"));
    setWindowFlags(
        windowFlags() | Qt::WindowMinMaxButtonsHint);
    setSizeGripEnabled(true);
    setSizePolicy(
        QSizePolicy::Expanding,
        QSizePolicy::Expanding);
    resize(1180, 820);
    auto* layout = new QVBoxLayout(this);
    auto* description = new QLabel(
        (projectPlayback || vulkan ? QStringLiteral(
            "按所选工序顺序播放刀路；刀具、装夹方向与剩余毛坯同步更新。\n"
            "视图操作（同步 OCCT：%1）：%2拖动旋转，%3拖动平移，"
            "滚轮缩放，双击复位。") : QStringLiteral(
            "材料播放仅使用 OpenGL 4.3 Compute Shader + GPU Multi-Dexel。"
            "Rapid/退刀只显示移动轨迹，不参与材料减除；完成帧继续显示 "
            "GPU 光顺派生表面，CPU 权威网格保留用于 OCCT 和独立余量审计。\n"
            "视图操作（同步 OCCT：%1）：%2拖动旋转，%3拖动平移，"
            "滚轮缩放，双击复位；可启用转台旋转和屏幕空间环境光遮蔽。"))
            .arg(rotationModeText(interaction.rotationMode))
            .arg(mouseButtonText(interaction.rotateButton))
            .arg(mouseButtonText(interaction.panButton)),
        this);
    description->setWordWrap(true);
    layout->addWidget(description);
    layout->addWidget(m_view->widget(), 1);

    auto* controls = new QVBoxLayout;
    controls->setContentsMargins(0, 0, 0, 0);
    auto* playbackControls = new QHBoxLayout;
    auto* viewControls = new QHBoxLayout;
    auto* play = new QPushButton(
        QStringLiteral("播放"), this);
    auto* singleStep = new QPushButton(
        QStringLiteral("单步"), this);
    auto* reset = new QPushButton(
        QStringLiteral("复位"), this);
    auto* finalFrame = new QPushButton(
        QStringLiteral("完成帧"), this);
    auto* viewAll = new QPushButton(
        QStringLiteral("适合窗口"), this);
    auto* speed = new QComboBox(this);
    for (int value : {1, 2, 5, 10, 25, 50})
        speed->addItem(
            QStringLiteral("×%1").arg(value), value);
    speed->setCurrentIndex(2);
    auto* stage = new QSlider(Qt::Horizontal, this);
    stage->setRange(0, 10000);
    auto* stageLabel = new QLabel(this);
    auto* path = new QCheckBox(
        QStringLiteral("显示刀路"), this);
    path->setChecked(true);
    auto* turntable = new QCheckBox(
        QStringLiteral("转台旋转"), this);
    auto* ambientOcclusion = new QCheckBox(
        QStringLiteral("环境光遮蔽"), this);
    ambientOcclusion->setToolTip(
        QStringLiteral(
            "使用屏幕空间深度采样增强槽腔内壁和转角层次"));
    auto* model = new QComboBox(this);
    model->addItem(QStringLiteral("仿真毛坯"), 0);
    model->addItem(QStringLiteral("目标模型"), 1);
    model->addItem(QStringLiteral("毛坯 + 目标"), 2);
    auto* projection = new QComboBox(this);
    projection->addItem(QStringLiteral("透视投影"), false);
    projection->addItem(QStringLiteral("正交投影"), true);
    play->setShortcut(QKeySequence(Qt::Key_Space));
    singleStep->setShortcut(QKeySequence(Qt::Key_Right));
    reset->setShortcut(QKeySequence(Qt::Key_Home));
    finalFrame->setShortcut(QKeySequence(Qt::Key_End));
    viewAll->setShortcut(QKeySequence(Qt::Key_F));
    play->setToolTip(QStringLiteral("播放/暂停（空格）"));
    singleStep->setToolTip(QStringLiteral("前进一步（→）"));
    reset->setToolTip(QStringLiteral("仿真回到起点（Home）"));
    finalFrame->setToolTip(
        QStringLiteral(
            "跳到统一 IPW 完成帧（End）"));
    viewAll->setToolTip(QStringLiteral("视图适合窗口（F）"));
    playbackControls->addWidget(play);
    playbackControls->addWidget(singleStep);
    playbackControls->addWidget(reset);
    playbackControls->addWidget(finalFrame);
    playbackControls->addWidget(speed);
    playbackControls->addWidget(stage, 1);
    playbackControls->addWidget(stageLabel);
    viewControls->addWidget(viewAll);
    viewControls->addWidget(path);
    viewControls->addWidget(turntable);
    viewControls->addWidget(ambientOcclusion);
    viewControls->addWidget(model);
    viewControls->addWidget(projection);
    viewControls->addStretch(1);
    auto* backendChoice = new QComboBox(this);
    backendChoice->setObjectName("simulationRenderBackend");
    backendChoice->addItem(QStringLiteral("Vulkan"), static_cast<int>(PocketCamBackend::Vulkan));
    backendChoice->addItem(QStringLiteral("OpenGL（兼容）"), static_cast<int>(PocketCamBackend::OpenGL));
    backendChoice->setCurrentIndex(backendChoice->findData(static_cast<int>(backend)));
    backendChoice->setToolTip(QStringLiteral("切换显示后端会重新打开仿真并回到起点，IPW 检查点保持不变。"));
    viewControls->addWidget(backendChoice);
    connect(backendChoice, &QComboBox::currentIndexChanged, this, [this, backendChoice] {
        m_backend = static_cast<PocketCamBackend>(backendChoice->currentData().toInt());
        done(SwitchBackend);
    });
    ambientOcclusion->setEnabled(m_view->supportsAmbientOcclusion());
    if (!m_view->supportsAmbientOcclusion())
        ambientOcclusion->setToolTip(QStringLiteral("当前显示后端不支持屏幕空间环境光遮蔽。"));
    controls->addLayout(playbackControls);
    controls->addLayout(viewControls);
    layout->addLayout(controls);

    QString detail = QStringLiteral(
        "FreeCAD 运动分辨率 %1 mm；显示移动 %2/%3；仿真步数 %4。")
        .arg(m_motionResolution, 0, 'f', 3)
        .arg(static_cast<qulonglong>(
            m_displayMotionCount))
        .arg(static_cast<qulonglong>(
            m_sourceMotionCount))
        .arg(static_cast<qulonglong>(
            m_totalSimulationSteps));
    detail += projectPlayback || vulkan ? QStringLiteral(
        "\n材料按原始刀路更新，快移和退刀不去除材料；支持暂停、单步、进度跳转及完成帧。"
        "复杂刀路更新材料帧时会等待计算完成。") : QStringLiteral(
        "\nGPU-only：中间帧由 OpenGL 4.3 Compute Shader 直接更新 "
        "Multi-Dexel；首个槽腔工序前的 IPW 检查点只上传一次，"
        "全部固定轴工序在同一 GPU 材料状态上连续执行，"
        "视角变化只重新绘制当前 GPU 材料；"
        "播放帧和完成帧均以窄带 TSDF 生成 Hermite 边交点，"
        "再由插值 Marching Squares 重建亚单元光顺边界；"
        "统一 Multi-Dexel IPW 网格仍作为 OCCT 与审计制品保留。");
    if (sharedMeshAccepted && !vulkan) {
        detail += QStringLiteral(
            "\n统一网格指纹：%1（OCCT/OpenGL 一致，%2 个三角形）。")
            .arg(QString::fromStdString(
                openGlMeshFingerprint))
            .arg(static_cast<qulonglong>(
                finalStockTriangleCount));
    }
    if (vulkan) detail += QStringLiteral("\nVulkan 显示采用后台 IPW 网格帧；材料精度不变，密集刀路可能等待网格更新。");
    for (const std::string& warning : warnings) {
        detail += QStringLiteral("\n提示：%1")
            .arg(QString::fromStdString(warning));
    }
    auto* gpuStatus = new QLabel(
        QStringLiteral("GPU 材料状态：初始化中"), this);
    gpuStatus->setTextInteractionFlags(
        Qt::TextSelectableByMouse);
    layout->addWidget(gpuStatus);
    auto* info = new QLabel(detail, this);
    info->setWordWrap(true);
    info->setTextInteractionFlags(
        Qt::TextSelectableByMouse);
    layout->addWidget(info);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Close, this);
    connect(
        buttons, &QDialogButtonBox::rejected,
        this, &QDialog::reject);
    layout->addWidget(buttons);

    auto updateStage = [this, stage, stageLabel] {
        const std::size_t total =
            m_view->totalSteps();
        const std::size_t current =
            m_view->currentStep();
        const int sliderValue = total == 0u
            ? 0
            : static_cast<int>(
                  (10000.0 * static_cast<double>(current)) /
                  static_cast<double>(total));
        {
            const QSignalBlocker blocker(stage);
            stage->setValue(sliderValue);
        }
        stageLabel->setText(
            QStringLiteral("%1 / %2")
                .arg(static_cast<qulonglong>(current))
                .arg(static_cast<qulonglong>(total)));
    };
    updateStage();

    connect(play, &QPushButton::clicked, this,
            [this, play] {
                m_playing = !m_playing;
                play->setText(
                    m_playing
                        ? QStringLiteral("暂停")
                        : QStringLiteral("播放"));
                m_elapsed.restart();
            });
    connect(singleStep, &QPushButton::clicked, this,
            [this, play, updateStage] {
                m_playing = false;
                play->setText(QStringLiteral("播放"));
                m_fractionalStep = static_cast<double>(m_view->currentStep());
                m_view->setCurrentStep(
                    std::min(
                        m_view->totalSteps(),
                        m_view->currentStep() + 1u));
                updateStage();
            });
    connect(reset, &QPushButton::clicked, this,
            [this, play, updateStage] {
                m_playing = false;
                play->setText(QStringLiteral("播放"));
                m_fractionalStep = 0.0;
                m_view->setCurrentStep(0u);
                updateStage();
            });
    connect(
        finalFrame, &QPushButton::clicked, this,
        [this, play, updateStage] {
            m_playing = false;
            play->setText(QStringLiteral("播放"));
            m_fractionalStep =
                static_cast<double>(
                    m_view->totalSteps());
            m_view->setCurrentStep(
                m_view->totalSteps());
            updateStage();
        });
    connect(viewAll, &QPushButton::clicked, this, [this] { m_view->resetCamera(); });
    connect(path, &QCheckBox::toggled, this, [this](bool value) { m_view->setPathVisible(value); });
    connect(ambientOcclusion, &QCheckBox::toggled, this, [this](bool value) { m_view->setAmbientOcclusionEnabled(value); });
    connect(model, &QComboBox::currentIndexChanged, this,
            [this, model](int) {
                m_view->setModelMode(
                    model->currentData().toInt());
            });
    connect(
        projection,
        &QComboBox::currentIndexChanged,
        this,
        [this, projection](int) {
            m_view->setOrthographicProjection(
                projection->currentData().toBool());
        });
    connect(stage, &QSlider::valueChanged, this,
            [this, stageLabel](int value) {
                const std::size_t total =
                    m_view->totalSteps();
                const std::size_t step =
                    static_cast<std::size_t>(
                        std::llround(
                            static_cast<double>(total) *
                            static_cast<double>(value) /
                            10000.0));
                m_fractionalStep =
                    static_cast<double>(step);
                m_view->setCurrentStep(step);
                stageLabel->setText(
                    QStringLiteral("%1 / %2")
                        .arg(static_cast<qulonglong>(step))
                        .arg(static_cast<qulonglong>(total)));
            });
    connect(&m_timer, &QTimer::timeout, this,
            [this, play, speed, turntable, gpuStatus, info,
             updateStage] {
                const QString materialStatus =
                    m_view->materialStatusString();
                if (materialStatus != m_lastMaterialStatus) {
                    m_lastMaterialStatus = materialStatus;
                    gpuStatus->setText(materialStatus);
                }
                const QString viewError =
                    m_view->errorString();
                if (!viewError.isEmpty() &&
                    viewError != m_lastViewError) {
                    m_playing = false;
                    play->setText(QStringLiteral("播放"));
                    m_lastViewError = viewError;
                    if (m_backend == PocketCamBackend::Vulkan) {
                        m_fallbackReason = viewError;
                        m_backend = PocketCamBackend::OpenGL;
                        done(SwitchBackend);
                        return;
                    }
                    info->setText(
                        info->text() +
                        QStringLiteral("\n仿真显示错误：%1")
                            .arg(viewError));
                }
                if (!m_elapsed.isValid())
                    m_elapsed.start();
                const double seconds =
                    static_cast<double>(m_elapsed.restart()) /
                    1000.0;
                if (turntable->isChecked()) {
                    m_view->rotateTurntable(
                        20.0 * seconds);
                }
                updateStage();
                if (!m_playing || !viewError.isEmpty() || m_view->playbackBusy())
                    return;
                const double stepsPerSecond =
                    60.0 *
                    static_cast<double>(
                        speed->currentData().toInt());
                m_fractionalStep = std::max(
                    m_fractionalStep,
                    static_cast<double>(
                        m_view->currentStep()));
                m_fractionalStep +=
                    seconds * stepsPerSecond;
                const std::size_t step =
                    std::min(
                        m_view->totalSteps(),
                        static_cast<std::size_t>(
                            m_fractionalStep));
                m_view->setCurrentStep(step);
                updateStage();
                if (step >= m_view->totalSteps()) {
                    m_playing = false;
                    play->setText(QStringLiteral("播放"));
                }
            });
    // 播放时钟保持基于时间的 30 Hz，避免排队冗余帧。
    m_timer.start(33);
    m_elapsed.start();
    // All simulation families start paused. Opening a viewer is not a request
    // to advance material; the Play button is the sole playback start action.

    if (vulkan && !visualAcceptanceDirectory.isEmpty()) {
        const QString directory = QDir(visualAcceptanceDirectory).absolutePath();
        if (visualAcceptanceView.valid) {
            path->setChecked(false);
            m_view->widget()->setFixedSize(1024, 1024);
            m_view->setVisualAcceptanceView(visualAcceptanceView);
        }
        m_view->setCurrentStep(m_view->totalSteps());
        auto* captureTimer = new QTimer(this);
        captureTimer->setInterval(100);
        QElapsedTimer deadline;
        deadline.start();
        connect(captureTimer, &QTimer::timeout, this, [this, captureTimer, directory, deadline, info] {
            if (!m_view->errorString().isEmpty() || deadline.elapsed() > 60000) {
                captureTimer->stop();
                info->setText(info->text() + QStringLiteral("\nVulkan 视觉验收失败或超时，未导出完成帧。"));
                return;
            }
            if (m_view->playbackBusy() || m_view->currentStep() != m_view->totalSteps()) return;
            captureTimer->stop();
            QDir().mkpath(directory);
            const QString imagePath = QDir(directory).filePath("vulkan-ipw-final.png");
            const bool saved = m_view->grabFramebuffer().save(imagePath);
            QJsonObject report{{"backend", "Vulkan"}, {"materialSource", "CPU Multi-Dexel"},
                {"completedStep", static_cast<qint64>(m_view->currentStep())},
                {"image", imagePath}, {"imageSaved", saved}};
            QSaveFile file(QDir(directory).filePath("vulkan-ipw-visual-acceptance.json"));
            const auto bytes = QJsonDocument(report).toJson(QJsonDocument::Indented);
            const bool reportSaved = file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size() && file.commit();
            info->setText(info->text() + QStringLiteral("\nVulkan 视觉验收导出：%1").arg(
                saved && reportSaved ? QStringLiteral("成功") : QStringLiteral("失败")));
        });
        captureTimer->start();
    }
    if (!vulkan && !visualAcceptanceDirectory.isEmpty()) {
        const QString captureDirectory =
            QDir(visualAcceptanceDirectory)
                .absolutePath();
        QTimer::singleShot(
            250, this,
            [this, info, updateStage,
             captureDirectory,
             occtMeshFingerprint =
                 std::move(occtMeshFingerprint),
             openGlMeshFingerprint =
                 std::move(openGlMeshFingerprint),
             sharedMeshAccepted,
             finalStockTriangleCount] {
                m_playing = false;
                m_fractionalStep =
                    static_cast<double>(
                        m_view->totalSteps());
                m_view->setCurrentStep(
                    m_view->totalSteps());
                updateStage();

                QDir().mkpath(captureDirectory);
                const QDir directory(
                    captureDirectory);
                const QString openGlImagePath =
                    directory.filePath(
                        QStringLiteral(
                            "opengl-ipw-final.png"));
                const bool openGlImageSaved =
                    m_view->grabFramebuffer().save(
                        openGlImagePath,
                        "PNG");
                const QString occtImagePath =
                    directory.filePath(
                        QStringLiteral(
                            "occt-ipw-final.png"));

                QJsonObject report;
                report.insert(
                    QStringLiteral("schemaVersion"),
                    1);
                report.insert(
                    QStringLiteral(
                        "occtMeshFingerprint"),
                    QString::fromStdString(
                        occtMeshFingerprint));
                report.insert(
                    QStringLiteral(
                        "openGlMeshFingerprint"),
                    QString::fromStdString(
                        openGlMeshFingerprint));
                report.insert(
                    QStringLiteral(
                        "fingerprintsMatch"),
                    sharedMeshAccepted &&
                        !occtMeshFingerprint.empty() &&
                        occtMeshFingerprint ==
                            openGlMeshFingerprint);
                report.insert(
                    QStringLiteral(
                        "sharedMeshAccepted"),
                    sharedMeshAccepted);
                report.insert(
                    QStringLiteral(
                        "finalStockTriangleCount"),
                    static_cast<qint64>(
                        finalStockTriangleCount));
                report.insert(
                    QStringLiteral("completedStep"),
                    static_cast<qint64>(
                        m_view->currentStep()));
                report.insert(
                    QStringLiteral("totalSteps"),
                    static_cast<qint64>(
                        m_view->totalSteps()));
                report.insert(
                    QStringLiteral("occtImage"),
                    occtImagePath);
                report.insert(
                    QStringLiteral(
                        "occtImageExists"),
                    QFileInfo::exists(
                        occtImagePath));
                report.insert(
                    QStringLiteral("openGlImage"),
                    openGlImagePath);
                report.insert(
                    QStringLiteral(
                        "openGlImageSaved"),
                    openGlImageSaved);

                const QString reportPath =
                    directory.filePath(
                        QStringLiteral(
                            "ipw-visual-acceptance.json"));
                QSaveFile reportFile(reportPath);
                bool reportSaved =
                    reportFile.open(
                        QIODevice::WriteOnly);
                if (reportSaved) {
                    const QByteArray document =
                        QJsonDocument(report)
                            .toJson(
                                QJsonDocument::Indented);
                    reportSaved =
                        reportFile.write(document) ==
                            document.size() &&
                        reportFile.commit();
                }
                info->setText(
                    info->text() +
                    QStringLiteral(
                        "\n视觉验收导出：OCCT=%1，OpenGL=%2，报告=%3。")
                        .arg(
                            QFileInfo::exists(
                                occtImagePath)
                                ? QStringLiteral("成功")
                                : QStringLiteral("失败"))
                        .arg(
                            openGlImageSaved
                                ? QStringLiteral("成功")
                                : QStringLiteral("失败"))
                        .arg(
                            reportSaved
                                ? QStringLiteral("成功")
                                : QStringLiteral("失败")));
            });
    }

    if (!vulkan && !visualAcceptanceDirectory.isEmpty() &&
        visualAcceptanceView.valid) {
        const QString captureDirectory =
            QDir(visualAcceptanceDirectory)
                .absolutePath();
        QTimer::singleShot(
            700, this,
            [this, info, path, captureDirectory,
             visualAcceptanceView,
             sharedMeshAccepted] {
                constexpr int imageSize = 1024;
                path->setChecked(false);
                m_view->setPathVisible(false);
                m_view->widget()->setFixedSize(
                    imageSize, imageSize);
                m_view->setVisualAcceptanceView(
                    visualAcceptanceView);
                m_view->widget()->update();

                QTimer::singleShot(
                    250, this,
                    [this, info, captureDirectory,
                      visualAcceptanceView,
                      sharedMeshAccepted] {
                        const QDir directory(
                            captureDirectory);
                        const QString openGlImagePath =
                            directory.filePath(
                                QStringLiteral(
                                    "opengl-ipw-roi.png"));
                        const bool openGlImageSaved =
                            m_view->grabFramebuffer()
                                .save(
                                    openGlImagePath,
                                    "PNG");
                        const QString occtImagePath =
                            directory.filePath(
                                QStringLiteral(
                                    "occt-ipw-roi.png"));

                        const auto pointArray = [](
                            const gp_XYZ& value) {
                            return QJsonArray{
                                value.X(), value.Y(),
                                value.Z()};
                        };
                        QJsonObject report;
                        const double worldUnitsPerPixel =
                            2.0 *
                            visualAcceptanceView
                                .halfHeight /
                            1024.0;
                        const double sampledHausdorff =
                            std::max(
                                visualAcceptanceView
                                    .maximumCurveToPolylineDeviation,
                                visualAcceptanceView
                                    .maximumPolylineToCurveDeviation);
                        const double gateTolerance =
                            1.05 *
                                visualAcceptanceView
                                    .linearDeflection +
                            1.0e-9;
                        const bool contourGatePassed =
                            sharedMeshAccepted &&
                            visualAcceptanceView
                                .boundaryDeviationMeasured &&
                            visualAcceptanceView
                                    .curvedBoundarySampleCount > 0u &&
                            sampledHausdorff <=
                                gateTolerance;
                        report.insert(
                            QStringLiteral(
                                "schemaVersion"),
                            2);
                        report.insert(
                            QStringLiteral(
                                "operationId"),
                            visualAcceptanceView
                                .operationId);
                        report.insert(
                            QStringLiteral(
                                "imageSizePixels"),
                            1024);
                        report.insert(
                            QStringLiteral("center"),
                            pointArray(
                                visualAcceptanceView
                                    .center.XYZ()));
                        report.insert(
                            QStringLiteral(
                                "eyeDirection"),
                            pointArray(
                                visualAcceptanceView
                                    .eyeDirection
                                    .XYZ()));
                        report.insert(
                            QStringLiteral("up"),
                            pointArray(
                                visualAcceptanceView
                                    .up.XYZ()));
                        report.insert(
                            QStringLiteral(
                                "orthographicHalfHeight"),
                            visualAcceptanceView
                                .halfHeight);
                        report.insert(
                            QStringLiteral(
                                "worldUnitsPerPixel"),
                            worldUnitsPerPixel);
                        report.insert(
                            QStringLiteral(
                                "meshLinearDeflectionBoundMm"),
                            visualAcceptanceView
                                .linearDeflection);
                        report.insert(
                            QStringLiteral(
                                "meshAngularDeflectionBoundRad"),
                            visualAcceptanceView
                                .angularDeflection);
                        report.insert(
                            QStringLiteral(
                                "toolpathVisible"),
                            false);
                        report.insert(
                            QStringLiteral(
                                "currentToolVisible"),
                            false);
                        report.insert(
                            QStringLiteral(
                                "sharedMeshAccepted"),
                            sharedMeshAccepted);
                        report.insert(
                            QStringLiteral(
                                "boundaryDeviationMeasured"),
                            visualAcceptanceView
                                .boundaryDeviationMeasured);
                        report.insert(
                            QStringLiteral(
                                "boundaryEdgeCount"),
                            static_cast<qint64>(
                                visualAcceptanceView
                                    .boundaryEdgeCount));
                        report.insert(
                            QStringLiteral(
                                "boundarySegmentCount"),
                            static_cast<qint64>(
                                visualAcceptanceView
                                    .boundarySegmentCount));
                        report.insert(
                            QStringLiteral(
                                "boundarySampleCount"),
                            static_cast<qint64>(
                                visualAcceptanceView
                                    .boundarySampleCount));
                        report.insert(
                            QStringLiteral(
                                "curvedBoundaryEdgeCount"),
                            static_cast<qint64>(
                                visualAcceptanceView
                                    .curvedBoundaryEdgeCount));
                        report.insert(
                            QStringLiteral(
                                "curvedBoundarySampleCount"),
                            static_cast<qint64>(
                                visualAcceptanceView
                                    .curvedBoundarySampleCount));
                        report.insert(
                            QStringLiteral(
                                "maximumCurveToPolylineDeviationMm"),
                            visualAcceptanceView
                                .maximumCurveToPolylineDeviation);
                        report.insert(
                            QStringLiteral(
                                "rmsCurveToPolylineDeviationMm"),
                            visualAcceptanceView
                                .rmsCurveToPolylineDeviation);
                        report.insert(
                            QStringLiteral(
                                "maximumPolylineToCurveDeviationMm"),
                            visualAcceptanceView
                                .maximumPolylineToCurveDeviation);
                        report.insert(
                            QStringLiteral(
                                "maximumCurvedBoundaryDeviationMm"),
                            visualAcceptanceView
                                .maximumCurvedBoundaryDeviation);
                        report.insert(
                            QStringLiteral(
                                "rmsCurvedBoundaryDeviationMm"),
                            visualAcceptanceView
                                .rmsCurvedBoundaryDeviation);
                        report.insert(
                            QStringLiteral(
                                "sampledBoundaryHausdorffEstimateMm"),
                            sampledHausdorff);
                        report.insert(
                            QStringLiteral(
                                "sampledBoundaryHausdorffEstimatePixels"),
                            worldUnitsPerPixel > 0.0
                                ? sampledHausdorff /
                                      worldUnitsPerPixel
                                : 0.0);
                        report.insert(
                            QStringLiteral(
                                "contourGateToleranceMm"),
                            gateTolerance);
                        report.insert(
                            QStringLiteral(
                                "contourGatePassed"),
                            contourGatePassed);
                        report.insert(
                            QStringLiteral(
                                "occtRoiImage"),
                            occtImagePath);
                        report.insert(
                            QStringLiteral(
                                "occtRoiImageExists"),
                            QFileInfo::exists(
                                occtImagePath));
                        report.insert(
                            QStringLiteral(
                                "openGlRoiImage"),
                            openGlImagePath);
                        report.insert(
                            QStringLiteral(
                                "openGlRoiImageSaved"),
                            openGlImageSaved);

                        const QString reportPath =
                            directory.filePath(
                                QStringLiteral(
                                    "ipw-roi-acceptance.json"));
                        QSaveFile reportFile(
                            reportPath);
                        bool reportSaved =
                            reportFile.open(
                                QIODevice::WriteOnly);
                        if (reportSaved) {
                            const QByteArray document =
                                QJsonDocument(report)
                                    .toJson(
                                        QJsonDocument::
                                            Indented);
                            reportSaved =
                                reportFile.write(
                                    document) ==
                                    document.size() &&
                                reportFile.commit();
                        }
                        info->setText(
                            info->text() +
                            QStringLiteral(
                             "\n局部圆角验收：操作 %1，"
                             "共同正交相机，刀路已隐藏；"
                                 "OCCT=%2，OpenGL=%3，报告=%4；"
                                 "轮廓偏差=%5 mm（%6 px），门禁=%7。")
                                .arg(
                                    visualAcceptanceView
                                        .operationId)
                                .arg(
                                    QFileInfo::exists(
                                        occtImagePath)
                                        ? QStringLiteral(
                                              "成功")
                                        : QStringLiteral(
                                              "失败"))
                                .arg(
                                    openGlImageSaved
                                        ? QStringLiteral(
                                              "成功")
                                        : QStringLiteral(
                                              "失败"))
                                 .arg(
                                     reportSaved
                                        ? QStringLiteral(
                                              "成功")
                                          : QStringLiteral(
                                                "失败"))
                                 .arg(
                                     sampledHausdorff,
                                     0, 'g', 6)
                                 .arg(
                                     worldUnitsPerPixel > 0.0
                                         ? sampledHausdorff /
                                               worldUnitsPerPixel
                                         : 0.0,
                                     0, 'g', 6)
                                 .arg(
                                     contourGatePassed
                                         ? QStringLiteral(
                                               "通过")
                                         : QStringLiteral(
                                               "失败")));
                    });
            });
    }
}

} // namespace oap::app::pocket_cam_detail
