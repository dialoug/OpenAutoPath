// Native GPU acceptance: real Vulkan pixels, authoritative IPW seeking and GL fallback.
#include "app/ui/dialogs/detail/PocketCamVulkanWidget.h"
#include "app/ui/dialogs/detail/PocketCamOpenGLWidget.h"
#include "app/ui/dialogs/detail/PocketCamDialog.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <OSD.hxx>
#include <Standard_Failure.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_Viewer.hxx>
#include <V3d_View.hxx>
#include <WNT_Window.hxx>
#include <STEPControl_Reader.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QElapsedTimer>
#include <QThread>
#include <QTimer>
#include <QSettings>
#include <QTemporaryDir>
#include <QPushButton>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <array>

using namespace oap::app;
using namespace oap::app::pocket_cam_detail;
using namespace oap::toolpath;
namespace {
class NativeOcctHost final : public QWidget {
public:
    NativeOcctHost() {
        setAttribute(Qt::WA_PaintOnScreen);
        setAttribute(Qt::WA_NoSystemBackground);
        setAttribute(Qt::WA_OpaquePaintEvent);
    }
    void initialize(const TopoDS_Shape& shape) {
        Handle(OpenGl_GraphicDriver) driver = new OpenGl_GraphicDriver(new Aspect_DisplayConnection);
        m_viewer = new V3d_Viewer(driver);
        m_viewer->SetDefaultLights(); m_viewer->SetLightOn();
        m_context = new AIS_InteractiveContext(m_viewer);
        m_view = m_viewer->CreateView();
        m_view->SetWindow(new WNT_Window(reinterpret_cast<Aspect_Handle>(winId())));
        m_context->Display(new AIS_Shape(shape), Standard_False);
        m_view->MustBeResized(); m_view->FitAll(); m_view->Redraw();
    }
    QPaintEngine* paintEngine() const override { return nullptr; }
protected:
    void paintEvent(QPaintEvent*) override { if (!m_view.IsNull()) m_view->Redraw(); }
    void resizeEvent(QResizeEvent*) override { if (!m_view.IsNull()) m_view->MustBeResized(); }
private:
    Handle(V3d_Viewer) m_viewer;
    Handle(AIS_InteractiveContext) m_context;
    Handle(V3d_View) m_view;
};
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
template<class Predicate> bool wait(Predicate predicate, int timeout = 20000)
{
    QElapsedTimer clock;
    clock.start();
    do {
        QApplication::processEvents();
        if (predicate()) return true;
        QThread::msleep(2);
    } while (clock.elapsed() < timeout);
    return false;
}
QImage capture(PocketCamView& view, const QString& name)
{
    require(wait([&] { return !view.playbackBusy() || !view.errorString().isEmpty(); }), "render timeout");
    if (!view.errorString().isEmpty()) throw std::runtime_error(view.errorString().toStdString());
    auto image = view.grabFramebuffer().convertToFormat(QImage::Format_RGB32);
    require(!image.isNull(), "framebuffer readback failed");
    const QColor background = image.pixelColor(0, 0);
    int foreground = 0;
    for (int y = 0; y < image.height(); ++y)
        for (int x = 0; x < image.width(); ++x) {
            const QColor pixel = image.pixelColor(x, y);
            if (std::abs(pixel.red() - background.red()) + std::abs(pixel.green() - background.green()) +
                std::abs(pixel.blue() - background.blue()) > 35) ++foreground;
        }
    require(foreground > image.width() * image.height() / 100, "blank rendered frame");
    QDir().mkpath("cam-renderer-artifacts");
    require(image.save("cam-renderer-artifacts/" + name + ".png"), "save framebuffer failed");
    std::cout << name.toStdString() << " visible_pixels=" << foreground << '\n';
    return image;
}
void compareImages(const QImage& gl, const QImage& vk, const char* name)
{
    require(gl.size() == vk.size(), "comparison framebuffer sizes differ");
    double difference = 0;
    for (int y = 0; y < gl.height(); ++y)
        for (int x = 0; x < gl.width(); ++x) {
            const auto a = gl.pixelColor(x, y), b = vk.pixelColor(x, y);
            difference += std::abs(a.red() - b.red()) + std::abs(a.green() - b.green()) + std::abs(a.blue() - b.blue());
        }
    difference /= gl.width() * gl.height() * 3.0;
    std::cout << name << " mean pixel difference=" << difference << '\n';
    require(difference < 3.0, "Vulkan camera, lighting or postprocessing differs from OpenGL");
}

void gpuMaterialAcceptance(bool benchmark, bool ball = false, bool vulkanFirst = false)
{
    const double scale = benchmark ? 4.0 : 1.0;
    StockSimulationOptions options;
    options.zMapResolution = benchmark ? 0.1 : 0.25;
    options.runParallel = false;
    const auto shape = BRepPrimAPI_MakeBox(24 * scale, 18 * scale, 8).Shape();
    const auto stock = MultiDexelStock::fromShape(shape, gp_Vec(0,0,1), 4, options);
    require(stock && stock->valid(), "GPU stock fixture failed");
    std::cout << "GPU fixture grid=" << stock->nx() << 'x' << stock->ny() << " resolution=" << stock->resolution() << '\n';
    ToolpathParams params;
    params.tool.diameter = 4;
    params.tool.cornerRadius = ball ? 2 : 0;
    PocketToolpathOperation operation;
    operation.operationId = "gpu-port";
    operation.outwardToolAxis = {0,0,1};
    operation.clearing.ok = true;
    const auto add = [&](ToolpathMoveType type, gp_Pnt from, gp_Pnt to) {
        ToolpathMove move;
        move.type = type;
        move.from = {from.X()*scale, from.Y()*scale, from.Z()};
        move.to = {to.X()*scale, to.Y()*scale, to.Z()};
        operation.clearing.moves.push_back(move);
    };
    add(ToolpathMoveType::Rapid, {3,5,12}, {3,5,10});
    add(ToolpathMoveType::Plunge, {3,5,10}, {3,5,3});
    add(ToolpathMoveType::Cut, {3,5,3}, {21,5,3});
    add(ToolpathMoveType::Cut, {21,5,3}, {21,13,3});
    add(ToolpathMoveType::Cut, {21,13,3}, {3,13,3});
    add(ToolpathMoveType::Retract, {3,13,3}, {3,13,12});
    auto built = PocketCamSimulationSceneBuilder().build(shape, {operation}, params);
    require(built.ok(), built.error.c_str());
    auto scene = built.scene;
    // Exercise the trusted analytic arc path, as well as linear and plunge cuts.
    if (!benchmark && !ball) {
        auto& arc = scene.motions[3];
        arc.analyticPlaybackTrusted = true;
        arc.analyticSweep.kind = PocketCamAnalyticSweepKind::CircularArc;
        arc.analyticSweep.center = {21,9,3};
        arc.analyticSweep.axis = {0,0,1};
        arc.analyticSweep.sweepRadians = 3.141592653589793;
        arc.analyticSweep.direction = ToolpathArcDirection::Clockwise;
    }
    if (!benchmark) {
        // A second profile exercises cached GL uniforms across tool changes and
        // the rewind/re-upload path. Keep the timing workload unchanged.
        auto smaller = scene.tools.front();
        smaller.diameter *= 0.75;
        for (auto& point : smaller.profile) point.first *= 0.75;
        scene.tools.push_back(std::move(smaller));
        scene.motions[4].toolIndex = 1;
        scene.motions[5].toolIndex = 1;
    }
    const auto end = [&](int i) { return scene.motions[i].firstSimulationStep + scene.motions[i].simulationStepCount; };
    const std::vector<std::size_t> checkpoints{0, end(0), end(1), end(1)+2, end(2), end(2)+2, end(3), end(4), end(1)+1, 0, scene.totalSimulationSteps};
    std::vector<QImage> referenceImages;
    std::vector<double> referenceVolumes;
    std::array<std::vector<double>, 2> timings;
    PocketCamVisualAcceptanceView camera;
    camera.valid = true; camera.center = {12*scale,9*scale,4};
    camera.eyeDirection = gp_Dir(1,1,0.8); camera.up = gp_Dir(0,0,1); camera.halfHeight = 18*scale;
    for (int pass=0; pass<2; ++pass) {
        const int backend = vulkanFirst ? 1-pass : pass;
        std::unique_ptr<PocketCamView> view;
        PocketCamOpenGLWidget* gl = nullptr;
        PocketCamVulkanWidget* vk = nullptr;
        const QVector3D maximum{float(24*scale),float(18*scale),8};
        if (backend == 0) {
            gl = new PocketCamOpenGLWidget(scene, {}, {}, {}, stock, {}, {}, {0,0,0}, maximum, true);
            view.reset(gl);
        } else {
            vk = new PocketCamVulkanWidget(scene, {}, {}, stock, {}, {}, {0,0,0}, maximum, true);
            view.reset(vk);
        }
        view->setVisualAcceptanceView(camera);
        view->setModelMode(0); view->setPathVisible(false);
        view->widget()->resize(800,600); view->widget()->show();
        const auto initialImage = capture(*view, backend ? "gpu-vulkan-initial" : "gpu-opengl-initial");
        std::cout << view->materialStatusString().toStdString() << '\n';
        require(backend ? vk->usingGpuMaterial() : gl->usingGpuMaterial(), "GPU test silently fell back to CPU");
        std::cout << (backend ? "Vulkan" : "OpenGL") << " device=" << (backend ? vk->gpuDeviceName() : gl->gpuDeviceName()).toStdString()
            << " framebuffer=" << initialImage.width() << 'x' << initialImage.height() << " ball=" << ball << '\n';
        if (!benchmark) {
            for (std::size_t i=0; i<checkpoints.size(); ++i) {
                view->setCurrentStep(checkpoints[i]);
                const auto image = capture(*view, QString("gpu-%1-%2").arg(backend ? "vulkan" : "opengl").arg(i));
                const auto volume = backend ? vk->displayedVolume() : gl->displayedGpuVolume();
                std::cout << "step=" << checkpoints[i] << " volume=" << volume << '\n';
                if (!backend) { referenceImages.push_back(image); referenceVolumes.push_back(volume); }
                else {
                    require(std::abs(volume-referenceVolumes[i]) < 1e-4, "Vulkan GPU cut volume differs from OpenGL");
                    compareImages(referenceImages[i], image, "GPU OpenGL/Vulkan");
                }
            }
            if (vk) require(vk->gpuDispatchCount() > 0, "Vulkan compute did not execute");
        } else {
            // Identical synchronous frame completion, including framebuffer readback;
            // exclude startup, shader compilation, volume diagnostics and file I/O.
            for (int round=0; round<4; ++round) {
                view->setCurrentStep(0); view->grabFramebuffer();
                for (int frame=1; frame<=40; ++frame) {
                    QElapsedTimer timer; timer.start();
                    view->setCurrentStep(scene.totalSimulationSteps * frame / 41);
                    const auto image = view->grabFramebuffer();
                    const double ms = timer.nsecsElapsed()/1e6;
                    require(!image.isNull() && view->errorString().isEmpty(), "benchmark render failed");
                    if (round) timings[backend].push_back(ms);
                }
            }
            auto& times = timings[backend];
            std::sort(times.begin(), times.end());
            std::cout << (backend ? "Vulkan" : "OpenGL") << " frame_ms median=" << times[times.size()/2]
                << " p95=" << times[times.size()*95/100] << " samples=" << times.size() << '\n';
            const auto finalImage = capture(*view, backend ? "gpu-vulkan-benchmark-final" : "gpu-opengl-benchmark-final");
            const double volume = backend ? vk->displayedVolume() : gl->displayedGpuVolume();
            if (pass == 0) { referenceImages.push_back(finalImage); referenceVolumes.push_back(volume); }
            else {
                require(std::abs(volume-referenceVolumes[0]) < 1e-4, "dense GPU benchmark material differs");
                compareImages(referenceImages[0], finalImage, "dense GPU OpenGL/Vulkan");
            }
        }
    }
    if (benchmark) std::cout << "Vulkan/OpenGL median ratio=" << timings[1][timings[1].size()/2] / timings[0][timings[0].size()/2] << '\n';
}
}

int main(int argc, char** argv)
{
    if (qEnvironmentVariableIntValue("OAP_TEST_NATIVE_GPU") != 1) {
        std::cout << "Set OAP_TEST_NATIVE_GPU=1 on a desktop with Vulkan and OpenGL support.\n";
        return 77;
    }
    OSD::SetSignal(Standard_False);
    std::cout << std::unitbuf;
    std::cout << "Creating native application\n";
    QApplication app(argc, argv);
    const bool switchAfterPlayback = app.arguments().contains("--switch-after-playback");
    const bool switchFirst = switchAfterPlayback || app.arguments().contains("--switch-first");
    try {
        if (app.arguments().contains("--gpu-material") || app.arguments().contains("--gpu-benchmark") || app.arguments().contains("--gpu-benchmark-vulkan-first")) {
            const bool benchmark = !app.arguments().contains("--gpu-material");
            gpuMaterialAcceptance(benchmark, false, app.arguments().contains("--gpu-benchmark-vulkan-first"));
            if (!benchmark) gpuMaterialAcceptance(false, true);
            return 0;
        }
        StockSimulationOptions options;
        options.zMapResolution = 0.5;
        options.runParallel = false;
        TopoDS_Shape shape = BRepPrimAPI_MakeBox(24, 18, 8).Shape();
        TopoDS_Shape targetShape = shape;
        gp_Pnt origin;
        double stockX = 24, stockY = 18, stockZ = 8;
        const QString stepFile = qEnvironmentVariable("OAP_TEST_CAM_STEP");
        if (!stepFile.isEmpty()) {
            require(switchFirst, "external STEP fixture requires a switch test mode");
            STEPControl_Reader reader;
            require(reader.ReadFile(stepFile.toUtf8().constData()) == IFSelect_RetDone && reader.TransferRoots() > 0,
                "cannot load external STEP fixture");
            targetShape = reader.OneShape();
            Bnd_Box bounds;
            BRepBndLib::AddOptimal(targetShape, bounds);
            double x0, y0, z0, x1, y1, z1;
            bounds.Get(x0, y0, z0, x1, y1, z1);
            origin = gp_Pnt(x0 - 1, y0 - 1, z0 - 1);
            stockX = x1 - x0 + 2; stockY = y1 - y0 + 2; stockZ = z1 - z0 + 2;
            shape = BRepPrimAPI_MakeBox(origin, stockX, stockY, stockZ).Shape();
            std::cout << "External STEP: " << stepFile.toStdString() << '\n';
        }
        std::cout << "Rasterizing fixture\n";
        auto initial = MultiDexelStock::fromShape(shape, gp_Vec(0, 0, 1), 4, options);
        require(initial && initial->valid(), "stock fixture failed");
        ToolpathParams params;
        params.tool.diameter = 4;
        PocketToolpathOperation operation;
        operation.operationId = "gpu-acceptance";
        operation.outwardToolAxis = gp_Vec(0, 0, 1);
        operation.clearing.ok = true;
        const auto move = [&](ToolpathMoveType type, gp_Pnt from, gp_Pnt to) {
            ToolpathMove result;
            const auto map = [&](const gp_Pnt& point) {
                return gp_Pnt(origin.X() + point.X() * stockX / 24,
                    origin.Y() + point.Y() * stockY / 18, origin.Z() + point.Z() * stockZ / 8);
            };
            result.type = type; result.from = map(from); result.to = map(to);
            operation.clearing.moves.push_back(result);
        };
        move(ToolpathMoveType::Rapid, {3, 5, 12}, {3, 5, 10});
        move(ToolpathMoveType::Plunge, {3, 5, 10}, {3, 5, 3});
        move(ToolpathMoveType::Cut, {3, 5, 3}, {21, 5, 3});
        move(ToolpathMoveType::Retract, {21, 5, 3}, {21, 5, 12});
        std::cout << "Applying fixture cut\n";
        auto finalStock = initial->applyPocket(operation, params, 0, options).stock;
        require(finalStock && finalStock->volume() < initial->volume(), "cut fixture failed");
        auto source = std::make_shared<IpwTransientFrameSource>();
        source->parentStock = initial;
        source->operations = std::make_shared<const std::vector<PocketToolpathOperation>>(std::vector{operation});
        source->params = params; source->options = options;
        std::vector<std::shared_ptr<const IpwTransientFrameSource>> sources{source};
        std::cout << "Preparing scene\n";
        auto built = PocketCamSimulationSceneBuilder().build(shape, *source->operations, params);
        require(built.ok(), built.error.c_str());
        auto scene = built.scene;
        require(scene.motions.size() == 4, "unexpected fixture motion count");
        QString error;
        const auto target = meshShape(targetShape, 0.1, error);
        require(error.isEmpty(), "target fixture mesh failed");
        QImage vulkanTarget, vulkanMaterial, vulkanAo;
        PocketCamVisualAcceptanceView comparisonView;
        comparisonView.valid = true;
        comparisonView.center = {12, 9, 4};
        comparisonView.eyeDirection = gp_Dir(1, 1, 0.8);
        comparisonView.up = gp_Dir(0, 0, 1);
        comparisonView.halfHeight = 18;
        if (!switchFirst) {
            std::cout << "Creating Vulkan widget\n";
            auto authoritativeScene = scene;
            authoritativeScene.authoritativeIpwPlayback = true;
            PocketCamVulkanWidget view(authoritativeScene, target, {}, initial, finalStock, sources, {0,0,0}, {24,18,8}, true);
            std::cout << "Showing Vulkan widget\n";
            view.resize(800, 600); view.show();
            const auto first = capture(view, "vulkan-initial");
            require(view.usingVulkan(), "test must execute the actual Vulkan backend");
            require(view.usingGpuMaterial(), "authoritative IPW must use GPU surfaces on this device");
            require(view.currentStep() == 0 && view.displayedVolume() == initial->volume(), "viewer must start paused at initial stock");
            const auto end = [&](int i) { return scene.motions[i].firstSimulationStep + scene.motions[i].simulationStepCount; };
            view.setCurrentStep(end(0)); capture(view, "vulkan-rapid");
            require(view.displayedVolume() == initial->volume(), "rapid must not remove material");
            const auto& cuttingMotion = scene.motions[2];
            const auto middle = cuttingMotion.firstSimulationStep + cuttingMotion.simulationStepCount / 2;
            view.setCurrentStep(middle); capture(view, "vulkan-partial-cut");
            auto partialOperation = operation;
            partialOperation.clearing.moves.resize(3);
            const double fraction = static_cast<double>(middle - cuttingMotion.firstSimulationStep) / cuttingMotion.simulationStepCount;
            partialOperation.clearing.moves[2].to = interpolate(cuttingMotion.move.from, cuttingMotion.move.to, fraction);
            const auto partial = initial->applyPocketPrefix(partialOperation, params, 0, 3, options).stock;
            std::cout << "partial step=" << view.currentStep() << "/" << middle
                << " volume=" << view.displayedVolume() << " reference=" << (partial ? partial->volume() : -1) << '\n';
            require(view.currentStep() == middle && partial && view.displayedVolume() == partial->volume(),
                "partial step must cut only up to the requested tool position");
            require(view.displayedVolume() > finalStock->volume(), "partial playback jumped to the endpoint");
            const auto earlier = cuttingMotion.firstSimulationStep + cuttingMotion.simulationStepCount / 4;
            view.setCurrentStep(earlier); capture(view, "vulkan-partial-rewind");
            require(view.currentStep() == earlier && view.displayedVolume() > partial->volume(),
                "rewinding within a move retained future material removal");
            view.setCurrentStep(end(2));
            const auto cut = capture(view, "vulkan-cut");
            const auto reference = initial->applyPocketPrefix(operation, params, 0, 3, options).stock;
            require(reference && view.displayedVolume() == reference->volume(), "displayed cut differs from CPU IPW reference");
            require(cut != first, "cut frame did not change");
            view.setCurrentStep(end(1));
            view.setCurrentStep(0);
            capture(view, "vulkan-rewind");
            require(view.currentStep() == 0 && view.displayedVolume() == initial->volume(), "stale worker frame overwrote rewind");
            view.setCurrentStep(view.totalSteps()); capture(view, "vulkan-final");
            require(view.displayedVolume() == finalStock->volume(), "final checkpoint differs");
            view.setOrthographicProjection(true); view.rotateTurntable(45); view.resize(900, 500);
            view.setPathVisible(false); view.setModelMode(2);
            capture(view, "vulkan-camera-overlay");
            view.setVisualAcceptanceView(comparisonView);
            view.resize(800, 600);
            const auto noAo = capture(view, "vulkan-material-no-ao");
            vulkanMaterial = noAo;
            view.setAmbientOcclusionEnabled(true);
            const auto withAo = capture(view, "vulkan-material-ao");
            vulkanAo = withAo;
            require(view.supportsAmbientOcclusion() && noAo != withAo, "Vulkan AO must affect cavity shading");
            view.setAmbientOcclusionEnabled(false);
            view.setModelMode(1);
            vulkanTarget = capture(view, "vulkan-target-comparison");
            view.setCurrentStep(end(1)); // Destruction must join a pending material job.
        }
        if (!switchFirst) {
            Handle(Poly_Triangulation) finalMesh;
            require(finalStock->buildDisplayMesh(finalMesh, nullptr, nullptr, {},
                IpwDisplayMeshController::meshOptionsForQuality(static_cast<int>(scene.quality), finalStock->resolution())),
                "comparison final mesh failed");
            const auto finalGeometry = meshTriangulation(finalMesh, error);
            require(error.isEmpty(), "comparison final mesh conversion failed");
            auto authoritativeScene = scene;
            authoritativeScene.authoritativeIpwPlayback = true;
            PocketCamOpenGLWidget view(authoritativeScene, finalGeometry, target, {}, initial, finalStock, sources, {0,0,0}, {24,18,8}, true);
            view.resize(800, 600); view.show();
            capture(view, "opengl-initial");
            require(view.currentStep() == 0, "OpenGL must remain paused");
            view.setVisualAcceptanceView(comparisonView);
            view.setModelMode(1);
            const auto glTarget = capture(view, "opengl-target-comparison");
            compareImages(glTarget, vulkanTarget, "OpenGL/Vulkan target");
            view.setModelMode(0);
            view.setCurrentStep(view.totalSteps());
            compareImages(capture(view, "opengl-material-no-ao"), vulkanMaterial, "OpenGL/Vulkan final stock");
            view.setAmbientOcclusionEnabled(true);
            compareImages(capture(view, "opengl-material-ao"), vulkanAo, "OpenGL/Vulkan final stock AO");
        }
        if (!switchFirst) {
            // A single display chord covers unequal original moves. Half of its
            // progress must map by source distance, not by source-move count.
            auto denseOperation = operation;
            auto secondCut = denseOperation.clearing.moves[2];
            denseOperation.clearing.moves[2].to = {6, 5, 3};
            secondCut.from = {6, 5, 3}; secondCut.to = {9, 5, 3};
            denseOperation.clearing.moves.insert(denseOperation.clearing.moves.begin() + 3, secondCut);
            secondCut.from = {9, 5, 3}; secondCut.to = {21, 5, 3};
            denseOperation.clearing.moves.insert(denseOperation.clearing.moves.begin() + 4, secondCut);
            auto denseSource = std::make_shared<IpwTransientFrameSource>(*source);
            denseSource->operations = std::make_shared<const std::vector<PocketToolpathOperation>>(std::vector{denseOperation});
            PocketCamSimulationOptions simplifiedOptions;
            simplifiedOptions.authoritativeIpwPlayback = true;
            simplifiedOptions.pathSimplificationTolerance = 0.1;
            auto simplified = PocketCamSimulationSceneBuilder().build(shape, *denseSource->operations, params, simplifiedOptions);
            require(simplified.ok() && simplified.scene.motions.size() == 4, "simplified playback fixture failed");
            const auto& motion = simplified.scene.motions[2];
            require(motion.sourceMoveEndCount == 5, "simplified motion lost its source prefix");
            PocketCamVulkanWidget view(simplified.scene, target, {}, initial, finalStock, {denseSource},
                {0,0,0}, {24,18,8}, true);
            view.resize(800, 600); view.show();
            const auto middle = motion.firstSimulationStep + motion.simulationStepCount / 2;
            view.setCurrentStep(middle);
            capture(view, "vulkan-simplified-partial");
            denseOperation.clearing.moves[4].to = {12, 5, 3};
            const auto reference = initial->applyPocketPrefix(denseOperation, params, 0, 5, options).stock;
            require(reference && view.currentStep() == middle && view.displayedVolume() == reference->volume(),
                "simplified display motion did not map to original partial material");
        }
        if (!switchFirst) {
            // Different setup axis plus a ball cutter, using the global IPW source layout.
            PocketToolpathOperation side = operation;
            side.operationId = "ball-side";
            side.outwardToolAxis = gp_Vec(-1, 0, 0);
            side.tool.valid = true;
            side.tool.toolId = "ball-side";
            side.tool.toolNumber = 2;
            side.tool.toolType = "BallEndMill";
            side.tool.diameter = 4;
            side.tool.cornerRadius = 2;
            side.tool.cuttingEdgeLength = params.tool.cuttingEdgeLength;
            side.clearing.moves.clear();
            ToolpathMove cut;
            cut.type = ToolpathMoveType::Cut;
            cut.from = {3, 3, 4}; cut.to = {3, 15, 4};
            side.clearing.moves.push_back(cut);
            cut.from = cut.to; cut.to = {3, 15, 6};
            side.clearing.moves.push_back(cut);
            auto operations = std::make_shared<const std::vector<PocketToolpathOperation>>(std::vector{operation, side});
            ToolpathParams ballParams = params;
            ballParams.tool.cornerRadius = 2;
            DexelToolProfileDefinition flat, ball;
            flat.diameter = ball.diameter = 4;
            flat.cuttingLength = ball.cuttingLength = params.tool.cuttingEdgeLength;
            ball.cornerRadius = 2;
            auto sideFinal = finalStock->applyToolpathPrefix(side, ballParams, ball, 1, 2, options).stock;
            require(sideFinal && sideFinal->volume() < finalStock->volume(), "mixed-axis fixture failed");
            std::vector<std::shared_ptr<const IpwTransientFrameSource>> mixedSources;
            for (int i = 0; i < 2; ++i) {
                auto mixed = std::make_shared<IpwTransientFrameSource>();
                mixed->parentStock = i == 0 ? initial : finalStock;
                mixed->operations = operations;
                mixed->toolProfiles = {flat, ball};
                mixed->operationParameters = {params, ballParams};
                mixed->params = params; mixed->options = options;
                mixed->firstMoveCount = mixed->completedMoveCount = i == 0 ? 0 : 4;
                mixedSources.push_back(mixed);
            }
            PocketCamSimulationOptions sceneOptions;
            sceneOptions.authoritativeIpwPlayback = true;
            auto mixedScene = PocketCamSimulationSceneBuilder().build(shape, *operations, params, sceneOptions);
            require(mixedScene.ok(), mixedScene.error.c_str());
            PocketCamVulkanWidget view(mixedScene.scene, target, {}, initial, sideFinal, mixedSources,
                {0,0,0}, {24,18,8}, true);
            view.resize(800, 600); view.show();
            const auto& motion = mixedScene.scene.motions.at(4);
            const auto middle = motion.firstSimulationStep + motion.simulationStepCount / 2;
            view.setCurrentStep(middle);
            capture(view, "vulkan-mixed-partial-ball-side");
            auto partialSide = side;
            partialSide.clearing.moves.resize(1);
            partialSide.clearing.moves[0].to = interpolate(motion.move.from, motion.move.to,
                static_cast<double>(middle - motion.firstSimulationStep) / motion.simulationStepCount);
            const auto partialReference = finalStock->applyToolpathPrefix(partialSide, ballParams, ball, 1, 1, options).stock;
            require(partialReference && view.currentStep() == middle && view.displayedVolume() == partialReference->volume(),
                "mixed-axis partial ball frame differs");
            view.setCurrentStep(motion.firstSimulationStep + motion.simulationStepCount);
            capture(view, "vulkan-mixed-ball-side");
            const auto reference = finalStock->applyToolpathPrefix(side, ballParams, ball, 1, 1, options).stock;
            require(reference && view.displayedVolume() == reference->volume(), "mixed-axis ball frame differs");
            view.setCurrentStep(view.totalSteps()); capture(view, "vulkan-mixed-final");
            require(view.displayedVolume() == sideFinal->volume(), "mixed final differs");
        }
        if (!switchFirst) {
            PocketCamDialog dialog(scene, {}, {}, target, {}, initial, finalStock, sources,
                {0,0,0}, {24,18,8}, true, {}, {}, false, 0, {}, {}, nullptr, PocketCamBackend::Vulkan);
            QTimer::singleShot(500, &dialog, [&] {
                auto* widget = dialog.findChild<QRhiWidget*>();
                require(widget != nullptr, "missing Vulkan child");
                // Simulate a device creation/loss notification through Qt's real failure signal.
                QMetaObject::invokeMethod(widget, "renderFailed", Qt::DirectConnection);
            });
            QTimer::singleShot(10000, &dialog, &QDialog::reject);
            require(dialog.exec() == PocketCamDialog::SwitchBackend && dialog.requestedBackend() == PocketCamBackend::OpenGL &&
                !dialog.fallbackReason().isEmpty(), "Vulkan failure did not request OpenGL fallback");
        }
        {
            // Exercise the public modal loop, not only the low-level failure return code.
            QTemporaryDir settingsDirectory;
            QSettings::setDefaultFormat(QSettings::IniFormat);
            QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, settingsDirectory.path());
            app.setOrganizationName("OAPRendererTest"); app.setApplicationName("NativeGPU");
            qputenv("OAP_CAM_RENDER_BACKEND", switchFirst ? "opengl" : "vulkan");
            PocketCamSimulatorInput input;
            input.stock = shape; input.target = targetShape; input.operations = {operation};
            input.params = params; input.initialIpwStock = initial; input.finalIpwStock = finalStock;
            input.operationInitialIpwSources = sources;
            NativeOcctHost owner;
            owner.resize(400, 300); owner.show();
            owner.initialize(targetShape);
            int phase = switchFirst ? (switchAfterPlayback ? -2 : -1) : 0;
            QTimer driver;
            QElapsedTimer deadline; deadline.start();
            QObject::connect(&driver, &QTimer::timeout, [&] {
                auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
                if (!dialog) return;
                if (deadline.elapsed() > 20000) { dialog->reject(); return; }
                if (phase == -2) {
                    if (auto* gl = dialog->findChild<QOpenGLWidget*>(); gl && gl->isValid()) {
                        for (auto* button : dialog->findChildren<QPushButton*>()) {
                            if (button->text() == QStringLiteral("播放")) {
                                button->click(); phase = -1; break;
                            }
                        }
                    }
                } else if (phase == -1) {
                    if (auto* gl = dialog->findChild<QOpenGLWidget*>(); gl && gl->isValid()) {
                        auto* view = dynamic_cast<PocketCamOpenGLWidget*>(gl);
                        if (switchAfterPlayback && (!view || view->currentStep() == 0 || view->playbackBusy())) return;
                        auto* choice = dialog->findChild<QComboBox*>("simulationRenderBackend");
                        phase = 0;
                        choice->setCurrentIndex(choice->findData(static_cast<int>(PocketCamBackend::Vulkan)));
                    }
                } else if (phase == 0) {
                    if (auto* rhiView = dialog->findChild<QRhiWidget*>()) {
                        auto* view = dynamic_cast<PocketCamVulkanWidget*>(rhiView);
                        if (!view || view->playbackBusy()) return;
                        phase = 1;
                        QMetaObject::invokeMethod(rhiView, "renderFailed", Qt::DirectConnection);
                    }
                } else if (phase == 1 && dialog->findChild<QOpenGLWidget*>()) {
                    auto* choice = dialog->findChild<QComboBox*>("simulationRenderBackend");
                    phase = 2;
                    choice->setCurrentIndex(choice->findData(static_cast<int>(PocketCamBackend::Vulkan)));
                } else if (phase == 2) {
                    if (auto* view = dynamic_cast<PocketCamVulkanWidget*>(dialog->findChild<QRhiWidget*>())) {
                        if (view->playbackBusy()) return;
                        if (view->usingVulkan() && view->currentStep() == 0) {
                            capture(*view, switchAfterPlayback ? "switch-after-playback" : "switch-initial");
                            phase = 3;
                        }
                        dialog->accept();
                    }
                }
            });
            driver.start(50);
            require(showPocketCamSimulatorDialog(&owner, std::move(input), &error), "public simulator failed");
            require(phase == 3, "Vulkan/OpenGL window replacement failed");
            qunsetenv("OAP_CAM_RENDER_BACKEND");
        }
        std::cout << "Vulkan IPW, rewind, framebuffer, OpenGL and failure routing passed\n";
    } catch (const Standard_Failure& error) { std::cerr << "OCCT exception: " << error.GetMessageString() << '\n'; return 1; }
      catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
