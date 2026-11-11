// 独立仿真窗口入口：共享场景/IPW，选择 Vulkan 或 OpenGL 显示后端。
// 把 OCCT 毛坯/刀路/IPW 检查点适配为显示场景，
// 再弹出 PocketCamDialog 执行逐工序播放;返回 false 仅表示场景无法准备或
// 对话框无法启动。
#include "app/ui/dialogs/PocketCamSimulatorDialog.h"
#include <QCoreApplication>
#include "app/ui/dialogs/detail/PocketCamDialog.h"
#include "app/ui/dialogs/detail/PocketCamRenderSupport.h"

#include "toolpath/pocket/simulation/DexelDisplayMesh.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "app/workflows/simulation/IpwDisplayMeshController.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <QString>
#include <QSettings>
#include "app/ui/dialogs/detail/PocketCamView.h"
#include <QVector3D>

namespace oap::app {
using namespace pocket_cam_detail;

bool showPocketCamSimulatorDialog(
    QWidget* parent,
    PocketCamSimulatorInput input,
    QString* errorMessage)
{
    const toolpath::PocketCamSimulationSceneResult prepared =
        toolpath::PocketCamSimulationSceneBuilder().build(
            input.stock, input.authoritativeOperations ? *input.authoritativeOperations : input.operations,
            input.params, input.options);
    if (!prepared.ok()) {
        if (errorMessage) {
            *errorMessage =
                QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(prepared.error).toUtf8().constData());
        }
        return false;
    }

    if (!input.initialIpwStock ||
        !input.initialIpwStock->valid()) {
        if (errorMessage) {
            *errorMessage = QCoreApplication::translate("OpenAutoPath", "加工仿真缺少有效的初始 IPW 检查点；"
                "已取消显示阶段的重复毛坯栅格化。");
        }
        return false;
    }
    if (!input.options.authoritativeIpwPlayback && !input.initialIpwStock->axisAligned(
            prepared.scene.outwardToolAxis)) {
        if (errorMessage) {
            *errorMessage = QCoreApplication::translate("OpenAutoPath", "OpenGL GPU Multi-Dexel 初始 IPW 的刀轴与仿真场景不一致。");
        }
        return false;
    }

    QVector3D stockBoundsMinimum;
    QVector3D stockBoundsMaximum;
    bool stockBoundsValid = false;
    try {
        Bnd_Box stockBounds;
        BRepBndLib::Add(input.stock, stockBounds);
        if (!stockBounds.IsVoid()) {
            double xMinimum = 0.0;
            double yMinimum = 0.0;
            double zMinimum = 0.0;
            double xMaximum = 0.0;
            double yMaximum = 0.0;
            double zMaximum = 0.0;
            stockBounds.Get(
                xMinimum, yMinimum, zMinimum,
                xMaximum, yMaximum, zMaximum);
            stockBoundsMinimum = QVector3D(
                static_cast<float>(xMinimum),
                static_cast<float>(yMinimum),
                static_cast<float>(zMinimum));
            stockBoundsMaximum = QVector3D(
                static_cast<float>(xMaximum),
                static_cast<float>(yMaximum),
                static_cast<float>(zMaximum));
            stockBoundsValid = true;
        }
    } catch (...) {
        // 边界只用于相机适配；下方仍可使用终态网格或刀路边界。
    }

    RenderMesh finalStock;
    QString finalStockError;
    QString finalStockWarning;
    std::string openGlMeshFingerprint;
    bool sharedMeshAccepted = false;
    bool openGlMeshPrepared = false;
    const auto prepareOpenGlMesh = [&] {
        openGlMeshPrepared = true;
        if (!input.finalIpwDisplayMesh.IsNull()) {
            // 优先复用主视口已生成的统一 IPW 显示网格;若其内容指纹与本次仿真
            // 的预期不一致,说明材料状态不同,拒绝共享并回退到材料检查点重建。
            const std::string actualFingerprint =
                toolpath::displayMeshContentFingerprint(
                    input.finalIpwDisplayMesh);
            openGlMeshFingerprint = actualFingerprint;
            if (!input.finalIpwDisplayMeshFingerprint.empty() &&
                actualFingerprint !=
                    input.finalIpwDisplayMeshFingerprint) {
                finalStockWarning = QCoreApplication::translate("OpenAutoPath", "主视口与 OpenGL 的统一 IPW 网格指纹不一致，"
                    "已拒绝该共享网格。");
            } else {
                finalStock = meshTriangulation(
                    input.finalIpwDisplayMesh,
                    finalStockError);
                sharedMeshAccepted =
                    finalStockError.isEmpty() &&
                    !finalStock.empty();
            }
        }
        if (finalStock.empty() && input.finalIpwStock) {
            if (!finalStockError.isEmpty()) {
                finalStockWarning = QCoreApplication::translate("OpenAutoPath", "共享 IPW 网格无法转换，已从材料检查点重建：%1")
                    .arg(finalStockError);
                finalStockError.clear();
            }
            Handle(Poly_Triangulation) triangulation;
            std::string displayMeshError;
            const auto meshOptions = IpwDisplayMeshController::meshOptionsForQuality(
                static_cast<int>(input.options.quality),
                input.finalIpwStock->resolution());
            if (input.finalIpwStock->buildDisplayMesh(
                    triangulation, nullptr,
                    &displayMeshError, {}, meshOptions)) {
                finalStock = meshTriangulation(
                    triangulation, finalStockError);
            } else {
                finalStockError =
                    displayMeshError.empty()
                    ? QCoreApplication::translate("OpenAutoPath", "无法生成统一 IPW 最终显示网格。")
                    : QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(
                          displayMeshError).toUtf8().constData());
            }
        }
    };
    QString targetError;
    RenderMesh target;
    if (!input.target.IsNull()) {
        target = meshShape(
            input.target,
            std::max(
                0.05,
                0.25 * prepared.scene.motionResolution),
            targetError);
        // 目标是一个可选叠加层。损坏的目标网格绝不能
        // 阻止毛坯/刀路仿真打开。
    }
    std::vector<std::string> warnings =
        prepared.warnings;
    if (!targetError.isEmpty())
        warnings.push_back(
            targetError.toStdString());
    QSettings settings;
    const QString requested = qEnvironmentVariable("OAP_CAM_RENDER_BACKEND",
        settings.value("simulation/renderBackend", "opengl").toString());
    PocketCamBackend backend = requested.compare("opengl", Qt::CaseInsensitive) == 0
        ? PocketCamBackend::OpenGL : PocketCamBackend::Vulkan;
    for (;;) {
        // Vulkan derives bounded meshes asynchronously; do not build an unused
        // full final mesh before opening its window.
        if (backend == PocketCamBackend::OpenGL && !openGlMeshPrepared) {
            prepareOpenGlMesh();
            if (!finalStockWarning.isEmpty()) warnings.push_back(finalStockWarning.toStdString());
            if (!finalStockError.isEmpty()) warnings.push_back(
                (QCoreApplication::translate("OpenAutoPath", "最终 IPW 网格不可用，完成帧将继续显示 GPU Multi-Dexel：") + finalStockError).toStdString());
        }
        // Each backend owns a separate top-level window/RHI lifetime. Do not
        // mix QOpenGLWidget and a Vulkan QRhiWidget in an initialized window.
        PocketCamDialog dialog(prepared.scene, warnings, finalStock, target, input.interaction,
            input.initialIpwStock, input.finalIpwStock, input.operationInitialIpwSources,
            stockBoundsMinimum, stockBoundsMaximum, stockBoundsValid,
            input.finalIpwDisplayMeshFingerprint, openGlMeshFingerprint, sharedMeshAccepted,
            finalStock.indices.size() / 3u, input.visualAcceptanceDirectory, input.visualAcceptanceView, parent, backend);
        if (!input.title.isEmpty()) dialog.setWindowTitle(input.title +
            (backend == PocketCamBackend::Vulkan ? " — Vulkan" : " — OpenGL"));
        if (dialog.exec() != PocketCamDialog::SwitchBackend) break;
        backend = dialog.requestedBackend();
        if (!dialog.fallbackReason().isEmpty())
            warnings.push_back((QCoreApplication::translate("OpenAutoPath", "Vulkan 显示失败，已回退 OpenGL：") + dialog.fallbackReason()).toStdString());
        else settings.setValue("simulation/renderBackend", backend == PocketCamBackend::Vulkan ? "vulkan" : "opengl");
    }
    return true;
}

} // namespace oap::app
