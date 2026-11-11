// IpwVisualAcceptanceCapture 实现（oap_app_presentation）。
// 可选的 IPW/OpenGL 视觉验收证据的 OCCT 视口捕获，见 IpwVisualAcceptanceCapture.h。
#include "app/presentation/IpwVisualAcceptanceCapture.h"
#include <QCoreApplication>

#include <algorithm>

#include <QDir>

#include <AIS_ListOfInteractive.hxx>
#include <Graphic3d_Camera.hxx>
#include <Image_AlienPixMap.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>

namespace oap::app::presentation {

IpwVisualAcceptanceCaptureResult
IpwVisualAcceptanceCapture::capture(
    const IpwVisualAcceptanceCaptureRequest& request)
{
    IpwVisualAcceptanceCaptureResult result;
    if (request.directory.trimmed().isEmpty())
        return result;

    QDir directory(request.directory);
    result.directory = directory.absolutePath();
    if (!QDir().mkpath(result.directory)) {
        result.warnings << QCoreApplication::translate("OpenAutoPath", "无法创建 IPW 视觉验收目录：%1").arg(result.directory);
        return result;
    }
    if (request.view.IsNull()) {
        result.warnings << QCoreApplication::translate("OpenAutoPath", "OCCT 视图不可用，无法导出 IPW 验收图。");
        return result;
    }

    const QString viewportPath = directory.filePath(
        QStringLiteral("occt-ipw-final.png"));
    request.view->Redraw();
    const QByteArray encodedViewportPath = viewportPath.toUtf8();
    result.finalViewportSaved =
        request.view->Dump(encodedViewportPath.constData());
    if (!result.finalViewportSaved) {
        result.warnings << QCoreApplication::translate("OpenAutoPath", "OCCT IPW 验收图导出失败：%1").arg(viewportPath);
    }

    result.roiRequested = request.camera.valid;
    if (!result.roiRequested || request.ipwPresentation.IsNull() ||
        request.context.IsNull()) {
        return result;
    }

    // ROI 特写:临时把除 IPW 外的所有对象擦除,再把相机切到正对中心的正交
    // 视图截一张特写。结束时无论成功与否都恢复原相机与显示状态。
    const Handle(Graphic3d_Camera) originalCamera =
        new Graphic3d_Camera(request.view->Camera());
    const bool ipwWasDisplayed =
        request.context->IsDisplayed(request.ipwPresentation);
    AIS_ListOfInteractive displayed;
    AIS_ListOfInteractive hidden;
    request.context->DisplayedObjects(displayed);
    for (AIS_ListOfInteractive::Iterator iterator(displayed);
         iterator.More(); iterator.Next()) {
        const Handle(AIS_InteractiveObject)& object = iterator.Value();
        if (object.get() == request.ipwPresentation.get())
            continue;
        hidden.Append(object);
        request.context->Erase(object, Standard_False);
    }
    request.context->Display(request.ipwPresentation, Standard_False);
    request.context->Redisplay(request.ipwPresentation, Standard_False);

    try {
        Handle(Graphic3d_Camera) roiCamera =
            new Graphic3d_Camera(originalCamera);
        const double eyeDistance = std::max(
            1000.0, 20.0 * request.camera.halfHeight);
        const gp_Pnt eye = request.camera.center.Translated(
            gp_Vec(request.camera.eyeDirection).Multiplied(eyeDistance));
        roiCamera->SetProjectionType(
            Graphic3d_Camera::Projection_Orthographic);
        roiCamera->SetEyeAndCenter(eye, request.camera.center);
        roiCamera->SetUp(request.camera.up);
        roiCamera->SetScale(2.0 * request.camera.halfHeight);
        roiCamera->SetZRange(0.0, 2.0 * eyeDistance);
        request.view->SetCamera(roiCamera);
        request.view->Redraw();

        Image_AlienPixMap image;
        if (request.view->ToPixMap(
                image, std::max(1, request.imageSize),
                std::max(1, request.imageSize),
                Graphic3d_BT_RGB, Standard_True)) {
            const QString roiPath = directory.filePath(
                QStringLiteral("occt-ipw-roi.png"));
            const QByteArray encodedRoiPath = roiPath.toUtf8();
            result.roiSaved = image.Save(
                TCollection_AsciiString(encodedRoiPath.constData()));
        }
    } catch (const Standard_Failure&) {
        result.roiSaved = false;
    } catch (...) {
        result.roiSaved = false;
    }

    request.view->SetCamera(originalCamera);
    for (AIS_ListOfInteractive::Iterator iterator(hidden);
         iterator.More(); iterator.Next()) {
        request.context->Display(iterator.Value(), Standard_False);
    }
    if (!ipwWasDisplayed)
        request.context->Erase(request.ipwPresentation, Standard_False);
    request.context->UpdateCurrentViewer();
    if (!result.roiSaved) {
        result.warnings << QCoreApplication::translate("OpenAutoPath", "OCCT 槽腔 ROI 验收图导出失败。");
    }
    return result;
}

} // namespace oap::app::presentation
