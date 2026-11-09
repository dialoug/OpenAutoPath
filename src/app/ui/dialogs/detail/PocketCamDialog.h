// Private dialog composition and playback controls.
#pragma once

#include <QDialog>
#include <QElapsedTimer>
#include <QTimer>
#include "app/ui/dialogs/detail/PocketCamRenderSupport.h"

namespace oap::app::pocket_cam_detail {

class PocketCamView;
enum class PocketCamBackend;

class PocketCamDialog final : public QDialog {
public:
    PocketCamDialog(
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
        QWidget* parent, PocketCamBackend backend);
    static constexpr int SwitchBackend = 42;
    PocketCamBackend requestedBackend() const { return m_backend; }
    QString fallbackReason() const { return m_fallbackReason; }

private:
    double m_motionResolution = 0.5;
    std::size_t m_displayMotionCount = 0u;
    std::size_t m_sourceMotionCount = 0u;
    std::size_t m_totalSimulationSteps = 0u;
    PocketCamView* m_view = nullptr;
    PocketCamBackend m_backend;
    QString m_fallbackReason;
    QTimer m_timer;
    QElapsedTimer m_elapsed;
    double m_fractionalStep = 0.0;
    bool m_playing = false;
    QString m_lastViewError;
    QString m_lastMaterialStatus;
};

} // namespace oap::app::pocket_cam_detail
