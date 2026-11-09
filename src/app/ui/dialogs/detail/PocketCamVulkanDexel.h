// Native Vulkan material buffers and compact indirect TSDF surface rendering.
// QRhi still owns the device, frame submission and all window/postprocess targets.
#pragma once
#include "app/ui/dialogs/detail/PocketCamRenderSupport.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include <memory>
class QRhi;
class QRhiCommandBuffer;
class QRhiRenderPassDescriptor;
namespace oap::app::pocket_cam_detail {
class PocketCamVulkanDexel {
public:
    PocketCamVulkanDexel();
    ~PocketCamVulkanDexel();
    bool initialize(QRhi* rhi, QRhiRenderPassDescriptor* pass,
        const toolpath::MultiDexelStock& initial, const toolpath::PocketCamSimulationScene& scene, QString& error);
    void advance(QRhiCommandBuffer* commands, const toolpath::PocketCamSimulationScene& scene, std::size_t step);
    // Reuse allocations for authoritative IPW frames. False requires rebuilding
    // for a changed grid/interval capacity; no GPU work is recorded in that case.
    bool uploadStock(QRhi* rhi, QRhiCommandBuffer* commands, const toolpath::MultiDexelStock& stock);
    void draw(QRhiCommandBuffer* commands, const QSize& size, const QMatrix4x4& projection, const QMatrix4x4& model);
    // Explicit synchronous diagnostic, never called by the playback/render loop.
    double volume();
    std::size_t dispatchCount() const;
    QString deviceName() const;
private:
    struct State;
    std::unique_ptr<State> m;
};
}
