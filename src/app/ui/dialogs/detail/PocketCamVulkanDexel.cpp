#include "app/ui/dialogs/detail/PocketCamVulkanDexel.h"
#include <rhi/qrhi.h>
#include <rhi/qrhi_platform.h>
#include <QVulkanFunctions>
#include <QFile>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace oap::app::pocket_cam_detail {
namespace {
void checked(VkResult result, const char* operation)
{
    if (result != VK_SUCCESS) throw std::runtime_error(std::string(operation) + ": " + std::to_string(result));
}
struct Buffer { VkBuffer handle = VK_NULL_HANDLE; VkDeviceMemory memory = VK_NULL_HANDLE; VkDeviceSize size = 0; void* mapped = nullptr; };
struct Grid {
    int sizes[4]{};
    int render[4]{};
    float extent[4]{}, bounds[4]{}, origin[4]{}, u[4]{}, v[4]{}, w[4]{};
};
struct Motion {
    float from[4]{}, to[4]{}, arc[4]{};
    int dispatch[4]{}, tool[4]{};
};
static_assert(sizeof(Grid) == 128 && sizeof(Motion) == 80);
}

struct PocketCamVulkanDexel::State {
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    QVulkanDeviceFunctions* f = nullptr;
    VkPhysicalDeviceMemoryProperties memory{};
    VkPhysicalDeviceProperties properties{};
    VkCommandPool pool = VK_NULL_HANDLE;
    VkDescriptorPool descriptors = VK_NULL_HANDLE;
    std::array<VkDescriptorSetLayout, 3> setLayouts{};
    std::array<VkPipelineLayout, 3> layouts{};
    std::array<VkDescriptorSet, 3> sets{};
    std::array<VkPipeline, 4> pipelines{}; // cut, classify, coarse, fine
    std::array<Buffer, 10> buffers{};
    std::array<Buffer, 3> baseline{};
    std::vector<Buffer> staging;
    std::vector<Buffer> frameUploads;
    Grid grid;
    std::vector<std::size_t> ends;
    std::vector<int> profileOffsets;
    gp_Pnt origin;
    gp_Vec u, v, w;
    std::size_t applied = 0, dispatches = 0;
    bool dirty = true;

    ~State() {
        if (!f) return;
        // Only resource teardown synchronizes the whole device. Normal frames
        // are recorded into Qt's queue with explicit GPU-side dependencies.
        f->vkDeviceWaitIdle(device);
        for (auto pipeline : pipelines) if (pipeline) f->vkDestroyPipeline(device, pipeline, nullptr);
        for (auto layout : layouts) if (layout) f->vkDestroyPipelineLayout(device, layout, nullptr);
        if (descriptors) f->vkDestroyDescriptorPool(device, descriptors, nullptr);
        for (auto layout : setLayouts) if (layout) f->vkDestroyDescriptorSetLayout(device, layout, nullptr);
        for (auto& b : buffers) destroy(b);
        for (auto& b : baseline) destroy(b);
        for (auto& b : staging) destroy(b);
        for (auto& b : frameUploads) destroy(b);
        if (pool) f->vkDestroyCommandPool(device, pool, nullptr);
    }
    void destroy(Buffer& b) {
        if (b.mapped) f->vkUnmapMemory(device, b.memory);
        if (b.handle) f->vkDestroyBuffer(device, b.handle, nullptr);
        if (b.memory) f->vkFreeMemory(device, b.memory, nullptr);
        b = {};
    }
    void allocate(Buffer& b, VkDeviceSize bytes, VkBufferUsageFlags usage, bool host = false) {
        b.size = bytes;
        VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        info.size = bytes; info.usage = usage; info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        checked(f->vkCreateBuffer(device, &info, nullptr, &b.handle), "vkCreateBuffer");
        VkMemoryRequirements requirements{};
        f->vkGetBufferMemoryRequirements(device, b.handle, &requirements);
        const VkMemoryPropertyFlags flags = host ? VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT : VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
        uint32_t type = memory.memoryTypeCount;
        for (uint32_t i = 0; i < memory.memoryTypeCount; ++i)
            if ((requirements.memoryTypeBits & (1u << i)) && (memory.memoryTypes[i].propertyFlags & flags) == flags) { type = i; break; }
        if (type == memory.memoryTypeCount) throw std::runtime_error("No compatible Vulkan memory type");
        VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
        checked(f->vkAllocateMemory(device, &allocation, nullptr, &b.memory), "vkAllocateMemory");
        checked(f->vkBindBufferMemory(device, b.handle, b.memory, 0), "vkBindBufferMemory");
        if (host) checked(f->vkMapMemory(device, b.memory, 0, bytes, 0, &b.mapped), "vkMapMemory");
    }
    VkCommandBuffer beginTransfer() {
        VkCommandBufferAllocateInfo info{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        info.commandPool = pool; info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; info.commandBufferCount = 1;
        VkCommandBuffer cb;
        checked(f->vkAllocateCommandBuffers(device, &info, &cb), "vkAllocateCommandBuffers");
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        checked(f->vkBeginCommandBuffer(cb, &begin), "vkBeginCommandBuffer");
        return cb;
    }
    void finishTransfer(VkCommandBuffer cb) {
        checked(f->vkEndCommandBuffer(cb), "vkEndCommandBuffer");
        VkFenceCreateInfo info{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        VkFence fence;
        checked(f->vkCreateFence(device, &info, nullptr, &fence), "vkCreateFence");
        VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submit.commandBufferCount = 1; submit.pCommandBuffers = &cb;
        const auto submitted = f->vkQueueSubmit(queue, 1, &submit, fence);
        const auto completed = submitted == VK_SUCCESS ? f->vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX) : submitted;
        f->vkDestroyFence(device, fence, nullptr);
        f->vkFreeCommandBuffers(device, pool, 1, &cb);
        checked(completed, "Vulkan transfer submission");
    }
    void barrier(VkCommandBuffer cb, VkPipelineStageFlags src, VkAccessFlags srcAccess,
                 VkPipelineStageFlags dst, VkAccessFlags dstAccess) {
        VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        b.srcAccessMask = srcAccess; b.dstAccessMask = dstAccess;
        f->vkCmdPipelineBarrier(cb, src, dst, 0, 1, &b, 0, nullptr, 0, nullptr);
    }
    void copy(VkCommandBuffer cb, const Buffer& from, const Buffer& to) {
        const VkBufferCopy region{0, 0, from.size};
        f->vkCmdCopyBuffer(cb, from.handle, to.handle, 1, &region);
    }
    void upload(VkCommandBuffer cb, int index, VkDeviceSize bytes, const void* data, VkBufferUsageFlags usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) {
        if (usage == VK_BUFFER_USAGE_STORAGE_BUFFER_BIT && bytes > properties.limits.maxStorageBufferRange)
            throw std::runtime_error("Vulkan material exceeds maxStorageBufferRange");
        allocate(buffers[index], bytes, usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
            (index == 4 ? VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT : 0));
        if (data) {
            staging.emplace_back();
            allocate(staging.back(), bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, true);
            std::memcpy(staging.back().mapped, data, static_cast<std::size_t>(bytes));
            copy(cb, staging.back(), buffers[index]);
        }
    }
    VkShaderModule shader(const char* name) {
        QFile file(QString::fromLatin1(name));
        if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Vulkan material shader missing");
        const auto shader = QShader::fromSerialized(file.readAll());
        QByteArray code;
        for (const auto& key : shader.availableShaders())
            if (key.source() == QShader::SpirvShader) { code = shader.shader(key).shader(); break; }
        if (code.isEmpty()) throw std::runtime_error("Vulkan material SPIR-V missing");
        VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        info.codeSize = static_cast<std::size_t>(code.size()); info.pCode = reinterpret_cast<const uint32_t*>(code.constData());
        VkShaderModule module;
        checked(f->vkCreateShaderModule(device, &info, nullptr, &module), "vkCreateShaderModule");
        return module;
    }
    void createSet(int slot, const std::vector<int>& bindings, VkShaderStageFlags stage, uint32_t pushBytes) {
        std::vector<VkDescriptorSetLayoutBinding> entries;
        for (int binding : bindings) entries.push_back({static_cast<uint32_t>(binding), binding == 7 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, stage, nullptr});
        VkDescriptorSetLayoutCreateInfo info{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        info.bindingCount = static_cast<uint32_t>(entries.size()); info.pBindings = entries.data();
        checked(f->vkCreateDescriptorSetLayout(device, &info, nullptr, &setLayouts[slot]), "vkCreateDescriptorSetLayout");
        VkPipelineLayoutCreateInfo layout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        layout.setLayoutCount = 1; layout.pSetLayouts = &setLayouts[slot];
        const VkPushConstantRange push{stage, 0, pushBytes};
        if (pushBytes) { layout.pushConstantRangeCount = 1; layout.pPushConstantRanges = &push; }
        checked(f->vkCreatePipelineLayout(device, &layout, nullptr, &layouts[slot]), "vkCreatePipelineLayout");
        VkDescriptorSetAllocateInfo allocation{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocation.descriptorPool = descriptors; allocation.descriptorSetCount = 1; allocation.pSetLayouts = &setLayouts[slot];
        checked(f->vkAllocateDescriptorSets(device, &allocation, &sets[slot]), "vkAllocateDescriptorSets");
        for (int binding : bindings) {
            const VkDescriptorBufferInfo buffer{buffers[binding].handle, 0, buffers[binding].size};
            VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            write.dstSet = sets[slot]; write.dstBinding = binding; write.descriptorCount = 1;
            write.descriptorType = binding == 7 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            write.pBufferInfo = &buffer;
            f->vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        }
    }
    void createPipelines(VkRenderPass pass) {
        const std::array<VkDescriptorPoolSize, 2> sizes{{{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 20}, {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 3}}};
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.maxSets = 3; poolInfo.poolSizeCount = 2; poolInfo.pPoolSizes = sizes.data();
        checked(f->vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptors), "vkCreateDescriptorPool");
        createSet(0, {0,1,5,6,7,8}, VK_SHADER_STAGE_COMPUTE_BIT, sizeof(Motion));
        createSet(1, {0,1,2,3,4,7,9}, VK_SHADER_STAGE_COMPUTE_BIT, 0);
        createSet(2, {0,1,2,3,5,7,9}, VK_SHADER_STAGE_VERTEX_BIT, 128);
        for (int i = 0; i < 2; ++i) {
            auto module = shader(i == 0 ? ":/cam/dexel_cut.comp.qsb" : ":/cam/dexel_surface.comp.qsb");
            VkComputePipelineCreateInfo info{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
            info.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, module, "main", nullptr};
            info.layout = layouts[i];
            const auto result = f->vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &pipelines[i]);
            f->vkDestroyShaderModule(device, module, nullptr);
            checked(result, "vkCreateComputePipelines");
        }
        auto vertex = shader(":/cam/dexel.vert.qsb");
        VkShaderModule fragment;
        try { fragment = shader(":/cam/dexel.frag.qsb"); }
        catch (...) { f->vkDestroyShaderModule(device, vertex, nullptr); throw; }
        VkPipelineVertexInputStateCreateInfo vertexInput{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        VkPipelineInputAssemblyStateCreateInfo assembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo viewport{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        viewport.viewportCount = viewport.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        raster.polygonMode = VK_POLYGON_MODE_FILL; raster.cullMode = VK_CULL_MODE_NONE; raster.lineWidth = 1;
        VkPipelineMultisampleStateCreateInfo samples{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        samples.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo depth{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
        depth.depthTestEnable = depth.depthWriteEnable = VK_TRUE; depth.depthCompareOp = VK_COMPARE_OP_LESS;
        VkPipelineColorBlendAttachmentState attachment{}; attachment.colorWriteMask = 15;
        VkPipelineColorBlendStateCreateInfo blend{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        blend.attachmentCount = 1; blend.pAttachments = &attachment;
        const VkDynamicState states[]{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dynamic.dynamicStateCount = 2; dynamic.pDynamicStates = states;
        for (int fine = 0; fine < 2; ++fine) {
            const VkBool32 enabled = fine;
            const VkSpecializationMapEntry entry{0, 0, sizeof(enabled)};
            const VkSpecializationInfo specialization{1, &entry, sizeof(enabled), &enabled};
            const VkPipelineShaderStageCreateInfo stages[]{
                {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, vertex, "main", &specialization},
                {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, fragment, "main", nullptr}};
            VkGraphicsPipelineCreateInfo info{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
            info.stageCount = 2; info.pStages = stages; info.pVertexInputState = &vertexInput; info.pInputAssemblyState = &assembly;
            info.pViewportState = &viewport; info.pRasterizationState = &raster; info.pMultisampleState = &samples;
            info.pDepthStencilState = &depth; info.pColorBlendState = &blend; info.pDynamicState = &dynamic;
            info.layout = layouts[2]; info.renderPass = pass;
            const auto result = f->vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &pipelines[2 + fine]);
            if (result != VK_SUCCESS) {
                f->vkDestroyShaderModule(device, vertex, nullptr); f->vkDestroyShaderModule(device, fragment, nullptr);
                checked(result, "vkCreateGraphicsPipelines");
            }
        }
        f->vkDestroyShaderModule(device, vertex, nullptr); f->vkDestroyShaderModule(device, fragment, nullptr);
    }
    QVector3D local(const gp_Pnt& p) const { const gp_Vec d(origin, p); return {float(d.Dot(u)), float(d.Dot(v)), float(d.Dot(w))}; }
    void cut(VkCommandBuffer cb, const toolpath::PocketCamSimulationScene& scene, const toolpath::PocketCamMotion& motion, double from, double to);
};

PocketCamVulkanDexel::PocketCamVulkanDexel() = default;
PocketCamVulkanDexel::~PocketCamVulkanDexel() = default;

bool PocketCamVulkanDexel::initialize(QRhi* rhi, QRhiRenderPassDescriptor* pass,
    const toolpath::MultiDexelStock& stock, const toolpath::PocketCamSimulationScene& scene, QString& error)
{
    try {
        m = std::make_unique<State>();
        auto& s = *m;
        const auto* native = static_cast<const QRhiVulkanNativeHandles*>(rhi->nativeHandles());
        s.device = native->dev; s.queue = native->gfxQueue;
        s.f = native->inst->deviceFunctions(s.device);
        native->inst->functions()->vkGetPhysicalDeviceMemoryProperties(native->physDev, &s.memory);
        native->inst->functions()->vkGetPhysicalDeviceProperties(native->physDev, &s.properties);
        const auto& limits = s.properties.limits;
        if (limits.maxPerStageDescriptorStorageBuffers < 6 || limits.maxDescriptorSetStorageBuffers < 6)
            throw std::runtime_error("Vulkan storage binding limit below six");
        if (limits.maxComputeWorkGroupInvocations < 256 || limits.maxComputeWorkGroupSize[0] < 16 || limits.maxComputeWorkGroupSize[1] < 16)
            throw std::runtime_error("Vulkan device cannot dispatch 16 x 16 workgroups");
        const int nx = stock.nx(), ny = stock.ny();
        const std::size_t cells = static_cast<std::size_t>(nx) * ny;
        if (!stock.valid() || nx <= 0 || ny <= 0 || cells > 8'000'000) throw std::runtime_error("Vulkan material grid exceeds budget");
        int intervalSlots = 1;
        for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) intervalSlots = std::max(intervalSlots, static_cast<int>(stock.cellIntervalsView(i,j).size()));
        if (cells * intervalSlots > 16'000'000) throw std::runtime_error("Vulkan material interval budget exceeded");
        if (uint32_t(intervalSlots) > limits.maxComputeWorkGroupCount[2] || uint32_t((nx+7)/8) > limits.maxComputeWorkGroupCount[0] ||
            uint32_t((ny+7)/8) > limits.maxComputeWorkGroupCount[1]) throw std::runtime_error("Vulkan dispatch dimensions exceed device limits");
        std::vector<QVector2D> intervals(cells * intervalSlots);
        std::vector<uint32_t> counts(cells), domain(cells);
        std::vector<float> distance(cells);
        for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) {
            const auto cell = static_cast<std::size_t>(j) * nx + i;
            const auto values = stock.cellIntervalsView(i,j);
            counts[cell] = static_cast<uint32_t>(values.size()); domain[cell] = values.empty() ? 0 : 1;
            for (std::size_t k = 0; k < values.size(); ++k) intervals[cell * intervalSlots + k] = {float(values[k].lower), float(values[k].upper)};
        }
        const float resolution = static_cast<float>(stock.resolution());
        const auto occupied = [&](int i, int j) { return i >= 0 && j >= 0 && i < nx && j < ny && counts[static_cast<std::size_t>(j) * nx + i] > 0; };
        for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) {
            const bool material = occupied(i,j);
            float d = 2 * resolution;
            for (int y = -2; y <= 2; ++y) for (int x = -2; x <= 2; ++x)
                if (occupied(i+x,j+y) != material) d = std::min(d, std::max(0.5f, std::sqrt(float(x*x+y*y))-0.5f) * resolution);
            distance[static_cast<std::size_t>(j) * nx+i] = material ? d : -d;
        }
        int stride = 1;
        while (static_cast<std::size_t>((nx+stride-1)/stride) * ((ny+stride-1)/stride) * intervalSlots > 500'000) ++stride;
        s.grid.sizes[0] = nx; s.grid.sizes[1] = ny; s.grid.sizes[2] = intervalSlots; s.grid.sizes[3] = stride;
        s.grid.render[0] = (nx+stride-1)/stride; s.grid.render[1] = (ny+stride-1)/stride; s.grid.render[2] = static_cast<int>(cells * intervalSlots);
        s.grid.extent[0] = float(stock.uMin()); s.grid.extent[1] = float(stock.vMin()); s.grid.extent[2] = resolution;
        s.grid.bounds[0] = float(stock.uMax()); s.grid.bounds[1] = float(stock.vMax());
        s.origin = stock.origin(); s.u = stock.uAxis(); s.v = stock.vAxis(); s.w = stock.wAxis();
        const auto set = [](float* out, const QVector3D& v) { out[0]=v.x(); out[1]=v.y(); out[2]=v.z(); };
        set(s.grid.origin, vector3(s.origin)); set(s.grid.u, vector3(s.u)); set(s.grid.v, vector3(s.v)); set(s.grid.w, vector3(s.w));
        std::vector<QVector2D> profiles;
        for (const auto& tool : scene.tools) {
            if (tool.profile.empty() || tool.profile.size() > 32) throw std::runtime_error("Unsupported Vulkan cutter profile");
            s.profileOffsets.push_back(static_cast<int>(profiles.size()));
            for (const auto& p : tool.profile) profiles.emplace_back(float(p.first), float(p.second));
        }
        if (profiles.empty()) throw std::runtime_error("No Vulkan tool profiles");
        for (const auto& motion : scene.motions) s.ends.push_back(motion.firstSimulationStep + motion.simulationStepCount);
        VkCommandPoolCreateInfo pool{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pool.queueFamilyIndex = native->gfxQueueFamilyIdx; pool.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
        checked(s.f->vkCreateCommandPool(s.device, &pool, nullptr, &s.pool), "vkCreateCommandPool");
        auto cb = s.beginTransfer();
        const auto coarse = static_cast<std::size_t>(s.grid.render[0]) * s.grid.render[1] * intervalSlots;
        s.upload(cb, 0, intervals.size()*sizeof(QVector2D), intervals.data());
        s.upload(cb, 1, counts.size()*4, counts.data());
        s.upload(cb, 2, coarse*4, nullptr);
        s.upload(cb, 3, cells*intervalSlots*4, nullptr);
        s.upload(cb, 4, 40, nullptr);
        s.upload(cb, 5, distance.size()*4, distance.data());
        s.upload(cb, 6, domain.size()*4, domain.data());
        s.upload(cb, 7, sizeof(Grid), &s.grid, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
        s.upload(cb, 8, profiles.size()*sizeof(QVector2D), profiles.data());
        s.upload(cb, 9, coarse*4, nullptr);
        s.barrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_READ_BIT);
        const int originals[]{0,1,5};
        for (int i = 0; i < 3; ++i) {
            s.allocate(s.baseline[i], s.buffers[originals[i]].size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
            s.copy(cb, s.buffers[originals[i]], s.baseline[i]);
        }
        s.barrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT);
        s.finishTransfer(cb);
        for (auto& b : s.staging) s.destroy(b);
        s.staging.clear();
        s.createPipelines(static_cast<const QRhiVulkanRenderPassNativeHandles*>(pass->nativeHandles())->renderPass);
        return true;
    } catch (const std::exception& e) { error = QString::fromUtf8(e.what()); m.reset(); return false; }
}

bool PocketCamVulkanDexel::uploadStock(QRhi* rhi, QRhiCommandBuffer* commands, const toolpath::MultiDexelStock& stock)
{
    auto& s = *m;
    if (!stock.valid() || stock.nx() != s.grid.sizes[0] || stock.ny() != s.grid.sizes[1] ||
        float(stock.resolution()) != s.grid.extent[2] || float(stock.uMin()) != s.grid.extent[0] ||
        float(stock.vMin()) != s.grid.extent[1] || float(stock.uMax()) != s.grid.bounds[0] ||
        float(stock.vMax()) != s.grid.bounds[1] || stock.origin().Distance(s.origin) > 1e-9 ||
        (stock.uAxis()-s.u).Magnitude() > 1e-9 || (stock.vAxis()-s.v).Magnitude() > 1e-9 ||
        (stock.wAxis()-s.w).Magnitude() > 1e-9) return false;
    const int nx = stock.nx(), ny = stock.ny(), capacity = s.grid.sizes[2];
    for (int j=0; j<ny; ++j) for (int i=0; i<nx; ++i)
        if (stock.cellIntervalsView(i,j).size() > static_cast<std::size_t>(capacity)) return false;
    // Qt has waited for this frame slot's fence before calling render(). Each
    // in-flight frame therefore has its own reusable coherent upload allocation.
    const auto slot = static_cast<std::size_t>(rhi->currentFrameSlot());
    if (s.frameUploads.size() <= slot) s.frameUploads.resize(slot+1);
    auto& upload = s.frameUploads[slot];
    const int indices[]{0,1,5,6};
    VkDeviceSize bytes=0, offsets[4]{};
    for (int i=0; i<4; ++i) { offsets[i]=bytes; bytes+=s.buffers[indices[i]].size; }
    if (!upload.handle) {
        try { s.allocate(upload, bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, true); }
        catch (const std::exception&) { s.destroy(upload); return false; }
    }
    auto* data = static_cast<char*>(upload.mapped);
    auto* intervals = reinterpret_cast<QVector2D*>(data);
    auto* counts = reinterpret_cast<uint32_t*>(data+offsets[1]);
    auto* distances = reinterpret_cast<float*>(data+offsets[2]);
    auto* domain = reinterpret_cast<uint32_t*>(data+offsets[3]);
    for (int j=0; j<ny; ++j) for (int i=0; i<nx; ++i) {
        const auto cell = static_cast<std::size_t>(j)*nx+i;
        const auto values = stock.cellIntervalsView(i,j);
        counts[cell] = static_cast<uint32_t>(values.size()); domain[cell] = values.empty() ? 0 : 1;
        for (std::size_t k=0; k<values.size(); ++k) intervals[cell*capacity+k] = {float(values[k].lower),float(values[k].upper)};
    }
    const auto occupied = [&](int i, int j) { return i>=0 && j>=0 && i<nx && j<ny && counts[static_cast<std::size_t>(j)*nx+i]>0; };
    const float resolution = s.grid.extent[2];
    for (int j=0; j<ny; ++j) for (int i=0; i<nx; ++i) {
        const bool material = occupied(i,j);
        float distance=2*resolution;
        for(int y=-2; y<=2; ++y) for(int x=-2; x<=2; ++x)
            if (occupied(i+x,j+y)!=material) distance=std::min(distance,std::max(0.5f,std::sqrt(float(x*x+y*y))-0.5f)*resolution);
        distances[static_cast<std::size_t>(j)*nx+i] = material ? distance : -distance;
    }
    commands->beginComputePass(nullptr, QRhiCommandBuffer::ExternalContent);
    commands->beginExternal();
    const auto cb=static_cast<const QRhiVulkanCommandBufferNativeHandles*>(commands->nativeHandles())->commandBuffer;
    s.barrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_ACCESS_MEMORY_READ_BIT|VK_ACCESS_MEMORY_WRITE_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);
    for (int i=0; i<4; ++i) {
        const VkBufferCopy region{offsets[i],0,s.buffers[indices[i]].size};
        s.f->vkCmdCopyBuffer(cb, upload.handle, s.buffers[indices[i]].handle, 1, &region);
    }
    s.barrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);
    commands->endExternal(); commands->endComputePass();
    s.applied=0; s.dirty=true;
    return true;
}

void PocketCamVulkanDexel::State::cut(VkCommandBuffer cb, const toolpath::PocketCamSimulationScene& scene,
    const toolpath::PocketCamMotion& move, double fromFraction, double toFraction)
{
    if (move.move.type == toolpath::ToolpathMoveType::Rapid || move.move.type == toolpath::ToolpathMoveType::Retract) return;
    const auto& tool = scene.tools.at(move.toolIndex);
    const auto from = local(move.playbackPointAt(fromFraction)), to = local(move.playbackPointAt(toFraction));
    Motion p;
    for (int i = 0; i < 3; ++i) { p.from[i] = from[i]; p.to[i] = to[i]; }
    float radius = 0;
    for (const auto& profile : tool.profile) radius = std::max(radius, float(profile.first));
    p.from[3] = radius;
    p.tool[0] = profileOffsets[move.toolIndex]; p.tool[1] = static_cast<int>(tool.profile.size()); p.tool[2] = 1;
    float lowU=std::min(from.x(),to.x()), highU=std::max(from.x(),to.x()), lowV=std::min(from.y(),to.y()), highV=std::max(from.y(),to.y());
    if (move.analyticPlaybackTrusted && tool.flatEnd && move.analyticSweep.kind == toolpath::PocketCamAnalyticSweepKind::CircularArc) {
        p.tool[2] = 2;
        const auto center = local(move.analyticSweep.center);
        p.arc[0] = center.x(); p.arc[1] = center.y();
        p.arc[2] = (move.analyticSweep.direction == toolpath::ToolpathArcDirection::Counterclockwise ? 1.0f : -1.0f) * (move.analyticSweep.axis.Dot(w) >= 0 ? 1.0f : -1.0f);
        p.to[3] = float(move.analyticSweep.sweepRadians * (toFraction-fromFraction));
        const float r = std::hypot(from.x()-center.x(),from.y()-center.y());
        const float startAngle = std::atan2(from.y()-center.y(), from.x()-center.x());
        constexpr float pi = 3.14159265358979323846f;
        for (int quadrant=0; quadrant<4; ++quadrant) {
            const float angle=quadrant*0.5f*pi;
            float delta=std::fmod(p.arc[2]*(angle-startAngle),2*pi);
            if (delta<0) delta+=2*pi;
            if (delta>p.to[3]+1e-5f) continue;
            const float x=center.x()+r*std::cos(angle), y=center.y()+r*std::sin(angle);
            lowU=std::min(lowU,x); highU=std::max(highU,x);
            lowV=std::min(lowV,y); highV=std::max(highV,y);
        }
    }
    const float band = radius + 2*grid.extent[2];
    const auto coordinate = [&](float x, int axis, bool high) {
        const double value = (x-grid.extent[axis])/grid.extent[2];
        return std::clamp(static_cast<int>(high ? std::ceil(value) : std::floor(value)), 0, grid.sizes[axis]);
    };
    const int x0=coordinate(lowU-band,0,false), x1=coordinate(highU+band,0,true), y0=coordinate(lowV-band,1,false), y1=coordinate(highV+band,1,true);
    if (x1 <= x0 || y1 <= y0) return;
    p.dispatch[0]=x0; p.dispatch[1]=y0; p.dispatch[2]=x1-x0; p.dispatch[3]=y1-y0;
    f->vkCmdPushConstants(cb, layouts[0], VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(p), &p);
    f->vkCmdDispatch(cb, (x1-x0+15)/16, (y1-y0+15)/16, 1);
    barrier(cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);
    ++dispatches; dirty = true;
}

void PocketCamVulkanDexel::advance(QRhiCommandBuffer* commands, const toolpath::PocketCamSimulationScene& scene, std::size_t step)
{
    auto& s = *m;
    if (step == s.applied && !s.dirty) return;
    commands->beginComputePass(nullptr, QRhiCommandBuffer::ExternalContent);
    commands->beginExternal();
    const auto cb = static_cast<const QRhiVulkanCommandBufferNativeHandles*>(commands->nativeHandles())->commandBuffer;
    s.barrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT);
    if (step < s.applied) {
        const int originals[]{0,1,5};
        for (int i=0; i<3; ++i) s.copy(cb, s.baseline[i], s.buffers[originals[i]]);
        s.barrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);
        s.applied = 0; s.dirty = true;
    }
    s.f->vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, s.pipelines[0]);
    s.f->vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE, s.layouts[0], 0, 1, &s.sets[0], 0, nullptr);
    auto index = static_cast<std::size_t>(std::upper_bound(s.ends.begin(), s.ends.end(), s.applied)-s.ends.begin());
    for (; index < scene.motions.size(); ++index) {
        const auto& motion = scene.motions[index];
        if (motion.firstSimulationStep >= step) break;
        const auto first=std::max(s.applied, motion.firstSimulationStep), last=std::min(step, s.ends[index]);
        if (last > first) s.cut(cb, scene, motion, double(first-motion.firstSimulationStep)/motion.simulationStepCount,
            double(last-motion.firstSimulationStep)/motion.simulationStepCount);
    }
    s.applied = step;
    if (s.dirty) {
        // Homogeneous tiles have no exposed side walls. Emit only their top and
        // bottom (12 vertices), shared with the optimized OpenGL path.
        const uint32_t counts[]{150,0,0,0,0,0,12,0,0,0};
        s.barrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);
        s.f->vkCmdUpdateBuffer(cb, s.buffers[4].handle, 0, sizeof(counts), counts);
        s.barrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);
        s.f->vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, s.pipelines[1]);
        s.f->vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE, s.layouts[1], 0, 1, &s.sets[1], 0, nullptr);
        s.f->vkCmdDispatch(cb, (s.grid.render[0]+7)/8, (s.grid.render[1]+7)/8, s.grid.sizes[2]);
        s.dirty = false;
    }
    s.barrier(cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT,
        VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_INDIRECT_COMMAND_READ_BIT);
    commands->endExternal(); commands->endComputePass();
}

void PocketCamVulkanDexel::draw(QRhiCommandBuffer* commands, const QSize& size, const QMatrix4x4& projection, const QMatrix4x4& model)
{
    auto& s = *m;
    commands->beginExternal();
    const auto cb = static_cast<const QRhiVulkanCommandBufferNativeHandles*>(commands->nativeHandles())->commandBuffer;
    const VkViewport viewport{0,0,float(size.width()),float(size.height()),0,1};
    const VkRect2D scissor{{0,0},{uint32_t(size.width()),uint32_t(size.height())}};
    s.f->vkCmdSetViewport(cb, 0, 1, &viewport); s.f->vkCmdSetScissor(cb, 0, 1, &scissor);
    float data[32];
    const auto mvp = projection * model;
    std::memcpy(data, mvp.constData(), 64); std::memcpy(data+16, model.constData(), 64);
    s.f->vkCmdPushConstants(cb, s.layouts[2], VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(data), data);
    s.f->vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, s.layouts[2], 0, 1, &s.sets[2], 0, nullptr);
    for (int fine=0; fine<2; ++fine) {
        s.f->vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, s.pipelines[2+fine]);
        s.f->vkCmdDrawIndirect(cb, s.buffers[4].handle, fine ? 0 : 24, 1, sizeof(VkDrawIndirectCommand));
    }
    commands->endExternal();
}

double PocketCamVulkanDexel::volume()
{
    auto& s = *m;
    Buffer read;
    try {
        s.allocate(read, s.buffers[0].size+s.buffers[1].size, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);
        auto cb=s.beginTransfer();
        s.barrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_ACCESS_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_READ_BIT);
        VkBufferCopy region{0,0,s.buffers[0].size};
        s.f->vkCmdCopyBuffer(cb,s.buffers[0].handle,read.handle,1,&region);
        region={0,s.buffers[0].size,s.buffers[1].size};
        s.f->vkCmdCopyBuffer(cb,s.buffers[1].handle,read.handle,1,&region);
        s.barrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_HOST_BIT, VK_ACCESS_HOST_READ_BIT);
        s.finishTransfer(cb);
        const auto* values=static_cast<const QVector2D*>(read.mapped);
        const auto* counts=reinterpret_cast<const uint32_t*>(static_cast<const char*>(read.mapped)+s.buffers[0].size);
        double volume=0;
        for (int j=0;j<s.grid.sizes[1];++j) for(int i=0;i<s.grid.sizes[0];++i) {
            const auto cell=static_cast<std::size_t>(j)*s.grid.sizes[0]+i;
            const double du=std::min(double(s.grid.extent[2]),double(s.grid.bounds[0])-s.grid.extent[0]-i*double(s.grid.extent[2]));
            const double dv=std::min(double(s.grid.extent[2]),double(s.grid.bounds[1])-s.grid.extent[1]-j*double(s.grid.extent[2]));
            for (uint32_t k=0;k<std::min(counts[cell],uint32_t(s.grid.sizes[2]));++k) {
                const auto v=values[cell*s.grid.sizes[2]+k]; volume+=std::max(0.0,double(v.y()-v.x()))*du*dv;
            }
        }
        s.destroy(read); return volume;
    } catch (...) { s.destroy(read); throw; }
}
std::size_t PocketCamVulkanDexel::dispatchCount() const { return m ? m->dispatches : 0; }
QString PocketCamVulkanDexel::deviceName() const { return m ? QString::fromUtf8(m->properties.deviceName) : QString(); }
}
