// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "overlay_preview.hpp"
#include <vulkan/vulkan.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void check(VkResult result) {
    if (result != VK_SUCCESS) throw std::runtime_error("Headset preview Vulkan error " + std::to_string(result));
}
std::vector<std::string> split(const std::string &value) {
    std::istringstream stream(value); std::vector<std::string> out;
    for (std::string part; stream >> part;) out.push_back(part);
    return out;
}
std::vector<const char *> pointers(const std::vector<std::string> &values) {
    std::vector<const char *> out;
    for (const auto &value : values) out.push_back(value.c_str());
    return out;
}
struct Preview {
    VkInstance instance{};
    VkPhysicalDevice physical{};
    VkDevice device{};
    VkQueue queue{};
    uint32_t family{};
    VkCommandPool pool{};
    VkCommandBuffer command{};
    VkFence fence{};
    VkImage image{};
    VkDeviceMemory image_memory{}, buffer_memory{};
    VkBuffer buffer{};
    uint32_t width{}, height{};
    bool image_initialized{};
    vr::IVROverlayView *api{};
    vr::VROverlayView_t view{};
    bool acquired{};
    std::vector<uint8_t> pixels;

    uint32_t memory_type(uint32_t bits, VkMemoryPropertyFlags flags) {
        VkPhysicalDeviceMemoryProperties properties{};
        vkGetPhysicalDeviceMemoryProperties(physical, &properties);
        for (uint32_t i = 0; i < properties.memoryTypeCount; ++i)
            if ((bits & (1u << i)) && (properties.memoryTypes[i].propertyFlags & flags) == flags) return i;
        throw std::runtime_error("Headset preview memory unavailable");
    }
    void initialize(vr::IVRSystem *system, vr::IVRCompositor *compositor) {
        uint32_t length = compositor->GetVulkanInstanceExtensionsRequired(nullptr, 0);
        if (!length || length > 65536) throw std::runtime_error("Headset preview extensions unavailable");
        std::string required(length, '\0'); compositor->GetVulkanInstanceExtensionsRequired(required.data(), length);
        auto names = split(required); auto extensions = pointers(names);
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "Rift S Settings"; app.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo create{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        create.pApplicationInfo = &app; create.enabledExtensionCount = extensions.size(); create.ppEnabledExtensionNames = extensions.data();
        check(vkCreateInstance(&create, nullptr, &instance));
        uint64_t output = 0; system->GetOutputDevice(&output, vr::TextureType_Vulkan, instance);
        physical = reinterpret_cast<VkPhysicalDevice>(output);
        if (!physical) throw std::runtime_error("Headset preview GPU unavailable");
        uint32_t count = 0; vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
        std::vector<VkQueueFamilyProperties> families(count); vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
        while (family < count && !(families[family].queueFlags & VK_QUEUE_GRAPHICS_BIT)) ++family;
        if (family == count) throw std::runtime_error("Headset preview queue unavailable");
        length = compositor->GetVulkanDeviceExtensionsRequired(physical, nullptr, 0);
        if (!length || length > 65536) throw std::runtime_error("Headset preview device extensions unavailable");
        required.assign(length, '\0'); compositor->GetVulkanDeviceExtensionsRequired(physical, required.data(), length);
        names = split(required); extensions = pointers(names);
        float priority = 1;
        VkDeviceQueueCreateInfo queue_create{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queue_create.queueFamilyIndex = family; queue_create.queueCount = 1; queue_create.pQueuePriorities = &priority;
        VkPhysicalDeviceTimelineSemaphoreFeatures timeline{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES};
        VkPhysicalDeviceFeatures2 features{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2}; features.pNext = &timeline;
        vkGetPhysicalDeviceFeatures2(physical, &features);
        VkDeviceCreateInfo device_create{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        device_create.queueCreateInfoCount = 1; device_create.pQueueCreateInfos = &queue_create;
        device_create.enabledExtensionCount = extensions.size(); device_create.ppEnabledExtensionNames = extensions.data(); device_create.pNext = &timeline;
        check(vkCreateDevice(physical, &device_create, nullptr, &device));
        vkGetDeviceQueue(device, family, 0, &queue);
        VkCommandPoolCreateInfo pool_create{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pool_create.queueFamilyIndex = family; pool_create.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        check(vkCreateCommandPool(device, &pool_create, nullptr, &pool));
        VkCommandBufferAllocateInfo allocate{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        allocate.commandPool = pool; allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; allocate.commandBufferCount = 1;
        check(vkAllocateCommandBuffers(device, &allocate, &command));
        VkFenceCreateInfo fence_create{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; check(vkCreateFence(device, &fence_create, nullptr, &fence));
    }
    void release() {
        if (acquired && api) api->ReleaseOverlayView(&view);
        acquired = false; view = {};
    }
    void clear_buffers() {
        if (buffer) vkDestroyBuffer(device, buffer, nullptr);
        if (image) vkDestroyImage(device, image, nullptr);
        if (buffer_memory) vkFreeMemory(device, buffer_memory, nullptr);
        if (image_memory) vkFreeMemory(device, image_memory, nullptr);
        buffer = {}; image = {}; buffer_memory = {}; image_memory = {}; image_initialized = false;
    }
    void resize(uint32_t w, uint32_t h) {
        if (w == width && h == height && buffer && image) return;
        clear_buffers(); width = w; height = h;
        VkImageCreateInfo create{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        create.imageType = VK_IMAGE_TYPE_2D; create.format = VK_FORMAT_R8G8B8A8_UNORM;
        create.extent = {w, h, 1}; create.mipLevels = create.arrayLayers = 1; create.samples = VK_SAMPLE_COUNT_1_BIT;
        create.tiling = VK_IMAGE_TILING_OPTIMAL; create.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        check(vkCreateImage(device, &create, nullptr, &image));
        VkMemoryRequirements requirements{}; vkGetImageMemoryRequirements(device, image, &requirements);
        VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = memory_type(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        check(vkAllocateMemory(device, &allocation, nullptr, &image_memory)); check(vkBindImageMemory(device, image, image_memory, 0));
        VkBufferCreateInfo buffer_create{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        buffer_create.size = size_t(w) * h * 4; buffer_create.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        check(vkCreateBuffer(device, &buffer_create, nullptr, &buffer)); vkGetBufferMemoryRequirements(device, buffer, &requirements);
        allocation.allocationSize = requirements.size;
        allocation.memoryTypeIndex = memory_type(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        check(vkAllocateMemory(device, &allocation, nullptr, &buffer_memory)); check(vkBindBufferMemory(device, buffer, buffer_memory, 0));
        pixels.resize(size_t(w) * h * 4);
    }
    void copy(const vr::VRVulkanTextureData_t &source, const vr::VRTextureBounds_t &bounds) {
        if (source.m_pDevice != device || source.m_nQueueFamilyIndex != family || source.m_nSampleCount != 1 ||
            source.m_nWidth == 0 || source.m_nHeight == 0 || source.m_nWidth > 16384 || source.m_nHeight > 16384)
            throw std::runtime_error("Headset preview texture unsupported");
        const float coords[] = {bounds.uMin, bounds.uMax, bounds.vMin, bounds.vMax};
        for (float value : coords) if (!std::isfinite(value) || value < 0 || value > 1) throw std::runtime_error("Headset preview bounds invalid");
        int x0 = std::lround(bounds.uMin * source.m_nWidth), x1 = std::lround(bounds.uMax * source.m_nWidth);
        int y0 = std::lround(bounds.vMin * source.m_nHeight), y1 = std::lround(bounds.vMax * source.m_nHeight);
        uint32_t w = std::abs(x1 - x0), h = std::abs(y1 - y0);
        if (!w || !h) throw std::runtime_error("Headset preview bounds empty");
        double scale = std::min(1.0, 512.0 / std::max(w, h));
        resize(std::max(1u, uint32_t(w * scale)), std::max(1u, uint32_t(h * scale)));
        VkFormatProperties properties{}; vkGetPhysicalDeviceFormatProperties(physical, VkFormat(source.m_nFormat), &properties);
        if (!(properties.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_SRC_BIT)) throw std::runtime_error("Headset preview format unsupported");
        VkImage source_image = reinterpret_cast<VkImage>(source.m_nImage);
        check(vkResetCommandBuffer(command, 0));
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        check(vkBeginCommandBuffer(command, &begin));
        // AcquireOverlayView grants a stable readable texture on this device. Keep
        // its sampled layout after the copy and finish GPU reads before releasing it.
        VkImageMemoryBarrier barriers[2]{};
        for (auto &barrier : barriers) {
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.srcQueueFamilyIndex = barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        }
        barriers[0].image = source_image; barriers[0].oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barriers[0].newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL; barriers[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        barriers[0].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        barriers[1].image = image; barriers[1].oldLayout = image_initialized ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED;
        barriers[1].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barriers[1].srcAccessMask = image_initialized ? VK_ACCESS_TRANSFER_READ_BIT : 0; barriers[1].dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 2, barriers);
        VkImageBlit blit{}; blit.srcSubresource = blit.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        blit.srcOffsets[0] = {x0, y0, 0}; blit.srcOffsets[1] = {x1, y1, 1}; blit.dstOffsets[1] = {int(width), int(height), 1};
        auto filter = properties.optimalTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT ? VK_FILTER_LINEAR : VK_FILTER_NEAREST;
        vkCmdBlitImage(command, source_image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, filter);
        barriers[0].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL; barriers[0].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barriers[0].srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT; barriers[0].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        barriers[1].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; barriers[1].newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        barriers[1].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; barriers[1].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 2, barriers);
        VkBufferImageCopy region{}; region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1}; region.imageExtent = {width, height, 1};
        vkCmdCopyImageToBuffer(command, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buffer, 1, &region);
        VkBufferMemoryBarrier host{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        host.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; host.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        host.srcQueueFamilyIndex = host.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; host.buffer = buffer; host.size = VK_WHOLE_SIZE;
        vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, 1, &host, 0, nullptr);
        check(vkEndCommandBuffer(command)); check(vkResetFences(device, 1, &fence));
        VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO}; submit.commandBufferCount = 1; submit.pCommandBuffers = &command;
        check(vkQueueSubmit(queue, 1, &submit, fence)); check(vkWaitForFences(device, 1, &fence, VK_TRUE, 500000000));
        image_initialized = true;
        void *mapped{}; check(vkMapMemory(device, buffer_memory, 0, pixels.size(), 0, &mapped));
        std::memcpy(pixels.data(), mapped, pixels.size()); vkUnmapMemory(device, buffer_memory);
    }
    ~Preview() {
        if (device) vkDeviceWaitIdle(device);
        release(); clear_buffers();
        if (fence) vkDestroyFence(device, fence, nullptr);
        if (pool) vkDestroyCommandPool(device, pool, nullptr);
        if (device) vkDestroyDevice(device, nullptr);
        if (instance) vkDestroyInstance(instance, nullptr);
    }
};
std::unique_ptr<Preview> preview;
std::string error;
}
const char *rifts_overlay_frame(vr::IVRSystem *system, vr::IVRCompositor *compositor, vr::IVROverlay *overlay,
                               vr::IVROverlayView *api, const uint8_t **data, size_t *bytes, uint32_t *w, uint32_t *h) {
    *data = nullptr; *bytes = 0; *w = *h = 0;
    if (!system || !compositor || !overlay || !api) return "Headset preview unavailable";
    vr::VROverlayHandle_t handle{};
    if (overlay->FindOverlay(vr::k_pchHeadsetViewOverlayKey, &handle) != vr::VROverlayError_None) return "Open VR View to preview";
    if (!api->IsViewingPermitted(handle)) return "Headset preview access unavailable";
    try {
        if (!preview) { preview = std::make_unique<Preview>(); preview->initialize(system, compositor); }
        preview->api = api;
        vr::VRVulkanDevice_t vulkan{preview->instance, preview->device, preview->physical, preview->queue, preview->family};
        vr::VRNativeDevice_t native{&vulkan, vr::DeviceType_Vulkan};
        auto acquired = api->AcquireOverlayView(handle, &native, &preview->view, sizeof(preview->view));
        // Even a view with no queued frame owns native resources and must be released.
        preview->acquired = true;
        if (acquired != vr::VROverlayError_None) throw std::runtime_error("Headset preview acquisition error " + std::to_string(acquired));
        if (!preview->view.texture.handle) {
            preview->release();
            auto activity = system->GetTrackedDeviceActivityLevel(0);
            return activity == vr::k_EDeviceActivityLevel_Standby ? "Put on the headset to preview" : "Headset view has no frame";
        }
        if (preview->view.texture.eType != vr::TextureType_Vulkan) throw std::runtime_error("Headset preview texture type unsupported");
        preview->copy(*static_cast<vr::VRVulkanTextureData_t *>(preview->view.texture.handle), preview->view.textureBounds);
        preview->release();
        *data = preview->pixels.data(); *bytes = preview->pixels.size(); *w = preview->width; *h = preview->height;
        return nullptr;
    } catch (const std::exception &failure) { error = failure.what(); preview.reset(); return error.c_str(); }
}
void rifts_overlay_shutdown() { preview.reset(); }
