// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "overlay_preview.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <vulkan/vulkan.h>

namespace {
void check(VkResult result) {
  if (result != VK_SUCCESS)
    throw std::runtime_error("Headset preview Vulkan error " +
                             std::to_string(result));
}
std::vector<std::string> split(const std::string &value) {
  std::istringstream stream(value);
  std::vector<std::string> out;
  for (std::string part; stream >> part;)
    out.push_back(part);
  return out;
}
std::vector<const char *> pointers(const std::vector<std::string> &values) {
  std::vector<const char *> out;
  for (const auto &value : values)
    out.push_back(value.c_str());
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
  VkDeviceMemory buffer_memory{};
  VkDescriptorSetLayout descriptor_layout{};
  VkDescriptorPool descriptor_pool{};
  VkDescriptorSet descriptor{};
  VkPipelineLayout pipeline_layout{};
  VkPipeline pipeline{};
  VkSampler sampler{};
  VkImageView source_view{};
  VkBuffer buffer{};
  uint32_t width{}, height{};
  vr::IVROverlayView *api{};
  vr::VROverlayView_t view{};
  bool acquired{};
  std::vector<uint8_t> pixels;

  uint32_t memory_type(uint32_t bits, VkMemoryPropertyFlags flags) {
    VkPhysicalDeviceMemoryProperties properties{};
    vkGetPhysicalDeviceMemoryProperties(physical, &properties);
    for (uint32_t i = 0; i < properties.memoryTypeCount; ++i)
      if ((bits & (1u << i)) &&
          (properties.memoryTypes[i].propertyFlags & flags) == flags)
        return i;
    throw std::runtime_error("Headset preview memory unavailable");
  }
  void initialize(vr::IVRSystem *system, vr::IVRCompositor *compositor) {
    uint32_t length =
        compositor->GetVulkanInstanceExtensionsRequired(nullptr, 0);
    if (!length || length > 65536)
      throw std::runtime_error("Headset preview extensions unavailable");
    std::string required(length, '\0');
    compositor->GetVulkanInstanceExtensionsRequired(required.data(), length);
    auto names = split(required);
    auto extensions = pointers(names);
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "Rift S Settings";
    app.apiVersion = VK_API_VERSION_1_1;
    VkInstanceCreateInfo create{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    create.pApplicationInfo = &app;
    create.enabledExtensionCount = extensions.size();
    create.ppEnabledExtensionNames = extensions.data();
    check(vkCreateInstance(&create, nullptr, &instance));
    uint64_t output = 0;
    system->GetOutputDevice(&output, vr::TextureType_Vulkan, instance);
    physical = reinterpret_cast<VkPhysicalDevice>(output);
    if (!physical)
      throw std::runtime_error("Headset preview GPU unavailable");
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
    while (family < count && (families[family].queueFlags &
                              (VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT)) !=
                                 (VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT))
      ++family;
    if (family == count)
      throw std::runtime_error("Headset preview queue unavailable");
    length =
        compositor->GetVulkanDeviceExtensionsRequired(physical, nullptr, 0);
    if (!length || length > 65536)
      throw std::runtime_error("Headset preview device extensions unavailable");
    required.assign(length, '\0');
    compositor->GetVulkanDeviceExtensionsRequired(physical, required.data(),
                                                  length);
    names = split(required);
    extensions = pointers(names);
    float priority = 1;
    VkDeviceQueueCreateInfo queue_create{
        VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queue_create.queueFamilyIndex = family;
    queue_create.queueCount = 1;
    queue_create.pQueuePriorities = &priority;
    VkPhysicalDeviceTimelineSemaphoreFeatures timeline{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES};
    VkPhysicalDeviceFeatures2 features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
    features.pNext = &timeline;
    vkGetPhysicalDeviceFeatures2(physical, &features);
    VkDeviceCreateInfo device_create{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    device_create.queueCreateInfoCount = 1;
    device_create.pQueueCreateInfos = &queue_create;
    device_create.enabledExtensionCount = extensions.size();
    device_create.ppEnabledExtensionNames = extensions.data();
    device_create.pNext = &timeline;
    check(vkCreateDevice(physical, &device_create, nullptr, &device));
    vkGetDeviceQueue(device, family, 0, &queue);
    VkCommandPoolCreateInfo pool_create{
        VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pool_create.queueFamilyIndex = family;
    pool_create.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    check(vkCreateCommandPool(device, &pool_create, nullptr, &pool));
    VkCommandBufferAllocateInfo allocate{
        VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocate.commandPool = pool;
    allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocate.commandBufferCount = 1;
    check(vkAllocateCommandBuffers(device, &allocate, &command));
    VkFenceCreateInfo fence_create{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    check(vkCreateFence(device, &fence_create, nullptr, &fence));
  }
  void initialize_sampling() {
    VkDescriptorSetLayoutBinding bindings[] = {
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
         VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT,
         nullptr}};
    VkDescriptorSetLayoutCreateInfo layout{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layout.bindingCount = 2;
    layout.pBindings = bindings;
    check(vkCreateDescriptorSetLayout(device, &layout, nullptr,
                                      &descriptor_layout));
    VkDescriptorPoolSize sizes[] = {
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1}};
    VkDescriptorPoolCreateInfo descriptors{
        VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    descriptors.maxSets = 1;
    descriptors.poolSizeCount = 2;
    descriptors.pPoolSizes = sizes;
    check(vkCreateDescriptorPool(device, &descriptors, nullptr,
                                 &descriptor_pool));
    VkDescriptorSetAllocateInfo allocate{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocate.descriptorPool = descriptor_pool;
    allocate.descriptorSetCount = 1;
    allocate.pSetLayouts = &descriptor_layout;
    check(vkAllocateDescriptorSets(device, &allocate, &descriptor));
    VkPushConstantRange push{VK_SHADER_STAGE_COMPUTE_BIT, 0, 28};
    VkPipelineLayoutCreateInfo pipeline_create{
        VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipeline_create.setLayoutCount = 1;
    pipeline_create.pSetLayouts = &descriptor_layout;
    pipeline_create.pushConstantRangeCount = 1;
    pipeline_create.pPushConstantRanges = &push;
    check(vkCreatePipelineLayout(device, &pipeline_create, nullptr,
                                 &pipeline_layout));
    // Generated from overlay_preview.comp with glslc -fshader-stage=compute
    // -mfmt=c.
    static constexpr uint32_t code[] =
#include "overlay_preview_shader.inc"
        ;
    VkShaderModuleCreateInfo module_create{
        VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    module_create.codeSize = sizeof(code);
    module_create.pCode = code;
    VkShaderModule module{};
    check(vkCreateShaderModule(device, &module_create, nullptr, &module));
    VkComputePipelineCreateInfo compute{
        VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    compute.layout = pipeline_layout;
    compute.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                     nullptr,
                     0,
                     VK_SHADER_STAGE_COMPUTE_BIT,
                     module,
                     "main",
                     nullptr};
    auto created = vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &compute,
                                            nullptr, &pipeline);
    vkDestroyShaderModule(device, module, nullptr);
    check(created);
    VkSamplerCreateInfo sample{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sample.magFilter = sample.minFilter = VK_FILTER_NEAREST;
    sample.addressModeU = sample.addressModeV = sample.addressModeW =
        VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    check(vkCreateSampler(device, &sample, nullptr, &sampler));
  }
  bool release() {
    auto result = acquired && api ? api->ReleaseOverlayView(&view)
                                  : vr::VROverlayError_None;
    acquired = false;
    view = {};
    return result == vr::VROverlayError_None;
  }
  void clear_buffers() {
    if (buffer)
      vkDestroyBuffer(device, buffer, nullptr);
    if (buffer_memory)
      vkFreeMemory(device, buffer_memory, nullptr);
    buffer = {};
    buffer_memory = {};
  }
  void resize(uint32_t w, uint32_t h) {
    if (w == width && h == height && buffer)
      return;
    clear_buffers();
    width = w;
    height = h;
    VkBufferCreateInfo create{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    create.size = size_t(w) * h * 4;
    create.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    check(vkCreateBuffer(device, &create, nullptr, &buffer));
    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(device, buffer, &requirements);
    VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocation.allocationSize = requirements.size;
    allocation.memoryTypeIndex = memory_type(
        requirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    check(vkAllocateMemory(device, &allocation, nullptr, &buffer_memory));
    check(vkBindBufferMemory(device, buffer, buffer_memory, 0));
    pixels.resize(size_t(w) * h * 4);
  }
  void copy(const vr::VRVulkanTextureData_t &source,
            const vr::VRTextureBounds_t &bounds) {
    if (source.m_pDevice != device || source.m_nQueueFamilyIndex != family ||
        source.m_nSampleCount != 1 || source.m_nWidth == 0 ||
        source.m_nHeight == 0 || source.m_nWidth > 16384 ||
        source.m_nHeight > 16384)
      throw std::runtime_error("Headset preview texture unsupported");
    const float coords[] = {bounds.uMin, bounds.uMax, bounds.vMin, bounds.vMax};
    for (float value : coords)
      if (!std::isfinite(value) || value < 0 || value > 1)
        throw std::runtime_error("Headset preview bounds invalid");
    uint32_t w =
        std::lround(std::abs(bounds.uMax - bounds.uMin) * source.m_nWidth);
    uint32_t h =
        std::lround(std::abs(bounds.vMax - bounds.vMin) * source.m_nHeight);
    if (!w || !h)
      throw std::runtime_error("Headset preview bounds empty");
    double scale = std::min(1.0, 512.0 / std::max(w, h));
    resize(std::max(1u, uint32_t(w * scale)),
           std::max(1u, uint32_t(h * scale)));
    auto format = VkFormat(source.m_nFormat);
    VkFormatProperties properties{};
    vkGetPhysicalDeviceFormatProperties(physical, format, &properties);
    if (!(properties.optimalTilingFeatures &
          VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT))
      throw std::runtime_error("Headset preview format unsupported");
    VkImageViewCreateInfo view_create{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    view_create.image = reinterpret_cast<VkImage>(source.m_nImage);
    view_create.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_create.format = format;
    view_create.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    check(vkCreateImageView(device, &view_create, nullptr, &source_view));
    VkDescriptorImageInfo sampled{sampler, source_view,
                                  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    VkDescriptorBufferInfo output{buffer, 0, pixels.size()};
    VkWriteDescriptorSet writes[2]{};
    for (auto &write : writes) {
      write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      write.dstSet = descriptor;
      write.descriptorCount = 1;
    }
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].pImageInfo = &sampled;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].pBufferInfo = &output;
    vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
    check(vkResetCommandBuffer(command, 0));
    VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    check(vkBeginCommandBuffer(command, &begin));
    // The acquired image is sampleable and stable. Never write or transition
    // it, and finish all GPU reads before ReleaseOverlayView frees the import.
    VkMemoryBarrier readable{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    readable.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
    readable.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &readable,
                         0, nullptr, 0, nullptr);
    vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdBindDescriptorSets(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                            pipeline_layout, 0, 1, &descriptor, 0, nullptr);
    struct {
      float bounds[4];
      uint32_t width, height, srgb;
    } push{{bounds.uMin, bounds.vMin, bounds.uMax, bounds.vMax},
           width,
           height,
           uint32_t(format == VK_FORMAT_R8G8B8A8_SRGB ||
                    format == VK_FORMAT_B8G8R8A8_SRGB ||
                    format == VK_FORMAT_A8B8G8R8_SRGB_PACK32)};
    vkCmdPushConstants(command, pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                       sizeof(push), &push);
    vkCmdDispatch(command, (width + 7) / 8, (height + 7) / 8, 1);
    VkBufferMemoryBarrier host{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
    host.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    host.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    host.srcQueueFamilyIndex = host.dstQueueFamilyIndex =
        VK_QUEUE_FAMILY_IGNORED;
    host.buffer = buffer;
    host.size = VK_WHOLE_SIZE;
    vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, 1, &host, 0,
                         nullptr);
    check(vkEndCommandBuffer(command));
    check(vkResetFences(device, 1, &fence));
    VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &command;
    check(vkQueueSubmit(queue, 1, &submit, fence));
    check(vkWaitForFences(device, 1, &fence, VK_TRUE, 500000000));
    vkDestroyImageView(device, source_view, nullptr);
    source_view = {};
    void *mapped{};
    check(vkMapMemory(device, buffer_memory, 0, pixels.size(), 0, &mapped));
    std::memcpy(pixels.data(), mapped, pixels.size());
    vkUnmapMemory(device, buffer_memory);
  }
  ~Preview() {
    if (device)
      vkDeviceWaitIdle(device);
    if (source_view)
      vkDestroyImageView(device, source_view, nullptr);
    release();
    clear_buffers();
    if (pipeline)
      vkDestroyPipeline(device, pipeline, nullptr);
    if (pipeline_layout)
      vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
    if (sampler)
      vkDestroySampler(device, sampler, nullptr);
    if (descriptor_pool)
      vkDestroyDescriptorPool(device, descriptor_pool, nullptr);
    if (descriptor_layout)
      vkDestroyDescriptorSetLayout(device, descriptor_layout, nullptr);
    if (fence)
      vkDestroyFence(device, fence, nullptr);
    if (pool)
      vkDestroyCommandPool(device, pool, nullptr);
    if (device)
      vkDestroyDevice(device, nullptr);
    if (instance)
      vkDestroyInstance(instance, nullptr);
  }
};
std::unique_ptr<Preview> preview;
std::string error;
} // namespace
const char *rifts_overlay_frame(vr::IVRSystem *system,
                                vr::IVRCompositor *compositor,
                                vr::IVROverlay *overlay,
                                vr::IVROverlayView *api, const uint8_t **data,
                                size_t *bytes, uint32_t *w, uint32_t *h) {
  *data = nullptr;
  *bytes = 0;
  *w = *h = 0;
  if (!system || !compositor || !overlay || !api)
    return "Headset preview unavailable";
  vr::VROverlayHandle_t handle{};
  if (overlay->FindOverlay(vr::k_pchHeadsetViewOverlayKey, &handle) !=
      vr::VROverlayError_None)
    return "Open VR View to preview";
  if (!api->IsViewingPermitted(handle))
    return "Headset preview access unavailable";
  try {
    if (!preview) {
      preview = std::make_unique<Preview>();
      preview->initialize(system, compositor);
      preview->initialize_sampling();
    }
    preview->api = api;
    vr::VRVulkanDevice_t vulkan{preview->instance, preview->device,
                                preview->physical, preview->queue,
                                preview->family};
    vr::VRNativeDevice_t native{&vulkan, vr::DeviceType_Vulkan};
    auto acquired = api->AcquireOverlayView(handle, &native, &preview->view,
                                            sizeof(preview->view));
    // Even a view with no queued frame owns native resources and must be
    // released.
    preview->acquired = true;
    if (acquired != vr::VROverlayError_None)
      throw std::runtime_error("Headset preview acquisition error " +
                               std::to_string(acquired));
    if (!preview->view.texture.handle) {
      if (!preview->release())
        throw std::runtime_error("Headset preview release failed");
      auto activity = system->GetTrackedDeviceActivityLevel(0);
      return activity == vr::k_EDeviceActivityLevel_Standby
                 ? "Put on the headset to preview"
                 : "Headset view has no frame";
    }
    if (preview->view.texture.eType != vr::TextureType_Vulkan)
      throw std::runtime_error("Headset preview texture type unsupported");
    preview->copy(
        *static_cast<vr::VRVulkanTextureData_t *>(preview->view.texture.handle),
        preview->view.textureBounds);
    if (!preview->release())
      throw std::runtime_error("Headset preview release failed");
    *data = preview->pixels.data();
    *bytes = preview->pixels.size();
    *w = preview->width;
    *h = preview->height;
    return nullptr;
  } catch (const std::exception &failure) {
    error = failure.what();
    preview.reset();
    return error.c_str();
  }
}
void rifts_overlay_shutdown() { preview.reset(); }
