// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
//
// Zero-copy headset preview.
//
// SteamVR renders its headset view into the public `system.HeadsetView`
// overlay. Once per compositor vsync the preview thread re-acquires that
// overlay through IVROverlayView into our Vulkan device on SteamVR's GPU and
// draws it into a Vulkan swapchain on a Wayland subsurface that sits over the
// preview box of the webview. Pixels never leave the GPU.
//
// Re-acquiring with the same VROverlayView_t (as the OpenVR header advises)
// reuses the client's imported images: ~25 us per call and no vrclient log
// line, versus ~1 ms and one log line for acquire+release.
//
// Threads:
//  - rifts_surface_place: GTK main thread, owns the subsurface position.
//  - rifts_headset_preview_step/_shutdown: preview thread with the OpenVR
//    client lock held, owns Vulkan and the subsurface contents.
//  - rifts_preview_status: any thread.
#define VK_USE_PLATFORM_WAYLAND_KHR
#include "headset_preview.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <vulkan/vulkan.h>
#include <wayland-client.h>

// The few GTK/GDK calls needed to find the toplevel wl_surface. Declared here
// to keep GTK headers out of the build; the binary links GTK through Tauri.
extern "C" {
struct _GtkWidget;
struct _GdkWindow;
struct _GdkDisplay;
_GtkWidget *gtk_widget_get_toplevel(_GtkWidget *);
_GdkWindow *gtk_widget_get_window(_GtkWidget *);
int gtk_widget_translate_coordinates(_GtkWidget *, _GtkWidget *, int, int,
                                     int *, int *);
void gtk_widget_queue_draw(_GtkWidget *);
_GdkDisplay *gdk_window_get_display(_GdkWindow *);
int gdk_window_get_scale_factor(_GdkWindow *);
unsigned long gdk_wayland_window_get_type();
int g_type_check_instance_is_a(void *, unsigned long);
wl_display *gdk_wayland_display_get_wl_display(_GdkDisplay *);
wl_surface *gdk_wayland_window_get_wl_surface(_GdkWindow *);
}

namespace {
constexpr int64_t retry_us = 500000;

// ---- Shared state ---------------------------------------------------------

struct Layout {
  int x{}, y{}, width{}, height{}, scale{1};
  bool visible{};
};
// Wayland objects live for the process. `surface` is ours; the subsurface
// role ties it to the GTK toplevel. Only the main thread changes the role and
// position; only the preview thread attaches buffers.
struct Wayland {
  wl_display *display{};
  wl_event_queue *queue{};
  wl_compositor *compositor{};
  wl_subcompositor *subcompositor{};
  wl_surface *parent{};
  wl_surface *surface{};
  wl_subsurface *subsurface{};
  bool unavailable{};
};
std::mutex layout_lock; // guards layout and the publication of wayland
Layout layout;
Wayland wayland;

std::mutex status_lock;
std::string status_error = "Starting preview";
uint64_t status_presented{};
uint32_t status_width{}, status_height{};
void set_error(const std::string &error) {
  std::lock_guard guard(status_lock);
  status_error = error;
}

void registry_global(void *, wl_registry *registry, uint32_t name,
                     const char *interface, uint32_t version) {
  if (!std::strcmp(interface, wl_compositor_interface.name))
    wayland.compositor = static_cast<wl_compositor *>(wl_registry_bind(
        registry, name, &wl_compositor_interface, std::min(version, 4u)));
  else if (!std::strcmp(interface, wl_subcompositor_interface.name))
    wayland.subcompositor = static_cast<wl_subcompositor *>(
        wl_registry_bind(registry, name, &wl_subcompositor_interface, 1));
}
void registry_remove(void *, wl_registry *, uint32_t) {}
const wl_registry_listener registry_listener{registry_global, registry_remove};

// Binds globals on a private queue so GTK's queue is never dispatched here.
bool connect_wayland(wl_display *display) {
  wayland.display = display;
  wayland.queue = wl_display_create_queue(display);
  auto *wrapper = static_cast<wl_display *>(wl_proxy_create_wrapper(display));
  wl_proxy_set_queue(reinterpret_cast<wl_proxy *>(wrapper), wayland.queue);
  wl_registry *registry = wl_display_get_registry(wrapper);
  wl_proxy_wrapper_destroy(wrapper);
  wl_registry_add_listener(registry, &registry_listener, nullptr);
  wl_display_roundtrip_queue(display, wayland.queue);
  wl_registry_destroy(registry);
  if (!wayland.compositor || !wayland.subcompositor)
    return false;
  wayland.surface = wl_compositor_create_surface(wayland.compositor);
  // Pointer input falls through to the webview (double-click, cursor).
  wl_region *empty = wl_compositor_create_region(wayland.compositor);
  wl_surface_set_input_region(wayland.surface, empty);
  wl_region_destroy(empty);
  return true;
}

// ---- Vulkan renderer ------------------------------------------------------

void check(VkResult result, const char *what) {
  if (result != VK_SUCCESS)
    throw std::runtime_error(std::string("Headset preview ") + what +
                             " failed (" + std::to_string(result) + ")");
}
std::vector<std::string> split(const std::string &value) {
  std::istringstream stream(value);
  std::vector<std::string> out;
  for (std::string part; stream >> part;)
    out.push_back(part);
  return out;
}
// Space-separated extension list from OpenVR plus our own.
std::vector<std::string> extensions(std::string required, const char *extra) {
  auto names = split(required);
  if (std::find(names.begin(), names.end(), extra) == names.end())
    names.push_back(extra);
  return names;
}
std::vector<const char *> pointers(const std::vector<std::string> &values) {
  std::vector<const char *> out;
  for (const auto &value : values)
    out.push_back(value.c_str());
  return out;
}
bool srgb(VkFormat format) {
  return format == VK_FORMAT_R8G8B8A8_SRGB ||
         format == VK_FORMAT_B8G8R8A8_SRGB ||
         format == VK_FORMAT_A8B8G8R8_SRGB_PACK32;
}

struct Renderer {
  VkInstance instance{};
  VkPhysicalDevice physical{};
  VkDevice device{};
  VkQueue queue{};
  uint32_t family{};
  VkCommandPool pool{};
  VkCommandBuffer command{};
  VkFence fence{};
  VkSemaphore acquired{};
  VkSampler sampler{};
  VkDescriptorSetLayout set_layout{};
  VkDescriptorPool descriptor_pool{};
  VkPipelineLayout pipeline_layout{};
  VkRenderPass pass{};
  VkPipeline pipeline{};
  VkFormat format{VK_FORMAT_UNDEFINED};
  double period{1.0 / 80};

  VkSurfaceKHR surface{};
  VkSwapchainKHR swapchain{};
  VkExtent2D extent{};
  // Layout size the swapchain was made for; extent may be clamped by caps.
  VkExtent2D requested{};
  int scale{1};
  bool stale{};
  bool mapped{};
  struct Target {
    VkImageView view{};
    VkFramebuffer framebuffer{};
    VkSemaphore rendered{};
  };
  std::vector<VkImage> images;
  std::vector<Target> targets;

  // Overlay images seen through the held view. The producer cycles through a
  // few; anything beyond the cache means it reallocated, so start over.
  vr::IVROverlayView *api{};
  vr::VROverlayView_t view{};
  bool holding{};
  struct Source {
    VkImage image{};
    VkImageView view{};
    VkDescriptorSet set{};
  };
  std::array<Source, 8> sources{};
  size_t source_count{};
  uint32_t source_width{}, source_height{};
  VkFormat source_format{VK_FORMAT_UNDEFINED};
  uint64_t last_frame{~0ull};
  uint32_t last_index{};
  std::chrono::steady_clock::time_point last_present{};

  void initialize(vr::IVRSystem *system, vr::IVRCompositor *compositor) {
    uint32_t length =
        compositor->GetVulkanInstanceExtensionsRequired(nullptr, 0);
    if (!length || length > 65536)
      throw std::runtime_error("Headset preview extensions unavailable");
    std::string required(length, '\0');
    compositor->GetVulkanInstanceExtensionsRequired(required.data(), length);
    auto names = extensions(required.c_str(), "VK_KHR_wayland_surface");
    auto list = pointers(names);
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "Rift S Settings";
    app.apiVersion = VK_API_VERSION_1_1;
    VkInstanceCreateInfo create{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    create.pApplicationInfo = &app;
    create.enabledExtensionCount = list.size();
    create.ppEnabledExtensionNames = list.data();
    check(vkCreateInstance(&create, nullptr, &instance), "instance");
    uint64_t output = 0;
    system->GetOutputDevice(&output, vr::TextureType_Vulkan, instance);
    physical = reinterpret_cast<VkPhysicalDevice>(output);
    if (!physical)
      throw std::runtime_error("Headset preview GPU unavailable");
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
    while (family < count &&
           (!(families[family].queueFlags & VK_QUEUE_GRAPHICS_BIT) ||
            !vkGetPhysicalDeviceWaylandPresentationSupportKHR(
                physical, family, wayland.display)))
      ++family;
    if (family == count)
      throw std::runtime_error("Headset preview cannot present on this GPU");
    length =
        compositor->GetVulkanDeviceExtensionsRequired(physical, nullptr, 0);
    if (!length || length > 65536)
      throw std::runtime_error("Headset preview device extensions unavailable");
    required.assign(length, '\0');
    compositor->GetVulkanDeviceExtensionsRequired(physical, required.data(),
                                                  length);
    names = extensions(required.c_str(), VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    list = pointers(names);
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
    device_create.pNext = &timeline;
    device_create.queueCreateInfoCount = 1;
    device_create.pQueueCreateInfos = &queue_create;
    device_create.enabledExtensionCount = list.size();
    device_create.ppEnabledExtensionNames = list.data();
    check(vkCreateDevice(physical, &device_create, nullptr, &device), "device");
    vkGetDeviceQueue(device, family, 0, &queue);

    VkCommandPoolCreateInfo pool_create{
        VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pool_create.queueFamilyIndex = family;
    pool_create.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    check(vkCreateCommandPool(device, &pool_create, nullptr, &pool), "pool");
    VkCommandBufferAllocateInfo allocate{
        VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocate.commandPool = pool;
    allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocate.commandBufferCount = 1;
    check(vkAllocateCommandBuffers(device, &allocate, &command), "commands");
    VkFenceCreateInfo fence_create{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fence_create.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    check(vkCreateFence(device, &fence_create, nullptr, &fence), "fence");
    VkSemaphoreCreateInfo semaphore{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    check(vkCreateSemaphore(device, &semaphore, nullptr, &acquired),
          "semaphore");

    VkSamplerCreateInfo sample{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sample.magFilter = sample.minFilter = VK_FILTER_LINEAR;
    sample.addressModeU = sample.addressModeV = sample.addressModeW =
        VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    check(vkCreateSampler(device, &sample, nullptr, &sampler), "sampler");
    VkDescriptorSetLayoutBinding binding{
        0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
        VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};
    VkDescriptorSetLayoutCreateInfo set_create{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    set_create.bindingCount = 1;
    set_create.pBindings = &binding;
    check(vkCreateDescriptorSetLayout(device, &set_create, nullptr,
                                      &set_layout),
          "descriptor layout");
    VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                              uint32_t(sources.size())};
    VkDescriptorPoolCreateInfo descriptors{
        VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    descriptors.maxSets = sources.size();
    descriptors.poolSizeCount = 1;
    descriptors.pPoolSizes = &size;
    check(vkCreateDescriptorPool(device, &descriptors, nullptr,
                                 &descriptor_pool),
          "descriptor pool");
    constexpr size_t slots = std::tuple_size_v<decltype(sources)>;
    std::array<VkDescriptorSetLayout, slots> layouts;
    layouts.fill(set_layout);
    std::array<VkDescriptorSet, slots> sets{};
    VkDescriptorSetAllocateInfo set_allocate{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    set_allocate.descriptorPool = descriptor_pool;
    set_allocate.descriptorSetCount = layouts.size();
    set_allocate.pSetLayouts = layouts.data();
    check(vkAllocateDescriptorSets(device, &set_allocate, sets.data()),
          "descriptor sets");
    for (size_t i = 0; i < sources.size(); ++i)
      sources[i].set = sets[i];
    VkPushConstantRange push{VK_SHADER_STAGE_VERTEX_BIT |
                                 VK_SHADER_STAGE_FRAGMENT_BIT,
                             0, 20};
    VkPipelineLayoutCreateInfo pipeline_create{
        VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipeline_create.setLayoutCount = 1;
    pipeline_create.pSetLayouts = &set_layout;
    pipeline_create.pushConstantRangeCount = 1;
    pipeline_create.pPushConstantRanges = &push;
    check(vkCreatePipelineLayout(device, &pipeline_create, nullptr,
                                 &pipeline_layout),
          "pipeline layout");

    vr::ETrackedPropertyError error{};
    float hz = system->GetFloatTrackedDeviceProperty(
        vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float,
        &error);
    if (error == vr::TrackedProp_Success && hz >= 30 && hz <= 240)
      period = 1.0 / hz;
  }

  // Render pass and pipeline depend on the swapchain format only.
  void build_pipeline(VkFormat target) {
    if (pipeline && target == format)
      return;
    if (pipeline)
      vkDestroyPipeline(device, pipeline, nullptr);
    if (pass)
      vkDestroyRenderPass(device, pass, nullptr);
    pipeline = {};
    pass = {};
    format = target;
    VkAttachmentDescription color{};
    color.format = format;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    VkAttachmentReference reference{0,
                                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &reference;
    // The swapchain image is ready once the acquire semaphore wait at this
    // stage completes.
    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    VkRenderPassCreateInfo pass_create{
        VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    pass_create.attachmentCount = 1;
    pass_create.pAttachments = &color;
    pass_create.subpassCount = 1;
    pass_create.pSubpasses = &subpass;
    pass_create.dependencyCount = 1;
    pass_create.pDependencies = &dependency;
    check(vkCreateRenderPass(device, &pass_create, nullptr, &pass),
          "render pass");

    // Generated from headset_preview.vert/.frag by build.rs (glslc -mfmt=c).
    static constexpr uint32_t vertex_code[] =
#include "headset_preview_vert.inc"
        ;
    static constexpr uint32_t fragment_code[] =
#include "headset_preview_frag.inc"
        ;
    auto module = [&](const uint32_t *code, size_t bytes) {
      VkShaderModuleCreateInfo create{
          VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
      create.codeSize = bytes;
      create.pCode = code;
      VkShaderModule out{};
      check(vkCreateShaderModule(device, &create, nullptr, &out), "shader");
      return out;
    };
    VkShaderModule vertex = module(vertex_code, sizeof(vertex_code));
    VkShaderModule fragment = module(fragment_code, sizeof(fragment_code));
    VkPipelineShaderStageCreateInfo stages[2]{};
    for (auto &stage : stages) {
      stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      stage.pName = "main";
    }
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vertex;
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fragment;
    VkPipelineVertexInputStateCreateInfo input{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo assembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo viewport{
        VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewport.viewportCount = viewport.scissorCount = 1;
    VkPipelineRasterizationStateCreateInfo raster{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.lineWidth = 1;
    VkPipelineMultisampleStateCreateInfo multisample{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState blend{};
    blend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                           VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo blending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    blending.attachmentCount = 1;
    blending.pAttachments = &blend;
    VkDynamicState dynamic_states[] = {VK_DYNAMIC_STATE_VIEWPORT,
                                       VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamic{
        VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamic.dynamicStateCount = 2;
    dynamic.pDynamicStates = dynamic_states;
    VkGraphicsPipelineCreateInfo create{
        VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    create.stageCount = 2;
    create.pStages = stages;
    create.pVertexInputState = &input;
    create.pInputAssemblyState = &assembly;
    create.pViewportState = &viewport;
    create.pRasterizationState = &raster;
    create.pMultisampleState = &multisample;
    create.pColorBlendState = &blending;
    create.pDynamicState = &dynamic;
    create.layout = pipeline_layout;
    create.renderPass = pass;
    auto created = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &create,
                                             nullptr, &pipeline);
    vkDestroyShaderModule(device, vertex, nullptr);
    vkDestroyShaderModule(device, fragment, nullptr);
    check(created, "pipeline");
  }

  void destroy_targets() {
    for (auto &target : targets) {
      vkDestroyFramebuffer(device, target.framebuffer, nullptr);
      vkDestroyImageView(device, target.view, nullptr);
      vkDestroySemaphore(device, target.rendered, nullptr);
    }
    targets.clear();
    images.clear();
  }
  // Drops the swapchain and unmaps the subsurface so the webview below shows
  // status text. The VkSurfaceKHR is kept for the next swapchain.
  void hide() {
    if (swapchain) {
      vkDeviceWaitIdle(device);
      destroy_targets();
      vkDestroySwapchainKHR(device, swapchain, nullptr);
      swapchain = {};
    }
    if (mapped) {
      wl_surface_attach(wayland.surface, nullptr, 0, 0);
      wl_surface_commit(wayland.surface);
      wl_display_flush(wayland.display);
      mapped = false;
    }
  }

  // Returns true when a new swapchain was created.
  bool ensure_swapchain(const Layout &target) {
    VkExtent2D want{uint32_t(target.width * target.scale),
                    uint32_t(target.height * target.scale)};
    if (swapchain && !stale && want.width == requested.width &&
        want.height == requested.height && target.scale == scale)
      return false;
    if (!surface) {
      VkWaylandSurfaceCreateInfoKHR create{
          VK_STRUCTURE_TYPE_WAYLAND_SURFACE_CREATE_INFO_KHR};
      create.display = wayland.display;
      create.surface = wayland.surface;
      check(vkCreateWaylandSurfaceKHR(instance, &create, nullptr, &surface),
            "surface");
      VkBool32 supported = VK_FALSE;
      vkGetPhysicalDeviceSurfaceSupportKHR(physical, family, surface,
                                           &supported);
      if (!supported)
        throw std::runtime_error("Headset preview surface unsupported");
    }
    vkDeviceWaitIdle(device);
    if (target.scale != scale) {
      // Applies with the first commit of the new swapchain, whose extent is a
      // multiple of the scale.
      wl_surface_set_buffer_scale(wayland.surface, target.scale);
      scale = target.scale;
    }
    VkSurfaceCapabilitiesKHR caps{};
    check(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical, surface, &caps),
          "surface capabilities");
    uint32_t count = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical, surface, &count, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(count);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical, surface, &count,
                                         formats.data());
    if (formats.empty())
      throw std::runtime_error("Headset preview surface has no formats");
    // A UNORM target receives sRGB-encoded values from the shader.
    auto chosen = formats[0];
    for (auto candidate : formats)
      if ((candidate.format == VK_FORMAT_B8G8R8A8_UNORM ||
           candidate.format == VK_FORMAT_R8G8B8A8_UNORM) &&
          candidate.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
        chosen = candidate;
        break;
      }
    vkGetPhysicalDeviceSurfacePresentModesKHR(physical, surface, &count,
                                              nullptr);
    std::vector<VkPresentModeKHR> modes(count);
    vkGetPhysicalDeviceSurfacePresentModesKHR(physical, surface, &count,
                                              modes.data());
    // Never block the paced thread in present: the newest frame wins.
    auto mode = VK_PRESENT_MODE_FIFO_KHR;
    for (auto preferred :
         {VK_PRESENT_MODE_MAILBOX_KHR, VK_PRESENT_MODE_IMMEDIATE_KHR})
      if (std::find(modes.begin(), modes.end(), preferred) != modes.end()) {
        mode = preferred;
        break;
      }
    requested = want;
    if (caps.currentExtent.width != UINT32_MAX)
      want = caps.currentExtent;
    want.width = std::clamp(want.width, caps.minImageExtent.width,
                            caps.maxImageExtent.width);
    want.height = std::clamp(want.height, caps.minImageExtent.height,
                             caps.maxImageExtent.height);
    uint32_t image_count = std::max(caps.minImageCount, 3u);
    if (caps.maxImageCount)
      image_count = std::min(image_count, caps.maxImageCount);
    VkSwapchainCreateInfoKHR create{
        VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    create.surface = surface;
    create.minImageCount = image_count;
    create.imageFormat = chosen.format;
    create.imageColorSpace = chosen.colorSpace;
    create.imageExtent = want;
    create.imageArrayLayers = 1;
    create.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    create.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    create.preTransform = caps.currentTransform;
    create.compositeAlpha =
        (caps.supportedCompositeAlpha & VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR)
            ? VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR
            : VkCompositeAlphaFlagBitsKHR(caps.supportedCompositeAlpha &
                                          -caps.supportedCompositeAlpha);
    create.presentMode = mode;
    create.clipped = VK_TRUE;
    create.oldSwapchain = swapchain;
    VkSwapchainKHR fresh{};
    check(vkCreateSwapchainKHR(device, &create, nullptr, &fresh), "swapchain");
    destroy_targets();
    if (swapchain)
      vkDestroySwapchainKHR(device, swapchain, nullptr);
    swapchain = fresh;
    extent = want;
    stale = false;
    build_pipeline(chosen.format);
    vkGetSwapchainImagesKHR(device, swapchain, &count, nullptr);
    images.resize(count);
    vkGetSwapchainImagesKHR(device, swapchain, &count, images.data());
    targets.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
      VkImageViewCreateInfo view_create{
          VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
      view_create.image = images[i];
      view_create.viewType = VK_IMAGE_VIEW_TYPE_2D;
      view_create.format = format;
      view_create.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
      check(vkCreateImageView(device, &view_create, nullptr, &targets[i].view),
            "target view");
      VkFramebufferCreateInfo framebuffer{
          VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
      framebuffer.renderPass = pass;
      framebuffer.attachmentCount = 1;
      framebuffer.pAttachments = &targets[i].view;
      framebuffer.width = extent.width;
      framebuffer.height = extent.height;
      framebuffer.layers = 1;
      check(vkCreateFramebuffer(device, &framebuffer, nullptr,
                                &targets[i].framebuffer),
            "framebuffer");
      VkSemaphoreCreateInfo semaphore{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
      check(vkCreateSemaphore(device, &semaphore, nullptr,
                              &targets[i].rendered),
            "semaphore");
    }
    return true;
  }

  void forget_sources() {
    for (size_t i = 0; i < source_count; ++i) {
      vkDestroyImageView(device, sources[i].view, nullptr);
      sources[i].view = {};
      sources[i].image = {};
    }
    source_count = 0;
  }
  // GPU work must be finished (fence waited) before this frees the imports.
  void release() {
    forget_sources();
    if (holding && api)
      api->ReleaseOverlayView(&view);
    holding = false;
    view = {};
  }
  const Source *source_for(const vr::VRVulkanTextureData_t &texture) {
    auto image = reinterpret_cast<VkImage>(texture.m_nImage);
    for (size_t i = 0; i < source_count; ++i)
      if (sources[i].image == image)
        return &sources[i];
    if (source_count == sources.size())
      return nullptr;
    auto &source = sources[source_count];
    VkImageViewCreateInfo view_create{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    view_create.image = image;
    view_create.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_create.format = VkFormat(texture.m_nFormat);
    view_create.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    check(vkCreateImageView(device, &view_create, nullptr, &source.view),
          "source view");
    source.image = image;
    VkDescriptorImageInfo sampled{sampler, source.view,
                                  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    write.dstSet = source.set;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.pImageInfo = &sampled;
    vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    ++source_count;
    return &source;
  }

  // Microseconds until shortly after the next compositor vsync, when the
  // headset view for the new frame is ready.
  int64_t until_next_frame(vr::IVRSystem *system) {
    float since = 0;
    uint64_t frame = 0;
    if (!system->GetTimeSinceLastVsync(&since, &frame))
      return int64_t(period * 1e6);
    double wait = period - since + 0.001;
    while (wait < 0.0005)
      wait += period;
    return int64_t(wait * 1e6);
  }

  int64_t step(vr::IVRSystem *system, vr::IVROverlay *overlay,
               vr::IVRCompositor *compositor, const Layout &target) {
    check(vkWaitForFences(device, 1, &fence, VK_TRUE, 500000000),
          "frame wait");
    float since = 0;
    uint64_t frame = 0;
    bool timed = system->GetTimeSinceLastVsync(&since, &frame);
    bool resized = !swapchain || stale ||
                   requested.width != uint32_t(target.width * target.scale) ||
                   requested.height != uint32_t(target.height * target.scale);
    // One present per compositor frame. Layout wake-ups mid-frame only
    // present when the surface changed.
    if (timed && frame == last_frame && !resized)
      return until_next_frame(system);
    // Any GPU submit at display rate costs vrcompositor CPU (NVIDIA), so skip
    // frames that cannot differ: in Standby the compositor stops rendering
    // and the headset view is static. Refresh once a second regardless.
    // Both reads are shared memory.
    vr::Compositor_FrameTiming timing{};
    timing.m_nSize = sizeof(timing);
    bool new_index = !compositor->GetFrameTiming(&timing, 0) ||
                     timing.m_nFrameIndex != last_index;
    auto now = std::chrono::steady_clock::now();
    if (!resized && !new_index && now - last_present < std::chrono::seconds(1) &&
        system->GetTrackedDeviceActivityLevel(vr::k_unTrackedDeviceIndex_Hmd) ==
            vr::k_EDeviceActivityLevel_Standby)
      return until_next_frame(system);

    vr::VROverlayHandle_t handle{};
    if (overlay->FindOverlay(vr::k_pchHeadsetViewOverlayKey, &handle) !=
        vr::VROverlayError_None) {
      release();
      hide();
      set_error("Open VR View to preview");
      return retry_us;
    }
    if (holding && view.overlayHandle != handle)
      release();
    if (!holding && !api->IsViewingPermitted(handle)) {
      hide();
      set_error("Headset preview access unavailable");
      return retry_us;
    }
    vr::VRVulkanDevice_t vulkan{instance, device, physical, queue, family};
    vr::VRNativeDevice_t native{&vulkan, vr::DeviceType_Vulkan};
    auto acquired_view =
        api->AcquireOverlayView(handle, &native, &view, sizeof(view));
    // Even a failed or empty view may own native resources.
    holding = true;
    if (acquired_view != vr::VROverlayError_None) {
      release();
      hide();
      set_error("Headset preview acquisition error " +
                std::to_string(acquired_view));
      return retry_us;
    }
    if (!view.texture.handle) {
      hide();
      set_error(system->GetTrackedDeviceActivityLevel(
                    vr::k_unTrackedDeviceIndex_Hmd) ==
                        vr::k_EDeviceActivityLevel_Standby
                    ? "Put on the headset to preview"
                    : "Headset view has no frame");
      return retry_us;
    }
    if (view.texture.eType != vr::TextureType_Vulkan)
      throw std::runtime_error("Headset preview texture type unsupported");
    const auto &texture =
        *static_cast<vr::VRVulkanTextureData_t *>(view.texture.handle);
    const auto &bounds = view.textureBounds;
    if (texture.m_pDevice != device || texture.m_nQueueFamilyIndex != family ||
        texture.m_nSampleCount != 1 || !texture.m_nWidth ||
        !texture.m_nHeight || texture.m_nWidth > 16384 ||
        texture.m_nHeight > 16384)
      throw std::runtime_error("Headset preview texture unsupported");
    for (float value : {bounds.uMin, bounds.uMax, bounds.vMin, bounds.vMax})
      if (!std::isfinite(value) || value < 0 || value > 1)
        throw std::runtime_error("Headset preview bounds invalid");
    auto texture_format = VkFormat(texture.m_nFormat);
    if (texture.m_nWidth != source_width ||
        texture.m_nHeight != source_height || texture_format != source_format) {
      // A new size or format means the producer reallocated. Free the old
      // imports, then pick up the new image.
      VkFormatProperties properties{};
      vkGetPhysicalDeviceFormatProperties(physical, texture_format,
                                          &properties);
      if (!(properties.optimalTilingFeatures &
            VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT))
        throw std::runtime_error("Headset preview format unsupported");
      forget_sources();
      source_width = texture.m_nWidth;
      source_height = texture.m_nHeight;
      source_format = texture_format;
    }
    const Source *source = source_for(texture);
    if (!source) {
      release();
      return 0;
    }

    resized = ensure_swapchain(target) || resized;
    uint32_t index = 0;
    auto next = vkAcquireNextImageKHR(device, swapchain, 0, acquired,
                                      VK_NULL_HANDLE, &index);
    if (next == VK_ERROR_OUT_OF_DATE_KHR) {
      stale = true;
      return 0;
    }
    if (next == VK_NOT_READY || next == VK_TIMEOUT)
      return until_next_frame(system);
    if (next == VK_SUBOPTIMAL_KHR)
      stale = true;
    else
      check(next, "image acquire");

    float du = std::abs(bounds.uMax - bounds.uMin) * texture.m_nWidth;
    float dv = std::abs(bounds.vMax - bounds.vMin) * texture.m_nHeight;
    float fit = std::min(extent.width / du, extent.height / dv);
    VkViewport viewport{};
    viewport.width = std::round(du * fit);
    viewport.height = std::round(dv * fit);
    viewport.x = std::floor((extent.width - viewport.width) / 2);
    viewport.y = std::floor((extent.height - viewport.height) / 2);
    viewport.maxDepth = 1;
    VkRect2D scissor{{int32_t(viewport.x), int32_t(viewport.y)},
                     {uint32_t(viewport.width), uint32_t(viewport.height)}};

    check(vkResetCommandBuffer(command, 0), "command reset");
    VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    check(vkBeginCommandBuffer(command, &begin), "command begin");
    // The acquired image is sampleable and stable. Never write or transition
    // it; the fence wait keeps reads ahead of the next acquire/release.
    VkMemoryBarrier readable{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    readable.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
    readable.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                         VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 1, &readable,
                         0, nullptr, 0, nullptr);
    VkClearValue black{};
    black.color.float32[3] = 1;
    VkRenderPassBeginInfo pass_begin{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    pass_begin.renderPass = pass;
    pass_begin.framebuffer = targets[index].framebuffer;
    pass_begin.renderArea.extent = extent;
    pass_begin.clearValueCount = 1;
    pass_begin.pClearValues = &black;
    vkCmdBeginRenderPass(command, &pass_begin, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipeline_layout, 0, 1, &source->set, 0, nullptr);
    vkCmdSetViewport(command, 0, 1, &viewport);
    vkCmdSetScissor(command, 0, 1, &scissor);
    struct {
      float bounds[4];
      uint32_t srgb;
    } push{{bounds.uMin, bounds.vMin, bounds.uMax, bounds.vMax},
           uint32_t(srgb(texture_format))};
    vkCmdPushConstants(command, pipeline_layout,
                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                       0, sizeof(push), &push);
    vkCmdDraw(command, 3, 1, 0, 0);
    vkCmdEndRenderPass(command);
    check(vkEndCommandBuffer(command), "command end");
    check(vkResetFences(device, 1, &fence), "fence reset");
    VkPipelineStageFlags wait_stage =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submit.waitSemaphoreCount = 1;
    submit.pWaitSemaphores = &acquired;
    submit.pWaitDstStageMask = &wait_stage;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &command;
    submit.signalSemaphoreCount = 1;
    submit.pSignalSemaphores = &targets[index].rendered;
    check(vkQueueSubmit(queue, 1, &submit, fence), "submit");
    VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores = &targets[index].rendered;
    present.swapchainCount = 1;
    present.pSwapchains = &swapchain;
    present.pImageIndices = &index;
    auto presented = vkQueuePresentKHR(queue, &present);
    if (presented == VK_ERROR_OUT_OF_DATE_KHR ||
        presented == VK_SUBOPTIMAL_KHR)
      stale = true;
    else
      check(presented, "present");
    mapped = true;
    last_frame = frame;
    last_index = timing.m_nFrameIndex;
    last_present = now;
    {
      std::lock_guard guard(status_lock);
      status_error.clear();
      ++status_presented;
      status_width = texture.m_nWidth;
      status_height = texture.m_nHeight;
    }
    return until_next_frame(system);
  }

  ~Renderer() {
    if (device)
      vkDeviceWaitIdle(device);
    release();
    hide();
    if (surface)
      vkDestroySurfaceKHR(instance, surface, nullptr);
    if (pipeline)
      vkDestroyPipeline(device, pipeline, nullptr);
    if (pass)
      vkDestroyRenderPass(device, pass, nullptr);
    if (pipeline_layout)
      vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
    if (descriptor_pool)
      vkDestroyDescriptorPool(device, descriptor_pool, nullptr);
    if (set_layout)
      vkDestroyDescriptorSetLayout(device, set_layout, nullptr);
    if (sampler)
      vkDestroySampler(device, sampler, nullptr);
    if (acquired)
      vkDestroySemaphore(device, acquired, nullptr);
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
std::unique_ptr<Renderer> renderer;
} // namespace

extern "C" void rifts_surface_place(void *webview, int x, int y, int width,
                                    int height, bool visible) {
  auto *widget = static_cast<_GtkWidget *>(webview);
  _GtkWidget *top = gtk_widget_get_toplevel(widget);
  _GdkWindow *window = top ? gtk_widget_get_window(top) : nullptr;
  std::lock_guard guard(layout_lock);
  if (!window) {
    // Not realized yet; the frontend reports again on the next resize.
    layout.visible = false;
    return;
  }
  if (!g_type_check_instance_is_a(window, gdk_wayland_window_get_type()))
    wayland.unavailable = true;
  wl_surface *parent =
      wayland.unavailable ? nullptr : gdk_wayland_window_get_wl_surface(window);
  if (parent && !wayland.display)
    wayland.unavailable = !connect_wayland(gdk_wayland_display_get_wl_display(
        gdk_window_get_display(window)));
  if (!parent || wayland.unavailable) {
    layout.visible = false;
    return;
  }
  wl_display_dispatch_queue_pending(wayland.display, wayland.queue);
  if (parent != wayland.parent) {
    // GTK recreates the toplevel surface when the window is re-shown.
    if (wayland.subsurface)
      wl_subsurface_destroy(wayland.subsurface);
    wayland.subsurface = wl_subcompositor_get_subsurface(
        wayland.subcompositor, wayland.surface, parent);
    wl_subsurface_set_desync(wayland.subsurface);
    wayland.parent = parent;
  }
  int origin_x = 0, origin_y = 0;
  gtk_widget_translate_coordinates(widget, top, 0, 0, &origin_x, &origin_y);
  wl_subsurface_set_position(wayland.subsurface, origin_x + x, origin_y + y);
  // Position is parent state; make GTK commit the toplevel once more.
  gtk_widget_queue_draw(top);
  layout = {x, y, width, height, std::max(1, gdk_window_get_scale_factor(window)),
            visible && width > 0 && height > 0};
}

int64_t rifts_headset_preview_step(vr::IVRSystem *system,
                                   vr::IVROverlay *overlay,
                                   vr::IVROverlayView *api,
                                   vr::IVRCompositor *compositor) {
  Layout target;
  bool unavailable;
  {
    std::lock_guard guard(layout_lock);
    target = layout;
    unavailable = wayland.unavailable;
  }
  if (!target.visible) {
    if (renderer) {
      renderer->release();
      renderer->hide();
    }
    if (unavailable)
      set_error("Headset preview needs a Wayland session");
    return -1;
  }
  if (!system || !overlay || !api || !compositor) {
    rifts_headset_preview_shutdown();
    set_error("SteamVR is not running");
    return retry_us;
  }
  try {
    if (!renderer) {
      auto fresh = std::make_unique<Renderer>();
      fresh->initialize(system, compositor);
      renderer = std::move(fresh);
    }
    renderer->api = api;
    return renderer->step(system, overlay, compositor, target);
  } catch (const std::exception &failure) {
    set_error(failure.what());
    // Start from a clean device on the next attempt.
    renderer.reset();
    return retry_us;
  }
}

void rifts_headset_preview_shutdown() { renderer.reset(); }

extern "C" size_t rifts_preview_status(uint64_t *presented, uint32_t *width,
                                       uint32_t *height, char *error,
                                       size_t capacity) {
  std::lock_guard guard(status_lock);
  *presented = status_presented;
  *width = status_width;
  *height = status_height;
  size_t length = std::min(status_error.size(), capacity);
  std::memcpy(error, status_error.data(), length);
  return length;
}
