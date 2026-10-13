// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "gpu.hpp"

#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <sstream>

namespace rd {
namespace {

constexpr VkFormat kFormat = VK_FORMAT_B8G8R8A8_UNORM;
constexpr VkImageUsageFlags kImportUsage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

std::vector<std::string>
words(const char *text)
{
	std::vector<std::string> out;
	std::istringstream in(text);
	for (std::string w; in >> w;) {
		out.push_back(w);
	}
	return out;
}

void
add_unique(std::vector<std::string> &list, const std::string &name)
{
	if (std::find(list.begin(), list.end(), name) == list.end()) {
		list.push_back(name);
	}
}

std::vector<const char *>
pointers(const std::vector<std::string> &list)
{
	std::vector<const char *> out;
	for (const auto &s : list) {
		out.push_back(s.c_str());
	}
	return out;
}

} // namespace

Gpu::~Gpu()
{
	if (device_) {
		vkDeviceWaitIdle(device_);
		vkDestroyFence(device_, fence_, nullptr);
		vkDestroyCommandPool(device_, pool_, nullptr);
		vkDestroyDevice(device_, nullptr);
	}
	if (instance_) {
		vkDestroyInstance(instance_, nullptr);
	}
}

bool
Gpu::init(const Vr &vr, std::string &error)
{
	char buffer[4096] = {};
	std::vector<std::string> instance_ext;
	if (vr.compositor) {
		vr.compositor->GetVulkanInstanceExtensionsRequired(buffer, sizeof(buffer));
		instance_ext = words(buffer);
	}
	VkApplicationInfo app = {VK_STRUCTURE_TYPE_APPLICATION_INFO};
	app.pApplicationName = "rifts-desktop";
	app.apiVersion = VK_API_VERSION_1_2;
	auto instance_names = pointers(instance_ext);
	VkInstanceCreateInfo ici = {VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
	ici.pApplicationInfo = &app;
	ici.enabledExtensionCount = uint32_t(instance_names.size());
	ici.ppEnabledExtensionNames = instance_names.data();
	if (vkCreateInstance(&ici, nullptr, &instance_) != VK_SUCCESS) {
		error = "vkCreateInstance failed";
		return false;
	}

	// SteamVR copies overlay textures on our queue, so we must use its GPU.
	uint64_t output = 0;
	if (vr.system) {
		vr.system->GetOutputDevice(&output, vr::TextureType_Vulkan, instance_);
	}
	physical_ = reinterpret_cast<VkPhysicalDevice>(output);
	if (!physical_) {
		uint32_t count = 1;
		vkEnumeratePhysicalDevices(instance_, &count, &physical_);
	}
	if (!physical_) {
		error = "no Vulkan device";
		return false;
	}

	uint32_t families = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(physical_, &families, nullptr);
	std::vector<VkQueueFamilyProperties> fprops(families);
	vkGetPhysicalDeviceQueueFamilyProperties(physical_, &families, fprops.data());
	family_ = UINT32_MAX;
	for (uint32_t i = 0; i < families; i++) {
		if (fprops[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
			family_ = i;
			break;
		}
	}
	if (family_ == UINT32_MAX) {
		error = "no graphics queue";
		return false;
	}

	uint32_t available_count = 0;
	vkEnumerateDeviceExtensionProperties(physical_, nullptr, &available_count, nullptr);
	std::vector<VkExtensionProperties> available(available_count);
	vkEnumerateDeviceExtensionProperties(physical_, nullptr, &available_count, available.data());
	auto has = [&](const char *name) {
		return std::any_of(available.begin(), available.end(),
		                   [&](const VkExtensionProperties &p) { return strcmp(p.extensionName, name) == 0; });
	};

	std::vector<std::string> device_ext;
	if (vr.compositor) {
		buffer[0] = '\0';
		vr.compositor->GetVulkanDeviceExtensionsRequired(physical_, buffer, sizeof(buffer));
		device_ext = words(buffer);
	}
	const char *dmabuf_ext[] = {VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME, VK_EXT_EXTERNAL_MEMORY_DMA_BUF_EXTENSION_NAME,
	                            VK_EXT_IMAGE_DRM_FORMAT_MODIFIER_EXTENSION_NAME,
	                            VK_EXT_QUEUE_FAMILY_FOREIGN_EXTENSION_NAME};
	dmabuf_supported = std::all_of(std::begin(dmabuf_ext), std::end(dmabuf_ext), has);
	if (dmabuf_supported) {
		for (const char *name : dmabuf_ext) {
			add_unique(device_ext, name);
		}
	}

	float priority = 1.0f;
	VkDeviceQueueCreateInfo qci = {VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
	qci.queueFamilyIndex = family_;
	qci.queueCount = 1;
	qci.pQueuePriorities = &priority;
	auto device_names = pointers(device_ext);
	VkDeviceCreateInfo dci = {VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
	dci.queueCreateInfoCount = 1;
	dci.pQueueCreateInfos = &qci;
	dci.enabledExtensionCount = uint32_t(device_names.size());
	dci.ppEnabledExtensionNames = device_names.data();
	if (vkCreateDevice(physical_, &dci, nullptr, &device_) != VK_SUCCESS) {
		error = "vkCreateDevice failed";
		return false;
	}
	vkGetDeviceQueue(device_, family_, 0, &queue_);

	VkCommandPoolCreateInfo pci = {VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
	pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	pci.queueFamilyIndex = family_;
	VkFenceCreateInfo fci = {VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
	if (vkCreateCommandPool(device_, &pci, nullptr, &pool_) != VK_SUCCESS ||
	    vkCreateFence(device_, &fci, nullptr, &fence_) != VK_SUCCESS) {
		error = "Vulkan command setup failed";
		return false;
	}
	if (dmabuf_supported) {
		get_fd_properties_ = reinterpret_cast<PFN_vkGetMemoryFdPropertiesKHR>(
		    vkGetDeviceProcAddr(device_, "vkGetMemoryFdPropertiesKHR"));
		dmabuf_supported = get_fd_properties_ != nullptr;
	}
	return true;
}

std::vector<uint64_t>
Gpu::import_modifiers() const
{
	std::vector<uint64_t> out;
	if (!dmabuf_supported) {
		return out;
	}
	VkDrmFormatModifierPropertiesListEXT list = {VK_STRUCTURE_TYPE_DRM_FORMAT_MODIFIER_PROPERTIES_LIST_EXT};
	VkFormatProperties2 props = {VK_STRUCTURE_TYPE_FORMAT_PROPERTIES_2, &list};
	vkGetPhysicalDeviceFormatProperties2(physical_, kFormat, &props);
	std::vector<VkDrmFormatModifierPropertiesEXT> mods(list.drmFormatModifierCount);
	list.pDrmFormatModifierProperties = mods.data();
	vkGetPhysicalDeviceFormatProperties2(physical_, kFormat, &props);

	for (const auto &m : mods) {
		if (!(m.drmFormatModifierTilingFeatures & VK_FORMAT_FEATURE_TRANSFER_SRC_BIT)) {
			continue;
		}
		VkPhysicalDeviceImageDrmFormatModifierInfoEXT mod_info = {
		    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_DRM_FORMAT_MODIFIER_INFO_EXT};
		mod_info.drmFormatModifier = m.drmFormatModifier;
		mod_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		VkPhysicalDeviceExternalImageFormatInfo ext_info = {
		    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_IMAGE_FORMAT_INFO, &mod_info};
		ext_info.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
		VkPhysicalDeviceImageFormatInfo2 info = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_FORMAT_INFO_2, &ext_info};
		info.format = kFormat;
		info.type = VK_IMAGE_TYPE_2D;
		info.tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT;
		info.usage = kImportUsage;
		VkExternalImageFormatProperties ext_props = {VK_STRUCTURE_TYPE_EXTERNAL_IMAGE_FORMAT_PROPERTIES};
		VkImageFormatProperties2 out_props = {VK_STRUCTURE_TYPE_IMAGE_FORMAT_PROPERTIES_2, &ext_props};
		if (vkGetPhysicalDeviceImageFormatProperties2(physical_, &info, &out_props) == VK_SUCCESS &&
		    (ext_props.externalMemoryProperties.externalMemoryFeatures &
		     VK_EXTERNAL_MEMORY_FEATURE_IMPORTABLE_BIT)) {
			out.push_back(m.drmFormatModifier);
		}
	}
	return out;
}

int
Gpu::memory_type(uint32_t bits, VkMemoryPropertyFlags flags) const
{
	VkPhysicalDeviceMemoryProperties props;
	vkGetPhysicalDeviceMemoryProperties(physical_, &props);
	for (uint32_t i = 0; i < props.memoryTypeCount; i++) {
		if ((bits & (1u << i)) && (props.memoryTypes[i].propertyFlags & flags) == flags) {
			return int(i);
		}
	}
	return -1;
}

VkCommandBuffer
Gpu::record_barrier(VkImage image, VkImageLayout from, VkImageLayout to, uint32_t src_family, uint32_t dst_family)
{
	VkCommandBufferAllocateInfo ai = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
	ai.commandPool = pool_;
	ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	ai.commandBufferCount = 1;
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	if (vkAllocateCommandBuffers(device_, &ai, &cmd) != VK_SUCCESS) {
		return VK_NULL_HANDLE;
	}
	VkCommandBufferBeginInfo bi = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
	vkBeginCommandBuffer(cmd, &bi);
	VkImageMemoryBarrier b = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
	b.srcAccessMask = 0;
	b.dstAccessMask = to == VK_IMAGE_LAYOUT_GENERAL ? 0 : VK_ACCESS_MEMORY_READ_BIT;
	b.oldLayout = from;
	b.newLayout = to;
	b.srcQueueFamilyIndex = src_family;
	b.dstQueueFamilyIndex = dst_family;
	b.image = image;
	b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0,
	                     nullptr, 1, &b);
	vkEndCommandBuffer(cmd);
	return cmd;
}

bool
Gpu::submit(VkCommandBuffer cmd, bool wait)
{
	VkSubmitInfo si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
	si.commandBufferCount = 1;
	si.pCommandBuffers = &cmd;
	if (vkQueueSubmit(queue_, 1, &si, wait ? fence_ : VK_NULL_HANDLE) != VK_SUCCESS) {
		return false;
	}
	if (wait) {
		bool ok = vkWaitForFences(device_, 1, &fence_, VK_TRUE, 1000000000ull) == VK_SUCCESS;
		vkResetFences(device_, 1, &fence_);
		return ok;
	}
	return true;
}

bool
Gpu::import_dmabuf(GpuImage &out, uint32_t width, uint32_t height, uint64_t modifier, const DmabufPlane *planes,
                   uint32_t count)
{
	if (!dmabuf_supported || count == 0 || count > 4) {
		return false;
	}
	VkSubresourceLayout layouts[4] = {};
	for (uint32_t i = 0; i < count; i++) {
		layouts[i].offset = planes[i].offset;
		layouts[i].rowPitch = planes[i].stride;
	}
	VkImageDrmFormatModifierExplicitCreateInfoEXT explicit_info = {
	    VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_EXPLICIT_CREATE_INFO_EXT};
	explicit_info.drmFormatModifier = modifier;
	explicit_info.drmFormatModifierPlaneCount = count;
	explicit_info.pPlaneLayouts = layouts;
	VkExternalMemoryImageCreateInfo ext = {VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO, &explicit_info};
	ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
	VkImageCreateInfo ci = {VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO, &ext};
	ci.imageType = VK_IMAGE_TYPE_2D;
	ci.format = kFormat;
	ci.extent = {width, height, 1};
	ci.mipLevels = 1;
	ci.arrayLayers = 1;
	ci.samples = VK_SAMPLE_COUNT_1_BIT;
	ci.tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT;
	ci.usage = kImportUsage;
	ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	if (vkCreateImage(device_, &ci, nullptr, &out.image) != VK_SUCCESS) {
		return false;
	}

	// All planes of the formats we negotiate live in one buffer object.
	VkMemoryFdPropertiesKHR fd_props = {VK_STRUCTURE_TYPE_MEMORY_FD_PROPERTIES_KHR};
	get_fd_properties_(device_, VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT, planes[0].fd, &fd_props);
	VkMemoryRequirements req;
	vkGetImageMemoryRequirements(device_, out.image, &req);
	int type = memory_type(req.memoryTypeBits & fd_props.memoryTypeBits, 0);
	int fd = type < 0 ? -1 : dup(planes[0].fd);
	if (fd < 0) {
		destroy(out);
		return false;
	}
	VkImportMemoryFdInfoKHR import = {VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR};
	import.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
	import.fd = fd; // owned by Vulkan on success
	VkMemoryDedicatedAllocateInfo dedicated = {VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO, &import};
	dedicated.image = out.image;
	VkMemoryAllocateInfo ai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, &dedicated};
	ai.allocationSize = req.size;
	ai.memoryTypeIndex = uint32_t(type);
	if (vkAllocateMemory(device_, &ai, nullptr, &out.memory) != VK_SUCCESS) {
		close(fd);
		destroy(out);
		return false;
	}
	if (vkBindImageMemory(device_, out.image, out.memory, 0) != VK_SUCCESS) {
		destroy(out);
		return false;
	}
	// The producer renders between our frames, so each frame borrows the image
	// from the foreign queue and gives it back once SteamVR's copy is done.
	out.acquire = record_barrier(out.image, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
	                             VK_QUEUE_FAMILY_FOREIGN_EXT, family_);
	out.release = record_barrier(out.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, family_,
	                             VK_QUEUE_FAMILY_FOREIGN_EXT);
	out.width = width;
	out.height = height;
	if (!out.acquire || !out.release) {
		destroy(out);
		return false;
	}
	return true;
}

bool
Gpu::upload(GpuImage &image, uint32_t width, uint32_t height, const uint8_t *data, uint32_t stride)
{
	if (image.width != width || image.height != height || !image.staging) {
		destroy(image);
		VkImageCreateInfo ci = {VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
		ci.imageType = VK_IMAGE_TYPE_2D;
		ci.format = kFormat;
		ci.extent = {width, height, 1};
		ci.mipLevels = 1;
		ci.arrayLayers = 1;
		ci.samples = VK_SAMPLE_COUNT_1_BIT;
		ci.tiling = VK_IMAGE_TILING_OPTIMAL;
		ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | kImportUsage;
		ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		VkBufferCreateInfo bci = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
		bci.size = VkDeviceSize(width) * height * 4;
		bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		if (vkCreateImage(device_, &ci, nullptr, &image.image) != VK_SUCCESS ||
		    vkCreateBuffer(device_, &bci, nullptr, &image.staging) != VK_SUCCESS) {
			destroy(image);
			return false;
		}
		VkMemoryRequirements ireq, breq;
		vkGetImageMemoryRequirements(device_, image.image, &ireq);
		vkGetBufferMemoryRequirements(device_, image.staging, &breq);
		int itype = memory_type(ireq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
		int btype = memory_type(breq.memoryTypeBits,
		                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
		VkMemoryAllocateInfo iai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, ireq.size, uint32_t(itype)};
		VkMemoryAllocateInfo bai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, breq.size, uint32_t(btype)};
		if (itype < 0 || btype < 0 || vkAllocateMemory(device_, &iai, nullptr, &image.memory) != VK_SUCCESS ||
		    vkAllocateMemory(device_, &bai, nullptr, &image.staging_memory) != VK_SUCCESS ||
		    vkBindImageMemory(device_, image.image, image.memory, 0) != VK_SUCCESS ||
		    vkBindBufferMemory(device_, image.staging, image.staging_memory, 0) != VK_SUCCESS ||
		    vkMapMemory(device_, image.staging_memory, 0, VK_WHOLE_SIZE, 0, &image.staging_map) != VK_SUCCESS) {
			destroy(image);
			return false;
		}

		VkCommandBufferAllocateInfo ai = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
		ai.commandPool = pool_;
		ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		ai.commandBufferCount = 1;
		vkAllocateCommandBuffers(device_, &ai, &image.acquire);
		VkCommandBufferBeginInfo bi = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
		vkBeginCommandBuffer(image.acquire, &bi);
		VkImageMemoryBarrier b = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
		b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		b.image = image.image;
		b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vkCmdPipelineBarrier(image.acquire, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0,
		                     nullptr, 0, nullptr, 1, &b);
		VkBufferImageCopy region = {};
		region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		region.imageExtent = {width, height, 1};
		vkCmdCopyBufferToImage(image.acquire, image.staging, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
		                       &region);
		b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		b.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		vkCmdPipelineBarrier(image.acquire, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
		                     nullptr, 0, nullptr, 1, &b);
		vkEndCommandBuffer(image.acquire);
		image.width = width;
		image.height = height;
	}
	auto *dst = static_cast<uint8_t *>(image.staging_map);
	const size_t row = size_t(width) * 4;
	for (uint32_t y = 0; y < height; y++) {
		memcpy(dst + y * row, data + size_t(y) * stride, row);
	}
	return submit(image.acquire, true);
}

bool
Gpu::begin(GpuImage &image)
{
	return !image.release || submit(image.acquire, false);
}

bool
Gpu::end(GpuImage &image)
{
	return !image.release || submit(image.release, true);
}

bool
Gpu::read_back(GpuImage &image, std::vector<uint8_t> &bgra)
{
	const VkDeviceSize size = VkDeviceSize(image.width) * image.height * 4;
	VkBufferCreateInfo bci = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
	bci.size = size;
	bci.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
	VkBuffer buffer = VK_NULL_HANDLE;
	VkDeviceMemory memory = VK_NULL_HANDLE;
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	bool ok = vkCreateBuffer(device_, &bci, nullptr, &buffer) == VK_SUCCESS;
	if (ok) {
		VkMemoryRequirements req;
		vkGetBufferMemoryRequirements(device_, buffer, &req);
		int type = memory_type(req.memoryTypeBits,
		                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
		VkMemoryAllocateInfo ai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, uint32_t(type)};
		ok = type >= 0 && vkAllocateMemory(device_, &ai, nullptr, &memory) == VK_SUCCESS &&
		     vkBindBufferMemory(device_, buffer, memory, 0) == VK_SUCCESS;
	}
	if (ok) {
		VkCommandBufferAllocateInfo ai = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
		ai.commandPool = pool_;
		ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		ai.commandBufferCount = 1;
		ok = vkAllocateCommandBuffers(device_, &ai, &cmd) == VK_SUCCESS;
	}
	if (ok) {
		VkCommandBufferBeginInfo bi = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
		vkBeginCommandBuffer(cmd, &bi);
		VkBufferImageCopy region = {};
		region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		region.imageExtent = {image.width, image.height, 1};
		vkCmdCopyImageToBuffer(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buffer, 1, &region);
		vkEndCommandBuffer(cmd);
		void *map = nullptr;
		ok = submit(cmd, true) && vkMapMemory(device_, memory, 0, size, 0, &map) == VK_SUCCESS;
		if (ok) {
			bgra.assign(static_cast<uint8_t *>(map), static_cast<uint8_t *>(map) + size);
			vkUnmapMemory(device_, memory);
		}
		vkFreeCommandBuffers(device_, pool_, 1, &cmd);
	}
	vkDestroyBuffer(device_, buffer, nullptr);
	vkFreeMemory(device_, memory, nullptr);
	return ok;
}

void
Gpu::destroy(GpuImage &image)
{
	if (!device_) {
		return;
	}
	VkCommandBuffer cmds[] = {image.acquire, image.release};
	for (VkCommandBuffer cmd : cmds) {
		if (cmd) {
			vkFreeCommandBuffers(device_, pool_, 1, &cmd);
		}
	}
	vkDestroyImage(device_, image.image, nullptr);
	vkFreeMemory(device_, image.memory, nullptr);
	vkDestroyBuffer(device_, image.staging, nullptr);
	vkFreeMemory(device_, image.staging_memory, nullptr);
	image = GpuImage{};
}

vr::VRVulkanTextureData_t
Gpu::texture(const GpuImage &image) const
{
	vr::VRVulkanTextureData_t t = {};
	t.m_nImage = reinterpret_cast<uint64_t>(image.image);
	t.m_pDevice = device_;
	t.m_pPhysicalDevice = physical_;
	t.m_pInstance = instance_;
	t.m_pQueue = queue_;
	t.m_nQueueFamilyIndex = family_;
	t.m_nWidth = image.width;
	t.m_nHeight = image.height;
	t.m_nFormat = kFormat;
	t.m_nSampleCount = 1;
	return t;
}

} // namespace rd
