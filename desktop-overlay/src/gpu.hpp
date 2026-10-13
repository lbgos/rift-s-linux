// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// Vulkan side of the overlay: imports PipeWire DMA-BUFs as images that SteamVR
// copies into the dashboard on our queue, with a staging upload for SHM streams.
#pragma once

#include "vr.hpp"

#include <vulkan/vulkan.h>

#include <cstdint>
#include <string>
#include <vector>

namespace rd {

struct DmabufPlane
{
	int fd;
	uint32_t offset;
	uint32_t stride;
};

//! An image SteamVR can read. DMA-BUF images belong to the producer between
//! frames and are borrowed with begin()/end(). SHM images are our own copy.
struct GpuImage
{
	VkImage image = VK_NULL_HANDLE;
	VkDeviceMemory memory = VK_NULL_HANDLE;
	VkBuffer staging = VK_NULL_HANDLE;
	VkDeviceMemory staging_memory = VK_NULL_HANDLE;
	void *staging_map = nullptr;
	VkCommandBuffer acquire = VK_NULL_HANDLE; // dma-buf: foreign -> transfer src; shm: copy staging
	VkCommandBuffer release = VK_NULL_HANDLE; // dma-buf: back to the foreign queue
	uint32_t width = 0, height = 0;
};

class Gpu
{
public:
	~Gpu();

	bool
	init(const Vr &vr, std::string &error);

	//! DRM modifiers for B8G8R8A8 that this device can import and copy from.
	std::vector<uint64_t>
	import_modifiers() const;

	bool
	import_dmabuf(GpuImage &out, uint32_t width, uint32_t height, uint64_t modifier, const DmabufPlane *planes, uint32_t count);

	//! Copies a tightly or loosely packed BGRA frame into `image`, (re)creating it on size change.
	bool
	upload(GpuImage &image, uint32_t width, uint32_t height, const uint8_t *data, uint32_t stride);

	//! Makes a dma-buf image readable by SteamVR. No-op for SHM images.
	bool
	begin(GpuImage &image);

	//! Returns a dma-buf image to its producer once all queued reads finished.
	bool
	end(GpuImage &image);

	//! Debug: copies the image (between begin and end) into tightly packed BGRA.
	bool
	read_back(GpuImage &image, std::vector<uint8_t> &bgra);

	void
	destroy(GpuImage &image);

	vr::VRVulkanTextureData_t
	texture(const GpuImage &image) const;

	bool dmabuf_supported = false;

private:
	bool
	submit(VkCommandBuffer cmd, bool wait);
	VkCommandBuffer
	record_barrier(VkImage image, VkImageLayout from, VkImageLayout to, uint32_t src_family, uint32_t dst_family);
	int
	memory_type(uint32_t bits, VkMemoryPropertyFlags flags) const;

	VkInstance instance_ = VK_NULL_HANDLE;
	VkPhysicalDevice physical_ = VK_NULL_HANDLE;
	VkDevice device_ = VK_NULL_HANDLE;
	VkQueue queue_ = VK_NULL_HANDLE;
	uint32_t family_ = 0;
	VkCommandPool pool_ = VK_NULL_HANDLE;
	VkFence fence_ = VK_NULL_HANDLE;
	PFN_vkGetMemoryFdPropertiesKHR get_fd_properties_ = nullptr;
};

} // namespace rd
