// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// SteamVR 2.17's sparse room-setup SDF upload needs defined storage, including
// before its first stroke. Its playspace meshes also upload 32-bit indices but
// bind them as 16-bit. Recognize those uploads without inspecting shader code.
#include <vulkan/vulkan.h>
#include <vulkan/vk_layer.h>
#include "rifts_passthrough_shm.h"
#include "ovrd_roomsetup_frame.hpp"
#include <algorithm>
#include <cstdio>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <optional>
#include <array>
#include <limits>
#include <unordered_map>
#include <unistd.h>
#include <vector>
#include <elf.h>
#include <sys/uio.h>
#include <sys/syscall.h>

namespace {
struct Instance
{
	VkInstance handle;
	PFN_vkGetInstanceProcAddr gipa;
	bool enabled;
};
struct Image
{
	VkExtent3D extent;
	bool empty;
	bool linear;
	bool initialized = false;
	VkDeviceMemory memory = VK_NULL_HANDLE;
	VkDeviceSize offset = 0;
	VkSubresourceLayout layout{};
};
struct IndexUpload
{
	uint32_t count;
	uint32_t maximum;
};
struct Buffer
{
	VkDeviceSize size;
	VkBufferUsageFlags usage;
	VkDeviceMemory memory = VK_NULL_HANDLE;
	VkDeviceSize memory_offset = 0;
	std::optional<IndexUpload> indices = std::nullopt;
	uint64_t id = 0;
	uint64_t fixes = 0;
};
struct Mapping
{
	void *data;
	VkDeviceSize offset;
	VkDeviceSize size;
};
// SteamVR 2.17.10 Linux x86-64 stores the mapped pointer, buffer, memory,
// and used bytes consecutively. The Vulkan objects provide the capacity.
// Only this exact executable build is supported; no executable bytes change.
struct RingFields
{
	uintptr_t mapped;
	VkBuffer buffer;
	VkDeviceMemory memory;
	uint64_t used;
};
static_assert(sizeof(RingFields) == 32 && offsetof(RingFields, used) == 24);
constexpr uint64_t ring_capacity = 256ULL * 1024 * 1024;
constexpr uint64_t ring_margin = 64ULL * 1024 * 1024;

bool local_read(uintptr_t address, void *out, size_t bytes)
{
	iovec local{out, bytes}, remote{reinterpret_cast<void *>(address), bytes};
	return syscall(SYS_process_vm_readv, getpid(), &local, 1, &remote, 1, 0) == static_cast<ssize_t>(bytes);
}
bool local_write(uintptr_t address, uint64_t value)
{
	iovec local{&value, sizeof(value)}, remote{reinterpret_cast<void *>(address), sizeof(value)};
	return syscall(SYS_process_vm_writev, getpid(), &local, 1, &remote, 1, 0) == static_cast<ssize_t>(sizeof(value));
}

bool supported_ring_build()
{
	// GNU build IDs identify the supported SteamVR executable without a code signature.
	constexpr std::array<unsigned char, 20> supported{
	    0xf5, 0xc5, 0x60, 0xdc, 0xaa, 0x16, 0xd7, 0xb2, 0x62, 0x5e,
	    0xce, 0x68, 0x4b, 0x02, 0x39, 0x65, 0x05, 0xe0, 0x76, 0x8f};
	int fd = open("/proc/self/exe", O_RDONLY | O_CLOEXEC);
	if (fd < 0) return false;
	Elf64_Ehdr header{};
	bool valid = pread(fd, &header, sizeof(header), 0) == sizeof(header) &&
	             memcmp(header.e_ident, ELFMAG, SELFMAG) == 0 && header.e_ident[EI_CLASS] == ELFCLASS64 &&
	             header.e_ident[EI_DATA] == ELFDATA2LSB && header.e_machine == EM_X86_64 &&
	             header.e_phentsize == sizeof(Elf64_Phdr) && header.e_phnum < 128;
	bool matched = false;
	for (uint16_t i = 0; valid && i < header.e_phnum; ++i) {
		Elf64_Phdr segment{};
		if (header.e_phoff > INT64_MAX - uint64_t(i) * sizeof(segment) ||
		    pread(fd, &segment, sizeof(segment), header.e_phoff + uint64_t(i) * sizeof(segment)) != sizeof(segment)) break;
		if (segment.p_type != PT_NOTE || segment.p_filesz > 65536 || segment.p_offset > INT64_MAX) continue;
		std::vector<unsigned char> bytes(segment.p_filesz);
		if (pread(fd, bytes.data(), bytes.size(), segment.p_offset) != static_cast<ssize_t>(bytes.size())) continue;
		for (size_t offset = 0; offset + sizeof(Elf64_Nhdr) <= bytes.size();) {
			Elf64_Nhdr note{};
			memcpy(&note, bytes.data() + offset, sizeof(note));
			offset += sizeof(note);
			uint64_t name_size = (uint64_t(note.n_namesz) + 3) & ~uint64_t{3};
			uint64_t data_size = (uint64_t(note.n_descsz) + 3) & ~uint64_t{3};
			if (name_size > bytes.size() - offset || data_size > bytes.size() - offset - name_size) break;
			if (note.n_type == NT_GNU_BUILD_ID && note.n_namesz == 4 && note.n_descsz == supported.size() &&
			    memcmp(bytes.data() + offset, "GNU", 4) == 0 &&
			    memcmp(bytes.data() + offset + name_size, supported.data(), supported.size()) == 0) matched = true;
			offset += name_size + data_size;
		}
	}
	close(fd);
	return matched;
}

struct RingCommand
{
	bool submitted = false;
	bool ended = false;
};
struct RingRecovery
{
	bool disabled = false;
	uintptr_t slot = 0;
	RingFields identity{};
	VkQueue queue = VK_NULL_HANDLE;
	pid_t owner = 0;
	uint64_t copied_end = 0;
	uint64_t resets = 0;
	std::unordered_map<VkCommandBuffer, RingCommand> commands;

	void disable(const char *reason)
	{
		if (!disabled) fprintf(stderr, "monado-ring: disabled: %s\n", reason);
		disabled = true;
	}
	bool matches(const RingFields &value, const Buffer &buffer, const Mapping &map) const
	{
		return value.mapped == reinterpret_cast<uintptr_t>(map.data) && value.buffer == identity.buffer &&
		       value.memory == identity.memory && value.used <= ring_capacity &&
		       buffer.size == ring_capacity && buffer.usage == VK_BUFFER_USAGE_TRANSFER_SRC_BIT &&
		       buffer.memory == identity.memory && buffer.memory_offset == 0 && map.offset == 0 &&
		       (map.size == VK_WHOLE_SIZE || map.size >= ring_capacity);
	}
	bool pending() const
	{
		return std::any_of(commands.begin(), commands.end(), [](const auto &entry) { return !entry.second.submitted; });
	}
	bool same_thread(bool establish = false)
	{
		auto current = static_cast<pid_t>(syscall(SYS_gettid));
		// Startup records uploads on the initializing thread and hands its command
		// buffer to Render. Establish ownership only after that first submission.
		if (owner && owner != current) { disable("upload author changed after first submit"); return false; }
		if (establish) owner = current;
		return true;
	}
	void copy(VkCommandBuffer command, uint64_t end)
	{
		if (disabled || !same_thread()) return;
		auto entry = commands.try_emplace(command).first;
		if (entry->second.submitted || entry->second.ended) { disable("copy outside a fresh recording"); return; }
		if (end != UINT64_MAX && end > ring_capacity) { disable("copy outside the ring"); return; }
		copied_end = end;
	}
	bool submitted(VkQueue target, const std::vector<VkCommandBuffer> &batch)
	{
		bool touched = false;
		for (auto command : batch) {
			auto found = commands.find(command);
			if (found == commands.end()) continue;
			if (found->second.submitted || !found->second.ended) { disable("upload resubmitted or still recording"); return false; }
			found->second.submitted = true;
			touched = true;
		}
		if (!touched || disabled) return false;
		if (queue && queue != target) { disable("transfer queue changed"); return false; }
		queue = target;
		return same_thread(true);
	}
	bool ready(uint64_t used) const
	{
		return !disabled && queue && !pending() && used == copied_end && used >= ring_capacity - ring_margin && used <= ring_capacity;
	}
};

struct Draw
{
	VkPipeline pipeline = VK_NULL_HANDLE;
	VkBuffer vertex = VK_NULL_HANDLE;
	VkBuffer index = VK_NULL_HANDLE;
	VkDeviceSize vertex_offset = 0;
	VkDeviceSize index_offset = 0;
	VkIndexType index_type = VK_INDEX_TYPE_UINT16;
};
struct Device
{
	PFN_vkGetDeviceProcAddr gdpa;
	VkDevice handle;
	bool enabled;
	std::unordered_map<VkImage, Image> images{};
	std::unordered_map<VkBuffer, Buffer> buffers{};
	std::unordered_map<VkPipeline, uint32_t> strides{};
	std::unordered_map<VkCommandBuffer, Draw> draws{};
	std::unordered_map<VkDeviceMemory, Mapping> mappings{};
	bool index_fix = true;
	bool sdf_fix = true;
	bool ring_allowed = false;
	std::unordered_map<VkBuffer, RingRecovery> rings{};
	std::unordered_map<VkImage, VkFormat> formats{};
	uint64_t next_buffer_id = 0;
};
std::mutex mutex;
std::unordered_map<void *, Instance> instances;
std::unordered_map<void *, std::shared_ptr<Device>> devices;

void *key(const void *handle)
{
	return *static_cast<void *const *>(handle);
}

bool monado_compositor()
{
	char executable[4096];
	ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
	if (length <= 0) return false;
	executable[length] = '\0';
	const char *name = strrchr(executable, '/');
	if (!name || strcmp(name + 1, "vrcompositor") != 0) return false;
	int fd = open(RIFTS_PASSTHROUGH_SHM_PATH, O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
	if (fd < 0) return false;
	uint32_t header[4]{};
	ssize_t count = pread(fd, header, sizeof(header), 0);
	struct flock owner{};
	owner.l_type = F_RDLCK;
	owner.l_whence = SEEK_SET;
	bool live = fcntl(fd, F_GETLK, &owner) == 0 && owner.l_type == F_WRLCK && owner.l_pid > 0;
	close(fd);
	return live && count == sizeof(header) && header[0] == RIFTS_PASSTHROUGH_MAGIC &&
	       header[1] == RIFTS_PASSTHROUGH_VERSION && header[2] == 640 && header[3] == 320;
}

std::shared_ptr<Device> device_for(const void *handle)
{
	std::lock_guard guard(mutex);
	auto found = devices.find(key(handle));
	return found == devices.end() ? nullptr : found->second;
}

template <typename T> T next(const std::shared_ptr<Device> &device, const char *name)
{
	return reinterpret_cast<T>(device->gdpa(device->handle, name));
}

void identify_ring(const std::shared_ptr<Device> &d, VkDeviceMemory memory, void **out)
{
	if (!d->ring_allowed) return;
	for (const auto &[buffer, state] : d->buffers) {
		if (state.memory != memory || state.size != ring_capacity || state.usage != VK_BUFFER_USAGE_TRANSFER_SRC_BIT) continue;
		if (d->rings.count(buffer)) { d->rings.at(buffer).disable("ring remapped"); continue; }
		RingRecovery recovery{};
		recovery.slot = reinterpret_cast<uintptr_t>(out);
		recovery.identity.buffer = buffer;
		recovery.identity.memory = memory;
		RingFields fields{};
		if (local_read(recovery.slot, &fields, sizeof(fields)) && recovery.matches(fields, state, d->mappings.at(memory))) {
			recovery.identity = fields;
			fprintf(stderr, "monado-ring: allocator validated, capacity=%llu\n", (unsigned long long)ring_capacity);
		} else recovery.disable("allocator layout or mapping mismatch");
		d->rings.emplace(buffer, std::move(recovery));
	}
}
std::optional<RingFields> validate_ring(const std::shared_ptr<Device> &d, RingRecovery &ring)
{
	if (ring.disabled) return std::nullopt;
	auto buffer = d->buffers.find(ring.identity.buffer);
	auto mapping = d->mappings.find(ring.identity.memory);
	RingFields fields{};
	if (buffer == d->buffers.end() || mapping == d->mappings.end() ||
	    !local_read(ring.slot, &fields, sizeof(fields)) || !ring.matches(fields, buffer->second, mapping->second)) {
		ring.disable("allocator identity changed");
		return std::nullopt;
	}
	return fields;
}
void ring_copy(const std::shared_ptr<Device> &d, VkCommandBuffer command, VkBuffer buffer, uint64_t end)
{
	auto ring = d->rings.find(buffer);
	if (ring == d->rings.end() || !validate_ring(d, ring->second)) return;
	ring->second.copy(command, end);
}
void recover_rings(const std::shared_ptr<Device> &d, VkQueue queue, const std::vector<VkCommandBuffer> &commands)
{
	for (auto &[buffer, ring] : d->rings) {
		if (ring.disabled || !ring.submitted(queue, commands)) continue;
		auto before = validate_ring(d, ring);
		if (!before || !ring.ready(before->used)) continue;
		auto wait = next<PFN_vkQueueWaitIdle>(d, "vkQueueWaitIdle");
		if (!wait) { ring.disable("transfer queue wait unavailable"); continue; }
		auto start = std::chrono::steady_clock::now();
		auto result = wait(queue);
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
		auto after = validate_ring(d, ring);
		// Waiting drains submitted reads. It cannot drain a still-recording upload.
		// A cursor change during the wait means an unobserved CPU reservation exists.
		if (result != VK_SUCCESS || !after || after->used != before->used || !ring.ready(after->used)) {
			ring.disable("queue drain failed or reservation changed during drain");
			continue;
		}
		if (!local_write(ring.slot + offsetof(RingFields, used), 0)) { ring.disable("cursor write failed"); continue; }
		ring.copied_end = 0;
		fprintf(stderr, "monado-ring: reset=%llu used=%llu stall_us=%lld\n", (unsigned long long)++ring.resets,
		        (unsigned long long)before->used, (long long)elapsed);
	}
}

bool sdf_image(const VkImageCreateInfo &info)
{
	return info.imageType == VK_IMAGE_TYPE_2D && info.format == VK_FORMAT_R8_UNORM &&
	       info.extent.depth == 1 && info.mipLevels == 1 && info.arrayLayers == 1 &&
	       info.samples == VK_SAMPLE_COUNT_1_BIT && info.flags == 0 &&
	       ((info.tiling == VK_IMAGE_TILING_LINEAR &&
	         info.usage == (VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT)) ||
	        (info.tiling == VK_IMAGE_TILING_OPTIMAL &&
	         info.usage == (VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
	                        VK_IMAGE_USAGE_SAMPLED_BIT)));
}
// These are the API-visible layouts of the playspace SDF and colored outline.
// Normal compositor/UI meshes and primitive-restart pipelines are excluded.
uint32_t playspace_stride(const VkGraphicsPipelineCreateInfo &info)
{
	auto *assembly = info.pInputAssemblyState;
	auto *vertex = info.pVertexInputState;
	if (!assembly || assembly->topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST ||
	    assembly->primitiveRestartEnable || !vertex || vertex->vertexBindingDescriptionCount != 1 ||
	    vertex->vertexAttributeDescriptionCount != 2) return 0;
	const auto &binding = vertex->pVertexBindingDescriptions[0];
	if (binding.binding != 0 || binding.inputRate != VK_VERTEX_INPUT_RATE_VERTEX) return 0;
	std::array<VkVertexInputAttributeDescription, 2> attributes{};
	for (uint32_t i = 0; i < 2; ++i) {
		const auto &attribute = vertex->pVertexAttributeDescriptions[i];
		if (attribute.location > 1 || attribute.binding != 0) return 0;
		attributes[attribute.location] = attribute;
	}
	if (vertex->pVertexAttributeDescriptions[0].location == vertex->pVertexAttributeDescriptions[1].location ||
	    attributes[0].offset != 0) return 0;
	if (binding.stride == 16 && attributes[0].format == VK_FORMAT_R32G32_SFLOAT &&
	    attributes[1].format == VK_FORMAT_R32G32_SFLOAT && attributes[1].offset == 8) return 16;
	if (binding.stride == 28 && attributes[0].format == VK_FORMAT_R32G32B32_SFLOAT &&
	    attributes[1].format == VK_FORMAT_R32G32B32A32_SFLOAT && attributes[1].offset == 12) return 28;
	return 0;
}

std::optional<IndexUpload> index_upload(VkDeviceSize size, const void *data)
{
	// Inspect bounded uploads, including copies from a mapped staging ring.
	// Require complete 32-bit triangles,
	// zero upper halves throughout, and at least one nondegenerate triangle.
	if (size < 12 || size > 4 * 1024 * 1024 || size % 12) return std::nullopt;
	uint32_t maximum = 0;
	bool triangle = false;
	auto *bytes = static_cast<const unsigned char *>(data);
	for (VkDeviceSize offset = 0; offset < size; offset += 12) {
		std::array<uint32_t, 3> indices;
		memcpy(indices.data(), bytes + offset, 12);
		for (auto index : indices) {
			if (index > std::numeric_limits<uint16_t>::max()) return std::nullopt;
			maximum = std::max(maximum, index);
		}
		triangle |= indices[0] != indices[1] && indices[0] != indices[2] && indices[1] != indices[2];
	}
	if (!triangle) return std::nullopt;
	return IndexUpload{static_cast<uint32_t>(size / 4), maximum};
}

// SteamVR 2.17 direct mode starts its frame timing at 90 Hz and adopts a display mode's refresh rate
// only when that mode matches the desired rate better than an earlier matching mode. A panel with a
// single mode (Rift S: 1440x2560@80) never gets its rate adopted, so the compositor paces 90 Hz frames
// on an 80 Hz panel. Listing a 1 mHz slower alias of the same mode first makes the real mode the better
// match. Both entries carry the same VkDisplayModeKHR, so either selection drives the same mode.
VkResult alias_single_display_mode(PFN_vkGetDisplayModePropertiesKHR get, VkPhysicalDevice physical,
                                   VkDisplayKHR display, uint32_t *count, VkDisplayModePropertiesKHR *out)
{
	uint32_t available = 0;
	VkDisplayModePropertiesKHR mode{};
	if (get(physical, display, &available, nullptr) != VK_SUCCESS || available != 1 ||
	    get(physical, display, &available, &mode) != VK_SUCCESS || available != 1 || mode.parameters.refreshRate <= 1)
		return get(physical, display, count, out);
	std::array<VkDisplayModePropertiesKHR, 2> modes{mode, mode};
	modes[0].parameters.refreshRate -= 1;
	if (!out) {
		*count = modes.size();
		return VK_SUCCESS;
	}
	*count = std::min<uint32_t>(*count, modes.size());
	std::copy_n(modes.begin(), *count, out);
	return *count < modes.size() ? VK_INCOMPLETE : VK_SUCCESS;
}
} // namespace

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkCreateImage(VkDevice handle, const VkImageCreateInfo *info, const VkAllocationCallbacks *allocator, VkImage *out)
{
	auto device = device_for(handle);
	VkImageCreateInfo adjusted = *info;
	bool tracked = device->enabled && sdf_image(adjusted);
	bool empty = adjusted.extent.width == 0 || adjusted.extent.height == 0;
	if (device->sdf_fix && tracked && empty) {
		adjusted.extent.width = std::max(1u, adjusted.extent.width);
		adjusted.extent.height = std::max(1u, adjusted.extent.height);
	}
	auto result = next<PFN_vkCreateImage>(device, "vkCreateImage")(handle, &adjusted, allocator, out);
	if (result == VK_SUCCESS && device->ring_allowed) {
		std::lock_guard guard(mutex);
		device->formats[*out] = adjusted.format;
	}
	if (result == VK_SUCCESS && tracked) {
		std::lock_guard guard(mutex);
		device->images.emplace(*out, Image{adjusted.extent, empty, adjusted.tiling == VK_IMAGE_TILING_LINEAR});
		fprintf(stderr, "monado-roomsetup: R8 %ux%u -> %ux%u, tiling=%u\n", info->extent.width,
		        info->extent.height, adjusted.extent.width, adjusted.extent.height, adjusted.tiling);
	}
	return result;
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkDestroyImage(VkDevice handle, VkImage image, const VkAllocationCallbacks *allocator)
{
	auto device = device_for(handle);
	{
		std::lock_guard guard(mutex);
		device->images.erase(image);
		device->formats.erase(image);
	}
	next<PFN_vkDestroyImage>(device, "vkDestroyImage")(handle, image, allocator);
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkBindImageMemory(VkDevice handle, VkImage image, VkDeviceMemory memory, VkDeviceSize offset)
{
	auto device = device_for(handle);
	auto result = next<PFN_vkBindImageMemory>(device, "vkBindImageMemory")(handle, image, memory, offset);
	if (result == VK_SUCCESS) {
		std::lock_guard guard(mutex);
		auto found = device->images.find(image);
		if (found != device->images.end() && found->second.linear) {
			auto &state = found->second;
			state.memory = memory;
			state.offset = offset;
			VkImageSubresource subresource{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
			next<PFN_vkGetImageSubresourceLayout>(device, "vkGetImageSubresourceLayout")(
			    handle, image, &subresource, &state.layout);
		}
	}
	return result;
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkMapMemory(VkDevice handle, VkDeviceMemory memory, VkDeviceSize offset, VkDeviceSize size,
            VkMemoryMapFlags flags, void **out)
{
	auto device = device_for(handle);
	auto result = next<PFN_vkMapMemory>(device, "vkMapMemory")(handle, memory, offset, size, flags, out);
	if (result != VK_SUCCESS) return result;
	std::lock_guard guard(mutex);
	device->mappings[memory] = Mapping{*out, offset, size};
	identify_ring(device, memory, out);
	for (auto &[buffer, state] : device->buffers) {
		if (state.memory == memory) state.indices.reset();
	}
	for (auto &[image, state] : device->images) {
		if (!device->sdf_fix || !state.linear || state.initialized || state.memory != memory) continue;
		VkDeviceSize start = state.offset + state.layout.offset;
		VkDeviceSize bytes = (state.extent.height - 1) * state.layout.rowPitch + state.extent.width;
		if (start < offset || (size != VK_WHOLE_SIZE && (start - offset > size || bytes > size - (start - offset))))
			continue;
		auto *pixels = static_cast<unsigned char *>(*out) + start - offset;
		// Positive distance saturates to 255. Keep existing strokes on later maps.
		for (uint32_t y = 0; y < state.extent.height; ++y)
			memset(pixels + y * state.layout.rowPitch, 255, state.extent.width);
		state.initialized = true;
		fprintf(stderr, "monado-roomsetup: initialized R8 %ux%u, row pitch=%llu\n",
		        state.extent.width, state.extent.height, static_cast<unsigned long long>(state.layout.rowPitch));
	}
	return result;
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdCopyImage(VkCommandBuffer command, VkImage source, VkImageLayout source_layout,
               VkImage destination, VkImageLayout destination_layout, uint32_t count, const VkImageCopy *regions)
{
	auto device = device_for(command);
	std::vector<VkImageCopy> adjusted;
	{
		std::lock_guard guard(mutex);
		auto src = device->images.find(source);
		auto dst = device->images.find(destination);
		bool source_empty = src != device->images.end() && src->second.empty;
		bool destination_empty = dst != device->images.end() && dst->second.empty;
		if (device->sdf_fix && (source_empty || destination_empty)) {
			for (uint32_t i = 0; i < count; ++i) {
				auto region = regions[i];
				// Copy the initialized sentinel only between two empty images.
				// An empty previous grid must not overwrite a new drawing.
				if (!source_empty || !destination_empty || region.srcOffset.x || region.srcOffset.y ||
				    region.dstOffset.x || region.dstOffset.y) continue;
				region.extent.width = std::min(src->second.extent.width, dst->second.extent.width);
				region.extent.height = std::min(src->second.extent.height, dst->second.extent.height);
				adjusted.push_back(region);
			}
			count = adjusted.size();
			regions = adjusted.data();
		}
	}
	if (count) next<PFN_vkCmdCopyImage>(device, "vkCmdCopyImage")(
	    command, source, source_layout, destination, destination_layout, count, regions);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdCopyBufferToImage(VkCommandBuffer command, VkBuffer source, VkImage destination, VkImageLayout layout,
                       uint32_t count, const VkBufferImageCopy *regions)
{
	auto device = device_for(command);
	{
		std::lock_guard guard(mutex);
		if (device->rings.count(source)) {
			auto format = device->formats.find(destination);
			for (uint32_t i = 0; i < count; ++i) {
				const auto &region = regions[i];
				uint64_t end = UINT64_MAX;
				// Exact last-reservation coverage is established for the unchanged RGBA camera.
				if (format != device->formats.end() && (format->second == VK_FORMAT_R8G8B8A8_UNORM ||
				    format->second == VK_FORMAT_R8G8B8A8_SRGB) && !region.bufferRowLength && !region.bufferImageHeight &&
				    region.imageExtent.depth == 1 && region.imageSubresource.layerCount == 1 &&
				    region.imageExtent.width <= ring_capacity / 4 && region.imageExtent.height &&
				    region.imageExtent.width * 4ULL <= ring_capacity / region.imageExtent.height) {
					uint64_t bytes = uint64_t(region.imageExtent.width) * region.imageExtent.height * 4;
					if (region.bufferOffset <= ring_capacity - bytes) end = region.bufferOffset + bytes;
				}
				ring_copy(device, command, source, end);
			}
		}
	}
	next<PFN_vkCmdCopyBufferToImage>(device, "vkCmdCopyBufferToImage")(command, source, destination, layout, count, regions);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkEndCommandBuffer(VkCommandBuffer command)
{
	auto device = device_for(command);
	auto result = next<PFN_vkEndCommandBuffer>(device, "vkEndCommandBuffer")(command);
	if (result == VK_SUCCESS) {
		std::lock_guard guard(mutex);
		for (auto &[buffer, ring] : device->rings) {
			auto found = ring.commands.find(command);
			if (found != ring.commands.end()) found->second.ended = true;
		}
	}
	return result;
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkQueueSubmit(VkQueue queue, uint32_t count, const VkSubmitInfo *infos, VkFence fence)
{
	auto device = device_for(queue);
	// Binary waits reference signal operations already submitted. A nonzero
	// timeline wait could instead depend on future CPU work, so leave it alone.
	std::vector<VkCommandBuffer> commands;
	bool future_wait = false;
	for (uint32_t i = 0; i < count; ++i) {
		for (auto *extension = static_cast<const VkBaseInStructure *>(infos[i].pNext); extension; extension = extension->pNext) {
			if (extension->sType != VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO) continue;
			const auto &timeline = *reinterpret_cast<const VkTimelineSemaphoreSubmitInfo *>(extension);
			for (uint32_t j = 0; j < timeline.waitSemaphoreValueCount; ++j)
				future_wait |= timeline.pWaitSemaphoreValues[j] != 0;
		}
		for (uint32_t j = 0; j < infos[i].commandBufferCount; ++j) commands.push_back(infos[i].pCommandBuffers[j]);
	}
	auto result = next<PFN_vkQueueSubmit>(device, "vkQueueSubmit")(queue, count, infos, fence);
	if (result == VK_SUCCESS && device->ring_allowed) {
		std::lock_guard guard(mutex);
		if (!future_wait) recover_rings(device, queue, commands);
		else for (auto &[buffer, ring] : device->rings) {
			bool touched = std::any_of(commands.begin(), commands.end(), [&ring](auto cmd) { return ring.commands.count(cmd); });
			if (touched) ring.disable("upload has a nonzero timeline wait");
		}
	}
	return result;
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkCreateBuffer(VkDevice handle, const VkBufferCreateInfo *info, const VkAllocationCallbacks *allocator, VkBuffer *out)
{
	auto device = device_for(handle);
	auto result = next<PFN_vkCreateBuffer>(device, "vkCreateBuffer")(handle, info, allocator, out);
	if (result == VK_SUCCESS && (info->usage & (VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT))) {
		std::lock_guard guard(mutex);
		auto [entry, inserted] = device->buffers.emplace(*out, Buffer{info->size, info->usage});
		if (inserted) entry->second.id = ++device->next_buffer_id;
	}
	return result;
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkDestroyBuffer(VkDevice handle, VkBuffer buffer, const VkAllocationCallbacks *allocator)
{
	auto device = device_for(handle);
	{
		std::lock_guard guard(mutex);
		device->buffers.erase(buffer);
		auto ring = device->rings.find(buffer);
		if (ring != device->rings.end()) ring->second.disable("ring destroyed");
	}
	next<PFN_vkDestroyBuffer>(device, "vkDestroyBuffer")(handle, buffer, allocator);
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkBindBufferMemory(VkDevice handle, VkBuffer buffer, VkDeviceMemory memory, VkDeviceSize offset)
{
	auto device = device_for(handle);
	auto result = next<PFN_vkBindBufferMemory>(device, "vkBindBufferMemory")(handle, buffer, memory, offset);
	if (result == VK_SUCCESS) {
		std::lock_guard guard(mutex);
		auto found = device->buffers.find(buffer);
		if (found != device->buffers.end()) { found->second.memory = memory; found->second.memory_offset = offset; found->second.indices.reset(); }
	}
	return result;
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkBindBufferMemory2(VkDevice handle, uint32_t count, const VkBindBufferMemoryInfo *infos)
{
	auto device = device_for(handle);
	auto result = next<PFN_vkBindBufferMemory2>(device, "vkBindBufferMemory2")(handle, count, infos);
	if (result == VK_SUCCESS) {
		std::lock_guard guard(mutex);
		for (uint32_t i = 0; i < count; ++i) {
			auto found = device->buffers.find(infos[i].buffer);
			if (found != device->buffers.end()) { found->second.memory = infos[i].memory; found->second.memory_offset = infos[i].memoryOffset; found->second.indices.reset(); }
		}
	}
	return result;
}

static bool correct_camera_uniform(void *data, size_t size)
{
	if (size != 400) return false;
	std::array<float, 100> before;
	memcpy(before.data(), data, size);
	bool corrected = roomsetup::correct_camera(data, size);
	static uint64_t last_trace = 0;
	auto now = roomsetup::monotonic_ns();
	if (getenv("MONADO_STEAMVR_BENCH") && now - last_trace > 500000000) {
		last_trace = now;
		fprintf(stderr, "monado-roomview-matrices: ns=%llu corrected=%d view=", (unsigned long long)now, corrected);
		for (int j = 16; j < 32; ++j) fprintf(stderr, " %.9g", before[j]);
		fprintf(stderr, " camera_before=");
		for (int j = 48; j < 64; ++j) fprintf(stderr, " %.9g", before[j]);
		std::array<float, 100> after;
		memcpy(after.data(), data, size);
		fprintf(stderr, " camera_after=");
		for (int j = 48; j < 64; ++j) fprintf(stderr, " %.9g", after[j]);
		fprintf(stderr, "\n");
	}
	return corrected;
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdUpdateBuffer(VkCommandBuffer command, VkBuffer buffer, VkDeviceSize offset, VkDeviceSize size, const void *data)
{
	auto device = device_for(command);
	std::array<unsigned char, 400> camera;
	{
		std::lock_guard guard(mutex);
		auto found = device->buffers.find(buffer);
		if (found != device->buffers.end() && (found->second.usage & VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT) &&
		    size == camera.size() && offset <= found->second.size && size <= found->second.size - offset) {
			memcpy(camera.data(), data, camera.size());
			if (correct_camera_uniform(camera.data(), camera.size())) {
				data = camera.data();
				++found->second.fixes;
			}
		}
		if (found != device->buffers.end() && (found->second.usage & VK_BUFFER_USAGE_INDEX_BUFFER_BIT)) {
			// A validated prefix is sufficient when the draw stays within it.
			// Updates elsewhere invalidate the inferred format.
			found->second.indices = offset == 0 && size <= found->second.size ? index_upload(size, data) : std::nullopt;
		}
	}
	next<PFN_vkCmdUpdateBuffer>(device, "vkCmdUpdateBuffer")(command, buffer, offset, size, data);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkUnmapMemory(VkDevice handle, VkDeviceMemory memory)
{
	auto device = device_for(handle);
	{
		std::lock_guard guard(mutex);
		device->mappings.erase(memory);
		for (auto &[buffer, ring] : device->rings)
			if (ring.identity.memory == memory) ring.disable("ring memory released");
	}
	next<PFN_vkUnmapMemory>(device, "vkUnmapMemory")(handle, memory);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkFreeMemory(VkDevice handle, VkDeviceMemory memory, const VkAllocationCallbacks *allocator)
{
	auto device = device_for(handle);
	{
		std::lock_guard guard(mutex);
		device->mappings.erase(memory);
		for (auto &[buffer, ring] : device->rings)
			if (ring.identity.memory == memory) ring.disable("ring memory released");
	}
	next<PFN_vkFreeMemory>(device, "vkFreeMemory")(handle, memory, allocator);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdCopyBuffer(VkCommandBuffer command, VkBuffer source, VkBuffer destination, uint32_t count, const VkBufferCopy *regions)
{
	auto device = device_for(command);
	{
		std::lock_guard guard(mutex);
		for (uint32_t i = 0; i < count; ++i) {
			const auto &region = regions[i];
			uint64_t end = region.srcOffset <= ring_capacity && region.size <= ring_capacity - region.srcOffset
			                   ? region.srcOffset + region.size : UINT64_MAX;
			ring_copy(device, command, source, end);
		}
		auto dst = device->buffers.find(destination);
		if (dst != device->buffers.end()) {
			dst->second.indices.reset();
			if ((dst->second.usage & VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT) && count == 1 && regions[0].size == 400 &&
			    regions[0].dstOffset <= dst->second.size && 400 <= dst->second.size - regions[0].dstOffset) {
				auto src=device->buffers.find(source);
				if (src != device->buffers.end() && regions[0].srcOffset <= src->second.size &&
				    400 <= src->second.size - regions[0].srcOffset &&
				    src->second.memory_offset <= UINT64_MAX - regions[0].srcOffset) {
					auto map=device->mappings.find(src->second.memory);
					auto start=src->second.memory_offset+regions[0].srcOffset;
					if (map!=device->mappings.end() && start>=map->second.offset &&
					    (map->second.size==VK_WHOLE_SIZE || (start-map->second.offset<=map->second.size &&
					     400<=map->second.size-(start-map->second.offset)))) {
						auto *data=static_cast<unsigned char *>(map->second.data)+start-map->second.offset;
						bool corrected = correct_camera_uniform(data, 400);
						if (corrected) {
							++dst->second.fixes;
							if ((dst->second.fixes & (dst->second.fixes-1))==0)
								fprintf(stderr,"monado-roomview: rotation-only hits=%llu\n",
								        (unsigned long long)dst->second.fixes);
						}
					}
				}
			}

			auto src = device->buffers.find(source);
			if ((dst->second.usage & VK_BUFFER_USAGE_INDEX_BUFFER_BIT) && src != device->buffers.end() &&
			    count == 1 && regions[0].dstOffset == 0 && regions[0].size <= dst->second.size &&
			    regions[0].srcOffset <= src->second.size && regions[0].size <= src->second.size - regions[0].srcOffset) {
				const auto &buffer = src->second;
				auto mapped = device->mappings.find(buffer.memory);
				if (mapped != device->mappings.end() && buffer.memory_offset <= UINT64_MAX - regions[0].srcOffset) {
					const auto &map = mapped->second;
					VkDeviceSize start = buffer.memory_offset + regions[0].srcOffset;
					if (start >= map.offset && (map.size == VK_WHOLE_SIZE ||
					    (start - map.offset <= map.size && regions[0].size <= map.size - (start - map.offset)))) {
						auto *data = static_cast<const unsigned char *>(map.data) + start - map.offset;
						dst->second.indices = index_upload(regions[0].size, data);
					}
				}
			}
		}
	}
	next<PFN_vkCmdCopyBuffer>(device, "vkCmdCopyBuffer")(command, source, destination, count, regions);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdFillBuffer(VkCommandBuffer command, VkBuffer buffer, VkDeviceSize offset, VkDeviceSize size, uint32_t data)
{
	auto device = device_for(command);
	{
		std::lock_guard guard(mutex);
		auto found = device->buffers.find(buffer);
		if (found != device->buffers.end()) found->second.indices.reset();
	}
	next<PFN_vkCmdFillBuffer>(device, "vkCmdFillBuffer")(command, buffer, offset, size, data);
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkCreateGraphicsPipelines(VkDevice handle, VkPipelineCache cache, uint32_t count,
                          const VkGraphicsPipelineCreateInfo *infos, const VkAllocationCallbacks *allocator, VkPipeline *out)
{
	auto device = device_for(handle);
	auto result = next<PFN_vkCreateGraphicsPipelines>(device, "vkCreateGraphicsPipelines")(handle, cache, count, infos, allocator, out);
	std::lock_guard guard(mutex);
	for (uint32_t i = 0; i < count; ++i) {
		if (out[i] != VK_NULL_HANDLE) device->strides[out[i]] = playspace_stride(infos[i]);
	}
	return result;
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkDestroyPipeline(VkDevice handle, VkPipeline pipeline, const VkAllocationCallbacks *allocator)
{
	auto device = device_for(handle);
	{ std::lock_guard guard(mutex); device->strides.erase(pipeline); }
	next<PFN_vkDestroyPipeline>(device, "vkDestroyPipeline")(handle, pipeline, allocator);
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkBeginCommandBuffer(VkCommandBuffer command, const VkCommandBufferBeginInfo *info)
{
	auto device = device_for(command);
	auto result = next<PFN_vkBeginCommandBuffer>(device, "vkBeginCommandBuffer")(command, info);
	if (result == VK_SUCCESS) {
		std::lock_guard guard(mutex);
		device->draws[command] = {};
		for (auto &[buffer, ring] : device->rings) {
			auto entry = ring.commands.find(command);
			if (entry != ring.commands.end() && !entry->second.submitted) ring.disable("unsubmitted upload discarded");
			ring.commands.erase(command);
		}
	}
	return result;
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkFreeCommandBuffers(VkDevice handle, VkCommandPool pool, uint32_t count, const VkCommandBuffer *commands)
{
	auto device = device_for(handle);
	{
		std::lock_guard guard(mutex);
		for (uint32_t i = 0; i < count; ++i) {
			device->draws.erase(commands[i]);
			for (auto &[buffer, ring] : device->rings) {
				auto found = ring.commands.find(commands[i]);
				if (found != ring.commands.end() && !found->second.submitted) ring.disable("unsubmitted upload freed");
				ring.commands.erase(commands[i]);
			}
		}
	}
	next<PFN_vkFreeCommandBuffers>(device, "vkFreeCommandBuffers")(handle, pool, count, commands);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdBindPipeline(VkCommandBuffer command, VkPipelineBindPoint point, VkPipeline pipeline)
{
	auto device = device_for(command);
	if (point == VK_PIPELINE_BIND_POINT_GRAPHICS) {
		std::lock_guard guard(mutex);
		device->draws[command].pipeline = pipeline;
	}
	next<PFN_vkCmdBindPipeline>(device, "vkCmdBindPipeline")(command, point, pipeline);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdBindVertexBuffers(VkCommandBuffer command, uint32_t first, uint32_t count, const VkBuffer *buffers, const VkDeviceSize *offsets)
{
	auto device = device_for(command);
	if (first == 0 && count) {
		std::lock_guard guard(mutex);
		auto &draw = device->draws[command];
		draw.vertex = buffers[0]; draw.vertex_offset = offsets[0];
	}
	next<PFN_vkCmdBindVertexBuffers>(device, "vkCmdBindVertexBuffers")(command, first, count, buffers, offsets);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdBindIndexBuffer(VkCommandBuffer command, VkBuffer buffer, VkDeviceSize offset, VkIndexType type)
{
	auto device = device_for(command);
	{
		std::lock_guard guard(mutex);
		auto &draw = device->draws[command];
		draw.index = buffer; draw.index_offset = offset; draw.index_type = type;
	}
	next<PFN_vkCmdBindIndexBuffer>(device, "vkCmdBindIndexBuffer")(command, buffer, offset, type);
}

extern "C" VKAPI_ATTR void VKAPI_CALL
vkCmdDrawIndexed(VkCommandBuffer command, uint32_t count, uint32_t instances, uint32_t first, int32_t base, uint32_t first_instance)
{
	auto device = device_for(command);
	Draw draw;
	bool repair = false;
	{
		std::lock_guard guard(mutex);
		draw = device->draws[command];
		auto index = device->buffers.find(draw.index);
		auto vertex = device->buffers.find(draw.vertex);
		auto pipeline = device->strides.find(draw.pipeline);
		if (device->index_fix && !device->images.empty() && index != device->buffers.end() && index->second.indices &&
		    vertex != device->buffers.end() && pipeline != device->strides.end() && pipeline->second &&
		    draw.index_type == VK_INDEX_TYPE_UINT16 && draw.index_offset == 0 && draw.vertex_offset == 0 &&
		    first == 0 && base == 0 && count && count % 3 == 0) {
			const auto &upload = *index->second.indices;
			repair = count <= upload.count && vertex->second.size >= (upload.maximum + 1ULL) * pipeline->second;
			if (repair) {
				auto &buffer = index->second;
				++buffer.fixes;
				if ((buffer.fixes & (buffer.fixes - 1)) == 0)
					fprintf(stderr, "monado-roomsetup: index-fix buffer=%llu vertex=%llu hits=%llu stride=%u index_bytes=%llu vertex_bytes=%llu usage=%u count=%u upload_count=%u maximum=%u\n",
					        (unsigned long long)buffer.id, (unsigned long long)vertex->second.id,
					        (unsigned long long)buffer.fixes, pipeline->second, (unsigned long long)buffer.size,
					        (unsigned long long)vertex->second.size, buffer.usage, count, upload.count, upload.maximum);
			}

		}
	}
	auto bind = next<PFN_vkCmdBindIndexBuffer>(device, "vkCmdBindIndexBuffer");
	if (repair) bind(command, draw.index, draw.index_offset, VK_INDEX_TYPE_UINT32);
	next<PFN_vkCmdDrawIndexed>(device, "vkCmdDrawIndexed")(command, count, instances, first, base, first_instance);
	// Keep the caller's binding valid for subsequent unrelated draws.
	if (repair) bind(command, draw.index, draw.index_offset, draw.index_type);
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkWaitForPresentKHR(VkDevice handle, VkSwapchainKHR swapchain, uint64_t present_id, uint64_t timeout_ns)
{
	auto device = device_for(handle);
	auto wait = next<PFN_vkWaitForPresentKHR>(device, "vkWaitForPresentKHR");
	auto start = std::chrono::steady_clock::now();
	auto result = wait(handle, swapchain, present_id, timeout_ns);
	if (result != VK_SUCCESS) {
		static std::atomic<uint64_t> failures{0};
		auto count = ++failures;
		// Bound logging during a fault while retaining its frequency and exact VkResult.
		if (count <= 8 || (count & (count - 1)) == 0) {
			auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
			    std::chrono::steady_clock::now() - start).count();
			fprintf(stderr, "monado-present: wait result=%d present_id=%llu timeout_ns=%llu elapsed_us=%lld failures=%llu\n",
			        result, (unsigned long long)present_id, (unsigned long long)timeout_ns,
			        (long long)elapsed, (unsigned long long)count);
		}
	}
	return result;
}

extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDevice(VkDevice handle, const VkAllocationCallbacks *allocator)
{
	auto device = device_for(handle);
	auto destroy = next<PFN_vkDestroyDevice>(device, "vkDestroyDevice");
	{
		std::lock_guard guard(mutex);
		devices.erase(key(handle));
	}
	destroy(handle, allocator);
}

extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyInstance(VkInstance handle, const VkAllocationCallbacks *allocator)
{
	Instance instance;
	{
		std::lock_guard guard(mutex);
		instance = instances.at(key(handle));
		instances.erase(key(handle));
	}
	reinterpret_cast<PFN_vkDestroyInstance>(instance.gipa(handle, "vkDestroyInstance"))(handle, allocator);
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkGetDisplayModePropertiesKHR(VkPhysicalDevice physical, VkDisplayKHR display, uint32_t *count,
                              VkDisplayModePropertiesKHR *out)
{
	Instance instance;
	{
		std::lock_guard guard(mutex);
		instance = instances.at(key(physical));
	}
	auto get = reinterpret_cast<PFN_vkGetDisplayModePropertiesKHR>(
	    instance.gipa(instance.handle, "vkGetDisplayModePropertiesKHR"));
	return alias_single_display_mode(get, physical, display, count, out);
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkCreateInstance(const VkInstanceCreateInfo *info, const VkAllocationCallbacks *allocator, VkInstance *out)
{
	auto *chain = reinterpret_cast<VkLayerInstanceCreateInfo *>(const_cast<void *>(info->pNext));
	while (chain && (chain->sType != VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO || chain->function != VK_LAYER_LINK_INFO))
		chain = reinterpret_cast<VkLayerInstanceCreateInfo *>(const_cast<void *>(chain->pNext));
	if (!chain) return VK_ERROR_INITIALIZATION_FAILED;
	auto gipa = chain->u.pLayerInfo->pfnNextGetInstanceProcAddr;
	auto create = reinterpret_cast<PFN_vkCreateInstance>(gipa(VK_NULL_HANDLE, "vkCreateInstance"));
	chain->u.pLayerInfo = chain->u.pLayerInfo->pNext;
	auto result = create(info, allocator, out);
	if (result == VK_SUCCESS) {
		bool enabled = monado_compositor();
		std::lock_guard guard(mutex);
		instances.emplace(key(*out), Instance{*out, gipa, enabled});
		if (enabled) fprintf(stderr, "monado-roomsetup: Vulkan SDF compatibility enabled\n");
	}
	return result;
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkCreateDevice(VkPhysicalDevice physical, const VkDeviceCreateInfo *info, const VkAllocationCallbacks *allocator, VkDevice *out)
{
	Instance instance;
	{
		std::lock_guard guard(mutex);
		instance = instances.at(key(physical));
	}
	auto *chain = reinterpret_cast<VkLayerDeviceCreateInfo *>(const_cast<void *>(info->pNext));
	while (chain && (chain->sType != VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO || chain->function != VK_LAYER_LINK_INFO))
		chain = reinterpret_cast<VkLayerDeviceCreateInfo *>(const_cast<void *>(chain->pNext));
	if (!chain) return VK_ERROR_INITIALIZATION_FAILED;
	auto gipa = chain->u.pLayerInfo->pfnNextGetInstanceProcAddr;
	auto gdpa = chain->u.pLayerInfo->pfnNextGetDeviceProcAddr;
	auto create = reinterpret_cast<PFN_vkCreateDevice>(gipa(instance.handle, "vkCreateDevice"));
	chain->u.pLayerInfo = chain->u.pLayerInfo->pNext;
	auto result = create(physical, info, allocator, out);
	if (result == VK_SUCCESS) {
		std::lock_guard guard(mutex);
		auto device = std::make_shared<Device>(Device{gdpa, *out, instance.enabled, {}});
		device->index_fix = !getenv("MONADO_DISABLE_ROOMSETUP_INDEX_FIX");
		device->sdf_fix = !getenv("MONADO_DISABLE_ROOMSETUP_SDF_FIX");
		device->ring_allowed = instance.enabled && !getenv("MONADO_DISABLE_TRANSFER_RING_RECOVERY") && supported_ring_build();
		static std::atomic_flag reported = ATOMIC_FLAG_INIT;
		if (instance.enabled && !device->ring_allowed && !reported.test_and_set())
			fprintf(stderr, "monado-ring: disabled: unknown SteamVR build or recovery disabled\n");
		devices.emplace(key(*out), device);
	}
	return result;
}

extern "C" VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vkGetDeviceProcAddr(VkDevice handle, const char *name)
{
	auto device = device_for(handle);
#define MATCH(n) if (strcmp(name, #n) == 0) return reinterpret_cast<PFN_vkVoidFunction>(n)
	MATCH(vkGetDeviceProcAddr); MATCH(vkDestroyDevice);
	if (device->enabled) {
		if (strcmp(name, "vkWaitForPresentKHR") == 0 && device->gdpa(handle, name))
			return reinterpret_cast<PFN_vkVoidFunction>(vkWaitForPresentKHR);
		MATCH(vkCreateImage); MATCH(vkDestroyImage); MATCH(vkBindImageMemory); MATCH(vkMapMemory); MATCH(vkCmdCopyImage);
		MATCH(vkQueueSubmit); MATCH(vkEndCommandBuffer); MATCH(vkCmdCopyBufferToImage);
		MATCH(vkCreateBuffer); MATCH(vkDestroyBuffer); MATCH(vkBindBufferMemory); MATCH(vkBindBufferMemory2);
		MATCH(vkCmdUpdateBuffer); MATCH(vkCmdCopyBuffer); MATCH(vkCmdFillBuffer); MATCH(vkUnmapMemory); MATCH(vkFreeMemory);
		MATCH(vkCreateGraphicsPipelines); MATCH(vkDestroyPipeline); MATCH(vkBeginCommandBuffer); MATCH(vkFreeCommandBuffers);
		MATCH(vkCmdBindPipeline); MATCH(vkCmdBindVertexBuffers); MATCH(vkCmdBindIndexBuffer); MATCH(vkCmdDrawIndexed);
	}
	return device->gdpa(handle, name);
}

extern "C" VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vkGetInstanceProcAddr(VkInstance handle, const char *name)
{
	MATCH(vkGetInstanceProcAddr); MATCH(vkCreateInstance); MATCH(vkCreateDevice); MATCH(vkDestroyInstance);
	MATCH(vkGetDeviceProcAddr); MATCH(vkDestroyDevice);
	if (!handle) return nullptr;
	Instance instance;
	{
		std::lock_guard guard(mutex);
		instance = instances.at(key(handle));
	}
	if (instance.enabled) {
		if (strcmp(name, "vkWaitForPresentKHR") == 0 && instance.gipa(handle, name))
			return reinterpret_cast<PFN_vkVoidFunction>(vkWaitForPresentKHR);
		if (strcmp(name, "vkGetDisplayModePropertiesKHR") == 0 && instance.gipa(handle, name))
			return reinterpret_cast<PFN_vkVoidFunction>(vkGetDisplayModePropertiesKHR);
		MATCH(vkCreateImage); MATCH(vkDestroyImage); MATCH(vkBindImageMemory); MATCH(vkMapMemory); MATCH(vkCmdCopyImage);
		MATCH(vkQueueSubmit); MATCH(vkEndCommandBuffer); MATCH(vkCmdCopyBufferToImage);
		MATCH(vkCreateBuffer); MATCH(vkDestroyBuffer); MATCH(vkBindBufferMemory); MATCH(vkBindBufferMemory2);
		MATCH(vkCmdUpdateBuffer); MATCH(vkCmdCopyBuffer); MATCH(vkCmdFillBuffer); MATCH(vkUnmapMemory); MATCH(vkFreeMemory);
		MATCH(vkCreateGraphicsPipelines); MATCH(vkDestroyPipeline); MATCH(vkBeginCommandBuffer); MATCH(vkFreeCommandBuffers);
		MATCH(vkCmdBindPipeline); MATCH(vkCmdBindVertexBuffers); MATCH(vkCmdBindIndexBuffer); MATCH(vkCmdDrawIndexed);
	}
	return instance.gipa(handle, name);
#undef MATCH
}

extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkNegotiateLoaderLayerInterfaceVersion(VkNegotiateLayerInterface *info)
{
	info->loaderLayerInterfaceVersion = std::min(info->loaderLayerInterfaceVersion, 2u);
	info->pfnGetInstanceProcAddr = vkGetInstanceProcAddr;
	info->pfnGetDeviceProcAddr = vkGetDeviceProcAddr;
	info->pfnGetPhysicalDeviceProcAddr = nullptr;
	return VK_SUCCESS;
}
