// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "../src/xrt/state_trackers/steamvr_drv/ovrd_roomsetup_vulkan.cpp"
#include <array>

namespace {
VkImageCreateInfo created{};
std::array<unsigned char, 128> storage;
uint32_t copy_count;
VkImageCopy copied{};
std::vector<VkIndexType> bindings;
VkIndexType drawn_type;
uint32_t idle_calls;
VkResult idle_result;
uint64_t *idle_cursor;
VkResult submit_result = VK_SUCCESS;
VkResult VKAPI_CALL fake_submit(VkQueue, uint32_t, const VkSubmitInfo *, VkFence) { return submit_result; }
VkResult VKAPI_CALL fake_idle(VkQueue) {
 ++idle_calls;
 if (idle_cursor) *idle_cursor += 4;
 return idle_result;
}
void VKAPI_CALL fake_bind(VkCommandBuffer, VkBuffer, VkDeviceSize, VkIndexType type) { bindings.push_back(type); }
void VKAPI_CALL fake_draw(VkCommandBuffer, uint32_t, uint32_t, uint32_t, int32_t, uint32_t)
{
	drawn_type = bindings.back();
}
void VKAPI_CALL fake_update(VkCommandBuffer, VkBuffer, VkDeviceSize, VkDeviceSize, const void *) {}
void VKAPI_CALL fake_buffer_copy(VkCommandBuffer, VkBuffer, VkBuffer, uint32_t, const VkBufferCopy *) {}
VkResult VKAPI_CALL fake_begin(VkCommandBuffer, const VkCommandBufferBeginInfo *) { return VK_SUCCESS; }
VkResult VKAPI_CALL fake_create(VkDevice, const VkImageCreateInfo *info, const VkAllocationCallbacks *, VkImage *out)
{
	created = *info;
	*out = reinterpret_cast<VkImage>(uintptr_t{42});
	return VK_SUCCESS;
}
VkResult VKAPI_CALL fake_map(VkDevice, VkDeviceMemory, VkDeviceSize offset, VkDeviceSize, VkMemoryMapFlags, void **out)
{
	*out = storage.data() + offset;
	return VK_SUCCESS;
}
void VKAPI_CALL fake_copy(VkCommandBuffer, VkImage, VkImageLayout, VkImage, VkImageLayout, uint32_t count, const VkImageCopy *regions)
{
	copy_count += count;
	copied = regions[0];
}
VkResult VKAPI_CALL fake_create_buffer(VkDevice, const VkBufferCreateInfo *, const VkAllocationCallbacks *, VkBuffer *out)
{
	*out = reinterpret_cast<VkBuffer>(uintptr_t{43});
	return VK_SUCCESS;
}
PFN_vkVoidFunction VKAPI_CALL fake_proc(VkDevice, const char *name)
{
	if (!strcmp(name, "vkQueueSubmit")) return reinterpret_cast<PFN_vkVoidFunction>(fake_submit);
	if (!strcmp(name, "vkQueueWaitIdle")) return reinterpret_cast<PFN_vkVoidFunction>(fake_idle);
	if (!strcmp(name, "vkCmdBindIndexBuffer")) return reinterpret_cast<PFN_vkVoidFunction>(fake_bind);
	if (!strcmp(name, "vkCmdDrawIndexed")) return reinterpret_cast<PFN_vkVoidFunction>(fake_draw);
	if (!strcmp(name, "vkCmdUpdateBuffer")) return reinterpret_cast<PFN_vkVoidFunction>(fake_update);
	if (!strcmp(name, "vkCmdCopyBuffer")) return reinterpret_cast<PFN_vkVoidFunction>(fake_buffer_copy);
	if (!strcmp(name, "vkBeginCommandBuffer")) return reinterpret_cast<PFN_vkVoidFunction>(fake_begin);
	if (!strcmp(name, "vkCreateBuffer")) return reinterpret_cast<PFN_vkVoidFunction>(fake_create_buffer);
	if (!strcmp(name, "vkCreateImage")) return reinterpret_cast<PFN_vkVoidFunction>(fake_create);
	if (!strcmp(name, "vkMapMemory")) return reinterpret_cast<PFN_vkVoidFunction>(fake_map);
	if (!strcmp(name, "vkCmdCopyImage")) return reinterpret_cast<PFN_vkVoidFunction>(fake_copy);
	return nullptr;
}
struct Fixture
{
	void *dispatch = this;
	VkDevice handle = reinterpret_cast<VkDevice>(&dispatch);
	std::shared_ptr<Device> state = std::make_shared<Device>(Device{fake_proc, handle, true, {}});
	Fixture()
	{
		devices.emplace(dispatch, state);
		storage.fill(0x5a);
		copy_count = 0;
		bindings.clear();
	}
	~Fixture() { devices.erase(dispatch); }
};
}

TEST_CASE("SteamVR SDF empty image gets valid storage without changing other image types")
{
	Fixture f;
	VkImageCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	info.imageType = VK_IMAGE_TYPE_2D;
	info.format = VK_FORMAT_R8_UNORM;
	info.extent = {0, 0, 1};
	info.mipLevels = info.arrayLayers = 1;
	info.samples = VK_SAMPLE_COUNT_1_BIT;
	info.tiling = VK_IMAGE_TILING_LINEAR;
	info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	VkImage image;
	REQUIRE(vkCreateImage(f.handle, &info, nullptr, &image) == VK_SUCCESS);
	CHECK(created.extent.width == 1);
	CHECK(created.extent.height == 1);
	CHECK(info.extent.width == 0);
	CHECK(f.state->images.at(image).empty);
	info.format = VK_FORMAT_R8G8B8A8_UNORM;
	REQUIRE(vkCreateImage(f.handle, &info, nullptr, &image) == VK_SUCCESS);
	CHECK(created.extent.width == 0);
	CHECK(created.extent.height == 0);
}

TEST_CASE("SteamVR SDF initializes sparse gaps using row pitch and keeps previous strokes")
{
	Fixture f;
	auto memory = reinterpret_cast<VkDeviceMemory>(uintptr_t{7});
	auto image = reinterpret_cast<VkImage>(uintptr_t{42});
	Image state{{5, 3, 1}, false, true};
	state.memory = memory;
	state.offset = 8;
	state.layout.offset = 4;
	state.layout.rowPitch = 16;
	f.state->images.emplace(image, state);
	void *mapped;
	REQUIRE(vkMapMemory(f.handle, memory, 8, 41, 0, &mapped) == VK_SUCCESS);
	for (size_t i = 0; i < storage.size(); ++i) {
		bool pixel = i >= 12 && i <= 48 && (i - 12) % 16 < 5;
		CHECK(storage[i] == (pixel ? 255 : 0x5a));
	}
	storage[12] = 91;
	REQUIRE(vkMapMemory(f.handle, memory, 8, VK_WHOLE_SIZE, 0, &mapped) == VK_SUCCESS);
	CHECK(storage[12] == 91);
}

TEST_CASE("SteamVR SDF leaves partial mappings and unrelated allocations alone")
{
	Fixture f;
	auto memory = reinterpret_cast<VkDeviceMemory>(uintptr_t{7});
	Image state{{5, 3, 1}, false, true};
	state.memory = memory;
	state.layout.rowPitch = 16;
	f.state->images.emplace(reinterpret_cast<VkImage>(uintptr_t{42}), state);
	void *mapped;
	REQUIRE(vkMapMemory(f.handle, memory, 0, 36, 0, &mapped) == VK_SUCCESS);
	CHECK_FALSE(f.state->images.begin()->second.initialized);
	CHECK(std::all_of(storage.begin(), storage.end(), [](auto value) { return value == 0x5a; }));
	REQUIRE(vkMapMemory(f.handle, reinterpret_cast<VkDeviceMemory>(uintptr_t{8}), 0, VK_WHOLE_SIZE, 0, &mapped) == VK_SUCCESS);
	CHECK(std::all_of(storage.begin(), storage.end(), [](auto value) { return value == 0x5a; }));
}

TEST_CASE("SteamVR copies an empty sentinel but does not copy empty history into a drawing")
{
	Fixture f;
	auto src = reinterpret_cast<VkImage>(uintptr_t{1});
	auto dst = reinterpret_cast<VkImage>(uintptr_t{2});
	f.state->images.emplace(src, Image{{1, 1, 1}, true, true});
	f.state->images.emplace(dst, Image{{1, 1, 1}, true, false});
	VkImageCopy region{};
	region.extent = {0, 0, 1};
	vkCmdCopyImage(reinterpret_cast<VkCommandBuffer>(f.handle), src, VK_IMAGE_LAYOUT_GENERAL, dst,
	               VK_IMAGE_LAYOUT_GENERAL, 1, &region);
	CHECK(copy_count == 1);
	CHECK(copied.extent.width == 1);
	CHECK(copied.extent.height == 1);
	f.state->images.at(dst).empty = false;
	vkCmdCopyImage(reinterpret_cast<VkCommandBuffer>(f.handle), src, VK_IMAGE_LAYOUT_GENERAL, dst,
	               VK_IMAGE_LAYOUT_GENERAL, 1, &region);
	CHECK(copy_count == 1);
}

TEST_CASE("SteamVR playspace index correction distinguishes 32-bit uploads from real 16-bit data")
{
	Fixture f;
	auto command = reinterpret_cast<VkCommandBuffer>(f.handle);
	auto index = reinterpret_cast<VkBuffer>(uintptr_t{20});
	auto vertex = reinterpret_cast<VkBuffer>(uintptr_t{21});
	auto pipeline = reinterpret_cast<VkPipeline>(uintptr_t{22});
	std::vector<uint32_t> mesh;
	for (uint32_t row = 0; row < 7; ++row) {
		for (uint32_t column = 0; column < 7; ++column) {
			uint32_t corner = row * 8 + column;
			mesh.insert(mesh.end(), {corner, corner + 1, corner + 8, corner + 1, corner + 9, corner + 8});
		}
	}
	f.state->buffers.emplace(index, Buffer{mesh.size() * 4, VK_BUFFER_USAGE_INDEX_BUFFER_BIT});
	f.state->buffers.emplace(vertex, Buffer{64 * 16, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT});
	f.state->strides[pipeline] = 16;
	f.state->images.emplace(reinterpret_cast<VkImage>(uintptr_t{23}), Image{{512, 512, 1}, false, false});
	f.state->draws[command] = Draw{pipeline, vertex, index};
	vkCmdBindIndexBuffer(command, index, 0, VK_INDEX_TYPE_UINT16);
	vkCmdUpdateBuffer(command, index, 0, mesh.size() * 4, mesh.data());
	REQUIRE(f.state->buffers.at(index).indices.has_value());
	vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
	CHECK(drawn_type == VK_INDEX_TYPE_UINT32);
	CHECK(bindings.back() == VK_INDEX_TYPE_UINT16);
	CHECK(f.state->draws.at(command).index_type == VK_INDEX_TYPE_UINT16);

	SECTION("index A/B switch leaves SDF resources and caller binding intact") {
		f.state->index_fix = false;
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
		CHECK_FALSE(f.state->images.empty());
		CHECK(bindings.back() == VK_INDEX_TYPE_UINT16);
	}
	SECTION("SDF A/B switch does not disable index repair") {
		f.state->sdf_fix = false;
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT32);
	}
	SECTION("outline draws can use a subset of a reserved 32-bit index buffer") {
		f.state->strides[pipeline] = 28;
		f.state->buffers.at(vertex).size = 64 * 28;
		vkCmdDrawIndexed(command, 42, 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT32);
	}
	SECTION("reserved vertex storage can exceed the referenced vertex range") {
		f.state->buffers.at(vertex).size = 80 * 16;
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT32);
	}
	SECTION("genuine 16-bit mesh uploads remain unchanged") {
		std::vector<uint16_t> compact(mesh.begin(), mesh.end());
		f.state->buffers.at(index).size = compact.size() * 2;
		vkCmdUpdateBuffer(command, index, 0, compact.size() * 2, compact.data());
		vkCmdDrawIndexed(command, compact.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
	}
	SECTION("nonmatching pipeline layouts remain unchanged") {
		f.state->strides[pipeline] = 0;
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
	}
	SECTION("out-of-bounds meshes remain unchanged") {
		f.state->buffers.at(vertex).size = 63 * 16;
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
	}
	SECTION("partial uploads invalidate the inferred index format") {
		vkCmdUpdateBuffer(command, index, 4, 4, mesh.data());
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
	}
	SECTION("an uploaded prefix covers draws without requiring unused index capacity") {
		f.state->buffers.at(index).size += 1024;
		vkCmdUpdateBuffer(command, index, 0, mesh.size() * 4, mesh.data());
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT32);
		vkCmdDrawIndexed(command, mesh.size() + 3, 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
	}
	SECTION("staging copies validate only the copied index range and honor memory offsets") {
		auto staging = reinterpret_cast<VkBuffer>(uintptr_t{30});
		auto memory = reinterpret_cast<VkDeviceMemory>(uintptr_t{31});
		std::vector<unsigned char> bytes(64 + mesh.size() * 4, 0x5a);
		memcpy(bytes.data() + 64, mesh.data(), mesh.size() * 4);
		Buffer source{mesh.size() * 4 + 16, VK_BUFFER_USAGE_TRANSFER_SRC_BIT};
		source.memory = memory; source.memory_offset = 48;
		f.state->buffers.emplace(staging, source);
		f.state->mappings[memory] = Mapping{bytes.data(), 0, bytes.size()};
		VkBufferCopy region{16, 0, mesh.size() * 4};
		vkCmdCopyBuffer(command, staging, index, 1, &region);
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT32);
		f.state->mappings[memory].size = bytes.size() - 1;
		vkCmdCopyBuffer(command, staging, index, 1, &region);
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
	}
	SECTION("unobserved copies invalidate the inferred index format") {
		VkBufferCopy transfer{0, 0, 4};
		vkCmdCopyBuffer(command, vertex, index, 1, &transfer);
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
	}
	SECTION("starting another command buffer recording clears stale bindings") {
		VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
		REQUIRE(vkBeginCommandBuffer(command, &begin) == VK_SUCCESS);
		vkCmdDrawIndexed(command, mesh.size(), 1, 0, 0, 0);
		CHECK(drawn_type == VK_INDEX_TYPE_UINT16);
	}
}

TEST_CASE("SteamVR index inference rejects degenerate and wide meshes")
{
	std::array<uint32_t, 3> indices{0, 1, 2};
	REQUIRE(index_upload(sizeof(indices), indices.data()).has_value());
	indices = {0, 0, 0};
	CHECK_FALSE(index_upload(sizeof(indices), indices.data()).has_value());
	indices = {0, 1, 65536};
	CHECK_FALSE(index_upload(sizeof(indices), indices.data()).has_value());
	CHECK_FALSE(index_upload(8, indices.data()).has_value());
}

TEST_CASE("SteamVR index workaround accepts only the measured playspace vertex layouts")
{
	VkVertexInputBindingDescription binding{0, 16, VK_VERTEX_INPUT_RATE_VERTEX};
	std::array<VkVertexInputAttributeDescription, 2> attributes{{
	    {0, 0, VK_FORMAT_R32G32_SFLOAT, 0}, {1, 0, VK_FORMAT_R32G32_SFLOAT, 8}}};
	VkPipelineVertexInputStateCreateInfo vertex{};
	vertex.vertexBindingDescriptionCount = 1; vertex.pVertexBindingDescriptions = &binding;
	vertex.vertexAttributeDescriptionCount = 2; vertex.pVertexAttributeDescriptions = attributes.data();
	VkPipelineInputAssemblyStateCreateInfo assembly{};
	assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	VkGraphicsPipelineCreateInfo pipeline{};
	pipeline.pVertexInputState = &vertex; pipeline.pInputAssemblyState = &assembly;
	CHECK(playspace_stride(pipeline) == 16);
	binding.stride = 28;
	attributes = {{{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 12}}};
	CHECK(playspace_stride(pipeline) == 28);
	assembly.primitiveRestartEnable = VK_TRUE;
	CHECK(playspace_stride(pipeline) == 0);
	assembly.primitiveRestartEnable = VK_FALSE;
	attributes[1].location = 0;
	CHECK(playspace_stride(pipeline) == 0);
}

TEST_CASE("SteamVR ring identification requires the complete Vulkan-backed identity")
{
	RingRecovery ring{};
	RingFields fields{uintptr_t{42}, reinterpret_cast<VkBuffer>(uintptr_t{20}), reinterpret_cast<VkDeviceMemory>(uintptr_t{21}), ring_capacity};
	ring.identity = fields;
	Buffer buffer{ring_capacity, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, fields.memory, 0};
	Mapping map{reinterpret_cast<void *>(fields.mapped), 0, VK_WHOLE_SIZE};
	REQUIRE(ring.matches(fields, buffer, map));
	SECTION("wrong mapped pointer") { fields.mapped++; }
	SECTION("wrong buffer") { fields.buffer = VK_NULL_HANDLE; }
	SECTION("wrong memory") { fields.memory = VK_NULL_HANDLE; }
	SECTION("overflowed cursor") { fields.used++; }
	SECTION("wrong capacity") { buffer.size--; }
	SECTION("other buffer use") { buffer.usage |= VK_BUFFER_USAGE_VERTEX_BUFFER_BIT; }
	SECTION("offset binding") { buffer.memory_offset = 4; }
	SECTION("partial map") { map.size = ring_capacity - 1; }
	CHECK_FALSE(ring.matches(fields, buffer, map));
}

TEST_CASE("SteamVR ring reset drains submitted reads and rejects outstanding reservations")
{
	Fixture f;
	f.state->ring_allowed = true;
	auto buffer = reinterpret_cast<VkBuffer>(uintptr_t{20});
	auto memory = reinterpret_cast<VkDeviceMemory>(uintptr_t{21});
	auto command = reinterpret_cast<VkCommandBuffer>(f.handle);
	auto queue = reinterpret_cast<VkQueue>(f.handle);
	RingFields fields{reinterpret_cast<uintptr_t>(storage.data()), buffer, memory, ring_capacity - ring_margin + 64};
	f.state->buffers.emplace(buffer, Buffer{ring_capacity, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, memory, 0});
	f.state->mappings.emplace(memory, Mapping{storage.data(), 0, VK_WHOLE_SIZE});
	identify_ring(f.state, memory, reinterpret_cast<void **>(&fields));
	REQUIRE(f.state->rings.size() == 1);
	auto &ring = f.state->rings.at(buffer);
	REQUIRE_FALSE(ring.disabled);
	ring.copy(command, fields.used);
	ring.commands.at(command).ended = true;
	idle_calls = 0;
	idle_result = VK_SUCCESS;
	idle_cursor = nullptr;
	const uint64_t original = fields.used;
	REQUIRE(ring.owner == 0);
	submit_result = VK_SUCCESS;
	VkSubmitInfo info{};
	info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	info.commandBufferCount = 1;
	info.pCommandBuffers = &command;

	SECTION("successful submit drains exactly its queue before resetting") {
		recover_rings(f.state, queue, {command});
		CHECK(idle_calls == 1);
		CHECK(fields.used == 0);
		CHECK(ring.resets == 1);
		CHECK_FALSE(ring.disabled);
	}
	SECTION("binary semaphore waits still drain the submitted upload") {
	 auto semaphore = reinterpret_cast<VkSemaphore>(uintptr_t{22});
	 VkPipelineStageFlags stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
	 info.waitSemaphoreCount = 1; info.pWaitSemaphores = &semaphore; info.pWaitDstStageMask = &stage;
	 REQUIRE(vkQueueSubmit(queue, 1, &info, VK_NULL_HANDLE) == VK_SUCCESS);
	 CHECK(idle_calls == 1); CHECK(fields.used == 0);
	 CHECK(ring.owner == static_cast<pid_t>(syscall(SYS_gettid)));
	}
	SECTION("nonzero timeline waits cannot block future CPU signal work") {
	 uint64_t value = 1;
	 VkTimelineSemaphoreSubmitInfo timeline{};
	 timeline.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
	 timeline.waitSemaphoreValueCount = 1; timeline.pWaitSemaphoreValues = &value;
	 info.pNext = &timeline;
	 REQUIRE(vkQueueSubmit(queue, 1, &info, VK_NULL_HANDLE) == VK_SUCCESS);
	 CHECK(idle_calls == 0); CHECK(fields.used == original); CHECK(ring.disabled);
	}
	SECTION("failed submission cannot retire or reset the upload") {
	 submit_result = VK_ERROR_DEVICE_LOST;
	 CHECK(vkQueueSubmit(queue, 1, &info, VK_NULL_HANDLE) == submit_result);
	 CHECK(idle_calls == 0); CHECK(fields.used == original); CHECK(ring.pending());
	}
	SECTION("upload authors cannot change after the first submission") {
	 ring.owner = static_cast<pid_t>(syscall(SYS_gettid)) + 1;
	 recover_rings(f.state, queue, {command});
	 CHECK(idle_calls == 0); CHECK(fields.used == original); CHECK(ring.disabled);
	}
	SECTION("another upload remains unsubmitted") {
		auto other = reinterpret_cast<VkCommandBuffer>(uintptr_t{22});
		ring.copy(other, original);
		recover_rings(f.state, queue, {command});
		CHECK(idle_calls == 0);
		CHECK(fields.used == original);
	}
	SECTION("CPU reserved bytes beyond the last recorded copy") {
		fields.used++;
		recover_rings(f.state, queue, {command});
		CHECK(idle_calls == 0);
		CHECK(fields.used == original + 1);
	}
	SECTION("queue drain failure leaves the cursor unchanged and disables recovery") {
		idle_result = VK_ERROR_DEVICE_LOST;
		recover_rings(f.state, queue, {command});
		CHECK(fields.used == original);
		CHECK(ring.disabled);
		CHECK(ring.resets == 0);
	}
	SECTION("reservation during drain leaves the changed cursor untouched") {
		idle_cursor = &fields.used;
		recover_rings(f.state, queue, {command});
		CHECK(fields.used == original + 4);
		CHECK(ring.disabled);
		CHECK(ring.resets == 0);
	}
	SECTION("stale allocator identity never drains or writes") {
		fields.memory = VK_NULL_HANDLE;
		recover_rings(f.state, queue, {command});
		CHECK(idle_calls == 0);
		CHECK(fields.used == original);
		CHECK(ring.disabled);
	}
	SECTION("still-recording commands cannot reset") {
		ring.commands.at(command).ended = false;
		recover_rings(f.state, queue, {command});
		CHECK(idle_calls == 0);
		CHECK(fields.used == original);
		CHECK(ring.disabled);
	}
	SECTION("repeated submission without a fresh recording is rejected") {
		ring.commands.at(command).submitted = true;
		recover_rings(f.state, queue, {command});
		CHECK(idle_calls == 0);
		CHECK(fields.used == original);
		CHECK(ring.disabled);
	}
	idle_cursor = nullptr;
}

TEST_CASE("SteamVR unknown builds and inaccessible allocator storage fail closed")
{
	CHECK_FALSE(supported_ring_build());
	RingFields fields{};
	CHECK_FALSE(local_read(1, &fields, sizeof(fields)));
	CHECK_FALSE(local_write(1, 0));
	Fixture f;
	f.state->ring_allowed = true;
	auto buffer = reinterpret_cast<VkBuffer>(uintptr_t{20});
	auto memory = reinterpret_cast<VkDeviceMemory>(uintptr_t{21});
	f.state->buffers.emplace(buffer, Buffer{ring_capacity, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, memory, 0});
	f.state->mappings.emplace(memory, Mapping{storage.data(), 0, VK_WHOLE_SIZE});
	identify_ring(f.state, memory, reinterpret_cast<void **>(uintptr_t{1}));
	REQUIRE(f.state->rings.size() == 1);
	CHECK(f.state->rings.at(buffer).disabled);
}


TEST_CASE("Room View tracks destination uniform buffers for camera correction")
{
	Fixture f;
	VkBufferCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	info.size = 400;
	info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
	VkBuffer buffer;
	REQUIRE(vkCreateBuffer(f.handle, &info, nullptr, &buffer) == VK_SUCCESS);
	REQUIRE(f.state->buffers.count(buffer) == 1);
	CHECK(f.state->buffers.at(buffer).usage == info.usage);
	CHECK(f.state->buffers.at(buffer).size == 400);
}

namespace {
std::vector<VkDisplayModePropertiesKHR> display_modes;
VkResult VKAPI_CALL fake_display_modes(VkPhysicalDevice, VkDisplayKHR, uint32_t *count, VkDisplayModePropertiesKHR *out)
{
	if (!out) {
		*count = display_modes.size();
		return VK_SUCCESS;
	}
	*count = std::min<uint32_t>(*count, display_modes.size());
	std::copy_n(display_modes.begin(), *count, out);
	return *count < display_modes.size() ? VK_INCOMPLETE : VK_SUCCESS;
}
std::vector<VkDisplayModePropertiesKHR> enumerate_modes()
{
	uint32_t count = 0;
	alias_single_display_mode(fake_display_modes, nullptr, nullptr, &count, nullptr);
	std::vector<VkDisplayModePropertiesKHR> modes(count);
	alias_single_display_mode(fake_display_modes, nullptr, nullptr, &count, modes.data());
	return modes;
}
// SteamVR 2.17.10 direct-mode rate choice: 90 Hz until a matching mode beats an earlier match.
float steamvr_rate(const std::vector<VkDisplayModePropertiesKHR> &modes, float desired)
{
	float rate = 90.0f, best = 0.0f;
	std::vector<float> seen;
	for (const auto &mode : modes) {
		float candidate = mode.parameters.refreshRate * 0.001f;
		if (std::find(seen.begin(), seen.end(), candidate) != seen.end()) continue;
		float difference = std::fabs(candidate - desired);
		if (seen.empty()) best = difference;
		else if (best > difference) rate = candidate, best = difference;
		seen.push_back(candidate);
	}
	return rate;
}
} // namespace

TEST_CASE("SteamVR adopts the refresh rate of a single-mode HMD panel")
{
	VkDisplayModeKHR handle = reinterpret_cast<VkDisplayModeKHR>(uintptr_t{7});
	display_modes = {{handle, {{1440, 2560}, 80000}}};
	CHECK(steamvr_rate(display_modes, 80.0f) == 90.0f);

	auto modes = enumerate_modes();
	REQUIRE(modes.size() == 2);
	CHECK(modes[0].displayMode == handle);
	CHECK(modes[1].displayMode == handle);
	CHECK(modes[1].parameters.refreshRate == 80000);
	CHECK(steamvr_rate(modes, 80.0f) == 80.0f);

	uint32_t count = 1;
	VkDisplayModePropertiesKHR first{};
	CHECK(alias_single_display_mode(fake_display_modes, nullptr, nullptr, &count, &first) == VK_INCOMPLETE);
	CHECK(count == 1);

	display_modes = {{handle, {{1440, 1600}, 90000}}, {handle, {{1440, 1600}, 144000}}};
	modes = enumerate_modes();
	REQUIRE(modes.size() == 2);
	CHECK(modes[0].parameters.refreshRate == 90000);
	CHECK(modes[1].parameters.refreshRate == 144000);
}
