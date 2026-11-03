// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s/rift_s_interface.h"
#include <array>
#include <chrono>

struct Prober : xrt_prober
{
	xrt_prober_device hmd{OCULUS_VR_INC_VID, OCULUS_RIFT_S_PID, XRT_BUS_TYPE_USB, 0};
	xrt_prober_device hub{OCULUS_VR_INC_VID, OCULUS_RIFT_S_USB3_HUB_PID, XRT_BUS_TYPE_USB, 0};
	std::array<xrt_prober_device *, 2> devices{&hmd, &hub};
	size_t count = 1;
	unsigned opens = 0, unlocks = 0;

	Prober() : xrt_prober{}
	{
		lock_list = [](xrt_prober *xp, xrt_prober_device ***out, size_t *size) {
			auto &p = *static_cast<Prober *>(xp);
			*out = p.devices.data();
			*size = p.count;
			return XRT_SUCCESS;
		};
		unlock_list = [](xrt_prober *xp, xrt_prober_device ***out) {
			static_cast<Prober *>(xp)->unlocks++;
			*out = nullptr;
			return XRT_SUCCESS;
		};
		open_hid_interface = [](xrt_prober *xp, xrt_prober_device *, int, os_hid_device **) {
			static_cast<Prober *>(xp)->opens++;
			// Stop at the first real hardware operation, before any startup sleeps.
			return -1;
		};
	}
};

TEST_CASE("Rift S startup without the USB 3 hub fails before opening HID or initializing tracking")
{
	Prober prober;
	auto *builder = rift_s_builder_create();
	xrt_system_devices *devices = nullptr;
	xrt_space_overseer *spaces = nullptr;
	auto start = std::chrono::steady_clock::now();
	CHECK(builder->open_system(builder, nullptr, &prober, nullptr, &devices, &spaces) ==
	      XRT_ERROR_DEVICE_CREATION_FAILED);
	CHECK(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));
	CHECK(prober.opens == 0);
	CHECK(prober.unlocks == 1);
	CHECK(devices == nullptr);
	CHECK(spaces == nullptr);
	builder->destroy(builder);
}

// The cameras enumerate only after power-on, so the hub alone must let startup proceed.
TEST_CASE("Rift S startup requires its USB 3 hub VID/PID, not the cameras")
{
	Prober prober;
	prober.count = 2;
	auto *builder = rift_s_builder_create();
	xrt_system_devices *devices = nullptr;
	xrt_space_overseer *spaces = nullptr;
	SECTION("hub present") {}
	SECTION("wrong vendor") { prober.hub.vendor_id = 0x1234; }
	SECTION("wrong product") { prober.hub.product_id = 0x1234; }
	SECTION("wrong bus") { prober.hub.bus = XRT_BUS_TYPE_BLUETOOTH; }
	CHECK(builder->open_system(builder, nullptr, &prober, nullptr, &devices, &spaces) ==
	      XRT_ERROR_DEVICE_CREATION_FAILED);
	bool hub_present = prober.hub.vendor_id == OCULUS_VR_INC_VID &&
	                   prober.hub.product_id == OCULUS_RIFT_S_USB3_HUB_PID && prober.hub.bus == XRT_BUS_TYPE_USB;
	CHECK(prober.opens == (hub_present ? 1 : 0));
	CHECK(prober.unlocks == 1);
	builder->destroy(builder);
}
