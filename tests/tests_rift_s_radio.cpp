// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
extern "C" {
#include "rift_s/rift_s_radio.h"
#include "rift_s/rift_s_haptics.h"
#include "rift_s/rift_s_protocol.h"
#include "util/u_time.h"
#include "util/u_logging.h"
#include <errno.h>
}

// rift_s.h declares this; provide it for the standalone radio object.
extern "C" {
enum u_logging_level rift_s_log_level = U_LOGGING_WARN;

} // extern "C"

#include <cstring>
#include <functional>
#include <vector>

static constexpr int64_t MS = U_TIME_1MS_IN_NS;
static constexpr int64_t TICK = MS / 2; // USB thread cadence
static constexpr int64_t T0 = 1000LL * 1000 * 1000 * 1000;

struct Cb
{
	int calls = 0;
	bool last_success = false;
	int last_len = -1;
	uint8_t last_data[64] = {0};
};

static void
test_cb(bool success, uint8_t *bytes, int len, void *data)
{
	Cb *c = (Cb *)data;
	c->calls++;
	c->last_success = success;
	c->last_len = len;
	if (bytes != nullptr && len > 0)
		memcpy(c->last_data, bytes, (size_t)(len > 64 ? 64 : len));
}

struct FakeHid
{
	struct os_hid_device base = {};
	int set_calls = 0;
	int get_calls = 0;
	int set_result = 0; // 0 success, <0 transport failure
	int set_errno = EPIPE;
	uint8_t last_report = 0;
	std::function<int(uint8_t *, size_t)> get_fn;
};

static int
fake_set(struct os_hid_device *dev, const uint8_t *data, size_t size)
{
	FakeHid *f = (FakeHid *)dev;
	f->set_calls++;
	if (size > 0)
		f->last_report = data[0];
	if (f->set_result < 0) {
		errno = f->set_errno != 0 ? f->set_errno : EPIPE;
		return -1;
	}
	return 0; // hidraw ioctl success
}

static int
fake_get(struct os_hid_device *dev, uint8_t report, uint8_t *data, size_t size)
{
	FakeHid *f = (FakeHid *)dev;
	f->get_calls++;
	if (f->get_fn)
		return f->get_fn(data, size);
	errno = EPIPE;
	return -1;
}

static int
fake_get_timeout(struct os_hid_device *, void *, size_t, uint32_t)
{
	errno = EPIPE;
	return -1;
}

static int
fake_write(struct os_hid_device *, const uint8_t *, size_t)
{
	errno = EPIPE;
	return -1;
}

static int
fake_read(struct os_hid_device *, uint8_t *, size_t, int)
{
	return 0;
}

static int
fake_phys(struct os_hid_device *, uint8_t *, size_t)
{
	return -1;
}

static void
fake_destroy(struct os_hid_device *)
{}

static FakeHid
make_fake()
{
	FakeHid f;
	f.base.set_feature = fake_set;
	f.base.get_feature = fake_get;
	f.base.get_feature_timeout = fake_get_timeout;
	f.base.write = fake_write;
	f.base.read = fake_read;
	f.base.get_physical_address = fake_phys;
	f.base.destroy = fake_destroy;
	return f;
}

static int
fill_response(uint8_t *data, size_t size, uint8_t seq, uint8_t busy, const uint8_t *payload, int payload_len)
{
	if (size < 3)
		return 0;
	data[0] = 0x0b;
	data[1] = seq;
	data[2] = busy;
	int n = payload_len;
	if (n > (int)size - 3)
		n = (int)size - 3;
	if (n > 0 && payload != nullptr)
		memcpy(data + 3, payload, (size_t)n);
	return 3 + n;
}

static void
queue_one(rift_s_radio_state *st, Cb *cb)
{
	uint8_t cmd[] = {0x32, 0x20, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	rift_s_radio_queue_command(st, 0x13001101, cmd, sizeof(cmd), test_cb, cb);
}

TEST_CASE("Radio success delivers once and pipelines the next command")
{
	rift_s_radio_state st;
	rift_s_radio_state_init(&st);
	FakeHid hid = make_fake();
	uint8_t seq = 10;
	hid.get_fn = [&](uint8_t *data, size_t size) {
		uint8_t payload[] = {0x00, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00, 0x02, 0x00};
		return fill_response(data, size, seq++, 0x00, payload, sizeof(payload));
	};

	Cb a, b;
	queue_one(&st, &a);
	queue_one(&st, &b);
	rift_s_radio_update_at(&st, &hid.base, T0);

	CHECK(a.calls == 1);
	CHECK(a.last_success);
	CHECK(b.calls == 1);
	CHECK(b.last_success);
	CHECK(st.pending_commands == nullptr);
	CHECK_FALSE(st.command_result_pending);
	CHECK(hid.set_calls == 2);
	CHECK(hid.get_calls == 2);
	rift_s_radio_state_clear(&st);
	CHECK(a.calls == 1); // clear does not re-fire
	CHECK(b.calls == 1);
}

TEST_CASE("Radio timeout blocks later commands until transport recovery")
{
	rift_s_radio_state st;
	rift_s_radio_state_init(&st);
	FakeHid hid = make_fake();
	hid.get_fn = [&](uint8_t *data, size_t size) {
		static uint8_t pad = 0;
		return fill_response(data, size, 7, 0x01, &pad, 1);
	};

	Cb a, b;
	queue_one(&st, &a);
	// Second command succeeds once the head fails: route on sets seen.
	queue_one(&st, &b);
	hid.get_fn = [&](uint8_t *data, size_t size) {
		if (hid.set_calls >= 2) {
			uint8_t payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
			return fill_response(data, size, 9, 0x00, payload, sizeof(payload));
		}
		static uint8_t pad = 0;
		return fill_response(data, size, 7, 0x01, &pad, 1);
	};

	int64_t now = T0;
	// Run past the first head's deadline; busy polls every tick (no backoff).
	for (int i = 0; i < 4000 && a.calls == 0; i++) {
		rift_s_radio_update_at(&st, &hid.base, now);
		now += TICK;
	}
	REQUIRE(a.calls == 1);
	CHECK_FALSE(a.last_success);
	// Later requests cannot consume an ambiguous late reply.
	for (int i = 0; i < 100 && b.calls == 0; i++) {
		rift_s_radio_update_at(&st, &hid.base, now);
		now += TICK;
	}
	CHECK(b.calls == 0);
	CHECK(st.recovery_required);
	CHECK(hid.set_calls == 1);
	rift_s_radio_state_clear(&st);
	CHECK(b.calls == 1);
	CHECK_FALSE(b.last_success);
}

TEST_CASE("Radio persistent send STALL backs off and fails bounded")
{
	rift_s_radio_state st;
	rift_s_radio_state_init(&st);
	FakeHid hid = make_fake();
	hid.set_result = -1;
	hid.set_errno = EPIPE;

	Cb a;
	queue_one(&st, &a);
	int64_t now = T0;
	for (int i = 0; i < 5000 && a.calls == 0; i++) {
		rift_s_radio_update_at(&st, &hid.base, now);
		now += TICK;
	}
	REQUIRE(a.calls == 1);
	CHECK_FALSE(a.last_success);
	CHECK(st.pending_commands == nullptr);
	// 8 bounded sends, not one per 0.5 ms tick (~4000 without backoff).
	CHECK(hid.set_calls == RIFT_S_RADIO_MAX_SEND_ATTEMPTS);
	CHECK(hid.set_calls <= 10);
	// No response polls on a head that never sent.
	CHECK(hid.get_calls == 0);
	rift_s_radio_state_clear(&st);
	CHECK(a.calls == 1);
}

TEST_CASE("Radio persistent response error backs off and fails bounded")
{
	rift_s_radio_state st;
	rift_s_radio_state_init(&st);
	FakeHid hid = make_fake();
	hid.get_fn = [&](uint8_t *, size_t) {
		errno = EPIPE;
		return -1;
	};

	Cb a;
	queue_one(&st, &a);
	int64_t now = T0;
	for (int i = 0; i < 6000 && a.calls == 0; i++) {
		rift_s_radio_update_at(&st, &hid.base, now);
		now += TICK;
	}
	REQUIRE(a.calls == 1);
	CHECK_FALSE(a.last_success);
	CHECK(st.pending_commands == nullptr);
	CHECK(hid.set_calls == 1); // sent once, then only polls
	CHECK(hid.get_calls == RIFT_S_RADIO_MAX_POLL_ERRORS);
	rift_s_radio_state_clear(&st);
}

TEST_CASE("Radio persistent short reads fail bounded like errors")
{
	rift_s_radio_state st;
	rift_s_radio_state_init(&st);
	FakeHid hid = make_fake();
	hid.get_fn = [&](uint8_t *, size_t) { return 2; }; // no busy/seq fields

	Cb a;
	queue_one(&st, &a);
	int64_t now = T0;
	for (int i = 0; i < 6000 && a.calls == 0; i++) {
		rift_s_radio_update_at(&st, &hid.base, now);
		now += TICK;
	}
	REQUIRE(a.calls == 1);
	CHECK_FALSE(a.last_success);
	CHECK(hid.get_calls == RIFT_S_RADIO_MAX_POLL_ERRORS);
	rift_s_radio_state_clear(&st);
}

TEST_CASE("Radio clear fails queued callbacks exactly once and is safe to repeat")
{
	rift_s_radio_state st;
	rift_s_radio_state_init(&st);
	FakeHid hid = make_fake();
	hid.get_fn = [&](uint8_t *data, size_t size) {
		static uint8_t pad = 0;
		return fill_response(data, size, 7, 0x01, &pad, 1);
	};

	Cb a, b;
	queue_one(&st, &a);
	queue_one(&st, &b);
	rift_s_radio_update_at(&st, &hid.base, T0); // send head, busy
	rift_s_radio_state_clear(&st);
	CHECK(a.calls == 1);
	CHECK_FALSE(a.last_success);
	CHECK(b.calls == 1);
	CHECK_FALSE(b.last_success);
	CHECK(st.pending_commands == nullptr);
	CHECK_FALSE(st.command_result_pending);
	rift_s_radio_state_clear(&st); // repeat safe, no second fire
	CHECK(a.calls == 1);
	CHECK(b.calls == 1);
	rift_s_radio_update_at(&st, &hid.base, T0 + 10 * MS); // no HID use when empty
	CHECK(hid.get_calls <= 1);
}

TEST_CASE("A late response with an advanced sequence never completes a later command")
{
	rift_s_radio_state st;
	rift_s_radio_state_init(&st);
	FakeHid hid = make_fake();
	hid.get_fn = [&](uint8_t *data, size_t size) {
		uint8_t payload[] = {0};
		return fill_response(data, size, 10, 1, payload, sizeof(payload));
	};
	Cb a, b;
	queue_one(&st, &a);
	queue_one(&st, &b);
	rift_s_radio_update_at(&st, &hid.base, T0);
	rift_s_radio_update_at(&st, &hid.base, T0 + RIFT_S_RADIO_CMD_TIMEOUT_MS * MS);
	REQUIRE(st.recovery_required);
	int reads = hid.get_calls;
	hid.get_fn = [&](uint8_t *data, size_t size) {
		uint8_t payload[] = {0, 0, 0, 0, 1, 0};
		return fill_response(data, size, 11, 0, payload, sizeof(payload));
	};
	rift_s_radio_update_at(&st, &hid.base, T0 + 2000 * MS);
	CHECK(hid.get_calls == reads);
	CHECK(hid.set_calls == 1);
	CHECK(a.calls == 1);
	CHECK_FALSE(a.last_success);
	CHECK(b.calls == 0);
	rift_s_radio_state_clear(&st);
	CHECK(b.calls == 1);
	CHECK_FALSE(b.last_success);
	rift_s_radio_state_init(&st);
	CHECK_FALSE(st.recovery_required);
}

TEST_CASE("Radio with no device keeps the queue until the deadline")
{
	rift_s_radio_state st;
	rift_s_radio_state_init(&st);
	Cb a;
	queue_one(&st, &a);
	rift_s_radio_update_at(&st, nullptr, T0);
	CHECK(a.calls == 0);
	CHECK(st.pending_commands != nullptr);
	rift_s_radio_update_at(&st, nullptr, T0 + (int64_t)RIFT_S_RADIO_CMD_TIMEOUT_MS * MS);
	CHECK(a.calls == 1);
	CHECK_FALSE(a.last_success);
	CHECK(st.pending_commands == nullptr);
	rift_s_radio_state_clear(&st);
}

TEST_CASE("Haptic latched stop survives a transport reset")
{
	// close_transport clears the request side but keeps the latched motor identity.
	struct rift_s_haptic_radio h = {};
	h.hand = 0;
	h.device_id = 0;
	h.amplitude = 0;
	h.end_ns = 0;
	h.applied_device_id = 0x13001101;
	h.applied_amplitude = 200;

	uint64_t id = 0;
	uint8_t amp = 0xFF;
	CHECK(rift_s_haptic_next(&h, T0, &id, &amp));
	CHECK(id == 0x13001101);
	CHECK(amp == 0);

	// Nothing ever latched: no spurious stop, no pairing-list probe needed.
	struct rift_s_haptic_radio idle = {};
	id = 0xDEAD;
	amp = 0xFF;
	CHECK_FALSE(rift_s_haptic_next(&idle, T0, &id, &amp));
}
