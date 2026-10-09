// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s_camera_metadata.h"
extern "C" {
#include "rift_s_firmware.h"
#include "rift_s_protocol.h"
#include "util/u_logging.h"
}
#include <array>
#include <cstring>
#include <vector>

enum u_logging_level rift_s_log_level = U_LOGGING_ERROR;

TEST_CASE("Rift S controller register decoding validates status and actual payload length")
{
	// 0x32 callback bytes, excluding HID report ID/sequence/busy: status, length, payload.
	std::array<uint8_t, 21> response = {0, 0, 0, 0, 16, 0x00, 0x7d, 0xa0, 0x0f, 0xf4, 0x01,
	                                    0xf4, 0x01, 0x00, 0x00, 0x80, 0x3a, 0xff, 0xff, 0xf9, 0x3d};
	rift_s_controller_config config{};
	REQUIRE(rift_s_decode_controller_config(response.data(), response.size(), &config));
	CHECK(config.accel_limit == 32000);
	CHECK(config.gyro_limit == 4000);
	CHECK(config.accel_hz == 500);
	CHECK(config.gyro_hz == 500);
	CHECK(config.accel_scale == Catch::Approx(1.0 / 1024));
	CHECK(config.gyro_scale == Catch::Approx(0.1220703));
	for (size_t len = 0; len < response.size(); len++)
		CHECK_FALSE(rift_s_decode_controller_config(response.data(), len, &config));
	for (unsigned i = 0; i < 4; i++) {
		auto failed = response;
		failed[i] = 1;
		CHECK_FALSE(rift_s_decode_controller_config(failed.data(), failed.size(), &config));
	}
	response[4] = 32; // Declared length cannot exceed the received bytes.
	CHECK_FALSE(rift_s_decode_controller_config(response.data(), response.size(), &config));
	response[4] = 16;
	std::fill(response.begin() + 13, response.begin() + 17, 0); // Zero accel scale.
	CHECK_FALSE(rift_s_decode_controller_config(response.data(), response.size(), &config));
	response[15] = 0xc0;
	response[16] = 0x7f; // NaN accel scale.
	CHECK_FALSE(rift_s_decode_controller_config(response.data(), response.size(), &config));

	response = {};
	response[4] = 16;
	std::memcpy(response.data() + 5, "LSM6DSL", 7);
	std::array<char, RIFT_S_CONTROLLER_IMU_DESCRIPTOR_SIZE + 1> descriptor{};
	REQUIRE(rift_s_decode_controller_imu_descriptor(response.data(), response.size(), descriptor.data()));
	CHECK(std::strcmp(descriptor.data(), "LSM6DSL") == 0);
	for (size_t len = 0; len < response.size(); len++)
		CHECK_FALSE(rift_s_decode_controller_imu_descriptor(response.data(), len, descriptor.data()));
	response[0] = 1;
	CHECK_FALSE(rift_s_decode_controller_imu_descriptor(response.data(), response.size(), descriptor.data()));
	response[0] = 0;
	response[4] = 15;
	CHECK_FALSE(rift_s_decode_controller_imu_descriptor(response.data(), response.size(), descriptor.data()));
	response[4] = 16;
	response[5] = 0xff;
	CHECK_FALSE(rift_s_decode_controller_imu_descriptor(response.data(), response.size(), descriptor.data()));
	response[5] = 0;
	CHECK_FALSE(rift_s_decode_controller_imu_descriptor(response.data(), response.size(), descriptor.data()));
	std::memcpy(response.data() + 5, "0123456789abcdef", 16);
	REQUIRE(rift_s_decode_controller_imu_descriptor(response.data(), response.size(), descriptor.data()));
	CHECK(std::strcmp(descriptor.data(), "0123456789abcdef") == 0);
}

TEST_CASE("Rift S LSM6DSL correction scales factory gyro matrix and sensor bias before applying them")
{
	rift_s_tracked_imu_calibration factory{};
	factory.num_values = 12;
	factory.matrix = {{0, 2, 0, -3, 0, 0, 0, 0, 4}};
	factory.offset = {0.1f, -0.2f, 0.3f};
	constexpr double factor = 0.8714285714285714;
	auto gyro = rift_s_controller_gyro_calibration_for_imu(&factory, "LSM6DSL");
	CHECK(gyro.num_values == 12);
	for (unsigned i = 0; i < 9; i++)
		CHECK(gyro.matrix.v[i] == (float)(factory.matrix.v[i] * factor));
	CHECK(gyro.offset.x == (float)(factory.offset.x * factor));
	CHECK(gyro.offset.y == (float)(factory.offset.y * factor));
	CHECK(gyro.offset.z == (float)(factory.offset.z * factor));
	xrt_vec3 sample{1, 2, 3};
	xrt_vec3 unbiased{sample.x - gyro.offset.x, sample.y - gyro.offset.y, sample.z - gyro.offset.z};
	xrt_vec3 corrected{};
	math_matrix_3x3_transform_vec3(&gyro.matrix, &unbiased, &corrected);
	CHECK(corrected.x == Catch::Approx(2 * factor * (2 + 0.2 * factor)));
	CHECK(corrected.y == Catch::Approx(-3 * factor * (1 - 0.1 * factor)));
	CHECK(corrected.z == Catch::Approx(4 * factor * (3 - 0.3 * factor)));
	// Re-derivation always starts from the factory data, so callbacks cannot compound the factor.
	auto again = rift_s_controller_gyro_calibration_for_imu(&factory, "LSM6DSL");
	CHECK(std::memcmp(&again, &gyro, sizeof(gyro)) == 0);
	for (const char *name : {"", "UNKNOWN", "lsm6dsl", "LSM6DSL-rev2"}) {
		auto unchanged = rift_s_controller_gyro_calibration_for_imu(&factory, name);
		CHECK(std::memcmp(&unchanged, &factory, sizeof(factory)) == 0);
	}
}

static void
put_le(uint8_t *dst, uint64_t value, size_t count)
{
	for (size_t i = 0; i < count; i++)
		dst[i] = (value >> (8 * i)) & 0xff;
}

static std::array<uint8_t, RIFT_S_METADATA_SIZE>
metadata()
{
	std::array<uint8_t, RIFT_S_METADATA_SIZE> raw{};
	raw[0] = 0x86;
	put_le(raw.data() + 1, 0xabcd, 2);
	put_le(raw.data() + 3, 65535, 2);
	put_le(raw.data() + 16, 0x0102030405060708ull, 8);
	put_le(raw.data() + 24, 123456, 4);
	for (int i = 0; i < 5; i++) {
		put_le(raw.data() + 28 + 2 * i, 38 + i, 2);
		raw[40 + i] = 64 + 16 * i;
	}
	put_le(raw.data() + 46, rift_s_metadata_crc(raw.data() + 16, 30), 2);
	put_le(raw.data() + 48, 0xface, 2);
	return raw;
}

TEST_CASE("Rift S metadata CRC checks the defined payload and retains kind and camera values")
{
	const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
	CHECK(rift_s_metadata_crc(check, sizeof(check)) == 0x29b1);
	auto raw = metadata();
	rift_s_frame_metadata out{};
	REQUIRE(rift_s_metadata_decode(raw.data(), &out) == RIFT_S_METADATA_OK);
	CHECK(out.frame_type == 0x86);
	CHECK(out.frame_ctr == 65535);
	CHECK(out.frame_ts_us == 0x0102030405060708ull);
	CHECK(out.frame_ctr2 == 123456);
	for (int i = 0; i < 5; i++) {
		CHECK(out.exposure[i] == 38 + i);
		CHECK(out.gain[i] == 64 + 16 * i);
	}
	for (size_t i = 16; i < 48; i++) {
		for (int bit = 0; bit < 8; bit++) {
			auto changed = raw;
			changed[i] ^= 1u << bit;
			CHECK(rift_s_metadata_decode(changed.data(), &out) == RIFT_S_METADATA_BAD_CRC);
		}
	}
	raw[0] = 0x06; // Frame kind and exposure counter are outside the firmware CRC payload.
	put_le(raw.data() + 3, 0, 2);
	REQUIRE(rift_s_metadata_decode(raw.data(), &out) == RIFT_S_METADATA_OK);
	CHECK(out.frame_type == 0x06);
	CHECK(out.frame_ctr == 0);
	raw[1] ^= 1;
	CHECK(rift_s_metadata_decode(raw.data(), &out) == RIFT_S_METADATA_BAD_MAGIC);
}

TEST_CASE("Rift S top-row extraction uses stride and rejects incomplete transport data")
{
	const size_t stride = RIFT_S_METADATA_WIDTH + 128;
	std::vector<uint8_t> pixels(stride * 8);
	auto raw = metadata();
	for (size_t i = 0; i < 50 * 8; i++) {
		pixels[4 * stride + 4 + 8 * i] = (raw[i / 8] & (1u << (7 - i % 8))) ? 255 : 0;
	}
	rift_s_frame_metadata out{};
	REQUIRE(rift_s_metadata_extract(pixels.data(), RIFT_S_METADATA_WIDTH, 8, stride, pixels.size(), &out) ==
	        RIFT_S_METADATA_OK);
	CHECK(out.frame_ts_us == 0x0102030405060708ull);
	CHECK(rift_s_metadata_extract(pixels.data(), RIFT_S_METADATA_WIDTH, 8, stride, 4 * stride, &out) ==
	      RIFT_S_METADATA_BAD_LAYOUT);
	CHECK(rift_s_metadata_extract(pixels.data(), RIFT_S_METADATA_WIDTH, 7, stride, pixels.size(), &out) ==
	      RIFT_S_METADATA_BAD_LAYOUT);
	CHECK(rift_s_metadata_extract(pixels.data(), RIFT_S_METADATA_WIDTH, 8, RIFT_S_METADATA_WIDTH - 1, pixels.size(),
	                              &out) == RIFT_S_METADATA_BAD_LAYOUT);
}

TEST_CASE("Rift S exposure counter accepts wrap and separates gaps from resets")
{
	uint16_t missing = 99;
	CHECK(rift_s_metadata_counter_delta(65535, 0, &missing));
	CHECK(missing == 0);
	CHECK(rift_s_metadata_counter_delta(65534, 2, &missing));
	CHECK(missing == 3);
	CHECK_FALSE(rift_s_metadata_counter_delta(42, 42, &missing));
	CHECK(missing == 0);
	CHECK_FALSE(rift_s_metadata_counter_delta(42, 41, &missing));
	CHECK_FALSE(rift_s_metadata_counter_delta(0, 32768, &missing));
}

TEST_CASE("Rift S firmware keeps numbered LED identities and each lensing model")
{
	// Reverse object insertion order to check identities rather than JSON traversal order.
	char json[] = R"({"TrackedObject":{
	 "FlsVersion":"1.0.10", "ImuPosition":[0.005,-0.007,0.020],
	 "AccCalibration":[0,1,0,-1,0,0,0,0,1,0.1,-0.2,0.3],
	 "GyroCalibration":[1,0,0,0,1,0,0,0,1,0.01,0.02,0.03],
	 "ModelPoints":{"Point1":[0.02,0.03,0.04,0,1,0,85,80,0],
	                "Point0":[-0.01,0.02,0.03,1,0,0,85,80,0]},
	 "Lensing":{"Model1":[4,11,12,13,14],"Model0":[4,1,2,3,4]}},
	 "acc_m":[1,0,0,0,0,-1,0,1,0], "acc_b":[0.1,0.2,0.3],
	 "gyro_m":[1,0,0,0,0,-1,0,1,0], "gyro_b":[0.01,0.02,0.03]})";
	rift_s_controller_imu_calibration c{};
	REQUIRE(rift_s_controller_parse_imu_calibration(json, &c) == 0);
	CHECK(c.num_leds == 2);
	CHECK(c.leds[0].pos.x == Catch::Approx(-0.01));
	CHECK(c.leds[0].dir.x == 1);
	CHECK(c.leds[1].pos.x == Catch::Approx(0.02));
	CHECK(c.leds[1].dir.y == 1);
	CHECK(c.imu_position.x == Catch::Approx(0.005));
	// TrackedObject IMU calibration: row-major 3x3 (elements 0..8) and offset (9..11).
	CHECK(c.accel_calibration.num_values == 12);
	CHECK(c.accel_calibration.matrix.v[1] == 1);
	CHECK(c.accel_calibration.matrix.v[3] == -1);
	CHECK(c.accel_calibration.matrix.v[8] == 1);
	CHECK(c.accel_calibration.offset.x == Catch::Approx(0.1));
	CHECK(c.accel_calibration.offset.y == Catch::Approx(-0.2));
	CHECK(c.accel_calibration.offset.z == Catch::Approx(0.3));
	CHECK(c.gyro_calibration.offset.z == Catch::Approx(0.03));
	CHECK(c.num_lensing_models == 2);
	for (int i = 0; i < 4; i++) {
		CHECK(c.lensing_models[0].points[i] == i + 1);
		CHECK(c.lensing_models[1].points[i] == i + 11);
	}
	xrt_vec3 raw_accel{0, 0, 1}, corrected{};
	math_matrix_3x3_transform_vec3(&c.accel.rectification, &raw_accel, &corrected);
	CHECK(corrected.x == 0);
	CHECK(corrected.y == -1);
	CHECK(corrected.z == 0);
	rift_s_controller_free_imu_calibration(&c);
	CHECK(c.leds == nullptr);
	CHECK(c.lensing_models == nullptr);
}

TEST_CASE("Rift S controller IR LED register matches the Windows radio requests")
{
	// Windows sends 0x3d bytes: report id, LE64 device id, then the request.
	STATIC_REQUIRE(sizeof(rift_s_hmd_radio_command_t) == 0x3d);

	std::array<uint8_t, 4> read{};
	REQUIRE(rift_s_encode_controller_irled_read(read.data()) == 4);
	CHECK(read == std::array<uint8_t, 4>{0x28, 0x00, 0xe8, 0x03});

	// The Windows runtime wrote 19/40000 to both controllers.
	rift_s_controller_irled_config cfg{40000, 19};
	std::array<uint8_t, 12> write{};
	REQUIRE(rift_s_encode_controller_irled_write(&cfg, write.data()) == 12);
	CHECK(write == std::array<uint8_t, 12>{0x28, 8, 0xe8, 0x03, 0x40, 0x9c, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00});

	// Read reply: LE32 status, length, LE32 period_us, LE32 ontime_us.
	std::array<uint8_t, 13> reply = {0, 0, 0, 0, 8, 0x05, 0x82, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00};
	rift_s_controller_irled_config got{};
	REQUIRE(rift_s_decode_controller_irled_config(reply.data(), reply.size(), &got));
	CHECK(got.period_us == 33285);
	CHECK(got.ontime_us == 19);
	for (size_t len = 0; len < reply.size(); len++)
		CHECK_FALSE(rift_s_decode_controller_irled_config(reply.data(), len, &got));
	reply[4] = 7;
	CHECK_FALSE(rift_s_decode_controller_irled_config(reply.data(), reply.size(), &got));
	reply[4] = 8;
	reply[1] = 1;
	CHECK_FALSE(rift_s_decode_controller_irled_config(reply.data(), reply.size(), &got));

	uint32_t status = 0xffffffff;
	std::array<uint8_t, 4> ok = {0, 0, 0, 0};
	CHECK(rift_s_controller_write_succeeded(ok.data(), ok.size(), &status));
	CHECK(status == 0);
	std::array<uint8_t, 4> bad = {0x05, 0, 0, 0};
	CHECK_FALSE(rift_s_controller_write_succeeded(bad.data(), bad.size(), &status));
	CHECK(status == 5);
	CHECK_FALSE(rift_s_controller_write_succeeded(ok.data(), 3, &status));
}
