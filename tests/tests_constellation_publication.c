// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include <stdio.h>
#include <string.h>

// Include the production publication path to check ownership before and after fusion acceptance.
#include "../src/xrt/tracking/constellation/t_constellation_tracking.c"

#define CHECK(condition)                                                                                             \
	do {                                                                                                               \
		if (!(condition)) {                                                                                         \
			fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition);                     \
			return 1;                                                                                              \
		}                                                                                                          \
	} while (0)

static bool accept_observation;
static unsigned observation_calls;

static xrt_result_t
fake_get_tracked_pose(struct xrt_device *xdev,
                      enum xrt_input_name name,
                      int64_t at_timestamp_ns,
                      struct xrt_space_relation *out_relation)
{
	(void)xdev;
	(void)name;
	(void)at_timestamp_ns;
	*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	out_relation->pose.orientation.w = 1.0f;
	out_relation->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	return XRT_SUCCESS;
}

static bool
fake_push_pose_observation(struct xrt_device *xdev, const struct t_constellation_pose_observation *obs)
{
	(void)xdev;
	if (obs->capture_ns != INT64_C(1100000000)) {
		return false;
	}
	observation_calls++;
	return accept_observation;
}

static int
run_submit_acceptance_contract(enum constellation_track_state initial_state)
{
	struct t_constellation_tracker tracker = {0};
	struct t_constellation_tracked_device_connection connection = {0};
	struct t_constellation_tracked_device_callbacks callbacks = {0};
	struct xrt_device xdev = {0};
	struct constellation_tracking_sample sample = {0};
	struct constellation_tracker_device *device = &tracker.devices[0];
	struct tracking_sample_device_state *state = &sample.devices[0];

	CHECK(os_mutex_init(&tracker.tracked_device_lock) == 0);
	CHECK(os_mutex_init(&connection.lock) == 0);
	tracker.log_level = U_LOGGING_ERROR;
	connection.xdev = &xdev;
	connection.cb = &callbacks;
	connection.tracker = &tracker;
	callbacks.push_pose_observation = fake_push_pose_observation;
	xdev.get_tracked_pose = fake_get_tracked_pose;
	device->connection = &connection;
	device->alias = "fixture";
	device->last_matched_cam = -1;
	device->led_model.P_device_model.orientation.w = 1.0f;
	device->led_model.P_model_device.orientation.w = 1.0f;

	constellation_track_init(&device->track);
	device->track.state = initial_state;
	device->track.ref.P_world_model.orientation.w = 1.0f;
	device->track.ref.imu_world.w = 1.0f;
	device->track.ref.capture_ns = INT64_C(1000000000);
	device->track.published = device->track.ref;
	device->track.have_published = true;
	device->track.ref_orientation_observed = false;
	device->have_last_seen_pose = true;
	device->last_seen_pose_ts = INT64_C(1000000000);
	device->last_seen_pose.position.x = -7.0f;
	device->last_matched_blobs = 17;
	device->diagnostics.published = 23;

	sample.timestamp = INT64_C(1100000000);
	sample.n_devices = 1;
	sample.n_views = 0;
	sample.views[0].P_cam_world.orientation.w = 1.0f;
	state->dev_index = 0;
	state->joint_result.P_world_model.orientation.w = 1.0f;
	state->joint_result.inliers = 1;
	state->final_pose.position.x = -9.0f;
	state->found_device_pose = false;

	const struct constellation_track before_reject = device->track;
	accept_observation = false;
	observation_calls = 0;
	CHECK(submit_joint_pose(&tracker, state, &sample, "fixture") == JOINT_POSE_FUSION_REJECTED);
	CHECK(observation_calls == 1);
	CHECK(device->track.state == before_reject.state);
	CHECK(device->track.ref.capture_ns == before_reject.ref.capture_ns);
	CHECK(device->track.ref.P_world_model.position.x == before_reject.ref.P_world_model.position.x);
	CHECK(device->track.ref.imu_world.w == before_reject.ref.imu_world.w);
	CHECK(device->track.published.capture_ns == before_reject.published.capture_ns);
	CHECK(device->track.have_published == before_reject.have_published);
	CHECK(device->track.ref_orientation_observed == before_reject.ref_orientation_observed);
	CHECK(device->track.consecutive_jumps == before_reject.consecutive_jumps);
	CHECK(device->have_last_seen_pose);
	CHECK(device->last_seen_pose_ts == INT64_C(1000000000));
	CHECK(device->last_seen_pose.position.x == -7.0f);
	CHECK(device->last_matched_blobs == 17);
	CHECK(device->diagnostics.published == 23);
	CHECK(device->diagnostics.confirmed == 0);
	CHECK(!state->found_device_pose);
	CHECK(state->final_pose.position.x == -9.0f);

	accept_observation = true;
	CHECK(submit_joint_pose(&tracker, state, &sample, "fixture") == JOINT_POSE_PUBLISHED);
	CHECK(observation_calls == 2);
	CHECK(state->found_device_pose);
	CHECK(device->last_seen_pose_ts == INT64_C(1100000000));
	CHECK(device->track.ref.capture_ns == INT64_C(1100000000));
	CHECK(device->diagnostics.published == 24);
	CHECK(device->diagnostics.confirmed == (initial_state == CONSTELLATION_TRACK_CONFIRMING));

	os_mutex_destroy(&connection.lock);
	os_mutex_destroy(&tracker.tracked_device_lock);
	return 0;
}

struct arbitration_device
{
	struct xrt_device base;
	bool accept;
	unsigned calls;
};

static bool
arbitration_push_pose(struct xrt_device *xdev, const struct t_constellation_pose_observation *obs)
{
	struct arbitration_device *device = (struct arbitration_device *)xdev;
	device->calls++;
	return device->accept && obs->capture_ns == INT64_C(1100000000);
}

static int
run_arbitration_contract(unsigned stronger_index, bool accept_stronger, bool hold_stronger)
{
	struct t_constellation_tracker tracker = {0};
	struct constellation_tracking_sample sample = {0};
	struct t_constellation_tracked_device_connection connections[2] = {0};
	struct t_constellation_tracked_device_callbacks callbacks = {.push_pose_observation = arbitration_push_pose};
	struct arbitration_device devices[2] = {0};
	bool candidates[CONSTELLATION_MAX_DEVICES] = {true, true};
	const unsigned weaker_index = 1 - stronger_index;
	CHECK(os_mutex_init(&tracker.tracked_device_lock) == 0);
	tracker.log_level = U_LOGGING_ERROR;
	sample.n_devices = 2;
	sample.timestamp = INT64_C(1100000000);
	sample.views[0].P_cam_world.orientation.w = 1.0f;
	for (unsigned d = 0; d < 2; d++) {
		CHECK(os_mutex_init(&connections[d].lock) == 0);
		connections[d].xdev = &devices[d].base;
		connections[d].cb = &callbacks;
		connections[d].tracker = &tracker;
		devices[d].base.get_tracked_pose = fake_get_tracked_pose;
		devices[d].accept = d == weaker_index || accept_stronger;
		struct constellation_tracker_device *device = &tracker.devices[d];
		device->connection = &connections[d];
		device->alias = d == stronger_index ? "stronger" : "weaker";
		device->led_model.P_device_model.orientation.w = 1.0f;
		device->led_model.P_model_device.orientation.w = 1.0f;
		constellation_track_init(&device->track);
		device->track.state = d == stronger_index && hold_stronger ? CONSTELLATION_TRACK_LOST
		                                                          : CONSTELLATION_TRACK_TRACKING;
		device->track.ref.capture_ns = INT64_C(1000000000);
		device->track.ref.P_world_model.orientation.w = 1.0f;
		device->track.ref.imu_world.w = 1.0f;
		struct tracking_sample_device_state *state = &sample.devices[d];
		state->dev_index = d;
		state->joint_result.P_world_model.orientation.w = 1.0f;
		state->joint_result.inliers = d == stronger_index ? 8 : 4;
		state->joint_result.distinct_leds = 4;
		state->joint_result.num_assignments = 1;
		state->joint_result.assignments[0].camera_index = 0;
		state->joint_result.assignments[0].blob_index = 0;
	}
	arbitrate_and_submit(&tracker, &sample, candidates, "fixture");
	CHECK(devices[stronger_index].calls == (hold_stronger ? 0 : 1));
	CHECK(devices[weaker_index].calls == (!accept_stronger && !hold_stronger ? 1 : 0));
	CHECK(sample.devices[stronger_index].found_device_pose == (accept_stronger && !hold_stronger));
	CHECK(sample.devices[weaker_index].found_device_pose == (!accept_stronger && !hold_stronger));
	CHECK(tracker.devices[stronger_index].diagnostics.published == (accept_stronger && !hold_stronger));
	CHECK(tracker.devices[weaker_index].diagnostics.published == (!accept_stronger && !hold_stronger));
	if (hold_stronger) {
		CHECK(tracker.devices[stronger_index].track.state == CONSTELLATION_TRACK_CONFIRMING);
	}
	for (unsigned d = 0; d < 2; d++) {
		os_mutex_destroy(&connections[d].lock);
	}
	os_mutex_destroy(&tracker.tracked_device_lock);
	return 0;
}

int
main(void)
{
	return run_submit_acceptance_contract(CONSTELLATION_TRACK_TRACKING) ||
	       run_submit_acceptance_contract(CONSTELLATION_TRACK_CONFIRMING) ||
	       run_arbitration_contract(0, false, false) || run_arbitration_contract(1, false, false) ||
	       run_arbitration_contract(0, true, false) || run_arbitration_contract(1, true, false) ||
	       run_arbitration_contract(0, false, true) || run_arbitration_contract(1, false, true);
}
