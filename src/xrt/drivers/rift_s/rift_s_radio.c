/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
/*!
 * @file
 * @brief  Oculus Rift S HMD Radio management
 *
 * Functions for serialising requests to communicate with
 * Touch controllers over the HMDs wireless radio link,
 * collecting responses and delivering them back via callbacks.
 *
 * Ported from OpenHMD
 *
 * @author Jan Schmidt <jan@centricular.com>
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "util/u_misc.h"
#include "util/u_time.h"
#include "os/os_time.h"
#include "rift_s.h"
#include "rift_s_radio.h"
#include "rift_s_protocol.h"

/* Struct that forms a double linked queue of pending commands,
 * with the head being the currently active command */
struct rift_s_radio_command
{
	rift_s_radio_command *prev;
	rift_s_radio_command *next;

	/* Request packet data; read_command.cmd is the HID report id */
	rift_s_hmd_radio_command_t read_command;

	/* Completion callback */
	rift_s_radio_completion_fn cb;
	void *cb_data;

	/* Monotonic deadline from first update (0 = not yet armed). Bounds total
	 * queued send + response wait so a missing reply cannot wedge the queue. */
	timepoint_ns deadline_ns;
	/* Earliest monotonic time for the next HID op after a transport error.
	 * Valid busy polls retry every update; only short/error backs off. */
	timepoint_ns next_attempt_ns;
	/* Bounded retry counters. Send retries reuse the same payload. */
	int send_attempts;
	int poll_errors;
	/* Set once os_hid_set_feature succeeds for this head. */
	bool sent;
};

static time_duration_ns
radio_retry_delay_ns(int failures)
{
	/* 5, 10, 20, 40, 80, 100, 100, ... ms. failures is 1-based. */
	time_duration_ns delay =
	    (time_duration_ns)RIFT_S_RADIO_RETRY_BASE_MS * U_TIME_1MS_IN_NS << (failures - 1);
	time_duration_ns cap = (time_duration_ns)RIFT_S_RADIO_RETRY_MAX_MS * U_TIME_1MS_IN_NS;
	if (delay > cap || delay <= 0)
		delay = cap;
	return delay;
}

/* Pop the head, report failure once, and leave the next command unsent.
 * Keeps last_radio_seqnum so a late duplicate with the old seq stays ignored. */
static void
radio_fail_head(rift_s_radio_state *state, rift_s_radio_command *cmd)
{
	state->pending_commands = cmd->next;
	if (state->pending_commands == NULL)
		state->pending_commands_tail = NULL;
	else
		state->pending_commands->prev = NULL;

	state->command_result_pending = false;

	if (cmd->cb)
		cmd->cb(false, NULL, 0, cmd->cb_data);
	free(cmd);
}

static int
get_radio_response_report(struct os_hid_device *hid, rift_s_hmd_radio_response_t *radio_response)
{
	int ret;

	radio_response->cmd = 0xb;
	ret = os_hid_get_feature(hid, radio_response->cmd, (uint8_t *)(radio_response),
	                         sizeof(rift_s_hmd_radio_response_t));

	return ret;
}

void
rift_s_radio_update_at(rift_s_radio_state *state, struct os_hid_device *hid, int64_t now_ns)
{
	if (state == NULL)
		return;

	bool read_another = false;
	timepoint_ns timeout_ns = (timepoint_ns)RIFT_S_RADIO_CMD_TIMEOUT_MS * U_TIME_1MS_IN_NS;

	do {
		read_another = false;

		rift_s_radio_command *cmd = state->pending_commands;
		if (cmd == NULL) {
			state->command_result_pending = false;
			break; /* Nothing to do right now */
		}

		/* Arm the monotonic deadline on first update so queue + send + response
		 * are bounded even if HID never succeeds. */
		if (cmd->deadline_ns == 0)
			cmd->deadline_ns = now_ns + timeout_ns;

		if (now_ns >= cmd->deadline_ns) {
			RIFT_S_WARN("Controller radio command timed out, failing head");
			radio_fail_head(state, cmd);
			read_another = state->pending_commands != NULL;
			continue;
		}

		if (hid == NULL)
			break; /* No device: keep queued until the deadline above expires. */

		/* Send a radio command if there is none active and some pending.
		 * Retries resend the same unsent head with identical bytes, so a late
		 * reply stays a reply to the same payload. Once sent, the head is only
		 * polled, never resent: the response carries no command echo (only
		 * seqnum/busy), so failing to the next command can misattribute a late
		 * reply and callers must validate payload status/length. */
		if (state->command_result_pending == false) {
			if (now_ns < cmd->next_attempt_ns)
				break; /* Send backoff after a transport error. */

			rift_s_hmd_radio_command_t *pkt = &cmd->read_command;

			/* Any negative HID return is a transport failure; errno semantics unchanged. */
			if (os_hid_set_feature(hid, (uint8_t *)pkt, sizeof(*pkt)) < 0) {
				cmd->send_attempts++;
				if (cmd->send_attempts >= RIFT_S_RADIO_MAX_SEND_ATTEMPTS) {
					RIFT_S_WARN("Controller radio send failed %d times, failing head",
					            cmd->send_attempts);
					radio_fail_head(state, cmd);
					break; /* One HID write per tick; next head runs next update. */
				}
				cmd->next_attempt_ns = now_ns + radio_retry_delay_ns(cmd->send_attempts);
				RIFT_S_DEBUG("Failed to send controller radio command, retrying");
				break;
			}
			// rift_s_hexdump_buffer ("ControllerFWSend", (unsigned char *)(pkt), sizeof(*pkt));
			cmd->sent = true;
			cmd->poll_errors = 0;
			cmd->next_attempt_ns = 0;
			state->command_result_pending = true;
		}

		if (now_ns < cmd->next_attempt_ns)
			break; /* Poll backoff after a short/error read. */

		/* There's a command result pending, poll the radio response until it's complete */
		rift_s_hmd_radio_response_t radio_response;

		/* The radio response is ready when the busy flag has cleared, and the seqnum
		 * has incremented */
		int ret = get_radio_response_report(hid, &radio_response);

		/* The busy flag and seqnum occupy the first three bytes; a shorter
		 * read carries no usable fields (busy_flag would be uninitialised stack). */
		if (ret < 4) {
			/* Transport error or short read: back off so a persistent STALL
			 * (EPIPE) cannot spin the queue, and bound consecutive failures
			 * so the head still fails instead of wedging. */
			cmd->poll_errors++;
			if (cmd->poll_errors >= RIFT_S_RADIO_MAX_POLL_ERRORS) {
				RIFT_S_WARN("Controller radio response failed %d times, failing head",
				            cmd->poll_errors);
				radio_fail_head(state, cmd);
				break; /* Next head runs next update. */
			}
			cmd->next_attempt_ns = now_ns + radio_retry_delay_ns(cmd->poll_errors);
			RIFT_S_DEBUG("Failed to read controller radio response, retrying");
			break;
		}

		cmd->poll_errors = 0;
		cmd->next_attempt_ns = 0;

		if (radio_response.busy_flag != 0x00 || radio_response.seqnum == state->last_radio_seqnum) {
			if (radio_response.busy_flag)
				state->last_radio_seqnum = radio_response.seqnum;
			break; /* Valid busy/not-ready: retry next update with no backoff. */
		}
		state->last_radio_seqnum = radio_response.seqnum;

		/* We have the controller response! */
		assert(ret > 3 && ret <= (int)sizeof(radio_response));

		state->command_result_pending = false;
		// rift_s_hexdump_buffer ("ControllerFWReply", (unsigned char *)(&radio_response), ret);

		assert(state->pending_commands != NULL);
		assert(cmd == state->pending_commands);

		/* Pop the head off the cmds queue, because it's complete now */
		state->pending_commands = cmd->next;
		if (state->pending_commands == NULL)
			state->pending_commands_tail = NULL;
		else
			state->pending_commands->prev = NULL;

		/* Call the completion callback */
		if (cmd->cb)
			cmd->cb(true, radio_response.response_bytes, ret - 3, cmd->cb_data);
		free(cmd);
		read_another = true;

	} while (read_another);
}

void
rift_s_radio_update(rift_s_radio_state *state, struct os_hid_device *hid)
{
	rift_s_radio_update_at(state, hid, os_monotonic_get_ns());
}

void
rift_s_radio_state_init(rift_s_radio_state *state)
{
	state->command_result_pending = false;
	state->pending_commands = NULL;
	state->pending_commands_tail = NULL;
	state->last_radio_seqnum = -1;
}

void
rift_s_radio_state_clear(rift_s_radio_state *state)
{
	/* Clear and free any pending commands. */
	rift_s_radio_command *head = state->pending_commands;
	while (head != NULL) {
		rift_s_radio_command *prev = head;
		head = prev->next;

		if (prev->cb)
			prev->cb(false, NULL, 0, prev->cb_data);
		free(prev);
	}

	state->pending_commands = state->pending_commands_tail = NULL;
	state->command_result_pending = false;
}

void
rift_s_radio_queue_command(rift_s_radio_state *state,
                           const uint64_t device_id,
                           const uint8_t *cmd_bytes,
                           const int cmd_bytes_len,
                           rift_s_radio_completion_fn cb,
                           void *cb_data)
{
	rift_s_radio_queue_report(state, RIFT_S_RADIO_REPORT_READ, device_id, cmd_bytes, cmd_bytes_len, cb, cb_data);
}

void
rift_s_radio_queue_report(rift_s_radio_state *state,
                          uint8_t report_id,
                          const uint64_t device_id,
                          const uint8_t *cmd_bytes,
                          const int cmd_bytes_len,
                          rift_s_radio_completion_fn cb,
                          void *cb_data)
{
	rift_s_radio_command *cmd = U_TYPED_CALLOC(rift_s_radio_command);
	if (cmd == NULL) {
		if (cb)
			cb(false, NULL, 0, cb_data);
		return;
	}

	if (cmd_bytes_len < 0 || cmd_bytes_len > (int)sizeof(cmd->read_command.cmd_bytes) ||
	    (cmd_bytes_len > 0 && cmd_bytes == NULL)) {
		free(cmd);
		if (cb)
			cb(false, NULL, 0, cb_data);
		return;
	}

	cmd->read_command.cmd = report_id;
	cmd->read_command.device_id = device_id;
	if (cmd_bytes_len > 0)
		memcpy(cmd->read_command.cmd_bytes, cmd_bytes, cmd_bytes_len);
	cmd->cb = cb;
	cmd->cb_data = cb_data;

	/* Append to the pending commands queue. The command itself will be sent by the update() function
	 * when possible */
	if (state->pending_commands_tail == NULL) {
		assert(state->pending_commands == NULL);
		state->pending_commands = state->pending_commands_tail = cmd;
	} else {
		state->pending_commands_tail->next = cmd;
		cmd->prev = state->pending_commands_tail;
		state->pending_commands_tail = cmd;
	}
}

/* The current blocks are ~2-3KB, so this should be enough to read
 * the JSON config: */
#define MAX_JSON_LEN 4096

typedef struct rift_s_radio_json_read_state
{
	rift_s_radio_state *state;
	uint64_t device_id;
	rift_s_radio_completion_fn cb;
	void *cb_data;

	uint32_t cur_offset;
	uint16_t block_len; /* Expected length, from the header */

	uint8_t data[MAX_JSON_LEN + 1];
	uint16_t data_len;

} rift_s_radio_json_read_state;

static void
read_json_cb(bool success, uint8_t *response_bytes, int response_bytes_len, rift_s_radio_json_read_state *json_read)
{
	if (!success) {
		/* Failed, report to the caller */
		goto fail;
	}

	if (response_bytes_len < 5) {
		RIFT_S_WARN("Not enough bytes in radio response - needed 5, got %d\n", response_bytes_len);
		goto fail;
	}

	uint8_t reply_len = response_bytes[4];
	if (reply_len > response_bytes_len - 5) {
		RIFT_S_WARN("Radio response claims %u bytes but carried %d\n", reply_len,
		            response_bytes_len - 5);
		goto fail;
	}
	response_bytes += 5;

	if (json_read->cur_offset == 0) {
		/* Read the header 0u32 0x20                    01 00 62 09 7b 22 67 79 | .{..... ..b.{"gy
		            72 6f 5f 6d 22 3a 5b 2d 30 2e 30 31 34 33 35 36 | ro_m":[-0.014356
		            38 38 36 36 2c 2d 30 2e                         | 8866,-0.
	= len 0x20, 0001 (file type, or sequence number?), 0x0962 = file length */
		if (reply_len < 4) {
			RIFT_S_WARN("Not enough bytes in remote configuration header - needed 4, got %d\n", reply_len);
			goto fail; /* Not enough bytes in header */
		}

		uint16_t file_type = (response_bytes[1] << 8) | response_bytes[0];
		uint16_t block_len = (response_bytes[3] << 8) | response_bytes[2];

		if (file_type != 1) {
			RIFT_S_WARN("Unknown file type in remote configuration header - expected 1, got %d\n",
			            file_type);
			goto fail;
		}
		/* Assert if the MAX_JSON_LEN ever needs expanding */
		assert(block_len <= MAX_JSON_LEN);
		if (block_len > MAX_JSON_LEN) {
			RIFT_S_WARN(
			    "Remote configuration block too long. Please expand the read buffer (needed %u bytes)\n",
			    block_len);
			goto fail;
		}

		json_read->block_len = block_len;
		json_read->cur_offset = 0x4;
	} else {
		uint16_t data_remain = json_read->block_len - json_read->data_len;

		if (reply_len > data_remain)
			reply_len = data_remain; /* Truncate any over-read */

		/* Append these bytes to the buffer */
		memcpy(json_read->data + json_read->data_len, response_bytes, reply_len);
		json_read->data_len += reply_len;
	}

	/* If there is no more to read, then report success to the caller and return */
	if (json_read->data_len >= json_read->block_len) {
		json_read->data[json_read->data_len] = 0;

		if (json_read->cb)
			json_read->cb(true, json_read->data, json_read->data_len, json_read->cb_data);
		free(json_read);
		return;
	}

	/* Otherwise request more data */
	uint8_t read_len = MIN(0x20, json_read->block_len - json_read->data_len);
	uint8_t read_cmd[] = {0x2b, 0x20, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00};

	read_cmd[4] = json_read->cur_offset;
	read_cmd[5] = json_read->cur_offset >> 8;
	read_cmd[6] = json_read->cur_offset >> 16;
	read_cmd[7] = json_read->cur_offset >> 24;
	read_cmd[8] = read_len;

	// Queue failure may synchronously finish and free json_read.
	json_read->cur_offset += read_len;
	rift_s_radio_queue_command(json_read->state, json_read->device_id, read_cmd, sizeof(read_cmd),
	                           (rift_s_radio_completion_fn)read_json_cb, json_read);
	return;

fail:
	if (json_read->cb)
		json_read->cb(false, json_read->data, json_read->data_len, json_read->cb_data);
	free(json_read);
	return;
}

void
rift_s_radio_get_json_block(rift_s_radio_state *state,
                            const uint64_t device_id,
                            rift_s_radio_completion_fn cb,
                            void *cb_data)
{
	/* Configuration JSON block reading */
	rift_s_radio_json_read_state *json_read = U_TYPED_CALLOC(rift_s_radio_json_read_state);
	if (json_read == NULL) {
		if (cb)
			cb(false, NULL, 0, cb_data);
		return;
	}
	/* cmd  = 0x2b  reply_buffer_len = 0x20  timeout(?) = 0x3e8 (=1000) offset = 0u32   len = 0x20 */
	const uint8_t read_cmd[] = {0x2b, 0x20, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00};

	json_read->state = state;
	json_read->device_id = device_id;
	json_read->cb = cb;
	json_read->cb_data = cb_data;

	rift_s_radio_queue_command(state, device_id, read_cmd, sizeof(read_cmd),
	                           (rift_s_radio_completion_fn)read_json_cb, json_read);
}
