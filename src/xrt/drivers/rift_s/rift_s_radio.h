/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 *
 * The Rift S talks to remote devices via HID radio control reports
 * the need to be serialised and coordinated to work properly.
 *
 */
/*!
 * @file
 * @brief  Oculus Rift S HMD Radio management interface
 *
 * Functions for serialising requests to communicate with
 * Touch controllers over the HMDs wireless radio link,
 * collecting responses and delivering them back via callbacks.
 *
 * Ported from OpenHMD
 *
 * @author Jan Schmidt <jan@centricular.com>
 */

#pragma once

#ifndef RIFT_S_RADIO_H
#define RIFT_S_RADIO_H

#include "os/os_hid.h"
#include "xrt/xrt_defines.h"

#include <stdint.h>

//! Host-side bound for one queued command from first update to response.
//! Device requests embed RIFT_S_RADIO_TIMEOUT_MS (1000 ms); the host waits longer to let the
//! device time out first, then fails the head so the queue cannot wedge forever.
#define RIFT_S_RADIO_CMD_TIMEOUT_MS 1500
//! Bounded sends per command before the head fails. Backoff between sends avoids a STALL storm.
#define RIFT_S_RADIO_MAX_SEND_ATTEMPTS 8
//! Bounded consecutive short/error polls per command before the head fails.
#define RIFT_S_RADIO_MAX_POLL_ERRORS 8
//! Exponential backoff base for send/poll transport errors (doubles to the max below).
#define RIFT_S_RADIO_RETRY_BASE_MS 5
#define RIFT_S_RADIO_RETRY_MAX_MS 100

typedef struct rift_s_radio_command rift_s_radio_command;
typedef struct rift_s_radio_state rift_s_radio_state;

typedef void (*rift_s_radio_completion_fn)(bool success,
                                           uint8_t *response_bytes,
                                           int response_bytes_len,
                                           void *cb_data);

struct rift_s_radio_state
{
	bool command_result_pending;
	//! An ambiguous completion requires a radio reset before any following command.
	bool recovery_required;
	int last_radio_seqnum;

	rift_s_radio_command *pending_commands;
	rift_s_radio_command *pending_commands_tail;
};

void
rift_s_radio_state_init(rift_s_radio_state *state);
void
rift_s_radio_state_clear(rift_s_radio_state *state);

void
rift_s_radio_update(rift_s_radio_state *state, struct os_hid_device *hid);
//! Same as rift_s_radio_update with an explicit monotonic now (ns) for tests.
//! Production passes os_monotonic_get_ns(); tests pass fake time.
void
rift_s_radio_update_at(rift_s_radio_state *state, struct os_hid_device *hid, int64_t now_ns);
void
rift_s_radio_queue_command(rift_s_radio_state *state,
                           const uint64_t device_id,
                           const uint8_t *cmd_bytes,
                           const int cmd_bytes_len,
                           rift_s_radio_completion_fn cb,
                           void *cb_data);
//! Like rift_s_radio_queue_command, but sent with an explicit report id (0x13 writes a register).
void
rift_s_radio_queue_report(rift_s_radio_state *state,
                          uint8_t report_id,
                          const uint64_t device_id,
                          const uint8_t *cmd_bytes,
                          const int cmd_bytes_len,
                          rift_s_radio_completion_fn cb,
                          void *cb_data);
void
rift_s_radio_get_json_block(rift_s_radio_state *state,
                            const uint64_t device_id,
                            rift_s_radio_completion_fn cb,
                            void *cb_data);
#endif
