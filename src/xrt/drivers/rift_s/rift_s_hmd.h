/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019-2022 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

/*!
 * @file
 * @brief  Interface to the Oculus Rift S HMD driver code.
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

#pragma once

#include "math/m_imu_3dof.h"
#include "rift_s_optics.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "rift_s.h"

/* Oculus Rift S HMD Internal Interface */
#ifndef RIFT_S_HMD_H
#define RIFT_S_HMD_H

struct rift_s_hmd
{
	struct xrt_device base;

	struct rift_s_system *sys;
	/* HMD config info (belongs to the system, which we have a ref to */
	struct rift_s_hmd_config *config;

	/* Pose tracker provided by the system */
	struct rift_s_tracker *tracker;

	/* Tracking to extend 32-bit HMD time to 64-bit nanoseconds */
	uint32_t last_imu_timestamp32; /* 32-bit µS device timestamp */
	timepoint_ns last_imu_timestamp_ns;

	/* Auxiliary state */
	float temperature;
	bool display_on;

	/* Per-eye projection and lens model. */
	struct rift_s_optics optics[2];

	/* Eye separation in metres. Rift S lenses are fixed, IPD is a rendering setting.
	 * Set by RIFT_S_IPD_MM, live-updated from ~/rift-s-ipd.conf (millimetres). */
	float ipd_m;
	char ipd_path[4096];
	int64_t ipd_next_poll_ns;
};

struct rift_s_hmd *
rift_s_hmd_create(struct rift_s_system *sys, const unsigned char *hmd_serial_no, struct rift_s_hmd_config *config);
void
rift_s_hmd_handle_report(struct rift_s_hmd *hmd, timepoint_ns local_ts, rift_s_hmd_report_t *report);
void
rift_s_hmd_set_proximity(struct rift_s_hmd *hmd, bool prox_sensor);

#endif
