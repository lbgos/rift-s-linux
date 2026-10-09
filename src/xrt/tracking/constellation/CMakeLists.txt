# Copyright 2024 Jan Schmidt
# SPDX-License-Identifier: BSL-1.0

add_library(
	constellation_tracker STATIC
	t_constellation_tracking.c
	t_led_models.c
	internal/blobwatch.c
	internal/blobwatch.h
	internal/correspondence_search.c
	internal/correspondence_search.h
	internal/constrained_pose.cpp
	internal/constrained_pose.h
	internal/debug_draw.cpp
	internal/debug_draw.h
	internal/lambdatwist/cubic.h
	internal/lambdatwist/eig3x3known0.h
	internal/lambdatwist/lambdatwist_p3p.h
	internal/lambdatwist/mat.h
	internal/lambdatwist/quadratic.h
	internal/lambdatwist/refine_lambda.h
	internal/lambdatwist/vec.h
	internal/pose_metrics.c
	internal/pose_metrics.h
	internal/ransac_pnp.cpp
	internal/ransac_pnp.h
	internal/sample.c
	internal/sample.h
	)

target_include_directories(constellation_tracker PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(
	constellation_tracker
	PRIVATE
		xrt-interfaces
		Eigen3::Eigen
		aux_os
		aux_util
		aux_math
		${OpenCV_LIBRARIES}
	)
