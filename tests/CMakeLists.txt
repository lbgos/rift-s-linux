# Copyright 2018-2024, Collabora, Ltd.
# Copyright 2024-2026, NVIDIA CORPORATION.
#
# SPDX-License-Identifier: BSL-1.0

set(tests
    tests_cxx_wrappers
    tests_deque
    tests_generic_callbacks
    tests_glue_device
    tests_history_buf
    tests_id_ringbuffer
    tests_json
    tests_lowpass_float
    tests_lowpass_integer
    tests_pacing
    tests_quatexpmap
    tests_quat_change_of_basis
    tests_quat_swing_twist
    tests_rational
    tests_relation_chain
    tests_vector
    tests_worker
    tests_pose
    tests_vec3_angle
	)
if(XRT_HAVE_D3D11)
	list(APPEND tests tests_aux_d3d_d3d11 tests_comp_client_d3d11)
endif()
if(XRT_HAVE_D3D12)
	list(APPEND tests tests_aux_d3d_d3d12 tests_comp_client_d3d12)
endif()
if(XRT_HAVE_VULKAN)
	list(APPEND tests tests_comp_client_vulkan tests_uv_to_tangent)
endif()
if(XRT_FEATURE_OPENXR)
	list(APPEND tests tests_input_transform)
endif()
if(XRT_HAVE_OPENGL
   AND XRT_HAVE_OPENGL_GLX
   AND XRT_HAVE_SDL2
   AND (WIN32 OR VK_USE_PLATFORM_XLIB_XRANDR_EXT)
	)
	set(_have_opengl_test ON)
	list(APPEND tests tests_comp_client_opengl)
endif()
if(XRT_BUILD_DRIVER_HANDTRACKING)
	list(APPEND tests tests_levenbergmarquardt)
endif()

foreach(testname ${tests})
	add_executable(${testname} ${testname}.cpp)
	target_link_libraries(${testname} PRIVATE xrt-external-catch2)
	target_link_libraries(${testname} PRIVATE aux_util)
	add_test(NAME ${testname} COMMAND ${testname} --success --allow-running-no-tests)
	set_target_properties(${testname} PROPERTIES FOLDER monado_tests)
endforeach()

# For tests that require more than just aux_util, link those other libs down here.

target_link_libraries(tests_cxx_wrappers PRIVATE xrt-interfaces)
target_link_libraries(tests_glue_device PRIVATE xrt-interfaces aux_glue)
target_link_libraries(tests_history_buf PRIVATE aux_math)
target_link_libraries(tests_lowpass_float PRIVATE aux_math)
target_link_libraries(tests_lowpass_integer PRIVATE aux_math)
target_link_libraries(tests_quatexpmap PRIVATE aux_math)
target_link_libraries(tests_rational PRIVATE aux_math)
target_link_libraries(tests_relation_chain PRIVATE aux_math)
target_link_libraries(tests_pose PRIVATE aux_math)
target_link_libraries(tests_quat_change_of_basis PRIVATE aux_math)
target_link_libraries(tests_quat_swing_twist PRIVATE aux_math)
target_link_libraries(tests_vec3_angle PRIVATE aux_math)

if(XRT_BUILD_DRIVER_HANDTRACKING)
	target_link_libraries(
		tests_levenbergmarquardt
		PRIVATE
			aux_math
			t_ht_mercury_includes
			t_ht_mercury_kine_lm_includes
			t_ht_mercury
			t_ht_mercury_kine_lm
		)
endif()

if(XRT_HAVE_D3D11)
	target_link_libraries(tests_aux_d3d_d3d11 PRIVATE aux_d3d)
	target_link_libraries(tests_comp_client_d3d11 PRIVATE comp_client comp_mock)
endif()

if(XRT_HAVE_D3D12)
	target_link_libraries(tests_aux_d3d_d3d12 PRIVATE aux_d3d)
	target_link_libraries(tests_comp_client_d3d12 PRIVATE comp_client comp_mock)
endif()

if(XRT_HAVE_VULKAN)
	target_link_libraries(
		tests_comp_client_vulkan PRIVATE comp_client comp_mock comp_util aux_vk
		)
	target_link_libraries(tests_uv_to_tangent PRIVATE comp_render)
endif()

if(XRT_FEATURE_OPENXR)
	target_link_libraries(
		tests_input_transform PRIVATE st_oxr xrt-interfaces xrt-external-openxr
		)
endif()
if(_have_opengl_test)
	target_link_libraries(
		tests_comp_client_opengl PRIVATE comp_client comp_mock aux_ogl SDL2::SDL2
		)
	target_include_directories(tests_comp_client_opengl PRIVATE SDL2::SDL2)
endif()

if(XRT_HAVE_VULKAN AND XRT_HAVE_D3D11)
	target_link_libraries(tests_aux_d3d_d3d11 PRIVATE comp_util aux_vk)
endif()

if(XRT_HAVE_VULKAN AND XRT_HAVE_D3D12)
	target_link_libraries(tests_aux_d3d_d3d12 PRIVATE comp_util aux_vk)
endif()

if(XRT_BUILD_DRIVER_RIFT_S)
	add_executable(
		tests_rift_s_position_filter tests_rift_s_position_filter.cpp
		../src/xrt/drivers/rift_s/rift_s_position_filter.c
		)
	target_include_directories(tests_rift_s_position_filter PRIVATE ../src/xrt/drivers)
	target_link_libraries(tests_rift_s_position_filter PRIVATE xrt-external-catch2 aux_util)
	add_test(NAME tests_rift_s_position_filter COMMAND tests_rift_s_position_filter)
	set_target_properties(tests_rift_s_position_filter PROPERTIES FOLDER monado_tests)

	add_executable(tests_rift_s_optics tests_rift_s_optics.cpp ../src/xrt/drivers/rift_s/rift_s_optics.c)
	target_include_directories(tests_rift_s_optics PRIVATE ../src/xrt/drivers)
	target_link_libraries(tests_rift_s_optics PRIVATE xrt-external-catch2 aux_util)
	add_test(NAME tests_rift_s_optics COMMAND tests_rift_s_optics)
	set_target_properties(tests_rift_s_optics PROPERTIES FOLDER monado_tests)
endif()

if(XRT_MODULE_CONSTELLATION_TRACKING)
	add_executable(tests_constellation tests_constellation.cpp)
	target_include_directories(tests_constellation PRIVATE ../src/xrt/tracking/constellation)
	target_link_libraries(tests_constellation PRIVATE xrt-external-catch2 constellation_tracker aux_math)
	add_test(NAME tests_constellation COMMAND tests_constellation)
endif()

if(XRT_BUILD_DRIVER_RIFT_S)
	add_executable(tests_rift_s_tracking tests_rift_s_tracking.cpp)
	target_include_directories(tests_rift_s_tracking PRIVATE ../src/xrt/drivers/rift_s)
	target_link_libraries(tests_rift_s_tracking PRIVATE xrt-external-catch2 drv_rift_s aux_math aux_tracking)
	add_test(NAME tests_rift_s_tracking COMMAND tests_rift_s_tracking)
endif()

if(XRT_BUILD_DRIVER_RIFT_S AND XRT_FEATURE_STEAMVR_PLUGIN)
	target_include_directories(tests_rift_s_tracking PRIVATE ../src/xrt/state_trackers/steamvr_drv)
	target_link_libraries(tests_rift_s_tracking PRIVATE xrt-external-openvr)
	target_compile_definitions(tests_rift_s_tracking PRIVATE TEST_RIFT_S_STEAMVR_POSE)
endif()

if(XRT_BUILD_DRIVER_RIFT_S)
	add_executable(tests_rift_s_metadata tests_rift_s_metadata.cpp)
	target_include_directories(tests_rift_s_metadata PRIVATE ../src/xrt/drivers/rift_s)
	target_link_libraries(tests_rift_s_metadata PRIVATE xrt-external-catch2 drv_rift_s aux_math)
	add_test(NAME tests_rift_s_metadata COMMAND tests_rift_s_metadata)
endif()

if(XRT_MODULE_CONSTELLATION_TRACKING)
	add_executable(tests_constellation_constrained tests_constellation_constrained.cpp)
	target_include_directories(tests_constellation_constrained PRIVATE ../src/xrt/tracking/constellation)
	target_link_libraries(tests_constellation_constrained PRIVATE xrt-external-catch2 constellation_tracker aux_math)
	add_test(NAME tests_constellation_constrained COMMAND tests_constellation_constrained)
endif()

if(XRT_BUILD_DRIVER_RIFT_S)
	add_executable(tests_rift_s_passthrough tests_rift_s_passthrough.cpp)
	target_include_directories(tests_rift_s_passthrough PRIVATE ../src/xrt/drivers/rift_s)
	target_link_libraries(tests_rift_s_passthrough PRIVATE xrt-external-catch2 drv_rift_s aux_math)
	add_test(NAME tests_rift_s_passthrough COMMAND tests_rift_s_passthrough)
endif()

if(XRT_FEATURE_STEAMVR_PLUGIN)
	add_executable(tests_steamvr_camera tests_steamvr_camera.cpp ../src/xrt/state_trackers/steamvr_drv/ovrd_camera.cpp)
	target_include_directories(tests_steamvr_camera PRIVATE ../src/xrt/state_trackers/steamvr_drv)
	target_link_libraries(tests_steamvr_camera PRIVATE xrt-external-catch2 xrt-external-openvr aux_math aux_util)
	target_compile_definitions(tests_steamvr_camera PRIVATE RIFTS_PASSTHROUGH_SHM_PATH="${CMAKE_CURRENT_BINARY_DIR}/rifts-passthrough-test.shm")
	add_test(NAME tests_steamvr_camera COMMAND tests_steamvr_camera)
endif()

if(XRT_BUILD_DRIVER_RIFT_S)
	add_executable(tests_rift_s_slam_guard tests_rift_s_slam_guard.cpp ../src/xrt/drivers/rift_s/rift_s_slam_guard.c)
	target_include_directories(tests_rift_s_slam_guard PRIVATE ../src/xrt/drivers)
	target_link_libraries(tests_rift_s_slam_guard PRIVATE xrt-external-catch2 aux_math)
	add_test(NAME tests_rift_s_slam_guard COMMAND tests_rift_s_slam_guard)
	set_target_properties(tests_rift_s_slam_guard PROPERTIES FOLDER monado_tests)
endif()

if(XRT_FEATURE_SLAM)
	add_executable(tests_slam_recovery tests_slam_recovery.cpp)
	target_link_libraries(tests_slam_recovery PRIVATE aux_tracking aux_util xrt-interfaces)
	add_test(NAME tests_slam_recovery COMMAND tests_slam_recovery)
	set_tests_properties(tests_slam_recovery PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 45)
	add_executable(tests_slam_restart tests_slam_restart.cpp)
	target_link_libraries(tests_slam_restart PRIVATE aux_tracking aux_util xrt-interfaces)
	find_package(TBB QUIET)
	if(TARGET TBB::tbb)
		target_link_libraries(tests_slam_restart PRIVATE TBB::tbb)
		target_compile_definitions(tests_slam_restart PRIVATE SLAM_TEST_HAVE_TBB)
	endif()
	add_test(NAME tests_slam_restart COMMAND tests_slam_restart)
	set_tests_properties(tests_slam_restart PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 45)
endif()

if(TARGET monado_roomsetup_vulkan)
	add_executable(tests_steamvr_roomsetup_vulkan tests_steamvr_roomsetup_vulkan.cpp)
	target_include_directories(tests_steamvr_roomsetup_vulkan PRIVATE ${Vulkan_INCLUDE_DIR})
	target_link_libraries(tests_steamvr_roomsetup_vulkan PRIVATE xrt-external-catch2)
	add_test(NAME tests_steamvr_roomsetup_vulkan COMMAND tests_steamvr_roomsetup_vulkan)
endif()

if(TARGET monado-roomsetup-guard)
	add_executable(tests_steamvr_roomsetup_guard tests_steamvr_roomsetup_guard.cpp)
	target_link_libraries(tests_steamvr_roomsetup_guard PRIVATE xrt-external-catch2 xrt-external-openvr aux_util ${CMAKE_DL_LIBS})
	add_test(NAME tests_steamvr_roomsetup_guard COMMAND tests_steamvr_roomsetup_guard)
endif()

if(XRT_BUILD_DRIVER_RIFT_S)
	add_executable(tests_rift_s_world_anchor tests_rift_s_world_anchor.cpp)
	target_include_directories(tests_rift_s_world_anchor PRIVATE ../src/xrt/drivers/rift_s ${OpenCV_INCLUDE_DIRS})
	target_link_libraries(tests_rift_s_world_anchor PRIVATE xrt-external-catch2 drv_rift_s aux_math aux_util ${OpenCV_LIBRARIES})
	add_test(NAME tests_rift_s_world_anchor COMMAND tests_rift_s_world_anchor)
endif()

if(XRT_BUILD_DRIVER_RIFT_S AND XRT_MODULE_CONSTELLATION_TRACKING)
	add_executable(tests_rift_s_controller_geometry tests_rift_s_controller_geometry.cpp)
	target_include_directories(tests_rift_s_controller_geometry PRIVATE ../src/xrt/tracking/constellation)
	target_link_libraries(tests_rift_s_controller_geometry PRIVATE xrt-external-catch2 constellation_tracker aux_math)
	add_test(NAME tests_rift_s_controller_geometry COMMAND tests_rift_s_controller_geometry)

	add_executable(tests_rift_s_controller_capture tests_rift_s_controller_capture.cpp)
	target_include_directories(tests_rift_s_controller_capture PRIVATE ../src/xrt/drivers/rift_s)
	target_link_libraries(tests_rift_s_controller_capture PRIVATE xrt-external-catch2 drv_rift_s aux_math aux_tracking aux_util_sink)
	add_test(NAME tests_rift_s_controller_capture COMMAND tests_rift_s_controller_capture)
endif()
