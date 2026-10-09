# Copyright 2020-2021, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

add_subdirectory(steamvr_bindings)

add_library(st_ovrd STATIC ovrd_driver.cpp ovrd_interface.h ovrd_pose.hpp ovrd_camera.cpp ovrd_camera.hpp rifts_passthrough_shm.h)

target_include_directories(st_ovrd INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	st_ovrd
	PRIVATE
		xrt-interfaces
		xrt-external-openvr
		aux_math
		aux_generated_bindings_helpers
		ovrd_generated_bindings
	)

# Linux SteamVR's sparse SDF staging images need initialized storage on NVIDIA.
if(XRT_HAVE_VULKAN AND CMAKE_SYSTEM_NAME STREQUAL "Linux" AND XRT_BUILD_DRIVER_RIFT_S AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    add_library(monado_roomsetup_vulkan SHARED ovrd_roomsetup_vulkan.cpp)
    target_include_directories(monado_roomsetup_vulkan PRIVATE ${Vulkan_INCLUDE_DIR})
    target_link_options(monado_roomsetup_vulkan PRIVATE "-Wl,-Bsymbolic")
    set_target_properties(monado_roomsetup_vulkan PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/steamvr-monado/bin/linux64")
    set(ROOMSETUP_LAYER_LIBRARY "${CMAKE_INSTALL_PREFIX}/share/steamvr-monado/bin/linux64/libmonado_roomsetup_vulkan.so")
    configure_file(monado_roomsetup_vulkan.json.in ${CMAKE_CURRENT_BINARY_DIR}/monado_roomsetup_vulkan.json @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/monado_roomsetup_vulkan.json DESTINATION ${CMAKE_INSTALL_DATADIR}/vulkan/implicit_layer.d)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND XRT_BUILD_DRIVER_RIFT_S AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    add_executable(monado-roomsetup-guard ovrd_roomsetup_guard.cpp)
    target_link_libraries(monado-roomsetup-guard PRIVATE xrt-external-openvr aux_util ${CMAKE_DL_LIBS})
    set_target_properties(monado-roomsetup-guard PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/steamvr-monado/bin/linux64")
endif()
