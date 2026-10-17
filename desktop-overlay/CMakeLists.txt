# Copyright 2026, lbgos
# SPDX-License-Identifier: BSL-1.0
#
# rifts-desktop: SteamVR dashboard tabs showing the Linux desktop. Builds as part
# of the Monado tree or standalone: cmake -S desktop-overlay -B build.

cmake_minimum_required(VERSION 3.16)
project(rifts-desktop LANGUAGES C CXX)

include(GNUInstallDirs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(RIFTS_DESKTOP_DEPS QUIET IMPORTED_TARGET wayland-client libpipewire-0.3 libsystemd)
pkg_get_variable(RIFTS_WAYLAND_SCANNER wayland-scanner wayland_scanner)
pkg_get_variable(RIFTS_WAYLAND_PROTOCOLS wayland-protocols pkgdatadir)
find_package(Vulkan QUIET)
find_package(PlasmaWaylandProtocols CONFIG QUIET)
set(RIFTS_PLASMA_WAYLAND_PROTOCOLS_DIR "${PLASMA_WAYLAND_PROTOCOLS_DIR}"
    CACHE PATH "Directory with plasma-wayland-protocols XML files")

set(_missing)
foreach(_dep RIFTS_DESKTOP_DEPS_FOUND Vulkan_FOUND RIFTS_WAYLAND_SCANNER RIFTS_WAYLAND_PROTOCOLS
             RIFTS_PLASMA_WAYLAND_PROTOCOLS_DIR)
	if(NOT ${_dep})
		list(APPEND _missing ${_dep})
	endif()
endforeach()
if(_missing)
	set(_text "rifts-desktop needs wayland, pipewire, libsystemd, vulkan, wayland-protocols and plasma-wayland-protocols (missing: ${_missing})")
	if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
		message(FATAL_ERROR "${_text}")
	endif()
	message(WARNING "${_text}; not building it")
	return()
endif()

set(_protocols
    "${RIFTS_PLASMA_WAYLAND_PROTOCOLS_DIR}/zkde-screencast-unstable-v1.xml"
    "${RIFTS_PLASMA_WAYLAND_PROTOCOLS_DIR}/fake-input.xml"
    "${RIFTS_PLASMA_WAYLAND_PROTOCOLS_DIR}/kde-output-order-v1.xml"
    "${RIFTS_WAYLAND_PROTOCOLS}/unstable/xdg-output/xdg-output-unstable-v1.xml")
set(_generated)
foreach(_xml ${_protocols})
	get_filename_component(_base "${_xml}" NAME_WE)
	set(_header "${CMAKE_CURRENT_BINARY_DIR}/${_base}-client-protocol.h")
	set(_code "${CMAKE_CURRENT_BINARY_DIR}/${_base}-protocol.c")
	add_custom_command(
		OUTPUT "${_header}" "${_code}"
		COMMAND "${RIFTS_WAYLAND_SCANNER}" client-header "${_xml}" "${_header}"
		COMMAND "${RIFTS_WAYLAND_SCANNER}" private-code "${_xml}" "${_code}"
		DEPENDS "${_xml}"
		VERBATIM)
	list(APPEND _generated "${_header}" "${_code}")
endforeach()

set(RIFTS_DESKTOP_MANIFEST "${CMAKE_INSTALL_FULL_DATADIR}/rifts-desktop/rifts-desktop.vrmanifest")
# Relocated installs find the manifest relative to the binary.
file(RELATIVE_PATH RIFTS_DESKTOP_MANIFEST_FROM_BIN "${CMAKE_INSTALL_FULL_BINDIR}" "${RIFTS_DESKTOP_MANIFEST}")
add_executable(rifts-desktop
	src/capture.cpp
	src/gpu.cpp
	src/kwin.cpp
	src/main.cpp
	src/portal.cpp
	src/vr.cpp
	${_generated})
set_target_properties(rifts-desktop PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)
target_include_directories(rifts-desktop PRIVATE
	"${CMAKE_CURRENT_BINARY_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}/../src/external/openvr_includes")
target_compile_definitions(rifts-desktop PRIVATE RIFTS_DESKTOP_MANIFEST="${RIFTS_DESKTOP_MANIFEST}"
                                                  RIFTS_DESKTOP_MANIFEST_FROM_BIN="${RIFTS_DESKTOP_MANIFEST_FROM_BIN}")
target_link_libraries(rifts-desktop PRIVATE PkgConfig::RIFTS_DESKTOP_DEPS Vulkan::Vulkan ${CMAKE_DL_LIBS})

configure_file(rifts-desktop.desktop.in rifts-desktop.desktop @ONLY)
configure_file(rifts-desktop.vrmanifest.in rifts-desktop.vrmanifest @ONLY)
install(TARGETS rifts-desktop RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
# KWin grants its screencast and fake-input protocols to the executable named in this file.
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rifts-desktop.desktop" DESTINATION "${CMAKE_INSTALL_DATADIR}/applications")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rifts-desktop.vrmanifest" DESTINATION "${CMAKE_INSTALL_DATADIR}/rifts-desktop")
