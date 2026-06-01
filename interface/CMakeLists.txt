# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

message(STATUS "slam-mer: build interface module")

set(
  SOURCES
    src/interface.cpp
    src/interface_rerun.cpp
    src/messages.cpp
    src/camera_location.cpp
    src/covisibility.cpp
    src/mappoints.cpp
    src/cells.cpp
    src/image_plot.cpp
    src/helper.cpp
    src/ply_writer.cpp
)
add_library(interface STATIC ${SOURCES})
target_include_directories(interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/visualization>
  )
# in the website, they suggest use PRIVATE
target_link_libraries(interface
  PRIVATE
    utils
    types
    log
    settings
    Eigen3::Eigen
    )

target_link_libraries(interface PRIVATE rerun_sdk)

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/visualization)
