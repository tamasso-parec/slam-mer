# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

message(STATUS "slam-mer: build map module")

add_library(map
  SHARED
    src/map.cpp
    src/rasterization_helper.cpp
    src/wrapper_visualization.cpp
    src/keyframe.cpp
  )

target_include_directories(map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/map>)

target_link_libraries(map
  PUBLIC
    Eigen3::Eigen
    types
    representation
    settings
    sensor_data)

if(DEPTH_INFERENCE)
  target_link_libraries(map PRIVATE depth_inference::depth_inference)
endif()

if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(map PUBLIC vpl::visual_place_recognition)
endif()

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/map)
