# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

message(STATUS "slam-mer: build adjustment module")

# this module will contain the mapping adjustment and covisibility graph
add_library(covisibility_graph SHARED
  src/covisibility_graph.cpp
  src/factors_noise_model.cpp
)

target_include_directories(covisibility_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/adjustment>
)

target_link_libraries(covisibility_graph
  PUBLIC
    types
    map
    descriptor_model
  PRIVATE
    gtsam
)

if(DEPTH_INFERENCE)
  target_link_libraries(covisibility_graph PRIVATE depth_inference::depth_inference)
endif()

add_library(loop_detection SHARED
  src/loop_detection.cpp
)

target_include_directories(loop_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/adjustment>
  )

target_link_libraries(loop_detection
  PUBLIC
    settings
    covisibility_graph
    pose_estimation
  PRIVATE
    gtsam
    )
if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(loop_detection PRIVATE vpl::visual_place_recognition)
endif()

if(DEPTH_INFERENCE)
  target_link_libraries(loop_detection PRIVATE depth_inference::depth_inference)
endif()



install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/include/adjustment)
