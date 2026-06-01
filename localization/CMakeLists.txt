# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

message(STATUS "slam-mer: build localization module")

list(APPEND CMAKE_MESSAGE_INDENT "  => Localization Module: ")

add_library(localization SHARED
  src/localization.cpp
  )
target_include_directories(localization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/localization>
    )


target_link_libraries(localization
  PUBLIC
    types
    settings
    image_processing
    pose_estimation
    registration
    map
    covisibility_graph
    kl_divergence
  PRIVATE
    gtsam
    )
if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(localization PUBLIC vpl::visual_place_recognition)
endif()


if(DEPTH_INFERENCE)
  target_link_libraries(localization PRIVATE depth_inference::depth_inference)
endif()



add_library(frame SHARED
  src/frame.cpp
  )

target_include_directories(frame
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/localization>
    )

target_link_libraries(frame
  PUBLIC
    types
    image_processing
    sensor_data
  )

if(DEPTH_INFERENCE)
  target_link_libraries(frame PRIVATE depth_inference::depth_inference)
endif()

if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(frame PUBLIC vpl::visual_place_recognition)
endif()

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/include/localization)
