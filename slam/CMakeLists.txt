# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

message(STATUS "slam-mer: build slam module")

# ---------------------------------------------------------------------------
# define the slam settings library
add_library(settings SHARED src/settings.cpp)
target_include_directories(settings
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/slam>
)
target_link_libraries(settings
PUBLIC
  image_processing_interface # needs interface, to avoid cycle
  types
  descriptor_model
  Eigen3::Eigen
PRIVATE
  yaml-cpp
)
if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(settings PUBLIC vpl::visual_place_recognition)
endif()

# ---------------------------------------------------------------------------
# define the sensor data library
add_library(sensor_data SHARED src/sensor_data.cpp)
set_target_properties(sensor_data PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(sensor_data
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/slam>
    )
target_link_libraries(sensor_data
  PUBLIC
    settings
    types
    log
    utils
    Eigen3::Eigen
  PRIVATE
    yaml-cpp
    )

if(DEPTH_INFERENCE)
  target_link_libraries(sensor_data PUBLIC depth_inference::depth_inference)
endif()

if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(sensor_data PRIVATE vpl::visual_place_recognition)
endif()

# ---------------------------------------------------------------------------
# define the slam library
add_library(slam STATIC src/slam.cpp)
target_include_directories(slam
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/slam>
    )

target_link_libraries(slam
  PUBLIC
    map
  PRIVATE
    settings
    localization
    frame
    loop_detection
    utils
    gtsam
    )
if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(slam PRIVATE vpl::visual_place_recognition)
endif()
if(DEPTH_INFERENCE)
  target_link_libraries(slam PRIVATE depth_inference::depth_inference)
endif()

# ---------------------------------------------------------------------------
# install the slam library
install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/slam)
