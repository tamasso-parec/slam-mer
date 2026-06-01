# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

# ---------------------------------------------------------------------------
# define the library for the point types
add_library(types
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/point3d.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/point2d.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pose.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vector.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/set.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dictionary.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/hash.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pair.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/queue.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/visualization.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/time_stamp.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/id.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/array.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mutex.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/translation.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rotation.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/match.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/output.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fail.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/image.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/image_io.hpp)

  target_include_directories(types
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(types
  INTERFACE
    Eigen3::Eigen
    PNG::PNG
    JPEG::JPEG)

if (DEPTH_INFERENCE)
  target_link_libraries(types
    INTERFACE
      depth_inference::depth_inference)
endif()

if (VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(types
    INTERFACE
      vpl::visual_place_recognition)
endif()

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/include/types)
