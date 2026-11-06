# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

message(STATUS "slam-mer: build utils module")

# ---------------------------------------------------------------------------
# The easiest way to add the cuda sift library is to
# create a subdirectory and add the library there. Notice
# most of the code in the cudaSift was written by Marten Bjorkman
# from KTH, Sweden, with some adaptations by the authors of this
# project.
add_subdirectory(cudaSift)
message(STATUS "slam-mer: CUDASIFT_INCLUDE_DIRS=${CUDASIFT_INCLUDE_DIRS}")

# ---------------------------------------------------------------------------
# Build ALIKED from utils/aliked when enabled
if(WITH_ALIKED)
  # Respect CMAKE_CUDA_HOST_COMPILER / CUDAHOSTCXX selected by the caller.
  set(LIBTORCH_DIR "${CMAKE_SOURCE_DIR}/thirdparty/libtorch")
  list(PREPEND CMAKE_PREFIX_PATH "${LIBTORCH_DIR}")
  find_package(Torch REQUIRED)
  message(STATUS "slam-mer: LibTorch configured for ALIKED")

  add_subdirectory(aliked)
  message(STATUS "slam-mer: ALIKED configured from utils/aliked")
endif()

# ---------------------------------------------------------------------------
# define the library for logging
add_library(log
  SHARED
    src/log.cpp)
target_include_directories(log
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/utils>)
target_link_libraries(log
  PUBLIC
    types)

# ---------------------------------------------------------------------------
# define the general utils libraries
add_library(utils
  SHARED
    src/debug_data.cpp)
target_include_directories(utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../slam/include>
    $<INSTALL_INTERFACE:include/utils>)

target_link_libraries(utils
  PUBLIC
    types
    settings
    log
  )

if(WITH_ALIKED)
  # Add ALIKED support if enabled.
  target_compile_definitions(utils PRIVATE WITH_ALIKED)
  target_link_libraries(utils
    PRIVATE
      lightglue::LightGlue_lib
      ${TORCH_LIBRARIES})
endif()

if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(utils
    PRIVATE
      vpl::visual_place_recognition)
endif()


# ---------------------------------------------------------------------------
# define the library for the image processing
add_library(image_processing
  SHARED
    src/image_processing.cpp
    )

target_include_directories(image_processing
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/utils>
    /usr/local/cuda/include)

target_link_libraries(image_processing
  PUBLIC
    types
    cudaSift
    log
    settings
    faiss
    OpenMP::OpenMP_CXX)

# Add ALIKED support if enabled.
if(WITH_ALIKED)
  target_compile_definitions(image_processing PRIVATE WITH_ALIKED)
  target_link_libraries(image_processing
    PRIVATE
      lightglue::LightGlue_lib
      ${TORCH_LIBRARIES})
endif()

  add_library(image_processing_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/image_processing.hpp)

  target_include_directories(image_processing_interface
    INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/utils>)

if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(image_processing PRIVATE vpl::visual_place_recognition)
endif()

# ---------------------------------------------------------------------------
# define the library for pose estimation
add_library(pose_estimation
  SHARED
    src/pose_estimation.cpp)
target_include_directories(pose_estimation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/utils>)
target_link_libraries(pose_estimation
  PUBLIC
    types
    settings
    sensor_data
    PoseLib::PoseLib
    )

if(DEPTH_INFERENCE)
  target_link_libraries(pose_estimation PRIVATE depth_inference::depth_inference)
endif()

if(VISUAL_PLACE_LOCALIZATION)
  target_link_libraries(pose_estimation PRIVATE vpl::visual_place_recognition)
endif()

# ---------------------------------------------------------------------------
# define the library for registration
add_library(registration
  SHARED
    src/registration.cpp)
target_include_directories(registration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/utils>)
target_link_libraries(registration
  PUBLIC
    types
    Eigen3::Eigen)

# ---------------------------------------------------------------------------
# define the library for kl_divergence
add_library(kl_divergence
  SHARED
    src/kl_divergence.cpp)
target_include_directories(kl_divergence
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/utils>)

target_link_libraries(kl_divergence PUBLIC types)

# ---------------------------------------------------------------------------
# define the library for representation
add_library(representation
  SHARED src/representation.cpp)

target_include_directories(representation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/utils>
  )

target_link_libraries(representation
  PUBLIC types)

# ---------------------------------------------------------------------------
# define the library for descriptor_model
add_library(descriptor_model SHARED src/descriptor_model.cpp)

target_include_directories(descriptor_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/utils>
  )

target_link_libraries(descriptor_model PUBLIC types)


install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/include/utils)
