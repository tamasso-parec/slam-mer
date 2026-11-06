# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.20)

# ---------------------------------------------------------------------------
# Export the json file for autocomplete
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

# ---------------------------------------------------------------------------
# Project title and version
# This is a C++ project, a V-SLAM library implemented at MERL
# by P. Miraldo and V. Piedade.
project(slam-mer
    VERSION 1.0
    LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Add the cmake directory to the path
# and include the modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Options)
include(ThirdParty)
# include(ExecuteLink)

# ---------------------------------------------------------------------------
# manage RPATH (this is important to make sure the instalation libraries
# are set correctly). These are standard CMake variables that are used
# to control the behavior of the RPATH settings.
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
get_filename_component(SLAMMER_INSTALL_PREFIX_ABS "${CMAKE_INSTALL_PREFIX}" ABSOLUTE)
set(CMAKE_INSTALL_RPATH
  "${SLAMMER_INSTALL_PREFIX_ABS}/lib"
  "${CMAKE_SOURCE_DIR}/thirdparty/PoseLib/build/install/lib"
  "${CMAKE_SOURCE_DIR}/thirdparty/faiss/build/install/lib"
  "${CMAKE_SOURCE_DIR}/thirdparty/gtsam/build/install/lib"
  "${CMAKE_SOURCE_DIR}/thirdparty/rerun_cpp_sdk/build/install/lib"
  "${CMAKE_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-gpu-1.22.0/lib"
)

if(WITH_ALIKED)
  list(APPEND CMAKE_INSTALL_RPATH
    "${CMAKE_SOURCE_DIR}/utils/Light_Glue_CPP/build/install/lib"
    "${CMAKE_SOURCE_DIR}/thirdparty/libtorch/lib")
endif()

if(VISUAL_PLACE_LOCALIZATION)
  list(APPEND CMAKE_INSTALL_RPATH
    "${CMAKE_SOURCE_DIR}/thirdparty/visual_place_localization/build/install/lib")
endif()

if(DEPTH_INFERENCE)
  list(APPEND CMAKE_INSTALL_RPATH
    "${CMAKE_SOURCE_DIR}/thirdparty/depth_inference/build/install/lib")
endif()

list(REMOVE_DUPLICATES CMAKE_INSTALL_RPATH)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Ensure build-tree binaries can resolve third-party shared libs without env vars.
set(CMAKE_BUILD_RPATH "${CMAKE_INSTALL_RPATH}")

# ---------------------------------------------------------------------------
# Ensure RPATH (not RUNPATH) so transitive deps like GTSAM's libcephes are found
# when loading through shared libraries in the build tree.
# if(UNIX AND NOT APPLE)
  add_link_options(-Wl,--disable-new-dtags)
# endif()


# ---------------------------------------------------------------------------
# Compile options
# Add the fast run comile option
# NOTE: x86-64-v3 causes segmentation fault for some CPUs
add_compile_options(-O3 -fopenmp -mtune=native -march=x86-64-v2)

# Add the c++ standard information
add_compile_options(--std=c++23)
# force colors compilation
add_compile_options(-fdiagnostics-color=always)
# wall for all warnings
add_compile_options(-Wall)
# add the debug information
add_compile_options(-lpthread)
# make sure we use the correct c++
set(CMAKE_CXX_COMPILER g++-14)

# for debug
add_compile_options($<$<CONFIG:Debug>:-DDEBUG=ON>)

if(USE_ASAN)
  # check whether WITH_ALIKED=OFF to prevent conflicts
  if(WITH_ALIKED)
    message(FATAL_ERROR "AddressSanitizer is not compatible with LibTorch (ALIKED, LightGlue). Please set WITH_ALIKED=OFF to use ASan.")
  endif()

  # Global ASan options
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)

  # Shared libraries must be PIC
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)

  # Disable PIE for executables (to avoid ASan shadow overlap)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")
  set(CMAKE_EXE_COMPILE_OPTIONS "${CMAKE_EXE_COMPILE_OPTIONS} -fno-pie")
endif()

# ---------------------------------------------------------------------------
# Submodueles of the project implemented at MERL
add_subdirectory(types)               # shared type definitions
add_subdirectory(utils)               # extra modules
add_subdirectory(slam)                # adds the slam module
add_subdirectory(localization)        # localization functions
add_subdirectory(map)                 # adds the map module
add_subdirectory(adjustment)  # local-mapping modules
add_subdirectory(interface)           # visualization tools

# We define the ${LIBS} variable to simplify the process of
# linking the libraries in the examples

set(
  EXPORT_LIBS
    cudaSift
    types
    log
    utils
    image_processing
    image_processing_interface
    pose_estimation
    registration
    kl_divergence
    descriptor_model
    representation
    settings
    localization
    frame
    map
    covisibility_graph
    loop_detection
    sensor_data
    slam
    interface
    )

# ---------------------------------------------------------------------------
# Install the libraries
# ---------------------------------------------------------------------------
# First creates the install target
install(
  TARGETS
    ${EXPORT_LIBS}
  EXPORT SlamMerTargets
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  )

# Then install the targets in the typical instalation directory,
# i.e., lib/cmake/slamMe. Notice we are using the modules perfix slamMe::
# to avoid conflicts with other libraries.
install(EXPORT SlamMerTargets
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/SlamMer
    NAMESPACE SlamMer:: )

export(EXPORT SlamMerTargets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/SlamMerTargets.cmake)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/SlamMerConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/SlamMerConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/SlamMer
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/SlamMerConfig.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/SlamMer)
