# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

## to generate a compile_commands.json file
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS)

# Some building options that can be set by the user.
# add the option to build
set(WITH_CUDA ON CACHE BOOL "Build with CUDA support")
if(WITH_CUDA)
  message(STATUS "slam-mer: WITH_CUDA=ON")
  add_definitions(-DWITH_CUDA)
endif()

# Select which transformation template instantiations are compiled.
set(SLAM_TRANSFORMATION_INSTANTIATIONS ALL CACHE STRING
    "Transformation template instantiations to compile: ALL, POSE3, SIMILARITY3")
set_property(CACHE SLAM_TRANSFORMATION_INSTANTIATIONS
    PROPERTY STRINGS ALL POSE3 SIMILARITY3)
string(TOUPPER
    "${SLAM_TRANSFORMATION_INSTANTIATIONS}"
    _slam_transformation_instantiations)
if(NOT _slam_transformation_instantiations MATCHES "^(ALL|POSE3|SIMILARITY3)$")
  message(FATAL_ERROR
      "SLAM_TRANSFORMATION_INSTANTIATIONS must be ALL, POSE3, or SIMILARITY3")
endif()
set(SLAM_TRANSFORMATION_INSTANTIATIONS
    "${_slam_transformation_instantiations}"
    CACHE STRING
    "Transformation template instantiations to compile: ALL, POSE3, SIMILARITY3"
    FORCE)

if(SLAM_TRANSFORMATION_INSTANTIATIONS STREQUAL "ALL")
  message(STATUS "slam-mer: SLAM_TRANSFORMATION_INSTANTIATIONS=ALL")
  add_definitions(-DSLAM_MER_COMPILE_POSE3 -DSLAM_MER_COMPILE_SIMILARITY3)
elseif(SLAM_TRANSFORMATION_INSTANTIATIONS STREQUAL "POSE3")
  message(STATUS "slam-mer: SLAM_TRANSFORMATION_INSTANTIATIONS=POSE3")
  add_definitions(-DSLAM_MER_COMPILE_POSE3)
elseif(SLAM_TRANSFORMATION_INSTANTIATIONS STREQUAL "SIMILARITY3")
  message(STATUS "slam-mer: SLAM_TRANSFORMATION_INSTANTIATIONS=SIMILARITY3")
  add_definitions(-DSLAM_MER_COMPILE_SIMILARITY3)
endif()
unset(_slam_transformation_instantiations)


# This is a very important option, that allows the user
# to run the clang-format to the source code, to make sure
# that the code is well formatted according to what was defined
# by the authors of this framework.
# set(CLANG_RUN ON CACHE STRING "Option to run the clang-format")
# if(CLANG_RUN)
#   message("OPTIONS: To run the clang-format on all the hpp and cpp files")
#   message("WARNING: CLANG_RUN option is still not working....")
# endif()

# Not showing all the warning related to the
# deprecated functionalities
set(HIDE_WARNINGS ON CACHE BOOL "Ignoring all the deprecated warnings")
if(HIDE_WARNINGS)
  message(STATUS "slam-mer: HIDE_WARNINGS=ON")
  add_compile_options(-Wno-deprecated)
else()
  add_compile_options(-Wall)
endif()

# Hide noisy NVCC warnings from CUDA targets (e.g., ALIKED/cudaSift).
set(HIDE_CUDA_NVCC_WARNINGS ON CACHE BOOL
    "Hide noisy NVCC warnings for CUDA targets (ALIKED/cudaSift)")
if(HIDE_CUDA_NVCC_WARNINGS)
  message(STATUS "slam-mer: HIDE_CUDA_NVCC_WARNINGS=ON")
endif()

# Define the version of the CUDA to be used
# Tested CUDA_VERSION: 12.1, 12.2, 12.3, 12.4, 12.5. We suggest using
# environment module to have different versions of CUDA and C++ if needed.
if(DEFINED ENV{CUDA_VERSION})
  set(CUDA_VERSION $ENV{CUDA_VERSION} CACHE STRING "CUDA version to be used")
else()
  set(CUDA_VERSION 12.8 CACHE STRING "CUDA version to be used")
endif()

# To speed up the compilation, and avoid to libraries for cuda
# architectures that will not be used, check and set what is the
# Cuda architecture that is used by your GPU, and set the following
# variable: (set default to 86, which is the architecture of latest gpus)
if(DEFINED ENV{CUDA_ARCH})
  set(CUDA_ARCH $ENV{CUDA_ARCH} CACHE STRING "CUDA architecture to be used")
else()
  set(CUDA_ARCH 86 CACHE STRING "CUDA architecture to be used")
endif()

# Option to build with ALIKED support (requires CUDA and PyTorch)
set(WITH_ALIKED ON CACHE BOOL "Build with ALIKED support")
if(WITH_ALIKED)
  # Check CUDA dependency
  if(NOT WITH_CUDA)
    message(WARNING
      "ALIKED requires CUDA support. Enabling WITH_CUDA automatically.")
    set(WITH_CUDA ON CACHE BOOL "Build with CUDA support" FORCE)
    add_definitions(-DWITH_CUDA)
  endif()

  # Check PyTorch availability
  set(LIBTORCH_DIR
    "${CMAKE_SOURCE_DIR}/thirdparty/libtorch")
  if(NOT EXISTS "${LIBTORCH_DIR}")
    message(FATAL_ERROR
      "ALIKED requires PyTorch (libtorch). "
      "Please install it in ${LIBTORCH_DIR}")
  endif()

  message(STATUS "slam-mer: WITH_ALIKED=ON (with CUDA and PyTorch)")
  add_definitions(-DWITH_ALIKED)

  # Torch uses TORCH_CUDA_ARCH_LIST, not CMAKE_CUDA_ARCHITECTURES.
  if(DEFINED CUDA_ARCH)
    # Convert e.g. 86 -> 8.6 for Torch.
    string(REGEX REPLACE "^([0-9])([0-9])$" "\\1.\\2" _torch_arch "${CUDA_ARCH}")
    set(TORCH_CUDA_ARCH_LIST "${_torch_arch}" CACHE STRING "Torch CUDA arch list" FORCE)
    unset(_torch_arch)
  endif()
  unset(CMAKE_CUDA_ARCHITECTURES CACHE)
  unset(CMAKE_CUDA_ARCHITECTURES)

  find_package(CUDA REQUIRED)

endif()

# Option to build with Visual Place Localization support
# Default OFF to avoid pulling onnxruntime from the VPL dependency unless explicitly requested.
set(VISUAL_PLACE_LOCALIZATION OFF CACHE BOOL "Build with Visual Place Localization support")
if(VISUAL_PLACE_LOCALIZATION)
  message(STATUS "slam-mer: VISUAL_PLACE_LOCALIZATION=ON")
  add_definitions(-DVISUAL_PLACE_LOCALIZATION)
endif()

# Option to build with Depth Inference support
set(DEPTH_INFERENCE OFF CACHE BOOL "Build with Depth Inference support")
if(DEPTH_INFERENCE)
  message(STATUS "slam-mer: DEPTH_INFERENCE=ON")
  add_definitions(-DDEPTH_INFERENCE)
endif()

# In case the user wants a quiet output,
# the messages will be dumped to the log file
# set by this variable.
set(LOG_FILE "slam" CACHE STRING "This is my variable")
file(TO_CMAKE_PATH "${LOG_FILE}" PATH_LOG_FILE)
add_definitions(-DLOG_FILE="${PATH_LOG_FILE}")

# Add the log options to visualize the output of the several modules.
# Legacy migration: accept VERBOSE* cache variables and map to LOG*.
if(DEFINED VERBOSE)
  if(NOT DEFINED LOG_SETTINGS)
    set(LOG_SETTINGS ${VERBOSE} CACHE BOOL "Log option for the settings module" FORCE)
  endif()
  if(NOT DEFINED LOG_SLAM)
    set(LOG_SLAM ${VERBOSE} CACHE BOOL "Log option for the SLAM module" FORCE)
  endif()
  if(NOT DEFINED LOG_ADJUSTMENT)
    set(LOG_ADJUSTMENT ${VERBOSE} CACHE BOOL
        "Log option for covisibility graph and loop closure modules" FORCE)
  endif()
endif()
if(DEFINED VERBOSE_TERMINAL AND NOT DEFINED LOG_TERMINAL)
  set(LOG_TERMINAL ${VERBOSE_TERMINAL} CACHE BOOL
      "To show the output on the terminal" FORCE)
endif()
if(DEFINED VERBOSE_INTERFACE AND NOT DEFINED LOG_INTERFACE)
  set(LOG_INTERFACE ${VERBOSE_INTERFACE} CACHE BOOL
      "Log option for the interface module" FORCE)
endif()
if(DEFINED VERBOSE_CUDA AND NOT DEFINED LOG_CUDA)
  set(LOG_CUDA ${VERBOSE_CUDA} CACHE BOOL
      "Log option for the CUDA SIFT module" FORCE)
endif()
if(DEFINED VERBOSE_TIME AND NOT DEFINED LOG_TIME)
  set(LOG_TIME ${VERBOSE_TIME} CACHE BOOL
      "Log option for showing times" FORCE)
endif()

# If one wants to see the output on the terminal, set LOG_TERMINAL to ON.
set(LOG_TERMINAL ON CACHE BOOL "To show the output on the terminal")
# This option defines whether logs are sent to the interface module.
set(LOG_INTERFACE ON CACHE BOOL "Log option for the interface module")
# Extra logs for the CUDA SIFT module.
set(LOG_CUDA OFF CACHE BOOL "Log option for the CUDA SIFT module")
# This option is used to define timing logs.
set(LOG_TIME OFF CACHE BOOL "Log option for showing times")
# To output more debug messages on the LOCALIZATION module.
set(LOG_LOCALIZATION OFF CACHE BOOL "Verbose option for the localization module")
# To output more debug messages on the pose estimation module.
set(LOG_POSE_ESTIMATION OFF CACHE BOOL "Verbose option for the pose estimation module")
# To output more debug messages on the settings module.
set(LOG_SETTINGS OFF CACHE BOOL "Log option for the settings module")
# To output more debug messages on the SLAM module.
set(LOG_SLAM OFF CACHE BOOL "Log option for the SLAM module")
# To output more debug messages on the adjustment module.
set(LOG_ADJUSTMENT ON CACHE BOOL
    "Log option for covisibility graph and loop closure modules")
# To output debug messages on the representation module.
set(LOG_REPRESENTATION OFF CACHE BOOL
    "Log option for the representation module")

string(TOUPPER "${CMAKE_BUILD_TYPE}" _build_type_upper)
if(_build_type_upper STREQUAL "RELEASE")
  message(STATUS
      "slam-mer: Release mode detected, forcing selected LOG_* options OFF (LOG_INTERFACE excluded)")
  foreach(_log_option
      LOG_TERMINAL
      LOG_CUDA
      LOG_TIME
      LOG_LOCALIZATION
      LOG_POSE_ESTIMATION
      LOG_SETTINGS
      LOG_SLAM
      LOG_ADJUSTMENT
      LOG_REPRESENTATION)
    if(${_log_option})
      message(STATUS
          "slam-mer: ${_log_option}=ON requested, forcing OFF for Release")
    else()
      message(STATUS "slam-mer: ${_log_option}=OFF (Release)")
    endif()
    set(${_log_option} OFF CACHE BOOL "Log option forced OFF in Release mode" FORCE)
  endforeach()
endif()
unset(_build_type_upper)

if(LOG_TERMINAL)
  message(STATUS "slam-mer: LOG_TERMINAL=ON")
  add_definitions(-DLOG_TERMINAL)
endif()

if(LOG_INTERFACE)
  message(STATUS "slam-mer: LOG_INTERFACE=ON")
  add_definitions(-DLOG_INTERFACE)
endif()

if(LOG_CUDA)
  message(STATUS "slam-mer: LOG_CUDA=ON")
  add_definitions(-DLOG_CUDA)
endif()

if(LOG_TIME)
  message(STATUS "slam-mer: LOG_TIME=ON")
  add_definitions(-DLOG_TIME)
endif()

if(LOG_LOCALIZATION)
  message(STATUS "slam-mer: LOG_LOCALIZATION=ON")
  add_definitions(-DLOG_LOCALIZATION)
endif()

if(LOG_POSE_ESTIMATION)
  message(STATUS "slam-mer: LOG_POSE_ESTIMATION=ON")
  add_definitions(-DLOG_POSE_ESTIMATION)
endif()

if(LOG_SETTINGS)
  message(STATUS "slam-mer: LOG_SETTINGS=ON")
  add_definitions(-DLOG_SETTINGS)
endif()

if(LOG_SLAM)
  message(STATUS "slam-mer: LOG_SLAM=ON")
  add_definitions(-DLOG_SLAM)
endif()

if(LOG_ADJUSTMENT)
  message(STATUS "slam-mer: LOG_ADJUSTMENT=ON")
  add_definitions(-DLOG_ADJUSTMENT)
endif()

if (LOG_REPRESENTATION)
  message(STATUS "slam-mer: LOG_REPRESENTATION=ON")
  add_definitions(-DLOG_REPRESENTATION)
endif()

# This option is used for using Address Sanitizer (ASan)
SET(USE_ASAN OFF CACHE BOOL "Enable Address Sanitizer")
if(USE_ASAN)
  # check whether WITH_ALIKED=OFF to prevent conflicts
  if(WITH_ALIKED)
    message(FATAL_ERROR "AddressSanitizer is not compatible with LibTorch \
        (ALIKED, LightGlue). Please set WITH_ALIKED=OFF to use ASan.")
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

# -----------------------------------------------------------------------------
# Next, we list a set of visualization options that can be used to
# visualize the output of the several modules of the framework.
# we might want to hide some of the visualization options for efficiency
# purposes.

set(SHOW_DENSE OFF CACHE BOOL
    "Show the dense map points in the visualization")
string(TOUPPER "${CMAKE_BUILD_TYPE}" _build_type_upper_show_dense)
if(_build_type_upper_show_dense STREQUAL "RELEASE")
  if(SHOW_DENSE)
    message(STATUS "slam-mer: SHOW_DENSE=ON (Release)")
  else()
    message(STATUS
        "slam-mer: SHOW_DENSE=OFF requested, forcing ON for Release")
  endif()
  set(SHOW_DENSE ON CACHE BOOL
      "Show the dense map points in the visualization" FORCE)
endif()
unset(_build_type_upper_show_dense)

if(SHOW_DENSE)
  message(STATUS "slam-mer: SHOW_DENSE=ON")
  add_definitions(-DSHOW_DENSE)
endif()
