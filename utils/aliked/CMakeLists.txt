# Copyright 2023 ETH Zurich and Janusch Patas
# Apache-2.0
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.26)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(LightGlue VERSION 1.0.0 LANGUAGES CXX CUDA)
else()
  # When included from slam-mer, CXX is already configured by the parent.
  project(LightGlue VERSION 1.0.0 LANGUAGES CUDA)
endif()

# Parent project options may inject flags NVCC cannot use with this toolchain.
get_directory_property(_inherited_compile_options COMPILE_OPTIONS)
if(_inherited_compile_options)
  set(_filtered_compile_options "")
  foreach(_opt IN LISTS _inherited_compile_options)
    if(_opt MATCHES "c\\+\\+23" OR _opt MATCHES "x86-64-v2")
      continue()
    endif()
    list(APPEND _filtered_compile_options "${_opt}")
  endforeach()
  set_property(DIRECTORY PROPERTY COMPILE_OPTIONS "${_filtered_compile_options}")
endif()
unset(_inherited_compile_options)
unset(_filtered_compile_options)
unset(_opt)

set(LIBTORCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/libtorch")
list(PREPEND CMAKE_PREFIX_PATH "${LIBTORCH_DIR}")

# Torch warns if CMAKE_CUDA_ARCHITECTURES is set during find_package(Torch).
if(DEFINED CMAKE_CUDA_ARCHITECTURES
   AND NOT "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL ""
   AND NOT "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "OFF")
  set(_saved_cuda_arch "${CMAKE_CUDA_ARCHITECTURES}")
  unset(CMAKE_CUDA_ARCHITECTURES CACHE)
  unset(CMAKE_CUDA_ARCHITECTURES)
endif()

if(DEFINED ENV{TORCH_CUDA_ARCH_LIST} AND NOT DEFINED TORCH_CUDA_ARCH_LIST)
  set(TORCH_CUDA_ARCH_LIST "$ENV{TORCH_CUDA_ARCH_LIST}" CACHE STRING "Torch CUDA arch list" FORCE)
endif()

# Keep third-party behavior used previously for ALIKED.
set(CAFFE2_USE_CUDNN ON CACHE BOOL "Enable cuDNN through Caffe2/Torch for ALIKED" FORCE)

find_package(Torch REQUIRED)
find_package(CUDAToolkit REQUIRED)

# Restore/fallback CUDA architectures after Torch configuration.
if(DEFINED _saved_cuda_arch AND NOT "${_saved_cuda_arch}" STREQUAL "")
  set(CMAKE_CUDA_ARCHITECTURES "${_saved_cuda_arch}" CACHE STRING "CUDA archs for ALIKED" FORCE)
elseif(DEFINED ENV{CUDA_ARCH} AND NOT "$ENV{CUDA_ARCH}" STREQUAL "")
  set(CMAKE_CUDA_ARCHITECTURES "$ENV{CUDA_ARCH}" CACHE STRING "CUDA archs for ALIKED" FORCE)
elseif(DEFINED CUDA_ARCH AND NOT "${CUDA_ARCH}" STREQUAL "")
  set(CMAKE_CUDA_ARCHITECTURES "${CUDA_ARCH}" CACHE STRING "CUDA archs for ALIKED" FORCE)
else()
  set(CMAKE_CUDA_ARCHITECTURES "86" CACHE STRING "CUDA archs for ALIKED" FORCE)
endif()
if("${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "" OR "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "OFF")
  set(CMAKE_CUDA_ARCHITECTURES "86" CACHE STRING "CUDA archs for ALIKED" FORCE)
endif()

if(CUDAToolkit_VERSION VERSION_LESS "12.0")
  message(FATAL_ERROR "ALIKED requires CUDA 12.0 or higher (found: ${CUDAToolkit_VERSION})")
endif()

add_library(LightGlue_lib STATIC
  src/lightglue/feature/aliked.cpp
  src/lightglue/feature/dkd.cpp
  src/lightglue/feature/sddh.cpp
  src/lightglue/feature/internal/input_padder.cpp
  src/lightglue/feature/internal/get_patches.cpp
  src/lightglue/feature/internal/blocks.cpp
  src/lightglue/ops/deform_conv2d.cpp
  src/lightglue/ops/deform_conv2d_kernel.cu
  src/lightglue/ops/get_patches_cuda.cu
)

add_library(LightGlue::lib ALIAS LightGlue_lib)
add_library(lightglue::LightGlue_lib ALIAS LightGlue_lib)

target_include_directories(LightGlue_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(LightGlue_lib PUBLIC cxx_std_20)
set_target_properties(LightGlue_lib PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CUDA_STANDARD 17
  CUDA_STANDARD_REQUIRED ON
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON
  POSITION_INDEPENDENT_CODE ON
  DEBUG_POSTFIX "d"
)

target_compile_definitions(LightGlue_lib
  PRIVATE
    LIGHTGLUE_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models"
    $<$<COMPILE_LANGUAGE:CUDA>:__CUDA_NO_HALF_OPERATORS__>
    $<$<COMPILE_LANGUAGE:CUDA>:__CUDA_NO_HALF_CONVERSIONS__>
    $<$<COMPILE_LANGUAGE:CUDA>:__CUDA_NO_HALF2_OPERATORS__>
    $<$<COMPILE_LANGUAGE:CUDA>:__CUDA_NO_BFLOAT16_CONVERSIONS__>
)

target_compile_options(LightGlue_lib
  PRIVATE
    $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:Debug>>:-G;-g>
    $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:Release>>:-O3;--use_fast_math;-DNDEBUG>
)

if(HIDE_CUDA_NVCC_WARNINGS)
  target_compile_options(LightGlue_lib
    PRIVATE
      $<$<COMPILE_LANGUAGE:CUDA>:-Wno-deprecated-gpu-targets>
  )
  target_link_options(LightGlue_lib PRIVATE $<DEVICE_LINK:-Xnvlink=-w>)
endif()

target_link_libraries(LightGlue_lib
  PUBLIC
    ${TORCH_LIBRARIES}
  PRIVATE
    CUDA::cudart
    CUDA::curand
    CUDA::cublas
)
