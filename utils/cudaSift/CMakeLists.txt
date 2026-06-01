# Copyright (c) 2017 Marten Bjorkman
# MIT License
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later


message(STATUS "Building CudaSift")

# get and set the CUDA verbose output option
if(VERBOSE_CUDA)
  message("OPTIONS: CUDA SIFT verbose enabled")
  add_definitions(-DVERBOSE_CUDA)
endif()

if(HIDE_CUDA_NVCC_WARNINGS)
  list(APPEND CUDA_NVCC_FLAGS -Wno-deprecated-gpu-targets)
endif()

cuda_add_library(cudaSift SHARED
  src/cudaImage.cu
  src/cudaSiftH.cu
  src/matching.cu
  src/geomFuncs.cpp
)

target_include_directories(cudaSift PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/cudaSift>
)

target_link_libraries(cudaSift ${CUDA_cudadevrt_LIBRARY} Eigen3::Eigen)

# install utils include libraries
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cudaImage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cudaSift.h
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/cudaSift)
