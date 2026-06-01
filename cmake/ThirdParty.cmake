# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later


# ---------------------------------------------------------------------------
# Third-party dependencies
# ---------------------------------------------------------------------------
message(STATUS "slam-mer: configuring third-party dependencies")

include(FetchContent)
find_package(CUDA REQUIRED)

# ---------------------------------------------------------------------------
# RERUN
find_package(rerun_sdk
  REQUIRED
  PATHS
    ${CMAKE_SOURCE_DIR}/thirdparty/rerun_cpp_sdk/build/install
    NO_DEFAULT_PATH)

# ---------------------------------------------------------------------------
# Eigen
find_package(Eigen3 CONFIG REQUIRED)
message(STATUS "slam-mer: Eigen3 configured at ${Eigen3_DIR}")

# ---------------------------------------------------------------------------
# GTSAM
find_package(GTSAM
  REQUIRED
  PATHS
    ${CMAKE_SOURCE_DIR}/thirdparty/gtsam/build/install/
    NO_DEFAULT_PATH)
message(STATUS "slam-mer: GTSAM configured")

# ---------------------------------------------------------------------------
# PoseLib
find_package(PoseLib
  REQUIRED
  PATHS
    ${CMAKE_SOURCE_DIR}/thirdparty/PoseLib/build/install
    NO_DEFAULT_PATH)
message(STATUS "slam-mer: PoseLib configured")

# ---------------------------------------------------------------------------
# OpenMP
find_package(OpenMP)

# ---------------------------------------------------------------------------
# Faiss
find_package(Faiss
  REQUIRED
  PATHS
    ${CMAKE_SOURCE_DIR}/thirdparty/faiss/build/install
    NO_DEFAULT_PATH)
message(STATUS "slam-mer: Faiss configured")

# ---------------------------------------------------------------------------
# tools for the image matcher
if(VISUAL_PLACE_LOCALIZATION)
  find_package(VisualPlaceLocalization
    REQUIRED
    PATHS
      ${CMAKE_SOURCE_DIR}/thirdparty/visual_place_localization/build/install
      NO_DEFAULT_PATH)
  message(STATUS "slam-mer: VisualPlaceLocalization configured at ${VisualPlaceLocalization_DIR}")
endif()

# ---------------------------------------------------------------------------
# libpng + libjpeg-turbo (OpenCV-free image I/O)
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
message(STATUS "slam-mer: PNG configured (${PNG_VERSION_STRING})")
message(STATUS "slam-mer: JPEG configured")

# ---------------------------------------------------------------------------
# tools for the image matcher
if(DEPTH_INFERENCE)
  find_package(DepthInference
    REQUIRED
    PATHS
      ${CMAKE_SOURCE_DIR}/thirdparty/depth_inference/build/install
      NO_DEFAULT_PATH)
  message(STATUS "slam-mer: DepthInference configured at ${DepthInference_DIR}")
endif()
