# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.26)
project(slam_mer_euroc LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(SLAM_MER_EXAMPLES_DIR /opt/slam-mer_examples CACHE PATH "Upstream examples checkout")
set(SLAM_MER_DIR /opt/slam-mer/build/install CACHE PATH "SLAM-MER installation")
list(APPEND CMAKE_PREFIX_PATH "${SLAM_MER_DIR}")
find_package(SlamMer REQUIRED CONFIG)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc calib3d)
add_subdirectory("${SLAM_MER_EXAMPLES_DIR}" upstream)
add_executable(slam-mer_euroc main_euroc.cpp)
target_include_directories(slam-mer_euroc PRIVATE
  "${SLAM_MER_EXAMPLES_DIR}/examples" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(slam-mer_euroc PRIVATE
  examples dataloaders_video SlamMer::slam SlamMer::interface SlamMer::types
  yaml-cpp ${OpenCV_LIBS})
target_link_options(slam-mer_euroc PRIVATE -Wl,--disable-new-dtags)
set_target_properties(slam-mer_euroc PROPERTIES
  BUILD_RPATH "${SLAM_MER_DIR}/lib")

enable_testing()
add_executable(visualization_queue_test test_visualization_queue.cpp)
target_link_libraries(visualization_queue_test PRIVATE
  SlamMer::interface SlamMer::slam SlamMer::types)
add_test(NAME visualization_queue COMMAND visualization_queue_test)
set_tests_properties(visualization_queue PROPERTIES
  WORKING_DIRECTORY "${SLAM_MER_EXAMPLES_DIR}")
