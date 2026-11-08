# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later
cmake_minimum_required(VERSION 3.26)
project(slam_mer_production_posterior_validation LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(SlamMer CONFIG REQUIRED)
find_package(Threads REQUIRED)
set(SLAM_MER_EXAMPLES_DIR /opt/slam-mer_examples CACHE PATH "Examples source checkout")
add_executable(test_backend_posterior test_backend_posterior.cpp)
target_link_libraries(test_backend_posterior PRIVATE SlamMer::slam SlamMer::interface SlamMer::covisibility_graph Threads::Threads)
target_link_options(test_backend_posterior PRIVATE -Wl,--disable-new-dtags)
enable_testing()
add_test(NAME backend_factors COMMAND test_backend_posterior factors)
add_test(NAME backend_worker COMMAND test_backend_posterior worker ${CMAKE_CURRENT_SOURCE_DIR}/pose3.yaml)
set_tests_properties(backend_factors backend_worker PROPERTIES WORKING_DIRECTORY ${SLAM_MER_EXAMPLES_DIR} TIMEOUT 30)
# The existing example dependencies are reused; source checkout is supplied explicitly.
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc calib3d)
add_subdirectory(${SLAM_MER_EXAMPLES_DIR} upstream)
add_executable(validate_euroc_posterior validate_euroc_posterior.cpp)
target_include_directories(validate_euroc_posterior PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/euroc
  ${SLAM_MER_EXAMPLES_DIR}/examples ${OpenCV_INCLUDE_DIRS})
target_link_libraries(validate_euroc_posterior PRIVATE examples dataloaders_video
  SlamMer::slam SlamMer::interface SlamMer::covisibility_graph Threads::Threads yaml-cpp ${OpenCV_LIBS})
target_link_options(validate_euroc_posterior PRIVATE -Wl,--disable-new-dtags)
add_executable(test_sim3_derivatives ../sim3/test_sim3_derivatives.cpp)
target_link_libraries(test_sim3_derivatives PRIVATE
  SlamMer::slam SlamMer::interface SlamMer::covisibility_graph)
target_link_options(test_sim3_derivatives PRIVATE -Wl,--disable-new-dtags)
add_test(NAME sim3_derivative_trace COMMAND test_sim3_derivatives trace ${CMAKE_CURRENT_BINARY_DIR}/sim3-trace.csv)
add_test(NAME sim3_fixed_objective COMMAND test_sim3_derivatives optimize ${CMAKE_CURRENT_BINARY_DIR}/sim3-optimize.csv)
set_tests_properties(sim3_derivative_trace sim3_fixed_objective PROPERTIES
  WORKING_DIRECTORY ${SLAM_MER_EXAMPLES_DIR} TIMEOUT 60)
add_test(NAME sim3_window_replacement COMMAND test_sim3_derivatives window ${CMAKE_CURRENT_BINARY_DIR}/sim3-window.csv)
set_tests_properties(sim3_window_replacement PROPERTIES
  WORKING_DIRECTORY ${SLAM_MER_EXAMPLES_DIR} TIMEOUT 60)

add_executable(test_sim3_analytic ../sim3/test_sim3_analytic.cpp)
target_link_libraries(test_sim3_analytic PRIVATE
  SlamMer::slam SlamMer::interface SlamMer::covisibility_graph)
target_link_options(test_sim3_analytic PRIVATE -Wl,--disable-new-dtags)
add_test(NAME sim3_analytic COMMAND test_sim3_analytic)
set_tests_properties(sim3_analytic PROPERTIES
  WORKING_DIRECTORY ${SLAM_MER_EXAMPLES_DIR} TIMEOUT 30)

add_executable(test_sim3_posterior ../sim3/test_sim3_posterior.cpp)
target_link_libraries(test_sim3_posterior PRIVATE
  SlamMer::slam SlamMer::interface SlamMer::covisibility_graph Threads::Threads)
target_link_options(test_sim3_posterior PRIVATE -Wl,--disable-new-dtags)
foreach(group linear geometry mean incremental edge)
  add_test(NAME sim3_posterior_${group} COMMAND test_sim3_posterior ${group})
  set_tests_properties(sim3_posterior_${group} PROPERTIES
    WORKING_DIRECTORY ${SLAM_MER_EXAMPLES_DIR} TIMEOUT 60)
endforeach()
add_test(NAME backend_worker_sim3 COMMAND test_backend_posterior worker-sim3 ${CMAKE_CURRENT_SOURCE_DIR}/similarity3.yaml)
set_tests_properties(backend_worker_sim3 PROPERTIES
  WORKING_DIRECTORY ${SLAM_MER_EXAMPLES_DIR} TIMEOUT 30)
add_executable(benchmark_sim3_posterior ../sim3/benchmark_sim3_posterior.cpp)
target_link_libraries(benchmark_sim3_posterior PRIVATE
  SlamMer::slam SlamMer::interface SlamMer::covisibility_graph)
target_link_options(benchmark_sim3_posterior PRIVATE -Wl,--disable-new-dtags)
