/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file interface.cpp
/// @brief Implements core interface lifecycle and shared visualization state.
/// @details Defines base interface control loops and pointer wiring.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

#include <interface.hpp>

// My libraries
#include <types/timer.hpp>

// Thirdparty libraries

// Std libraries

namespace slam_mer {
namespace interface {

extern std::vector<std::array<float, 3>> __listKeyframes;
extern std::vector<std::array<double, 7>> __keyframesPose;
extern std::vector<std::array<float, 3>> __pointsDense;
extern std::vector<uint32_t> __colorsDense;
extern std::vector<time_stamp_t> __keyframesTimeStamp;

const std::vector<std::array<float, 3>>& get_list_keyframes() {
  return __listKeyframes;
}
const std::vector<std::array<double, 7>>& get_keyframes_pose() {
  return __keyframesPose;
}
const std::vector<std::array<float, 3>>& get_points_dense() {
  return __pointsDense;
}
const std::vector<uint32_t>& get_colors_dense() {
  return __colorsDense;
}
const std::vector<time_stamp_t>& get_keyframes_time_stamp() {
  return __keyframesTimeStamp;
}

namespace {

/// Frequency of the map log loop
double __mapLogFrequency = 10.0;  // Hz
/// Frequency of the camera log loop
double __cameraLogFrequency = 10.0;  // Hz
/// Frequency of the internal points update loop
constexpr double __internalPointsUpdateFrequency = 10.0;  // Hz

}  // namespace

// ----------------------------------------------------------
// Rerun global variables

// Attributes that come from the main SLAM execution code
// The easiest way of dealing with the visualization variables is to have
// some global pointers to the visualization variables that come from the SLAM
// framework.
/// Global variable to store the new points needed to be logged
set_t<point_id_t>::shared_ptr __newPoints;
/// Global variable to store the` points to be updated
set_t<point_id_t>::shared_ptr __updatePoints;
/// Global variable to store the points to be hidden
set_t<point_id_t>::shared_ptr __hidePoint;

/// Global variable to store the cells to be visualized
set_t<cell_id_t>::shared_ptr __newCells;
/// Global variable to store the index of the cells to be updated
set_t<cell_id_t>::shared_ptr __updateCells;

// To visualize the camera trajectory
/// Global variable to store the new points needed to be logged
set_t<keyframe_id_t>::shared_ptr __newKeyframe;
/// Global variable to store the points to be updated
set_t<keyframe_id_t>::shared_ptr __updateKeyframe;
/// Global variable to store the points to be hidden
set_t<keyframe_id_t>::shared_ptr __hideKeyframe;
/// List of keyframe connections that need to be added
set_t<keyframe_id_t::pair>::shared_ptr __newKeyframeConnections;
/// List of keyframe connections that need to be updated
set_t<keyframe_id_t::pair>::shared_ptr __updatedKeyframeConnections;
/// List of keyframe connections that need to be deleted
set_t<keyframe_id_t::pair>::shared_ptr __deletedKeyframeConnections;

/// List of histogram keyframes in localization counts to be logged
vector_t<vector_t<size_t>>::shared_ptr __histogramCounts;
/// List of keyframe creation criteria to be logged
vector_t<dictionary_t<uint8_t, int>>::shared_ptr __keyframeCriteria;
/// Query callback to retrieve a map point by ID.
std::function<output_t<point3d_t>(point_id_t)> ___get_point;
/// Query callback to retrieve a keyframe pose by ID.
std::function<output_t<pose_t>(keyframe_id_t)> ___get_keyframe;
/// Query callback to retrieve one map cell by ID.
std::function<cell3d_t(cell_id_t)> ___get_cell;
/// Query callback to retrieve neighbor observations for a map point.
std::function<neighbor_t::vector(point_id_t)> ___get_neighbors;
/// Query callback to retrieve keyframe timestamp by ID.
std::function<time_stamp_t(keyframe_id_t)> ___get_keyframe_timeStamp;

/// @brief Emit queued textual messages.
extern void __log_messages();
/// @brief Emit covisibility graph entities.
extern void __log_covisibility();
/// @brief Emit camera pose entities.
extern void __log_camera_location();
/// @brief Emit image entities.
extern void __log_images();
/// @brief Emit scalar/feature plots.
extern void __log_plots();
/// @brief Emit dense point cloud entities.
extern void __log_dense_point_cloud();
/// @brief Emit sparse map-point entities.
extern void __log_points();
/// @brief Emit map-cell entities.
extern void __log_cells();

/// @brief Main loop for map-related backend logging.
/// @param token_ Stop token used for cooperative thread shutdown.
void AbstractInterface::__map_loop(std::stop_token token_) {

  // This function:
  // 1 - Instantiates a timer object to fix max Hz
  // 2 - In a loop, logs the dense point cloud, points and cells at
  //    the specified frequency

  timer_t timer;
  while (token_.stop_requested() == false) {
    timer.tic();
#ifdef SHOW_DENSE
    __log_dense_point_cloud();
#endif
#ifdef DEBUG
    __log_points();
    __log_cells();
#endif
    timer.pace(duration_t::from_fps(__mapLogFrequency));
  }
}

/// @brief Main loop for camera/images/diagnostics backend logging.
/// @param token_ Stop token used for cooperative thread shutdown.
void AbstractInterface::__camera_loop(std::stop_token token_) {

  // This function:
  // 1 - Instantiates a timer object to fix max Hz
  // 2 - In a loop, logs the camera location and images at
  //    the specified frequency

  timer_t timer;
  while (token_.stop_requested() == false) {
    timer.tic();
    __log_messages();
#ifdef DEBUG
    __log_covisibility();
#endif
    __log_camera_location();
    __log_images();
#ifdef DEBUG
    __log_plots();
#endif
    timer.pace(duration_t::from_fps(__cameraLogFrequency));
  }
}

/// @brief Internal loop that synchronizes map/cell state from shared queues.
/// @param token_ Stop token used for cooperative thread shutdown.
void AbstractInterface::__internal_update_loop(std::stop_token token_) {

  // This function:
  // 1 - Instantiates a timer object to fix max Hz
  // 2 - In a loop, updates the points and cells to be displayed

  timer_t timer;
  while (token_.stop_requested() == false) {
    timer.tic();
    log_points();
    log_cells();
    timer.pace(duration_t::from_fps(__internalPointsUpdateFrequency));
  }
}

/// @brief Stop all interface worker threads and clear shared references.
/// @return `true` when shutdown completes.
bool AbstractInterface::stop_interface() {
  using std::println;

  // Stop the data update loop
  if (_updateDataThread.joinable()) {
    _updateDataThread.request_stop();
    _updateDataThread.join();
    println("[Interface] Ended points update thread.");
  }
  // Stop the rerun log loop
  if (_mapLogThread.joinable()) {
    _mapLogThread.request_stop();
    _mapLogThread.join();
    println("[Interface] Ended map log thread.");
  }
  // Stop the rerun camera log loop
  if (_cameraLogThread.joinable()) {
    _cameraLogThread.request_stop();
    _cameraLogThread.join();
    println("[Interface] Ended camera log thread.");
  }

  // 3D representation
  __newPoints.reset();
  __updatePoints.reset();
  __hidePoint.reset();
  __newCells.reset();
  __updateCells.reset();

  // Keyframes and covisibility
  __newKeyframe.reset();
  __updateKeyframe.reset();
  __hideKeyframe.reset();
  __newKeyframeConnections.reset();
  __updatedKeyframeConnections.reset();
  __deletedKeyframeConnections.reset();

  // Log keyframes in localization queue
  __histogramCounts.reset();

  // Log keyframe creation criteria
  __keyframeCriteria.reset();

  println("[Interface] Successfully stopped all threads.");

  return true;
}

/// @brief Initialize shared bindings and launch interface worker threads.
/// @param pointers_ Struct with shared data sources and query callbacks.
/// @param queryMapLogFrequency_ Target map logging frequency in Hz.
/// @param queryCameraLogFrequency_ Target camera logging frequency in Hz.
void AbstractInterface::initiate(
    PointersVisualization pointers_,
    const double queryMapLogFrequency_,
    const double queryCameraLogFrequency_) {

  // Define the pointers to the visualization variables that come from the SLAM
  // framework. This is done for efficiency purposes, so we don't have to
  // copy the variables every time we want to log them. This is a thread-safe
  // operation, so we can do it in parallel with the SLAM framework.

  // Mappoint structures
  __newPoints = pointers_.newPoints;
  __updatePoints = pointers_.updatePoints;
  __hidePoint = pointers_.hidePoint;
  // 3D representation
  __newCells = pointers_.newCells;
  __updateCells = pointers_.updateCells;
  // Keyframe structures
  __newKeyframe = pointers_.newKeyframes;
  __updateKeyframe = pointers_.updateKeyframes;
  __hideKeyframe = pointers_.deleteKeyframes;
  // Keyframe connections
  __newKeyframeConnections = pointers_.newKeyframeConnections;
  __updatedKeyframeConnections = pointers_.updatedKeyframeConnections;
  __deletedKeyframeConnections = pointers_.deletedKeyframeConnections;
  // Keyframes in localization queue
  __histogramCounts = pointers_.keyframePerFrame;
  // Keyframe creation criteria
  __keyframeCriteria = pointers_.keyframeCriteria;

  // Set the global log frequencies
  __mapLogFrequency = queryMapLogFrequency_;
  __cameraLogFrequency = queryCameraLogFrequency_;

  // Set the functions to query points and poses
  ___get_point = pointers_.get_point;
  ___get_keyframe = pointers_.get_keyframe;
  ___get_cell = pointers_.get_cell;
  ___get_neighbors = pointers_.get_neighbors;
  ___get_keyframe_timeStamp = pointers_.get_keyframe_timeStamp;

  // Launch a loop to update certain variables to display in rerun
  _updateDataThread = std::jthread(
      [this](std::stop_token token__) { __internal_update_loop(token__); });

  // Launch the rerun log loop in a separate thread
  _mapLogThread = std::jthread(
      [this](std::stop_token token__) { this->__map_loop(token__); });

  // Launch the rerun log loop for the camera in a separate thread
  _cameraLogThread = std::jthread(
      [this](std::stop_token token__) { this->__camera_loop(token__); });
}

}  // namespace interface
}  // namespace slam_mer
