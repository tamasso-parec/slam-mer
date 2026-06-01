/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file map.cpp
/// @brief Implements `map::Map` operations and map-global runtime state.
/// @details Defines map lifecycle, data access, representation calls, and
/// visualization wiring used by the SLAM mapping backend.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <map.hpp>
#include <print>
#include <rasterization_helper.hpp>
#include <settings.hpp>
#include "types/id.hpp"

// Standard C++ libraries

namespace slam_mer {
namespace map {
namespace {
cell_id_t::set __lastUpdatedCells;
std::shared_mutex __sortMutex;
pose_t __currentCameraPose = pose_t();
intrinsics_t __currentIntrinsics = intrinsics_t();
keyframe_id_t::set __localizationBufferKeyframes = keyframe_id_t::set();

Representation::shared_ptr __representation;

mutex_t __debugVariablesMutex;
size_t __numberVisibleCells = 0;
size_t __numberClosestCells = 0;

}  // namespace

// ---------------------------------------------------------------------------
// Construct map and initialize visualization and representation bindings
// ---------------------------------------------------------------------------
Map::Map() {
  using map::initiate_interface_attributes;
  using map::set_getters_visualization;
  using slam_mer::settings::sharedSettings;
  using std::bind;

  // Instanciate the map object
  // 1 - Initiate the camera location
  // 2 - Initiate the interface attributes
  // 3 - Set the getters for the visualization interface (functions to get)
  // 4 - Instanciate the 3D pointcloud representation (octree, helper, etc)
  // 5 - Launch the thread that will sort the cells by distance

  // Camera location
  _cameraPose = std::make_shared<pose_t>(pose_t::Identity());

  // Interface
  initiate_interface_attributes();
  set_getters_visualization(
      [this](const point_id_t id__) -> output_t<point3d_t> {
        return this->get(id__);
      },
      [this](const keyframe_id_t id__) -> output_t<pose_t> {
        return this->get(id__);
      },
      bind(&Map::__get, this, std::placeholders::_1),
      bind(&Map::get_mappoints_neighboors_x, this, std::placeholders::_1),
      bind(&Map::__get_timestamp, this, std::placeholders::_1));

  // 3D pointcloud representation
  __representation = Representation::make_shared(
      sharedSettings->map_voxel_half_size(),
      sharedSettings->map_coarse_half_size(),
      sharedSettings->map_scale_factor());
}

// ---------------------------------------------------------------------------
// Destroy map and stop internal worker threads
// ---------------------------------------------------------------------------
Map::~Map() {

  // 1. Wait until the thread that sorts cells by distance is stopped
  if (_sortCellsThread.joinable()) {
    _sortCellsThread.request_stop();
    _sortCellsThread.join();
  }

  std::println("[Map] Stopped successfully.");
}

// ---------------------------------------------------------------------------
// Start internal map worker threads
// ---------------------------------------------------------------------------
void Map::__start_threads() {
  // If a thread is already running, stop it before starting a new one
  if (_sortCellsThread.joinable()) {
    _sortCellsThread.request_stop();
    _sortCellsThread.join();
  }
  // Launch the thread that will sort the cells by distance to the camera
  _sortCellsThread =
      std::jthread([this](std::stop_token token__) { __sort_cells(token__); });
}

// ---------------------------------------------------------------------------
// Get number of allocated map cells
// ---------------------------------------------------------------------------
std::tuple<size_t, size_t, size_t> Map::__get_number_cells() const {

  size_t numberVisibleCells = 0;
  size_t numberClosestCells = 0;
  {
    read_lock_t lock(__debugVariablesMutex);
    numberVisibleCells = __numberVisibleCells;
    numberClosestCells = __numberClosestCells;
  }
  const size_t total = __representation->get_number_cells();
  return std::make_tuple(numberVisibleCells, numberClosestCells, total);
}

// ---------------------------------------------------------------------------
// Check cell existence without external locking
// ---------------------------------------------------------------------------
bool Map::__exists(const cell_id_t& cellId_) const {
  return __representation->exists(cellId_);
}

// ---------------------------------------------------------------------------
// Check cell existence with map-level lock
// ---------------------------------------------------------------------------
bool Map::exists(const cell_id_t& cellId_) const {
  read_lock_t lock(_mutex);
  return __exists(cellId_);
}

// ---------------------------------------------------------------------------
// Validate that cell exists in representation
// ---------------------------------------------------------------------------
void Map::__sanity_exists(const cell_id_t& cellId__) const {
  if (!__representation->exists(cellId__))
    __throw__(
        "In get_visible_cells, cell with ID " + std::string(cellId__) +
        " does not exist in the map.");
}

// ---------------------------------------------------------------------------
// Retrieve cell data for visualization and querying
// ---------------------------------------------------------------------------
output_t<cell3d_t> Map::__get(const cell_id_t& cellId_) const {

  const array3D_t::pair corners = __representation->get_corners(cellId_);
  const array3D_t& minBound = corners.first;
  const array3D_t& maxBound = corners.second;
  cell3d_t cell(minBound, maxBound);
  cell.set_highlighted(__representation->is_highlighted(cellId_));
  cell.set_point_count(__representation->get_point_count(cellId_));
  return cell;
}

// ---------------------------------------------------------------------------
// Retrieve all point IDs contained in a cell
// ---------------------------------------------------------------------------
point_id_t::vector Map::__get_points_in_cell_x(const cell_id_t& cellId_) const {
  output_t<point_id_t::set> pointsSet = __representation->get_points(cellId_);
  if (pointsSet.fail()) __throw__("Cell ID does not exist in the map.");
  return pointsSet.get().to_vector();
}

// ---------------------------------------------------------------------------
// Retrieve timestamp associated with keyframe ID
// ---------------------------------------------------------------------------
time_stamp_t Map::__get_timestamp(const keyframe_id_t keyframeId_) const {
  using data_type::SensorData;
  using std::println;

  read_lock_t lock(_mutex);
  const keyframe_t::shared_ptr& data = _keyframes.at(keyframeId_);
  const SensorData::weak_ptr& cameraData = data->get_camera_data();
  const output_t<time_stamp_t> timeStamp =
      SensorData::get_timestamp(cameraData);
  if (timeStamp.fail()) {
    println(
        "[Map] {}Warning{}: Failed to get timestamp for keyframe ID {}.",
        yellow,
        reset,
        std::string(keyframeId_));
  }
  return timeStamp;
}

// ---------------------------------------------------------------------------
// Insert point into map and representation
// ---------------------------------------------------------------------------
output_t<pair_t<point_id_t, cell_id_t>> Map::insert(const point3d_t& point_) {
  using map::add_cell_to_interface;
  using map::add_new_point_to_interface;

  // This function can be a bit tricky because of the complexity involved
  // in the map representation. The steps are as follows:
  // 1 - Get a new point_ ID
  // 2 - Insert the point_ into the map _points structure
  // 3 - Insert the point_ into the _neighbors structure
  // 4 - Insert the point_ into the map representation (cells, voxels, etc)
  // 5 - If any of the steps fail, remove the point_ from the map and
  //     _neighbors structure
  // 6 - Return the point_ ID and cell ID

  __lock_points();

  const point_id_t candidateId(__get_next_point_id());
  if (__exists(candidateId) == fail_t::NO_FAIL) {
    __unlock_points();
    return fail_t::ALREADY_EXISTS;
  }

  point3d_t::shared_ptr pointPtr_ = point3d_t::make_shared(point_);
  _points[candidateId] = pointPtr_;
  _neighbors[candidateId] = neighbor_t::vector();
  __unlock_points();

  // Only after knowing that the _points could be created properly,
  // we can try to add them to the representation. We want to do this
  // to avoid blocking the point_ and neighboors.
  const cell_id_t cellId_ = __representation->get_cell_id(point_);
  const fail_t error = __representation->insert(cellId_, candidateId);

  if (error) {
    __lock_points();
    if (_points.contains(candidateId)) _points.erase(candidateId);
    if (_neighbors.contains(candidateId)) _neighbors.erase(candidateId);
    __unlock_points();
    return error;
  }

  add_new_point_to_interface(candidateId);
  add_cell_to_interface(cellId_);

  // and outputs the right variables
  return pair_t<point_id_t, cell_id_t>(candidateId, cellId_);
  ;
}

// ---------------------------------------------------------------------------
// Insert keyframe into map
// ---------------------------------------------------------------------------
output_t<keyframe_id_t> Map::insert(
    const pose_t& pose_,
    const keyframe_t& keyframeData_) {
  using map::add_new_keyframe_to_interface;

  // To insert a keyframe_, we need to do the following steps:
  // 1 - Get a new keyframe_ ID
  // 2 - Insert the keyframe_ pose_ into the map
  // 3 - Insert the keyframe_ data into the map
  // 4 - If any of the steps fail, remove the keyframe_ from the map
  // 5 - Return the keyframe_ ID

  const pose_t::shared_ptr posePtr_ = pose_t::make_shared(pose_);
  const keyframe_t::shared_ptr keyframePtr =
      keyframe_t::make_shared(keyframeData_);
  keyframePtr->set_pose(posePtr_);

  __lock_keyframes();

  const keyframe_id_t candidateId(__get_next_keyframe_id());

  if (__exists(candidateId) == fail_t::NO_FAIL) {
    __unlock_keyframes();
    return fail_t::ALREADY_EXISTS;
  }

  _keyframes[candidateId] = keyframePtr;

  __unlock_keyframes();

  add_new_keyframe_to_interface(candidateId);

  // Returns the keyframe_ id.
  return candidateId;
}

// ---------------------------------------------------------------------------
// Update point coordinates and cell assignment
// ---------------------------------------------------------------------------
fail_t Map::update(const point_id_t& pointId_, const array3D_t& point_) {
  using map::add_cell_to_interface;
  using map::add_changed_point_to_interface;
  using map::change_cell_to_interface;

  // ---------------------------------------------------------------------------
  // lambda functions with __throws__. Things that should not happen.
  auto __sanity_in_cell = [&](output_t<bool> inCell__,
                              const cell_id_t& initialCellId__) -> void {
    if (inCell__.fail())
      __throw__(
          "In update point_, failed to check if point_ is in cell " +
          std::string(initialCellId__) +
          ". Error message: " + inCell__.get_error().get_message());
  };
  auto __sanity_same_id = [&](const cell_id_t& oldCellId__,
                              const cell_id_t& newCellId__) -> void {
    if (oldCellId__ == newCellId__)
      __throw__(
          "In update point_, after reassigning the point_, the new cell is the "
          "same as the old one. (oldCellId: " +
          std::string(oldCellId__) +
          ", newCellId: " + std::string(newCellId__) + ")");
  };
  auto __sanity_fail_switch_cell = [&](const fail_t& error__,
                                       const cell_id_t& initialCellId__,
                                       const cell_id_t& newCellId__) -> void {
    if (error__)
      __throw__(
          "In update point_, failed to switch the point_ from cell " +
          std::string(initialCellId__) + " to cell " +
          std::string(newCellId__) +
          ". Error message: " + error__.get_message());
  };

  // ---------------------------------------------------------------------------

  // This can be confusion to the SLAM is something happens to the
  // variables, _points and cells in between this call.
  // This function:
  // 1 - Get the point_ from the map
  // 2 - Change the coordinates of the point_
  // 3 - Check if the point_ is still in the same cell, otherwise reassign
  // 4 - Update the visualization
  // 5 - If any point_ not updated return an error.
  // 6 - If cells not updated, __throw__ an error.

  const output_t<point3d_t::weak_ptr> pointPtr_ = this->at(pointId_);
  if (pointPtr_.fail()) return pointPtr_.get_error();

  const cell_id_t initialCellId = __representation->get_cell_id(pointId_);

  if (!point3d_t::set_coordinates(pointPtr_.get(), point_))
    return fail_t(fail_t::LOST_PTR, "[MAP] Point is not available anymore");

  const auto inCell = __representation->contains(initialCellId, point_);
  __sanity_in_cell(inCell, initialCellId);

  if (!inCell.get()) {
    const cell_id_t newCellId = __representation->get_cell_id(point_);
    __sanity_same_id(initialCellId, newCellId);
    const fail_t error =
        __representation->switch_cell(initialCellId, newCellId, pointId_);
    __sanity_fail_switch_cell(error, initialCellId, newCellId);

    // We also need to update the cells if something changed.
    // This can happen if the point_ is moving from one cell to another.
    // Notice that at this point_, we know that the point_ will be updated.
    change_cell_to_interface(initialCellId);
    add_cell_to_interface(newCellId);
  }
  add_changed_point_to_interface(pointId_);

  return fail_t::NO_FAIL;
}

// ---------------------------------------------------------------------------
// Update keyframe pose
// ---------------------------------------------------------------------------
fail_t Map::update(const keyframe_id_t& keyframeId_, const pose_t& pose_) {
  using map::change_keyframe_to_interface;

  // The updates of the keyframe_ locations is straight forward
  // 0 - Sanity check if the keyframe_ exists (we don't remove _keyframes)
  // 1 - Get the keyframe_ from the map
  // 2 - Update the pose_
  // 3 - Update the interface
  // 3 - If any of the steps fail, return the error

  __sanity_exists(keyframeId_);

  output_t<keyframe_t::weak_ptr> outputPtr = this->at(keyframeId_);
  if (outputPtr.fail()) return outputPtr.get_error();
  const pose_t::weak_ptr posePtr_ = keyframe_t::get_pose(outputPtr.get());
  const fail_t error = pose_t::update(posePtr_, pose_);
  change_keyframe_to_interface(keyframeId_);
  return error;
}

// ---------------------------------------------------------------------------
// Remove point from map and representation
// ---------------------------------------------------------------------------
fail_t Map::remove(const point_id_t& pointId_) {
  using map::change_cell_to_interface;
  using map::hide_point_from_interface;

  // Not sure if this is the most optimal way of doing it.
  // I'm assuming that the delete from the representation is the most
  // likely to fail, so we keep it for the end, after removing from the map.
  // 1 - Remove from the map _points structure
  // 2 - Remove from the map _neighbors structure
  // 3 - Remove from the map representation (cells, voxels, etc)
  // 4 - If any of the steps fail, return the error
  // 5 - Tell the visualization that there was some changes
  // 6 - Return no fail if everything went well

  __lock_points();
  __sanity_sync(pointId_);

  if (!_points.contains(pointId_)) {
    __unlock_points();
    return fail_t(
        fail_t::NOT_FOUND,
        "When trying to delete, point_ with ID " + std::string(pointId_) +
            " does not exist in the map.");
  }

  _points.erase(pointId_);
  _neighbors.erase(pointId_);

  const cell_id_t cellId_ = __representation->get_cell_id(pointId_);
  const fail_t error = __representation->remove(cellId_, pointId_);
  ___stanity_exists(error, cellId_, pointId_);
  __unlock_points();

  hide_point_from_interface(pointId_);
  change_cell_to_interface(cellId_);

  return fail_t::NO_FAIL;
}

// ---------------------------------------------------------------------------
// Sort cells by camera distance in background worker thread
// ---------------------------------------------------------------------------
void Map::__sort_cells(std::stop_token token_) {

  // This function will run in a separate thread and will
  // continuously sort the cells by distance to the current camera pose_.
  // 1 - While the stop token_ is not requested, do:
  //    a - Collect the current camera pose_ and intrinsics_
  //    b - If the pose_ or intrinsics_ are not set, sleep and continue
  //    c - Sort the cells by distance to the current camera pose_
  //    d - Sleep for a short amount of time to avoid busy waiting

  timer_t timer;
  uint64_t millisecondsWait = 100;

  while (!token_.stop_requested()) {
    timer.tic();

    pose_t localCameraPose;
    intrinsics_t localIntrinsics;
    {
      read_lock_t lock(__sortMutex);
      localCameraPose = __currentCameraPose;
      localIntrinsics = __currentIntrinsics;
    }

    if (localCameraPose.is_unset() || localIntrinsics.empty()) {
      timer.pace(duration_t::from_ms(millisecondsWait));
      continue;
    }

    __representation->sort_cells(localCameraPose, localIntrinsics);
    timer.pace(duration_t::from_ms(millisecondsWait));
  }
}

// ---------------------------------------------------------------------------
// Compute visible cells and refresh highlight state
// ---------------------------------------------------------------------------
cell_id_t::set Map::get_visible_cells(
    size_t requestedCells_,
    const intrinsics_t intrinsics_,
    const array2Size_t imageSize_,
    const pose_t cameraToWorldTransformation_) {
  using helper::iou;
  using helper::project;
  using map::change_cell_to_interface;
  using slam_mer::settings::sharedSettings;

  cell_id_t::set cellsToBeUpdated;

  // ---------------------------------------------------------------------------
  // Lambda functions
  auto ___clear_highlights_save_pose = [&]() -> void {
    // clear last updated cells
    for (const cell_id_t& cellId_ : __lastUpdatedCells) {
      if (__representation->exists(cellId_)) {
        __representation->unhighlight(cellId_);
        cellsToBeUpdated.insert(cellId_);
      }
    }
    __lastUpdatedCells.clear();
    // Save pose_
    {
      write_lock_t lock(__sortMutex);
      __currentCameraPose = cameraToWorldTransformation_;
      __currentIntrinsics = intrinsics_;
    }
  };
  // ---------------------------------------------------------------------------

  // This function:
  // 1 - Clear previous highlights (__lastUpdatedCells) and save pose_
  // 2 - Save a copy of the current camera pose_ and intrinsics_
  // 3 - Iterate through the closest cells and project them to the image
  // 4 - Check for intersections and highlight the cells that are visible
  // 5 - Update the interface for the changed cells

  ___clear_highlights_save_pose();

  // Correct for the case where we still don't have enough cells.
  const size_t& totalCells = __representation->size_closest_cells();
  const size_t numberCellsToHighlight = std::min(requestedCells_, totalCells);
  const pose_t worldToCamera = cameraToWorldTransformation_.get_invert();
  const double& iouThreshold = sharedSettings->map_visibility_iou_threshold();

  for (size_t index = 0; index < numberCellsToHighlight; index++) {
    const auto queryCellIdOutput = __representation->get_closes_cell_x(index);
    if (queryCellIdOutput == fail_t::OUT_OF_BOUNDS) break;
    const cell_id_t& queryCellId = queryCellIdOutput;
    __sanity_exists(queryCellId);
    const auto [queryBondingBoxMin, queryBondingBoxMax] =
        __representation->get_corners(queryCellId);
    const auto& [valid, queryImageBox] = project(
        queryBondingBoxMin,
        queryBondingBoxMax,
        worldToCamera,
        intrinsics_);
    if (!valid) continue;  // If out of bouds -- ignore
    if (__lastUpdatedCells.size() == 0) {
      __representation->highlight(queryCellId);
      __lastUpdatedCells.insert(queryCellId);
      cellsToBeUpdated.insert(queryCellId);
    } else {
      bool toAdd = true;
      for (const cell_id_t& otherCellId : __lastUpdatedCells) {
        __sanity_exists(otherCellId);
        const auto [testBondingBoxMin, testBondingBoxMax] =
            __representation->get_corners(otherCellId);
        const auto& [validTest, testImageBox] = project(
            testBondingBoxMin,
            testBondingBoxMax,
            worldToCamera,
            intrinsics_);
        if (!validTest) continue;
        const double intersection =
            iou(queryImageBox, testImageBox, imageSize_);
        if (intersection > iouThreshold) {
          toAdd = false;
          break;
        }
      }
      if (toAdd) {
        __representation->highlight(queryCellId);
        __lastUpdatedCells.insert(queryCellId);
        cellsToBeUpdated.insert(queryCellId);
      }
    }
  }
  // Go through the cells that need to be updated
  for (const cell_id_t& cellId_ : cellsToBeUpdated) {
    change_cell_to_interface(cellId_);
  }

  // Update variables for debuging
  {
    write_lock_t lock(__debugVariablesMutex);
    __numberVisibleCells = __lastUpdatedCells.size();
    __numberClosestCells = numberCellsToHighlight;
  }

  return __lastUpdatedCells;
}

// ---------------------------------------------------------------------------
// Find keyframes newer than starting keyframe ID
// ---------------------------------------------------------------------------
keyframe_id_t::ordered Map::find_recent_keyframes(
    const keyframe_id_t startingKeyframe_) const {

  // This function finds all the _keyframes that were added
  // after the given starting keyframe_.
  // 1 - Find the starting keyframe_ in the map
  // 2 - Iterate through all the remaining _keyframes
  // 3 - Add the keyframe_ IDs to the output set

  keyframe_id_t::ordered recentKeyframe_ids;
  __lock_keyframes();

  auto keyframeIt = _keyframes.find(startingKeyframe_);
  keyframeIt++;
  for (; keyframeIt != _keyframes.end(); keyframeIt++)
    recentKeyframe_ids.insert(keyframeIt->first);
  __unlock_keyframes();
  return recentKeyframe_ids;
}

// ---------------------------------------------------------------------------
// Set keyframes currently stored in localization buffer
// ---------------------------------------------------------------------------
void Map::set_localization_buffer_keyframes(
    const keyframe_id_t::ordered& keyframesInBuffer_) {

  __localizationBufferKeyframes.lock();
  __localizationBufferKeyframes.clear();
  for (const keyframe_id_t& keyframeId_ : keyframesInBuffer_)
    __localizationBufferKeyframes.insert(keyframeId_);
  __localizationBufferKeyframes.unlock();
}

// ---------------------------------------------------------------------------
// Get keyframes currently stored in localization buffer
// ---------------------------------------------------------------------------
keyframe_id_t::set Map::get_localization_buffer_keyframes() const {
  return __localizationBufferKeyframes;
}

// ---------------------------------------------------------------------------
// Get timestamp and pose for all keyframes
// ---------------------------------------------------------------------------
dictionary_t<keyframe_id_t, pair_t<pose_t, time_stamp_t>>
Map::get_timestamp_and_poses() const {
  using data_type::SensorData;

  // This function returns a dictionary with all the _keyframes
  // timestamp and pose_.
  // 1 - Iterate through all the _keyframes
  // 2 - For each keyframe_, get the timestamp and pose_
  // 3 - Add the keyframe_ ID, timestamp and pose_ to the output dictionary

  dictionary_t<keyframe_id_t, pair_t<pose_t, time_stamp_t>>
      keyFramesTimeStampPose;
  __lock_keyframes();
  for (const auto& item : _keyframes) {
    const keyframe_id_t& id = item.first;
    const keyframe_t::shared_ptr& data = item.second;
    const pose_t pose_(data->get_pose());
    const SensorData::weak_ptr sensorData = data->get_camera_data();
    const output_t<time_stamp_t> timeStamp =
        SensorData::get_timestamp(sensorData);
    if (timeStamp.fail()) {
      std::println(
          "[Map] {}Warning{}: Failed to get timestamp for keyframe ID {}.",
          yellow,
          reset,
          std::string(id));
    }
    keyFramesTimeStampPose[id] = pair_t<pose_t, time_stamp_t>(pose_, timeStamp);
  }

  __unlock_keyframes();
  return keyFramesTimeStampPose;
}

}  // namespace map
}  //namespace slam_mer
