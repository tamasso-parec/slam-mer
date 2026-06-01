/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file wrapper_visualization.cpp
/// @brief Implements buffered visualization update wrappers for map entities.
/// @details Defines shared update buffers and callback bindings.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <log.hpp>
#include <types/hash.hpp>
#include <wrapper_visualization.hpp>

// Standard C++ libraries

namespace slam_mer {
namespace map {
namespace {

// Define the variables for the interface visualization
// These variables are used to let other libraries know
// which points are new, updated or deleted
// in the map.
/// List of points that were added to the map and not yet in visualization
set_t<point_id_t>::shared_ptr __newPointsId;
/// List of point that were updated in the map and not yet in visualization
set_t<point_id_t>::shared_ptr __updatedPointsId;
/// List of points that were deleted from the map and not yet in visualization
set_t<point_id_t>::shared_ptr __deletedPointsId;

/// Cells in the point cloud representation
set_t<cell_id_t>::shared_ptr __newCells;
/// Cell in the point cloud that need to be updated in some way
set_t<cell_id_t>::shared_ptr __updateCells;

/// List of keyframes to be visualized. We keep it different from the maps
hash_t<keyframe_id_t, array3D_t>::shared_ptr __visualizationKeyframes;
/// List of keyframes that were added to the map and not yet in visualization
set_t<keyframe_id_t>::shared_ptr __newKeyframeId;
/// List of keyframes that were updated in the map and not yet
set_t<keyframe_id_t>::shared_ptr __updatedKeyframeId;
/// List of keyframes that were deleted from the map and not yet
set_t<keyframe_id_t>::shared_ptr __deletedKeyframeId;

// For the covisibility connections, the locations of the keyframes
// is already known and stored in visualizationKeyframes.
// We only need to store the connections between the keyframes.
/// List of covisibility connections that were added to the map and not yet
set_t<keyframe_id_t::pair>::shared_ptr __newCovisibilityConnections;
/// List of covisibility connections that were updated in the map and not yet
set_t<keyframe_id_t::pair>::shared_ptr __updatedCovisibilityConnections;
/// List of covisibility connections that were deleted from the map
set_t<keyframe_id_t::pair>::shared_ptr __deletedCovisibilityConnections;

/// We keep track of the number of keyframes in buffer
vector_t<vector_t<size_t>>::shared_ptr __keyframePerFrame;
// We keep track of the keyframe creation criteria
vector_t<dictionary_t<uint8_t, int>>::shared_ptr __keyframeCriteria;

// Getters for the visualization variables
std::function<output_t<point3d_t>(point_id_t)> ___get_point;
std::function<output_t<pose_t>(keyframe_id_t)> ___get_keyframe;
std::function<slam_mer::time_stamp_t(keyframe_id_t)> ___get_keyframe_timeStamp;
// Getters for the cell info
std::function<slam_mer::cell3d_t(cell_id_t)> ___get_cell;
// Getters for the neighbor info
std::function<neighbor_t::vector(point_id_t)> ___get_neighbors;

}  // namespace

// ---------------------------------------------------------------------------
// Initialize shared visualization data structures
// ---------------------------------------------------------------------------
void initiate_interface_attributes() {

  // --------------------------------------------------
  // Instantiate the points for visualization data structure, and
  // the vectors that will be used to store the new, updated and
  // deleted points.
  __newPointsId = set_t<point_id_t>::make_shared();
  __updatedPointsId = set_t<point_id_t>::make_shared();
  __deletedPointsId = set_t<point_id_t>::make_shared();

  // Same for the new cells to be added to the visualization
  __newCells = set_t<cell_id_t>::make_shared();
  __updateCells = set_t<cell_id_t>::make_shared();

  // Same for keyframes
  using hash_keyframe_points_t = hash_t<keyframe_id_t, array3D_t>;
  __visualizationKeyframes = hash_keyframe_points_t::make_shared();

  __newKeyframeId = set_t<keyframe_id_t>::make_shared();
  __updatedKeyframeId = set_t<keyframe_id_t>::make_shared();
  __deletedKeyframeId = set_t<keyframe_id_t>::make_shared();

  // Same for covisibility connections
  using set_id_pairs_t = set_t<keyframe_id_t::pair>;
  __newCovisibilityConnections = set_id_pairs_t::make_shared();
  __updatedCovisibilityConnections = set_id_pairs_t::make_shared();
  __deletedCovisibilityConnections = set_id_pairs_t::make_shared();
  // Same for the keyframes in buffer
  __keyframePerFrame = vector_t<vector_t<size_t>>::make_shared();

  // Same for the keyframe creation criteria
  using vector_divector_t = vector_t<dictionary_t<uint8_t, int>>;
  __keyframeCriteria = vector_divector_t::make_shared();
}

// ---------------------------------------------------------------------------
// Get pointers to shared visualization buffers and callbacks
// ---------------------------------------------------------------------------
PointersVisualization& get_pointers_visualization() {

  static PointersVisualization pointers;

  // Variables
  pointers.newPoints = __newPointsId;
  pointers.updatePoints = __updatedPointsId;
  pointers.hidePoint = __deletedPointsId;
  pointers.newCells = __newCells;
  pointers.updateCells = __updateCells;
  pointers.newKeyframes = __newKeyframeId;
  pointers.updateKeyframes = __updatedKeyframeId;
  pointers.deleteKeyframes = __deletedKeyframeId;
  pointers.newKeyframeConnections = __newCovisibilityConnections;
  pointers.updatedKeyframeConnections = __updatedCovisibilityConnections;
  pointers.deletedKeyframeConnections = __deletedCovisibilityConnections;
  pointers.keyframePerFrame = __keyframePerFrame;
  pointers.keyframeCriteria = __keyframeCriteria;

  // Methods
  pointers.get_point = ___get_point;
  pointers.get_keyframe = ___get_keyframe;
  pointers.get_cell = ___get_cell;
  pointers.get_neighbors = ___get_neighbors;
  pointers.get_keyframe_timeStamp = ___get_keyframe_timeStamp;

  // Return the pointers to the visualization variables
  return pointers;
}

// ---------------------------------------------------------------------------
// Register callbacks used by visualization wrappers
// ---------------------------------------------------------------------------
void set_getters_visualization(
    std::function<output_t<point3d_t>(point_id_t)> getPoint_,
    std::function<output_t<pose_t>(keyframe_id_t)> getKeyframe_,
    std::function<cell3d_t(cell_id_t)> getCell_,
    std::function<neighbor_t::vector(point_id_t)> getNeighbors_,
    std::function<time_stamp_t(keyframe_id_t)> getKeyframeTimestamp_) {
  ___get_point = getPoint_;
  ___get_keyframe = getKeyframe_;
  ___get_cell = getCell_;
  ___get_neighbors = getNeighbors_;
  ___get_keyframe_timeStamp = getKeyframeTimestamp_;
}

// ---------------------------------------------------------------------------
// Add a new covisibility connection to visualization buffers
// ---------------------------------------------------------------------------
void add_new_covisibility_connections(
    const keyframe_id_t keyframeId1_,
    const keyframe_id_t keyframeId2_) {

  if (keyframeId1_ == keyframeId2_)
    // We don't want to add a connection to itself
    __throw__(
        "Trying to add a connection to itself. Keyframe id: " +
        keyframeId1_.to_string());

  const auto newConnection = std::make_pair(keyframeId1_, keyframeId2_);
  __newCovisibilityConnections->insert_(newConnection);
}

// ---------------------------------------------------------------------------
// Add a new keyframe to visualization buffers
// ---------------------------------------------------------------------------
void add_new_keyframe_to_interface(const keyframe_id_t keyframeId_) {
  __newKeyframeId->insert_(keyframeId_);
}

// ---------------------------------------------------------------------------
// Mark keyframe as changed for visualization
// ---------------------------------------------------------------------------
void change_keyframe_to_interface(const keyframe_id_t keyframeId_) {
  __updatedKeyframeId->insert_(keyframeId_);
}

// ---------------------------------------------------------------------------
// Mark keyframe as hidden for visualization
// ---------------------------------------------------------------------------
void hide_keyframe_from_interface(const keyframe_id_t keyframeId_) {
  __deletedKeyframeId->insert_(keyframeId_);
}

// ---------------------------------------------------------------------------
// Add a new point to visualization buffers
// ---------------------------------------------------------------------------
void add_new_point_to_interface(const point_id_t pointId_) {
  __newPointsId->insert_(pointId_);
}

// ---------------------------------------------------------------------------
// Add a new cell to visualization buffers
// ---------------------------------------------------------------------------
void add_cell_to_interface(const cell_id_t cellId_) {
  __newCells->insert_(cellId_);
}

// ---------------------------------------------------------------------------
// Mark cell as changed for visualization
// ---------------------------------------------------------------------------
void change_cell_to_interface(const cell_id_t cellId_) {
  __updateCells->insert_(cellId_);
}

// ---------------------------------------------------------------------------
// Mark point as changed for visualization
// ---------------------------------------------------------------------------
void add_changed_point_to_interface(const point_id_t pointId_) {
  __updatedPointsId->insert_(pointId_);
}

// ---------------------------------------------------------------------------
// Mark point as hidden for visualization
// ---------------------------------------------------------------------------
void hide_point_from_interface(const point_id_t pointId_) {
  __deletedPointsId->insert_(pointId_);
}

// ---------------------------------------------------------------------------
// Store keyframes currently used by localization buffer
// ---------------------------------------------------------------------------
void set_keyframes_in_buffer(
    const std::map<keyframe_id_t, int>& keyframesInBuffer_,
    const keyframe_id_t maxKeyframeId_) {

  // Then save them in the vector
  vector_t<size_t> countMap;
  countMap.resize(maxKeyframeId_.key() + 1, 0);
  for (size_t id = 0; id <= maxKeyframeId_.key(); ++id) {
    const keyframe_id_t keyframeId(id);
    countMap[id] = keyframesInBuffer_.contains(keyframeId)
                       ? static_cast<size_t>(keyframesInBuffer_.at(keyframeId))
                       : static_cast<size_t>(0);
  }
  __keyframePerFrame->unique_lock();
  __keyframePerFrame->clear();  // We don't need more than one vector
  __keyframePerFrame->push_back(countMap);
  __keyframePerFrame->unlock_unique();
}

// ---------------------------------------------------------------------------
// Store keyframe creation metrics for visualization
// ---------------------------------------------------------------------------
void set_keyframe_creation_metrics(
    const dictionary_t<uint8_t, int> keyframeCreationCriteria_) {
  __keyframeCriteria->unique_lock();
  __keyframeCriteria->clear();  // We don't need more than one vector
  __keyframeCriteria->push_back(keyframeCreationCriteria_);
  __keyframeCriteria->unlock_unique();
}

}  // namespace map
}  // namespace slam_mer
