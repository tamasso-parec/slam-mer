/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file helper.cpp
/// @brief Implements shared helper state for interface point-cloud handling.
/// @details Defines global helper buffers and indexing structures.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

// My libraries
#include <interface_rerun.hpp>
#include <settings.hpp>
#include <types/dictionary.hpp>
#include <types/hash.hpp>

// Third-party libraries

// Standard libraries

namespace slam_mer {
namespace interface {

namespace {

/// Global variable to store the index of the points on the interface
hash_t<point_id_t, size_t> __pointIdToIndex;
/// Next available index for a new point
size_t __nextAvailableIndex = 0;

}  // namespace

// External variables that are used to store the
// points and their connections in the visualization interface.
// Check interface.cpp for the definition of these variables.
extern std::function<neighbor_t::vector(point_id_t)> ___get_neighbors;
extern std::function<output_t<pose_t>(keyframe_id_t)> ___get_keyframe;

/// Global variable to store the points to be logged in the visualization
std::vector<std::array<float, 3>> __points;
/// Global variable to store the colors of the points to be logged (RGBA32)
std::vector<uint32_t> __colors;
/// Global variable to store the radii of the points to be logged
std::vector<float> __radii;
/// Mutex to protect access to the point cloud data
std::shared_mutex __pointCloud_mutex;

/// Map from neighbor point id to index in the neighbor array
hash_t<point_id_t, dictionary_t<point_id_t, size_t>> __neighborIdToIndex;
/// Iterator for the next id of the point on the dense point cloud
size_t __nextIndexDense = 0;
/// Global variable to store the points to be logged in the visualization
std::vector<std::array<float, 3>> __pointsDense;
/// Global variable to store the colors of the points to be logged (RGBA32)
std::vector<uint32_t> __colorsDense;
/// Global variable to store the radii of the points to be logged
std::vector<float> __radiiDense;
/// Mutex to protect access to the dense point cloud data
std::shared_mutex __pointCloudDense_mutex;

// -------------------------------------------------------------------------
// helper functions

/// @brief Validate that an anchor point ID is set.
/// @param pointId_ Anchor point identifier.
/// @return `true` when `pointId_` is valid (not UNSET).
[[nodiscard("Use to check if the point ID is valid")]]
bool __sanity_anchor_id(const point_id_t& pointId_) {
  using std::println;
  if (pointId_ == point_id_t::UNSET) {
    println(
        "[Rerun Interface] {}Warning{}: Anchor point ID is UNSET.",
        yellow,
        reset);
    return false;
  }
  return true;
}

/// @brief Check whether an anchor already tracks a given neighbor.
/// @param pointId_ Anchor point identifier.
/// @param neighborId_ Neighbor point identifier.
/// @return `true` when the neighbor index already exists.
[[nodiscard("Use to check if the neighbor index already exists")]]
bool __neighbor_exists(
    const point_id_t& pointId_,
    const point_id_t& neighborId_) {
  return __neighborIdToIndex[pointId_].contains(neighborId_);
}

/// @brief Check whether dense-neighbor bookkeeping exists for an anchor.
/// @param pointId_ Anchor point identifier.
/// @return `true` when the anchor has a neighbor dictionary entry.
[[nodiscard("Use to check if the anchor has a neighbor dictionary entry")]]
bool __anchor_exists(const point_id_t& pointId_) {
  return __neighborIdToIndex.contains(pointId_);
}

/// @brief Add one transformed dense neighbor sample for an anchor.
/// @param pointId_ Anchor point identifier.
/// @param neighborId_ Neighbor point identifier.
/// @param neighbor_ Neighbor position/color encoded as `{x,y,z,r,g,b}`.
void __add_neighbor(
    const point_id_t& pointId_,
    const point_id_t& neighborId_,
    const array6D_t& neighbor_) {
  using slam_mer::settings::sharedSettings;
  // This functions will:
  // 1 - Add the point to the dense point cloud
  // 2 - Store the index of the point in the neighborIdToIndex map
  write_lock_t lock(__pointCloudDense_mutex);
  __pointsDense.push_back(
      {static_cast<float>(neighbor_[0]),
       static_cast<float>(neighbor_[1]),
       static_cast<float>(neighbor_[2])});
  // RGBA32 format
  __colorsDense.push_back(
      (uint32_t)neighbor_[3] << 24 | (uint32_t)neighbor_[4] << 16 |
      (uint32_t)neighbor_[5] << 8 | 255);
  __radiiDense.push_back(sharedSettings->interface_dense_point_size());
  __neighborIdToIndex[pointId_][neighborId_] = __nextIndexDense++;
}

/// @brief Update one transformed dense neighbor sample for an anchor.
/// @param pointId_ Anchor point identifier.
/// @param neighborId_ Neighbor point identifier.
/// @param neighbor_ Neighbor position/color encoded as `{x,y,z,r,g,b}`.
void __update_neighbor(
    const point_id_t& pointId_,
    const point_id_t& neighborId_,
    const array6D_t& neighbor_) {
  size_t index = __neighborIdToIndex[pointId_][neighborId_];
  write_lock_t lock(__pointCloudDense_mutex);
  __pointsDense[index] = {
      static_cast<float>(neighbor_[0]),
      static_cast<float>(neighbor_[1]),
      static_cast<float>(neighbor_[2])};
  __colorsDense[index] = (uint32_t)neighbor_[3] << 24 |
                         (uint32_t)neighbor_[4] << 16 |
                         (uint32_t)neighbor_[5] << 8 | 255;
}

/// @brief Transform a neighbor point from keyframe-local to world coordinates.
/// @param pose_ Keyframe camera-to-world pose used for transformation.
/// @param point_ Neighbor point in keyframe-local coordinates.
/// @param anchor_ Anchor position used as world-space offset.
/// @return Transformed point encoded as `{x,y,z,r,g,b}`.
[[nodiscard(
    "Use to transform a neighbor point from keyframe-local to \
    world coordinates")]]
array6D_t __transform_neighbor(
    const pose_t& pose_,
    const point3d_t& point_,
    const array3D_t& anchor_) {
  const auto [rotation, _] = pose_.get_rotation_translation();
  const point3d_t& rotatedPoint = rotation * point_;
  array6D_t pointArray = rotatedPoint.to_6d_array();
  pointArray[0] += anchor_[0];
  pointArray[1] += anchor_[1];
  pointArray[2] += anchor_[2];
  return pointArray;
}

/// @brief Hide one dense neighbor sample associated with an anchor.
/// @param pointId_ Anchor point identifier.
/// @param neighborId_ Neighbor point identifier.
void __delete_neighbor(
    const point_id_t& pointId_,
    const point_id_t& neighborId_) {
  size_t index = __neighborIdToIndex[pointId_][neighborId_];
  write_lock_t lock(__pointCloudDense_mutex);
  __radiiDense[index] = 0.0f;
  __pointsDense[index] = {0.0f, 0.0f, 0.0f};
}

/// @brief Throw when keyframe pose retrieval fails.
/// @param fail_ Failure object returned by keyframe query.
/// @param keyframeId_ Keyframe identifier used in the exception message.
void __keyframe_sanity(const fail_t& fail_, const keyframe_id_t& keyframeId_) {
  if (fail_.fail())
    __throw__(
        "Keyframe ID " + std::string(keyframeId_) +
        " not found in get_keyframe.");
}

/// @brief Build dense neighbors for one newly inserted anchor point.
/// @param anchorId_ Anchor point identifier.
/// @param anchor_ Anchor point data used as transformation offset.
extern void __add_anchor_neighbors(
    const point_id_t& anchorId_,
    const point3d_t& anchor_) {

  // This function will add the dense point to the point cloud
  // 1 - Get the neighbors of the point
  // 2 - For each neighbor, transform it to the world frame
  // 3 - Add the transformed point to the point cloud
  if (__sanity_anchor_id(anchorId_) == false) return;
  if (__anchor_exists(anchorId_) == false) {
    __neighborIdToIndex.insert({anchorId_, dictionary_t<point_id_t, size_t>()});
  }

  const neighbor_t::vector& neighbors = ___get_neighbors(anchorId_);

  neighbors.shared_lock();
  for (const neighbor_t& neighbor : neighbors) {
    const point_id_t& neighborId = neighbor;
    const point3d_t& neighborPoint = neighbor;
    const keyframe_id_t& keyframeId = neighbor;
    const output_t<pose_t> keyframePose = ___get_keyframe(keyframeId);
    __keyframe_sanity(keyframePose, keyframeId);

    const array6D_t& neighborWorld =
        __transform_neighbor(keyframePose, neighborPoint, anchor_);
    __add_neighbor(anchorId_, neighborId, neighborWorld);
  }
  neighbors.unlock_shared();
}

/// @brief Refresh dense neighbors for one existing anchor point.
/// @param anchorId_ Anchor point identifier.
/// @param anchor_ Anchor point data used as transformation offset.
extern void __update_neighbors_point(
    const point_id_t& anchorId_,
    const point3d_t& anchor_) {

  // This function will update the dense point in the point cloud
  // 1 - Get the neighbors of the point
  // 2 - For each neighbor, transform it to the world frame
  // 3 - Update the transformed point in the point cloud
  if (__sanity_anchor_id(anchorId_) == false) return;
  if (__anchor_exists(anchorId_) == false) return;

  const neighbor_t::vector& neighbors = ___get_neighbors(anchorId_);

  neighbors.shared_lock();
  for (const neighbor_t& neighbor : neighbors) {
    const point_id_t& neighborId = neighbor;
    const point3d_t& neighborPoint = neighbor;
    const keyframe_id_t& keyframeId = neighbor;
    const output_t<pose_t> keyframePose = ___get_keyframe(keyframeId);
    __keyframe_sanity(keyframePose, keyframeId);

    const array6D_t& neighborWorld =
        __transform_neighbor(keyframePose, neighborPoint, anchor_);

    if (__neighbor_exists(anchorId_, neighborId) == false)
      __add_neighbor(anchorId_, neighborId, neighborWorld);
    else
      __update_neighbor(anchorId_, neighborId, neighborWorld);
  }
  neighbors.unlock_shared();
}

/// @brief Hide all dense neighbors associated with an anchor point.
/// @param pointId_ Anchor point identifier.
extern void __delete_neighbors_point(const point_id_t& pointId_) {

  // Simple function to delete all neighbors of a point
  // 1 - Check if the point exists
  // 2 - If the point does not exist, it was removed already, so do nothing
  // 3 - For each neighbor, delete the point from the point cloud
  if (__sanity_anchor_id(pointId_) == false) return;
  if (__anchor_exists(pointId_) == false) return;

  const dictionary_t<point_id_t, size_t>& neighborDict =
      __neighborIdToIndex[pointId_];

  for (const auto& [neighborId, index] : neighborDict)
    __delete_neighbor(pointId_, neighborId);

  __neighborIdToIndex.erase(pointId_);
}

/// @brief Check whether a point identifier is unset.
/// @param pointId_ Point identifier to validate.
/// @return `true` when `pointId_` is UNSET.
[[nodiscard("Use to check if the point ID is valid")]]
bool __is_unset(const point_id_t& pointId_) {
  using std::println;
  if (pointId_ == point_id_t::UNSET) {
    println(
        "[Rerun Interface] {}Warning{}: Trying to process an UNSET point ID.",
        yellow,
        reset);
    return true;
  }
  return false;
}

/// @brief Insert one sparse map point into interface buffers.
/// @param pointId_ Point identifier.
/// @param point3d_ Point geometry and color.
void __insert_point(const point_id_t& pointId_, const point3d_t& point3d_) {
  // To insert a point into the global visualization structures
  // 1 - Add the point coordinates to the points vector
  // 2 - Add the point color to the colors vector
  // 3 - Add the point radius to the radii vector
  // 4 - Map the point ID to the index in the vectors
  using slam_mer::settings::sharedSettings;
  const array6D_t pointArray = point3d_;
  write_lock_t lock(__pointCloud_mutex);
  __points.push_back(
      {static_cast<float>(pointArray[0]),
       static_cast<float>(pointArray[1]),
       static_cast<float>(pointArray[2])});
  __colors.push_back(
      (uint32_t)pointArray[3] << 24 | (uint32_t)pointArray[4] << 16 |
      (uint32_t)pointArray[5] << 8 | 255);
  __radii.push_back(sharedSettings->interface_sparse_point_size());
  __pointIdToIndex[pointId_] = __nextAvailableIndex++;
}

/// @brief Update one sparse map point already stored in interface buffers.
/// @param pointId_ Point identifier.
/// @param point3d_ Updated point geometry and color.
void __update_point(const point_id_t& pointId_, const point3d_t& point3d_) {
  // Help with updating a point in the global visualization
  // 1 - Get the index of the point in the vectors
  // 2 - Update the point coordinates in the points vector
  size_t index = __pointIdToIndex[pointId_];
  const array6D_t pointArray = point3d_;
  write_lock_t lock(__pointCloud_mutex);
  __points[index] = {
      static_cast<float>(pointArray[0]),
      static_cast<float>(pointArray[1]),
      static_cast<float>(pointArray[2])};
}

/// @brief Hide one sparse map point in interface buffers.
/// @param pointId_ Point identifier.
void __delete_point(const point_id_t& pointId_) {
  // Help with deleting a point from the global visualization
  // 1 - Get the index of the point in the vectors
  // 2 - Set the radius of the point to zero (it will not be rendered
  // anymore)
  // 3 - Move the coordinates to (0,0,0) to avoid any issues with rendering
  size_t index = __pointIdToIndex[pointId_];
  write_lock_t lock(__pointCloud_mutex);
  __radii[index] = 0.0f;
  __points[index] = {0.0f, 0.0f, 0.0f};
}

/// @brief Check whether a point is absent from sparse visualization buffers.
/// @param pointId_ Point identifier.
/// @return `true` when the point is not currently buffered.
[[nodiscard(
    "Use to check if the point ID is not present in \
    visualization buffers")]]
bool __not_contains(const point_id_t& pointId_) {
  // Method to check if point is not contained in the visualization
  if (__pointIdToIndex.contains(pointId_) == false) {
    return true;
  }
  return false;
}

/// @brief Check whether a point is already buffered for visualization.
/// @param pointId_ Point identifier.
/// @return `true` when the point already exists in interface buffers.
[[nodiscard(
    "Use to check if the point ID is already present in \
    visualization buffers")]]
bool __contains(const point_id_t& pointId_) {
  using std::println;
  if (__pointIdToIndex.contains(pointId_) == true) {
    println(
        "[Rerun Interface] {}Warning{}: Point already exists ID: {}",
        yellow,
        reset,
        std::string(pointId_));
    return true;
  }
  return false;
}

/// @brief Handle point retrieval errors with warning or exception.
/// @param error_ Failure reason from map retrieval.
void __retrieval_error(fail_t& error_) {
  using std::println;
  if (error_ == fail_t::NOT_FOUND)
    println(
        "[Rerun Interface] {}Warning{}: Could not retrieve. Probably deleted.",
        yellow,
        reset);
  else
    __throw__(
        "[Rerun Interface] Error retrieving new point. Error: " +
        error_.get_message());
}

/// @brief Validate point query output and emit retrieval diagnostics.
/// @param point_ Point query output to validate.
/// @param pointId_ Point identifier associated with `point_`.
/// @return `true` when query failed.
[[nodiscard("Use to check if point retrieval failed")]]
bool __update_query_fail(
    output_t<point3d_t>& point_,
    const point_id_t& pointId_) {
  if (point_.fail()) {
    __retrieval_error(point_.get_error());
    return true;
  }
  return false;
};

}  // namespace interface
}  // namespace slam_mer
