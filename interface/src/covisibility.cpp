/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file covisibility.cpp
/// @brief Implements covisibility-edge and trajectory visualization updates.
/// @details Defines covisibility connection buffering and logging.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

// My libraries
#include <interface_rerun.hpp>
#include <settings.hpp>
#include <types/array.hpp>
#include <types/hash.hpp>

// Third-party libraries
#include <rerun.hpp>

// Standard libraries

namespace slam_mer {
namespace interface {
namespace {

/// Mutex to protect the global variables
std::shared_mutex __covisibility_mutex;
/// Global variable to store the list of keyframe covisibility connections
std::vector<std::vector<std::array<float, 3>>> __rerunKeyframeEdges;

// List of keyframe connections
/// Map the keyframe to its connections and id to the list of edges
hash_t<keyframe_id_t, vector_t<pair_t<keyframe_id_t, size_t>>>
    __keyframeToEdgeToIndex;
/// Map the keyframe to its connections and id to the list of edges -- Reversed
/// ordered
hash_t<keyframe_id_t, vector_t<pair_t<keyframe_id_t, size_t>>>
    __keyframeToEdgeToIndexInvert;

/// Iterator for the next id of the keyframe covisibility connection
size_t __nextCovisibilityId = 0;
/// Global iterator for the next id of the camera trajectory
size_t __nextCameraTrajectoryId = 0;

/// To store the index of the camera trajectory with respect to the SLAM id
hash_t<keyframe_id_t, size_t> __cameraTrajectoryIdToIndex;
/// List of hidden keyframes, in rerunCameraTrajectory_global ref.
set_t<size_t> __hiddenKeyframes;

}  // namespace

/// To return camera positions for vizualization
std::vector<std::array<float, 3>> __listKeyframes;
/// To return camera poses in double datatype
std::vector<std::array<double, 7>> __keyframesPose;
/// To return keyframe time stamps
std::vector<time_stamp_t> __keyframesTimeStamp;

// ----------------------------------------------------------------------
// These are external variables that are used to store the
// keyframes and their connections in the visualization interface.
// They are defined in the interface.cpp.
/// Global variable to store the new points needed to be logged
extern set_t<keyframe_id_t>::shared_ptr __newKeyframe;
/// Global variable to store the points to be updated
extern set_t<keyframe_id_t>::shared_ptr __updateKeyframe;
/// Global variable to store the points to be hidden
extern set_t<keyframe_id_t>::shared_ptr __hideKeyframe;
/// List of keyframe connections that need to be added
extern set_t<keyframe_id_t::pair>::shared_ptr __newKeyframeConnections;
/// List of keyframe connections that need to be updated
extern set_t<keyframe_id_t::pair>::shared_ptr __updatedKeyframeConnections;
/// List of keyframe connections that need to be deleted
extern set_t<keyframe_id_t::pair>::shared_ptr __deletedKeyframeConnections;
/// Query callback to retrieve keyframe pose by keyframe ID.
extern std::function<output_t<pose_t>(keyframe_id_t)> ___get_keyframe;
/// Query callback to retrieve keyframe timestamp by keyframe ID.
extern std::function<time_stamp_t(keyframe_id_t)> ___get_keyframe_timeStamp;

#if defined(DEBUG) || defined(SHOW_DENSE)
/// External variable to store the cell data for the visualization interface
extern std::shared_ptr<rerun::RecordingStream> __recording;
#endif

/// @brief Check whether a covisibility connection is between consecutive IDs.
/// @param connection_ Pair of connected keyframe IDs.
/// @return `true` when keyframes are consecutive.
bool __covisibility_consecutive(const keyframe_id_t::pair& connection_);
/// @brief Ensure adjacency structures exist for both keyframes in a connection.
/// @param connection_ Pair of connected keyframe IDs.
void __covisibility_contains(const keyframe_id_t::pair& connection_);
/// @brief Register a covisibility connection and assign a rerun edge index.
/// @param connection_ Pair of connected keyframe IDs.
void __covisibility_insert(const keyframe_id_t::pair& connection_);
/// @brief Check whether a keyframe is already present in trajectory buffers.
/// @param id_ Keyframe identifier.
/// @param warn_ Report an unexpected missing keyframe.
/// @return `true` when the keyframe exists in local interface state.
bool __sanity_exists(const keyframe_id_t& id_, bool warn_ = true);
/// @brief Validate that a pose query succeeded for a keyframe.
/// @param pose_ Pose query output.
/// @param id_ Keyframe identifier associated with `pose_`.
/// @return `true` when pose retrieval succeeded.
bool __sanity_pose(const output_t<pose_t>& pose_, const keyframe_id_t& id_);
/// @brief Append one covisibility edge geometry to rerun edge buffers.
/// @param pose1_ Pose of first keyframe.
/// @param pose2_ Pose of second keyframe.
void __insert_edge(const pose_t& pose1_, const pose_t& pose2_);
/// @brief Check whether there are no trajectory updates pending.
/// @return `true` when no keyframe insert/update/hide operations are pending.
bool __no_changes();
/// @brief Update an existing keyframe pose in trajectory buffers.
/// @param keyframeId_ Keyframe identifier to update.
/// @param position_ Updated keyframe position.
/// @param rotation_ Updated keyframe quaternion.
/// @param timeStamp_ Updated keyframe timestamp.
void __update_camera_pose(
    const keyframe_id_t& keyframeId_,
    const array3D_t& position_,
    const array4D_t& rotation_,
    const time_stamp_t& timeStamp_);
/// @brief Append a new keyframe pose to trajectory buffers.
/// @param keyframeId_ New keyframe identifier.
/// @param position_ Keyframe position.
/// @param quaternions_ Keyframe quaternion.
/// @param timeStamp_ Keyframe timestamp.
void __new_camera_pose(
    const keyframe_id_t& keyframeId_,
    const array3D_t& position_,
    const array4D_t& quaternions_,
    const time_stamp_t& timeStamp_);
/// @brief Update all stored edges connected to one moved keyframe.
/// @param keyframeId_ Moved keyframe identifier.
/// @param position_ Updated keyframe position.
void __update_covisibility_connections(
    const keyframe_id_t& keyframeId_,
    const array3D_t& position_);

/// @brief Synchronize new covisibility connections into interface edge buffers.
void log_covisibility_connections() {

  // Now process the new connections
  // 1 - Check whether the connection is between consecutive keyframes
  //     if so, skip it. Same if the pose does not exist
  // 2 - Ensure that both keyframes are in the maps
  // 3 - Insert the connection in the helpers
  // 4 - Insert the edge in the rerun structure
  const auto connectionsToAdd = __newKeyframeConnections->to_vector();
  vector_t<keyframe_id_t::pair> connectionsAdded;
  for (keyframe_id_t::pair connection : connectionsToAdd) {

    // if (__covisibility_consecutive(connection)) continue;
    if (!__sanity_exists(connection.first)) continue;
    if (!__sanity_exists(connection.second)) continue;

    const output_t<pose_t> pose1 = ___get_keyframe(connection.first);
    const output_t<pose_t> pose2 = ___get_keyframe(connection.second);
    if (!__sanity_pose(pose1, connection.first)) continue;
    if (!__sanity_pose(pose2, connection.second)) continue;

    __covisibility_contains(connection);
    __covisibility_insert(connection);
    __insert_edge(pose1, pose2);

    connectionsAdded.push_back(connection);
  }
  __newKeyframeConnections->clear_(connectionsAdded);
}

/// @brief Synchronize keyframe trajectory poses into interface buffers.
void log_poses() {

  // Exits early if there are no changes
  if (__no_changes()) return;

  // Checking for the new keyframes to be added
  // 1 - Pull the new keyframes and add them to the rerun camera trajectory
  // 2 - Iterate over the new keyframes
  // 3 - For each keyframe, get its pose
  // 4 - Add the pose to the rerun camera trajectory
  // 5 - Store the time stamp of the keyframe
  // 6 - Clear the list of new keyframes
  const keyframe_id_t::vector insert = __newKeyframe->to_vector();
  keyframe_id_t::vector inserted;
  for (const keyframe_id_t& keyframeId : insert) {

    if (__sanity_exists(keyframeId, false)) continue;
    const output_t<pose_t> poseOutput = ___get_keyframe(keyframeId);
    if (!__sanity_pose(poseOutput, keyframeId)) continue;

    const pose_t pose = poseOutput;
    const array3D_t position = pose.get_position();
    const auto [rotation, _] = pose.get_rotation_translation();
    const array4D_t quaternions = rotation.get_quaternions();
    const time_stamp_t timeStamp = ___get_keyframe_timeStamp(keyframeId);
    __new_camera_pose(keyframeId, position, quaternions, timeStamp);
    inserted.push_back(keyframeId);
  }
  __newKeyframe->clear_(inserted);

  // Checking for the keyframes to be updated
  // 1 - Pull the new keyframes and add them to the rerun camera trajectory
  // 2 - Iterate over the keyframes to be updated
  // 3 - For each keyframe, get its new pose
  // 4 - Update the pose in the rerun camera trajectory
  // 5 - Update the covisibility connections
  // 6 - Clear the list of keyframes to be updated
  const keyframe_id_t::vector update = __updateKeyframe->to_vector();
  keyframe_id_t::vector updated;
  for (const keyframe_id_t& keyframeId : update) {
    if (!__sanity_exists(keyframeId)) continue;
    const output_t<pose_t> poseOutput = ___get_keyframe(keyframeId);
    if (!__sanity_pose(poseOutput, keyframeId)) continue;
    const pose_t pose = poseOutput;
    const array3D_t position = pose.get_position();
    const auto [rotation, _] = pose.get_rotation_translation();
    const array4D_t quaternions = rotation.get_quaternions();
    const time_stamp_t timeStamp = ___get_keyframe_timeStamp(keyframeId);
    __update_camera_pose(keyframeId, position, quaternions, timeStamp);
    __update_covisibility_connections(keyframeId, position);
    updated.push_back(keyframeId);
  }
  __updateKeyframe->clear_(updated);
}

// ----------------------------------------------------------------------
// This function can be used to log the covisibility connections
// to the visualization.
// ----------------------------------------------------------------------

/// @brief Publish covisibility edges to the Rerun stream.
void InterfaceRerun::__log_covisibility() {
#ifdef DEBUG
  using rerun::LineStrips3D;
  using slam_mer::settings::sharedSettings;
  read_lock_t lock(__covisibility_mutex);
  if (!__recording->is_enabled()) return;
  if (__rerunKeyframeEdges.size()) {
    const auto color = sharedSettings->interface_covisibility_line_color();
    __recording->log(
        "world/root/edges",
        LineStrips3D(__rerunKeyframeEdges)
            .with_colors(rerun::Color(color[0], color[1], color[2], color[3]))
            .with_radii(
                sharedSettings->interface_covisibility_line_thickness()));
  }
#endif
}

// ----------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------

/// @brief Check whether a connection is between consecutive keyframe IDs.
/// @param connection_ Pair of connected keyframe IDs.
/// @return `true` when keyframes are consecutive.
bool __covisibility_consecutive(const keyframe_id_t::pair& connection_) {
  if (connection_.first == connection_.second + 1 ||
      connection_.first == connection_.second - 1)
    return true;
  return false;
}

/// @brief Ensure adjacency structures exist for both keyframes in a connection.
/// @param connection_ Pair of connected keyframe IDs.
void __covisibility_contains(const keyframe_id_t::pair& connection_) {
  if (!__keyframeToEdgeToIndex.contains(connection_.first)) {
    const vector_t<pair_t<keyframe_id_t, size_t>> empty;
    __keyframeToEdgeToIndex[connection_.first] = empty;
  }
  if (!__keyframeToEdgeToIndexInvert.contains(connection_.second)) {
    const vector_t<pair_t<keyframe_id_t, size_t>> empty;
    __keyframeToEdgeToIndexInvert[connection_.second] = empty;
  }
}

/// @brief Register a covisibility connection and assign a rerun edge index.
/// @param connection_ Pair of connected keyframe IDs.
void __covisibility_insert(const keyframe_id_t::pair& connection_) {
  __keyframeToEdgeToIndex[connection_.first].push_back(
      std::make_pair(connection_.second, __nextCovisibilityId));
  __keyframeToEdgeToIndexInvert[connection_.second].push_back(
      std::make_pair(connection_.first, __nextCovisibilityId));
  __nextCovisibilityId++;
}

/// @brief Check whether a keyframe is already present in trajectory buffers.
/// @param id_ Keyframe identifier.
/// @return `true` when the keyframe exists in local interface state.
bool __sanity_exists(const keyframe_id_t& id_, bool warn_) {
  using std::println;
  if (!__cameraTrajectoryIdToIndex.contains(id_)) {
    if (warn_)
      println(
          "[Rerun Interface] {}Warning{}: Keyframe id: {} not found.",
          yellow,
          reset,
          std::string(id_));
    return false;
  }
  return true;
}

/// @brief Validate that a pose query succeeded for a keyframe.
/// @param pose_ Pose query output.
/// @param id_ Keyframe identifier associated with `pose_`.
/// @return `true` when pose retrieval succeeded.
bool __sanity_pose(const output_t<pose_t>& pose_, const keyframe_id_t& id_) {
  using std::println;
  if (pose_.fail()) {
    println(
        "[Rerun Interface] {}Warning{}: Pose with id: {} not found.",
        yellow,
        reset,
        std::string(id_));
    return false;
  }
  return true;
}

/// @brief Append one covisibility edge geometry to rerun edge buffers.
/// @param pose1_ Pose of first keyframe.
/// @param pose2_ Pose of second keyframe.
void __insert_edge(const pose_t& pose1_, const pose_t& pose2_) {
  const array3D_t array1 = pose1_.get_position();
  const array3D_t array2 = pose2_.get_position();
  std::vector<std::array<float, 3>> lineStrip;
  lineStrip.push_back(
      {static_cast<float>(array1[0]),
       static_cast<float>(array1[1]),
       static_cast<float>(array1[2])});
  lineStrip.push_back(
      {static_cast<float>(array2[0]),
       static_cast<float>(array2[1]),
       static_cast<float>(array2[2])});
  {
    write_lock_t lock(__covisibility_mutex);
    __rerunKeyframeEdges.push_back(lineStrip);
  }
}

/// @brief Check whether there are no trajectory updates pending.
/// @return `true` when no keyframe insert/update/hide operations are pending.
bool __no_changes() {
  if (__newKeyframe->size() == 0 && __updateKeyframe->size() == 0 &&
      __hideKeyframe->size() == 0)
    return true;
  return false;
}

/// @brief Update an existing keyframe pose in trajectory buffers.
/// @param keyframeId_ Keyframe identifier to update.
/// @param position_ Updated keyframe position.
/// @param rotation_ Updated keyframe quaternion.
/// @param timeStamp_ Updated keyframe timestamp.
void __update_camera_pose(
    const keyframe_id_t& keyframeId_,
    const array3D_t& position_,
    const array4D_t& rotation_,
    const time_stamp_t& timeStamp_) {
  __listKeyframes[__cameraTrajectoryIdToIndex[keyframeId_]] = {
      static_cast<float>(position_[0]),
      static_cast<float>(position_[1]),
      static_cast<float>(position_[2])};
  __keyframesPose[__cameraTrajectoryIdToIndex[keyframeId_]] = {
      position_[0],
      position_[1],
      position_[2],
      rotation_[0],
      rotation_[1],
      rotation_[2],
      rotation_[3]};
  __keyframesTimeStamp[__cameraTrajectoryIdToIndex[keyframeId_]] = timeStamp_;
}

/// @brief Append a new keyframe pose to trajectory buffers.
/// @param keyframeId_ New keyframe identifier.
/// @param position_ Keyframe position.
/// @param quaternions_ Keyframe quaternion.
/// @param timeStamp_ Keyframe timestamp.
void __new_camera_pose(
    const keyframe_id_t& keyframeId_,
    const array3D_t& position_,
    const array4D_t& quaternions_,
    const time_stamp_t& timeStamp_) {
  // Add the pose to the list of poses
  __listKeyframes.push_back(
      {static_cast<float>(position_[0]),
       static_cast<float>(position_[1]),
       static_cast<float>(position_[2])});
  __keyframesPose.push_back(
      {position_[0],
       position_[1],
       position_[2],
       quaternions_[0],
       quaternions_[1],
       quaternions_[2],
       quaternions_[3]});
  __keyframesTimeStamp.push_back(timeStamp_);
  __cameraTrajectoryIdToIndex[keyframeId_] = __nextCameraTrajectoryId++;
}

/// @brief Update all stored edges connected to one moved keyframe.
/// @param keyframeId_ Moved keyframe identifier.
/// @param position_ Updated keyframe position.
void __update_covisibility_connections(
    const keyframe_id_t& keyframeId_,
    const array3D_t& position_) {

  const auto edgesIds = __keyframeToEdgeToIndex[keyframeId_];
  for (const auto& edgeId : edgesIds) {
    {
      write_lock_t lock(__covisibility_mutex);
      __rerunKeyframeEdges[edgeId.second][0] = {
          static_cast<float>(position_[0]),
          static_cast<float>(position_[1]),
          static_cast<float>(position_[2])};
    }
  }
  const auto edgesIdsInvert = __keyframeToEdgeToIndexInvert[keyframeId_];
  for (const auto& edgeId : edgesIdsInvert) {
    {
      write_lock_t lock(__covisibility_mutex);
      __rerunKeyframeEdges[edgeId.second][1] = {
          static_cast<float>(position_[0]),
          static_cast<float>(position_[1]),
          static_cast<float>(position_[2])};
    }
  }
}

}  // namespace interface
}  // namespace slam_mer
