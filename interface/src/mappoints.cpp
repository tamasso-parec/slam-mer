/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file mappoints.cpp
/// @brief Implements map-point buffering and visualization synchronization.
/// @details Defines point add/update/hide and dense-cloud update routines.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

// My libraries
#include <interface_rerun.hpp>
#include <types/timeout_queue.hpp>

// Third-party libraries
#include <rerun.hpp>

// Standard libraries
#include <chrono>
#include <cstdlib>

namespace slam_mer {
namespace interface {

namespace {

// Since there might be unsync issues between new and update points,
// we keep a temporal queue of points that need to be added to the
// visualization after being updated.
timeout_queue_t<point_id_t> __pendingUpdate{duration_t::from_s(5)};

}  // namespace

/// Global variable to store the points to be logged in the visualization
extern std::vector<std::array<float, 3>> __points;
/// Global variable to store the colors of the points to be logged
extern std::vector<uint32_t> __colors;
/// Global variable to store the radii of the points to be logged
extern std::vector<float> __radii;
/// Mutex to protect access to the point cloud data
extern std::shared_mutex __pointCloud_mutex;

extern set_t<point_id_t>::shared_ptr __newPoints;
extern set_t<point_id_t>::shared_ptr __updatePoints;
extern set_t<point_id_t>::shared_ptr __hidePoint;

/// Query callback to retrieve a map point by ID.
extern std::function<output_t<point3d_t>(point_id_t)> ___get_point;

// Reference to the denser variables. These are defined in the
// helper_dense_cloud.cpp
/// Global variable to store the points in the dense point cloud
extern std::vector<std::array<float, 3>> __pointsDense;
/// Global variable to store the colors of the points in the dense point cloud
extern std::vector<uint32_t> __colorsDense;
/// Global variable to store the radii of the points in the dense point cloud
extern std::vector<float> __radiiDense;
/// Mutex to protect access to the dense point cloud data
extern std::shared_mutex __pointCloudDense_mutex;

/// External variable to store the cell data for the visualization interface
extern std::shared_ptr<rerun::RecordingStream> __recording;

// To avoid cluttering of the log of mappoints, we define here several
// helper functions to deal with the logging of points in the visualization
// These helpers are defined in the helper.cpp file.
/// @brief Add transformed neighbors around one anchor point to dense buffers.
/// @param anchorId_ Anchor point identifier.
/// @param anchor_ Anchor point data in world coordinates.
extern void __add_anchor_neighbors(
    const point_id_t& anchorId_,
    const point3d_t& anchor_);
/// @brief Update transformed neighbors around one anchor point in dense buffers.
/// @param anchorId_ Anchor point identifier.
/// @param anchor_ Anchor point data in world coordinates.
extern void __update_neighbors_point(
    const point_id_t& anchorId_,
    const point3d_t& anchor_);
/// @brief Hide all dense neighbors associated with one anchor.
/// @param pointId_ Anchor point identifier.
extern void __delete_neighbors_point(const point_id_t& pointId_);
/// @brief Check whether a point identifier is unset.
/// @param pointId_ Point identifier to validate.
/// @return `true` when `pointId_` is unset.
extern bool __is_unset(const point_id_t& pointId_);
/// @brief Insert one sparse point into interface buffers.
/// @param pointId_ Point identifier.
/// @param point3d_ Point geometry/color data.
extern void __insert_point(
    const point_id_t& pointId_,
    const point3d_t& point3d_);
/// @brief Update one sparse point already stored in interface buffers.
/// @param pointId_ Point identifier.
/// @param point3d_ Updated point geometry/color data.
extern void __update_point(
    const point_id_t& pointId_,
    const point3d_t& point3d_);
/// @brief Hide one sparse point in interface buffers.
/// @param pointId_ Point identifier.
extern void __delete_point(const point_id_t& pointId_);
/// @brief Check whether a point is absent from interface buffers.
/// @param pointId_ Point identifier.
/// @return `true` when the point is not present.
extern bool __not_contains(const point_id_t& pointId_);
/// @brief Check whether a point already exists in interface buffers.
/// @param pointId_ Point identifier.
/// @return `true` when the point is already present.
extern bool __contains(const point_id_t& pointId_);
/// @brief Handle retrieval failure for a point query.
/// @param error_ Failure reason returned by map query.
extern void __retrieval_error(fail_t& error_);
/// @brief Check whether a point query failed and emit the corresponding warning.
/// @param point_ Query output wrapper to validate.
/// @param pointId_ Point identifier being queried.
/// @return `true` when retrieval failed.
extern bool __update_query_fail(
    output_t<point3d_t>& point_,
    const point_id_t& pointId_);

/// @brief Synchronize sparse map-point add/update/delete queues.
void AbstractInterface::log_points() {

  // -------------------------------------------------------------------------
  // Pull the new points to be added to the visualization, and clear them
  // from the global variable
  // then we update the points in the interface variables
  // 0 - Check if the point is set. Otherwise, put it back to the new points
  // 1 - Iterate over all the new points.
  // 2 - If the point already exist in the visualization, skip it. (Warning)
  // 3 - If the point does not exist in the map, probably was deleted already.
  // we should skip it.
  // 4 - Otherwise, add the point to the visualization structures.
  const point_id_t::vector insert = __newPoints->to_vector();
  point_id_t::vector inserted;
  for (const point_id_t& pointId : insert) {
    if (__is_unset(pointId) || !__not_contains(pointId)) {
      inserted.push_back(pointId);
      continue;
    }

    output_t<point3d_t> point = ___get_point(pointId);
    if (point.fail()) {
      // Points are published only after map insertion. A missing ID here
      // was culled before visualization caught up and must not be retried.
      if (point.get_error() == fail_t::NOT_FOUND) {
        inserted.push_back(pointId);
        continue;
      }
      __retrieval_error(point.get_error());
      continue;
    }

    __insert_point(pointId, point);
    inserted.push_back(pointId);
    __add_anchor_neighbors(pointId, point);
  }
  __newPoints->clear_(inserted);

  // -------------------------------------------------------------------------
  // Next, we pull the list of points to be updated
  // and we update their position in the visualization structures
  // 1 - Iterate over all the points to be updated
  // 2 - If the point does not exist in the visualization, add it to
  // a temporal queue to be updated later. There might be a sync issue
  // with new and update.
  // 3 - Get the point from the SLAM system. If it does not exist, it means
  // it was deleted before being updated. Just in case, add it to the
  // deleted points list. (Send Warning)
  // 4 - Otherwise, update the point in the visualization structures.
  // 5 - After updating all the pulled from the map, we go over the points
  // on the pending update queue, and try to update them again.

  const point_id_t::vector update = __updatePoints->to_vector();
  point_id_t::vector updated;
  for (const point_id_t& pointId : update) {
    if (__not_contains(pointId)) continue;
    updated.push_back(pointId);
    output_t<point3d_t> point = ___get_point(pointId);
    if (point.fail() && point.get_error() == fail_t::NOT_FOUND) {
      __hidePoint->insert_(pointId);
      continue;
    }
    if (__update_query_fail(point, pointId)) {
      __pendingUpdate.insert(pointId);
      continue;
    }
    __update_point(pointId, point);
    __update_neighbors_point(pointId, point);
  }
  __updatePoints->clear_(updated);

  // Now we go over the pending update queue
  // 5.1 - For each point in the queue, check if it is now if can be inserted
  // 5.2 - Check if we can retrieve the point from the SLAM system
  // 5.3 - If both checks are ok, update the point in the visualization
  // structures, and remove it from the pending update queue.
  vector_t<point_id_t> pulledOut;
  for (const point_id_t& pointID : __pendingUpdate) {
    if (__not_contains(pointID)) continue;
    output_t<point3d_t> point = ___get_point(pointID);
    if (point.fail() && point.get_error() == fail_t::NOT_FOUND) {
      __hidePoint->insert_(pointID);
      pulledOut.push_back(pointID);
      continue;
    }
    if (__update_query_fail(point, pointID)) continue;
    __update_point(pointID, point);
    __update_neighbors_point(pointID, point);
    pulledOut.push_back(pointID);
  }
  __pendingUpdate.clear(pulledOut);

  // Finally, we clear the pending update queue of timed out points
  // All the points that could not be updated after several attempts
  // will be moved to the deleted points list.
  for (const point_id_t& pointID : __pendingUpdate.timed_out()) {
    __hidePoint->insert_(pointID);
  }
  __pendingUpdate.clear_timed_out();

  // -------------------------------------------------------------------------
  // Finally, we pull the list of points to be deleted from the visualization
  // and we delete them from the visualization structures
  // 1 - Iterate over all the points to be deleted
  // 2 - If the point does not exist in the visualization, skip it. It
  // means it was already deleted.
  // 3 - Otherwise, delete the point from the visualization structures.
  // 4 - Remove the point from the update and new points lists, in case
  // they are there.

  const point_id_t::vector hide = __hidePoint->to_vector();
  __hidePoint->clear_(hide);
  for (const point_id_t& pointId : hide) {
    __updatePoints->erase_(pointId);
    __newPoints->erase_(pointId);
    if (__not_contains(pointId)) continue;
    __delete_point(pointId);
    __delete_neighbors_point(pointId);
  }
}

/// @brief Publish sparse map points to the Rerun stream.
void InterfaceRerun::__log_points() {
  using rerun::Points3D;

  read_lock_t lock(__pointCloud_mutex);
  if (__points.empty()) return;
  __recording->log(
      "world/root/mappoints",
      Points3D(__points).with_colors(__colors).with_radii(__radii));
}

/// @brief Publish dense point cloud samples to the Rerun stream.
void InterfaceRerun::__log_dense_point_cloud() {
  using rerun::Points3D;

  // A recording retains every snapshot. Bound its map history rather than
  // serializing the complete growing map at the interactive refresh rate.
  const bool recording = std::getenv("SLAM_MER_RERUN_PATH") != nullptr;
  static auto lastRecorded = std::chrono::steady_clock::time_point{};
  const auto now = std::chrono::steady_clock::now();
  if (recording && now - lastRecorded < std::chrono::seconds(1)) return;

  read_lock_t lock(__pointCloudDense_mutex);
  if (__pointsDense.empty()) return;

  if (recording) {
    constexpr size_t maxPoints = 50000;
    const size_t stride = (__pointsDense.size() + maxPoints - 1) / maxPoints;
    std::vector<std::array<float, 3>> points;
    std::vector<uint32_t> colors;
    std::vector<float> radii;
    points.reserve(maxPoints);
    colors.reserve(maxPoints);
    radii.reserve(maxPoints);
    for (size_t i = 0; i < __pointsDense.size(); i += stride) {
      if (__radiiDense[i] <= 0) continue;
      points.push_back(__pointsDense[i]);
      colors.push_back(__colorsDense[i]);
      radii.push_back(__radiiDense[i]);
    }
    __recording->log(
        "scene/point_cloud",
        Points3D(points).with_colors(colors).with_radii(radii));
    lastRecorded = now;
    return;
  }

  __recording->log(
      "scene/point_cloud",
      Points3D(__pointsDense)
          .with_colors(__colorsDense)
          .with_radii(__radiiDense));
}

}  // namespace interface
}  // namespace slam_mer
