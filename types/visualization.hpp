/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file visualization.hpp
/// @brief Shared data structures for visualization transport and rendering
/// interfaces.
/// @details Aggregates common visualization-oriented types and dependencies for
/// cells, poses, points, and timestamps. It provides a stable include
/// surface for UI and debug rendering code.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_VISUALIZATION_HPP
#define TYPE_VISUALIZATION_HPP

// Our includes
#include <types/cell.hpp>
#include <types/dictionary.hpp>
#include <types/id.hpp>
#include <types/mutex.hpp>
#include <types/output.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>
#include <types/set.hpp>
#include <types/time_stamp.hpp>
#include <types/vector.hpp>

// Standard libraries
#include <functional>

namespace slam_mer {

/// @brief Collection of visualization update buffers and lookup callbacks.
struct PointersVisualization {
  /// @brief Default constructor.
  PointersVisualization() = default;
  /// Points that should be created in the visualization.
  set_t<point_id_t>::shared_ptr newPoints;
  /// Points that should be updated in the visualization.
  set_t<point_id_t>::shared_ptr updatePoints;
  /// Points that should be hidden in the visualization.
  set_t<point_id_t>::shared_ptr hidePoint;
  /// Cells that should be created in the visualization.
  set_t<cell_id_t>::shared_ptr newCells;
  /// Cells that should be updated in the visualization.
  set_t<cell_id_t>::shared_ptr updateCells;
  /// Keyframes that should be created in the visualization.
  set_t<keyframe_id_t>::shared_ptr newKeyframes;
  /// Keyframes that should be updated in the visualization.
  set_t<keyframe_id_t>::shared_ptr updateKeyframes;
  /// Keyframes that should be removed from the visualization.
  set_t<keyframe_id_t>::shared_ptr deleteKeyframes;
  /// New keyframe-connection pairs to display.
  set_t<keyframe_id_t::pair>::shared_ptr newKeyframeConnections;
  /// Existing keyframe-connection pairs that changed.
  set_t<keyframe_id_t::pair>::shared_ptr updatedKeyframeConnections;
  /// Keyframe-connection pairs that should be removed.
  set_t<keyframe_id_t::pair>::shared_ptr deletedKeyframeConnections;
  /// Mapping of frame index to visible keyframe indices.
  vector_t<vector_t<size_t>>::shared_ptr keyframePerFrame;
  /// Per-keyframe criteria metadata used by the renderer.
  vector_t<dictionary_t<uint8_t, int>>::shared_ptr keyframeCriteria;

  /// Callback to fetch a point by ID.
  /// @param point_id_t Point identifier requested by the visualization layer.
  /// @return Point payload or failure information for that identifier.
  std::function<output_t<slam_mer::point3d_t>(point_id_t)> get_point;
  /// Callback to fetch a keyframe pose by ID.
  /// @param keyframe_id_t Keyframe identifier requested by the renderer.
  /// @return Pose payload or failure information for that keyframe.
  std::function<output_t<slam_mer::pose_t>(keyframe_id_t)> get_keyframe;
  /// Callback to fetch a cell by ID.
  /// @param cell_id_t Cell identifier requested for visualization update.
  /// @return Cell data associated with the requested identifier.
  std::function<slam_mer::cell3d_t(cell_id_t)> get_cell;
  /// Callback to fetch neighbors of a point.
  /// @param point_id_t Point identifier whose neighbor list is requested.
  /// @return Neighbor descriptors linked to the requested point.
  std::function<slam_mer::neighbor_t::vector(point_id_t)> get_neighbors;
  /// Callback to fetch the timestamp of a keyframe.
  /// @param keyframe_id_t Keyframe identifier whose timestamp is requested.
  /// @return Timestamp associated with the requested keyframe.
  std::function<slam_mer::time_stamp_t(keyframe_id_t)> get_keyframe_timeStamp;
};

}  // namespace slam_mer

#endif  // TYPE_VISUALIZATION_HPP
