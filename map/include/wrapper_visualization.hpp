/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file wrapper_visualization.hpp
/// @brief Visualization-wrapper API for buffering map update events.
/// @details Declares functions that collect point, cell, keyframe, and
/// covisibility updates into shared buffers consumed by visualization clients.
/// It also exposes callback registration and state-access helpers.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef WRAPPER_VISUALIZATION_HPP
#define WRAPPER_VISUALIZATION_HPP

// My includes
#include <types/cell.hpp>
#include <types/dictionary.hpp>
#include <types/mutex.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>
#include <types/time_stamp.hpp>
#include <types/vector.hpp>
#include <types/visualization.hpp>

// Standard C++ includes
#include <map>

namespace slam_mer {
namespace map {

/// @brief Initialize shared visualization buffers and synchronization state.
/// @details Call this before any other function in this header.
void initiate_interface_attributes();

/// @brief Get shared pointers to visualization buffer structures.
/// @return Reference to the global `PointersVisualization` state container.
[[nodiscard(
    "Use returned visualization pointers to access interface update buffers")]]
PointersVisualization& get_pointers_visualization();

/// @brief Register map getter callbacks used by the visualization wrapper.
/// @param getPoint_ Callback that retrieves point data for a point ID.
/// @param getKeyframe_ Callback that retrieves keyframe pose for a keyframe ID.
/// @param getCell_ Callback that retrieves cell data for a cell ID.
/// @param getNeighbors_ Callback that retrieves neighbors for a point ID.
/// @param getKeyframeTimestamp_ Callback that retrieves keyframe timestamp.
void set_getters_visualization(
    std::function<output_t<point3d_t>(point_id_t)> getPoint_,
    std::function<output_t<pose_t>(keyframe_id_t)> getKeyframe_,
    std::function<slam_mer::cell3d_t(cell_id_t)> getCell_,
    std::function<neighbor_t::vector(point_id_t)> getNeighbors_,
    std::function<time_stamp_t(keyframe_id_t)> getKeyframeTimestamp_);

// ------------------------------------------------------------------------
// Functions that will be used to update the visualization interface.
// These are supposed to be used by the map to update the visualization.

/// @brief Add a newly created map point to visualization buffers.
/// @param pointId_ Point identifier to publish.
void add_new_point_to_interface(const point_id_t pointId_);

/// @brief Mark an existing map point as changed for visualization updates.
/// @param pointId_ Point identifier to publish as changed.
void add_changed_point_to_interface(const point_id_t pointId_);

/// @brief Mark a map point as hidden in visualization buffers.
/// @param pointId_ Point identifier to hide.
void hide_point_from_interface(const point_id_t pointId_);

/// @brief Add a cell entry to visualization buffers.
/// @param cellId_ Cell identifier to publish.
void add_cell_to_interface(const cell_id_t cellId_);

/// @brief Mark a cell as changed for visualization updates.
/// @param cellId_ Cell identifier to publish as changed.
void change_cell_to_interface(const cell_id_t cellId_);

/// @brief Add a newly created keyframe to visualization buffers.
/// @param keyframeId_ Keyframe identifier to publish.
void add_new_keyframe_to_interface(const keyframe_id_t keyframeId_);

/// @brief Mark an existing keyframe as changed for visualization updates.
/// @param keyframeId_ Keyframe identifier to publish as changed.
void change_keyframe_to_interface(const keyframe_id_t keyframeId_);

/// @brief Mark a keyframe as hidden in visualization buffers.
/// @param keyframeId_ Keyframe identifier to hide.
void hide_keyframe_from_interface(const keyframe_id_t keyframeId_);

/// @brief Add a covisibility edge between two keyframes.
/// @param keyframeId1_ First keyframe identifier.
/// @param keyframeId2_ Second keyframe identifier.
void add_new_covisibility_connections(
    const keyframe_id_t keyframeId1_,
    const keyframe_id_t keyframeId2_);

/// @brief Set keyframes currently stored in the localization buffer.
/// @param keyframesInBuffer_ Map of keyframe IDs and associated buffer slots.
/// @param maxKeyframeId_ Highest keyframe ID currently known by the map.
void set_keyframes_in_buffer(
    const std::map<keyframe_id_t, int>& keyframesInBuffer_,
    const keyframe_id_t maxKeyframeId_);

/// @brief Set keyframe-creation metrics for visualization overlays.
/// @param keyframeCreationCriteria_ Dictionary of metric IDs and metric values.
void set_keyframe_creation_metrics(
    const dictionary_t<uint8_t, int> keyframeCreationCriteria_);

}  // namespace map
}  // namespace slam_mer

#endif  // WRAPPER_VISUALIZATION_HPP
