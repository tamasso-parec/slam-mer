/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file interface.hpp
/// @brief Abstract visualization interface used by SLAM frontends/backends.
/// @details Declares the base visualization API, shared update-loop controls,
/// and common data structures used to connect map/localization outputs to
/// concrete rendering backends such as Rerun.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

#ifndef INTERFACE_HPP
#define INTERFACE_HPP

// Out includes
#include <debug_data.hpp>
#include <types/mutex.hpp>
#include <types/rotation.hpp>
#include <types/translation.hpp>
#include <types/vector.hpp>
#include <types/verbose.hpp>
#include <types/visualization.hpp>

// Standard libraries
#include <thread>

namespace slam_mer {
namespace interface {

/// @brief Retrieve cached keyframe positions for visualization.
/// @return Reference to 3D keyframe positions as `{x, y, z}` in world
/// coordinates.
const std::vector<std::array<float, 3>>& get_list_keyframes();

/// @brief Retrieve cached keyframe poses for trajectory visualization.
/// @return Reference to keyframe poses as `{qx, qy, qz, qw, tx, ty, tz}`.
const std::vector<std::array<double, 7>>& get_keyframes_pose();

/// @brief Retrieve cached dense-map points.
/// @return Reference to dense point cloud positions as `{x, y, z}`.
const std::vector<std::array<float, 3>>& get_points_dense();

/// @brief Retrieve cached per-point colors for the dense map.
/// @return Reference to packed color values aligned with `get_points_dense()`
/// ordering.
const std::vector<uint32_t>& get_colors_dense();

/// @brief Retrieve cached keyframe timestamps for temporal visualization.
/// @return Reference to keyframe timestamps aligned with `get_list_keyframes()`
/// ordering.
const std::vector<time_stamp_t>& get_keyframes_time_stamp();

/// @brief Abstract base class for visualization backends.
/// @details Owns common worker loops and shared wiring between SLAM state
/// producers and concrete visualization sinks.
class AbstractInterface {

public:
  /// @brief Construct the base interface object.
  AbstractInterface() = default;
  /// @brief Destroy the interface object.
  virtual ~AbstractInterface() = default;

  /// @brief Initialize data bindings and start interface worker threads.
  /// @param pointers_ Collection of shared queues/functions used to fetch
  /// map, keyframe, and debug data from the SLAM pipeline.
  /// @param queryMapLogFrequency_ Target frequency (Hz) of the map logging
  /// loop (points/cells/cloud updates).
  /// @param queryCameraLogFrequency_ Target frequency (Hz) of the camera and
  /// diagnostics logging loop.
  void initiate(
      PointersVisualization pointers_,
      const double queryMapLogFrequency_ = 10.0,
      const double queryCameraLogFrequency_ = 10.0);

  /// @brief Stop worker threads and release shared interface resources.
  /// @return `true` when shutdown completed.
  virtual bool stop_interface();

private:
  /// @brief Emit queued textual messages to the backend.
  virtual void __log_messages() = 0;
  /// @brief Emit latest image/depth/debug frames to the backend.
  virtual void __log_images() = 0;
  /// @brief Emit dense point cloud data to the backend.
  virtual void __log_dense_point_cloud() = 0;
  /// @brief Emit sparse map points to the backend.
  virtual void __log_points() = 0;
  /// @brief Emit voxel/cell primitives to the backend.
  virtual void __log_cells() = 0;
  /// @brief Emit camera transform/state to the backend.
  virtual void __log_camera_location() = 0;
  /// @brief Emit localization histogram metrics to the backend.
  virtual void __log_histogram_counts() = 0;
  /// @brief Emit keyframe-creation criteria metrics to the backend.
  virtual void __log_keyframe_criteria() = 0;
  /// @brief Emit covisibility graph edges to the backend.
  virtual void __log_covisibility() = 0;

protected:
  /// @brief Synchronize sparse map-point buffers from SLAM to interface state.
  void log_points();
  /// @brief Synchronize map-cell buffers from SLAM to interface state.
  void log_cells();

  /// @brief Thread body that publishes map-layer visuals at a fixed rate.
  /// @param token_ Cooperative stop token used to terminate the loop.
  void __map_loop(std::stop_token token_);
  /// @brief Thread body that publishes camera/images/diagnostics at a fixed
  /// rate.
  /// @param token_ Cooperative stop token used to terminate the loop.
  void __camera_loop(std::stop_token token_);
  /// @brief Thread body that refreshes interface-side caches from shared queues.
  /// @param token_ Cooperative stop token used to terminate the loop.
  void __internal_update_loop(std::stop_token token_);

  /// Updating threads
  std::jthread _mapLogThread;
  /// Lunch the rerun log loop for the camera in a separate thread
  std::jthread _cameraLogThread;
  /// Updating threads
  std::jthread _updateDataThread;
};

/// @brief Cache one RGB source frame for asynchronous visualization.
/// @param data Debug payload containing the source RGB image and metadata.
void log_source_image(const utils::ImageDebugData& data);

/// @brief Cache one source depth frame for asynchronous visualization.
/// @param data Debug payload containing the source depth image.
void log_source_depth_image(const utils::ImageDebugData& data);

/// @brief Cache one debug/overlay frame and associated counters.
/// @param data Debug payload containing rendered debug image, feature sets,
/// and per-frame metrics.
void log_debug_image(const utils::ImageDebugData& data);

/// @brief Cache the latest camera pose for visualization logging.
/// @param rotation_ Camera orientation in world coordinates.
/// @param translation_ Camera position in world coordinates.
void log_camera_location(rotation_t rotation_, translation_t translation_);

/// @brief Synchronize keyframe trajectory poses from SLAM to interface buffers.
/// @details Processes new/updated keyframes and updates trajectory/camera pose
/// structures used by the backend logger.
void log_poses();

/// @brief Synchronize covisibility edge updates from SLAM to interface buffers.
void log_covisibility_connections();

/// @brief Enqueue one text message for deferred backend emission.
/// @param message_ Message content to be displayed in the visualization UI.
/// @param verboseType_ Message severity used to select backend log level.
void log_messages_global(
    const std::string& message_,
    VerboseLevels verboseType_);

}  // namespace interface
}  // namespace slam_mer

#endif  // INTERFACE_HPP
