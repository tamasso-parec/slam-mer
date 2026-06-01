/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file interface_rerun.hpp
/// @brief Rerun-backed visualization interface declaration.
/// @details Declares `InterfaceRerun`, a concrete implementation of the
/// abstract interface that publishes map points, cells, trajectories, images,
/// and diagnostics to a Rerun recording stream.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

#ifndef INTERFACE_RERUN_HPP
#define INTERFACE_RERUN_HPP

#include <eigen3/Eigen/Core>

// Our includes
#include <interface.hpp>
#include <types/mutex.hpp>
#include <types/vector.hpp>
#include <types/verbose.hpp>
#include <types/visualization.hpp>

namespace slam_mer {
namespace interface {

/// @brief Concrete visualization backend that publishes data to Rerun.
/// @details Implements all backend hooks required by `AbstractInterface` and
/// owns the lifecycle of the shared Rerun recording stream.
class InterfaceRerun : public AbstractInterface {
public:
  /// @brief Access the singleton backend instance.
  /// @return Reference to the process-wide `InterfaceRerun` object.
  static InterfaceRerun& get_instance() {
    static InterfaceRerun instance;
    return instance;
  }

  /// @brief Initialize the Rerun stream and start interface worker loops.
  /// @param pointers_ Shared bindings/queues used by the interface loops.
  /// @param queryMapLogFrequency_ Map logging frequency in Hz.
  /// @param queryCameraLogFrequency_ Camera/diagnostic logging frequency in Hz.
  /// @param serverAddress_ Optional gRPC address. If empty, launches a local
  /// Rerun viewer process.
  void initiate(
      PointersVisualization pointers_,
      const double queryMapLogFrequency_ = 10.0,
      const double queryCameraLogFrequency_ = 10.0,
      const std::string& serverAddress_ = "");

  /// @brief Stop interface loops and release the Rerun recording stream.
  /// @return `true` when shutdown completed.
  bool stop_interface() override;

private:
  /// @brief Construct the singleton backend.
  InterfaceRerun() : AbstractInterface() {};

  /// @brief Destroy the backend and ensure shutdown.
  ~InterfaceRerun() { stop_interface(); };

  /// @brief Deleted copy constructor (singleton).
  InterfaceRerun(const InterfaceRerun&) = delete;

  /// @brief Deleted copy assignment operator (singleton).
  InterfaceRerun& operator=(const InterfaceRerun&) = delete;

  /// @brief Publish queued text messages.
  void __log_messages() override;

  /// @brief Publish RGB/depth/debug images.
  void __log_images() override;

  /// @brief Publish sparse map points.
  void __log_points() override;

  /// @brief Publish map cells/voxel boxes.
  void __log_cells() override;

  /// @brief Publish dense point cloud.
  void __log_dense_point_cloud() override;

  /// @brief Publish current camera transform and camera diagnostics.
  void __log_camera_location() override;

  /// @brief Publish localization histogram counts.
  void __log_histogram_counts() override;

  /// @brief Publish keyframe-creation criteria histogram.
  void __log_keyframe_criteria() override;

  /// @brief Publish covisibility graph edges.
  void __log_covisibility() override;
};

}  // namespace interface
}  // namespace slam_mer

#endif  // INTERFACE_RERUN_HPP
