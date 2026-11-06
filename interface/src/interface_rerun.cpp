/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file interface_rerun.cpp
/// @brief Implements Rerun backend startup, shutdown, and stream ownership.
/// @details Defines Rerun recording initialization and teardown behavior.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

#include <cstdlib>
#include <interface_rerun.hpp>
#include <rerun.hpp>

namespace slam_mer {
namespace interface {

/// Shared recording stream used by all Rerun logging helpers.
std::shared_ptr<rerun::RecordingStream> __recording = nullptr;

/// @brief Stop the backend and release the active recording stream.
/// @return `true` when shutdown has completed.
bool InterfaceRerun::stop_interface() {
  AbstractInterface::stop_interface();
  __recording.reset();
  return true;
}

/// @brief Initialize Rerun output and start base interface loops.
/// @param pointers_ Struct with shared data sources and query callbacks.
/// @param queryMapLogFrequency_ Target map logging frequency in Hz.
/// @param queryCameraLogFrequency_ Target camera logging frequency in Hz.
/// @param serverAddress_ Optional Rerun gRPC endpoint; empty launches a local
/// viewer.
void InterfaceRerun::initiate(
    PointersVisualization pointers_,
    const double queryMapLogFrequency_,
    const double queryCameraLogFrequency_,
    const std::string& serverAddress_) {
  using rerun::Error;
  using rerun::Pinhole;
  using rerun::ViewCoordinates;

  // Columns express a visualization world's +X forward, +Y left, +Z up
  // axes in the unchanged optical SLAM coordinates (right, down, forward).
  // Rerun applies the inverse when this child frame is a view's origin.
  const rerun::datatypes::Vec3D worldAxes[3] = {
      {0.0F, 0.0F, 1.0F},
      {-1.0F, 0.0F, 0.0F},
      {0.0F, -1.0F, 0.0F}};

  // 1 - Check if the interface has already been initialized
  // 2 - Create the rerun recording stream
  // 3 - Set up the rerun logging parameters
  // 4 - Call the base class initiate function

  if (__recording != nullptr)
    __throw__("InterfaceRerun: Rerun interface already initialized.");

  __recording = std::make_shared<rerun::RecordingStream>("rerun_example_cpp");

  const char* recordingPath = std::getenv("SLAM_MER_RERUN_PATH");
  if (recordingPath != nullptr && recordingPath[0] != '\0') {
    __recording->save(recordingPath).exit_on_failure();
  } else if (serverAddress_.empty()) {
    __recording->spawn().exit_on_failure();
  } else {
    const Error error = __recording->connect_grpc(serverAddress_);
    if (error.is_err()) {
      __throw__(
          "InterfaceRerun: Failed to connect to Rerun server at " +
          serverAddress_ + ": " + error.description);
    }
  }

#ifdef DEBUG
  // SLAM coordinates follow the initial optical camera frame: x right,
  // y down, z forward. This metadata controls viewing, not SLAM geometry.
  __recording->log_static("world", ViewCoordinates::RDF);
  __recording->log_static(
      "world/visualization",
      rerun::Transform3D::from_mat3x3(worldAxes).with_axis_length(0.5F),
      ViewCoordinates::FLU);
  __recording->log(
      "world/root/camera",
      Pinhole::from_focal_length_and_resolution({500.0, 500.0}, {640.0, 480.0})
          .with_camera_xyz(rerun::components::ViewCoordinates::RDF));
#endif

#ifdef SHOW_DENSE
  __recording->log_static("scene", ViewCoordinates::RDF);
  __recording->log_static(
      "scene/world",
      rerun::Transform3D::from_mat3x3(worldAxes).with_axis_length(0.5F),
      ViewCoordinates::FLU);
  __recording->log(
      "scene/camera",
      Pinhole::from_focal_length_and_resolution({500.0, 500.0}, {640.0, 480.0})
          .with_camera_xyz(rerun::components::ViewCoordinates::RDF));
#endif

  AbstractInterface::initiate(
      pointers_,
      queryMapLogFrequency_,
      queryCameraLogFrequency_);
}

}  // namespace interface
}  // namespace slam_mer
