/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file camera_location.cpp
/// @brief Implements camera-pose and camera diagnostics logging.
/// @details Defines camera-location and histogram logging routines.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

// Our includes
#include <interface_rerun.hpp>

// Third-party libraries
#include <rerun.hpp>

namespace slam_mer {
namespace interface {

// We define some global variables to store the camera location
namespace {
/// Mutex to protect access to the camera location
std::shared_mutex __cameraLocation_mutex;
/// Rotation of the camera in world coordinates
rotation_t __rotation;
/// Translation of the camera in world coordinates
translation_t __translation;
}  // namespace

/// List of histogram counts to be logged
extern vector_t<vector_t<size_t>>::shared_ptr __histogramCounts;
/// List of keyframe creation criteria to be logged
extern vector_t<dictionary_t<uint8_t, int>>::shared_ptr __keyframeCriteria;
/// External variable to store the cell data for the visualization interface
extern std::shared_ptr<rerun::RecordingStream> __recording;

/// @brief Cache the latest camera pose for asynchronous visualization logging.
/// @param rotation_ Camera orientation in world coordinates.
/// @param translation_ Camera position in world coordinates.
void log_camera_location(rotation_t rotation_, translation_t translation_) {
  write_lock_t lock(__cameraLocation_mutex);
  __rotation = rotation_;
  __translation = translation_;
}

/// @brief Publish localization keyframe histogram counts to Rerun.
void InterfaceRerun::__log_histogram_counts() {
  using rerun::BarChart;

  __histogramCounts->shared_lock();
  if (__histogramCounts->empty()) {
    __histogramCounts->unlock_shared();
    return;
  }
  const auto histogramCounts_temp = __histogramCounts->back();
  __histogramCounts->unlock_shared();
  __histogramCounts->clear_();

  __recording->log(
      "plots/localization_keyframes",
      BarChart::i64(histogramCounts_temp));
}

/// @brief Publish keyframe-creation criteria histogram to Rerun.
void InterfaceRerun::__log_keyframe_criteria() {
  using rerun::BarChart;
  using rerun::TensorData;

  __keyframeCriteria->shared_lock();
  if (__keyframeCriteria->empty()) {
    __keyframeCriteria->unlock_shared();
    return;
  }

  const auto keyframeCriteria_temp = __keyframeCriteria->back();
  __keyframeCriteria->unlock_shared();
  __keyframeCriteria->clear_();

  std::vector<int64_t> x;
  x.reserve(keyframeCriteria_temp.size());
  std::vector<int64_t> y;
  y.reserve(keyframeCriteria_temp.size());
  for (const auto& [criteria, count] : keyframeCriteria_temp) {
    x.push_back(criteria);  // x-axis values (abscissa)
    y.push_back(count);     // bar heights
  }
  auto x_tensor = TensorData(rerun::Collection{x.size()}, x);

  // Log the histogram counts to the global recording stream
  __recording->log(
      "plots/keyframe_creation",
      BarChart::i64(y).with_abscissa(x_tensor));
}

/// @brief Publish camera transform and associated diagnostics to Rerun.
void InterfaceRerun::__log_camera_location() {
  using rerun::Mat3x3;
  using rerun::Transform3D;
  using rerun::Vec3D;

  // Save the current camera location to avoid slow down due to mutex locking
  rotation_t rotation;
  translation_t translation;
  {
    read_lock_t lock(__cameraLocation_mutex);
    rotation = __rotation;
    translation = __translation;
  }

  // Log the camera location to the global recording stream
#ifdef DEBUG
  __recording->log(
      "world/root/camera",
      Transform3D(
          Vec3D(translation.to_rerun().data()),
          Mat3x3(rotation.to_rerun().data())));
#endif

#ifdef SHOW_DENSE
  __recording->log(
      "scene/camera",
      Transform3D(
          Vec3D(translation.to_rerun().data()),
          Mat3x3(rotation.to_rerun().data())));
#endif

  // #ifndef DEBUG
  //   rotation_t rotationInverse = rotation.inverse();
  // translation_t translationInversexw = rotationInverse * (translation *
  // -1.0);
  //   __recording->log(
  //       "world/root",
  //       rerun::Transform3D(
  //           rerun::Vec3D(translationInverse.to_rerun().data()),
  //           rerun::Mat3x3(rotationInverse.to_rerun().data())));
  // #endif

#ifdef DEBUG
  __log_histogram_counts();
  __log_keyframe_criteria();
#endif
}

}  // namespace interface
}  // namespace slam_mer
