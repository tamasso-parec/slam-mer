/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file keyframe.cpp
/// @brief Implements `keyframe_t` construction and static data access helpers.
/// @details Defines weak-pointer-safe getters/setters for keyframe-owned data.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <keyframe.hpp>

namespace slam_mer {
using data_type::SensorData;

// ---------------------------------------------------------------------------
// Construct keyframe from sensor data, pose, and features
// ---------------------------------------------------------------------------
keyframe_t::keyframe_t(
    const SensorData::weak_ptr& cameraData_,
    const pose_t& initPose_,
    const point2d_t::vector& allKeypoints_,
    const descriptor_t::vector& allDescriptors_) {

  auto sharedCameraData = cameraData_.lock();
  if (!sharedCameraData) {
    __throw__(
        "In keyframe_t constructor: Failed to lock camera\
        data weak pointer");
  }
  _cameraData = sharedCameraData;
  _initialPose = initPose_;
  _allKeypoints = allKeypoints_;
  _allDescriptors = allDescriptors_;
}

// ---------------------------------------------------------------------------
// Get intrinsics from keyframe weak pointer
// ---------------------------------------------------------------------------
output_t<intrinsics_t> keyframe_t::get_intrinsics(
    const keyframe_t::weak_ptr& ptr_) {
  if (auto shared = ptr_.lock()) {
    if (shared->_cameraData->calibrated())
      return shared->_cameraData->get_intrinsics();
    else
      return fail_t::IS_EMPTY;
  } else {
    return fail_t::LOST_PTR;
  }
}

// ---------------------------------------------------------------------------
// Get color image from keyframe weak pointer
// ---------------------------------------------------------------------------
output_t<image_t> keyframe_t::get_color_image(
    const keyframe_t::weak_ptr& ptr_) {
  if (auto shared = ptr_.lock())
    return shared->_cameraData->get_color_image();
  else
    return fail_t::LOST_PTR;
}

// ---------------------------------------------------------------------------
// Get descriptor vector from keyframe weak pointer
// ---------------------------------------------------------------------------
output_t<descriptor_t::vector> keyframe_t::get_descriptors(
    const keyframe_t::weak_ptr& ptr_) {
  if (auto shared = ptr_.lock())
    return shared->get_all_descriptors();
  else
    return fail_t::LOST_PTR;
}

// ---------------------------------------------------------------------------
// Set image descriptor through keyframe weak pointer
// ---------------------------------------------------------------------------
fail_t keyframe_t::set_image_descriptor(
    const keyframe_t::weak_ptr& ptr_,
    const descriptor_t& descriptor_) {
  if (auto shared = ptr_.lock()) {
    shared->_imageDescriptor = descriptor_;
    return fail_t::NO_FAIL;
  } else {
    return fail_t::LOST_PTR;
  }
}

// ---------------------------------------------------------------------------
// Get image descriptor from keyframe weak pointer
// ---------------------------------------------------------------------------
output_t<descriptor_t> keyframe_t::get_image_descriptor(
    const keyframe_t::weak_ptr& ptr_) {
  if (auto shared = ptr_.lock())
    return shared->get_image_descriptor();
  else
    return fail_t::LOST_PTR;
}

// ---------------------------------------------------------------------------
// Get camera data from keyframe weak pointer
// ---------------------------------------------------------------------------
output_t<SensorData::weak_ptr> keyframe_t::get_camera_data(
    const keyframe_t::weak_ptr& ptr_) {
  if (auto shared = ptr_.lock())
    return SensorData::weak_ptr(shared->_cameraData);
  else
    return fail_t::LOST_PTR;
}

// ---------------------------------------------------------------------------
// Get pose pointer from keyframe weak pointer
// ---------------------------------------------------------------------------
output_t<pose_t::weak_ptr> keyframe_t::get_pose(
    const keyframe_t::weak_ptr& ptr_) {
  if (auto shared = ptr_.lock())
    return pose_t::weak_ptr(shared->_cameraPose);
  else
    return fail_t::LOST_PTR;
}

// ---------------------------------------------------------------------------
// Convert keyframe weak pointer to shared pointer
// ---------------------------------------------------------------------------
output_t<keyframe_t::shared_ptr> keyframe_t::to_shared(
    const keyframe_t::weak_ptr& ptr_) {
  using std::println;
  keyframe_t::shared_ptr sharedPtr = ptr_.lock();
  if (!sharedPtr) {
    println(
        "[Keyframe] {}Warning{}: Failed to convert keyframe weak to "
        "shared_ptr.",
        yellow,
        reset);
    return fail_t::LOST_PTR;
  }
  return sharedPtr;
}

}  // namespace slam_mer
