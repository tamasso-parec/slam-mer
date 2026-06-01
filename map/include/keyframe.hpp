/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file keyframe.hpp
/// @brief Keyframe data container and access helpers for the SLAM map module.
/// @details Declares `keyframe_t`, which stores camera sensor data, initial and
/// optimized poses, 2D keypoints, descriptor sets, and map-point links.
/// The API provides access helpers and weak-pointer-safe static wrappers used
/// by mapping, localization, and visualization code paths.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

// My includes
#include <sensor_data.hpp>
#include <types/descriptor.hpp>
#include <types/output.hpp>
#include <types/point2d.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>

namespace slam_mer {

/// @brief Stores all data associated with a map keyframe.
/// @details A keyframe owns camera sensor data, initial and optimized poses,
/// keypoints, descriptors, and an optional global image descriptor used for
/// place recognition.
class keyframe_t {

public:
  /// Shared pointer type for keyframe_t.
  using shared_ptr = std::shared_ptr<keyframe_t>;
  /// Weak pointer type for keyframe_t.
  using weak_ptr = std::weak_ptr<keyframe_t>;
  /// Alias for sensor data type.
  using SensorData = data_type::SensorData;

  /// @brief Create a heap-allocated copy of an existing keyframe.
  /// @param other_ Source keyframe to copy.
  /// @return Shared pointer owning the copied keyframe.
  [[nodiscard(
      "Use returned shared_ptr to keep keyframe ownership and lifetime")]]
  static shared_ptr make_shared(const keyframe_t& other_) {
    return std::make_shared<keyframe_t>(other_);
  }

  /// @brief Retrieve camera intrinsics from a keyframe weak pointer.
  /// @param ptr_ Weak pointer to the keyframe.
  /// @return Intrinsics wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned intrinsics output to check retrieval success")]]
  static output_t<intrinsics_t> get_intrinsics(const weak_ptr& ptr_);

  /// @brief Retrieve the color image from a keyframe weak pointer.
  /// @param ptr_ Weak pointer to the keyframe.
  /// @return Color image wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned image output to check retrieval success")]]
  static output_t<image_t> get_color_image(const weak_ptr& ptr_);

  /// @brief Retrieve all local descriptors from a keyframe weak pointer.
  /// @param ptr_ Weak pointer to the keyframe.
  /// @return Descriptor vector wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned descriptor output to check retrieval success")]]
  static output_t<descriptor_t::vector> get_descriptors(const weak_ptr& ptr_);

  /// @brief Set the global image descriptor for a keyframe weak pointer.
  /// @param ptr_ Weak pointer to the keyframe.
  /// @param descriptor_ Descriptor to store in the keyframe.
  /// @return fail_t::NO_FAIL on success, otherwise a fail_t error.
  [[nodiscard("WARNING: Check to ensure descriptor update succeeded")]]
  static fail_t set_image_descriptor(
      const weak_ptr& ptr_,
      const descriptor_t& descriptor_);

  /// @brief Retrieve the global image descriptor from a keyframe.
  /// @param ptr_ Weak pointer to the keyframe.
  /// @return Global descriptor wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned descriptor output to check retrieval success")]]
  static output_t<descriptor_t> get_image_descriptor(const weak_ptr& ptr_);

  /// @brief Retrieve camera sensor data from a keyframe weak pointer.
  /// @param ptr_ Weak pointer to the keyframe.
  /// @return SensorData weak pointer wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned sensor data output to check retrieval success")]]
  static output_t<SensorData::weak_ptr> get_camera_data(const weak_ptr& ptr_);

  /// @brief Retrieve optimized camera pose pointer from a keyframe.
  /// @param ptr_ Weak pointer to the keyframe.
  /// @return pose_t weak pointer wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned pose output to check retrieval success")]]
  static output_t<pose_t::weak_ptr> get_pose(const weak_ptr& ptr_);

  /// @brief Convert keyframe weak pointer to a shared pointer.
  /// @param ptr_ Weak pointer to convert.
  /// @return Shared pointer wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned shared_ptr output to verify weak_ptr conversion")]]
  static output_t<shared_ptr> to_shared(const weak_ptr& ptr_);

  /// @brief Construct an empty keyframe instance.
  keyframe_t() = default;

  /// @brief Construct a keyframe from camera data, pose, and feature data.
  /// @param cameraData_ Camera data associated with the keyframe.
  /// @param initPose_ Initial pose of the keyframe.
  /// @param allKeypoints_ All keypoints detected in the keyframe.
  /// @param allDescriptors_ All descriptors associated with the keyframe.
  keyframe_t(
      const SensorData::weak_ptr& cameraData_,
      const pose_t& initPose_,
      const point2d_t::vector& allKeypoints_,
      const descriptor_t::vector& allDescriptors_);

  /// @brief Get optimized camera pose pointer associated with this keyframe.
  /// @return Weak pointer to the optimized camera pose.
  [[nodiscard("Use returned pose pointer to access optimized keyframe pose")]]
  inline pose_t::weak_ptr get_pose() const {
    return _cameraPose;
  }

  /// @brief Get initial camera pose associated with this keyframe.
  /// @return Initial pose used when the keyframe was inserted.
  /// @note This camera pose is the one that was estimated using pose estimation
  /// algorithms, such as PnP or ICP, during localization.
  [[nodiscard("Use returned initial pose for tracking or optimization state")]]
  inline pose_t get_initial_pose() const {
    return _initialPose;
  }

  /// @brief Get all detected 2D keypoints stored in this keyframe.
  /// @return Const reference to keypoint vector.
  [[nodiscard("Use returned keypoints for matching or map update steps")]]
  inline const point2d_t::vector& get_all_keypoints() const {
    return _allKeypoints;
  }

  /// @brief Get all local feature descriptors stored in this keyframe.
  /// @return Const reference to descriptor vector.
  [[nodiscard("Use returned descriptors for feature matching operations")]]
  inline const descriptor_t::vector& get_all_descriptors() const {
    return _allDescriptors;
  }

  /// @brief Set optimized camera pose from a pose value.
  /// @param newPose_ New pose value.
  inline void set_pose(const pose_t& newPose_) {
    _cameraPose = pose_t::make_shared(newPose_);
  }

  /// @brief Set optimized camera pose from a shared pose pointer.
  /// @param newPose_ New shared pose pointer.
  inline void set_pose(const pose_t::shared_ptr newPose_) {
    _cameraPose = newPose_;
  }

  /// @brief Set the global image descriptor for this keyframe.
  /// @param descriptor_ Descriptor to store.
  inline void set_image_descriptor(const descriptor_t& descriptor_) {
    _imageDescriptor = descriptor_;
  }

  /// @brief Get the global image descriptor for this keyframe.
  /// @return Const reference to the global image descriptor.
  [[nodiscard("Use returned descriptor for place recognition queries")]]
  inline const descriptor_t& get_image_descriptor() const {
    return _imageDescriptor;
  }

  /// @brief Get the keyframe color image.
  /// @return Const reference to the color image.
  [[nodiscard("Use returned image for visualization or feature tracking")]]
  const inline image_t& get_image() const {
    return _cameraData->get_color_image();
  }

  /// @brief Get the keyframe grayscale image.
  /// @return Const reference to the grayscale image.
  [[nodiscard("Returns grayscale image for tracking and feature extraction")]]
  const inline image_t& get_gray_image() const {
    return _cameraData->get_grayscale_image();
  }

  /// @brief Get the keyframe depth image.
  /// @return Const reference to the depth image.
  [[nodiscard("Use returned depth image for geometric reconstruction")]]
  const inline image_t& get_depth_image() const {
    return _cameraData->get_depth_image();
  }

  /// @brief Get camera intrinsics associated with this keyframe.
  /// @return Const reference to camera intrinsics.
  [[nodiscard("Use returned intrinsics for projection and geometry checks")]]
  inline const intrinsics_t& get_intrinsics() const {
    return _cameraData->get_intrinsics();
  }

  /// @brief Get distortion coefficients associated with this keyframe.
  /// @return Distortion coefficients copied from sensor data.
  [[nodiscard("Returns distortion coefficients for image undistortion steps")]]
  inline const distortion_t get_distortion() const {
    return _cameraData->get_image_distortion();
  }

  /// @brief Get camera data weak pointer associated with this keyframe.
  /// @return Weak pointer to SensorData associated with this keyframe.
  [[nodiscard("Use returned sensor data pointer for direct sensor access")]]
  inline SensorData::weak_ptr get_camera_data() const {
    return _cameraData;
  }

private:
  /// current camera pose in world coordinates
  pose_t::shared_ptr _cameraPose;
  /// camera data
  SensorData::shared_ptr _cameraData;
  /// initial camera pose for this keyframe
  pose_t _initialPose;
  /// all keypoints in the keyframe
  point2d_t::vector _allKeypoints;
  /// all descriptors in the keyframe
  descriptor_t::vector _allDescriptors;
  /// Global image descriptor for visual place recognition
  descriptor_t _imageDescriptor;
};

/// @brief Throw if a shared keyframe lookup result contains a failure.
/// @param keyframe__ Retrieval result wrapping a shared keyframe pointer.
/// @param keyframeId__ Keyframe identifier used for error context.
inline void ___saniy_retrieve(
    const output_t<keyframe_t::shared_ptr>& keyframe__,
    const keyframe_id_t& keyframeId__) {
  if (keyframe__ != fail_t::NO_FAIL)
    __throw__(
        "[Keyframe_t] Could not output the keyframe " +
        std::string(keyframeId__) + " because it does not exist in the map.");
}

/// @brief Throw if a weak keyframe lookup result contains a failure.
/// @param keyframe__ Retrieval result wrapping a weak keyframe pointer.
/// @param keyframeId__ Keyframe identifier used for error context.
inline void __sanity_retrieve(
    const output_t<keyframe_t::weak_ptr>& keyframe__,
    const keyframe_id_t& keyframeId__) {
  if (keyframe__ != fail_t::NO_FAIL)
    __throw__(
        "[Keyframe_t] Could not output the keyframe " +
        std::string(keyframeId__) + " because it does not exist in the map.");
}

/// @brief Throw if a weak keyframe lookup result contains a failure.
/// @param keyframe__ Retrieval result wrapping a weak keyframe pointer.
inline void __sanity_retrieve(
    const output_t<keyframe_t::weak_ptr>& keyframe__) {
  if (keyframe__ != fail_t::NO_FAIL)
    __throw__("[Keyframe_t] Could not output the keyframe.");
}

/// @brief Check shared keyframe retrieval failure and print warning when needed.
/// @param keyframe__ Retrieval result wrapping a shared keyframe pointer.
/// @param keyframeId__ Keyframe identifier used for warning context.
/// @return `true` when retrieval failed; otherwise `false`.
[[nodiscard("Use to check if shared keyframe retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<keyframe_t::shared_ptr>& keyframe__,
    const keyframe_id_t& keyframeId__) {
  if (keyframe__ != fail_t::NO_FAIL) {
    std::println(
        "{}WARNING{} [keyframe_t] Failed to retrieve keyframe {}",
        yellow,
        reset,
        std::string(keyframeId__));
    return true;
  }
  return false;
}

/// @brief Check weak keyframe retrieval failure and print warning when needed.
/// @param keyframe__ Retrieval result wrapping a weak keyframe pointer.
/// @param keyframeId__ Keyframe identifier used for warning context.
/// @return `true` when retrieval failed; otherwise `false`.
[[nodiscard("Use to check if weak keyframe retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<keyframe_t::weak_ptr>& keyframe__,
    const keyframe_id_t& keyframeId__) {
  if (keyframe__ != fail_t::NO_FAIL) {
    std::println(
        "{}WARNING{} [keyframe_t] Failed to retrieve keyframe {}",
        yellow,
        reset,
        std::string(keyframeId__));
    return true;
  }
  return false;
}

/// @brief Check weak keyframe retrieval failure and print warning when needed.
/// @param keyframe__ Retrieval result wrapping a weak keyframe pointer.
/// @return `true` when retrieval failed; otherwise `false`.
[[nodiscard("Use to check if weak keyframe retrieval failed.")]]
inline bool __fail_retrieve(const output_t<keyframe_t::weak_ptr>& keyframe__) {
  if (keyframe__ != fail_t::NO_FAIL) {
    std::println(
        "{}WARNING{} [keyframe_t] Failed to retrieve keyframe",
        yellow,
        reset);
    return true;
  }
  return false;
}

}  // namespace slam_mer

#endif  // KEYFRAME_HPP
