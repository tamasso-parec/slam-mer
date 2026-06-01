/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file pose.hpp
/// @brief SE(3) pose container with transform and validation utilities.
/// @details Implements rotation/translation storage, composition, inversion,
/// and point transformation helpers for SLAM estimation workflows.
/// Includes retrieval and fail-check utilities for pointer and
/// output-based interfaces.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPES_POSE_HPP
#define TYPES_POSE_HPP

// Our includes
#include <unistd.h>
#include <types/mutex.hpp>
#include <types/output.hpp>
#include <types/point3d.hpp>
#include <types/rotation.hpp>
#include <types/translation.hpp>

// STD libraries
#include <limits>
#include <print>
#include <shared_mutex>
#include "types/array.hpp"

namespace slam_mer {

/// @brief Represents a 3D pose as rotation and translation.
///
/// The pose supports inversion, composition, and point transformation.
/// It represents a transformation from local coordinates into world/camera
/// coordinates used by the SLAM pipeline.
class pose_t {

public:
  /// Typedef for shared pointer to pose_t
  using shared_ptr = std::shared_ptr<pose_t>;
  /// Typedef for unique pointer to pose_t
  using unique_ptr = std::unique_ptr<pose_t>;
  /// Typedef for weak pointer to pose_t
  using weak_ptr = std::weak_ptr<pose_t>;
  /// Make shared pointer function
  /// @return Shared pointer owning a new pose initialized as unset.
  static pose_t::shared_ptr make_shared() { return std::make_shared<pose_t>(); }
  /// Make shared pointer function
  /// @param other_ Pose value copied into the new shared instance.
  /// @return Shared pointer owning a copy of `other_`.
  static pose_t::shared_ptr make_shared(const pose_t& other_) {
    return std::make_shared<pose_t>(other_);
  }

  // ------------------------------------------------------------------------
  // Static methods

  /// Update the pose pointed to by the weak_ptr with the values from the target
  /// pose
  /// @param other_ Weak pointer to the destination pose to update.
  /// @param target_ Pose data copied into the destination pose.
  /// @return `fail_t::NO_FAIL` on success, otherwise a pointer-lock failure.
  [[nodiscard("WARN: Success not checked.")]]
  static fail_t update(const pose_t::weak_ptr& other_, const pose_t& target_) {
    if (auto sharedOther = other_.lock()) {
      sharedOther->set_pose(target_);
      return fail_t::NO_FAIL;
    } else {
      return fail_t(fail_t::LOST_PTR, "Failed to lock weak pointer");
    }
  }

  /// Get the pose as a 4x4 transformation matrix.
  /// @param ptr_ Weak pointer to the pose to read.
  /// @return 4x4 transform matrix, or failure if the pointer is expired.
  [[nodiscard("Use to get the pose as a 4x4 transformation matrix.")]]
  static output_t<Eigen::Matrix4d> get_matrix(const pose_t::weak_ptr& ptr_) {
    if (auto shared = ptr_.lock()) {
      read_lock_t lock(shared->_poseMutex);
      return shared->get_matrix();
    } else {
      return fail_t(fail_t::LOST_PTR, "Failed to lock weak pointer");
    }
  }

  /// Get the identity pose.
  /// @return Pose initialized with identity rotation and zero translation.
  [[nodiscard("Use to get the identity pose.")]]
  static pose_t Identity() {
    return pose_t(Eigen::Matrix4d::Identity());
  }

  /// Compute the error between two poses.
  /// @param pose1_ First pose used as the error reference.
  /// @param pose2_ Second pose compared against `pose1_`.
  /// @return `{rotation_error_deg, translation_error_norm}`.
  [[nodiscard("Use to compute the error between two poses.")]]
  static array2D_t computeError(const pose_t& pose1_, const pose_t& pose2_) {
    using std::tie;
    const auto [rotation1, translation1] = pose1_.get_rotation_translation();
    const auto [rotation2, translation2] = pose2_.get_rotation_translation();
    const double rotationError = rotation_t::computeError(rotation1, rotation2);
    const double translationError = (translation1 - translation2).norm();
    return {rotationError, translationError};
  }

  // ------------------------------------------------------------------------

  /// @brief Construct a new pose_t object.
  pose_t() {
    using std::numeric_limits;
    _rotation = rotation_t::Identity();
    const double& max = numeric_limits<double>::max();
    _translation = translation_t(max, max, max);
  }

  /// @brief Construct a new pose_t object.
  /// @param pose_ 4x4 transform matrix containing rotation and translation.
  pose_t(const Eigen::Matrix4d& pose_)
      : _rotation(pose_.block<3, 3>(0, 0)),
        _translation(pose_.block<3, 1>(0, 3)) {}

  /// @brief Construct a new pose_t object
  /// @param rotation_ Rotation matrix used to initialize orientation.
  /// @param translation_ Translation vector used to initialize position.
  pose_t(const Eigen::Matrix3d& rotation_, const Eigen::Vector3d& translation_)
      : _rotation(rotation_t(rotation_)),
        _translation(translation_t(translation_)) {}

  /// @brief Constructs a pose from a rotation/translation tuple.
  /// @param pose_ Tuple containing rotation and translation components.
  pose_t(std::tuple<rotation_t, translation_t> pose_)
      : _rotation(std::get<0>(pose_)), _translation(std::get<1>(pose_)) {}

  /// @brief Construct a new pose_t object.
  /// @param rotation_ Rotation component of the pose.
  /// @param translation_ Translation component of the pose.
  pose_t(const rotation_t& rotation_, const translation_t& translation_)
      : _rotation(rotation_), _translation(translation_) {}

  /// @brief Constructs a pose from a shared pose pointer.
  /// @param other_ Shared pointer to a pose whose values are copied.
  pose_t(const pose_t::shared_ptr& other_) {
    read_lock_t lock(other_->_poseMutex);
    const auto [rotation, translation] = other_->get_rotation_translation();
    _rotation = rotation;
    _translation = translation;
  }

  /// @brief Constructs a pose from a weak pose pointer.
  /// @param other_ Weak pointer to a pose whose values are copied if valid.
  pose_t(const pose_t::weak_ptr& other_) {
    using std::numeric_limits;
    if (auto shared = other_.lock()) {
      read_lock_t lock(shared->_poseMutex);
      const auto [rotation, translation] = shared->get_rotation_translation();
      _rotation = rotation;
      _translation = translation;
    } else {
      _rotation = rotation_t::Identity();
      const double& max = numeric_limits<double>::max();
      _translation = translation_t(max, max, max);
    }
  }

  /// @brief Construct a new pose_t object
  /// @param other_ Pose object copied into this instance.
  pose_t(const pose_t& other_)
      : _rotation(other_._rotation), _translation(other_._translation) {}

  /// @brief Destroy the pose_t object
  ~pose_t() {};

  /// @brief Check if pose is unset (i.e., has invalid translation).
  /// @return `true` when translation equals the unset sentinel values.
  [[nodiscard("Use to check if the pose is unset or not.")]]
  bool is_unset() const {
    read_lock_t lock(_poseMutex);
    return (
        _translation.x() == std::numeric_limits<double>::max() &&
        _translation.y() == std::numeric_limits<double>::max() &&
        _translation.z() == std::numeric_limits<double>::max());
  }

  /// @brief Check if pose is set
  /// @return `true` when the pose contains a valid translation.
  inline operator bool() const {
    read_lock_t lock(_poseMutex);
    return !is_unset();
  }

  /// @brief Pointer-like access to this pose.
  [[deprecated("Avoid using these kind of operators")]]
  auto operator->() {
    return this;
  }
  /// @brief Pointer-like access to this pose (const).
  [[deprecated("Avoid using these kind of operators")]]
  auto operator->() const {
    return this;
  }

  /// @brief Assignment operator for pose_t.
  /// @param other_ Pose whose values are copied into this object.
  /// @return Reference to this pose.
  pose_t& operator=(const pose_t& other_) {
    if (this != &other_) {
      write_lock_t lock_write(_poseMutex);
      std::tie(_rotation, _translation) = other_.get_rotation_translation();
    }
    return *this;
  }

  /// @brief Assignment operator for pose_t from a weak_ptr.
  /// @param other_ Weak pointer to the pose source.
  /// @return Reference to this pose. (unset if the weak_ptr is expired)
  pose_t& operator=(const pose_t::weak_ptr& other_) {
    if (auto sharedOther = other_.lock()) {
      write_lock_t lock_write(_poseMutex);
      std::tie(_rotation, _translation) =
          sharedOther->get_rotation_translation();
    } else {
      write_lock_t lock_write(_poseMutex);
      _rotation = rotation_t::Identity();
      _translation = translation_t(
          std::numeric_limits<double>::max(),
          std::numeric_limits<double>::max(),
          std::numeric_limits<double>::max());
    }
    return *this;
  }

  /// @brief Operator for pose not defined
  bool operator!() {
    read_lock_t lock(_poseMutex);
    return is_unset();
  }

  /// @brief Check if two poses are equal.
  /// @param other_ Shared pointer to the pose compared against this one.
  /// @return `true` when both rotation and translation are equal.
  bool operator==(const pose_t::shared_ptr& other_) {
    read_lock_t lock(_poseMutex);
    const auto [rotation, translation] = other_->get_rotation_translation();
    return (_rotation == rotation) && (_translation == translation);
  }

  /// @brief Invert the pose that is stored in this variable.
  inline void invert() {
    write_lock_t lock(_poseMutex);
    // Invert the pose by inverting the rotation and negating the translation
    _rotation = _rotation.inverse();
    _translation = -(_rotation * _translation);
  }

  /// @brief Outputs the inverse of the pose.
  /// @note Be aware that this function is allocating a new pose object
  /// and returns it. The used of this function should be done with care
  /// to avoid memory leaks.
  /// @return both rotation and translation of the inverted pose
  [[nodiscard("Use to get the inverse of the pose.")]]
  inline std::tuple<rotation_t, translation_t> get_invert() const {
    read_lock_t lock(_poseMutex);
    pose_t inverted = pose_t(_rotation, _translation);
    inverted.invert();
    const auto [rotation, translation] = inverted.get_rotation_translation();
    return std::make_tuple(rotation, translation);
  }

  /// @brief Transform a 3D point using the pose.
  /// @param point_ 3D point to transform with this pose.
  /// @return The transformed 3D point.
  [[nodiscard("Use to get the transformed point using the pose.")]]
  inline point3d_t transform(const point3d_t point_) const {
    point3d_t transformedPoint = point_;
    const array3D_t coords = point_;
    const array3D_t transformedCoords = this->transform(coords);
    const double& x = transformedCoords[0];
    const double& y = transformedCoords[1];
    const double& z = transformedCoords[2];
    transformedPoint.set_coordinates(x, y, z);
    return transformedPoint;
  }

  /// @brief Transform a 3D point using the pose.
  /// @param point_ Point coordinates represented as `{x, y, z}`.
  /// @return The transformed 3D point.
  [[nodiscard("Use to get the transformed point using the pose.")]]
  inline array3D_t transform(const array3D_t& point_) const {
    const array3D_t transformedPoint = _rotation * point_ + _translation;
    return transformedPoint;
  }

  /// @brief Transform a 3D point using the pose.
  /// @param point_ Point coordinates represented as `{x, y, z}`.
  /// @return The transformed 3D point.
  inline array3D_t operator*(const array3D_t& point_) const {
    return this->transform(point_);
  }

  /// @brief Transform a 3D point using the pose.
  /// @param point_ 3D point object transformed by this pose.
  /// @return The transformed 3D point.
  inline point3d_t operator*(const point3d_t& point_) const {
    point3d_t transformedPoint = point_;
    const array3D_t coords = point_;
    const array3D_t transformedCoords = this->transform(coords);
    const double& x = transformedCoords[0];
    const double& y = transformedCoords[1];
    const double& z = transformedCoords[2];
    transformedPoint.set_coordinates(x, y, z);
    return transformedPoint;
  }

  /// @brief Transform a 3D point using the pose.
  /// @param point_ Shared pointer to the point transformed by this pose.
  /// @return The transformed 3D point.
  inline point3d_t operator*(point3d_t::shared_ptr& point_) const {
    point3d_t transformedPoint = point_;
    const array3D_t coords = *point_;
    const array3D_t transformedCoords = this->transform(coords);
    const double& x = transformedCoords[0];
    const double& y = transformedCoords[1];
    const double& z = transformedCoords[2];
    transformedPoint.set_coordinates(x, y, z);
    return transformedPoint;
  }

  /// @brief Transform a 3D point using the pose.
  /// @param point_ Eigen vector representation of the point to transform.
  /// @return The transformed 3D point.
  [[deprecated(
      "Avoid using this operations. Use either point3d_t or \
      array3D_t instad.")]]
  inline Eigen::Vector3d operator*(const Eigen::Vector3d point_) const {
    const array3D_t coords = {point_.x(), point_.y(), point_.z()};
    const array3D_t newCoords = this->transform(coords);
    return Eigen::Vector3d(newCoords[0], newCoords[1], newCoords[2]);
  }

  /// @brief Overload the multiplication operator to combine two poses.
  /// @param other_ Shared pointer to the pose composed with this pose.
  /// @return The combined pose.
  inline pose_t operator*(const pose_t::shared_ptr& other_) const {
    read_lock_t lock(_poseMutex);
    // Combine the rotation and translation of both poses
    const auto [rotation, translation] = other_->get_rotation_translation();
    const rotation_t combinedRotation = _rotation * rotation;
    const auto combinedTranslation = _rotation * translation + _translation;
    return pose_t(combinedRotation, combinedTranslation);
  }

  /// @brief Multiplication operator to transform a 3D point using the pose.
  /// @param pose_ Pose composed with this pose (`this * pose_`).
  /// @return The combined pose.
  inline pose_t operator*(const pose_t& pose_) const {
    read_lock_t lock(_poseMutex);
    const auto [rotation, translation] = pose_.get_rotation_translation();
    const rotation_t combinedRotation = _rotation * rotation;
    const translation_t combinedTranslation =
        _rotation * translation + _translation;
    return pose_t(combinedRotation, combinedTranslation);
  }

  /// @brief Transforms the current pose given as input another pose. T1 = T2*T1
  /// @param other_ Shared pointer to the pose used for in-place composition.
  void update_pose(const pose_t::shared_ptr& other_) {
    write_lock_t lock(_poseMutex);
    // Combine the rotation and translation of both poses
    const auto [rotation, translation] = other_->get_rotation_translation();
    _rotation = _rotation * rotation;
    _translation = _rotation * translation + _translation;
  }

  // -----------------------------------------------------------------------
  // Setters

  /// @brief Set the pose using a 4x4 Eigen matrix.
  /// @param pose_ 4x4 transform matrix containing rotation and translation.
  [[deprecated(
      "Use set_pose(const rotation_t& rotation_, const translation_t& \
      translation_) or set_pose(const pose_t& other_) instead")]]
  inline void set_pose(const Eigen::Matrix4d& pose_) {
    write_lock_t lock(_poseMutex);
    _rotation = rotation_t(pose_.block<3, 3>(0, 0));
    _translation = translation_t(pose_.block<3, 1>(0, 3));
  }

  /// @brief Set the pose object using a rotation and translation.
  /// @param rotation_ Rotation component assigned to the pose.
  /// @param translation_ Translation component assigned to the pose.
  inline void set_pose(
      const rotation_t& rotation_,
      const translation_t& translation_) {
    write_lock_t lock(_poseMutex);
    _rotation = rotation_;
    _translation = translation_;
  }

  /// @brief Set the pose object using another pose.
  /// @param other_ Pose whose rotation and translation are copied.
  inline void set_pose(const pose_t& other_) {
    write_lock_t lock(_poseMutex);
    if (&other_ == this) __throw__("[pose_t] cannot set pose from itself");
    other_.lock_read();
    _rotation = other_._rotation;
    _translation = other_._translation;
    other_.unlock_read();
  }

  /// @brief Set the rotation part of the pose.
  /// @param rotation_ Rotation quaternion assigned to the pose.
  [[deprecated(
      "Use set_pose(const rotation_t& rotation_, \
      const translation_t& translation_) or \
        set_pose(const pose_t& other_) instead")]]
  inline void set_rotation(const rotation_t& rotation_) {
    write_lock_t lock(_poseMutex);
    _rotation = rotation_;
  }

  /// @brief Set the rotation part of the pose.
  /// @param rotation_ Rotation matrix converted and assigned to the pose.
  [[deprecated(
      "Use set_pose(const rotation_t& rotation_, \
      const translation_t& translation_) or \
          set_pose(const pose_t& other_) instead")]]
  inline void set_rotation(const Eigen::Matrix3d& rotation_) {
    write_lock_t lock(_poseMutex);
    _rotation = rotation_t(rotation_);
  }

  /// @brief Set the translation part of the pose.
  /// @param translation_ Translation vector assigned to the pose.
  [[deprecated(
      "Use set_pose(const rotation_t& rotation_, \
      const translation_t& translation_) or \
          set_pose(const pose_t& other_) instead")]]
  inline void set_translation(const translation_t& translation_) {
    write_lock_t lock(_poseMutex);
    _translation = translation_;
  }

  // ------------------------------------------------------------------------
  // Getters
  /// @brief Retrieve the rotation.
  /// @return Rotation component currently stored in the pose.
  [[deprecated("Use get_rotation_translation() instead.")]]
  inline rotation_t get_rotation() const {
    read_lock_t lock(_poseMutex);
    return _rotation;
  }

  /// @brief Return the translation.
  /// @return Translation component currently stored in the pose.
  [[deprecated("Use get_rotation_translation() instead.")]]
  inline translation_t get_translation() const {
    read_lock_t lock(_poseMutex);
    return _translation;
  }

  /// @brief Get both the rotation and translation of the pose.
  /// @return Tuple containing the rotation and translation.
  [[nodiscard("Use to get both the rotation and translation of the pose.")]]
  inline std::tuple<rotation_t, translation_t> get_rotation_translation()
      const {
    read_lock_t lock(_poseMutex);
    return std::make_tuple(_rotation, _translation);
  }

  /// @brief Get the transformation matrix of the pose in the eigen format.
  /// @return The transformation matrix of the pose (Eigen::Matrix4d).
  [[nodiscard("Use to get the transformation matrix of the pose.")]]
  inline Eigen::Matrix4d get_matrix() const {
    read_lock_t lock(_poseMutex);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = _rotation.toRotationMatrix();
    transform.block<3, 1>(0, 3) = _translation;
    return transform;
  }

  /// @brief Get the position (translation) of the pose as an array.
  /// @return The position of the pose as an array (array3D_t).
  [[nodiscard("Use to get the position of the pose as an array.")]]
  inline array3D_t get_position() const {
    read_lock_t lock(_poseMutex);
    return array3D_t(_translation.x(), _translation.y(), _translation.z());
  }

  /// @brief Get the rotation (quaternion) of the pose as an array.
  /// @return The rotation of the pose as an array (array4D_t).
  [[deprecated(
      "Avoid using this. Get the rotation_t instead, and\
      ask for quaternion representaion")]]
  inline array4D_t get_quaternions() const {
    read_lock_t lock(_poseMutex);
    return {_rotation.x(), _rotation.y(), _rotation.z(), _rotation.w()};
  }

  /// @brief Print the pose information to the terminal.
  /// @param prefix_ Prefix prepended to each printed output line.
  inline void print(std::string prefix_ = "") const {
    using std::println;
    read_lock_t lock(_poseMutex);
    println("{} Pose:\n", prefix_);
    println("  := Rotation (quaternion): {}\n", _rotation.get_quaternions());
    println("  := Translation: {}\n", _translation);
  }

  /// @brief Locking functions for thread-safe access to the pose.
  void lock() const { _poseMutex.lock(); }
  /// @brief Unlock the pose mutex.
  void unlock() const { _poseMutex.unlock(); }
  /// @brief Lock the pose for reading (shared lock).
  void lock_read() const { _poseMutex.lock_shared(); }
  /// @brief Unlock the pose read lock.
  void unlock_read() const { _poseMutex.unlock_shared(); }

private:
  /// Quaternion representing the rotation part of the pose
  rotation_t _rotation;
  /// Vector representing the translation part of the pose
  translation_t _translation;
  /// Mutex for thread-safe access to the pose
  mutable std::shared_mutex _poseMutex;
};

// ----------------------------------------------------------------------------
// Sanity and failure checks for pose
// ----------------------------------------------------------------------------

/// @brief Validates weak pose retrieval for a keyframe and throws on failure.
/// @param pose__ Output wrapper holding a weak pose pointer.
/// @param keyframeId__ Keyframe identifier used in the error message.
inline void __sanity_retrieve(
    const output_t<pose_t::weak_ptr>& pose__,
    const keyframe_id_t& keyframeId__) {
  if (pose__ != fail_t::NO_FAIL)
    __throw__(
        "Keyframe " + std::string(keyframeId__) +
        " does not have a valid pose.");
}

/// @brief Validates pose retrieval for a keyframe and throws on failure.
/// @param pose__ Output wrapper holding the pose value.
/// @param keyframeId__ Keyframe identifier used in the error message.
inline void __sanity_retrieve(
    const output_t<pose_t>& pose__,
    const keyframe_id_t& keyframeId__) {
  if (pose__ != fail_t::NO_FAIL)
    __throw__(
        "Keyframe " + std::string(keyframeId__) +
        " does not have a valid pose.");
}

/// @brief Validates a pose value for a keyframe and throws if unset.
/// @param pose__ Pose value to validate.
/// @param keyframeId__ Keyframe identifier used in the error message.
inline void __sanity_pose(
    const pose_t& pose__,
    const keyframe_id_t& keyframeId__) {
  if (!pose__)
    __throw__(
        "Keyframe " + std::string(keyframeId__) +
        " does not have a valid pose.");
}

/// @brief Validates weak pose retrieval and throws on failure.
/// @param pose__ Output wrapper holding a weak pose pointer.
inline void __sanity_retrieve(const output_t<pose_t::weak_ptr>& pose__) {
  if (pose__ != fail_t::NO_FAIL)
    __throw__("Keyframe does not have a valid pose.");
}

/// @brief Validates pose retrieval and throws on failure.
/// @param pose__ Output wrapper holding the pose value.
inline void __sanity_retrieve(const output_t<pose_t>& pose__) {
  if (pose__ != fail_t::NO_FAIL)
    __throw__("Keyframe does not have a valid pose.");
}

/// @brief Validates a pose value and throws if unset.
/// @param pose__ Pose value to validate.
inline void __sanity_pose(const pose_t& pose__) {
  if (!pose__) __throw__("Keyframe does not have a valid pose.");
}

/// @brief Logs a warning when weak pose retrieval fails for a keyframe.
/// @param pose__ Output wrapper holding a weak pose pointer.
/// @param keyframeId__ Keyframe identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if weak pose retrieval failed for a keyframe.")]]
inline bool __fail_retrieve(
    const output_t<pose_t::weak_ptr>& pose__,
    const keyframe_id_t& keyframeId__) {
  if (pose__ != fail_t::NO_FAIL) {
    std::println(
        "{}[WARNING]{} Keyframe {} does not have a valid pose.",
        yellow,
        reset,
        std::string(keyframeId__));
    return true;
  }
  return false;
}

/// @brief Logs a warning when pose retrieval fails for a keyframe.
/// @param pose__ Output wrapper holding the pose value.
/// @param keyframeId__ Keyframe identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if pose retrieval failed for a keyframe.")]]
inline bool __fail_retrieve(
    const output_t<pose_t>& pose__,
    const keyframe_id_t& keyframeId__) {
  if (pose__ != fail_t::NO_FAIL) {
    std::println(
        "{}[WARNING]{} Keyframe {} does not have a valid pose.",
        yellow,
        reset,
        std::string(keyframeId__));
    return true;
  }
  return false;
}

/// @brief Logs a warning when a keyframe pose value is unset.
/// @param pose__ Pose value to validate.
/// @param keyframeId__ Keyframe identifier used in the warning.
/// @return `true` if pose is unset, `false` otherwise.
[[nodiscard("Use to check if a keyframe pose value is unset.")]]
inline bool __fail_set(
    const pose_t& pose__,
    const keyframe_id_t& keyframeId__) {
  if (!pose__) {
    std::println(
        "{}[WARNING]{} Keyframe {} does not have a valid pose.",
        yellow,
        reset,
        std::string(keyframeId__));
    return true;
  }
  return false;
}

/// @brief Logs a warning when weak pose retrieval fails.
/// @param pose__ Output wrapper holding a weak pose pointer.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if weak pose retrieval failed for a keyframe.")]]
inline bool __fail_retrieve(const output_t<pose_t::weak_ptr>& pose__) {
  if (pose__ != fail_t::NO_FAIL) {
    std::println(
        "{}[WARNING]{} Keyframe does not have a valid pose.",
        yellow,
        reset);
    return true;
  }
  return false;
}

/// @brief Logs a warning when pose retrieval fails.
/// @param pose__ Output wrapper holding the pose value.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if pose retrieval failed for a keyframe.")]]
inline bool __fail_retrieve(const output_t<pose_t>& pose__) {
  if (pose__ != fail_t::NO_FAIL) {
    std::println(
        "{}[WARNING]{} Keyframe does not have a valid pose.",
        yellow,
        reset);
    return true;
  }
  return false;
}

/// @brief Logs a warning when a pose value is unset.
/// @param pose__ Pose value to validate.
/// @return `true` if pose is unset, `false` otherwise.
[[nodiscard("Use to check if a pose value is unset.")]]
inline bool __fail_set(const pose_t& pose__) {
  if (!pose__) {
    std::println(
        "{}[WARNING]{} Keyframe does not have a valid pose.",
        yellow,
        reset);
    return true;
  }
  return false;
}

}  // namespace slam_mer

#endif  // TYPES_POSE_HPP
