/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file rotation.hpp
/// @brief Quaternion-based 3D rotation type and geometric helpers.
/// @details Builds on Eigen::Quaterniond to expose project-specific operations
/// for angle error computation and rotation application to
/// points/translations. Used throughout pose and transform code.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef ROTATION_HPP
#define ROTATION_HPP

// Our includes
#include <types/point3d.hpp>
#include <types/translation.hpp>

// Third party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

/// @brief Represents a 3D rotation using `Eigen::Quaterniond`.
///
/// This wrapper adds utility methods for SLAM operations such as rotation
/// error computation and applying rotations to points and vectors.
/// @note Inherits constructors from `Eigen::Quaterniond`.
/// @see Eigen::Quaterniond
namespace slam_mer {

/// Quaternion type, using Eigen Quaterniond
class rotation_t : public Eigen::Quaterniond {
public:
  using Eigen::Quaterniond::Quaterniond;

  /// @brief Computes the angular error (in degrees) between two rotations.
  /// @param rotation1_ Source rotation used as the reference orientation.
  /// @param rotation2_ Rotation compared against `rotation1_`.
  /// @note  The error is defined as the angle of the relative rotation between
  /// rotation1 and rotation2.
  /// @return The rotation error in degrees.
  [[nodiscard("Use to compute the angular error between two rotations.")]]
  static double computeError(
      const rotation_t& rotation1_,
      const rotation_t& rotation2_) {
    Eigen::Matrix3d relative =
        rotation1_.matrix().inverse() * rotation2_.matrix();
    Eigen::AngleAxisd angleAxis(relative);
    return std::abs(angleAxis.angle()) * 180.0 / M_PI;
  }

  /// @brief Applies this rotation to a 3D point.
  /// @param point_ Point coordinates to rotate with this quaternion.
  /// @return The rotated point.
  point3d_t operator*(const point3d_t& point_) const {
    point3d_t pointCopy = point_;
    const Eigen::Vector3d& pointEigen = pointCopy.to_eigen();
    const Eigen::Quaterniond& rotationCopy = *this;
    const Eigen::Vector3d rotatedPoint = rotationCopy * pointEigen;
    const double x = rotatedPoint.x();
    const double y = rotatedPoint.y();
    const double z = rotatedPoint.z();
    pointCopy.set_coordinates(x, y, z);
    return pointCopy;
  }

  /// @brief Applies this rotation to a translation vector.
  /// @param translation_ Translation vector expressed in the source frame.
  /// @return The rotated translation.
  translation_t operator*(const translation_t& translation_) const {
    // Not very elegant, but Eigen does not allow to pass translation_ directly
    const Eigen::Vector3d& vectorCopy = translation_.to_eigen();
    const Eigen::Quaterniond& rotationCopy = *this;
    return rotationCopy * vectorCopy;
  }

  /// @brief Composes this rotation with another rotation.
  /// @param other_ Rotation applied after this rotation in the composition.
  /// @return The result of applying this rotation followed by the other.
  rotation_t operator*(const rotation_t& other_) const {
    const Eigen::Quaterniond& otherCopy = other_;
    const Eigen::Quaterniond& rotationCopy = *this;
    return rotationCopy * otherCopy;
  }

  /// @brief Applies this rotation to a 3D Eigen vector.
  /// @param vec_ 3D vector to rotate using this quaternion.
  /// @return The rotated 3D vector.
  Eigen::Vector3d operator*(const Eigen::Vector3d& vec_) const {
    const Eigen::Quaterniond& rotationCopy = *this;
    return rotationCopy * vec_;
  }

  /// @brief Computes and returns the inverse of this rotation.
  /// @return The inverse of the current rotation.
  [[nodiscard("Use to get the inverse of the rotation.")]]
  rotation_t inverse() const {
    const Eigen::Quaterniond& rotationCopy = *this;
    return rotationCopy.inverse();
  }

  /// @brief Rerun wants to get the rotation as a set of floats.
  /// To ease the conversion, we provide this function, which outputs the
  /// rotation matrix as 3x3 matrix of floats.
  /// @return Rotation matrix as floats.
  [[nodiscard("Use to get the rotation matrix as floats for Rerun.")]]
  Eigen::Matrix3f to_rerun() const {
    Eigen::Matrix3d rotMatrix = this->toRotationMatrix();
    return Eigen::Matrix3f(rotMatrix.cast<float>().eval());
  }

  /// @brief Set the rotation in the quaternion format.
  /// @return The rotation as a quaternion (x, y, z, w).
  [[nodiscard("Use to get the rotation in quaternion format.")]]
  array4D_t get_quaternions() const {
    return array4D_t(this->x(), this->y(), this->z(), this->w());
  }
};

}  // namespace slam_mer

#endif  // TYPES_ROTATION_HPP
