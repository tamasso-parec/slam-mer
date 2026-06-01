/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file camera.hpp
/// @brief Camera intrinsic and distortion models with projection helpers.
/// @details Provides calibrated camera data types for intrinsics and distortion
/// coefficients, validation methods, and point projection routines for
/// 2D/3D conversions. The interfaces are used by tracking, mapping,
/// and localization components.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPES_CAMERA_HPP
#define TYPES_CAMERA_HPP

// Our includes
#include <types/array.hpp>
#include <types/output.hpp>
#include <types/point2d.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>

// Third-party includes
#include <Eigen/Core>

// C++ includes
#include <cstddef>
#include <initializer_list>
#include <print>
#include <string>
#include <tuple>

namespace slam_mer {

/// @brief Camera intrinsic-parameter model.
///
/// This class stores focal length, principal point, and skew, and provides
/// projection utilities.
class intrinsics_t {
public:
  /// Default constructor initializes an empty intrinsics model.
  intrinsics_t() : _intrinsics(Eigen::Matrix3d::Zero()), _isSet(false) {}
  /// Constructs an intrinsics model from a given 3x3 matrix.
  /// @param matrix_ Full camera calibration matrix in pixel units.
  explicit intrinsics_t(const Eigen::Matrix3d& matrix_)
      : _intrinsics(matrix_), _isSet(true) {}
  /// Constructs an intrinsics model from an initializer list of parameters.
  /// @param fx_ Focal length along the image x-axis.
  /// @param fy_ Focal length along the image y-axis.
  /// @param cx_ Principal-point x coordinate.
  /// @param cy_ Principal-point y coordinate.
  intrinsics_t(
      const double fx_,
      const double fy_,
      const double cx_,
      const double cy_)
      : _intrinsics(Eigen::Matrix3d::Zero()), _isSet(true) {
    _intrinsics(0, 0) = fx_;
    _intrinsics(1, 1) = fy_;
    _intrinsics(0, 2) = cx_;
    _intrinsics(1, 2) = cy_;
    _intrinsics(2, 2) = 1.0;
  }
  /// Checks if the intrinsics model is empty (i.e., not set).
  /// @return `true` when no valid intrinsic parameters are stored.
  [[nodiscard("Check if intrinsics are set before using.")]]
  inline bool empty() const {
    return !_isSet;
  }
  /// Creates a copy of the current intrinsics model.
  /// @return Copy of this intrinsics object.
  [[nodiscard("Retrieving intrinsics.")]]
  inline intrinsics_t clone() const {
    return *this;
  }
  /// Accesses the underlying 3x3 matrix of the intrinsics model.
  /// @return Reference to the internal 3x3 intrinsic calibration matrix.
  [[nodiscard("Retrieving intrinsics matrix.")]]
  inline const Eigen::Matrix3d& matrix() const {
    __check_not_empty();
    return _intrinsics;
  }
  /// Focal length in x
  /// @return Horizontal focal length in pixels.
  [[deprecated("Use get_parameters(.) instead")]]
  double fx() const {
    __check_not_empty();
    return _intrinsics(0, 0);
  }
  /// Focal length in y
  /// @return Vertical focal length in pixels.
  [[deprecated("Use get_parameters(.) instead")]]
  double fy() const {
    __check_not_empty();
    return _intrinsics(1, 1);
  }
  /// Principal point x
  /// @return Principal-point x coordinate in pixels.
  [[deprecated("Use get_parameters(.) instead")]]
  double cx() const {
    __check_not_empty();
    return _intrinsics(0, 2);
  }
  /// Principal point y
  /// @return Principal-point y coordinate in pixels.
  [[deprecated("Use get_parameters(.) instead")]]
  double cy() const {
    __check_not_empty();
    return _intrinsics(1, 2);
  }
  /// Skew factor
  /// @return Pixel skew coefficient between x and y axes.
  [[deprecated("Use get_parameters(.) instead")]]
  double skew() const {
    __check_not_empty();
    return _intrinsics(0, 1);
  }

  /// Get all parameters as a tuple (fx, fy, skew, cx, cy)
  /// @return Tuple `(fx, fy, skew, cx, cy)` extracted from the matrix.
  [[nodiscard("Retrieving intrinsics parameters.")]]
  std::tuple<double, double, double, double, double> get_parameters() const {
    __check_not_empty();

    const double& fx = _intrinsics(0, 0);
    const double& fy = _intrinsics(1, 1);
    const double& cx = _intrinsics(0, 2);
    const double& cy = _intrinsics(1, 2);
    const double& skew = _intrinsics(0, 1);
    return std::make_tuple(fx, fy, skew, cx, cy);
  }

  /// Get the image size image (approximate) from the intrinsics.
  /// @return Approximate image size `{width, height}` derived from `(cx, cy)`.
  [[nodiscard("Retrieving image size from intrinsics.")]]
  inline array2D_t get_image_size() const {
    __check_not_empty();
    const double& cx = _intrinsics(0, 2);
    const double& cy = _intrinsics(1, 2);
    return {cx * 2, cy * 2};
  }

  /// Check if a 2D point is within the image bounds defined by the intrinsics.
  /// @param point_ Pixel coordinate evaluated against image bounds.
  /// @return `true` when the pixel lies inside `[0,width) x [0,height)`.
  [[nodiscard("Checking if point is within image bounds.")]]
  inline bool in_image(const array2D_t& point_) const {
    const double& cx = _intrinsics(0, 2);
    const double& cy = _intrinsics(1, 2);
    return point_[0] >= 0 && point_[0] < cx * 2 && point_[1] >= 0 &&
           point_[1] < cy * 2;
  }

  /// @brief Projects a 3D world point onto the image plane.
  /// @param pointWorld_ 3D point expressed in world coordinates.
  /// @param worldToCamera_ Pose transforming world coordinates into camera
  /// coordinates.
  ///
  /// The input point is transformed to camera coordinates with `worldToCamera`,
  /// then projected with the intrinsic parameters.
  /// @return Tuple with validity flag and projected pixel coordinates.
  [[nodiscard("Projecting 3D point to 2D image plane.")]]
  std::tuple<bool, array2D_t> project(
      const array3D_t& pointWorld_,
      slam_mer::pose_t worldToCamera_ = slam_mer::pose_t::Identity()) const {
    using std::make_tuple;
    // Transform the point to camera coordinates
    const array3D_t pointCamera = worldToCamera_ * pointWorld_;
    // Project the point onto the image plane
    const double& fx = _intrinsics(0, 0);
    const double& fy = _intrinsics(1, 1);
    const double& cx = _intrinsics(0, 2);
    const double& cy = _intrinsics(1, 2);
    const double& skew = _intrinsics(0, 1);
    const double x = pointCamera[0] / pointCamera[2];
    const double y = pointCamera[1] / pointCamera[2];
    const double u = fx * x + skew * y + cx;
    const double v = fy * y + cy;

    if (pointCamera[2] <= 0.0) return make_tuple(false, array2D_t{u, v});
    if (!this->in_image(array2D_t{u, v}))
      return make_tuple(false, array2D_t{u, v});
    return make_tuple(true, array2D_t{u, v});
  }

  /// @brief Projects a 3D world point onto the image plane.
  /// @param pointWorld_ 3D point object expressed in world coordinates.
  /// @param worldToCamera_ Pose transforming world coordinates into camera
  /// coordinates.
  ///
  /// This overload accepts `point3d_t` and returns `point2d_t`.
  /// @return Tuple with validity flag and projected 2D point.
  [[nodiscard("Projecting 3D point to 2D image plane.")]]
  std::tuple<bool, slam_mer::point2d_t> project(
      const slam_mer::point3d_t& pointWorld_,
      slam_mer::pose_t worldToCamera_ = slam_mer::pose_t::Identity()) const {
    // Transform the point to camera coordinates
    const array3D_t pointArray = pointWorld_.to_3d_array();
    const auto& [valid, pointCamera] =
        this->project(pointArray, worldToCamera_);
    return std::make_tuple(valid, slam_mer::point2d_t(pointCamera));
  }

  /// @brief Prints the camera intrinsic parameters.
  /// @param prefix_ Prefix prepended to each printed output line.
  void print(std::string prefix_ = "") const {
    using std::println;
    __check_not_empty();
    const double& fx = _intrinsics(0, 0);
    const double& fy = _intrinsics(1, 1);
    const double& cx = _intrinsics(0, 2);
    const double& cy = _intrinsics(1, 2);
    const double& skew = _intrinsics(0, 1);
    println("{}Camera Intrinsics:\n", prefix_);
    println("  := {}fx: {}\n", prefix_, fx);
    println("  := {}fy: {}\n", prefix_, fy);
    println("  := {}cx: {}\n", prefix_, cx);
    println("  := {}cy: {}\n", prefix_, cy);
    println("  := {}skew: {}\n", prefix_, skew);
  }

private:
  /// Throws if intrinsics have not been initialized.
  inline void __check_not_empty() const {
    if (this->empty()) __throw__("Intrinsics empty");
  }
  /// Intrinsic calibration matrix.
  Eigen::Matrix3d _intrinsics;
  /// Indicates whether intrinsics are initialized.
  bool _isSet;
};

/// @brief Camera distortion-parameter model.
///
/// This class stores lens distortion coefficients.
class distortion_t {
public:
  /// Default constructor initializes an empty distortion model.
  distortion_t() : _coefficients(), _isSet(false) {}
  /// Constructs a distortion model from a given vector of coefficients.
  /// @param coefficients_ Distortion coefficient vector in calibration order.
  explicit distortion_t(const Eigen::VectorXd& coefficients_)
      : _coefficients(coefficients_), _isSet(coefficients_.size() > 0) {}
  /// Constructs a distortion model from an initializer list of coefficients.
  /// @param coefficients_ Distortion coefficients in calibration order.
  distortion_t(std::initializer_list<double> coefficients_)
      : _coefficients(static_cast<Eigen::Index>(coefficients_.size())),
        _isSet(coefficients_.size() > 0) {
    Eigen::Index index = 0;
    for (const double coefficient : coefficients_) {
      _coefficients(index++) = coefficient;
    }
  }

  /// Checks if the distortion model is empty (i.e., has no coefficients).
  /// @return `true` when no distortion coefficients are available.
  inline bool empty() const { return !_isSet || _coefficients.size() == 0; }

  /// Creates a copy of the current distortion model.
  /// @return Copy of this distortion object.
  inline distortion_t clone() const { return *this; }
  /// Returns the number of distortion coefficients.
  /// @return Number of stored distortion coefficients.
  inline size_t size() const {
    return static_cast<size_t>(_coefficients.size());
  }

  /// Accesses the distortion coefficient at the specified index.
  /// @param index_ Zero-based coefficient index to access.
  /// @return Distortion coefficient value at `index_`.
  inline double operator[](const size_t index_) const {
    const Eigen::Index eigenIndex = static_cast<Eigen::Index>(index_);
    return _coefficients(eigenIndex);
  }

  /// Retrieves the vector of distortion coefficients.
  /// @return Const reference to the full coefficient vector.
  inline const Eigen::VectorXd& coefficients() const { return _coefficients; }

private:
  /// Distortion coefficients.
  Eigen::VectorXd _coefficients;
  /// Indicates whether distortion coefficients are initialized.
  bool _isSet;
};

}  // namespace slam_mer

#endif  // TYPES_CAMERA_HPP
