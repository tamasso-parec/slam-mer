/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file point2d.hpp
/// @brief 2D point type with conversion and retrieval helper functions.
/// @details Provides a compact point2d_t representation, arithmetic/access
/// helpers, and conversions to array/Eigen forms. Includes
/// output-based validation helpers used by tracking and projection
/// code.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPES_POINT2D_HPP
#define TYPES_POINT2D_HPP

// My includes
#include <types/array.hpp>
#include <types/id.hpp>
#include <types/pair.hpp>
#include <types/translation.hpp>
#include <types/vector.hpp>

// Third-party includes
#include <Eigen/Core>

// Standard includes
#include <print>
#include <vector>

namespace slam_mer {

class point2d_t {

public:
  /// Typedef for shared pointer to point2d_t
  using shared_ptr = std::shared_ptr<point2d_t>;
  /// Typedef for unique pointer to point2d_t
  using unique_ptr = std::unique_ptr<point2d_t>;
  /// Typedef for weak pointer to point2d_t
  using weak_ptr = std::weak_ptr<point2d_t>;
  /// Typedef for vector of point2d_t
  using vector = std::vector<point2d_t>;
  /// Pair
  using pair = pair_t<point2d_t, point2d_t>;

  /// Constructors
  point2d_t() : _u(0), _v(0) {}

  /// @brief Constructor with parameters
  /// @param u_ Parameter `u_`.
  /// @param v_ Parameter `v_`.
  point2d_t(const double u_, const double v_) : _u(u_), _v(v_) {}

  /// Copy constructor
  /// @param other_ Parameter `other_`.
  point2d_t(const point2d_t& other_) : _u(other_._u), _v(other_._v) {}

  /// @brief Constructor from array2D_t
  /// @param pointArray_ Parameter `pointArray_`.
  point2d_t(const array2D_t& pointArray_)
      : _u(pointArray_[0]), _v(pointArray_[1]) {}

  /// @brief Constructor from Eigen
  /// @param pointEigen_ Parameter `pointEigen_`.
  point2d_t(const Eigen::Vector2d& pointEigen_)
      : _u(pointEigen_.x()), _v(pointEigen_.y()) {}

  // Operators
  /// @brief Pointer-like access to this point.
  auto operator->() { return this; }
  /// @brief Pointer-like access to this point (const).
  auto operator->() const { return this; }

  /// @brief Defines the == operator for point2d_t.
  /// @param other_ Parameter `other_`.
  /// Compares two point2d_t objects for equality based on their coordinates.
  /// @return true if both points have the same coordinates, false otherwise.
  bool operator==(const point2d_t& other_) const {
    return (_u == other_._u) && (_v == other_._v);
  }

  /// Print function
  /// @param prefix_ Parameter `prefix_`.
  void print(std::string prefix_) const {
    using std::println;
    println("{}point2d_t: ({}, {})", prefix_, _u, _v);
  }

  // Getters

  /// @brief Get the point type variable.
  /// @return Returned value.
  [[deprecated("Use clone() instead")]]
  inline point2d_t get_point() const {
    return point2d_t(_u, _v);
  }

  /// @brief Get the point type variable.
  /// @return Returned value.
  [[nodiscard("Copying by cloning to a new point2d_t.")]]
  inline point2d_t clone() const {
    return point2d_t(_u, _v);
  }

  /// @brief Converts the point to a 2D array representation.
  /// @return Returned value.
  [[nodiscard("Retrieving point as array2D_t.")]]
  inline array2D_t to_2d_array() const {
    return {_u, _v};
  }

  /// @brief Get the eigen coordinates object.
  /// @return Returned value.
  [[nodiscard("Retrieving point as Eigen::Vector2d.")]]
  inline Eigen::Vector2d to_eigen() const {
    return Eigen::Vector2d(_u, _v);
  }

  /// @brief Convert from point2d_t to array2D_t
  /// @return Returned value.
  inline operator array2D_t() const { return this->to_2d_array(); }

  /// @brief Convert from point2d_t to Eigen::Vector2d
  /// @return Returned vector
  inline operator Eigen::Vector2d() const { return this->to_eigen(); }

  /// @brief Overloaded operator to access the coordinates of the point.
  /// @param index_ Parameter `index_`.
  /// @return coordinate value
  /// @throws std::out_of_range if the index is not 0 or 1
  [[deprecated("Use to_2d_array(), to_eigen(), at(), or operators.")]]
  double operator[](const int index_) const {
    if (index_ == 0)
      return _u;
    else if (index_ == 1)
      return _v;
    else
      throw std::out_of_range("Index out of range for point2d_t");
  }

  /// @brief To access the coordinates of the point.
  /// @param index_ Parameter `index_`. 1st correspoints to u, 2nd to v.
  /// @return Returned value.
  [[nodiscard("Use to_2d_array(), to_eigen() or respective operators.")]]
  double at(const int index_) const {
    if (index_ == 0)
      return _u;
    else if (index_ == 1)
      return _v;
    else
      __throw__("Index out of range for point2d_t");
  }

private:
  /// U coordinate
  double _u;
  /// V coordinate
  double _v;
};

// ----------------------------------------------------------------------------
// Some useful sanity checks for points
// ------------------------------------------------------------------------

/// @brief Validates shared point existence and throws on failure.
/// @param pointOut_ Output wrapper holding a shared point pointer.
/// @param pointId_ Point identifier used in the error message.
inline void __sanity_exists(
    const output_t<point2d_t::shared_ptr>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for point " + std::string(pointId_));
}

/// @brief Validates weak point existence and throws on failure.
/// @param pointOut_ Output wrapper holding a weak point pointer.
/// @param pointId_ Point identifier used in the error message.
inline void __sanity_exists(
    const output_t<point2d_t::weak_ptr>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for point " + std::string(pointId_));
}

/// @brief Validates shared point existence and throws on failure.
/// @param pointOut_ Output wrapper holding a shared point pointer.
inline void __sanity_exists(const output_t<point2d_t::shared_ptr>& pointOut_) {
  if (pointOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for point");
}

/// @brief Validates point retrieval and throws on failure.
/// @param pointOut_ Output wrapper holding the point value.
/// @param pointId_ Point identifier used in the error message.
inline void __sanity_retrieve(
    const output_t<point2d_t>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for point " + std::string(pointId_));
}

/// @brief Logs a warning when shared point retrieval fails.
/// @param pointOut_ Output wrapper holding a shared point pointer.
/// @param pointId_ Point identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if shared point retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<point2d_t::shared_ptr>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL) {
    std::println(
        "{}[WARNING]{} Lost pointer for point {}",
        yellow,
        reset,
        std::string(pointId_));
    return true;
  }
  return false;
}

/// @brief Logs a warning when weak point retrieval fails.
/// @param pointOut_ Output wrapper holding a weak point pointer.
/// @param pointId_ Point identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if weak point retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<point2d_t::weak_ptr>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL) {
    std::println(
        "[Point2D] {}Warning{}: Empty pointer for point {}",
        yellow,
        reset,
        std::string(pointId_));
    return true;
  }
  return false;
}

/// @brief Logs a warning when point retrieval fails.
/// @param pointOut_ Output wrapper holding the point value.
/// @param pointId_ Point identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if point retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<point2d_t>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL) {
    std::println(
        "[Point2D] {}Warning{}: Empty pointer for point {}",
        yellow,
        reset,
        std::string(pointId_));
    return true;
  }
  return false;
}

/// @brief Logs a warning when point retrieval fails.
/// @param pointOut_ Output wrapper holding the point value.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if point retrieval failed.")]]
inline bool __fail_retrieve(const output_t<point2d_t>& pointOut_) {
  if (pointOut_ != fail_t::NO_FAIL) {
    std::println(
        "[Point2D] {}Warning{}: Empty pointer for point",
        yellow,
        reset);
    return true;
  }
  return false;
}

}  // namespace slam_mer

#endif  // TYPES_POINT2D_HPP
