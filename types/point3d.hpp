/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file point3d.hpp
/// @brief 3D point representation with color, neighborhood, and utility
/// operations.
/// @details Defines point3d_t with coordinate and color handling, geometric
/// operators, conversions, and neighbor metadata structures. Also
/// includes output-based retrieval checks for robust map-point access.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPES_POINT3D_HPP
#define TYPES_POINT3D_HPP

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
#include <shared_mutex>
#include <string>
#include <vector>

namespace slam_mer {

/// @brief 3D point representation for the SLAM framework.
///
/// This class stores position and color and exposes conversion and arithmetic
/// helpers used across mapping, tracking, and optimization modules.
class point3d_t {

public:
  // ------------------------------------------------------------------------
  // Typedefs and constants

  /// Typedef for shared pointer to point3d_t
  using shared_ptr = std::shared_ptr<point3d_t>;
  /// Typedef for weak pointer to point3d_t
  using weak_ptr = std::weak_ptr<point3d_t>;
  /// Bad pointer
  inline static const point3d_t::shared_ptr BAD_POINTER = nullptr;
  /// Make shared pointer function
  /// @return Returned the new empty object.
  [[nodiscard("Use to create a shared pointer to a point3d_t.")]]
  static point3d_t::shared_ptr make_shared() {
    return std::make_shared<point3d_t>();
  }
  /// Sets the coordinates of a point3d_t object from a weak pointer.
  /// @param pointPtr_ Parameter `pointPtr_`.
  /// @param point_ Parameter `point_`.
  /// @return Returned the pointer to the new object.
  static bool set_coordinates(
      point3d_t::weak_ptr pointPtr_,
      const array3D_t& point_) {
    if (auto sharedPtr = pointPtr_.lock()) {
      const array3D_t pointArray = point_;
      const double& x = pointArray[0];
      const double& y = pointArray[1];
      const double& z = pointArray[2];
      sharedPtr->set_coordinates(x, y, z);
      return true;
    }
    return false;
  }

  /// Make shared pointer function with parameters
  /// @param x_ Parameter `x_`.
  /// @param y_ Parameter `y_`.
  /// @param z_ Parameter `z_`.
  /// @param red_ Parameter `red_`.
  /// @param green_ Parameter `green_`.
  /// @param blue_ Parameter `blue_`.
  /// @return Returned the pointer to the new object.
  [[nodiscard(
      "Use to create a shared pointer to a point3d_t with parameters.")]]
  static point3d_t::shared_ptr make_shared(
      const double x_,
      const double y_,
      const double z_,
      const double red_ = 0,
      const double green_ = 0,
      const double blue_ = 0) {
    return std::make_shared<point3d_t>(x_, y_, z_, red_, green_, blue_);
  }

  /// Make shared pointer function from another point3d_t
  /// @param other_ Parameter `other_`.
  /// @return Returned the pointer to the new object.
  static point3d_t::shared_ptr make_shared(const point3d_t& other_) {
    return std::make_shared<point3d_t>(other_);
  }

  /// Typedef for unique pointer to point3d_t
  using unique_ptr = std::unique_ptr<point3d_t>;
  /// Typedef for vector of point3d_t
  using vector = vector_t<point3d_t>;
  /// Typedef for pair of point3d_t
  using pair = pair_t<point3d_t, point3d_t>;

  /// Typedef a pair of id and shared pointer to point3d_t
  struct id_pair {
    using vector = std::vector<id_pair>;
    /// Id of the neighbor point
    point_id_t pointId;
    /// Id of the keyframe
    keyframe_id_t keyframeId;
    /// Shared pointer to the neighbor point
    point3d_t::shared_ptr shared_ptr;
    /// Constructor
    /// @param pointId_ Parameter `pointId_`.
    /// @param keyframeId_ Parameter `keyframeId_`.
    /// @param ptr_ Parameter `ptr_`.
    id_pair(
        point_id_t pointId_,
        keyframe_id_t keyframeId_,
        point3d_t::shared_ptr ptr_)
        : pointId(pointId_), keyframeId(keyframeId_), shared_ptr(ptr_) {}
  };

  // ------------------------------------------------------------------------
  // Modules

  /// @brief Default constructor initializing all coordinates and color to zero.
  point3d_t()
      : _x(std::numeric_limits<double>::max()),
        _y(std::numeric_limits<double>::max()),
        _z(std::numeric_limits<double>::max()),
        _red(std::numeric_limits<double>::max()),
        _green(std::numeric_limits<double>::max()),
        _blue(std::numeric_limits<double>::max()) {}

  /// @brief Construct a 3D point with position and color.
  /// @param x_ Parameter `x_`.
  /// @param y_ Parameter `y_`.
  /// @param z_ Parameter `z_`.
  /// @param red_ Parameter `red_`.
  /// @param green_ Parameter `green_`.
  /// @param blue_ Parameter `blue_`.
  point3d_t(
      const double x_,
      const double y_,
      const double z_,
      const double red_ = 0,
      const double green_ = 0,
      const double blue_ = 0)
      : _x(x_), _y(y_), _z(z_), _red(red_), _green(green_), _blue(blue_) {}

  /// @brief Construct a new Point 3d object.
  /// @param other_ Parameter `other_`.
  point3d_t(const point3d_t& other_) {
    const array6D_t pointArray = other_;
    _x = pointArray[0];
    _y = pointArray[1];
    _z = pointArray[2];
    _red = pointArray[3];
    _green = pointArray[4];
    _blue = pointArray[5];
  }

  /// @brief Construct a new Point 3d object.
  /// @param other_ Parameter `other_`.
  point3d_t(point3d_t::shared_ptr other_) {
    const array6D_t pointArray = other_->to_6d_array();
    _x = pointArray[0];
    _y = pointArray[1];
    _z = pointArray[2];
    _red = pointArray[3];
    _green = pointArray[4];
    _blue = pointArray[5];
  }

  /// @brief  Construct a new Point 3d object from a weak pointer
  /// @param other_ Parameter `other_`.
  point3d_t(const point3d_t::weak_ptr other_) {
    if (auto shared = other_.lock()) {
      const array6D_t pointArray = shared->to_6d_array();
      _x = pointArray[0];
      _y = pointArray[1];
      _z = pointArray[2];
      _red = pointArray[3];
      _green = pointArray[4];
      _blue = pointArray[5];
    } else {
      _x = std::numeric_limits<double>::max();
      _y = std::numeric_limits<double>::max();
      _z = std::numeric_limits<double>::max();
      _red = std::numeric_limits<double>::max();
      _green = std::numeric_limits<double>::max();
      _blue = std::numeric_limits<double>::max();
    }
  }

  /// @brief Construct a new Point 3d object.
  /// @param pointEigen_ Parameter `pointEigen_`.
  /// @param colorEigen_ Parameter `colorEigen_`.
  point3d_t(
      const Eigen::Vector3d& pointEigen_,
      const Eigen::Vector3d& colorEigen_ = Eigen::Vector3d(0, 0, 0))
      : _x(pointEigen_.x()),
        _y(pointEigen_.y()),
        _z(pointEigen_.z()),
        _red(colorEigen_.x()),
        _green(colorEigen_.y()),
        _blue(colorEigen_.z()) {}

  /// @brief Constructs a 3D point from an array of three doubles.
  /// @param pointArray_ Parameter `pointArray_`.
  point3d_t(const array3D_t& pointArray_)
      : _x(pointArray_[0]),
        _y(pointArray_[1]),
        _z(pointArray_[2]),
        _red(0),
        _green(0),
        _blue(0) {}

  // Operators
  /// @brief Pointer-like access to this point.
  [[deprecated("Avoid using these kind of operators")]]
  auto operator->() {
    return this;
  }
  /// @brief Pointer-like access to this point (const).
  [[deprecated("Avoid using these kind of operators")]]
  auto operator->() const {
    return this;
  }

  /// @brief Checks if the point is set (i.e., has valid coordinates and color).
  /// @return true if was defined before
  /// @return false if was not defined before
  [[nodiscard("Use to check if the point is set or not.")]]
  bool is_unset() const {
    using std::numeric_limits;
    read_lock_t lock(_mutex);
    return (_x == numeric_limits<double>::max()) &&
           (_y == numeric_limits<double>::max()) &&
           (_z == numeric_limits<double>::max());
  }

  /// @brief Returns true when the point is unset.
  inline bool operator!() const { return is_unset(); }

  /// @brief Checks if the point is set (i.e., has valid coordinates and color).
  /// @return true if was defined before
  /// @return false if was not defined before
  inline operator bool() const { return !is_unset(); }

  /// @brief Prints the 3D point coordinates and color values
  void print() const {
    using std::println;
    read_lock_t lock(_mutex);
    println(
        "point3d_t: ({}, {}, {}), Color: ({}, {}, {})",
        _x,
        _y,
        _z,
        _red,
        _green,
        _blue);
  }

  /// @brief Defines the == operator for point3d_t.
  /// @param other_ Parameter `other_`.
  bool operator==(const point3d_t& other_) const {
    read_lock_t lock_this(_mutex);
    read_lock_t lock_other(other_._mutex);
    return (_x == other_._x) && (_y == other_._y) && (_z == other_._z) &&
           (_red == other_._red) && (_green == other_._green) &&
           (_blue == other_._blue);
  }

  /// @brief Assignment operator for point3d_t.
  /// @param other_ Parameter `other_`.
  /// @return Reference to this object
  point3d_t& operator=(const point3d_t& other_) {
    if (this != &other_) {
      write_lock_t lock_write(_mutex);
      read_lock_t lock_read(other_._mutex);
      array6D_t pointArray = other_.to_6d_array();
      _x = pointArray[0];
      _y = pointArray[1];
      _z = pointArray[2];
      _red = pointArray[3];
      _green = pointArray[4];
      _blue = pointArray[5];
    }
    return *this;
  }

  /// @brief Scales the coordinates of the point3d_t by a given factor.
  /// @param scale_ Parameter `scale_`.
  /// @return Reference to the scaled point3d_t.
  point3d_t& operator*=(const double scale_) {
    write_lock_t lock(_mutex);
    _x *= scale_;
    _y *= scale_;
    _z *= scale_;
    return *this;
  }

  /// @brief Adds the coordinates of another point3d_t to this point3d_t.
  /// @param other_ Parameter `other_`.
  /// @return The result of the coordinate-wise addition.
  point3d_t operator+(const point3d_t& other_) {
    // Copy, means that the result will have the same color
    point3d_t result(*this);
    read_lock_t lock_this(_mutex);
    read_lock_t lock_other(other_._mutex);
    const array3D_t otherPointArray = other_.to_3d_array();
    const array3D_t thisPointArray = this->to_3d_array();
    result.set_coordinates(
        thisPointArray[0] + otherPointArray[0],
        thisPointArray[1] + otherPointArray[1],
        thisPointArray[2] + otherPointArray[2]);
    return result;
  }

  /// @brief Adds the coordinates of another point3d_t to this point3d_t.
  /// @param other_ Parameter `other_`.
  /// @return The result of the coordinate-wise addition.
  point3d_t operator+(const translation_t& other_) {
    point3d_t result(*this);
    array3D_t thisPointArray;
    {  // only lock when needed
      read_lock_t lock(_mutex);
      thisPointArray = this->to_3d_array();
    }
    result.set_coordinates(
        thisPointArray[0] + other_[0],
        thisPointArray[1] + other_[1],
        thisPointArray[2] + other_[2]);
    return result;
  }

  /// @brief Adds the coordinates of another point3d_t to this point
  /// @param other_ Parameter `other_`.
  /// @return Reference to this point3d_t object after addition.
  point3d_t& operator+=(const point3d_t& other_) {
    if (this == &other_) {
      write_lock_t lock_write(_mutex);
      _x += _x;
      _y += _y;
      _z += _z;
      return *this;
    }
    write_lock_t lock_write(_mutex);
    read_lock_t lock_read(other_._mutex);
    const array3D_t pointArray = other_.to_3d_array();
    _x += pointArray[0];
    _y += pointArray[1];
    _z += pointArray[2];
    return *this;
  }

  /// @brief Subtracts the coordinates of another point3d_t from this point3d_t.
  /// @param other_ Parameter `other_`.
  /// @return The result of the coordinate-wise subtraction.
  point3d_t operator-(const point3d_t& other_) const {
    // Copy, means that the result will have the same color
    point3d_t result(*this);
    array3D_t thisArray;
    array3D_t otherArray;
    {  // Only lock when needed
      read_lock_t lock_this(_mutex);
      read_lock_t lock_other(other_._mutex);
      thisArray = this->to_3d_array();
      otherArray = other_.to_3d_array();
    }
    result.set_coordinates(
        thisArray[0] - otherArray[0],
        thisArray[1] - otherArray[1],
        thisArray[2] - otherArray[2]);
    return result;
  }

  /// @brief Subtracts the coordinates of another point3d_t from this point3d_t.
  /// @param other_ Parameter `other_`.
  /// @return The result of the coordinate-wise subtraction.
  point3d_t operator-(const translation_t& other_) const {
    point3d_t result(*this);
    read_lock_t lock(_mutex);
    const array3D_t thisArray = this->to_3d_array();
    result.set_coordinates(
        thisArray[0] - other_[0],
        thisArray[1] - other_[1],
        thisArray[2] - other_[2]);
    return result;
  }

  /// @brief Subtracts the coordinates of another point3d_t
  /// @param other_ Parameter `other_`.
  /// from this point and assigns the result to this point.
  /// @return Reference to this point3d_t object after subtraction.
  point3d_t& operator-=(const point3d_t& other_) {
    if (this == &other_) {
      write_lock_t lock_write(_mutex);
      _x -= _x;
      _y -= _y;
      _z -= _z;
      return *this;
    }
    write_lock_t lock_write(_mutex);
    read_lock_t lock_read(other_._mutex);
    const array3D_t pointArray = other_.to_3d_array();
    _x -= pointArray[0];
    _y -= pointArray[1];
    _z -= pointArray[2];
    return *this;
  }

  /// @brief Subtracts the coordinates of another point3d_t from this point
  /// @param other_ Parameter `other_`.
  /// and assigns the result to this point.
  /// @return Reference to this point3d_t object after subtraction.
  point3d_t& operator-=(const point3d_t::shared_ptr other_) {
    if (this == other_.get()) {
      write_lock_t lock_write(_mutex);
      _x -= _x;
      _y -= _y;
      _z -= _z;
      return *this;
    }
    write_lock_t lock_write(_mutex);
    read_lock_t lock_read(other_->_mutex);
    const array3D_t pointArray = other_->to_3d_array();
    _x -= pointArray[0];
    _y -= pointArray[1];
    _z -= pointArray[2];
    return *this;
  }

  // -----------------------------------------------------------------------
  // Setters

  /// @brief Set the coordinates object
  /// @param coordinates_ New coordinates for the point`.
  [[deprecated(
      "Use set_coordinates(const double x_, const double y_, \
      const double z_) instead")]]
  inline void set_coordinates(const Eigen::Vector3d& coordinates_) {
    write_lock_t lock(_mutex);
    _x = coordinates_.x();
    _y = coordinates_.y();
    _z = coordinates_.z();
  }

  /// @brief Set the color of the point using an Eigen vector.
  /// @param color_ New color for the point.
  inline void set_color(const Eigen::Vector3d& color_) {
    write_lock_t lock(_mutex);
    _red = color_.x();
    _green = color_.y();
    _blue = color_.z();
  }

  /// @brief Set the color of the point using an Eigen vector.
  /// @param color_ New color for the point.
  [[deprecated(
      "Use set_color(const double red_, const double green_, \
      const double blue_) instead")]]
  inline void set_color(const array3Uchar_t& color_) {
    write_lock_t lock(_mutex);
    _red = static_cast<double>(color_[0]);
    _green = static_cast<double>(color_[1]);
    _blue = static_cast<double>(color_[2]);
  }

  /// @brief Set the coordinates object
  /// @param coordinates_ New coordinates for the point.
  /// @return The previous coordinates of the point
  [[deprecated(
      "Use set_coordinates(const double x_, const double y_, \
      const double z_) instead")]]
  inline array3D_t set_coordinates(const array3D_t& coordinates_) {
    write_lock_t lock(_mutex);
    array3D_t previousCoordinates = {_x, _y, _z};
    _x = coordinates_[0];
    _y = coordinates_[1];
    _z = coordinates_[2];
    return previousCoordinates;
  }

  /// @brief Set the coordinates of the point using individual double values.
  /// @param x_ New x coordinate for the point.
  /// @param y_ New y coordinate for the point.
  /// @param z_ New z coordinate for the point.
  inline void
  set_coordinates(const double x_, const double y_, const double z_) {
    write_lock_t lock(_mutex);
    _x = x_;
    _y = y_;
    _z = z_;
  }

  /// @brief Set the color of the point using individual double values.
  /// @param red_ Parameter `red_`.
  /// @param green_ Parameter `green_`.
  /// @param blue_ Parameter `blue_`.
  inline void
  set_color(const double red_, const double green_, const double blue_) {
    write_lock_t lock(_mutex);
    _red = red_;
    _green = green_;
    _blue = blue_;
  }

  /// @brief Access the coordinates of the point.
  /// @param index_ Parameter `index_`.
  [[nodiscard(
      "Used to get the coordinate value at a specific index \
    (0 for x, 1 for y, 2 for z).")]]
  double operator[](const int index_) const {
    if (index_ == 0)
      return _x;
    else if (index_ == 1)
      return _y;
    else if (index_ == 2)
      return _z;
    else
      __throw__("Index out of range for point3d_t");
  }

  // -----------------------------------------------------------------------
  // Conversion methods

  /// @brief Get the 3D point coordinates as an Eigen vector.
  /// @return Returned value.
  [[nodiscard("Use to get the 3D point coordinates as an Eigen vector.")]]
  inline Eigen::Vector3d to_eigen() const {
    read_lock_t lock(_mutex);
    return Eigen::Vector3d(_x, _y, _z);
  }

  /// @brief Spatial and color components to a 6-element array.
  /// @return Returned value.
  [[nodiscard("Use to get the 3D point coordinates and color as an array.")]]
  inline array6D_t to_6d_array() const {
    read_lock_t lock(_mutex);
    return {_x, _y, _z, _red, _green, _blue};
  }

  /// @brief Converts the point coordinates to a 3D array
  /// @return Returned value.
  [[nodiscard("Use to get the 3D point coordinates as an array.")]]
  inline array3D_t to_3d_array() const {
    read_lock_t lock(_mutex);
    return {_x, _y, _z};
  }

  /// Conversion operators: array3D
  /// @return Returned value.
  inline operator array3D_t() const { return this->to_3d_array(); }

  /// Conversion operators: array6D
  /// @return Returned value.
  inline operator array6D_t() const { return this->to_6d_array(); }

  /// Conversion operators: Eigen::Vector3d
  /// @return Returned value.
  inline operator Eigen::Vector3d() const { return this->to_eigen(); }

  /// Conversion operators: std::string
  /// @return Returned value.
  inline operator std::string() const {
    read_lock_t lock(_mutex);
    return "point3d_t: (" + std::to_string(_x) + ", " + std::to_string(_y) +
           ", " + std::to_string(_z) + "), Color: (" + std::to_string(_red) +
           ", " + std::to_string(_green) + ", " + std::to_string(_blue) + ")";
  }

  /// @brief Apply a scale factor to the point coordinates.
  /// @param scale_ Scale parameter.
  inline void apply_scale(const double scale_) {
    write_lock_t lock(_mutex);
    _x *= scale_;
    _y *= scale_;
    _z *= scale_;
  }

private:
  double _x;                         //< X coordinate
  double _y;                         //< Y coordinate
  double _z;                         //< Z coordinate
  double _red;                       //< Color red component
  double _green;                     //< Color green component
  double _blue;                      //< Color blue component
  mutable std::shared_mutex _mutex;  //< Mutex for thread safety
};

// ----------------------------------------------------------------------------

// To keep a nicer and denser representation of our world, we
// associate to each 3D point a list of neighbors
/// @brief Class representing a neighbor of a 3D point.
class neighbor_t : public point3d_t::id_pair {
public:
  // Type alias for a vector of neighbors
  using vector = vector_t<neighbor_t>;

  /// @brief Constructs a new neighbor link.
  /// @param pointId_ Parameter `pointId_`.
  /// @param keyframeId_ Parameter `keyframeId_`.
  /// @param neighbor_ Parameter `neighbor_`.
  neighbor_t(
      const point_id_t pointId_,
      const keyframe_id_t keyframeId_,
      const point3d_t::shared_ptr neighbor_)
      : point3d_t::id_pair(pointId_, keyframeId_, neighbor_) {}

  // Getters
  /// @brief Get the neighbor object
  /// @return the pointer to the neighbor point
  [[nodiscard("Use to get the neighbor point object.")]]
  inline point3d_t::shared_ptr get_neighbor() const {
    return this->shared_ptr;
  }

  /// @brief Get the neighbor id
  /// @return Id of the neighbor point
  [[nodiscard("Use to get the neighbor point id.")]]
  inline point_id_t id() const {
    return this->pointId;
  }

  /// @brief Get the keyframe id
  /// @return Returned value.
  [[nodiscard("Use to get the keyframe id associated with neighbor point.")]]
  inline keyframe_id_t get_keyframe_id() const {
    return this->keyframeId;
  }

  /// @brief Change the neighbor point.
  /// @param neighbor_ Parameter `neighbor_`.
  inline void set_neighbor(point3d_t::shared_ptr neighbor_) {
    this->shared_ptr = neighbor_;
  }

  /// Conversion operators: point3d
  /// @return Returned value.
  operator point3d_t() const { return point3d_t(this->shared_ptr); }

  /// Conversion operators: shared pointer to point3d
  /// @return Returned value.
  operator point3d_t::shared_ptr() const { return this->shared_ptr; }

  /// Conversion operators: point id
  /// @return Returned value.
  operator point_id_t() const { return this->pointId; }

  /// Conversion operators: keyframe id
  /// @return Returned value.
  operator keyframe_id_t() const { return this->keyframeId; }
};

// ----------------------------------------------------------------------------
// Some useful sanity checks for points
// ------------------------------------------------------------------------

/// @brief Validates shared point retrieval and throws on failure.
/// @param pointOut_ Output wrapper holding a shared point pointer.
/// @param pointId_ Point identifier used in the error message.
inline void __sanity_retrieve(
    const output_t<point3d_t::shared_ptr>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for point " + std::string(pointId_));
}

/// @brief Validates weak point retrieval and throws on failure.
/// @param pointOut_ Output wrapper holding a weak point pointer.
/// @param pointId_ Point identifier used in the error message.
inline void __sanity_retrieve(
    const output_t<point3d_t::weak_ptr>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for point " + std::string(pointId_));
}

/// @brief Validates shared point retrieval and throws on failure.
/// @param pointOut_ Output wrapper holding a shared point pointer.
inline void __sanity_retrieve(
    const output_t<point3d_t::shared_ptr>& pointOut_) {
  if (pointOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for point");
}

/// @brief Validates point retrieval and throws on failure.
/// @param pointOut_ Output wrapper holding the point value.
/// @param pointId_ Point identifier used in the error message.
inline void __sanity_retrieve(
    const output_t<point3d_t>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for point " + std::string(pointId_));
}

/// @brief Validates point set output and throws if unset.
/// @param pointOut_ Point value to validate.
/// @param pointId_ Point identifier used in the error message.
inline void __sanity_set(
    const point3d_t& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_.is_unset())
    __throw__("Unexpected: Empty pointer for point " + std::string(pointId_));
}

/// @brief Logs a warning when shared point retrieval fails.
/// @param pointOut_ Output wrapper holding a shared point pointer.
/// @param pointId_ Point identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if shared point retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<point3d_t::shared_ptr>& pointOut_,
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
[[nodiscard("Use to check if shared point retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<point3d_t::weak_ptr>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL) {
    std::println(
        "[Point3D] {}Warning{}: Empty pointer for point {}",
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
[[nodiscard("Use to check if shared point retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<point3d_t>& pointOut_,
    const point_id_t& pointId_) {
  if (pointOut_ != fail_t::NO_FAIL) {
    std::println(
        "[Point3D] {}Warning{}: Empty pointer for point {}",
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
[[nodiscard("Use to check if shared point retrieval failed.")]]
inline bool __fail_retrieve(const output_t<point3d_t>& pointOut_) {
  if (pointOut_ != fail_t::NO_FAIL) {
    std::println(
        "[Point3D] {}Warning{}: Empty pointer for point",
        yellow,
        reset);
    return true;
  }
  return false;
}

/// @brief Logs a warning when a point-set result is unset.
/// @param pointOut_ Point value to validate.
/// @param pointId_ Point identifier used in the warning.
/// @return `true` if point is unset, `false` otherwise.
[[nodiscard("Use to check if shared point retrieval failed.")]]
inline bool __fail_set(const point3d_t& pointOut_, const point_id_t& pointId_) {
  if (pointOut_.is_unset()) {
    std::println(
        "[Point3D] {}Warning{}: Empty pointer for point {}",
        yellow,
        reset,
        std::string(pointId_));
    return true;
  }
  return false;
}

}  // namespace slam_mer

#endif  // TYPES_POINT3D_HPP
