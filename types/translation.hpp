/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file translation.hpp
/// @brief 3D translation vector type and conversion helpers.
/// @details Wraps Eigen::Vector3d with convenience constructors and conversion
/// methods to project array and visualization-friendly
/// representations. Used with rotation and pose abstractions.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TRANSLATION_HPP
#define TRANSLATION_HPP

// My includes
#include <types/array.hpp>

// Third-party includes
#include <Eigen/Core>
#include <format>

namespace slam_mer {

/// Translation type, using Eigen Vector3d
class translation_t : public Eigen::Vector3d {
public:
  /// Inherit constructors from Eigen::Vector3d
  using Eigen::Vector3d::Vector3d;

  /// Convert to array3D_t (custom array type)
  /// @return Translation components as `{x, y, z}` in double precision.
  array3D_t to_array() const {
    return array3D_t{this->x(), this->y(), this->z()};
  }

  /// Convert to Eigen::Vector3d
  /// @return Copy of this translation as an `Eigen::Vector3d`.
  Eigen::Vector3d to_eigen() const {
    return Eigen::Vector3d(this->x(), this->y(), this->z());
  }

  /// Scale the translation by a scalar value
  /// @param scalar_ Scale factor applied to each translation component.
  /// @return Scaled translation vector.
  translation_t operator*(double scalar_) const {
    return translation_t(
        this->x() * scalar_,
        this->y() * scalar_,
        this->z() * scalar_);
  }

  /// Convert to Eigen::Vector3f (float precision, for rerun)
  /// @return Translation components cast to `float` for visualization.
  Eigen::Vector3f to_rerun() const {
    return Eigen::Vector3f(this->x(), this->y(), this->z());
  }

  /// @brief Convert to array3D_t
  /// @return Translation components as `{x, y, z}` in double precision.
  inline operator array3D_t() const { return to_array(); }
};

}  // namespace slam_mer

/// Formatter for translation_t
namespace std {
template <>
struct formatter<slam_mer::translation_t> : formatter<string> {
  /// Format translation_t as a string: [x: ..., y: ..., z: ...]
  /// @param t_ Translation object whose coordinates will be formatted.
  /// @param ctx_ Format context that receives the generated text.
  /// @return Iterator to the end of the written characters in `ctx_`.
  template <typename FormatContext>
  auto format(const slam_mer::translation_t& t_, FormatContext& ctx_) const {
    return std::format_to(
        ctx_.out(),
        "[x: {}, y: {}, z: {}]",
        t_.x(),
        t_.y(),
        t_.z());
  }
};
}  // namespace std

#endif  // TRANSLATION_HPP
