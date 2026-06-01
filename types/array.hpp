/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file array.hpp
/// @brief Fixed-size numeric array types and formatting helpers.
/// @details Defines templated fixed-size array wrappers, convenience aliases
/// for common dimensions, stable hashing utilities, and std::formatter
/// support. These lightweight containers are reused across geometry,
/// mapping, and serialization code.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_ARRAY_HPP
#define TYPE_ARRAY_HPP

// Our includes
#include <types/set.hpp>
#include <types/vector.hpp>

// C++ includes
#include <array>
#include <format>

namespace slam_mer {

// --------------------------------------------------------------------------
/// Generic fixed-size array wrapper for type `type` and size `N`.
template <typename type, size_t N>
class arrayNX_t : public std::array<type, N> {
public:
  using std::array<type, N>::array;
  using vector = vector_t<arrayNX_t<type, N>>;
  using pair = std::pair<arrayNX_t<type, N>, arrayNX_t<type, N>>;
  using set = set_t<arrayNX_t<type, N>>;
  /// Default constructor
  arrayNX_t() = default;
  /// Construct a new arrayNX t object from std::array
  /// @param array_ Source fixed-size array copied into this wrapper.
  arrayNX_t(const std::array<type, N>& array_) {
    for (size_t i = 0; i < N; ++i) {
      this->at(i) = array_[i];
    }
  }

  /// Addition operator
  /// @param other_ Array added component-wise to this array.
  /// @return Component-wise sum of the two arrays.
  arrayNX_t operator+(const arrayNX_t& other_) const {
    arrayNX_t result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = (*this)[i] + other_[i];
    }
    return result;
  }

  /// Subtraction operator
  /// @param other_ Array subtracted component-wise from this array.
  /// @return Component-wise difference of the two arrays.
  arrayNX_t operator-(const arrayNX_t& other_) const {
    arrayNX_t result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = (*this)[i] - other_[i];
    }
    return result;
  }

  /// Multiplication by scalar operator
  /// @param scalar_ Scalar multiplier applied to each component.
  /// @return Array with each component multiplied by `scalar_`.
  arrayNX_t operator*(const type& scalar_) const {
    arrayNX_t result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = static_cast<type>((*this)[i] * scalar_);
    }
    return result;
  }

  /// Division by scalar operator
  /// @param scalar_ Scalar divisor applied to each component.
  /// @return Array with each component divided by `scalar_`.
  arrayNX_t operator/(const type& scalar_) const {
    arrayNX_t result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = static_cast<type>((*this)[i] / scalar_);
    }
    return result;
  }

  /// Allow {v0, v1, ...} to construct arrayNX_t
  /// @brief Constructs from exactly `N` values convertible to `type`.
  /// @param args Values copied in order into the fixed-size array.
  template <class... Args>
  explicit arrayNX_t(Args... args)
    requires(sizeof...(Args) == N) &&
            (std::conjunction_v<std::is_convertible<Args, type>...>)
      : std::array<type, N>{static_cast<type>(args)...} {}
};

// --------------------------------------------------------------------------
// Type definitions for common fixed-size arrays
/// Fixed-size array of doubles.
template <size_t N>
using arrayND_t = arrayNX_t<double, N>;
/// Fixed-size array of floats.
template <size_t N>
using arrayNF_t = arrayNX_t<float, N>;
/// Fixed-size array of ints.
template <size_t N>
using arrayNInt_t = arrayNX_t<int, N>;
/// Fixed-size array of unsigned chars.
template <size_t N>
using arrayNUchar_t = arrayNX_t<unsigned char, N>;
/// Fixed-size array of size_t.
template <size_t N>
using arrayNSize_t = arrayNX_t<size_t, N>;
/// Fized-size array of uint16_t.
template <size_t N>
using arrayNInt16_t = arrayNX_t<int16_t, N>;

// --------------------------------------------------------------------------
/// 2D array of doubles.
class array2D_t : public arrayND_t<2> {
  using base_t = arrayND_t<2>;

public:
  using base_t::base_t;
  using vector = vector_t<array2D_t>;
  /// Default constructor
  array2D_t() = default;
  /// Construct a new array2D t object
  /// @param u_ First component of the 2D array.
  /// @param v_ Second component of the 2D array.
  array2D_t(const double u_, const double v_) {
    this->at(0) = u_;
    this->at(1) = v_;
  }
};

/// 3D array of doubles.
class array3D_t : public arrayND_t<3> {
  using base_t = arrayND_t<3>;

public:
  using base_t::base_t;
  using vector = vector_t<array3D_t>;
  /// Default constructor
  array3D_t() = default;
  /// Construct a new array3D t object
  /// @param x_ X component of the 3D array.
  /// @param y_ Y component of the 3D array.
  /// @param z_ Z component of the 3D array.
  array3D_t(const double x_, const double y_, const double z_) {
    this->at(0) = x_;
    this->at(1) = y_;
    this->at(2) = z_;
  }
};

/// 4D array of doubles.
class array4D_t : public arrayND_t<4> {
  using base_t = arrayND_t<4>;

public:
  using base_t::base_t;
  using vector = vector_t<array4D_t>;
  /// Default constructor
  array4D_t() = default;
  /// Construct a new array4D t object
  /// @param x_ First component of the 4D array.
  /// @param y_ Second component of the 4D array.
  /// @param z_ Third component of the 4D array.
  /// @param w_ Fourth component of the 4D array.
  array4D_t(
      const double x_,
      const double y_,
      const double z_,
      const double w_) {
    this->at(0) = x_;
    this->at(1) = y_;
    this->at(2) = z_;
    this->at(3) = w_;
  }
};

/// 6D array of doubles.
class array6D_t : public arrayND_t<6> {
  using base_t = arrayND_t<6>;

public:
  using base_t::base_t;
  using vector = vector_t<array6D_t>;
  /// Default constructor
  array6D_t() = default;
  /// Construct a new array6D t object
  /// @param x_ First spatial component.
  /// @param y_ Second spatial component.
  /// @param z_ Third spatial component.
  /// @param r_ First extra component (often red or roll).
  /// @param g_ Second extra component (often green or pitch).
  /// @param b_ Third extra component (often blue or yaw).
  array6D_t(
      const double x_,
      const double y_,
      const double z_,
      const double r_,
      const double g_,
      const double b_) {
    this->at(0) = x_;
    this->at(1) = y_;
    this->at(2) = z_;
    this->at(3) = r_;
    this->at(4) = g_;
    this->at(5) = b_;
  }
};

// --------------------------------------------------------------------------
// Float arrays
/// 2D array of floats.
using array2F_t = arrayNF_t<2>;
/// 3D array of floats.
using array3F_t = arrayNF_t<3>;

// --------------------------------------------------------------------------
// Arrays of integers
/// 2D array of ints.
using array2Int_t = arrayNInt_t<2>;
/// 3D array of ints.
using array3Int_t = arrayNInt_t<3>;

// --------------------------------------------------------------------------
// Array of size_t
/// 2D array of size_t.
using array2Size_t = arrayNSize_t<2>;

// --------------------------------------------------------------------------
// Array uchar
/// 2D array of unsigned chars.
using array2Uchar_t = arrayNUchar_t<2>;
/// 3D array of unsigned chars.
using array3Uchar_t = arrayNUchar_t<3>;

// --------------------------------------------------------------------------
// Hash functions
// --------------------------------------------------------------------------
static inline uint64_t __mix64(uint64_t x_) noexcept {
  x_ += 0x9e3779b97f4a7c15ULL;
  x_ = (x_ ^ (x_ >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x_ = (x_ ^ (x_ >> 27)) * 0x94d049bb133111ebULL;
  return x_ ^ (x_ >> 31);
}

}  // namespace slam_mer

namespace std {
/// Hash function for array3Int_t
template <>
struct hash<slam_mer::array3Int_t> {
  /// @brief Computes a stable hash for a 3D integer array key.
  /// @param a_ Integer triplet used as the hash key.
  /// @return Hash value suitable for unordered associative containers.
  std::size_t operator()(const slam_mer::array3Int_t& a_) const noexcept {
    const uint64_t x = static_cast<uint32_t>(a_[0]);
    const uint64_t y = static_cast<uint32_t>(a_[1]);
    const uint64_t z = static_cast<uint32_t>(a_[2]);
    uint64_t h = slam_mer::__mix64(x);
    h ^= slam_mer::__mix64(y + 0x9e3779b9U);
    h ^= slam_mer::__mix64(z + 0x85ebca6bU);
    h = slam_mer::__mix64(h);
    return static_cast<std::size_t>(h);
  }
};

/// Formatter for arrayNX_t
template <typename type, size_t N, typename CharT>
struct formatter<slam_mer::arrayNX_t<type, N>, CharT> {
  /// @brief Parses format specifiers for each array value.
  /// @param ctx_ Parse context containing format-specifier input.
  /// @return Iterator to the end of the parsed format specifier range.
  constexpr auto parse(basic_format_parse_context<CharT>& ctx_) {
    return valueFormatter.parse(ctx_);
  }

  /// @brief Formats an array as `[v0, v1, ...]`.
  /// @param value_ Array value whose elements are formatted.
  /// @param ctx_ Format context receiving the generated characters.
  /// @return Iterator to the end of the written output range.
  template <typename FormatContext>
  auto format(const slam_mer::arrayNX_t<type, N>& value_, FormatContext& ctx_)
      const {
    auto out = ctx_.out();
    *out++ = '[';
    for (size_t i = 0; i < N; ++i) {
      if (i != 0) {
        *out++ = ',';
        *out++ = ' ';
      }
      out = valueFormatter.format(value_[i], ctx_);
    }
    *out++ = ']';
    return out;
  }

private:
  /// @brief Formatter reused for each individual element.
  mutable formatter<type, CharT> valueFormatter;
};

/// Formatter specializations for common array types (array2D_t)
template <typename CharT>
struct formatter<slam_mer::array2D_t, CharT>
    : formatter<slam_mer::arrayNX_t<double, 2>, CharT> {};

/// Formatter specialization for array3D_t
template <typename CharT>
struct formatter<slam_mer::array3D_t, CharT>
    : formatter<slam_mer::arrayNX_t<double, 3>, CharT> {};

/// Formatter specialization for array4D_t
template <typename CharT>
struct formatter<slam_mer::array4D_t, CharT>
    : formatter<slam_mer::arrayNX_t<double, 4>, CharT> {};

/// Formatter specialization for array6D_t
template <typename CharT>
struct formatter<slam_mer::array6D_t, CharT>
    : formatter<slam_mer::arrayNX_t<double, 6>, CharT> {};
}  // namespace std

#endif  // TYPE_ARRAY_HPP
