/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file id.hpp
/// @brief Strongly typed identifier classes and hashing support.
/// @details Implements CRTP-based IDs for type-safe indexing of frames, points,
/// keyframes, and cells. Includes arithmetic and comparison helpers
/// plus std::hash specializations for container integration.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPES_ID_HPP
#define TYPES_ID_HPP

// My includes
#include <types/mutex.hpp>
#include <types/pair.hpp>
#include <types/queue.hpp>
#include <types/set.hpp>
#include <types/vector.hpp>

// Standard libraries
#include <cstddef>
#include <cstdint>
#include <limits>
#include <print>
#include <string>

namespace slam_mer {

/// @brief Generic CRTP ID wrapper for strong typing and utilities.
///
/// This template wraps an integer ID type and provides typed arithmetic,
/// comparisons, and helpers while preserving the derived ID type.
/// @tparam derived_id_t Strongly typed ID class (e.g., `frame_id_t`).
/// @tparam T Underlying integer type (e.g., `uint32_t`).
template <class derived_id_t, typename T>
class templated_id_t {
public:
  /// Sentinel value used to represent an unset identifier.
  /// It marks IDs that have not been assigned yet.
  static constexpr T UNSET = std::numeric_limits<T>::max();

  /// Default constructor initializing to UNSET
  templated_id_t() : _key(UNSET) {}

  /// Copy constructor.
  templated_id_t(const templated_id_t&) = default;
  /// Move constructor.
  templated_id_t(templated_id_t&&) noexcept = default;
  /// Copy assignment.
  templated_id_t& operator=(const templated_id_t&) = default;
  /// Move assignment.
  templated_id_t& operator=(templated_id_t&&) noexcept = default;

  /// Constructor from a given ID value
  /// @param id_ Raw numeric identifier stored by this wrapper.
  templated_id_t(T id_) : _key(id_) {}

  /// Get the underlying ID value
  /// @return Raw numeric value represented by this typed ID.
  [[nodiscard("Use to retrieve the raw ID value.")]]
  inline T key() const {
    return _key;
  }

  /// Function call operator returns the key value
  /// @return Raw numeric value represented by this typed ID.
  [[deprecated("Use key() method instead of operator().")]]
  inline T operator()() const {
    return _key;
  }

  /// Arrow operator for pointer-like behavior (returns derived_id_t*)
  derived_id_t* operator->() { return static_cast<derived_id_t*>(this); }
  const derived_id_t* operator->() const {
    return static_cast<const derived_id_t*>(this);
  }

  /// Assignment from nullptr sets to UNSET
  /// @param nullptr_ Null token used to reset this ID to `UNSET`.
  /// @return Reference to this ID after reset.
  inline derived_id_t& operator=(std::nullptr_t nullptr_) {
    (void)nullptr_;
    _key = UNSET;
    return derived();
  }

  /// Simple add operator (derived_id_t + raw)
  /// @param v_ Raw numeric offset added to this ID.
  /// @return New typed ID with value `key() + v_`.
  inline derived_id_t operator+(T v_) const {
    return derived_id_t(static_cast<T>(_key + v_));
  }

  /// Simple subtract operator (derived_id_t - raw)
  /// @param v_ Raw numeric offset subtracted from this ID.
  /// @return New typed ID with value `key() - v_`.
  inline derived_id_t operator-(T v_) const {
    return derived_id_t(static_cast<T>(_key - v_));
  }

  /// Simple add operator (derived_id_t + derived_id_t)
  /// @param other_ Typed ID whose raw value is added to this ID.
  /// @return New typed ID with value `key() + other_.key()`.
  inline derived_id_t operator+(const derived_id_t& other_) const {
    return derived_id_t(static_cast<T>(_key + other_.key()));
  }

  /// Simple subtract operator (derived_id_t - derived_id_t)
  /// @param other_ Typed ID whose raw value is subtracted from this ID.
  /// @return New typed ID with value `key() - other_.key()`.
  inline derived_id_t operator-(const derived_id_t& other_) const {
    return derived_id_t(static_cast<T>(_key - other_.key()));
  }

  /// Prefix increment operator
  /// @return This ID after incrementing its raw value.
  inline derived_id_t operator++() {
    ++_key;
    return derived();
  }

  /// Postfix increment operator
  /// @param int_ Dummy integer used to select postfix increment syntax.
  /// @return Copy of the ID value before incrementing.
  inline derived_id_t operator++(int int_) {
    (void)int_;
    derived_id_t oldValue(derived());
    ++_key;
    return oldValue;
  }

  /// Equality comparison
  /// @param other_ Typed ID compared against this instance.
  /// @return `true` when both IDs store the same raw value.
  inline bool operator==(const derived_id_t& other_) const {
    return _key == other_.key();
  }

  /// Equality comparison with raw ID value
  /// @param other_ Raw numeric value compared against this ID.
  /// @return `true` when `key()` equals `other_`.
  inline bool operator==(const T other_) const { return _key == other_; }

  /// Inequality comparison
  /// @param other_ Typed ID compared against this instance.
  /// @return `true` when raw values differ.
  inline bool operator!=(const derived_id_t& other_) const {
    return _key != other_.key();
  }

  /// Inequality comparison with raw ID value
  /// @param other_ Raw numeric value compared against this ID.
  /// @return `true` when `key()` differs from `other_`.
  inline bool operator!=(const T other_) const { return _key != other_; }

  /// Less-than comparison
  /// @param other_ Typed ID compared against this instance.
  /// @return `true` when this ID raw value is smaller.
  inline bool operator<(const derived_id_t& other_) const {
    return _key < other_.key();
  }

  /// Less-than comparison with raw ID value
  /// @param other_ Raw numeric value compared against this ID.
  /// @return `true` when `key()` is smaller than `other_`.
  inline bool operator<(const T other_) const { return _key < other_; }

  /// Greater-than comparison
  /// @param other_ Typed ID compared against this instance.
  /// @return `true` when this ID raw value is greater.
  inline bool operator>(const derived_id_t& other_) const {
    return _key > other_.key();
  }

  /// Greater-than comparison with raw ID value
  /// @param other_ Raw numeric value compared against this ID.
  /// @return `true` when `key()` is greater than `other_`.
  inline bool operator>(const T other_) const { return _key > other_; }

  /// Less-than-or-equal comparison
  /// @param other_ Typed ID compared against this instance.
  /// @return `true` when this ID raw value is less or equal.
  inline bool operator<=(const derived_id_t& other_) const {
    return _key <= other_.key();
  }

  /// Less-than-or-equal comparison with raw ID value
  /// @param other_ Raw numeric value compared against this ID.
  /// @return `true` when `key()` is less than or equal to `other_`.
  inline bool operator<=(const T other_) const { return _key <= other_; }

  /// Greater-than-or-equal comparison
  /// @param other_ Typed ID compared against this instance.
  /// @return `true` when this ID raw value is greater or equal.
  inline bool operator>=(const derived_id_t& other_) const {
    return _key >= other_.key();
  }

  /// Greater-than-or-equal comparison with raw ID value
  /// @param other_ Raw numeric value compared against this ID.
  /// @return `true` when `key()` is greater than or equal to `other_`.
  inline bool operator>=(const T other_) const { return _key >= other_; }

  /// to string
  /// @return `"UNSET"` for unset IDs, otherwise the numeric ID as text.
  [[nodiscard("Use to get string representation of the ID.")]]
  inline std::string to_string() const {
    if (_key == UNSET) return "UNSET";
    return std::to_string(_key);
  }

  /// Print function
  /// @param prefix_ Prefix prepended to the printed ID line.
  inline void print(std::string prefix_ = "") const {
    using std::println;
    println("{}id: {}", prefix_, this->to_string());
  }

  /// Conversion operator to std::string
  /// @return String representation of this typed ID.
  inline operator std::string() const { return this->to_string(); }

  /// Stream operator (prints derived_id_t)
  /// @param os_ Output stream receiving the formatted ID.
  /// @param id_ ID value written to `os_`.
  /// @return Reference to `os_` for stream chaining.
  friend std::ostream& operator<<(std::ostream& os_, const derived_id_t& id_) {
    os_ << "key: " << id_.to_string();
    return os_;
  }

protected:
  /// @brief Returns this object cast to the CRTP-derived type.
  /// @return Reference to this object cast as `derived_id_t`.
  inline derived_id_t& derived() { return static_cast<derived_id_t&>(*this); }
  /// @brief Returns this object cast to the const CRTP-derived type.
  /// @return Const reference to this object cast as `derived_id_t`.
  inline const derived_id_t& derived() const {
    return static_cast<const derived_id_t&>(*this);
  }

private:
  T _key;  //< The underlying ID value
};

// ---------------------------------------------------------------------------
// Concrete ID types

/// Type for point IDs
class point_id_t : public templated_id_t<point_id_t, uint32_t> {
public:
  using base_t = templated_id_t<point_id_t, uint32_t>;
  using base_t::base_t;     // inherit constructors
  using base_t::operator=;  // expose base operator= (e.g. nullptr)

  using pair = pair_t<point_id_t, point_id_t>;
  using set = set_t<point_id_t>;
  using ordered = ordered_t<point_id_t>;
  using vector = vector_t<point_id_t>;
};

/// Type for frame IDs
class frame_id_t : public templated_id_t<frame_id_t, uint32_t> {
public:
  using base_t = templated_id_t<frame_id_t, uint32_t>;
  using base_t::base_t;
  using base_t::operator=;

  using pair = pair_t<frame_id_t, frame_id_t>;
  using set = set_t<frame_id_t>;
  using ordered = ordered_t<frame_id_t>;
  using vector = vector_t<frame_id_t>;
};

/// Type for cell IDs
class cell_id_t : public templated_id_t<cell_id_t, uint32_t> {
public:
  using base_t = templated_id_t<cell_id_t, uint32_t>;
  using base_t::base_t;
  using base_t::operator=;

  using pair = pair_t<cell_id_t, cell_id_t>;
  using set = set_t<cell_id_t>;
  using ordered = ordered_t<cell_id_t>;
  using vector = vector_t<cell_id_t>;
};

/// Type for keyframe IDs
class keyframe_id_t : public templated_id_t<keyframe_id_t, uint16_t> {
public:
  using base_t = templated_id_t<keyframe_id_t, uint16_t>;
  using base_t::base_t;
  using base_t::operator=;

  using pair = pair_t<keyframe_id_t, keyframe_id_t>;
  using set = set_t<keyframe_id_t>;
  using ordered = ordered_t<keyframe_id_t>;
  using vector = vector_t<keyframe_id_t>;
  using queue = queue_t<keyframe_id_t>;

  /// Keyframe state used when pose estimation fails.
  /// This value flags keyframes whose pose estimation step failed.
  static constexpr uint16_t FAIL_POSE_ESTIMATION =
      std::numeric_limits<uint16_t>::max() - 1;
  /// Keyframe state used when creation fails.
  /// This value flags keyframes that failed to be created.
  static constexpr uint16_t FAIL_TO_CREATE =
      std::numeric_limits<uint16_t>::max() - 2;
};

/// Type for generic indices
class index_t : public templated_id_t<index_t, uint32_t> {
public:
  using base_t = templated_id_t<index_t, uint32_t>;
  using base_t::base_t;
  using base_t::operator=;

  using set = set_t<index_t>;
  using ordered = ordered_t<index_t>;
  using vector = vector_t<index_t>;

  /// Converts this index to `size_t`.
  /// @return Raw index value converted to `size_t`.
  explicit operator size_t() const { return static_cast<size_t>(this->key()); }
};

}  // namespace slam_mer

// ---------------------------------------------------------------------------
// Hash functions

namespace std {

template <>
struct hash<slam_mer::point_id_t> {
  /// @brief Hashes a point identifier.
  /// @param pointId_ Point identifier used as hash input.
  /// @return Hash value of the underlying point ID key.
  std::size_t operator()(const slam_mer::point_id_t& pointId_) const noexcept {
    return std::hash<uint32_t>{}(pointId_.key());
  }
};

template <>
struct hash<slam_mer::cell_id_t> {
  /// @brief Hashes a cell identifier.
  /// @param cellId_ Cell identifier used as hash input.
  /// @return Hash value of the underlying cell ID key.
  std::size_t operator()(const slam_mer::cell_id_t& cellId_) const noexcept {
    return std::hash<uint32_t>{}(cellId_.key());
  }
};

template <>
struct hash<slam_mer::keyframe_id_t> {
  /// @brief Hashes a keyframe identifier.
  /// @param keyframeId_ Keyframe identifier used as hash input.
  /// @return Hash value of the underlying keyframe ID key.
  std::size_t operator()(
      const slam_mer::keyframe_id_t& keyframeId_) const noexcept {
    return std::hash<uint16_t>{}(keyframeId_.key());
  }
};

template <>
struct hash<slam_mer::keyframe_id_t::pair> {
  /// @brief Hashes a pair of keyframe identifiers.
  /// @param p_ Pair of keyframe identifiers used as hash input.
  /// @return Combined hash value for the two keyframe IDs.
  std::size_t operator()(
      const slam_mer::keyframe_id_t::pair& p_) const noexcept {
    std::size_t h1 = std::hash<slam_mer::keyframe_id_t>{}(p_.first);
    std::size_t h2 = std::hash<slam_mer::keyframe_id_t>{}(p_.second);
    // A simple but decent hash combine (same idea as boost::hash_combine)
    h2 ^= 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2);
    return h1 ^ h2;
  }
};

template <>
struct hash<slam_mer::frame_id_t> {
  /// @brief Hashes a frame identifier.
  /// @param frameId_ Frame identifier used as hash input.
  /// @return Hash value of the underlying frame ID key.
  std::size_t operator()(const slam_mer::frame_id_t& frameId_) const noexcept {
    return std::hash<uint32_t>{}(frameId_.key());
  }
};

}  // namespace std

#endif  // TYPES_ID_HPP
