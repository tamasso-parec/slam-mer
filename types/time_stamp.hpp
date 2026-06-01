/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file time_stamp.hpp
/// @brief Timestamp wrapper utilities for microsecond-resolution timing.
/// @details Defines a small time_stamp_t type with comparisons, conversion
/// helpers, and distance computation. It is used to keep temporal
/// metadata explicit and consistent across modules.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPES_TIME_HPP
#define TYPES_TIME_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

namespace slam_mer {

/// @brief A simple class to represent a time stamp. It is a wrapper around a
/// uint64_t value, which represents the time in microseconds since the epoch.
class time_stamp_t {
public:
  /// Typedef for a vector of time stamps
  using vector = std::vector<time_stamp_t>;
  /// A constant value representing an unset time stamp
  /// It is used as a sentinel for missing or invalid timestamps.
  static constexpr uint64_t UNSET = std::numeric_limits<uint64_t>::max();
  /// Default constructor, initializes the time stamp to the unset value
  time_stamp_t() : time(std::numeric_limits<uint64_t>::max()) {}
  /// Construct a new time_stamp_t object from a uint64_t value
  /// @param time_ Timestamp value in microseconds since epoch.
  time_stamp_t(const uint64_t& time_) : time(time_) {}
  /// Static function to compute the distance between two time stamps
  /// @param t1_ First timestamp in the distance computation.
  /// @param t2_ Second timestamp in the distance computation.
  /// @return Absolute difference between `t1_` and `t2_` in microseconds.
  static uint64_t distance(const time_stamp_t& t1_, const time_stamp_t& t2_) {
    return std::abs(
        static_cast<int64_t>(t1_.time) - static_cast<int64_t>(t2_.time));
  }
  /// Assignment operator from a uint64_t value
  /// @param other_ New timestamp value in microseconds.
  /// @return Updated timestamp object.
  inline time_stamp_t operator=(const uint64_t& other_) {
    this->time = other_;
    return *this;
  }
  /// Comparison operators
  /// @param other_ Timestamp to compare against this instance.
  /// @return `true` when both timestamps store the same value.
  inline bool operator==(const time_stamp_t& other_) const {
    return this->time == other_.time;
  }
  /// Comparison operators
  /// @param other_ Timestamp to compare against this instance.
  /// @return `true` when timestamp values differ.
  inline bool operator!=(const time_stamp_t& other_) const {
    return this->time != other_.time;
  }
  /// Comparison operators
  /// @param other_ Timestamp to compare against this instance.
  /// @return `true` when this timestamp is earlier than `other_`.
  inline bool operator<(const time_stamp_t& other_) const {
    return this->time < other_.time;
  }
  /// Comparison operators
  /// @param other_ Timestamp to compare against this instance.
  /// @return `true` when this timestamp is earlier than or equal to `other_`.
  inline bool operator<=(const time_stamp_t& other_) const {
    return this->time <= other_.time;
  }
  /// Comparison operators
  /// @param other_ Timestamp to compare against this instance.
  /// @return `true` when this timestamp is later than or equal to `other_`.
  inline bool operator>=(const time_stamp_t& other_) const {
    return this->time >= other_.time;
  }
  /// Comparison operators
  /// @param other_ Timestamp to compare against this instance.
  /// @return `true` when this timestamp is later than `other_`.
  inline bool operator>(const time_stamp_t& other_) const {
    return this->time > other_.time;
  }
  /// Output stream operator for time_stamp_t
  /// @param os_ Output stream receiving the formatted timestamp.
  /// @param t_ Timestamp instance that will be written to `os_`.
  /// @return Reference to `os_` for stream chaining.
  inline friend std::ostream& operator<<(
      std::ostream& os_,
      const time_stamp_t& t_) {
    os_ << t_.time;
    return os_;
  }
  /// Convert the time stamp to a string representation
  /// @return Decimal string representation of the timestamp value.
  std::string to_string() const { return std::to_string(this->time); }
  /// Get the time value of the time stamp
  /// @return Stored timestamp value in microseconds.
  inline uint64_t get() const { return this->time; }
  /// Check if the time stamp is unset (i.e., has the UNSET value)
  /// @return `true` when the timestamp equals `UNSET`, `false` otherwise.
  inline bool is_unset() const { return this->time == UNSET; }

private:
  /// The time value of the time stamp
  uint64_t time;
};

}  // namespace slam_mer

#endif  // TYPES_TIME_HPP
