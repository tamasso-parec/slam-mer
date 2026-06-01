/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file timer.hpp
/// @brief Duration and wall-clock timer utilities for pacing and profiling.
/// @details Provides duration_t conversions, elapsed-time measurement, wait
/// helpers, and formatted timing output. These utilities support
/// diagnostics, scheduling, and runtime control loops.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <cmath>
#include <memory>
#include <print>
#include <string>
#include <thread>

namespace slam_mer {

// Avoids typing std::chrono everywhere
// Be careful with namespace pollution
using namespace std::chrono;

/// hz_t is used to represent frequency in Hertz (Hz) from timer_t
class duration_t {
public:
  /// Static conversion function from raw microseconds.
  /// @param us_ Duration value already expressed in microseconds.
  /// @return Input value unchanged, interpreted as microseconds.
  static uint64_t from_int64(uint64_t us_) { return us_; }
  /// Static conversion functions: from hz
  /// @param hz_ Frequency in Hertz to convert into period duration.
  /// @return Equivalent period in microseconds for frequency `hz_`.
  static uint64_t from_hz(double hz_) { return 1000000.0 / hz_; }
  /// Static conversion functions: from milliseconds
  /// @param ms_ Duration value in milliseconds.
  /// @return Equivalent duration in microseconds.
  static uint64_t from_ms(double ms_) { return 1000.0 * ms_; }
  /// Static conversion functions: from microseconds
  /// @param us_ Duration value in microseconds.
  /// @return Input value unchanged.
  static uint64_t from_us(uint64_t us_) { return us_; }
  /// Static conversion functions: from frames per second
  /// @param fps_ Frame rate in frames per second.
  /// @return Equivalent frame period in microseconds.
  static uint64_t from_fps(double fps_) { return from_hz(fps_); }
  /// Static conversion functions: from seconds
  /// @param seconds_ Duration value in seconds.
  /// @return Equivalent duration in microseconds.
  static uint64_t from_s(double seconds_) { return 1000000.0 * seconds_; }
  /// Constant representing an unset duration
  /// It can be used as a sentinel for invalid duration values.
  static constexpr uint64_t UNSET = std::numeric_limits<uint64_t>::max();

  // ------------------------------------------------------------------------
  /// Default Constructor
  duration_t() : microseconds(0) {}
  /// Constructor
  /// @param microseconds_ Stored duration value in microseconds.
  duration_t(double microseconds_) : microseconds(microseconds_) {}
  /// Default destructor
  ~duration_t() = default;
  // ------------------------------------------------------------------------
  // Methods
  /// Convert to frequency in Hz
  /// @return Frequency corresponding to this period, in Hertz.
  inline double hz() const { return 1000000.0 / this->microseconds; }
  /// convert to milliseconds
  /// @return Duration value in milliseconds.
  inline double ms() const { return this->microseconds / 1000.0; }
  /// convert to microseconds
  /// @return Duration value in microseconds.
  inline uint64_t us() const { return this->microseconds; }
  /// convert to frames per second
  /// @return Frame rate equivalent of this period, in FPS.
  inline double fps() const { return this->hz(); }
  /// convert to seconds
  /// @return Duration value in seconds.
  inline double s() const { return this->microseconds / 1000000.0; }
  /// convert to int64_t microseconds
  /// @return Duration in microseconds rounded to `int64_t`.
  inline int64_t int64() const {
    return static_cast<int64_t>(std::round(this->microseconds));
  }

  // ------------------------------------------------------------------------
  // Operators
  /// Implicit conversion to double (microseconds)
  /// @return Duration value in microseconds.
  explicit operator uint64_t() const noexcept { return this->microseconds; }
  /// Operator ==
  /// @param other_ Duration to compare against this object.
  /// @return `true` when both durations have the same microsecond value.
  inline bool operator==(const duration_t& other_) const {
    return this->microseconds == other_.microseconds;
  }
  /// Equality operator with raw microseconds.
  /// @param other_ Raw duration in microseconds for comparison.
  /// @return `true` when `other_` equals this duration value.
  inline bool operator==(const uint64_t other_) const {
    return this->microseconds == other_;
  }
  /// Greater than operator
  /// @param other_ Duration to compare against this object.
  /// @return `true` when this duration is longer than `other_`.
  inline bool operator>(const duration_t& other_) const {
    return this->microseconds > other_.microseconds;
  }
  /// Less than operator
  /// @param other_ Duration to compare against this object.
  /// @return `true` when this duration is shorter than `other_`.
  inline bool operator<(const duration_t& other_) const {
    return this->microseconds < other_.microseconds;
  }
  /// To string
  /// @return Human-readable duration string in microseconds.
  explicit operator std::string() const {
    return std::to_string(static_cast<long long>(microseconds)) + " us";
  }
  /// Operator<<
  /// @param os_ Output stream receiving the formatted duration.
  /// @param dt_ Duration value to print.
  /// @return Reference to `os_` for stream chaining.
  friend std::ostream& operator<<(std::ostream& os_, const duration_t& dt_) {
    os_ << std::string(dt_);
    return os_;
  }

private:
  /// Stored duration in microseconds.
  uint64_t microseconds;
};

/// @brief Utility timer for elapsed-time measurement and pacing.
///
/// The timer measures elapsed time between `tic()` and `toc()` and provides
/// sleep/pacing helpers for loop-rate control.
class timer_t {

public:
  /// Smart pointers -- unique
  using unique_ptr = std::unique_ptr<timer_t>;
  /// Smart pointers -- shared
  using shared_ptr = std::shared_ptr<timer_t>;
  /// Make unique pointer function
  /// @return Unique pointer owning a new timer instance.
  static timer_t::unique_ptr make_unique() {
    return std::make_unique<timer_t>();
  }
  /// Make shared pointer function
  /// @return Shared pointer owning a new timer instance.
  static timer_t::shared_ptr make_shared() {
    return std::make_shared<timer_t>();
  }

  /// Default constructor. This function starts the timer.
  timer_t() { this->tic(); };
  /// Default destructor.
  ~timer_t() = default;

  /// Set/Reset the timer for measuring elapsed time.
  inline void tic() { this->startTime = high_resolution_clock::now(); }

  /// Stops the timer and returns the elapsed time.
  /// @return Elapsed duration since the last `tic()`.
  inline duration_t toc() { return __toc(); }

  /// Checks if the timer is running for more than the specified duration
  /// @param minDuration_ Duration threshold used for the comparison.
  /// @return `true` when elapsed time is greater than `minDuration_`.
  inline bool over(duration_t minDuration_) {
    duration_t duration = __toc();
    return duration > minDuration_;
  }

  /// Either do nothing or sleep to pace the execution to a minimum FPS.
  /// @param minDuration_ Minimum cycle duration to enforce.
  /// @return Elapsed duration since the last `tic()` after pacing.
  inline duration_t pace(duration_t minDuration_) {
    __sleep(__elapsed_time(minDuration_));
    return __toc();
  }

  /// Waits for a specified duration.
  /// @param duration_ Sleep duration requested for the current thread.
  inline void sleep(duration_t duration_) { __sleep(duration_); }

  /// Prints elapsed time since the last `tic()`.
  /// @param prefix_ Prefix prepended to the printed timer message.
  void print(std::string prefix_ = "") {
    using std::println;
    println("{}This time: {} have passed.", prefix_, std::string(__toc()));
  }

private:
  /// Start time
  time_point<high_resolution_clock> startTime;

  // ------------------------------------------------------------
  // private methods

  /// Computes the elapsed time since the last tic() call (0 if negative)
  /// @param offset_ Target duration used to compute remaining wait time.
  /// @return Remaining duration until `offset_`, clamped to zero.
  inline duration_t __elapsed_time(duration_t offset_) {
    time_point<high_resolution_clock> now = high_resolution_clock::now();
    int64_t duration =
        duration_cast<microseconds>(now - this->startTime).count();
    int64_t delta = offset_.int64() - duration;
    if (delta < 0) delta = 0;
    return duration_t::from_int64(delta);
  }

  /// Computes the elapsed time since the last tic() call
  /// @return Elapsed duration in microseconds since `tic()`.
  inline duration_t __toc() {
    time_point<high_resolution_clock> now = high_resolution_clock::now();
    int64_t duration =
        duration_cast<microseconds>(now - this->startTime).count();
    return duration_t::from_int64(duration);
  }

  /// Sleeps a thread for a specified duration
  /// @param duration_ Duration used in `std::this_thread::sleep_for`.
  inline void __sleep(duration_t duration_) {
    std::this_thread::sleep_for(microseconds(duration_.int64()));
  }
};

}  // namespace slam_mer

#endif  // TIMER_HPP
