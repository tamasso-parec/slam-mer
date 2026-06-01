/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file fail.hpp
/// @brief Failure code and message structures for structured error reporting.
/// @details Defines the fail_t type, failure categories, string conversion
/// helpers, and terminal color constants used by output wrappers. This
/// centralizes error semantics across modules.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef FAIL_HPP
#define FAIL_HPP

#include <cstdint>
#include <print>
#include <string>
#include <string_view>
#include <utility>

namespace slam_mer {

/// @brief ANSI color code for yellow terminal output.
static constexpr const char* yellow = "\033[33m";
/// @brief ANSI color code for red terminal output.
static constexpr const char* red = "\033[31m";
/// @brief ANSI code to reset terminal color formatting.
static constexpr const char* reset = "\033[0m";
/// @brief ANSI color code for green terminal output
static constexpr const char* green = "\033[32m";

/// This class represents a failure with a type and an optional message.
class fail_t {
public:
  // Failure types
  /// No failure.
  static const fail_t NO_FAIL;
  /// Unknown failure.
  static const fail_t UNKNOWN;
  /// Operation timed out.
  static const fail_t TIMEOUT;
  /// Memory allocation failed.
  static const fail_t OUT_OF_MEMORY;
  /// Input arguments are invalid.
  static const fail_t INVALID_INPUT;
  /// Runtime error occurred.
  static const fail_t RUNTIME_ERROR;
  /// Requested element was not found.
  static const fail_t NOT_FOUND;
  /// Element already exists.
  static const fail_t ALREADY_EXISTS;
  /// Container or resource is empty.
  static const fail_t IS_EMPTY;
  /// Container or resource is full.
  static const fail_t IS_FULL;
  /// Operation is not permitted.
  static const fail_t PERMISSION_DENIED;
  /// Referenced pointer was lost or invalid.
  static const fail_t LOST_PTR;
  /// Cell reached its capacity.
  static const fail_t CELL_FULL;
  /// Generic failure state.
  static const fail_t FAIL;
  /// Element insertion failed.
  static const fail_t INSERTATION_FAILED;
  /// Index or coordinate is outside valid bounds.
  static const fail_t OUT_OF_BOUNDS;

  /// @brief Converts a failure code to its symbolic name.
  /// @param t_ Numeric failure type code.
  /// @return String view with the corresponding failure name.
  [[nodiscard("Use to convert fail type to string.")]]
  static constexpr std::string_view type_name(const uint8_t t_) noexcept {
    switch (t_) {
    case 0:
      return "NO_FAIL";
    case 1:
      return "UNKNOWN";
    case 2:
      return "TIMEOUT";
    case 3:
      return "OUT_OF_MEMORY";
    case 4:
      return "INVALID_INPUT";
    case 5:
      return "RUNTIME_ERROR";
    case 6:
      return "NOT_FOUND";
    case 7:
      return "ALREADY_EXISTS";
    case 8:
      return "IS_EMPTY";
    case 9:
      return "IS_FULL";
    case 10:
      return "PERMISSION_DENIED";
    case 11:
      return "LOST_PTR";
    case 12:
      return "CELL_FULL";
    case 13:
      return "FAIL";
    case 14:
      return "INSERTATION_FAILED";
    case 15:
      return "OUT_OF_BOUNDS";
    default:
      return "UNRECOGNIZED_FAIL_TYPE";
    }
  }

  // Constructors
  /// Default constructor
  constexpr fail_t() : _type(0), _message("") {}

  /// Construct from type and message
  /// @param type_ Numeric failure type code.
  /// @param message_ Human-readable description of the failure.
  fail_t(const uint8_t type_, const std::string& message_)
      : _type(type_), _message(message_) {}

  /// Construct from fail type and message
  /// @param fail_ Failure template whose type is reused.
  /// @param message_ Human-readable description replacing the template message.
  fail_t(const fail_t& fail_, const std::string& message_)
      : _type(fail_._type), _message(message_) {}

  /// Construct from type only
  /// @param type_ Numeric failure type code.
  constexpr fail_t(const uint8_t type_) : _type(type_), _message("") {}

  /// Construct from {uint8_t, string} pair
  /// @param pair_ Pair containing `{type_code, message}`.
  fail_t(const std::pair<uint8_t, std::string>& pair_)
      : _type(pair_.first), _message(pair_.second) {}

  /// Construct from {fail_t, string} pair
  /// @param pair_ Pair containing `{fail_template, message}`.
  fail_t(const std::pair<fail_t, std::string>& pair_)
      : _type(pair_.first._type), _message(pair_.second) {}

  /// Copy assignment operator
  /// @param other_ Failure object copied into this instance.
  /// @return Reference to this failure object.
  fail_t& operator=(const fail_t& other_) {
    _type = other_._type;
    _message = other_._message;
    return *this;
  }

  /// Assignment operator from uint8_t
  /// @param t_ New numeric failure type code.
  /// @return Reference to this failure object.
  fail_t& operator=(uint8_t t_) {
    _type = t_;
    _message.clear();
    return *this;
  }

  /// Comparison operators
  /// @param other_ Failure object compared against this instance.
  /// @return `true` when both failure objects have the same type.
  bool operator==(const fail_t& other_) const { return _type == other_._type; }

  /// Comparison operators
  /// @param t_ Numeric failure type compared against this instance.
  /// @return `true` when this failure type equals `t_`.
  bool operator==(const uint8_t& t_) const { return _type == t_; }

  /// Comparison operators
  /// @param type_ Numeric failure type compared against this instance.
  /// @return `true` when this failure type differs from `type_`.
  bool operator!=(const uint8_t& type_) const { return _type != type_; }

  /// Comparison operator
  /// @param other_ Failure object compared against this instance.
  /// @return `true` when failure types differ.
  bool operator!=(const fail_t& other_) const { return _type != other_._type; }

  /// Implicit conversion to bool
  /// @return `true` when this object represents a failure.
  explicit operator bool() const noexcept { return this->fail(); }

  /// Implicit conversion to bool
  /// @return `true` when this object represents no failure.
  bool operator!() const noexcept { return !this->fail(); }

  /// Implicit conversion to string for printing
  /// @return String representation with type name and message.
  operator std::string() const {
    if (!this->fail()) return "NO_FAIL";
    return std::string(type_name(_type)) + ": " + _message;
  }

  // Methods

  /// Check if it is a fail
  /// @return `true` when the failure type is different from `NO_FAIL`.
  [[nodiscard("Use to check if this is a fail.")]]
  inline bool fail() const {
    return _type != 0;
  }

  /// Get the message
  /// @return Failure message text associated with this error type.
  [[nodiscard("Use to get the fail message.")]]
  inline std::string get_message() const {
    return _message;
  }

  /// Print the fail
  void print() const {
    using std::println;
    const std::string_view& typeName = type_name(_type);
    const int& typeInt = static_cast<int>(_type);
    println("Fail Type: {} ({})", typeName, typeInt);
    println("  := Message: {}", _message);
  }

private:
  /// Failure type
  uint8_t _type;
  /// Failure message
  std::string _message;
};

inline constexpr fail_t fail_t::NO_FAIL{0};
inline constexpr fail_t fail_t::UNKNOWN{1};
inline constexpr fail_t fail_t::TIMEOUT{2};
inline constexpr fail_t fail_t::OUT_OF_MEMORY{3};
inline constexpr fail_t fail_t::INVALID_INPUT{4};
inline constexpr fail_t fail_t::RUNTIME_ERROR{5};
inline constexpr fail_t fail_t::NOT_FOUND{6};
inline constexpr fail_t fail_t::ALREADY_EXISTS{7};
inline constexpr fail_t fail_t::IS_EMPTY{8};
inline constexpr fail_t fail_t::IS_FULL{9};
inline constexpr fail_t fail_t::PERMISSION_DENIED{10};
inline constexpr fail_t fail_t::LOST_PTR{11};
inline constexpr fail_t fail_t::CELL_FULL{12};
inline constexpr fail_t fail_t::FAIL{13};
inline constexpr fail_t fail_t::INSERTATION_FAILED{14};
inline constexpr fail_t fail_t::OUT_OF_BOUNDS{15};

}  // namespace slam_mer

#endif  // FAIL_HPP
