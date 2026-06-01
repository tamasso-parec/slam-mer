/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file output.hpp
/// @brief Value-or-error wrapper and throw helpers for module interfaces.
/// @details Defines output_t as a lightweight transport for either valid values
/// or fail_t errors, with source-aware throw utilities. This pattern
/// standardizes error handling across retrieval and processing code
/// paths.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef OUTPUT_HPP
#define OUTPUT_HPP

#include <types/fail.hpp>

#include <cassert>
#include <memory>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace slam_mer {

/// @brief Throws a `std::runtime_error` with source context.
/// @param fail_ Failure object whose details are printed before throwing.
/// @param loc_ Source location captured at the call site.
[[noreturn]] inline void __throw__(
    const fail_t& fail_,
    std::source_location loc_ = std::source_location::current()) {
  std::string msg;
  msg.reserve(512);
  msg += "  |==> at ";
  msg += loc_.file_name();
  msg += ":";
  msg += std::to_string(loc_.line());
  msg += " in ";
  msg += loc_.function_name();

  if (fail_ != fail_t::NO_FAIL) fail_.print();
  throw std::runtime_error(msg);
}

/// @brief Throws a `std::runtime_error` unconditionally.
[[noreturn]] inline void __throw__() {
  __throw__(fail_t());
}

/// @brief Throws a `std::runtime_error` with a custom message and source
///   context.
/// @param message_ Custom error message prepended to source-location details.
/// @param loc_ Source location captured at the call site.
[[noreturn]] inline void __throw__(
    const std::string_view& message_,
    std::source_location loc_ = std::source_location::current()) {
  std::string msg = std::string(message_);
  msg += "\n";
  msg += "  |==> at ";
  msg += loc_.file_name();
  msg += ":";
  msg += std::to_string(loc_.line());
  msg += " in ";
  msg += loc_.function_name();
  throw std::runtime_error(msg);
}

/// @brief Generic output wrapper holding a value and an error state.
/// @tparam type Stored value type.
template <typename type>
class output_t {
public:
  // Constructors
  /// @brief Default constructor.
  output_t() : _error(), _value() {}
  /// @brief Constructs from value by copy.
  /// @param value_ Value copied into the output container.
  output_t(const type& value_) : _error(), _value(value_) {}
  /// @brief Constructs from value by move.
  /// @param value_ Value moved into the output container.
  output_t(type&& value_) : _error(), _value(std::move(value_)) {}
  /// @brief Constructs from value and error by copy.
  /// @param value_ Value copied into the output container.
  /// @param error_ Failure state copied into the output container.
  output_t(const type& value_, const fail_t& error_)
      : _error(error_), _value(value_) {}
  /// @brief Constructs from moved value and copied error.
  /// @param value_ Value moved into the output container.
  /// @param error_ Failure state copied into the output container.
  output_t(type&& value_, const fail_t& error_)
      : _error(error_), _value(std::move(value_)) {}
  /// @brief Constructs from copied value and moved error.
  /// @param value_ Value copied into the output container.
  /// @param error_ Failure state moved into the output container.
  output_t(const type& value_, fail_t&& error_)
      : _error(std::move(error_)), _value(value_) {}
  /// @brief Constructs from moved value and moved error.
  /// @param value_ Value moved into the output container.
  /// @param error_ Failure state moved into the output container.
  output_t(type&& value_, fail_t&& error_)
      : _error(std::move(error_)), _value(std::move(value_)) {}
  /// @brief Constructs from error only.
  /// @param error_ Failure state copied into the output container.
  output_t(const fail_t& error_) : _error(error_), _value(type()) {}
  /// @brief Constructs from moved error only.
  /// @param error_ Failure state moved into the output container.
  output_t(fail_t&& error_) : _error(std::move(error_)), _value(type()) {}

  // Methods
  /// @brief Returns the stored value.
  /// @return Mutable reference to the stored value.
  [[nodiscard("Retrieving stored value.")]]
  inline type& get() {
    return _value;
  }

  /// @brief Returns the stored value (const).
  /// @return Const reference to the stored value.
  [[nodiscard("Retrieving stored value.")]]
  inline const type& get() const {
    return _value;
  }

  /// @brief Returns the stored error.
  /// @return Mutable reference to the stored failure state.
  [[nodiscard("Retrieving stored error.")]]
  inline fail_t& get_error() {
    return _error;
  }

  /// @brief Returns the stored error (const).
  /// @return Const reference to the stored failure state.
  [[nodiscard("Retrieving stored error.")]]
  inline const fail_t& get_error() const {
    return _error;
  }

  /// @brief Returns whether this output represents a failure.
  /// @return `true` when the stored `fail_t` indicates an error.
  [[nodiscard("Checking if output is a failure.")]]
  inline bool fail() const {
    return _error.fail();
  }

  /// @brief Implicit conversion to the contained value (mutable lvalue).
  operator type&() & {
    __check_ok();
    return _value;
  }

  /// @brief Implicit conversion to the contained value (const lvalue).
  operator const type&() const& {
    __check_ok();
    return _value;
  }

  /// @brief Implicit conversion to the contained value (rvalue).
  operator type&&() && {
    __check_ok();
    return std::move(_value);
  }

  /// @brief Implicit conversion to the contained `fail_t`.
  /// @return Copy of the stored failure state.
  operator fail_t() const { return _error; }

  /// @brief Compares the stored error to a failure code.
  /// @param fail_ Failure code compared against the stored error.
  /// @return `true` when the stored failure equals `fail_`.
  bool operator==(const fail_t& fail_) const { return _error == fail_; }
  /// @brief Compares the stored error to a failure code.
  /// @param fail_ Failure code compared against the stored error.
  /// @return `true` when the stored failure differs from `fail_`.
  bool operator!=(const fail_t& fail_) const { return _error != fail_; }

  /// @brief Pointer-like access to the stored value.
  auto operator->() {
    __check_ok();
    if constexpr (std::is_pointer_v<type>)
      return _value;
    else
      return _value.operator->();  // e.g., shared_ptr::operator->()
  }

  /// @brief Pointer-like access to the stored value (const).
  auto operator->() const {
    __check_ok();
    if constexpr (std::is_pointer_v<type>)
      return _value;
    else
      return _value.operator->();
  }
  /// @brief Dereferences the stored pointer-like value (const).
  decltype(auto) operator*() const {
    __check_ok();
    return *(_value);
  }

  /// @brief Dereferences the stored pointer-like value.
  decltype(auto) operator*() {
    __check_ok();
    return *(_value);
  }

private:
  /// @brief Asserts that this output is not a failure.
  void __check_ok() const { assert(!this->fail()); }

  /// @brief Stored failure state.
  fail_t _error;
  /// @brief Stored value.
  type _value;
};

/// @brief Reference specialization of `output_t`.
/// @tparam type Referenced value type.
template <typename type>
class output_t<type&> {
public:
  // Constructors
  /// @brief Default constructor creating an invalid reference output.
  output_t() : _error(fail_t::LOST_PTR), _value(nullptr) {}
  /// @brief Constructs from referenced value.
  /// @param value_ Referenced object wrapped by this output container.
  output_t(type& value_) : _error(), _value(std::addressof(value_)) {}
  /// @brief Constructs from referenced value and copied error.
  /// @param value_ Referenced object wrapped by this output container.
  /// @param error_ Failure state copied into the output container.
  output_t(type& value_, const fail_t& error_)
      : _error(error_), _value(std::addressof(value_)) {}
  /// @brief Constructs from referenced value and moved error.
  /// @param value_ Referenced object wrapped by this output container.
  /// @param error_ Failure state moved into the output container.
  output_t(type& value_, fail_t&& error_)
      : _error(std::move(error_)), _value(std::addressof(value_)) {}
  /// @brief Constructs from error only.
  /// @param error_ Failure state copied into the output container.
  output_t(const fail_t& error_) : _error(error_), _value(nullptr) {}
  /// @brief Constructs from moved error only.
  /// @param error_ Failure state moved into the output container.
  output_t(fail_t&& error_) : _error(std::move(error_)), _value(nullptr) {}

  // Methods
  /// @brief Returns the referenced value.
  /// @return Mutable reference to the wrapped object.
  [[nodiscard("Retrieving referenced value.")]]
  inline type& get() {
    __check_ok();
    return *(_value);
  }

  /// @brief Returns the referenced value (const).
  /// @return Mutable reference to the wrapped object.
  [[nodiscard("Retrieving referenced value.")]]
  inline type& get() const {
    __check_ok();
    return *(_value);
  }

  /// @brief Returns the stored error.
  /// @return Mutable reference to the stored failure state.
  [[nodiscard("Retrieving stored error.")]]
  inline fail_t& get_error() {
    return _error;
  }

  /// @brief Returns the stored error (const).
  /// @return Const reference to the stored failure state.
  [[nodiscard("Retrieving stored error.")]]
  inline const fail_t& get_error() const {
    return _error;
  }

  /// @brief Returns whether this output represents a failure.
  /// @return `true` when the failure is set or the reference is null.
  [[nodiscard("Checking if output is a failure.")]]
  inline bool fail() const {
    return _error.fail() || _value == nullptr;
  }

  /// @brief Implicit conversion to the contained reference.
  operator type&() {
    __check_ok();
    return *(_value);
  }

  /// @brief Implicit conversion to the contained reference (const).
  operator type&() const {
    __check_ok();
    return *(_value);
  }

  /// @brief Implicit conversion to the contained `fail_t`.
  /// @return Copy of the stored failure state.
  operator fail_t() const { return _error; }

  /// @brief Pointer-like access to the stored reference.
  auto operator->() {
    __check_ok();
    return _value;
  }

  /// @brief Pointer-like access to the stored reference (const).
  auto operator->() const {
    __check_ok();
    return _value;
  }

  /// @brief Dereferences the stored reference.
  decltype(auto) operator*() {
    __check_ok();
    return *(_value);
  }

  /// @brief Dereferences the stored reference (const).
  decltype(auto) operator*() const {
    __check_ok();
    return *(_value);
  }

private:
  /// @brief Asserts that this output is not a failure.
  void __check_ok() const { assert(!this->fail()); }

  /// @brief Stored failure state.
  fail_t _error;
  /// @brief Pointer to referenced value.
  type* _value;
};

}  // namespace slam_mer

#endif  // OUTPUT_HPP
