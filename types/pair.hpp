/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file pair.hpp
/// @brief Thin std::pair extension with convenience aliases and streaming.
/// @details Wraps std::pair to expose project-level naming consistency and
/// utility typedefs. It is used as a simple building block for typed
/// associations in containers and APIs.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef PAIR_HPP
#define PAIR_HPP

// Standard libraries
#include <iostream>
#include <utility>

namespace slam_mer {

/// @brief Simple pair class extending `std::pair`.
/// @tparam type1 Type of the first element.
/// @tparam type2 Type of the second element.
template <typename type1, typename type2>
class pair_t : public std::pair<type1, type2> {
public:
  /// @brief Inherits constructors from `std::pair`.
  using std::pair<type1, type2>::pair;

  /// @brief Default constructor.
  pair_t() : std::pair<type1, type2>() {}

  /// @brief Constructs a pair from two elements.
  /// @param first_ Value copied into the pair first component.
  /// @param second_ Value copied into the pair second component.
  pair_t(const type1& first_, const type2& second_)
      : std::pair<type1, type2>(first_, second_) {}

  /// @brief Constructs from a `std::pair`.
  /// @param p_ Source `std::pair` copied into this wrapper.
  pair_t(const std::pair<type1, type2>& p_)
      : std::pair<type1, type2>(p_.first, p_.second) {}

  /// @brief Stream insertion for formatted printing.
  /// @param os_ Output stream receiving the formatted pair.
  /// @param p_ Pair value written to `os_`.
  /// @return Reference to `os_` for stream chaining.
  friend std::ostream& operator<<(std::ostream& os_, const pair_t& p_) {
    os_ << "(" << p_.first << ", " << p_.second << ")";
    return os_;
  }
};

}  // namespace slam_mer

#endif  // PAIR_HPP
