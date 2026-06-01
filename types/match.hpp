/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file match.hpp
/// @brief Generic typed match container for pairwise associations.
/// @details Defines a small utility class for storing matched elements and
/// transporting them as vectors. It is used where correspondences
/// between features, points, or frames are required.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_MATCH_HPP
#define TYPE_MATCH_HPP

#include <types/vector.hpp>

namespace slam_mer {

/// @brief This class represents a match between two types.
/// @note Future work: later on, we can have a confidence level associated
/// with a match.
template <typename type>
class match_t {

public:
  /// Typedef for vector of matches_t
  using vector = vector_t<match_t>;

  /// @brief Creates a new match_t object
  /// @param first_ First endpoint of the association.
  /// @param second_ Second endpoint matched with `first_`.
  match_t(type first_ = type(), type second_ = type())
      : _first(first_), _second(second_) {}

  /// @brief Getter for the first element of the match.
  /// @return First element of the match.
  [[nodiscard("Retrieving first element of the match.")]]
  inline type first_element() const {
    return _first;
  }

  /// @brief Getter for the second element of the match.
  /// @return Second element of the match.
  [[nodiscard("Retrieving second element of the match.")]]
  inline type second_element() const {
    return _second;
  }

private:
  /// First element of the match
  type _first;
  /// Second element of the match
  type _second;
};

}  // namespace slam_mer

#endif  // TYPE_MATCH_HPP
