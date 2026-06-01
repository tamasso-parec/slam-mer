/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file verbose.hpp
/// @brief Verbosity level enumeration for logging and diagnostics.
/// @details Declares common verbosity categories used to gate console output
/// and debug information across modules. This keeps logging behavior
/// consistent in the full pipeline.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_VERBOSE_HPP
#define TYPE_VERBOSE_HPP

namespace slam_mer {

/// @brief Verbosity levels used to gate runtime logging output.
enum VerboseLevels {
  /// Log only high-level informational messages.
  INFO = 0,
  /// Log error conditions requiring attention.
  ERROR = 1,
  /// Log warning conditions and recoverable issues.
  WARNING = 2,
  /// Log detailed debug diagnostics.
  DEBUG = 3
};

}  // namespace slam_mer

#endif  // TYPE_VERBOSE_HPP
