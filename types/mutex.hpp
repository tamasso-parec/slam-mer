/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file typedef.hpp
/// @brief Centralized project-wide type aliases and shared utility typedefs.
/// @details Collects common aliases such as lock types and image matrix
/// placeholders to reduce duplication and maintain consistent naming
/// across the codebase. This header acts as a foundational dependency
/// for container wrappers.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_DEF_HPP
#define TYPE_DEF_HPP

// Add the necessary STD includes
#include <shared_mutex>

// We define the basic types used in the entire project here for

using mutex_t = std::shared_mutex;  ///< Mutex type

/// Write lock type
using write_lock_t = std::lock_guard<std::shared_mutex>;
/// Read lock type
using read_lock_t = std::shared_lock<std::shared_mutex>;

#endif  // TYPE_DEF_HPP
