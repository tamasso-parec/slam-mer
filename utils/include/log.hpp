/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file log.hpp
/// @brief Logging interfaces, message categories, and terminal formatting.
/// @details Declares the public logging API, verbosity categories, and helpers
/// used to emit standardized messages to terminal or file backends.
/// This header centralizes log-level semantics across modules.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef UTILS_INCLUDE_LOG_HPP_
#define UTILS_INCLUDE_LOG_HPP_

#include <string>

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

namespace slam_mer {
namespace log {

/// @brief Module labels used to prefix log messages by source component.
/// `EMPTY`: generic messages without a specific module source.
/// `MAIN`: messages emitted by the main executable flow.
/// `SLAM`: messages from the top-level SLAM pipeline.
/// `MAP`: messages from map-management routines.
/// `TRACKING`: messages from localization/tracking routines.
/// `MAPPING`: messages from local mapping routines.
/// `LOOP_CLOSURE`: messages from loop-closure routines.
/// `IMAGE_PROCESSING`: messages from image-processing routines.
/// `KEYFRAME_DATABASE`: messages from keyframe-database routines.
/// `RANSAC`: messages from robust-estimation (RANSAC) routines.
/// `VISUALIZATION`: messages from visualization routines.
/// `DATALOADER`: messages from data-loading routines.
/// `settings_global`: legacy settings-loader messages.
/// `SETTINGS`: messages from the settings module.
enum Module {
  /// Generic message source.
  EMPTY,
  /// Main module source.
  MAIN,
  /// SLAM pipeline source.
  SLAM,
  /// Map module source.
  MAP,
  /// Tracking module source.
  TRACKING,
  /// Mapping module source.
  MAPPING,
  /// Loop-closure module source.
  LOOP_CLOSURE,
  /// Image-processing module source.
  IMAGE_PROCESSING,
  /// Keyframe-database module source.
  KEYFRAME_DATABASE,
  /// RANSAC module source.
  RANSAC,
  /// Visualization module source.
  VISUALIZATION,
  /// Data-loader module source.
  DATALOADER,
  /// Legacy settings module source.
  settings_global,
  /// Settings module source.
  SETTINGS
};

/// @brief This enum variable is used to define the set of different
/// type of messages to be displayed. It can be either a general message,
/// warning or error, depending on the context.
/// GENERAL: for normal verbose messages
/// WARNING: for important messages to the user
/// ERROR: critical information stopping the process
enum TypeMessage {
  /// Informational message severity.
  GENERAL,
  /// Warning message severity.
  WARNING,
  /// Error message severity.
  ERROR
};

/// @brief General log message.
/// @param str_ Message text sent to the configured logging backend.
/// @param type_ Message severity used to route/format `str_`.
/// @param module_ Source module label prepended to the message.
void message(
    const std::string& str_,
    TypeMessage type_,
    Module module_ = Module::EMPTY);

/// @brief General log message.
/// @param str_ Informational message text sent to the logging backend.
/// @param module_ Source module label prepended to the message.
void message(const std::string& str_, Module module_ = Module::EMPTY);

/// @brief Warning message.
/// @param str_ Warning message text sent to the logging backend.
/// @param module_ Source module label prepended to the message.
void warning(const std::string& str_, Module module_ = Module::EMPTY);

/// @brief Error message.
/// @param str_ Error message text sent to the logging backend.
/// @param module_ Source module label prepended to the message.
void error(const std::string& str_, Module module_ = Module::EMPTY);

}  // namespace log
}  // namespace slam_mer

#endif  // UTILS_INCLUDE_LOG_HPP_
