/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file messages.cpp
/// @brief Implements text-message logging for the visualization frontend.
/// @details Defines queued message formatting and publishing to Rerun.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

// My libraries
#include <interface_rerun.hpp>
#include <types/queue.hpp>

// Rerun libraries
#include <rerun.hpp>

// Standard libraries

namespace slam_mer {
namespace interface {

namespace {
queue_t<pair_t<std::string, VerboseLevels>> __messages;
}  // namespace

/// External variable to store the cell data for the visualization interface
extern std::shared_ptr<rerun::RecordingStream> __recording;

/// @brief Map internal verbosity levels to Rerun text log levels.
/// @param level_ Internal message severity.
/// @return Corresponding `rerun::components::TextLogLevel` value.
inline rerun::components::TextLogLevel get_log_level(VerboseLevels level_) {

  switch (level_) {
  case VerboseLevels::INFO:
    return rerun::TextLogLevel::Info;
  case VerboseLevels::ERROR:
    return rerun::TextLogLevel::Error;
  case VerboseLevels::WARNING:
    return rerun::TextLogLevel::Warning;
  case VerboseLevels::DEBUG:
    return rerun::TextLogLevel::Debug;
  }
  return rerun::TextLogLevel::Info;
}

/// @brief Flush queued interface messages to the Rerun log stream.
void InterfaceRerun::__log_messages() {
  using rerun::TextLog;

  while (!__messages.empty()) {
    using message_t = pair_t<std::string, VerboseLevels>;
    const message_t message = __messages.pop_front_safe();
    const std::string& text = message.first;
    const VerboseLevels& level = message.second;
    __recording->log("logs", TextLog(text).with_level(get_log_level(level)));
  }
}

}  // namespace interface

/// @brief Queue a message for deferred visualization logging.
/// @param message_ Message payload to be displayed.
/// @param verboseType_ Severity used to choose the output log level.
void log_messages_global(
    const std::string& message_,
    VerboseLevels verboseType_) {
  ::slam_mer::interface::__messages.push_back_safe(
      std::make_pair(message_, verboseType_));
}

}  // namespace slam_mer
