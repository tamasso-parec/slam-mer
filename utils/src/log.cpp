/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file log.cpp
/// @brief Logging backend implementation for console and file sinks.
/// @details Implements message formatting and dispatch for runtime logging
/// according to configured verbosity and output backend. Provides
/// shared behavior used across modules.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <log.hpp>
#include <types/verbose.hpp>

#include <chrono>
#include <fstream>
#ifdef LOG_TERMINAL
#include <print>
#endif  // LOG_TERMINAL

namespace slam_mer {

#ifdef LOG_INTERFACE
extern void log_messages_global(
    const std::string& message_,
    VerboseLevels verboseType_);
#endif  // LOG_INTERFACE

namespace log {

#ifdef LOG_TERMINAL
// output to terminal
#else   // LOG_TERMINAL
std::ofstream _log{std::string(LOG_FILE) + ".log", std::ios::app};
#endif  // LOG_TERMINAL

/// @brief Writes one fully formatted message to the active log sink.
/// @param message_ Preformatted message text to print or append.
void outputMessage(std::string message_) {
#ifdef LOG_TERMINAL
  using std::println;
  println("[Log] {}", message_);
#else
  _log << message_ << "\n";
#endif  // LOG_TERMINAL
}

/// @brief Converts a module enum value to a short textual label.
/// @param module_ Module identifier converted to a display label.
/// @return Module label string.
std::string get_module_name(Module module_) {
  std::string output;

  switch (module_) {
  case Module::EMPTY:
    output = "General:";
    break;
  case Module::MAIN:
    output = "Main:";
    break;
  case Module::SLAM:
    output = "SLAM:";
    break;
  case Module::MAP:
    output = "MAP:";
    break;
  case Module::TRACKING:
    output = "Tracking:";
    break;
  case Module::MAPPING:
    output = "Local Mapping:";
    break;
  case Module::LOOP_CLOSURE:
    output = "Loop Closure:";
    break;
  case Module::IMAGE_PROCESSING:
    output = "Image Processing:";
    break;
  case Module::KEYFRAME_DATABASE:
    output = "Keyframe Database:";
    break;
  case Module::RANSAC:
    output = "RANSAC:";
    break;
  case Module::VISUALIZATION:
    output = "VISUALIZATION:";
    break;
  case Module::DATALOADER:
    output = "DATALOADER:";
    break;
  case Module::SETTINGS:
    output = "SETTINGS:";
    break;
  default:
    break;
  }
  return output;
}

// ---------------------------------------------------------------------------
// Auxiliary function for getting a label for each of the modules in
// log::Module
// ---------------------------------------------------------------------------

/// @brief Builds a prefixed module string with a timestamp.
/// @param module_ Module identifier used to build the log prefix.
/// @return Timestamped module prefix.
std::string get_module_string(Module module_) {
  using std::chrono::system_clock;

  // Some info
  system_clock::time_point now = system_clock::now();
  std::time_t nowTimeT = system_clock::to_time_t(now);
  std::string timeStamp = "[" + std::to_string(nowTimeT) + "] ";

  // output string
  std::string output = timeStamp + (std::string)BOLDBLACK +
                       (std::string)get_module_name(module_) +
                       (std::string)RESET + (std::string) " ";

  return output;
}

// ---------------------------------------------------------------------------
// General log message with explicit severity type.
// ---------------------------------------------------------------------------
void message(const std::string& str_, TypeMessage type_, Module module_) {

#ifdef LOG_INTERFACE
  // Set the log level based on the VerboseLevels enum
  VerboseLevels level = VerboseLevels::INFO;
  switch (type_) {
  case TypeMessage::GENERAL:
    level = VerboseLevels::INFO;
    break;
  case TypeMessage::WARNING:
    level = VerboseLevels::WARNING;
    break;
  case TypeMessage::ERROR:
    level = VerboseLevels::ERROR;
    break;
  }
  log_messages_global(get_module_name(module_) + ":" + str_, level);
#else
  switch (type_) {
  case TypeMessage::GENERAL:
    message(str_, module_);
    break;
  case TypeMessage::WARNING:
    warning(str_, module_);
    break;
  case TypeMessage::ERROR:
    error(str_, module_);
    break;
  default:
    error("Type option for verbose message not valid", Module::EMPTY);
    break;
  }
#endif  // LOG_INTERFACE
}

// ---------------------------------------------------------------------------
// General log message.
// ---------------------------------------------------------------------------
void message(const std::string& str_, Module module_) {
#ifdef LOG_INTERFACE
  log_messages_global(
      get_module_name(module_) + " " + str_,
      VerboseLevels::INFO);
#else
  std::string message = get_module_string(module_) + str_ + RESET;
  outputMessage(message);
#endif  // LOG_INTERFACE
}

// ---------------------------------------------------------------------------
// Warning message.
// ---------------------------------------------------------------------------
void warning(const std::string& str_, Module module_) {
#ifdef LOG_INTERFACE
  log_messages_global(
      get_module_name(module_) + " " + str_,
      VerboseLevels::WARNING);
#else
  std::string message =
      get_module_string(module_) + YELLOW + "WAR -- " + str_ + RESET;
  outputMessage(message);
#endif  // LOG_INTERFACE
}

// ---------------------------------------------------------------------------
// Error message.
// ---------------------------------------------------------------------------
void error(const std::string& str_, Module module_) {
#ifdef LOG_INTERFACE
  log_messages_global(
      get_module_name(module_) + " " + str_,
      VerboseLevels::ERROR);
#else
  std::string message =
      get_module_string(module_) + RED + "ERR -- " + str_ + RESET;
  outputMessage(message);
#endif  // LOG_INTERFACE
}

}  // namespace log
}  // namespace slam_mer
