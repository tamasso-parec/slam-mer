/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @file main_euroc.cpp
/// @brief Run monocular SLAM on an unmodified EuRoC camera sequence.

#include <arg_parser.hpp>
#include <common.hpp>
#include "dataloader_euroc.hpp"

/// @brief Parse EuRoC input paths and dispatch the upstream SLAM runner.
int main(int argc_, char** argv_) {
  try {
    parser::ArgParser parser("slam-mer_euroc");
    parser.add_argument(
        "--dataset",
        "-d",
        "/datasets",
        "EuRoC dataset root or sequence directory");
    parser.add_argument(
        "--sequence",
        "-seq",
        "MH_01_easy",
        "Sequence name; ignored for direct sequence paths");
    parser.add_argument(
        "--settings",
        "-s",
        "settings/tum_monocular.yaml",
        "SLAM settings YAML");
    parser.add_argument(
        "--outTrajFileTUM",
        "-outTUM",
        "/output/trajectory.txt",
        "Output camera trajectory");
    parser.add_argument(
        "--max-frames",
        "",
        "0",
        "Limit frames; zero processes the full sequence");
    parser.add_argument(
        "--check",
        "",
        "0",
        "Decode the sequence without running SLAM [0 or 1]");
    parser.parse(argc_, argv_);
    std::filesystem::path path = parser.get_argument("--dataset");
    if (std::filesystem::is_directory(
            path / "EuroC" / parser.get_argument("--sequence")))
      path /= "EuroC";
    if (!std::filesystem::exists(path / "mav0") &&
        !std::filesystem::exists(path / "cam0") &&
        !std::filesystem::exists(path / "sensor.yaml"))
      path /= parser.get_argument("--sequence");
    const std::string limit = parser.get_argument("--max-frames");
    if (limit.empty() ||
        limit.find_first_not_of("0123456789") != std::string::npos)
      throw std::runtime_error("--max-frames must be a nonnegative integer");
    auto loader = std::make_unique<slam_mer::dataloader::DataloaderEuroc>(
        path,
        std::stoull(limit));
    if (parser.get_argument("--check") == "1") {
      size_t count = 0;
      while (auto sample = loader->get_data()) {
        if (!sample->is_valid())
          throw std::runtime_error("Invalid decoded sensor sample");
        if (count++ == 0)
          std::println(
              "First timestamp (microseconds): {}",
              sample->get_time_stamp().get());
      }
      std::println("Validated {} calibrated EuRoC frames", count);
      return 0;
    }
    return slam_mer::run(
        std::move(loader),
        parser.get_argument("--settings"),
        parser.get_argument("--sequence"),
        20,
        parser.get_argument("--outTrajFileTUM"));
  } catch (const std::exception& error) {
    std::println(stderr, "EuRoC run failed: {}", error.what());
    return 1;
  }
}
