/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @file main_euroc.cpp
/// @brief Run monocular SLAM on an unmodified EuRoC camera sequence.

#include <yaml-cpp/yaml.h>
#include <arg_parser.hpp>
#include <common.hpp>
#include <dataloader_video.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <print>
#include <stdexcept>
#include <vector>

namespace slam_mer::dataloader {

/// @brief Read cam0 frames with EuRoC calibration and real capture times.
class DataloaderEuroc : public Dataloader {
public:
  /// @brief Load a sequence directory, mav0 directory, or camera directory.
  /// @param path_ Input directory in the original EuRoC layout.
  /// @param maxFrames_ Maximum frames to process; zero means the full sequence.
  DataloaderEuroc(std::filesystem::path path_, size_t maxFrames_) {
    if (std::filesystem::is_directory(path_ / "mav0")) path_ /= "mav0";
    if (std::filesystem::is_directory(path_ / "cam0")) path_ /= "cam0";
    _cameraPath = std::filesystem::canonical(path_);
    const YAML::Node sensor =
        YAML::LoadFile((_cameraPath / "sensor.yaml").string());
    if (sensor["camera_model"].as<std::string>() != "pinhole" ||
        sensor["distortion_model"].as<std::string>() != "radial-tangential")
      throw std::runtime_error(
          "Expected a pinhole camera with radial-tangential distortion");
    const auto intrinsics = sensor["intrinsics"].as<std::vector<double>>();
    const auto distortion =
        sensor["distortion_coefficients"].as<std::vector<double>>();
    const auto resolution = sensor["resolution"].as<std::vector<int>>();
    if (intrinsics.size() != 4 || distortion.size() != 4 ||
        resolution.size() != 2 || intrinsics[0] <= 0 || intrinsics[1] <= 0 ||
        resolution[0] <= 0 || resolution[1] <= 0)
      throw std::runtime_error("Invalid EuRoC camera calibration");
    _intrinsics << intrinsics[0], 0, intrinsics[2], 0, intrinsics[1],
        intrinsics[3], 0, 0, 1;
    const cv::Mat camera =
        (cv::Mat_<double>(3, 3) << intrinsics[0],
         0,
         intrinsics[2],
         0,
         intrinsics[1],
         intrinsics[3],
         0,
         0,
         1);
    _resolution = cv::Size(resolution[0], resolution[1]);
    cv::initUndistortRectifyMap(
        camera,
        cv::Mat(distortion),
        cv::Mat(),
        camera,
        _resolution,
        CV_32FC1,
        _mapX,
        _mapY);

    std::ifstream input(_cameraPath / "data.csv");
    if (!input)
      throw std::runtime_error(
          "Cannot open " + (_cameraPath / "data.csv").string());
    std::string line;
    uint64_t previous = 0;
    while (std::getline(input, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty() || line.front() == '#') continue;
      const size_t comma = line.find(',');
      if (comma == std::string::npos)
        throw std::runtime_error("Invalid EuRoC CSV row: " + line);
      const std::string stamp = line.substr(0, comma);
      if (stamp.empty() ||
          stamp.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("Invalid EuRoC timestamp: " + stamp);
      // time_stamp_t uses microseconds. Convert with integer arithmetic, without
      // inventing a frame rate or losing precision through floating point.
      const uint64_t microseconds = std::stoull(stamp) / 1000;
      if (!_frames.empty() && microseconds <= previous)
        throw std::runtime_error(
            "EuRoC timestamps must increase at microsecond resolution");
      const auto image = _cameraPath / "data" / line.substr(comma + 1);
      if (!std::filesystem::is_regular_file(image))
        throw std::runtime_error("Missing EuRoC image: " + image.string());
      _frames.push_back({microseconds, image});
      previous = microseconds;
    }
    if (_frames.empty())
      throw std::runtime_error("No frames found in EuRoC data.csv");
    if (maxFrames_ > 0 && maxFrames_ < _frames.size())
      _frames.resize(maxFrames_);
    _timeStampPrecision = 6;
    std::println(
        "EuRoC camera: {} ({} frames, {}x{})",
        _cameraPath.string(),
        _frames.size(),
        _resolution.width,
        _resolution.height);
    std::println(
        "Camera intrinsics: fx={}, fy={}, cx={}, cy={}",
        intrinsics[0],
        intrinsics[1],
        intrinsics[2],
        intrinsics[3]);
  }

  /// @brief Decode, undistort, and return the next calibrated RGB frame.
  /// @return Sensor data, or nullptr after the sequence ends.
  data_type::SensorData::shared_ptr get_data() override {
    if (_current == _frames.size()) {
      _end = true;
      return nullptr;
    }
    const auto& [timestamp, path] = _frames[_current++];
    const cv::Mat source = cv::imread(path.string(), cv::IMREAD_COLOR);
    if (source.empty() || source.size() != _resolution)
      throw std::runtime_error(
          "Cannot read image at calibrated resolution: " + path.string());
    cv::Mat corrected, rgb;
    cv::remap(source, corrected, _mapX, _mapY, cv::INTER_LINEAR);
    cv::cvtColor(corrected, rgb, cv::COLOR_BGR2RGB);
    image_t image(rgb.rows, rgb.cols, 3, image_t::Type::U8);
    for (int row = 0; row < rgb.rows; ++row)
      std::memcpy(
          image.ptr(row),
          rgb.ptr(row),
          static_cast<size_t>(rgb.cols) * 3);
    // The upstream sensor adapter scales both image and intrinsics to its
    // inference resolution. Images are already undistorted at this point.
    return SensorDataVideo::make_shared(
        image,
        time_stamp_t(timestamp),
        _intrinsics);
  }

  /// @brief Report whether an end-of-sequence sample has been requested.
  /// @return True after the last valid frame has been consumed.
  bool done() override { return _end; }
  /// @brief Return the number of selected frames.
  int get_num_images() const override {
    return static_cast<int>(_frames.size());
  }
  /// @brief Return the index of the last emitted frame, or -1 before reading.
  int get_current_image_id() const override {
    return static_cast<int>(_current) - 1;
  }

private:
  std::filesystem::path _cameraPath;
  std::vector<std::pair<uint64_t, std::filesystem::path>> _frames;
  Eigen::Matrix3d _intrinsics;
  cv::Size _resolution;
  cv::Mat _mapX, _mapY;
  size_t _current = 0;
  bool _end = false;
};
}  // namespace slam_mer::dataloader

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
