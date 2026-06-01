/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file slam.hpp
/// @brief Declares the main SLAM pipeline facade (`slam_mer::Slam`).
///
/// This interface owns the high-level lifecycle of the SLAM runtime and its
/// major modules. It provides entry points for camera data ingestion, IMU
/// ingestion, and retrieval of current pose/map debugging information. Global
/// shared objects declared here expose the active settings and map instances.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki
/// @author Lalit Manam
#ifndef SLAM_HPP
#define SLAM_HPP

// Our headers
#include <debug_data.hpp>
#include <map.hpp>
#include <sensor_data.hpp>
#include <settings.hpp>

// Standard C++ headers
#include <memory>
#include <string>
#include <vector>

namespace slam_mer {

/// @brief Main entry point for running the SLAM pipeline.
class Slam {

  enum class transformation_t { Pose3, Similarity3, UNDEFINED };

public:
  /// Unique ownership pointer type.
  typedef std::unique_ptr<Slam> unique_ptr;
  /// Shared ownership pointer type.
  typedef std::shared_ptr<Slam> shared_ptr;

  /// @brief Creates a `std::unique_ptr<Slam>`.
  /// @param settingsPath_ Path to the settings file.
  /// @return Unique pointer to a new `Slam` instance.
  [[nodiscard("Use make_unique() to create a Slam instance")]]
  static Slam::unique_ptr make_unique(const std::string& settingsPath_) {
    return std::make_unique<Slam>(settingsPath_);
  }

  /// @brief Creates a `std::shared_ptr<Slam>`.
  /// @param settingsPath_ Path to the settings file.
  /// @return Shared pointer to a new `Slam` instance.
  [[nodiscard("Use make_shared() to create a Slam instance")]]
  static Slam::shared_ptr make_shared(const std::string& settingsPath_) {
    return std::make_shared<Slam>(settingsPath_);
  }

  /// @brief Constructs the SLAM pipeline.
  /// @param settingsPath_ Path to the settings YAML file.
  Slam(const std::string& settingsPath_);

  /// @brief Constructs the SLAM pipeline using default settings.
  Slam() : Slam(std::string()) {}

  /// @brief Destroys the SLAM pipeline and releases all modules.
  ~Slam();

  /// @brief Pushes a new camera sample into the SLAM pipeline.
  /// @param data_ Input camera sample (monocular or RGB-D).
  void add(data_type::SensorData::weak_ptr data_);

  /// @brief Pushes a new IMU reading into the SLAM pipeline.
  /// @param imu_ IMU sample payload.
  void add(std::vector<double> imu_);

  /// @brief Returns the current camera pose in world coordinates.
  /// @return Pair containing camera rotation and translation.
  [[nodiscard("Use get_camera_pose() to get the current camera pose")]]
  pair_t<rotation_t, translation_t> get_camera_pose() const;

  /// @brief Get the size of the map.
  /// @return Tuple with number of keyframes, number of map points,
  /// number of query cells, number of closest cells, and total number of cells.
  [[nodiscard("Use get_map_size() to get the current map size")]]
  std::tuple<size_t, size_t, size_t, size_t, size_t> get_map_size() const;

  /// @brief Fills debug visualization data for the latest processed frame.
  /// @param data_ Sensor sample associated with the visualization request.
  /// @param outputData_ Output structure filled with debug render content.
  void get_image_visualization_data(
      data_type::SensorData::shared_ptr data_,
      utils::ImageDebugData& outputData_);

  /// @brief Get the current slam fps.
  /// @return Frames per second.
  double get_frames_per_second();

private:
  /// @brief Instanciate graph, loop, and localization modules based on settings
  template <typename transformation_t>
  void __instanciate_modules();

  /// @brief Terminate all SLAM modules and clear global state.
  template <typename transformation_t>
  void __terminate_modules();

  /// @brief Wrapper to run add() that handles the transformation type
  /// dispatching.
  template <typename transformation_t>
  void __add(data_type::SensorData::weak_ptr data_);

  /// @brief Wrapper to run get_camera_pose() that handles the transformation
  /// type dispatching.
  template <typename transformation_t>
  pair_t<rotation_t, translation_t> __get_camera_pose() const;

  /// @brief Wrapper to the get image visualization data function
  /// that handles the transformation type dispatching.
  template <typename transformation_t>
  void __get_image_visualization_data(
      data_type::SensorData::shared_ptr data_,
      utils::ImageDebugData& outputData_);

  /// @brief Internal transformation type used for dispatching module
  /// implementations.
  transformation_t _type = transformation_t::UNDEFINED;
};

}  // namespace slam_mer

#endif  // SLAM_HPP
