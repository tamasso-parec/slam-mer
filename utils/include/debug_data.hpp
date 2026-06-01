/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file debug_data.hpp
/// @brief Shared debug-image payloads for visualization and diagnostics.
/// @details Defines lightweight enum/struct types used to transfer image,
/// feature, and status-overlay data into debug visualization flows.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef DEBUG_DATA_HPP
#define DEBUG_DATA_HPP

// Our headers
#include <types/image.hpp>
#include <types/point2d.hpp>

// Standard C++ headers
#include <string>
#include <vector>

namespace slam_mer {
namespace utils {

/// @brief Selects which feature subset is rendered in debug views.
enum TypeImageForDebugging { ALL, Inliers };

/// @brief Bundle of image and overlay data consumed by debug visualization.
struct ImageDebugData {
  /// Image array
  image_t _image;
  /// Depth image array
  image_t _depthImage;
  /// Grayscale image
  image_t _grayscaleImage;
  /// List of inliers
  point2d_t::vector _inliers;
  /// List of all features
  point2d_t::vector _allFeatures;
  /// Frames per second
  size_t _fps;
  /// Number of mappoints
  size_t _numMappoints;
  /// Number of cells
  size_t _numCells;
  /// Number of keyframes
  size_t _numKeyframes;
  /// Number of query cells
  size_t _numQueryCells;
  /// Number of closest cells
  size_t _numClosestCells;

  /// Message to show in the image (highlight)
  std::string _highlightMessage;
};

}  // namespace utils
}  // namespace slam_mer

#endif  // DEBUG_DATA_HPP
