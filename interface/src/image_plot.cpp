/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file image_plot.cpp
/// @brief Implements image, depth, and debug-frame logging.
/// @details Defines image buffer updates and Rerun image publishing.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

// My libraries
#include <interface_rerun.hpp>
#ifdef DEBUG
#include <settings.hpp>
#endif

// Third-party libraries
#include <rerun.hpp>

// Standard library
#ifdef DEBUG
#include <algorithm>
#include <array>
#include <vector>
#endif

namespace slam_mer {
namespace interface {

namespace {

#ifdef DEBUG
/// @brief Overlay color for the image-coverage grid.
constexpr std::array<uint8_t, 3> imageCoverageGridColor = {255, 255, 0};
#endif

#ifdef DEBUG
/// @brief Convert 2D image dimensions into Rerun line strips for the coverage
/// grid overlay.
/// @param width_ Image width in pixels.
/// @param height_ Image height in pixels.
/// @param rows_ Number of grid rows.
/// @param cols_ Number of grid columns.
/// @return Vertical and horizontal grid lines in image coordinates.
std::vector<rerun::LineStrip2D> ___to_image_coverage_grid(
    const uint32_t width_,
    const uint32_t height_,
    const int rows_,
    const int cols_) {
  std::vector<rerun::LineStrip2D> strips;
  strips.reserve(
      static_cast<size_t>(std::max(rows_ - 1, 0) + std::max(cols_ - 1, 0)));

  for (int col = 1; col < cols_; ++col) {
    const float x =
        static_cast<float>(width_ * col) / static_cast<float>(cols_);
    strips.emplace_back(
        rerun::Collection<rerun::Vec2D>{
            {x, 0.0F},
            {x, static_cast<float>(height_)}});
  }

  for (int row = 1; row < rows_; ++row) {
    const float y =
        static_cast<float>(height_ * row) / static_cast<float>(rows_);
    strips.emplace_back(
        rerun::Collection<rerun::Vec2D>{
            {0.0F, y},
            {static_cast<float>(width_), y}});
  }

  return strips;
}
#endif

#ifdef DEBUG
/// @brief Convert 2D keypoints to Rerun point primitives.
/// @param points_ Input image-space points.
/// @return Vector of `rerun::Position2D` objects in the same order as input.
std::vector<rerun::Position2D> ___to_rerun_positions(
    const point2d_t::vector& points_) {
  std::vector<rerun::Position2D> positions;
  positions.reserve(points_.size());
  for (const point2d_t& point : points_) {
    positions.emplace_back(
        static_cast<float>(point.at(0)),
        static_cast<float>(point.at(1)));
  }
  return positions;
}
#endif

/// Mutexes for thread safety
std::shared_mutex __image_mutex;
/// Source image to be logged
image_t __sourceImage;
/// Source depth image to be logged
image_t __sourceDepth;
/// Debug image to be logged
image_t __debugImage;
/// @brief Cached image resolution used by image logging.
struct ImageSize {
  /// Image width in pixels.
  uint32_t _width;
  /// Image height in pixels.
  uint32_t _height;
} __imageSize = {0, 0};
/// Tells the library if it has been initialized (need width and height)
bool __initialized = false;

/// List of inliers
point2d_t::vector __inliers;
/// List of all features
point2d_t::vector __allFeatures;
/// Controls one-time static plot style setup.
// bool __featureCountPlotConfigured = false;
/// Frames per second
#if defined(DEBUG)
size_t __fps;
/// Number of mappoints
size_t __numMappoints;
/// Number of cells
size_t __numCells;
/// Number of keyframes
size_t __numKeyframes;
/// Number of query cells
size_t __numQueryCells;
/// Number of closest cells
size_t __numClosestCells;
#endif

}  // namespace

/// External variable to store the cell data for the visualization interface
#if defined(DEBUG) || defined(SHOW_DENSE)
extern std::shared_ptr<rerun::RecordingStream> __recording;
#endif

/// @brief Log frame-level debug plots and feature overlays to Rerun.
void __log_plots() {
  using rerun::Clear;
  using rerun::LineStrip2D;
  using rerun::LineStrips2D;
  using rerun::Points2D;
  using rerun::Radius;
  using rerun::Scalars;
  using rerun::SeriesLines;

#ifdef DEBUG
  using slam_mer::settings::sharedSettings;

  if (sharedSettings->use_image_coverage_condition() && __initialized &&
      __imageSize._width > 0 && __imageSize._height > 0) {
    __recording->log(
        "image/debug/image_coverage_grid",
        LineStrips2D(___to_image_coverage_grid(
                         __imageSize._width,
                         __imageSize._height,
                         sharedSettings->image_coverage_grid_rows(),
                         sharedSettings->image_coverage_grid_cols()))
            .with_colors(
                rerun::Color(
                    imageCoverageGridColor[0],
                    imageCoverageGridColor[1],
                    imageCoverageGridColor[2]))
            .with_radii(Radius::ui_points(1.0F))
            .with_draw_order(rerun::components::DrawOrder(10.0F)));
  } else {
    __recording->log("image/debug/image_coverage_grid", Clear::FLAT);
  }

  if (!__allFeatures.empty()) {
    const auto color = sharedSettings->interface_image_all_point_color();
    __recording->log(
        "image/debug/all_features",
        Points2D(___to_rerun_positions(__allFeatures))
            .with_colors(rerun::Color(color[0], color[1], color[2]))
            .with_radii(Radius::ui_points(2.0F)));
  } else {
    __recording->log("image/debug/all_features", Clear::FLAT);
  }

  if (!__inliers.empty()) {
    const auto color = sharedSettings->interface_image_tracking_color();
    __recording->log(
        "image/debug/inliers",
        Points2D(___to_rerun_positions(__inliers))
            .with_colors(rerun::Color(color[0], color[1], color[2]))
            .with_radii(Radius::ui_points(3.0F)));
  } else {
    __recording->log("image/debug/inliers", Clear::FLAT);
  }

  __recording->log(
      "plots_features/feature_counts",
      Scalars(
          std::vector<rerun::components::Scalar>{
              static_cast<double>(__allFeatures.size()),
              static_cast<double>(__inliers.size())}));
  __recording->log(
      "plots_fps/fps",
      Scalars(
          std::vector<rerun::components::Scalar>{static_cast<double>(__fps)}));
  __recording->log(
      "plots_map/mappoints_keyframes_cells",
      Scalars(
          std::vector<rerun::components::Scalar>{
              static_cast<double>(__numMappoints),
              static_cast<double>(__numKeyframes),
              static_cast<double>(__numCells)}));
  __recording->log(
      "plots_cells/query_and_closest_cells",
      Scalars(
          std::vector<rerun::components::Scalar>{
              static_cast<double>(__numQueryCells),
              static_cast<double>(__numClosestCells)}));
#endif
}

/// @brief Cache the latest RGB source image for backend logging.
/// @param data Debug payload containing the source RGB image.
void log_source_image(const utils::ImageDebugData& data) {

  // This function:
  // 1 - Checks if the interface has been initialized (width and height)
  // 2 - Locks the image mutex for thread safety
  // 3 - Converts the input image to RGB format and stores it in the
  //    global source image variable
  const image_t& image_ = data._image;
  if (!__initialized) {
    __imageSize._width = image_.cols();
    __imageSize._height = image_.rows();
    __initialized = true;
  }
  write_lock_t lock(__image_mutex);
  __sourceImage = image_;
}

/// @brief Cache the latest source depth image for backend logging.
/// @param data Debug payload containing the source depth image.
void log_source_depth_image(const utils::ImageDebugData& data) {

  // This function:
  // 1 - Checks if the interface has been initialized (width and height)
  // 2 - Locks the image mutex for thread safety
  // 3 - Clones the input image to the global source depth variable
  const image_t& image_ = data._depthImage;
  if (!__initialized) {
    __imageSize._width = image_.cols();
    __imageSize._height = image_.rows();
    __initialized = true;
  }
  write_lock_t lock(__image_mutex);
  __sourceDepth = image_.clone();
}

/// @brief Cache the latest debug image and per-frame counters.
/// @param data Debug payload containing debug image, feature tracks, and
/// scalar metrics.
void log_debug_image(const utils::ImageDebugData& data) {

  // This function:
  // 1 - Checks if the interface has been initialized (width and height)
  // 2 - Locks the image mutex for thread safety
  // 3 - Clones the input image to the global debug image variable
  const image_t& image_ = data._image;
  if (!__initialized) {
    __imageSize._width = image_.cols();
    __imageSize._height = image_.rows();
    __initialized = true;
  }
  write_lock_t lock(__image_mutex);

#ifdef DEBUG
  __debugImage = data._grayscaleImage;
  __inliers = data._inliers;
  __allFeatures = data._allFeatures;
  __fps = data._fps;
  __numMappoints = data._numMappoints;
  __numKeyframes = data._numKeyframes;
  __numCells = data._numCells;
  __numQueryCells = data._numQueryCells;
  __numClosestCells = data._numClosestCells;
#endif
}

/// @brief Publish cached RGB/depth/debug images to the Rerun stream.
void InterfaceRerun::__log_images() {
  using rerun::borrow;
  using rerun::Image;

  // This function is used to log the images to the rerun interface.
  // It is called periodically to ensure that the images are always up to date
  // in the rerun visualization.
  // 1 - Lock the mutex to ensure thread safety
  // 2 - Log the source image
  // 3 - Log the source depth image
  // 4 - Log the debug image

  read_lock_t lock(__image_mutex);

#ifdef SHOW_DENSE
  if (!__sourceImage.empty()) {
    __recording->log(
        "scene/camera/image",
        Image::from_rgb24(
            borrow(
                __sourceImage.data(),
                __sourceImage.total() * __sourceImage.elem_size()),
            {__imageSize._width, __imageSize._height}));
  }

  if (!__sourceDepth.empty()) {
    // Convert F32 depth to U8 for Rerun visualization
    // TUM depth values are in range 0-65535, so scale to 0-255
    const size_t n = __sourceDepth.total();
    image_t depthU8(
        __sourceDepth.rows(),
        __sourceDepth.cols(),
        1,
        image_t::Type::U8);

    const float* src = __sourceDepth.ptr<float>(0);
    uint8_t* dst = depthU8.data();
    for (size_t i = 0; i < n; ++i) {
      // Scale from 0-65535 range to 0-255 range
      const float scaled = src[i] / 256.0f;
      dst[i] = (scaled <= 0.0f)     ? uint8_t{0}
               : (scaled >= 255.0f) ? uint8_t{255}
                                    : static_cast<uint8_t>(scaled);
    }

    __recording->log(
        "image/depth",
        Image::from_grayscale8(
            borrow(depthU8.data(), depthU8.total() * depthU8.elem_size()),
            {__imageSize._width, __imageSize._height}));
  }
#endif
#if defined(DEBUG) || defined(SHOW_DENSE)
  if (!__debugImage.empty()) {
    const size_t n = __debugImage.total() * __debugImage.channels();
    image_t debugImageDark(
        __debugImage.rows(),
        __debugImage.cols(),
        __debugImage.channels(),
        image_t::Type::U8);
    uint8_t* debugDst = debugImageDark.data();

    if (__debugImage.type() == image_t::Type::U8) {
      const uint8_t* debugSrc = __debugImage.data();
      for (size_t i = 0; i < n; ++i)
        debugDst[i] = static_cast<uint8_t>(debugSrc[i] / 2U);
    } else {
      const float* debugSrc = __debugImage.ptr<float>(0);
      for (size_t i = 0; i < n; ++i) {
        const float scaled = debugSrc[i] * 0.5f;
        debugDst[i] = (scaled <= 0.0f)     ? uint8_t{0}
                      : (scaled >= 255.0f) ? uint8_t{255}
                                           : static_cast<uint8_t>(scaled);
      }
    }
    __recording->log(
        "image/debug",
        Image::from_grayscale8(
            borrow(
                debugImageDark.data(),
                debugImageDark.total() * debugImageDark.elem_size()),
            {__imageSize._width, __imageSize._height}));
  }
#endif
}

}  // namespace interface
}  // namespace slam_mer
