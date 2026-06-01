/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file ply_writer.cpp
/// @brief Implements PLY export utilities for interface point clouds.
/// @details Defines ASCII/binary PLY serialization helpers.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

#include <ply_writer.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>

namespace {

/// @brief Write a PLY header to an output stream.
/// @param os_ Output stream.
/// @param nPoints_ Number of vertex entries in the file.
/// @param hasColor_ Whether color properties must be emitted.
/// @param binary_ Whether the file format is binary little-endian.
void write_header(
    std::ostream& os_,
    size_t nPoints_,
    bool hasColor_,
    bool binary_) {
  os_ << "ply\n";
  os_ << (binary_ ? "format binary_little_endian 1.0\n" : "format ascii 1.0\n");
  os_ << "element vertex " << nPoints_ << "\n";
  os_ << "property float x\n";
  os_ << "property float y\n";
  os_ << "property float z\n";
  if (hasColor_) {
    os_ << "property uchar red\n";
    os_ << "property uchar green\n";
    os_ << "property uchar blue\n";
  }
  os_ << "end_header\n";

  if (!os_.good()) throw std::runtime_error("Failed writing PLY header");
}

/// @brief Remove points with non-finite coordinates from a point set.
/// @param points_ Point vector filtered in place.
/// @param colors_ Optional color vector filtered in lockstep with `points_`.
void filter_nan(
    std::vector<std::array<float, 3>>& points_,
    std::vector<std::array<uint8_t, 3>>* colors_) {
  if (colors_ && points_.size() != colors_->size())
    throw std::runtime_error("Point and color vector sizes differ.");

  size_t writeIdx = 0;
  for (size_t i = 0; i < points_.size(); ++i) {
    const auto& p = points_[i];
    if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
      points_[writeIdx] = p;
      if (colors_) (*colors_)[writeIdx] = (*colors_)[i];
      ++writeIdx;
    }
  }
  points_.resize(writeIdx);
  if (colors_) colors_->resize(writeIdx);
}

/// @brief Write one POD value in little-endian byte order.
/// @tparam T Trivially copyable POD type.
/// @param os_ Output stream.
/// @param value_ Value to serialize.
template <typename T>
void write_le(std::ostream& os_, T value_) {
  static_assert(std::is_trivially_copyable<T>::value, "T must be POD");
  uint8_t buf[sizeof(T)];
  std::memcpy(buf, &value_, sizeof(T));

  const uint16_t one = 1;
  const bool host_le = (*reinterpret_cast<const uint8_t*>(&one) == 1);
  if (!host_le) {
    for (size_t i = 0; i < sizeof(T) / 2; ++i)
      std::swap(buf[i], buf[sizeof(T) - 1 - i]);
  }
  os_.write(reinterpret_cast<const char*>(buf), sizeof(T));
}

}  // namespace

namespace slam_mer {
namespace utils {

/// @brief Write a point cloud as ASCII PLY.
/// @param filename_ Output filename.
/// @param points_ Input point positions.
/// @param colors_ Optional RGB colors aligned with `points_`.
/// @param flip_ Whether to convert coordinates from camera `rbf` to PLY `flu`.
void write_ply_ascii(
    const std::string& filename_,
    const std::vector<std::array<float, 3>>& points_,
    const std::vector<std::array<uint8_t, 3>>* const colors_,
    bool flip_) {
  if (colors_ && points_.size() != colors_->size())
    throw std::runtime_error("Point and color vector sizes differ.");

  std::ofstream os(filename_);
  if (!os) throw std::runtime_error("Cannot open file: " + filename_);

  auto points = points_;
  auto colors = colors_ ? *colors_ : std::vector<std::array<uint8_t, 3>>();
  filter_nan(points, &colors);

  const bool hasColor = (colors_ != nullptr);
  write_header(os, points.size(), hasColor, false);

  os.setf(std::ios::fixed, std::ios::floatfield);
  os << std::setprecision(6);

  for (size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i];
    if (flip_) {
      // camera coordinates (rbf) -> ply coordinates (flu)
      os << p[2] << ' ' << -p[0] << ' ' << -p[1];
    } else {
      os << p[0] << ' ' << p[1] << ' ' << p[2];
    }

    if (hasColor) {
      const auto& c = colors[i];
      os << ' ' << static_cast<int>(c[0]) << ' ' << static_cast<int>(c[1])
         << ' ' << static_cast<int>(c[2]);
    }
    os << '\n';
  }

  if (!os.good()) throw std::runtime_error("Failed writing ASCII PLY file");
}

/// @brief Write a point cloud as binary little-endian PLY.
/// @param filename_ Output filename.
/// @param points_ Input point positions.
/// @param colors_ Optional RGB colors aligned with `points_`.
/// @param flip_ Whether to convert coordinates from camera `rbf` to PLY `flu`.
void write_ply_binary(
    const std::string& filename_,
    const std::vector<std::array<float, 3>>& points_,
    const std::vector<std::array<uint8_t, 3>>* const colors_,
    bool flip_) {
  if (colors_ && points_.size() != colors_->size())
    throw std::runtime_error("Point and color vector sizes differ.");

  std::ofstream os(filename_, std::ios::binary);
  if (!os) throw std::runtime_error("Cannot open file: " + filename_);

  auto points = points_;
  auto colors = colors_ ? *colors_ : std::vector<std::array<uint8_t, 3>>();
  filter_nan(points, &colors);

  const bool hasColor = (colors_ != nullptr);
  write_header(os, points.size(), hasColor, true);

  for (size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i];
    if (flip_) {
      // camera coordinates (rbf) -> ply coordinates (flu)
      write_le<float>(os, p[2]);
      write_le<float>(os, -p[0]);
      write_le<float>(os, -p[1]);
    } else {
      write_le<float>(os, p[0]);
      write_le<float>(os, p[1]);
      write_le<float>(os, p[2]);
    }

    if (hasColor) {
      const auto& c = colors[i];
      const uint8_t rgb[3] = {c[0], c[1], c[2]};
      os.write(reinterpret_cast<const char*>(rgb), 3 * sizeof(uint8_t));
    }
  }

  if (!os.good()) throw std::runtime_error("Failed writing binary PLY file");
}

/// @brief Write ASCII PLY with colors provided as packed RGBA `uint32_t`.
/// @param filename_ Output filename.
/// @param points_ Input point positions.
/// @param colors_ Optional packed RGBA colors aligned with `points_`.
/// @param flip_ Whether to convert coordinates from camera `rbf` to PLY `flu`.
void write_ply_ascii(
    const std::string& filename_,
    const std::vector<std::array<float, 3>>& points_,
    const std::vector<uint32_t>* const colors_,
    bool flip_) {
  std::vector<std::array<uint8_t, 3>> color_array;
  if (colors_) {
    color_array.reserve(colors_->size());
    for (const auto& c : *colors_) {
      color_array.push_back(
          {static_cast<uint8_t>((c >> 24) & 0xFF),
           static_cast<uint8_t>((c >> 16) & 0xFF),
           static_cast<uint8_t>((c >> 8) & 0xFF)});
    }
  }
  write_ply_ascii(filename_, points_, colors_ ? &color_array : nullptr, flip_);
}

/// @brief Write binary PLY with colors provided as packed RGBA `uint32_t`.
/// @param filename_ Output filename.
/// @param points_ Input point positions.
/// @param colors_ Optional packed RGBA colors aligned with `points_`.
/// @param flip_ Whether to convert coordinates from camera `rbf` to PLY `flu`.
void write_ply_binary(
    const std::string& filename_,
    const std::vector<std::array<float, 3>>& points_,
    const std::vector<uint32_t>* const colors_,
    bool flip_) {
  std::vector<std::array<uint8_t, 3>> color_array;
  if (colors_) {
    color_array.reserve(colors_->size());
    for (const auto& c : *colors_) {
      color_array.push_back(
          {static_cast<uint8_t>((c >> 24) & 0xFF),
           static_cast<uint8_t>((c >> 16) & 0xFF),
           static_cast<uint8_t>((c >> 8) & 0xFF)});
    }
  }
  write_ply_binary(filename_, points_, colors_ ? &color_array : nullptr, flip_);
}

}  // namespace utils
}  // namespace slam_mer
