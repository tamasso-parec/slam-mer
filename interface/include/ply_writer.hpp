/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file ply_writer.hpp
/// @brief Utilities to export point clouds to PLY files.
/// @details Declares ASCII and binary PLY writers for point clouds with
/// optional color attributes and optional coordinate flipping for visualization
/// compatibility with external tools.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

#ifndef PLY_WRITER_HPP
#define PLY_WRITER_HPP

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace slam_mer {
namespace utils {

/// @brief Write a point cloud to an ASCII PLY file.
/// @param filename_ Output path of the generated PLY file.
/// @param points_ Point positions in XYZ order.
/// @param colors_ Optional RGB colors aligned with `points_`.
/// @param flip_ When `true`, converts coordinates from camera `rbf` to PLY
/// `flu`.
void write_ply_ascii(
    const std::string& filename_,
    const std::vector<std::array<float, 3>>& points_,
    const std::vector<std::array<uint8_t, 3>>* const colors_,
    bool flip_ = false);

/// @brief Write a point cloud to a binary little-endian PLY file.
/// @param filename_ Output path of the generated PLY file.
/// @param points_ Point positions in XYZ order.
/// @param colors_ Optional RGB colors aligned with `points_`.
/// @param flip_ When `true`, converts coordinates from camera `rbf` to PLY
/// `flu`.
void write_ply_binary(
    const std::string& filename_,
    const std::vector<std::array<float, 3>>& points_,
    const std::vector<std::array<uint8_t, 3>>* const colors_,
    bool flip_ = false);

/// @brief Write a point cloud to an ASCII PLY file using packed RGBA colors.
/// @param filename_ Output path of the generated PLY file.
/// @param points_ Point positions in XYZ order.
/// @param colors_ Optional packed RGBA colors (high byte = red, then green,
/// blue, alpha).
/// @param flip_ When `true`, converts coordinates from camera `rbf` to PLY
/// `flu`.
void write_ply_ascii(
    const std::string& filename_,
    const std::vector<std::array<float, 3>>& points_,
    const std::vector<uint32_t>* const colors_ = nullptr,
    bool flip_ = false);

/// @brief Write a binary little-endian PLY file using packed RGBA colors.
/// @param filename_ Output path of the generated PLY file.
/// @param points_ Point positions in XYZ order.
/// @param colors_ Optional packed RGBA colors (high byte = red, then green,
/// blue, alpha).
/// @param flip_ When `true`, converts coordinates from camera `rbf` to PLY
/// `flu`.
void write_ply_binary(
    const std::string& filename_,
    const std::vector<std::array<float, 3>>& points_,
    const std::vector<uint32_t>* const colors_ = nullptr,
    bool flip_ = false);

}  // namespace utils
}  // namespace slam_mer

#endif  // PLY_WRITER_HPP
