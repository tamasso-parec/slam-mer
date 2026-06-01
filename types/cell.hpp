/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file cell.hpp
/// @brief 3D spatial cell representation for map partitioning and
/// visualization.
/// @details Implements a compact voxel-like cell type with bounds, center,
/// highlight state, and point count metadata. It is intended for
/// occupancy-style grouping, neighborhood processing, and debug
/// visualization flows.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPES_CELL_HPP
#define TYPES_CELL_HPP

// Need to include the representation for the octree node
#include <types/array.hpp>

/// Represents a 3D cell with bounds, _center, point count, and highlight state.
namespace slam_mer {

/// Class implementing a 3D cell, which can be used in spatial data
/// structures like octrees or voxel grids.
class cell3d_t {
public:
  /// Default constructor initializes an empty cell with zero bounds and
  /// _center.
  cell3d_t() = default;
  /// Constructs a cell with specified minimum and maximum bounds, and computes
  /// the _center.
  /// @param minBound_ Minimum corner of the cell bounding box.
  /// @param maxBound_ Maximum corner of the cell bounding box.
  cell3d_t(const array3D_t& minBound_, const array3D_t& maxBound_)
      : _minBound(minBound_),
        _maxBound(maxBound_),
        _center((minBound_ + maxBound_) / 2.0) {}

  /// Set the state of the cell to highlighted or not
  /// @param highlight_ New highlight flag used for visualization emphasis.
  inline void set_highlighted(const bool highlight_) {
    _isHighlighted = highlight_;
  }

  /// Set the point count of the cell
  /// @param count_ Number of map points currently assigned to this cell.
  inline void set_point_count(const size_t count_) { _pointCount = count_; }

  /// Get _center of the cell
  /// @return Center coordinate of the cell bounding box.
  [[nodiscard("Retrieving cell center.")]]
  inline array3D_t get_center() const {
    return _center;
  }

  /// Get min bound of the cell
  /// @return Minimum corner of the cell bounding box.
  [[nodiscard("Retrieving cell minimum bound.")]]
  inline array3D_t get_min_bound() const {
    return _minBound;
  }

  /// Get max bound of the cell
  /// @return Maximum corner of the cell bounding box.
  [[nodiscard("Retrieving cell maximum bound.")]]
  inline array3D_t get_max_bound() const {
    return _maxBound;
  }

  /// Returns whether the cell is currently highlighted.
  /// @return `true` when this cell is marked as highlighted.
  [[nodiscard("Checking if cell is highlighted.")]]
  inline bool is_highlighted() const {
    return _isHighlighted;
  }

  /// Get point count of the cell
  /// @return Number of points stored in this cell.
  [[nodiscard("Retrieving cell point count.")]]
  inline size_t get_point_count() const {
    return _pointCount;
  }

private:
  /// Minimum bound of the cell
  array3D_t _minBound;
  /// Maximum bound of the cell
  array3D_t _maxBound;
  /// Center point of the cell
  array3D_t _center;
  /// Number of points within the cell
  size_t _pointCount{0};
  /// Highlight state of the cell
  bool _isHighlighted{false};
};

}  // namespace slam_mer

#endif  // TYPES_CELL_HPP
