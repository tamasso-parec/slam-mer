/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file representation.hpp
/// @brief Spatial representation structures for 3D map organization.
/// @details Declares octree-related and partitioning utilities that organize
/// points into spatial cells for efficient lookup and updates. The
/// abstractions support mapping and visualization subsystems.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef REPRESENTATION_DEV_HPP
#define REPRESENTATION_DEV_HPP

#include <types/array.hpp>
#include <types/camera.hpp>
#include <types/hash.hpp>
#include <types/id.hpp>
#include <types/output.hpp>
#include <types/pose.hpp>
#include <types/vector.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <shared_mutex>

namespace slam_mer {

/// @brief 3D spatial representation backed by coarse and fine voxel grids.
/// @details Coarse cells group nearby fine voxels for fast neighborhood
/// retrieval. Fine voxels hold point-to-cell assignments and visibility
/// metadata used by mapping and visualization routines.
class Representation {
public:
  /// Shared pointer type
  using shared_ptr = std::shared_ptr<Representation>;
  /// @brief Creates a shared representation instance.
  /// @param voxelHalfSize_ Half-size of fine voxels in world units.
  /// @param coarseHalfSize_ Half-size of coarse cells in world units.
  /// @return Shared pointer to a new `Representation`.
  static Representation::shared_ptr make_shared(
      double voxelHalfSize_,
      double coarseHalfSize_) {
    return std::make_shared<Representation>(voxelHalfSize_, coarseHalfSize_);
  }

  /// @brief Creates a shared representation instance with explicit scale.
  /// @param voxelHalfSize_ Half-size of fine voxels in world units.
  /// @param coarseHalfSize_ Half-size of coarse cells in world units.
  /// @param scale_ Conversion scale from world units to internal units.
  /// @return Shared pointer to a new `Representation`.
  static Representation::shared_ptr
  make_shared(double voxelHalfSize_, double coarseHalfSize_, double scale_) {
    return std::make_shared<Representation>(
        voxelHalfSize_,
        coarseHalfSize_,
        scale_);
  }

  /// @brief Constructor
  /// @param voxelHalfSizeUnscaled_ Fine-voxel half-size in world units.
  /// @param coarseHalfSizeUnscaled_ Coarse-cell half-size in world units.
  /// @param scale_ Conversion scale from world units to internal units.
  Representation(
      const double voxelHalfSizeUnscaled_,
      const double coarseHalfSizeUnscaled_,
      const double scale_ = 10000.0);

  /// @brief Destructor.
  ~Representation() = default;

  /// @brief Returns the number of currently ordered cells.
  /// @return Number of entries in `orderedCellIds`.
  inline size_t size_closest_cells() const {
    return _orderedCellIds.get_size_safe();
  }

  /// @brief Returns the number of cells currently in the representation.
  /// @return Number of cells in the representation {numberQueryCells,
  /// numberClosestCells, totalNumberCells}.
  inline size_t get_number_cells() { return _allCells.size_(); }

  /// @brief Checks whether a cell exists.
  /// @param cellId_ Cell identifier checked for existence.
  /// @return `true` if the cell exists.
  bool exists(const cell_id_t& cellId_) const;

  /// @brief Get coarse cells (ids) for sorting, given a point
  /// @param pointUnscaled_ Query point in world coordinates (camera location).
  /// @return Cell ids associated with the coarse bucket around the query.
  cell_id_t::vector get_cells(const array3D_t& pointUnscaled_);

  /// @brief Get cell id for a given point in the world
  /// @param pointUnscaled_ Query point in world coordinates.
  /// @return Cell id of the voxel containing the query point.
  cell_id_t get_cell_id(const array3D_t& pointUnscaled_);

  /// @brief Gets the cell that currently contains a point id.
  /// @param pointId_ Point identifier mapped to a voxel cell.
  /// @return Cell identifier containing the point.
  cell_id_t get_cell_id(const point_id_t& pointId_) const;

  /// @brief Get the corners of a given cell
  /// @param cellId_ Cell identifier whose corners are requested.
  /// @return Pair containing minimum and maximum cell corners.
  inline array3D_t::pair get_cell_corners(const cell_id_t& cellId_) const {
    read_lock_t lock(_mutex);
    return __get_cell_corners(cellId_);
  };

  /// @brief Gets the center of a given cell.
  /// @param cellId_ Cell identifier whose center is requested.
  /// @return Cell center in world coordinates.
  inline array3D_t get_center(const cell_id_t& cellId_) const {
    read_lock_t lock(_mutex);
    return __get_center(cellId_);
  }

  /// @brief Gets all point ids contained in a cell.
  /// @param cellId_ Cell identifier whose points are requested.
  /// @return Set of point ids, or a failure code.
  output_t<point_id_t::set> get_points(const cell_id_t& cellId_);

  /// @brief Inserts a point into the representation
  /// @param cellId_ Destination cell receiving the point id.
  /// @param pointId_ Point identifier inserted into `cellId_`.
  /// @return Success or failure status of the insertion.
  fail_t insert(const cell_id_t& cellId_, const point_id_t& pointId_);

  /// @brief Removes a point from the representation
  /// @param cellId_ Source cell from which the point id is removed.
  /// @param pointId_ Point identifier removed from `cellId_`.
  /// @return Success or failure status of the removal.
  fail_t remove(const cell_id_t& cellId_, const point_id_t& pointId_);

  /// @brief Removes a point from the representation by point id only.
  /// @param pointId_ Point identifier removed from its currently mapped cell.
  /// @return Success or failure state.
  fail_t remove(const point_id_t& pointId_);

  /// @brief Checks if a point is contained in a given cell
  /// @param cellId_ Cell identifier used for the containment test.
  /// @param point_ Query point coordinates in world units.
  /// @return `true` if `point_` lies inside `cellId_`.
  output_t<bool> contains(const cell_id_t& cellId_, const array3D_t& point_)
      const;

  /// @brief Moves a point from one cell to another.
  /// @param oldCellId_ Cell currently holding `pointId_`.
  /// @param newCellId_ Destination cell receiving `pointId_`.
  /// @param pointId_ Point identifier moved between cells.
  /// @return Success or failure state.
  fail_t switch_cell(
      const cell_id_t& oldCellId_,
      const cell_id_t& newCellId_,
      const point_id_t& pointId_);

  /// @brief Gets the corners of a cell.
  /// @param cellId_ Cell identifier whose corners are requested.
  /// @return Pair `(minCorner, maxCorner)` in world coordinates.
  array3D_t::pair get_corners(const cell_id_t& cellId_) const;

  /// @brief Marks a cell as highlighted.
  /// @param cellId_ Cell identifier marked as highlighted.
  void highlight(const cell_id_t& cellId_);

  /// @brief Clears highlight state for a cell.
  /// @param cellId_ Cell identifier marked as not highlighted.
  void unhighlight(const cell_id_t& cellId_);

  /// @brief Checks whether a cell is highlighted.
  /// @param cellId_ Cell identifier queried for highlight state.
  /// @return `true` when the cell is highlighted.
  inline bool is_highlighted(const cell_id_t& cellId_) const {
    read_lock_t lock(_mutex);
    return _allCells.at_(cellId_)->is_highlighted();
  }

  /// @brief Returns the number of points stored in a cell.
  /// @param cellId_ Cell identifier queried for point count.
  /// @return Point count for the cell.
  inline size_t get_point_count(const cell_id_t& cellId_) const {
    read_lock_t lock(_mutex);
    return _allCells.at_(cellId_)->get_point_count();
  }

  /// @brief Sorts cells according to camera visibility heuristics.
  /// @param cameraToWorldTransformation_ Camera-to-world pose used to rank
  /// cells by distance and visibility.
  /// @param intrinsics_ Camera intrinsics used for image-visibility checks.
  void sort_cells(
      const pose_t& cameraToWorldTransformation_,
      const intrinsics_t& intrinsics_);

  /// @brief Returns one cell id from the ordered-cell buffer.
  /// @param item_ Zero-based index into the ordered-cell buffer.
  /// @return Cell identifier at the requested position.
  inline output_t<cell_id_t> get_closes_cell_x(size_t item_) {
    _orderedCellIds.shared_lock();
    output_t<cell_id_t> output = fail_t::NO_FAIL;
    if (item_ < _orderedCellIds.size())
      output = _orderedCellIds.at(item_);
    else
      output = fail_t::OUT_OF_BOUNDS;
    _orderedCellIds.unlock_shared();
    return output;
  }

private:
  // --------------------------------------------------------------------------
  /// Class that represents a 3D voxel (cell) in space.
  class Voxel {
  public:
    /// Pointer types: shared pointer
    using shared_ptr = std::shared_ptr<Voxel>;
    /// Weak pointer type
    using weak_ptr = std::weak_ptr<Voxel>;
    /// @brief Creates a shared voxel instance.
    /// @param cornerMin_ Minimum voxel corner in internal integer units.
    /// @param cornerMax_ Maximum voxel corner in internal integer units.
    /// @param cellId_ Unique cell identifier assigned to the voxel.
    /// @return Shared pointer to a new `Voxel`.
    static shared_ptr make_shared(
        const array3Int_t& cornerMin_,
        const array3Int_t& cornerMax_,
        const cell_id_t& cellId_) {
      return std::make_shared<Voxel>(cornerMin_, cornerMax_, cellId_);
    }

    /// @brief Constructs a voxel from integer-space corners.
    /// @param cornerMin_ Minimum voxel corner in internal integer units.
    /// @param cornerMax_ Maximum voxel corner in internal integer units.
    /// @param cellId_ Unique cell identifier assigned to the voxel.
    Voxel(
        const array3Int_t& cornerMin_,
        const array3Int_t& cornerMax_,
        const cell_id_t& cellId_)
        : _cornerMin(cornerMin_), _cornerMax(cornerMax_), _cellId(cellId_) {}

    /// @brief Checks whether a point id is registered in this voxel.
    /// @param pointId_ Point identifier tested for membership in this voxel.
    /// @return `true` if the point id exists in the voxel set.
    bool contains(const point_id_t& pointId_) const;

    /// @brief Checks whether coordinates lie inside voxel bounds.
    /// @param point_ Query coordinates in internal units.
    /// @return `true` if the point is inside the voxel AABB.
    bool contains(const array3D_t& point_) const;

    /// @brief Returns voxel integer corners.
    /// @return Pair `(minCorner, maxCorner)`.
    array3Int_t::pair get_corners() const;

    /// @brief Returns voxel center.
    /// @return Center coordinates.
    array3D_t get_center() const;

    /// @brief Inserts a point id into the voxel.
    /// @param pointId_ Point identifier inserted into this voxel.
    /// @return Success or failure state.
    fail_t insert(const point_id_t& pointId_);

    /// @brief Removes a point id from the voxel.
    /// @param pointId_ Point identifier removed from this voxel.
    /// @return Success or failure state.
    fail_t remove(const point_id_t& pointId_);

    /// @brief Returns the cell identifier.
    /// @return Cell identifier.
    cell_id_t get_id() const { return _cellId; }

    /// @brief Returns all point ids currently stored in the voxel.
    /// @return Copy of the point-id set.
    point_id_t::set get_points() {
      read_lock_t lock(_mutex);
      return _points;
    }

    /// @brief Marks the voxel as visible.
    void highlight() {
      write_lock_t lock(_mutex);
      _isVisible = true;
    }

    /// @brief Marks the voxel as not visible.
    void unhighlight() {
      write_lock_t lock(_mutex);
      _isVisible = false;
    }

    /// @brief Returns whether the voxel is visible.
    /// @return `true` if the voxel is highlighted.
    inline bool is_highlighted() const {
      read_lock_t lock(_mutex);
      return _isVisible;
    }

    /// @brief Returns the number of points in the voxel.
    /// @return Number of stored point ids.
    size_t get_point_count() const {
      read_lock_t lock(_mutex);
      return _points.size();
    }

  private:
    /// Corners of the cell: Min
    array3Int_t _cornerMin{0, 0, 0};
    /// Corners of the cell: Max
    array3Int_t _cornerMax{0, 0, 0};
    /// Points contained in the cell
    point_id_t::set _points;
    /// cell id of the voxel
    cell_id_t _cellId;
    /// Whether the voxel is visible or not
    bool _isVisible{false};

    /// Mutex for thread-safe access
    mutable std::shared_mutex _mutex;
    /// @brief Internal point-id containment check without locking.
    /// @param pointId_ Point identifier tested for membership in `_points`.
    /// @return `true` if the point id exists in `points`.
    bool __contains(const point_id_t& pointId_) const;
  };
  // --------------------------------------------------------------------------

  /// half the size of the coarse cell (in internal representation)
  int _coarseHalfSize;
  /// half the size of the voxel cell (in internal representation)
  int _voxelHalfSize;
  /// Scale to the internal representation
  double _scale;
  /// Mutex for thread-safe access
  mutable std::shared_mutex _mutex;
  /// Ordered list of all the octree nodes
  cell_id_t::vector _orderedCellIds;

  /// Coarse mapping from coarse cell center to list of voxel centers
  hash_t<array3Int_t, cell_id_t::vector> _coarseRepresentation;

  /// All cells by their id
  hash_t<cell_id_t, Voxel::shared_ptr> _allCells;

  /// Fine representation mapping from voxel center to Voxel object.
  hash_t<array3Int_t, cell_id_t> _fineRepresentation;

  /// Mapping from point id to cell id
  hash_t<point_id_t, cell_id_t> _pointToCellMap;

  /// @brief Computes a cell center from its corners.
  /// @param cellId_ Cell identifier whose center is computed.
  /// @return Cell center in world coordinates.
  array3D_t __get_center(const cell_id_t& cellId_) const {
    array3D_t::pair corners = __get_cell_corners(cellId_);
    return array3D_t{
        (corners.first[0] + corners.second[0]) / 2.0,
        (corners.first[1] + corners.second[1]) / 2.0,
        (corners.first[2] + corners.second[2]) / 2.0};
  }

  /// @brief Checks whether a cell contains no points.
  /// @param cellId_ Cell identifier checked for emptiness.
  /// @return `true` if the cell has zero points.
  bool __is_cell_empty(const cell_id_t& cellId_) const {
    return _allCells.at_(cellId_)->get_points().get_size_safe() == 0;
  }

  /// @brief Retrieves cell corners in world coordinates.
  /// @param cellId_ Cell identifier whose corners are requested.
  /// @return Pair `(minCorner, maxCorner)`.
  array3D_t::pair __get_cell_corners(const cell_id_t& cellId_) const;

  /// @brief Validates that a cell id exists.
  /// @param cellId_ Cell identifier checked for existence.
  /// @return `fail_t::NO_FAIL` when valid, otherwise an error code.
  fail_t __sanity_exists(const cell_id_t& cellId_) const;

  /// @brief Validates that a cell id does not exist.
  /// @param cellId_ Cell identifier checked for non-existence.
  /// @return `fail_t::NO_FAIL` when valid, otherwise an error code.
  fail_t __sanity_not_exists(const cell_id_t& cellId_) const;

  /// @brief Validates that a point id exists.
  /// @param pointId_ Point identifier checked for existence.
  /// @return `fail_t::NO_FAIL` when valid, otherwise an error code.
  fail_t __sanity_exists(const point_id_t& pointId_) const;

  /// @brief Validates that a point id does not exist.
  /// @param pointId_ Point identifier checked for non-existence.
  /// @return `fail_t::NO_FAIL` when valid, otherwise an error code.
  fail_t __sanity_not_exists(const point_id_t& pointId_) const;

  /// @brief Checks whether a coarse cell entry exists.
  /// @param coarseCenter_ Coarse-cell center in internal integer units.
  /// @return `true` if the coarse entry exists.
  bool __coarse_exists(const array3Int_t& coarseCenter_) const;

  /// @brief Computes coarse-cell center for a world point.
  /// @param point_ Query point in scaled internal coordinates.
  /// @return Coarse-cell center.
  array3Int_t __compute_coarse_center(const array3D_t& point_) const;
  /// @brief Computes voxel center for a world point.
  /// @param point_ Query point in scaled internal coordinates.
  /// @return Voxel center.
  array3Int_t __compute_voxel_center(const array3D_t& point_) const;
  /// @brief Creates a new voxel and assigns it a new cell id.
  /// @param voxelCenter_ Center of the voxel in internal integer units.
  /// @return Pair `(cellId, voxelPointer)` for the created voxel.
  pair_t<cell_id_t, Voxel::shared_ptr> __create_voxel(
      const array3Int_t& voxelCenter_);

  /// @brief Prints a terminal histogram of fine-cell entries per coarse cell.
  /// @details Each line shows coarse center coordinates, a star bar, and count.
  void __print_coarse_cells_histogram();

  /// Scale the point from world units to internal representation (*100).
  /// @param point_ Point in world units.
  /// @return Point converted to scaled internal coordinates.
  inline array3D_t __scale(const array3D_t& point_) const {
    return array3D_t{
        point_[0] * _scale,
        point_[1] * _scale,
        point_[2] * _scale};
  }

  /// @brief Scales a scalar from world units to internal units.
  /// @param value_ Scalar value in world units.
  /// @return Scaled scalar value.
  inline double __scale(const double& value_) const { return value_ * _scale; }

  /// Descale the point from internal representation to world units (/100).
  /// @param point_ Point in scaled internal coordinates.
  /// @return Point converted back to world units.
  inline array3D_t __descale(const array3Int_t& point_) const {
    return array3D_t{
        static_cast<double>(point_[0]) / _scale,
        static_cast<double>(point_[1]) / _scale,
        static_cast<double>(point_[2]) / _scale};
  }

  /// @brief Descales a scalar from internal units to world units.
  /// @param value_ Scalar value in scaled internal units.
  /// @return Descaled scalar value.
  inline double __descale(const double& value_) const {
    return value_ / _scale;
  }
};

/// @brief Throws when a synchronized remove operation fails.
/// @param fail_ Failure status returned by remove operation.
/// @param cellId_ Cell id involved in the remove operation.
/// @param pointId_ Point id involved in the remove operation.
inline void ___stanity_exists(
    const fail_t& fail_,
    const cell_id_t& cellId_,
    const point_id_t& pointId_) {
  if (fail_)
    __throw__(
        "[Map::remove] Out of Sync. Failed to remove point " +
        std::string(pointId_) + " from cell " + std::string(cellId_) +
        ". Error message: " + fail_.get_message());
};

}  // namespace slam_mer

#endif  // REPRESENTATION_HPP
