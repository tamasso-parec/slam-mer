/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file representation.cpp
/// @brief Spatial representation implementation for map partitioning.
/// @details Implements cell-based partitioning and octree-related operations
/// declared in representation.hpp. Supports efficient spatial indexing
/// for map management.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <representation.hpp>

// Standard library includes
#include <print>
#include <string>

namespace slam_mer {

namespace {
static uint64_t __cellIdCounter = 0;
}

/// @brief Snaps a coordinate to the center of the nearest integer grid cell.
/// @param x_ Coordinate value in scaled internal units.
/// @param size_ Cell size used to quantize `x_` onto the grid.
/// @return Snapped center coordinate.
static inline int __snap_center(double x_, int size_) {
  // half-open intervals: [k*size - size/2, k*size + size/2)
  const double k = std::floor(
      (x_ + 0.5 * static_cast<double>(size_)) / static_cast<double>(size_));
  return static_cast<int>(k) * size_;
}

// ----------------------------------------------------------------------------
// Returns the corners of the cell.
// ----------------------------------------------------------------------------
array3Int_t::pair Representation::Voxel::get_corners() const {
  read_lock_t lock(_mutex);
  return {_cornerMin, _cornerMax};
}

// ----------------------------------------------------------------------------
// Returns the center of the cell.
// ----------------------------------------------------------------------------
array3D_t Representation::Voxel::get_center() const {
  read_lock_t lock(_mutex);
  return array3D_t{
      0.5 * (_cornerMin[0] + _cornerMax[0]),
      0.5 * (_cornerMin[1] + _cornerMax[1]),
      0.5 * (_cornerMin[2] + _cornerMax[2])};
}

// ----------------------------------------------------------------------------
// Checks whether a point is inside the cell.
// ----------------------------------------------------------------------------
bool Representation::Voxel::__contains(const point_id_t& pointId_) const {
  return _points.contains_(pointId_);
}
bool Representation::Voxel::contains(const point_id_t& pointId_) const {
  read_lock_t lock(_mutex);
  return __contains(pointId_);
}

// ----------------------------------------------------------------------------
// Checks whether a point is inside the cell (AABB containment check).
// ----------------------------------------------------------------------------
bool Representation::Voxel::contains(const array3D_t& point_) const {
  read_lock_t lock(_mutex);
  return (
      point_[0] >= _cornerMin[0] && point_[0] < _cornerMax[0] &&
      point_[1] >= _cornerMin[1] && point_[1] < _cornerMax[1] &&
      point_[2] >= _cornerMin[2] && point_[2] < _cornerMax[2]);
}

// ----------------------------------------------------------------------------
// Insert point into the cell
// ----------------------------------------------------------------------------
fail_t Representation::Voxel::insert(const point_id_t& pointId_) {
  write_lock_t lock(_mutex);
  _points.insert_(pointId_);
  return fail_t::NO_FAIL;
}

// ----------------------------------------------------------------------------
// Remove point from the cell
// ----------------------------------------------------------------------------
fail_t Representation::Voxel::remove(const point_id_t& pointId_) {
  write_lock_t lock(_mutex);

  // This function removes a point ID from the voxel's point set.
  // 1 - It first checks if the point ID exists in the set.
  // 2 - If it does not exist, it returns a NOT_FOUND fail_t error.
  // 3 - If it exists, it removes the point ID from the set.
  // 5 - Finally, it returns a NO_FAIL status.

  if (!__contains(pointId_))
    return fail_t(
        fail_t::NOT_FOUND,
        "Point ID " + std::string(pointId_) + " not found in cell ID " +
            std::string(_cellId) + ".");
  _points.erase_(pointId_);
  return fail_t::NO_FAIL;
}

// ----------------------------------------------------------------------------
// Representation ctor: validates and caches integer grid sizes.
// ----------------------------------------------------------------------------
Representation::Representation(
    const double voxelHalfSizeUnscaled_,
    const double coarseHalfSizeUnscaled_,
    const double scale_) {

  auto ___is_integer_like = [](double v__) -> bool {
    return std::abs(v__ - std::round(v__)) < 1e-9;
  };

  _scale = scale_;

  // This function:
  // 1 - Validates that the provided voxelHalfSize and coarseHalfSize
  //     can be represented as integers when scaled.
  // 2 - Ensures that the coarseHalfSize is an integer multiple of
  //     voxelHalfSize.
  // 3 - Sets the scaled integer sizes for efficient internal use.

  if (voxelHalfSizeUnscaled_ <= 0.0)
    __throw__("[3D Representation] Voxel half-size must be positive.");
  if (coarseHalfSizeUnscaled_ <= 0.0)
    __throw__("[3D Representation] Coarse half-size must be positive.");

  double voxelHalfSize = __scale(voxelHalfSizeUnscaled_);
  double coarseHalfSize = __scale(coarseHalfSizeUnscaled_);
  if (!___is_integer_like(voxelHalfSize))
    __throw__(
        "[3D Representation] Voxel half-size must map to an integer. Got " +
        std::to_string(voxelHalfSize) + ", for scale value " +
        std::to_string(_scale));
  if (!___is_integer_like(coarseHalfSize))
    __throw__(
        "[3D Representation] Coarse half-size must map to an integer. Got " +
        std::to_string(coarseHalfSize) + ", for scale value " +
        std::to_string(_scale));
  _voxelHalfSize = static_cast<int>(std::round(voxelHalfSize));
  _coarseHalfSize = static_cast<int>(std::round(coarseHalfSize));

  if ((_coarseHalfSize % _voxelHalfSize) != 0)
    __throw__(
        "[3D Representation] Coarse size must contain an integer number of "
        "voxels per axis.");
}

// ----------------------------------------------------------------------------
// Plots and histogram of coarse cells and their fine cell counts.
// ----------------------------------------------------------------------------
void Representation::__print_coarse_cells_histogram() {
  using std::println;

  println("[Representation::get_cells] {}info{}: Histogram:", green, reset);
  _coarseRepresentation.lock_read();
  for (const auto& [coarseCenter, cellIds] : _coarseRepresentation) {
    const array3D_t descaledCoarseCenter = __descale(coarseCenter);
    const std::size_t cellCount = cellIds.size();
    const std::size_t barLength =
        (cellCount == 0)
            ? 0
            : static_cast<std::size_t>(
                  std::log(static_cast<double>(cellCount)) / std::log(5.0)) +
                  1;
    const std::string bar(barLength, '*');
    println(
        "({:>3}, {:>3}, {:>3}) | {} ({})",
        static_cast<int>(std::round(descaledCoarseCenter[0])),
        static_cast<int>(std::round(descaledCoarseCenter[1])),
        static_cast<int>(std::round(descaledCoarseCenter[2])),
        bar,
        cellCount);
  }
  _coarseRepresentation.unlock_read();
}

// ----------------------------------------------------------------------------
// Check if cellId exists
// ----------------------------------------------------------------------------
bool Representation::exists(const cell_id_t& cellId_) const {
  read_lock_t lock(_mutex);
  return _allCells.contains_(cellId_);
}

// ----------------------------------------------------------------------------
// Get the cells corresponding to a given camera location.
// Avoid shared->unique lock upgrade deadlock by double-checked locking.
// ----------------------------------------------------------------------------

cell_id_t::vector Representation::get_cells(const array3D_t& pointUnscaled_) {
  using std::println;

  // This function:
  // 1. Computes the coarse center for the given point.
  // 2. Checks if the coarseRepresentation already contains this center.
  // 3. If it exists, retrieves the list of voxel cell IDs associated with
  //    that coarse center.
  // 4. If it does not exist, creates a new entry in coarseRepresentation
  //    with an empty list of voxel cell IDs.
  // 5. Additionally, it also checks the 26 neighboring coarse centers to
  //    include their voxel cell IDs as well.

  // Move the point to the internal scaled representation
  const array3D_t point = __scale(pointUnscaled_);
  const array3Int_t coarseCenter = __compute_coarse_center(point);

  write_lock_t lock(_mutex);
  _coarseRepresentation.try_emplace_(coarseCenter, cell_id_t::vector{});

  const int coarseStep = _coarseHalfSize * 2;
  cell_id_t::vector output;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const array3Int_t neighborCoarseCenter{
            coarseCenter[0] + dx * coarseStep,
            coarseCenter[1] + dy * coarseStep,
            coarseCenter[2] + dz * coarseStep};

        _coarseRepresentation.lock_read();
        const auto it = _coarseRepresentation.find(neighborCoarseCenter);
        if (it == _coarseRepresentation.end()) {
          _coarseRepresentation.unlock_read();
          continue;
        }
        const auto& neighborCells = it->second;
        const auto& begin = neighborCells.begin();
        const auto& end = neighborCells.end();
        output.insert(output.end(), begin, end);

#ifdef LOG_REPRESENTATION
        const size_t outputBefore = output.size();
        const size_t pickedCells = neighborCells.size();
        const size_t pushedToOutput = output.size() - outputBefore;
        const array3D_t descaledCoarseCenter = __descale(neighborCoarseCenter);
        println(
            "[Representation::get_cells] {}info{}: Coarse cell "
            "( {}, {}, {} ) picked cells={}, pushed to output={}",
            green,
            reset,
            descaledCoarseCenter[0],
            descaledCoarseCenter[1],
            descaledCoarseCenter[2],
            pickedCells,
            pushedToOutput);
#endif
        _coarseRepresentation.unlock_read();
      }
    }
  }

#ifdef LOG_REPRESENTATION
  const array3D_t coarseCenterDescaled = __descale(coarseCenter);
  println(
      "[Representation::get_cells] {}info{}: total picked/pushed entries={}"
      " for coarse center ( {}, {}, {} )",
      green,
      reset,
      output.size(),
      coarseCenterDescaled[0],
      coarseCenterDescaled[1],
      coarseCenterDescaled[2]);

  __print_coarse_cells_histogram();
#endif

  return output;
}

cell_id_t Representation::get_cell_id(const array3D_t& pointUnscaled_) {

  // This function:
  // 1. Computes the voxel center for the given point.
  // 2. Checks if a voxel already exists at that center in the
  //    fineRepresentation.
  // 3. If it exists, retrieves the corresponding Voxel object.
  // 4. If it does not exist, creates a new Voxel, assigns it a unique cell ID,
  //    and stores it in both fineRepresentation and allCells.
  // 5. Finally, returns the cell ID of the Voxel containing the point.

  // Move the point to the internal scaled representation
  const array3D_t point = __scale(pointUnscaled_);
  const array3Int_t voxelCenter = __compute_voxel_center(point);

  write_lock_t lock(_mutex);
  if (_fineRepresentation.contains_(voxelCenter)) {
    const cell_id_t cellId = _fineRepresentation.at_(voxelCenter);
    if (__sanity_exists(cellId))
      __throw__("[3D Representation] Inconsistent state: cellId not found.");
    return cellId;
  } else {
    const auto [cellId, newVoxelPtr] = __create_voxel(voxelCenter);
    _fineRepresentation.insert_({voxelCenter, cellId});
    _allCells.insert_({cellId, newVoxelPtr});

    const array3Int_t coarseCenter = __compute_coarse_center(point);
    if (!__coarse_exists(coarseCenter))
      _coarseRepresentation.insert_({coarseCenter, cell_id_t::vector{}});
    _coarseRepresentation[coarseCenter].push_back(cellId);
    return cellId;
  }
}

// ---------------------------------------------------------------------------
// Get cell id for a given point id
// ----------------------------------------------------------------------------
cell_id_t Representation::get_cell_id(const point_id_t& pointId_) const {
  read_lock_t lock(_mutex);
  if (__sanity_exists(pointId_))
    __throw__("[3D Representation] Point ID not found in pointToCellMap.");
  return _pointToCellMap.at_(pointId_);
}

/// ---------------------------------------------------------------------------
// Get the corners of a given cell
// ----------------------------------------------------------------------------
array3D_t::pair Representation::__get_cell_corners(
    const cell_id_t& cellId_) const {

  // This function:
  // 1 - Sanity check: verifies that the cellId exists in allCells.
  // 2 - Retrieves the Voxel object correspondin` to the cellId.
  // 3 - Gets the corners of the Voxel in scaled integer representation.
  // 4 - Descales the corners back to the original floating-point.
  // 5 - Returns the descales corners as a pair of array3D_t objects

  if (__sanity_exists(cellId_))
    __throw__("[3D Representation] Cell ID not found in allCells.");
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId_);
  const array3Int_t::pair corners = voxelPtr->get_corners();
  const array3D_t descaleMin = __descale(corners.first);
  const array3D_t descaleMax = __descale(corners.second);
  return array3D_t::pair{descaleMin, descaleMax};
}

// ----------------------------------------------------------------------------
// Get all points in a given cell
// ----------------------------------------------------------------------------
output_t<point_id_t::set> Representation::get_points(const cell_id_t& cellId_) {

  // This function:
  // 1 - Sanity check: verifies that the cellId exists in allCells.
  // 2 - Retrieves the Voxel object corresponding to the cellId.
  // 3 - Gets all points contained in the Voxel.

  read_lock_t lock(_mutex);
  if (__sanity_exists(cellId_)) return __sanity_exists(cellId_);
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId_);
  return voxelPtr->get_points();
}

// ----------------------------------------------------------------------------
// Insert a point into the representation
// ----------------------------------------------------------------------------
fail_t Representation::insert(
    const cell_id_t& cellId_,
    const point_id_t& pointId_) {

  // This function:
  // 1 - Sanity check: verifies that the cellId exists in allCells.
  // 2 - Retrieves the Voxel object corresponding to the cellId.
  // 3 - Calls the insert method of the Voxel to add the pointId.
  // 4 - If insertion is successful, updates the pointToCellMap.

  write_lock_t lock(_mutex);
  if (__sanity_exists(cellId_)) return __sanity_exists(cellId_);
  if (__sanity_not_exists(pointId_)) return __sanity_not_exists(pointId_);
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId_);
  const fail_t error = voxelPtr->insert(pointId_);
  if (!error) _pointToCellMap.insert_({pointId_, cellId_});
  return error;
}

// ----------------------------------------------------------------------------
// Remove a point from the representation
// ----------------------------------------------------------------------------
fail_t Representation::remove(
    const cell_id_t& cellId_,
    const point_id_t& pointId_) {

  // This function:
  // 1 - Sanity check: verifies that the cellId exists in allCells.
  // 2 - Sanity check: verifies that the pointId exists in pointToCellMap.
  // 3 - Retrieves the Voxel object corresponding to the cellId.
  // 4 - Calls the remove method of the Voxel to remove the pointId.

  write_lock_t lock(_mutex);
  if (__sanity_exists(cellId_)) return __sanity_exists(cellId_);
  if (__sanity_exists(pointId_)) return __sanity_exists(pointId_);
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId_);
  const fail_t error = voxelPtr->remove(pointId_);
  if (!error) _pointToCellMap.erase_(pointId_);
  return error;
}

// ----------------------------------------------------------------------------
// Remove a point from the representation by point id only
// ----------------------------------------------------------------------------
fail_t Representation::remove(const point_id_t& pointId_) {
  write_lock_t lock(_mutex);
  if (__sanity_exists(pointId_)) return __sanity_exists(pointId_);
  const cell_id_t cellId = _pointToCellMap.at_(pointId_);
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId);
  const fail_t error = voxelPtr->remove(pointId_);
  if (!error) _pointToCellMap.erase_(pointId_);
  return error;
}

// ---------------------------------------------------------------------------
// Check if a point is contained in a given cell
// ----------------------------------------------------------------------------
output_t<bool> Representation::contains(
    const cell_id_t& cellId_,
    const array3D_t& point_) const {

  // This function:
  // 1 - Sanity check: verifies that the cellId exists in allCells.
  // 2 - Retrieves the Voxel object corresponding to the cellId.
  // 3 - Computes the voxel center for the given point.
  // 4 - Checks if the voxel center matches the center of the Voxel.

  read_lock_t lock(_mutex);
  if (__sanity_exists(cellId_))
    return fail_t(
        fail_t::NOT_FOUND,
        "[3D Representation] Cell ID " + std::string(cellId_) +
            " does not exist.");
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId_);
  const array3D_t scaledPoint = __scale(point_);
  return voxelPtr->contains(scaledPoint);
}

// ----------------------------------------------------------------------------
// Switch a point from one cell to another
// ----------------------------------------------------------------------------
fail_t Representation::switch_cell(
    const cell_id_t& oldCellId_,
    const cell_id_t& newCellId_,
    const point_id_t& pointId_) {

  // This function:
  // 1 - Sanity check: verifies that both oldCellId and newCellId exist
  //     in allCells.
  // 2 - Sanity check: verifies that pointId exists in pointToCellMap.
  // 3 - Retrieves the Voxel objects corresponding to both cell IDs.
  // 4 - Removes the pointId from the old Voxel and inserts it into the new
  //     Voxel.

  write_lock_t lock(_mutex);
  if (__sanity_exists(oldCellId_)) return __sanity_exists(oldCellId_);
  if (__sanity_exists(newCellId_)) return __sanity_exists(newCellId_);
  if (__sanity_exists(pointId_)) return __sanity_exists(pointId_);
  const Voxel::shared_ptr oldVoxelPtr = _allCells.at_(oldCellId_);
  const Voxel::shared_ptr newVoxelPtr = _allCells.at_(newCellId_);
  const fail_t fail = oldVoxelPtr->remove(pointId_);
  if (fail) return fail;
  newVoxelPtr->insert(pointId_);
  _pointToCellMap[pointId_] = newCellId_;

  return fail_t::NO_FAIL;
}

// ---------------------------------------------------------------------------
// Get the corners of a voxel
// ----------------------------------------------------------------------------
array3D_t::pair Representation::get_corners(const cell_id_t& cellId_) const {
  read_lock_t lock(_mutex);
  if (__sanity_exists(cellId_))
    __throw__("[3D Representation] Cell ID not found in allCells.");
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId_);
  const array3Int_t::pair corners = voxelPtr->get_corners();
  const array3D_t descaleMin = __descale(corners.first);
  const array3D_t descaleMax = __descale(corners.second);
  return array3D_t::pair{descaleMin, descaleMax};
}

// ----------------------------------------------------------------------------
// Highlight a cell
// ----------------------------------------------------------------------------
void Representation::highlight(const cell_id_t& cellId_) {
  read_lock_t lock(_mutex);
  if (__sanity_exists(cellId_))
    __throw__("[3D Representation] Cell ID not found in allCells.");
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId_);
  voxelPtr->highlight();
}

// ----------------------------------------------------------------------------
// Unhighlight a cell
// ----------------------------------------------------------------------------
void Representation::unhighlight(const cell_id_t& cellId_) {
  read_lock_t lock(_mutex);
  if (__sanity_exists(cellId_))
    __throw__("[3D Representation] Cell ID not found in allCells.");
  const Voxel::shared_ptr voxelPtr = _allCells.at_(cellId_);
  voxelPtr->unhighlight();
}

// ----------------------------------------------------------------------------
// Sort cells based on camera pose
// ----------------------------------------------------------------------------
void Representation::sort_cells(
    const pose_t& cameraToWorldTransformation_,
    const intrinsics_t& intrinsics_) {

  // -------------------------------------------------------------------------
  // Lambda function to compute squared distance between two 3D points
  auto ___square_distance = [](const array3D_t& a__,
                               const array3D_t& b__) -> double {
    return (a__[0] - b__[0]) * (a__[0] - b__[0]) +
           (a__[1] - b__[1]) * (a__[1] - b__[1]) +
           (a__[2] - b__[2]) * (a__[2] - b__[2]);
  };
  // -------------------------------------------------------------------------

  // Compute the camera location
  const pose_t worldToCameraTransformation =
      cameraToWorldTransformation_.get_invert();
  const auto [rotation, translation] =
      worldToCameraTransformation.get_rotation_translation();
  const array3D_t cameraLocationArray =
      ((rotation.inverse() * translation) * (-1)).to_array();

  // Get the closest cells to the camera location
  cell_id_t::vector localOrderedCellIds = this->get_cells(cameraLocationArray);

  // Sort!
  // NO CELL WILL EVERY BE REMOVED. SO IT IS SAFE TO ACCESS THEM, IF THEY EXIST.
  // IF THEY EXIST, THEY WILL NOT BE REMOVED.

  std::sort(
      localOrderedCellIds.begin(),
      localOrderedCellIds.end(),
      [this,
       cameraLocationArray,
       worldToCameraTransformation,
       ___square_distance,
       intrinsics_](const cell_id_t a__, const cell_id_t b__) {
        if (__sanity_exists(a__)) return false;
        if (__sanity_exists(b__)) return true;
        const array3D_t& centroidA = __get_center(a__);
        const array3D_t& centroidB = __get_center(b__);
        const array3D_t& inCameraA = worldToCameraTransformation * centroidA;
        const array3D_t& inCameraB = worldToCameraTransformation * centroidB;
        const auto& [validA, pointImageA] = intrinsics_.project(inCameraA);
        const auto& [validB, pointImageB] = intrinsics_.project(inCameraB);
        if (!validA) return false;
        if (!validB) return true;
        return ___square_distance(cameraLocationArray, centroidA) <
               ___square_distance(cameraLocationArray, centroidB);
      });

  // Update the ordered cell ids list in the class. Only for this, we need to
  // lock the class mutex.

  _orderedCellIds.assign_(
      localOrderedCellIds.begin(),
      localOrderedCellIds.end());
}

// ----------------------------------------------------------------------------
// Check if cellId exists
// ----------------------------------------------------------------------------
fail_t Representation::__sanity_exists(const cell_id_t& cellId_) const {
  if (!_allCells.contains_(cellId_))
    return fail_t(
        fail_t::NOT_FOUND,
        "[3D Representation] Cell ID " + std::to_string(cellId_.key()) +
            " does not exist.");
  return fail_t::NO_FAIL;
}

// ----------------------------------------------------------------------------
// Check if cellId does not exist
// ----------------------------------------------------------------------------
fail_t Representation::__sanity_not_exists(const cell_id_t& cellId_) const {
  if (_allCells.contains_(cellId_))
    return fail_t(
        fail_t::ALREADY_EXISTS,
        "[3D Representation] Cell ID " + std::to_string(cellId_.key()) +
            " already exists.");
  return fail_t::NO_FAIL;
}

// ----------------------------------------------------------------------------
// Check if pointId exists
// ----------------------------------------------------------------------------
fail_t Representation::__sanity_exists(const point_id_t& pointId_) const {
  if (!_pointToCellMap.contains_(pointId_))
    return fail_t(
        fail_t::NOT_FOUND,
        "[3D Representation] Point ID " + std::to_string(pointId_.key()) +
            " does not exist.");
  return fail_t::NO_FAIL;
}

// ----------------------------------------------------------------------------
// Sanity do not exist
// ----------------------------------------------------------------------------
fail_t Representation::__sanity_not_exists(const point_id_t& pointId_) const {
  if (_pointToCellMap.contains_(pointId_))
    return fail_t(
        fail_t::ALREADY_EXISTS,
        "[3D Representation] Point ID " + std::to_string(pointId_.key()) +
            " already exists.");
  return fail_t::NO_FAIL;
}

// ----------------------------------------------------------------------------
// Check if coarse center exists
// ----------------------------------------------------------------------------
bool Representation::__coarse_exists(const array3Int_t& coarseCenter_) const {
  return _coarseRepresentation.contains_(coarseCenter_);
}

pair_t<cell_id_t, Representation::Voxel::shared_ptr>
Representation::__create_voxel(const array3Int_t& voxelCenter_) {
  // This function:
  // 1 - Computes the corners of the voxel based on its center and the
  //     voxelHalfSize.
  // 2 - Generates a unique cell ID for the new voxel.
  // 3 - Creates a new Voxel object with the computed corners and cell ID.
  // 4 - Returns the cell ID and shared pointer to the new Voxel.

  const int h = _voxelHalfSize;  // scaled units
  const array3Int_t cornerMin{
      voxelCenter_[0] - h,
      voxelCenter_[1] - h,
      voxelCenter_[2] - h};
  const array3Int_t cornerMax{
      voxelCenter_[0] + h,
      voxelCenter_[1] + h,
      voxelCenter_[2] + h};
  const cell_id_t cellId(__cellIdCounter++);
  return {cellId, Voxel::make_shared(cornerMin, cornerMax, cellId)};
}

// ----------------------------------------------------------------------------
// Compute snapped centers: voxel
// ----------------------------------------------------------------------------
array3Int_t Representation::__compute_voxel_center(
    const array3D_t& point_) const {
  const int voxelSize = 2 * _voxelHalfSize;
  return array3Int_t{
      __snap_center(point_[0], voxelSize),
      __snap_center(point_[1], voxelSize),
      __snap_center(point_[2], voxelSize)};
}

// ----------------------------------------------------------------------------
// Compute snapped centers: coarse
// ----------------------------------------------------------------------------
array3Int_t Representation::__compute_coarse_center(
    const array3D_t& point_) const {
  const int coarseSize = 2 * _coarseHalfSize;
  return array3Int_t{
      __snap_center(point_[0], coarseSize),
      __snap_center(point_[1], coarseSize),
      __snap_center(point_[2], coarseSize)};
}

}  // namespace slam_mer
