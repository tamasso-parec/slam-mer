/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file cells.cpp
/// @brief Implements cell-geometry buffering and rendering updates.
/// @details Defines cell add/update/hide logging routines for Rerun.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

// My libraries
#include <interface_rerun.hpp>
#include <settings.hpp>
#include <types/cell.hpp>
#include <types/hash.hpp>

// Third-party libraries
#include <rerun.hpp>

// Standard libraries

namespace slam_mer {
namespace interface {

namespace {
// Global variable to store the index of the cells on the interface
hash_t<cell_id_t, size_t> __cellIdToIndex;
/// Iterator for the next id of the cell on the interface
size_t __nextCellIndex = 0;

// Attributes needed for the visualization
// We are not using slam_mer types here to keep things consistant with
// rerun types.
/// Contains the ids of the cells (position of the text)
std::vector<rerun::components::Position3D> __textPosition;
/// Contains the ids of the cells (actual text)
std::vector<std::string> __idText;
/// Contains the cells center
std::vector<rerun::components::Position3D> __rerunCellsCenters;
/// Contains the cells' half sizes
std::vector<rerun::components::HalfSize3D> __rerunCellsHalfSizes;
/// Contains the cells' colors
std::vector<rerun::Color> __rerunCellscolors;
/// Contains the cells' fill mode
std::vector<rerun::components::FillMode> __rerunCellsFill;
/// radius of the cells lines
std::vector<float> __cellsRadii;

/// Mutex to protect access to the cell data
std::shared_mutex __cellData_mutex;

}  // namespace

// External variables that are used to store the
// points and their connections in the visualization interface.
// Check interface.cpp for the definition of these variables.
extern set_t<cell_id_t>::shared_ptr __newCells;
extern set_t<cell_id_t>::shared_ptr __updateCells;
extern std::function<slam_mer::cell3d_t(cell_id_t)> ___get_cell;

/// External variable to store the cell data for the visualization interface
extern std::shared_ptr<rerun::RecordingStream> __recording;

/// @brief Check whether a cell already has an allocated visualization slot.
/// @param cellId_ Cell identifier in map space.
/// @return `true` when the cell is already present in interface buffers.
bool __cell_exists(const cell_id_t& cellId_);
/// @brief Mark a cell as empty in visualization buffers.
/// @param cellId_ Cell identifier to update.
void __now_empty(const cell_id_t& cellId_);
/// @brief Mark a cell as highlighted in visualization buffers.
/// @param cellId_ Cell identifier to update.
void __now_highlighted(const cell_id_t& cellId_);
/// @brief Mark a cell as normal (non-highlighted, non-empty).
/// @param cellId_ Cell identifier to update.
void __now_normal(const cell_id_t& cellId_);
/// @brief Compute center and half-size vectors for a cell box.
/// @param cell3d_ Cell geometry in map coordinates.
/// @return Tuple `(center, halfSize)` as float arrays.
std::tuple<array3F_t, array3F_t> __get_center_hs(const cell3d_t& cell3d_);
/// @brief Append one new cell primitive to visualization buffers.
/// @param cellId_ Cell identifier to insert.
/// @param center_ Box center in world coordinates.
/// @param halfSize_ Box half-size in world coordinates.
void __add_new_cell(
    const cell_id_t& cellId_,
    const array3F_t& center_,
    const array3F_t& halfSize_);
/// @brief Append one cell label primitive to visualization buffers.
/// @param cellId_ Cell identifier used in label text.
/// @param center_ Label anchor position.
void __add_cell_id_text(const cell_id_t& cellId_, const array3F_t& center_);

/// @brief Synchronize cell add/update queues into interface-local cell buffers.
void AbstractInterface::log_cells() {

  // We start with adding the new cells
  // 1 - Pull the new cells from the map
  // 2 - For each new cell, get its center and half size
  // 3 - Add the new cell to the rerun visualization
  cell_id_t::vector cellsToAdd = __newCells->to_vector();
  cell_id_t::vector addedCells;
  for (const cell_id_t& cellId : cellsToAdd) {
    addedCells.emplace_back(cellId);
    if (__cell_exists(cellId)) {
      __updateCells->insert_(cellId);
      continue;
    }
    const cell3d_t cell3d = ___get_cell(cellId);
    auto [center, half_size] = __get_center_hs(cell3d);
    __add_new_cell(cellId, center, half_size);
    // __add_cell_id_text(cellId, center);
  }
  __newCells->clear_(addedCells);

  // Now we update the existing cells
  // 1 - Pull the cells to update from the map
  // 2 - For each cell, check its status (highlighted, empty, normal)
  // 3 - Update the cell color and fill mode accordingly
  cell_id_t::vector cellsToUpdate = __updateCells->to_vector();
  cell_id_t::vector updatedCells;
  for (const cell_id_t& cellId : cellsToUpdate) {
    if (!__cell_exists(cellId)) continue;
    const slam_mer::cell3d_t cell3d(___get_cell(cellId));
    const bool isHighlighted = cell3d.is_highlighted();
    const bool isEmpty = cell3d.get_point_count() == 0;
    if (isEmpty) {
      __now_empty(cellId);
      continue;
    }
    if (isHighlighted)
      __now_highlighted(cellId);
    else
      __now_normal(cellId);
    updatedCells.emplace_back(cellId);
  }
  __updateCells->clear_(updatedCells);
}

/// @brief Publish cell boxes and labels to the Rerun stream.
void InterfaceRerun::__log_cells() {
  using rerun::Collection;
  using rerun::ComponentColumn;
  using rerun::TimeColumn;
  using rerun::archetypes::Boxes3D;
  using rerun::archetypes::Points3D;

  read_lock_t lock(__cellData_mutex);
  if (__rerunCellsHalfSizes.empty()) return;
  const auto boxes = Boxes3D::from_centers_and_half_sizes(
                         __rerunCellsCenters,
                         __rerunCellsHalfSizes)
                         .with_colors(__rerunCellscolors)
                         .with_many_fill_mode(__rerunCellsFill)
                         .with_radii(__cellsRadii);
#if 0
  const uint32_t N = static_cast<uint32_t>(__rerunCellsHalfSizes.size());
  const Collection<ComponentColumn> comp_cols = boxes.columns({N});
  Collection<TimeColumn> time_cols;
  rec_global->send_columns("world/root/cells", time_cols, comp_cols);
#else
#ifdef DEBUG
  __recording->log("world/root/cells", boxes);
#endif
#ifdef SHOW_DENSE
  __recording->log(
      "world/root/cell_ids",
      Points3D(__textPosition).with_labels(__idText).with_show_labels(true));
#endif
#endif
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// @brief Check whether a cell already has an allocated visualization slot.
/// @param cellId_ Cell identifier in map space.
/// @return `true` when the cell is already present in interface buffers.
bool __cell_exists(const cell_id_t& cellId_) {
  return __cellIdToIndex.contains_(cellId_);
}

/// @brief Mark a cell as empty in visualization buffers.
/// @param cellId_ Cell identifier to update.
void __now_empty(const cell_id_t& cellId_) {
  using rerun::Color;
  using rerun::components::FillMode;

  write_lock_t lock(__cellData_mutex);
  const size_t index = __cellIdToIndex[cellId_];
  __rerunCellscolors[index] = Color(0, 0, 0, 0);
  __rerunCellsFill[index] = FillMode::MajorWireframe;
  __cellsRadii[index] = 0.0f;
}

/// @brief Mark a cell as highlighted in visualization buffers.
/// @param cellId_ Cell identifier to update.
void __now_highlighted(const cell_id_t& cellId_) {
  using rerun::Color;
  using rerun::components::FillMode;
  using slam_mer::settings::sharedSettings;

  write_lock_t lock(__cellData_mutex);
  const size_t index = __cellIdToIndex[cellId_];
  const auto color = sharedSettings->interface_cell_highlighted_color();
  const Color cellColor(color[0], color[1], color[2], color[3]);
  __rerunCellscolors[index] = cellColor;
  __rerunCellsFill[index] = FillMode::MajorWireframe;
  __cellsRadii[index] = sharedSettings->interface_cell_highlighted_thickness();
}

/// @brief Mark a cell as normal (non-highlighted, non-empty).
/// @param cellId_ Cell identifier to update.
void __now_normal(const cell_id_t& cellId_) {
  using rerun::Color;
  using rerun::components::FillMode;
  using slam_mer::settings::sharedSettings;

  write_lock_t lock(__cellData_mutex);
  const size_t index = __cellIdToIndex[cellId_];
  const auto color = sharedSettings->interface_cell_color();
  const Color cellColor(color[0], color[1], color[2], color[3]);
  __rerunCellscolors[index] = cellColor;
  __rerunCellsFill[index] = FillMode::MajorWireframe;
  __cellsRadii[index] = sharedSettings->interface_cell_thickness();
}

/// @brief Compute center and half-size vectors for a cell box.
/// @param cell3d_ Cell geometry in map coordinates.
/// @return Tuple `(center, halfSize)` as float arrays.
std::tuple<array3F_t, array3F_t> __get_center_hs(const cell3d_t& cell3d_) {
  const array3D_t& point1 = cell3d_.get_min_bound();
  const array3D_t& point2 = cell3d_.get_max_bound();
  array3F_t center;
  array3F_t half_size;
  for (int i = 0; i < 3; ++i) {
    center[i] = static_cast<float>((point1[i] + point2[i]) / 2.0);
    half_size[i] = static_cast<float>(std::abs(point1[i] - point2[i]) / 2.0);
  }
  return {center, half_size};
}

/// @brief Append one new cell primitive to visualization buffers.
/// @param cellId_ Cell identifier to insert.
/// @param center_ Box center in world coordinates.
/// @param halfSize_ Box half-size in world coordinates.
void __add_new_cell(
    const cell_id_t& cellId_,
    const array3F_t& center_,
    const array3F_t& halfSize_) {
  using rerun::Color;
  using slam_mer::settings::sharedSettings;

  write_lock_t lock(__cellData_mutex);
  __rerunCellsCenters.push_back(center_);
  __rerunCellsHalfSizes.push_back(halfSize_);
  const auto color = sharedSettings->interface_cell_color();
  const Color cellColor(color[0], color[1], color[2], color[3]);
  __rerunCellscolors.push_back(cellColor);
  __rerunCellsFill.push_back(rerun::components::FillMode::MajorWireframe);
  __cellsRadii.push_back(sharedSettings->interface_cell_thickness());
  __cellIdToIndex[cellId_] = __nextCellIndex++;
}

/// @brief Append one cell label primitive to visualization buffers.
/// @param cellId_ Cell identifier used in label text.
/// @param center_ Label anchor position.
void __add_cell_id_text(const cell_id_t& cellId_, const array3F_t& center_) {
  write_lock_t lock(__cellData_mutex);
  __textPosition.push_back({center_[0], center_[1], center_[2]});
  __idText.push_back("Cell " + std::string(cellId_));
}

}  // namespace interface
}  // namespace slam_mer
