/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file map.hpp
/// @brief Main 3D map interface for _keyframes, _points, cells, and queries.
/// @details Declares `slam_mer::map::Map`, the central map API used by the
/// SLAM pipeline. It provides keyframe_/point_ insertion and retrieval,
/// covisibility and neighborhood operations, map optimization integration,
/// and visualization synchronization hooks.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef map_HPP
#define map_HPP

// Our includes
#include <keyframe.hpp>
#include <log.hpp>
#include <representation.hpp>
#include <types/array.hpp>
#include <types/camera.hpp>
#include <types/cell.hpp>
#include <types/dictionary.hpp>
#include <types/hash.hpp>
#include <types/id.hpp>
#include <types/mutex.hpp>
#include <types/output.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>
#include <types/time_stamp.hpp>
#include <types/timer.hpp>
#include <types/vector.hpp>
#include <types/visualization.hpp>
#include <wrapper_visualization.hpp>

// Standard C++ libraries
#include <shared_mutex>

/// @brief Core map module namespace.
/// @details Contains map storage, accessors, and synchronization utilities
/// used by the SLAM pipeline to manage keyframes, points, and cells.
namespace slam_mer {
namespace map {

/// @brief Thread-safe 3D map representation for SLAM state.
/// @details Owns keyframes, points, cell indexing, and helper operations for
/// insertion, updates, retrieval, and visualization buffering.
class Map {

public:
  /// Shared pointer type for Map.
  using shared_ptr = std::shared_ptr<Map>;
  /// Weak pointer type for Map.
  using weak_ptr = std::weak_ptr<Map>;

  /// @brief Construct a shared map instance and start internal worker threads.
  /// @return Shared pointer to initialized Map instance.
  [[nodiscard("Use returned shared_ptr to keep map ownership")]]
  static Map::shared_ptr make_shared() {
    Map::shared_ptr map(new Map());
    map->__start_threads();
    return map;
  }

  /// @brief Acquire shared lock on the map-level mutex.
  void shared_lock() const { _mutex.lock_shared(); }

  /// @brief Release shared lock on the map-level mutex.
  void unlock_shared() const { _mutex.unlock_shared(); }

  /// @brief Acquire exclusive lock on the map-level mutex.
  void lock() const { _mutex.lock(); }

  /// @brief Release exclusive lock on the map-level mutex.
  void unlock() const { _mutex.unlock(); }

  /// @brief Destroy map instance and stop internal resources.
  ~Map();

  /// @brief Get current map cardinality counters.
  /// @return Tuple `(numKeyframes, numPoints, numberQueryCells,
  /// numberClosestCells, totalNumberCells)`.
  [[nodiscard("Use returned tuple to consume current map size counters")]]
  inline std::tuple<size_t, size_t, size_t, size_t, size_t> size() const {
    __read_lock_points();
    __read_lock_keyframes();
    const auto [numKeyframes, numMappoints] = __size();
    __read_unlock_keyframes();
    __read_unlock_points();
    const auto [query, closest, total] = __get_number_cells();
    return std::make_tuple(numKeyframes, numMappoints, query, closest, total);
  }

  /// @brief Check whether a keyframe ID exists in the map.
  /// @param keyframeId_ Keyframe identifier to query.
  /// @return `true` if the keyframe exists, otherwise `false`.
  [[nodiscard("Use returned bool to branch on keyframe existence")]]
  inline bool exists(const keyframe_id_t& keyframeId_) const {
    if (__exists(keyframeId_).fail()) return false;
    return true;
  }

  /// @brief Check whether a point ID exists in the map.
  /// @param pointId_ Point identifier to query.
  /// @return `true` if the point exists, otherwise `false`.
  [[nodiscard("Use returned bool to branch on point existence")]]
  inline bool exists(const point_id_t& pointId_) const {
    __read_lock_points();
    __sanity_sync(pointId_);
    const bool output = _points.contains(pointId_);
    __read_unlock_points();
    return output;
  }

  /// @brief Check whether a cell ID exists in the map.
  /// @param cellId_ Cell identifier to query.
  /// @return `true` if the cell exists, otherwise `false`.
  [[nodiscard("Use returned bool to branch on cell existence")]]
  bool exists(const cell_id_t& cellId_) const;

  /// @brief Retrieve a keyframe pointer by keyframe ID.
  /// @param keyframeId_ Keyframe identifier.
  /// @return Weak pointer wrapped in output_t on success.
  [[deprecated("Use at() or get() for keyframe retrieval instead")]]
  inline const output_t<keyframe_t::weak_ptr> operator[](
      const keyframe_id_t& keyframeId_) const {
    return this->at(keyframeId_);
  }

  /// @brief Retrieve a keyframe pointer by keyframe ID.
  /// @param keyframeId_ Keyframe identifier.
  /// @return Weak pointer wrapped in output_t on success.
  [[nodiscard("Use returned keyframe output to check retrieval success")]]
  inline output_t<keyframe_t::weak_ptr> at(
      const keyframe_id_t& keyframeId_) const {

    // Simple function to output a keyframe_ given its ID (non-blocking)
    // 1 - Check if the keyframe_ exists
    // 2 - If it exists, output the keyframe_
    // 3 - If it does not exist, output an error
    __read_lock_keyframes();
    __sanity_exists(keyframeId_);
    keyframe_t::weak_ptr output = _keyframes.at(keyframeId_);
    __read_unlock_keyframes();
    return output;
  }

  /// @brief Retrieve pose value associated with a keyframe ID.
  /// @param keyframeId_ Keyframe identifier.
  /// @return Pose wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned pose output to check retrieval success")]]
  inline output_t<pose_t> get(const keyframe_id_t& keyframeId_) const {
    output_t<keyframe_t::weak_ptr> keyframe_ = this->at(keyframeId_);
    if (keyframe_.fail()) return fail_t(keyframe_.get_error());
    pose_t::weak_ptr posePtr_ = keyframe_t::get_pose(keyframe_);
    return pose_t(posePtr_);
  }

  /// @brief Retrieve a map point pointer by point ID.
  /// @param pointId_ Point identifier.
  /// @return Weak pointer wrapped in output_t, or a fail_t error.
  [[deprecated("Use at() or get() instead")]]
  inline output_t<point3d_t::weak_ptr> operator[](
      const point_id_t& pointId_) const {
    return this->at(pointId_);
  }

  /// @brief Retrieve a map point pointer by point ID.
  /// @param pointId_ Point identifier.
  /// @return Weak pointer wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned point output to check retrieval success")]]
  inline output_t<point3d_t::weak_ptr> at(const point_id_t pointId_) const {

    // Simple function to output a 3D point_ given its ID (non-blocking)
    // 1 - Check if the point_ exists
    // 2 - If it exists, output the point_
    // 3 - If it does not exist, output an error

    __read_lock_points();
    __sanity_sync(pointId_);
    if (!_points.contains(pointId_)) {
      __read_unlock_points();
      return fail_t(fail_t::NOT_FOUND, "Point not found in the map");
    }
    point3d_t::weak_ptr output = _points.at(pointId_);
    __read_unlock_points();
    return output;
  }

  /// @brief Retrieve map point value associated with a point ID.
  /// @param pointId_ Point identifier.
  /// @return Point value wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned point output to check retrieval success")]]
  inline output_t<point3d_t> get(const point_id_t& pointId_) const {
    output_t<point3d_t::weak_ptr> pointPtr_ = this->at(pointId_);
    if (pointPtr_.fail()) return fail_t(pointPtr_.get_error());
    return point3d_t(pointPtr_);
  }

  /// @brief Insert a new 3D point into the map.
  /// @param point_ Point data to insert.
  /// @return Inserted `(pointId, cellId)` wrapped in output_t.
  [[nodiscard("WARNING: Not using returned point and cell IDs")]]
  output_t<pair_t<point_id_t, cell_id_t>> insert(const point3d_t& point_);

  /// @brief Insert a keyframe into the map.
  /// @param pose_ Pose associated with the keyframe.
  /// @param keyframeData_ Keyframe payload to insert.
  /// @return Inserted keyframe ID wrapped in output_t.
  [[nodiscard("WARNING: Not using returned keyframe ID")]]
  output_t<keyframe_id_t> insert(
      const pose_t& pose_,
      const keyframe_t& keyframeData_);

  /// @brief Update point coordinates using a point object.
  /// @param pointId_ Point identifier to update.
  /// @param point_ Point object containing the updated position.
  /// @return fail_t::NO_FAIL on success, otherwise a fail_t error.
  [[nodiscard("WARNING: Check returned fail_t to verify point update success")]]
  inline fail_t update(const point_id_t& pointId_, const point3d_t& point_) {
    const array3D_t pointArray = point_;
    return this->update(pointId_, pointArray);
  }

  /// @brief Update point coordinates using a weak point pointer.
  /// @param pointId_ Point identifier to update.
  /// @param pointPtr_ Weak pointer to point data.
  /// @return fail_t::NO_FAIL on success, otherwise a fail_t error.
  [[nodiscard("WARNING: Check returned fail_t to verify point update success")]]
  inline fail_t update(
      const point_id_t& pointId_,
      const point3d_t::weak_ptr& pointPtr_) {

    const point3d_t point_(pointPtr_);
    if (point_.is_unset()) return fail_t(fail_t::LOST_PTR);
    const array3D_t pointArray = point_;
    return this->update(pointId_, pointArray);
  }

  /// @brief Update point coordinates using a 3D array value.
  /// @param pointId_ Point identifier to update.
  /// @param point_ New point coordinates.
  /// @return fail_t::NO_FAIL on success, otherwise a fail_t error.
  [[nodiscard("WARNING: Check returned fail_t to verify point update success")]]
  fail_t update(const point_id_t& pointId_, const array3D_t& point_);

  /// @brief Update keyframe pose using a weak pose pointer.
  /// @param keyframeId_ Keyframe identifier to update.
  /// @param posePtr_ Weak pointer to the new pose.
  /// @return fail_t::NO_FAIL on success, otherwise a fail_t error.
  [[nodiscard("WARNING: Check returned fail_t to verify keyframe pose update")]]
  inline fail_t update(
      const keyframe_id_t& keyframeId_,
      const pose_t::weak_ptr& posePtr_) {
    return this->update(keyframeId_, pose_t(posePtr_));
  }

  /// @brief Update keyframe pose using a pose value.
  /// @param keyframeId_ Keyframe identifier to update.
  /// @param pose_ New pose value.
  /// @return fail_t::NO_FAIL on success, otherwise a fail_t error.
  [[nodiscard("WARNING: Check returned fail_t to verify keyframe pose update")]]
  fail_t update(const keyframe_id_t& keyframeId_, const pose_t& pose_);

  /// @brief Remove a point from the map.
  /// @param pointId_ Point identifier to remove.
  /// @return fail_t::NO_FAIL on success, otherwise a fail_t error.
  [[nodiscard("WARNING: Check returned fail_t to verify point removal")]]
  fail_t remove(const point_id_t& pointId_);

  /// @brief Get timestamp and pose for every keyframe currently in the map.
  /// @return Map of keyframe IDs to pose and timestamp pairs.
  [[nodiscard("Returns map to inspect keyframe timeline and estimated poses")]]
  inline dictionary_t<keyframe_id_t, pair_t<pose_t, time_stamp_t>>
  get_timestamp_and_poses() const;

  /// @brief Get keyframe data pointer for a keyframe ID.
  /// @param keyframeId_ Keyframe identifier.
  /// @return Weak pointer to keyframe data.
  [[nodiscard("Use returned keyframe pointer for follow-up map operations")]]
  inline keyframe_t::weak_ptr get_keyframe_data_x(
      const keyframe_id_t keyframeId_) const {
    if (!this->exists(keyframeId_))
      __throw__(
          "In get_keyframe_data_x, Keyframe with ID " +
          std::string(keyframeId_) + " does not exist in the map.");
    read_lock_t lock(_mutex);
    return _keyframes.at(keyframeId_);
  }

  /// @brief Get point neighbors for a point ID.
  /// @param pointId_ Point identifier.
  /// @return Const reference to neighbor list for the input point.
  [[nodiscard("Use returned neighbor list for local map connectivity checks")]]
  inline const neighbor_t::vector& get_mappoints_neighboors_x(
      point_id_t pointId_) const {
    read_lock_t lock(_mutex);
    return __get_mappoints_neighboors_x(pointId_);
  }

  /// @brief Get all point IDs that belong to a specific 3D cell.
  /// @param cellId_ Cell identifier.
  /// @return Vector of point IDs contained in the cell.
  [[nodiscard("Use returned point IDs to iterate cell contents")]]
  inline point_id_t::vector get_points_in_cell_x(
      const cell_id_t cellId_) const {
    read_lock_t lock(_mutex);
    return __get_points_in_cell_x(cellId_);
  }

  /// @brief Get current camera world pose pointer.
  /// @return Weak pointer to camera pose in world coordinates.
  [[nodiscard("Use returned pose pointer to read current camera state")]]
  inline pose_t::weak_ptr get_camera_world_pose() const {
    read_lock_t lock(_mutex);
    return _cameraPose;
  }

  /// @brief Sets the current estimated camera pose_.
  /// @param rotation_ Rotation of the camera.
  /// @param translation_ Translation of the camera.
  inline void set_camera_world_pose(
      const rotation_t& rotation_,
      const translation_t& translation_) {
    write_lock_t lock(_mutex);
    _cameraPose->set_pose(rotation_, translation_);
  }

  /// @brief Sets the current estimated camera pose_.
  /// @param newPose_ New current camera pose_ in world coordinates.
  inline void set_camera_world_pose(const pose_t& newPose_) {
    write_lock_t lock(_mutex);
    _cameraPose = pose_t::make_shared(newPose_);
  }

  /// @brief Add a covisibility edge between two keyframes.
  /// @param pair_ Pair of keyframe IDs that define the edge.
  inline void add_new_covisibility_connections_map(keyframe_id_t::pair pair_) {
    using map::add_new_covisibility_connections;
    add_new_covisibility_connections(pair_.first, pair_.second);
  }

  /// @brief Compute visible cells for the current camera view.
  /// @param nCells_ Number of cells to retrieve.
  /// @param intrinsics_ Camera intrinsics.
  /// @param imageSize_ Size of the image.
  /// @param cameraToWorldTransformation_ Transformation from camera to world.
  /// @return Set of visible cell IDs.
  [[nodiscard("Use returned cell set to drive map visibility and rendering")]]
  cell_id_t::set get_visible_cells(
      size_t nCells_,
      const intrinsics_t intrinsics_,
      const array2Size_t imageSize_,
      const pose_t cameraToWorldTransformation_);

  /// @brief Store current keyframe visualization-buffer membership.
  /// @param keyframesInBuffer_ Map from keyframe IDs to buffer slots.
  inline void aux_buffer_for_visualization(
      const dictionary_t<keyframe_id_t, int>& keyframesInBuffer_) {
    using map::set_keyframes_in_buffer;
    set_keyframes_in_buffer(keyframesInBuffer_, _keyframes.size());
  }

  /// @brief Store keyframe creation metrics for visualization.
  /// @param keyframeCreationCriteria_ Map of metric identifiers and values.
  inline void aux_keyframe_criteria_visualization(
      const dictionary_t<uint8_t, int>& keyframeCreationCriteria_) {
    map::set_keyframe_creation_metrics(keyframeCreationCriteria_);
  }

  /// @brief Get mutable access to the full point-neighbor lookup table.
  /// @note This structure can be large; avoid unnecessary copies.
  /// @return Reference to all point neighbors.
  [[nodiscard("Use returned reference to inspect or modify neighbor topology")]]
  inline hash_t<point_id_t, neighbor_t::vector>& get_all_neighbors() {
    return _neighbors;
  }

  /// @brief Find keyframes with IDs greater than the given starting keyframe.
  /// @param startingKeyframe_ Starting keyframe identifier.
  /// @return Ordered set of keyframe IDs greater than `startingKeyframe_`.
  [[nodiscard("Use returned keyframe IDs for recent-history processing")]]
  keyframe_id_t::ordered find_recent_keyframes(
      const keyframe_id_t startingKeyframe_) const;

  /// @brief Update the set of keyframes used by the localization buffer.
  /// @param keyframesInBuffer_ Ordered keyframes to keep in the buffer.
  /// @return Final set of keyframes stored in the localization buffer.
  void set_localization_buffer_keyframes(
      const keyframe_id_t::ordered& keyframesInBuffer_);

  /// @brief Get current set of keyframes used by the localization buffer.
  /// @return Set of keyframe IDs in the localization buffer.
  [[nodiscard("Use returned set to inspect localization buffer content")]]
  keyframe_id_t::set get_localization_buffer_keyframes() const;

private:
  /// @brief Construct an empty map instance.
  Map();

  /// @brief Start internal worker threads used by the map.
  void __start_threads();

  // ------------------------------------------------------------------------
  // ------------------------------------------------------------------------
  // Auxiliary functions the the map. These are suppose to be
  // only used inside the map class.

  /// @brief Retrieve cell data associated with a cell ID.
  /// @param cellId_ Cell identifier.
  /// @return Cell wrapped in output_t, or a fail_t error.
  [[nodiscard("Use returned cell output to check retrieval success")]]
  output_t<cell3d_t> __get(const cell_id_t& cellId_) const;

  /// @brief Get map size counters without additional locking.
  /// @return Pair `(numKeyframes, numPoints)`.
  [[nodiscard(
      "Use returned pair to consume current map keyframe/point counts")]]
  inline pair_t<size_t, size_t> __size() const {
    return pair_t<size_t, size_t>(_keyframes.size(), _points.size());
  }

  /// @brief Check whether a cell exists without external locking.
  /// @param cellId_ Cell identifier.
  /// @return `true` if cell exists, otherwise `false`.
  [[nodiscard("Use returned bool to branch on internal cell existence")]]
  bool __exists(const cell_id_t& cellId_) const;

  /// @brief Retrieve current number of allocated cells.
  /// @return Number of cells in the map {numberQueryCells, numberClosestCells,
  /// totalNumberCells}.
  [[nodiscard("Use returned cell count to size map-cell operations")]]
  std::tuple<size_t, size_t, size_t> __get_number_cells() const;

  /// @brief Get next point ID and increment the point counter.
  /// @return Newly assigned point ID.
  [[nodiscard("Use returned point ID to index inserted map point")]]
  inline point_id_t __get_next_point_id() {
    point_id_t pointId_ = _nextPointId;
    _nextPointId++;
    return pointId_;
  }

  /// @brief Get next keyframe ID and increment the keyframe counter.
  /// @return Newly assigned keyframe ID.
  [[nodiscard("Use returned keyframe ID to index inserted keyframe")]]
  inline keyframe_id_t __get_next_keyframe_id() {
    keyframe_id_t keyframeId_ = _nextKeyframeId;
    _nextKeyframeId++;
    return keyframeId_;
  }

  /// @brief Validate internal point-neighbor synchronization.
  /// @param pointId_ Point identifier to validate.
  inline void __sanity_sync(const point_id_t& pointId_) const {
    if (_points.contains(pointId_) && !_neighbors.contains(pointId_))
      __throw__("Point exists but _neighbors do not.");
    if (!_points.contains(pointId_) && _neighbors.contains(pointId_))
      __throw__("Neighbors exist but point_ does not.");
  };

  /// @brief Return failure status if a point ID does not exist.
  /// @param pointId_ Point identifier to validate.
  /// @return fail_t::NO_FAIL if valid, otherwise a fail_t error.
  [[nodiscard("Use returned fail_t to validate point existence checks")]]
  inline fail_t __exists(const point_id_t& pointId_) const {
    // first, we make sure that everything is sync. This will throw an
    // and error and leave the program.
    __sanity_sync(pointId_);
    // Then return the appropriate error if something is missing.
    if (!_points.contains(pointId_))
      return fail_t(fail_t::NOT_FOUND, "Point ID does not exist.");
    return fail_t::NO_FAIL;
  };

  /// @brief Return failure status if a keyframe ID does not exist.
  /// @param keyframeId_ Keyframe identifier to validate.
  /// @return fail_t::NO_FAIL if valid, otherwise a fail_t error.
  [[nodiscard("Use returned fail_t to validate keyframe existence checks")]]
  inline fail_t __exists(const keyframe_id_t& keyframeId_) const {
    if (!_keyframes.contains(keyframeId_))
      return fail_t(fail_t::NOT_FOUND, "Keyframe ID does not exist.");
    return fail_t::NO_FAIL;
  };

  /// @brief Throw if a point ID does not exist.
  /// @param pointId_ Point identifier to validate.
  inline void __sanity_exists(const point_id_t& pointId_) const {
    // first, we make sure that everything is sync. This will throw an
    // and error and leave the program.
    __sanity_sync(pointId_);
    // Then return the appropriate error if something is missing.
    if (!_points.contains(pointId_)) __throw__("Point ID does not exist.");
  };

  /// @brief Throw if a keyframe ID does not exist.
  /// @param keyframeId_ Keyframe identifier to validate.
  inline void __sanity_exists(const keyframe_id_t& keyframeId_) const {
    if (!_keyframes.contains(keyframeId_))
      __throw__("Keyframe ID does not exist.");
  };

  /// @brief Throw if a cell ID does not exist.
  /// @param cellId_ Cell identifier to validate.
  void __sanity_exists(const cell_id_t& cellId_) const;

  /// @brief Acquire exclusive locks for point and neighbor containers.
  inline void __lock_points() const {
    _points.lock();
    _neighbors.lock();
  }

  /// @brief Release exclusive locks for point and neighbor containers.
  inline void __unlock_points() const {
    _neighbors.unlock();
    _points.unlock();
  }

  /// @brief Acquire shared locks for point and neighbor containers.
  inline void __read_lock_points() const {
    _points.lock_read();
    _neighbors.lock_read();
  }

  /// @brief Release shared locks for point and neighbor containers.
  inline void __read_unlock_points() const {
    _neighbors.unlock_read();
    _points.unlock_read();
  }

  /// @brief Acquire exclusive lock for keyframe container.
  inline void __lock_keyframes() const { _keyframes.lock(); }

  /// @brief Release exclusive lock for keyframe container.
  inline void __unlock_keyframes() const { _keyframes.unlock(); }

  /// @brief Acquire shared lock for keyframe container.
  inline void __read_lock_keyframes() const { _keyframes.lock_read(); }

  /// @brief Release shared lock for keyframe container.
  inline void __read_unlock_keyframes() const { _keyframes.unlock_read(); }

  /// @brief Worker thread routine that reorders visible cells by distance.
  /// @param token_ Stop token used to interrupt the worker thread.
  void __sort_cells(std::stop_token token_);

  /// @brief Retrieve point neighbors without taking additional locks.
  /// @param pointId_ Point identifier.
  /// @return Const reference to neighbor list, or an empty static list.
  [[nodiscard("Use returned reference to inspect local point neighborhood")]]
  inline const neighbor_t::vector& __get_mappoints_neighboors_x(
      point_id_t pointId_) const {
    if (!_neighbors.contains(pointId_)) {
      static const neighbor_t::vector empty;
      return empty;
    }
    return _neighbors.at(pointId_);
  }

  /// @brief Retrieve point IDs contained in a 3D cell.
  /// @param cellId_ Cell identifier.
  /// @return Vector of point IDs in the requested cell.
  [[nodiscard("Use returned point IDs to traverse cell content")]]
  point_id_t::vector __get_points_in_cell_x(const cell_id_t& cellId_) const;

  /// @brief Retrieve timestamp associated with a keyframe ID.
  /// @param keyframeId_ Keyframe identifier.
  /// @return Timestamp value for the requested keyframe.
  [[nodiscard("Use returned timestamp for temporal ordering operations")]]
  time_stamp_t __get_timestamp(const keyframe_id_t keyframeId_) const;

  // -------------------------------------------------------------------------
  // List of useful mutexes to protect the map data structures from
  // concurrent access by multiple threads.
  /// Mutex to stop multiple changes of the map attributes at once
  mutable std::shared_mutex _mutex;

  // -------------------------------------------------------------------------
  // Relevant data structures that are used in the map.
  /// Current position of the camera in world coordinates
  pose_t::shared_ptr _cameraPose;

  // to keep a dictionary with keyframe_ data, that includes the pose_.
  dictionary_t<keyframe_id_t, keyframe_t::shared_ptr> _keyframes;

  /// All the 3D _points in the map
  hash_t<point_id_t, point3d_t::shared_ptr> _points;
  /// All the neighboors of the _points
  hash_t<point_id_t, neighbor_t::vector> _neighbors;

  /// All the _neighbors for the 3D _points in the map
  hash_t<point_id_t, point3d_t::vector> _pointNeighbors;

  // -------------------------------------------------------------------------
  // Static variables used to generate unique point and keyframe IDs.
  /// Next point ID to be assigned.
  inline static point_id_t _nextPointId = point_id_t(0);
  /// Next keyframe ID to be assigned.
  inline static keyframe_id_t _nextKeyframeId = keyframe_id_t(0);

  std::jthread _sortCellsThread;

};  // class Map

}  // namespace map
}  // namespace slam_mer

#endif  // MAP_HPP
