/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file localization.hpp
/// @brief Localization module API for frame buffering and pose estimation.
/// @details Declares local-frame buffering and localization orchestration
/// interfaces that ingest sensor frames, match observations against map/local
/// data, estimate poses, and decide when to promote frames to keyframes.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

// SLAM-MER includes
#include <covisibility_graph.hpp>
#include <frame.hpp>
#include <loop_detection.hpp>
#include <map.hpp>
#include <types/descriptor.hpp>
#include <types/dictionary.hpp>
#include <types/mutex.hpp>
#include <types/point2d.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>
#include <types/set.hpp>
#include <types/vector.hpp>

#ifdef VISUAL_PLACE_LOCALIZATION
#include <vpl/visual_place_recognition.hpp>
#endif

// Standard C++ libraries
#include <memory>
#include <shared_mutex>

namespace slam_mer {
namespace localization {

#ifdef VISUAL_PLACE_LOCALIZATION
using vpl::visual_place_recognition::VisualPlaceRecognition;
#endif

/// Localization running modes.
enum LocalizationState {
  NORMAL,  //< Regular mode of creating frames/keyframes and estimating pose
  NOT_INITIALIZED,  //< Starting mode before receiving any inputs
  RELOCALIZATION,   //< Localization was lost and is being recovered
};

/// @brief Aggregates map-side world data used for frame localization.
/// @details Stores 3D points, descriptors, keyframe observations, and point
/// IDs extracted from keyframes selected into the local buffer.
class WorldData {
public:
  /// @brief Clear all buffered world-data containers.
  inline void clear() {
    _points.clear();
    _descriptors.clear();
    _observations.clear();
    _pointIds.clear();
  }
  /// @brief Check whether world-data buffers are empty.
  /// @return `true` if no world-data entries are stored, otherwise `false`.
  [[nodiscard("Use returned bool to check if world-data buffer is empty")]]
  inline bool empty() const {
    return _points.empty();
  }

  /// @brief Get the number of points.
  /// @return Number of points in the buffer.
  [[nodiscard("Use returned size to check if world-data buffer is empty")]]
  inline size_t size() const {
    return _points.size();
  }

  /// @brief Get buffered 3D point pointers from keyframes.
  /// @return Const reference to buffered world points.
  [[nodiscard("Use returned point list for matching against current frame")]]
  inline const vector_t<point3d_t::weak_ptr>& points() const {
    return _points;
  }

  /// @brief Get buffered descriptors associated with world points.
  /// @return Const reference to buffered descriptors.
  [[nodiscard("Use returned descriptors for feature matching")]]
  inline const vector_t<descriptor_t>& descriptors() const {
    return _descriptors;
  }

  /// @brief Get keyframe observations associated with buffered points.
  /// @return Const reference to point-observation lists.
  [[nodiscard("Use returned observations to recover keyframe visibility info")]]
  inline const vector_t<keyframe_id_t::vector>& observations() const {
    return _observations;
  }

  /// @brief Get map-point IDs associated with buffered world points.
  /// @return Const reference to point ID list.
  [[nodiscard("Use returned point IDs to map matches back to map points")]]
  inline const point_id_t::vector& point_ids() const {
    return _pointIds;
  }
  /// @brief Append one world-data entry to the buffer.
  /// @param point_ World point pointer.
  /// @param descriptor_ Descriptor for the world point.
  /// @param observation_ Keyframes observing this world point.
  /// @param pointId_ Map-point identifier.
  inline void push_back(
      const point3d_t::weak_ptr& point_,
      const descriptor_t& descriptor_,
      const keyframe_id_t::vector& observation_,
      const point_id_t& pointId_) {
    _points.push_back(point_);
    _descriptors.push_back(descriptor_);
    _observations.push_back(observation_);
    _pointIds.push_back(pointId_);
  }

private:
  /// Data members: Points
  vector_t<point3d_t::weak_ptr> _points;
  /// Data members: Descriptors
  vector_t<descriptor_t> _descriptors;
  /// Data members: Observations
  vector_t<keyframe_id_t::vector> _observations;
  /// Data members: Point IDs
  point_id_t::vector _pointIds;
};

/// @brief Maintains temporal localization buffer and keyframe-promotion logic.
/// @details Stores recent frames and keyframe references, gathers world data
/// from nearby keyframes, and decides when to create and submit new keyframes.
template <typename TransformationType>
class FramesBuffer {

public:
  /// Typed alias for the FramesBuffer class template
  using FramesBufferTyped = FramesBuffer<TransformationType>;

  /// Auxiliary typedefs for covisibility graph point definition
  using CovisibilityTyped =
      covisibility_graph::CovisibilityGraph<TransformationType>;
  /// Shared pointer alias for the covisibility graph.
  using CovisibilityPointer = CovisibilityTyped::shared_ptr;

  /// Unique pointer alias for FramesBuffer.
  using unique_ptr = std::unique_ptr<FramesBufferTyped>;

  /// @brief Allocates a new object of type FramesBuffer.
  /// @param map_ Slam system map.
  /// @param covisibilityGraph_ Slam system covisibility graph.
  /// @return Unique pointer to created FramesBuffer object.
  [[nodiscard("Use returned unique_ptr to keep FramesBuffer ownership")]]
  static unique_ptr make_unique(
      const map::Map::shared_ptr map_,
      const CovisibilityPointer covisibilityGraph_) {
    return std::make_unique<FramesBufferTyped>(map_, covisibilityGraph_);
  }

  /// @brief Construct a new FramesBuffer object.
  /// @param map_ Slam system map.
  /// @param covisibilityGraph_ Slam system covisibility graph.
  FramesBuffer(
      const map::Map::shared_ptr map_,
      const CovisibilityPointer covisibilityGraph_)
      : _map(map_), _covisibilityGraph(covisibilityGraph_) {}

  /// @brief Destroys the FramesBuffer object.
  ~FramesBuffer() = default;

  /// @brief Check whether temporal frame buffer reached configured capacity.
  /// @return `true` when buffer size is at or above configured limit.
  [[nodiscard("Use returned bool to decide when to evict or promote frames")]]
  bool frame_buffer_full() const {
    using slam_mer::settings::sharedSettings;
    const int& maxBufferInt = sharedSettings->temporal_buffer_size();
    const size_t maxBufferSize = static_cast<size_t>(maxBufferInt);
    return _framesInBuffer.size() >= maxBufferSize;
  }

  /// @brief Query map data near the provided prior pose into internal buffer.
  /// @param priorPose_ Prior camera pose used to query local map neighborhood.
  void query_3d_points(const pose_t& priorPose_);

  /// @brief Check whether queried world-data buffer is empty.
  /// @return `true` if no world data is available, otherwise `false`.
  [[nodiscard("Use returned bool to guard pose-estimation inputs")]]
  inline bool is_empty() const {
    return _bufferData.empty();
  }

  /// @brief Getter for world data in the buffer (3d points, descriptors,
  /// observations, point IDs).
  /// @return Const reference to buffered WorldData.
  [[nodiscard("Use returned world data for frame localization")]]
  inline const WorldData& get_world_data() const {
    return _bufferData;
  }

  /// @brief Insert a frame and optionally create a new keyframe.
  /// @param frame_ Current frame instance.
  /// @param state_ Current localization state.
  /// @param forceKeyframeCreation_ Force keyframe creation for this frame.
  /// @return `true` when a new keyframe was created, otherwise `false`.
  [[nodiscard("Use returned bool to detect keyframe-creation events")]]
  bool insert_new_frame(
      Frame::shared_ptr frame_,
      const LocalizationState state_,
      const bool forceKeyframeCreation_ = false);

  /// @brief Resets the buffer. Cleans all frames and other data.
  void reset_buffer();

  /// @brief Getter for the IDs of all the keyframes in the VO buffer.
  /// @return Ordered set of keyframe IDs currently in the buffer.
  [[nodiscard("Use returned keyframe IDs to synchronize localization state")]]
  inline keyframe_id_t::ordered get_keyframes_in_buffer() const {
    return _keyframesInBuffer;
  }

  /// Checks if the localization is lost using the buffer.
  /// @return `true` if tracking still has support in the buffer.
  [[nodiscard("Use returned bool to update localization tracking state")]]
  inline bool check_tracking_state() {
    // This function:
    // 1 - Check if there is at least one keyframe in the buffer with
    //     matches. If there is, localization is considered to be OK.
    // 2 - Otherwise return false.
    for (const auto& [_, count] : _observableKeyframes)
      if (count != 0) return true;
    return false;
  }

private:
  /// Adds a new frame to the buffer. The buffer will own the frame.
  /// @param frame_ Frame to append to the temporal buffer.
  void __add_frame_to_buffer(const Frame::shared_ptr frame_);

  /// @brief Decide whether a new keyframe should be created.
  /// @param frame_ Output frame chosen for keyframe creation when decision is
  /// positive.
  /// @param force_ Force keyframe creation if `true`.
  /// @return `true` if keyframe should be created, otherwise `false`.
  [[nodiscard("Use returned bool to gate keyframe-creation execution")]]
  bool __keyframe_creation_decision(
      Frame::shared_ptr& frame_,
      const bool force_ = false);

  /// @brief Create and submit a keyframe from the selected frame.
  /// @param frame_ Frame to upgrade to a keyframe.
  /// @return `true` if keyframe creation succeeded, otherwise `false`.
  [[nodiscard("Use returned bool to handle keyframe-creation failures")]]
  bool __keyframe_creation(const Frame::shared_ptr frame_);

  /// @brief Add frame reference-keyframe contribution to observable counts.
  /// @param frameToAddId_ Frame ID whose reference keyframe will be counted.
  void __add_to_observable_keyframes(const frame_id_t frameToAddId_);

  /// @brief Remove frame reference-keyframe contribution from observable counts.
  /// @param frameToEraseId_ Frame ID whose reference contribution will be
  /// removed.
  void __remove_from_observable_keyframes(const frame_id_t frameToEraseId_);

  /// @brief Estimate scale correction for frame 3D points after keyframe
  /// creation.
  /// @param frame_ Current frame.
  /// @param newKeyframeId_ ID of keyframe created from current frame.
  void __fix_point3d_scale(
      const Frame::shared_ptr frame_,
      const keyframe_id_t newKeyframeId_);

  /// @brief Ensure frame has usable depth map data.
  /// @param frame_ Frame to validate or augment with depth.
  void __ensure_depth_map(const Frame::shared_ptr frame_);

  /// Pointer to the SLAM map object
  map::Map::shared_ptr _map;
  /// Local visual odometry frames
  dictionary_t<frame_id_t, Frame::shared_ptr> _framesInBuffer;
  /// Ordered set with IDs of the keyframes in the VO buffer
  keyframe_id_t::ordered _keyframesInBuffer;
  /// Map with the counting of the number of times each keyframe in the
  /// buffer is a reference keyframe to the frames in the buffer.
  dictionary_t<keyframe_id_t, int> _observableKeyframes;
  /// 3d points, descriptors, observations and point IDs in the buffer.
  WorldData _bufferData;
  /// Covisibility graph object
  CovisibilityPointer _covisibilityGraph;
  /// Mutex for the buffer data
  mutable std::shared_mutex _mutex;
};

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

/// @brief Localization pipeline that creates frames, estimates pose, and
/// maintains tracking state.
/// @details Consumes sensor data, matches current observations against local
/// map data, computes camera pose, and orchestrates keyframe promotion.
template <typename TransformationType>
class Localization {
public:
  /// Typed alias for the Localization class template
  using LocalizationTyped = Localization<TransformationType>;
  /// Typed alias for the FramesBuffer class template
  using FramesBufferTyped = FramesBuffer<TransformationType>;

  /// Auxiliary typedefs for covisibility graph point definition
  using CovisibilityTyped =
      covisibility_graph::CovisibilityGraph<TransformationType>;
  /// Shared pointer alias for the covisibility graph.
  using CovisibilityPointer = CovisibilityTyped::shared_ptr;

  /// Unique pointer alias for Localization.
  using unique_ptr = std::unique_ptr<LocalizationTyped>;
  /// Shared pointer alias for Localization.
  using shared_ptr = std::shared_ptr<LocalizationTyped>;
  /// Weak pointer alias for Localization.
  using weak_ptr = std::weak_ptr<LocalizationTyped>;

  /// @brief Allocates a new object of type Localization.
  /// @param map_ Slam system map.
  /// @param covisibilityGraph_ Slam system covisibility graph.
  /// @return Unique pointer to the created Localization object.
  [[nodiscard("Returns unique_ptr to keep Localization ownership")]]
  static unique_ptr make_unique(
      const map::Map::shared_ptr map_,
      const CovisibilityPointer covisibilityGraph_) {
    return unique_ptr(
        new LocalizationTyped(std::move(map_), std::move(covisibilityGraph_)));
  }

  /// @brief Allocates a new object of type Localization.
  /// @param map_ Slam system map.
  /// @param covisibilityGraph_ Slam system covisibility graph.
  /// @return Shared pointer to the created Localization object.
  [[nodiscard("Returns shared_ptr to keep Localization ownership")]]
  static shared_ptr make_shared(
      const map::Map::shared_ptr map_,
      const CovisibilityPointer covisibilityGraph_) {
    return shared_ptr(
        new LocalizationTyped(std::move(map_), std::move(covisibilityGraph_)));
  }

  /// Destructor of the Localization object.
  ~Localization();

#ifdef VISUAL_PLACE_LOCALIZATION

  /// @brief Sets the keyframe database pointer.
  /// @param visualPlaceRecognition_ Weak pointer to visual-place-recognition
  /// backend used for relocalization.
  void set_keyframe_database(
      VisualPlaceRecognition::weak_ptr visualPlaceRecognition_);

#endif  // end VISUAL_PLACE_LOCALIZATION

  /// @brief This is the localization main function.
  /// @note Compute its pose in world coordinates using a local buffer.
  /// The created frame with the local buffer of frames will hold the most
  /// recent keyframes and decide with which to create keyframes.
  /// @param data_ Input sensor data for the current timestep.
  void run(data_type::SensorData::weak_ptr data_);

  /// @brief Getter for the current camera pose in world reference.
  /// @return Transformation from current frame to world frame.
  [[nodiscard("Use returned pose for downstream mapping and visualization")]]
  inline pose_t get_camera_world_pose() {
    return _currentCameraWorldPose;
  }

  /// @brief Function that can be used to tell if the localization
  /// module is lost or if it is still localization keypoints on the image.
  /// @return True if lost, false otherwise.
  [[nodiscard("Use returned bool to branch on relocalization state")]]
  inline bool is_lost() const {
    return _state == LocalizationState::RELOCALIZATION;
  }

  /// @brief Outputs all the latest keypoints detected in the current frame.
  /// @return All the keypoints detected in the current frame.
  [[nodiscard("Use returned keypoints for diagnostics and visualization")]]
  point2d_t::vector get_all_frame_keypoints();

  /// @brief Retrieves the set of inlier keypoints obtained in pose estimation.
  /// @return Inlier keypoints from pose estimation.
  [[nodiscard("Use returned inlier points to evaluate localization quality")]]
  point2d_t::vector get_inlier_keypoints();

private:
  /// @brief Constructor for the Localization object.
  /// @param map_ SLAM system map.
  /// @param covisibilityGraph_ Slam system covisibility graph.
  Localization(
      const map::Map::shared_ptr map_,
      const CovisibilityPointer covisibilityGraph_);

  /// @brief Creates a new Frame object from the input data.
  /// @param data_ Input sensor data.
  /// @return Output structure with the created frame or error info.
  [[nodiscard("Use returned output to handle frame-construction failures")]]
  output_t<Frame::shared_ptr> __create_frame(
      data_type::SensorData::weak_ptr data_);

  /// @brief Estimate world pose of current frame from local world-data buffer.
  /// @param frame_ Current frame.
  void __localize(const Frame::shared_ptr frame_);

#ifdef VISUAL_PLACE_LOCALIZATION

  /// @brief Attempt relocalization against keyframe database candidates.
  /// @param frame_ Current frame.
  void __relocalize(const Frame::shared_ptr frame_);

#endif  // end VISUAL_PLACE_LOCALIZATION

  /// @brief Estimate frame pose from 2D-3D correspondences.
  /// @param frame_ Current frame.
  /// @param worldData_ Buffered world data used for matching.
  /// @param inliers_ Output inlier mask for selected correspondences.
  /// @param matches_ Output 2D-3D match list.
  /// @param checkPoseLimits_ Whether to enforce configured pose limits.
  /// @return `true` when pose estimation succeeds, otherwise `false`.
  [[nodiscard("Use returned bool to branch on pose-estimation success")]]
  bool __pose_estimation(
      const Frame::shared_ptr frame_,
      const WorldData& worldData_,
      std::vector<bool>& inliers_,
      match_t<index_t>::vector& matches_,
      const bool checkPoseLimits_);

  /// @brief Selects the reference keyframe for the current frame.
  /// @param frame_ Current frame.
  /// @param worldData_ Observations and IDs for map 3D points.
  /// @param inliers_ Inlier mask from pose estimation.
  /// @param matches_ 2D-3D matches from pose estimation.
  void __select_reference_keyframe(
      const Frame::shared_ptr frame_,
      const WorldData& worldData_,
      const std::vector<bool>& inliers_,
      const match_t<index_t>::vector& matches_);

  /// Map pointer (under development)
  map::Map::shared_ptr _map;
  /// Covisibility graph object
  CovisibilityPointer _covisibilityGraph;
  /// Relative transformation from the current frame to the world frame
  pose_t _currentCameraWorldPose;
  /// Tracking state
  LocalizationState _state;
  /// Local visual odometry frames
  typename FramesBufferTyped::unique_ptr _framesBuffer;

#ifdef VISUAL_PLACE_LOCALIZATION
  /// Pointer to the keyframes database and place recognition method
  VisualPlaceRecognition::weak_ptr _visualPlaceRecognition;
#endif
};

}  // namespace localization
}  // namespace slam_mer

#endif  // LOCALIZATION_HPP
