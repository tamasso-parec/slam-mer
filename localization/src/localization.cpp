/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file localization.cpp
/// @brief Implements frame buffering and localization pipeline execution.
/// @details Defines local matching, pose estimation, and keyframe decisions.
/// @author Pedro Miraldo
/// @author Valter Piedade

// SLAM-MER includes
#include <image_processing.hpp>
#include <kl_divergence.hpp>
#include <localization.hpp>
#include <pose_estimation.hpp>
#include <registration.hpp>
#include <types/timer.hpp>

// Third-party includes
#include <gtsam/geometry/Similarity3.h>

// Standard library includes
#include <ranges>

namespace slam_mer {
namespace localization {

namespace {

/// counter of the number of points in the current frame tracked by keyframes
dictionary_t<keyframe_id_t, int> __auxKeyframeReferenceCount;
dictionary_t<keyframe_id_t, int> __auxPreviousKeyframeReferenceCount;
/// Flag that indicates if keyframe creation being executed
bool __inKeyframeCreation = false;
/// Mutex for the inKeyframeCreation global variable
std::shared_mutex __inKeyframeCreationMutex;

/// Previously estimated absolute pose (ie, last frame world pose)
pose_t __previousFramePose = pose_t::Identity();
/// Id of the frame that last created a keyframe
frame_id_t __lastFrameId = frame_id_t::UNSET;
/// Total number of keyframes created
size_t __totalKeyframesCreated = 0;

/// All keypoints detected in the current frame
point2d_t::vector __allDetectedImagePoints;
/// Inliers from pose estimation
point2d_t::vector __poseInlierKeypoints;

/// Flag to set the use of frame images for depth inference. If false,
/// images from keyframes in the buffer are used instead
bool __useRecentFramesForInference = false;
/// Flag to set the use of 3D registration to obtain odometry measurements
/// between keyframes. If false, the current keyframe poses are used to
// compute the relative pose measurement
bool __odometryMeasurementsByRegistration = true;

/// Class to represent the status of keyframe creation
class KeyframeCreationStatus {
public:
  static constexpr uint8_t UNSET = 0;
  static constexpr uint8_t FAILED_STILL_CREATING = 3;
  static constexpr uint8_t FAILED_LESS_X_SINCE_LAST_KEYFRAME = 4;
  static constexpr uint8_t FAILED_OTHER = 5;
  static constexpr uint8_t KL_DIVERGENCE = 6;
  static constexpr uint8_t NUMBER_POINTS = 7;
  static constexpr uint8_t IMAGE_COVERAGE = 8;
  static constexpr uint8_t POSITION_VARIATION = 9;
  static constexpr uint8_t FORCED = 10;
  static constexpr uint8_t EMPTY_BUFFER = 11;

  /// Statistics of keyframe creation
  static dictionary_t<uint8_t, int> _statistic;

  /// Constructor, default to unset (not created)
  KeyframeCreationStatus() : _status(UNSET) {}
  /// Equality operator to compare with a uint8_t status
  /// @param other_ Parameter `other_`.
  bool operator==(const uint8_t other_) const { return _status == other_; }
  /// Sets the status of the keyframe creation
  /// @param newStatus_ Parameter `newStatus_`.
  void set(uint8_t newStatus_) {
    _status = newStatus_;
    if (!failed()) KeyframeCreationStatus::_statistic[_status]++;
  }
  /// Gets the status of the keyframe creation
  /// @return Returned value.
  uint8_t get() const { return _status; }
  /// Checks if the keyframe creation has failed
  /// @return Returned value.
  bool failed() const {
    if (_status == FAILED_STILL_CREATING) return true;
    if (_status == FAILED_LESS_X_SINCE_LAST_KEYFRAME) return true;
    if (_status == FAILED_OTHER) return true;
    if (_status == UNSET) return true;
    return false;
  }
  /// Returns a message describing the status of the keyframe creation
  /// @param prefix_ Parameter `prefix_`.
  /// @return Returned value.
  std::string message(std::string prefix_ = "") const {
    std::string output;
    if (failed()) {
      output = prefix_ + "Keyframe creation failed. Reason: " + __message();
    } else {
      output = prefix_ + "Keyframe creation successful. Reason: " + __message();
    }
    return output;
  }
  /// Returns true if the keyframe creation was successful
  /// @return Returned value.
  inline operator bool() const { return !this->failed(); }
  /// Checks if the keyframe creation was successful
  /// @return Returned value.
  bool successful() const { return !this->failed(); }

private:
  std::string __message() const {
    switch (_status) {
    case UNSET:
      return "Unset";
    case FAILED_STILL_CREATING:
      return "Fail still creating one keyframe";
    case FAILED_LESS_X_SINCE_LAST_KEYFRAME:
      return "Fail less X since last";
    case FAILED_OTHER:
      return "Failed other";
    case KL_DIVERGENCE:
      return "KL divergence";
    case NUMBER_POINTS:
      return "Number points";
    case IMAGE_COVERAGE:
      return "Image coverage";
    case POSITION_VARIATION:
      return "Position variation";
    case FORCED:
      return "Forced";
    case EMPTY_BUFFER:
      return "Empty buffer";
    default:
      return "UNKNOWN";
    }
  }
  uint8_t _status;
};

/// Statistics of keyframe creation. Each entry counts the number of times
/// each status occurred
dictionary_t<uint8_t, int> KeyframeCreationStatus::_statistic = {
    {KL_DIVERGENCE, 0},
    {NUMBER_POINTS, 0},
    {IMAGE_COVERAGE, 0},
    {POSITION_VARIATION, 0},
    {FORCED, 0},
    {EMPTY_BUFFER, 0}};

}  // namespace

// ---------------------------------------------------------------------------
// Insert frame into localization buffer and optionally create keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
bool FramesBuffer<TransformationType>::insert_new_frame(
    Frame::shared_ptr frame_,
    const LocalizationState state_,
    const bool forceKeyframeCreation_) {

  // --------------------------------------------------------------------------
  // Lambda functions
  // --------------------------------------------------------------------------
  // Lambda to handle keyframe creation for a decided frame
  auto ___call_keyframe_creation =
      [this, state_](const Frame::shared_ptr& frame__) -> bool {
    log::message(
        "Creating a new keyframe for frame with " +
            std::string(frame__->get_id()),
        log::Module::TRACKING);

    bool success = true;
    // For monocular data, we create the keyframe in a parallel thread to not
    // block the tracking thread as inference can take some time. The
    // exceptions are the first keyframe and when getting out of relocalization.
    if (frame__->is_monocular()) {
      if (state_ == LocalizationState::NORMAL) {
        std::jthread parallelProcess(
            &FramesBufferTyped::__keyframe_creation,
            this,
            frame__);
        parallelProcess.detach();
      } else {
        success = __keyframe_creation(frame__);
      }
    } else {
      success = __keyframe_creation(frame__);
    }
    if (success) {
      __lastFrameId = frame__->get_id();
      __totalKeyframesCreated++;
    }
    return success;
  };

  // This function:
  // 1 - Adds the new frame to the temporal buffer
  // 2 - If reference keyframe is valid or if set to force, enters
  //     keyframe creation.
  // 3 - If so, first checks whether the keyframe creation decision is positive.
  //     If positive, creates the keyframe in a parallel thread.
  bool createdNewKeyframe = false;

  __add_frame_to_buffer(frame_);

  const bool& isSet =
      frame_->get_reference_keyframe_id() != keyframe_id_t::UNSET;
  if (isSet || forceKeyframeCreation_) {
    // Check criteria for keyframe creation
    Frame::shared_ptr decidedFrame;
    const bool createKeyframe =
        __keyframe_creation_decision(decidedFrame, forceKeyframeCreation_);
    // If the decision is positive, create the keyframe with the decided frame
    if (createKeyframe) {
      createdNewKeyframe = ___call_keyframe_creation(decidedFrame);
    }

  } else {
    log::warning(
        "No reference keyframe -- no keyframe created",
        log::Module::TRACKING);
  }

  // Update map variables
  _map->set_localization_buffer_keyframes(_keyframesInBuffer);
  _map->aux_buffer_for_visualization(__auxKeyframeReferenceCount);
  _map->aux_keyframe_criteria_visualization(KeyframeCreationStatus::_statistic);

#ifdef LOG_LOCALIZATION
  std::string bufferKeyframes = "";
  for (const auto& keyframeId : _keyframesInBuffer)
    bufferKeyframes += keyframeId.to_string() + ", ";
  log::message("Keyframes in buffer: [" + bufferKeyframes + "]", log::TRACKING);
  log::message(
      "Buffer current size is " + std::to_string(_framesInBuffer.size()),
      log::TRACKING);
#endif

  return createdNewKeyframe;
}

// ---------------------------------------------------------------------------
// Decide whether a new keyframe should be created
// ---------------------------------------------------------------------------
template <typename TransformationType>
bool FramesBuffer<TransformationType>::__keyframe_creation_decision(
    Frame::shared_ptr& frame_,
    const bool force_) {
  using data_type::SensorData;
  using slam_mer::settings::sharedSettings;
  using statistics::kl_divergence;
  using std::println;

  // --------------------------------------------------------------------------
  // Lambda functions
  // Count the number of points in each cell of the grid
  // Lambda to get image size from the current frame's camera data
  auto ___get_image_size = [](const Frame::shared_ptr& frame__,
                              output_t<array2Size_t>& sizeOut__,
                              KeyframeCreationStatus& status__) -> bool {
    const output_t<SensorData::weak_ptr> cameraData =
        frame__->get_camera_data();
    if (cameraData.fail()) {
      std::println(
          "[Localization] {}Warning{}: Keyframe creation failed to get camera "
          "data.",
          yellow,
          reset);
      return false;
    }

    const output_t<array2Size_t> size = SensorData::get_image_size(cameraData);
    if (size.fail()) {
      std::println(
          "[Localization] {}Warning{}: Keyframe creation failed to get image "
          "size.",
          yellow,
          reset);
      return false;
    }
    sizeOut__ = size;
    return true;
  };
  // --------------------------------------------------------------------------

  // This function decides if a new keyframe should be created based on
  // several criteria defined in the settings:
  // 1 - If there are no keyframes in the buffer, we create a keyframe with
  // the most recent frame. This can occur at the start or after a buffer reset
  // 2 - If the force flag is set to true, we create a keyframe with the most
  //     recent frame
  // 3 - If a keyframe creation is already being executed in parallel, we do not
  //     create a new keyframe
  // 4 - If the last keyframe was created less than a set number of frames ago,
  //     we do not create a new keyframe
  // 5 - If there are high sudden variations in the histogram of tracked points
  //     across keyframes (KL divergence), we create a new keyframe
  // 6 - If the most recent frame has less than the required matched points
  //     against the buffer, we create a new keyframe
  // 7 - If the tracked points in the frame are not well spread across the
  // image,
  //     we create a new keyframe
  // 8 - If the error between the current frame and the previous keyframe
  //     exceeds defined limits, we create a new keyframe

  const Frame::shared_ptr currentFrame = _framesInBuffer.rbegin()->second;
  KeyframeCreationStatus status;

  // 1-------------------------------------------------------------------------
  if (_keyframesInBuffer.empty()) {
    status.set(KeyframeCreationStatus::EMPTY_BUFFER);
    frame_ = currentFrame;
    goto __DONE__;
  }

  // 2-------------------------------------------------------------------------
  if (force_) {
    status.set(KeyframeCreationStatus::FORCED);
    frame_ = currentFrame;
    goto __DONE__;
  }

  // 3-------------------------------------------------------------------------
  {
    read_lock_t lock(__inKeyframeCreationMutex);
    if (__inKeyframeCreation) {
      status.set(KeyframeCreationStatus::FAILED_STILL_CREATING);
      goto __DONE__;
    }
  }

  // 4-------------------------------------------------------------------------
  {
    const frame_id_t currentFrameId = currentFrame->get_id();
    const frame_id_t diffFrames = currentFrameId - __lastFrameId;
    const frame_id_t maximum =
        sharedSettings->minimum_frames_between_keyframes();
    if (diffFrames < maximum) {
      status.set(KeyframeCreationStatus::FAILED_LESS_X_SINCE_LAST_KEYFRAME);
      goto __DONE__;
    }
  }

  // 5-------------------------------------------------------------------------
  if (sharedSettings->use_kl_divergence_condition()) {
    // skip if observation counts are empty
    const bool empty = __auxKeyframeReferenceCount.empty();
    const bool emptyPrev = __auxPreviousKeyframeReferenceCount.empty();
    if (!empty && !emptyPrev) {
      // only compute KL divergence if last bin in current count is zero
      // or if last bin in previous count is non-zero
      const auto& count = __auxKeyframeReferenceCount;
      const auto& previousCount = __auxPreviousKeyframeReferenceCount;
      const float lastBinCount = count.rbegin()->second;
      const float lastBinCountPrev = previousCount.rbegin()->second;

      if (lastBinCount == 0 || lastBinCountPrev != 0) {
        const float klScore = kl_divergence(count, previousCount);
        if (klScore > sharedSettings->kl_divergence_threshold()) {
          frame_ = currentFrame;
          status.set(KeyframeCreationStatus::KL_DIVERGENCE);
          goto __DONE__;
        }
      }
    }
  }

  // 6-------------------------------------------------------------------------
  if (sharedSettings->use_tracked_points_condition()) {
    // Check if the current frame have less than the required
    // number of matched points
    const int numTrackedPoints = currentFrame->get_number_tracked_points();
    const int minimum = sharedSettings->minimum_tracked_points();
    if (numTrackedPoints < minimum) {
      frame_ = currentFrame;
      status.set(KeyframeCreationStatus::NUMBER_POINTS);
      goto __DONE__;
    }
  }

  // 7-------------------------------------------------------------------------
  if (sharedSettings->use_image_coverage_condition()) {
    output_t<array2Size_t> size;
    if (!___get_image_size(currentFrame, size, status)) {
      status.set(KeyframeCreationStatus::FAILED_OTHER);
      goto __DONE__;
    }

    Eigen::MatrixXi counts = image_processing::count_points_per_cell(
        __poseInlierKeypoints,
        size,
        sharedSettings->image_coverage_grid_rows(),
        sharedSettings->image_coverage_grid_cols());

    // Check if enough cells have at least the minimum number of points
    const int minimumPointsPerCell =
        sharedSettings->image_coverage_minimum_points_per_grid();
    int cellsCovered = 0;
    for (int row = 0; row < counts.rows(); ++row)
      for (int col = 0; col < counts.cols(); ++col)
        if (counts(row, col) >= minimumPointsPerCell) cellsCovered++;

    // Check if the number of covered cells is below the threshold
    const int minimumCellsCovered =
        sharedSettings->image_coverage_minimum_grids_filled();
    if (cellsCovered <= minimumCellsCovered) {
      frame_ = currentFrame;
      status.set(KeyframeCreationStatus::IMAGE_COVERAGE);
      goto __DONE__;
    }

    // Compute points coverage in the image
    // To be used in keyframe creation decision
    // const double imageCoverageScore =
    // image_processing::compute_coverage_ratio(
    //     __poseInlierKeypoints,
    //     currentFrame->get_all_detected_image_points());
    // const double& scoreThreshold = sharedSettings->image_coverage_threshold();
    // if (imageCoverageScore < scoreThreshold) {
    //   frame = currentFrame;
    //   status.set(KeyframeCreationStatus::IMAGE_COVERAGE);
    //   goto __DONE__;
    // }
  }

__DONE__:

#ifdef LOG_LOCALIZATION
  log::message(status.message(), log::TRACKING);
#endif

  // return false (not create) unless one of the keyframe criteria is met
  return status.successful();
}

// ---------------------------------------------------------------------------
// Add frame reference keyframe to observable-keyframe counters
// ---------------------------------------------------------------------------
template <typename TransformationType>
void FramesBuffer<TransformationType>::__add_to_observable_keyframes(
    const frame_id_t frameToAddId_) {

  // This function:
  // 1 - Gets the reference keyframe of the input frame ID
  // 2 - Adds the reference keyframe to the observable keyframes map
  // 3 - Increments the count of the reference keyframe in the map
  // 4 - Adds the reference keyframe to the keyframes in buffer set if not
  //     already present

  write_lock_t lock(_mutex);
  keyframe_id_t _referenceKeyframe =
      _framesInBuffer[frameToAddId_]->get_reference_keyframe_id();
  if (_observableKeyframes.find(_referenceKeyframe) ==
      _observableKeyframes.end()) {
    _observableKeyframes.insert(std::make_pair(_referenceKeyframe, 1));
    _keyframesInBuffer.insert(_referenceKeyframe);
  } else {
    // Add one observation to _observableKeyframes
    _observableKeyframes[_referenceKeyframe] += 1;
  }
}

// ---------------------------------------------------------------------------
// Remove frame reference keyframe from observable-keyframe counters
// ---------------------------------------------------------------------------
template <typename TransformationType>
void FramesBuffer<TransformationType>::__remove_from_observable_keyframes(
    const frame_id_t frameToEraseId_) {

  // This function:
  // 1 - Gets the reference keyframe of the input frame ID
  // 2 - Decrements the count of the reference keyframe in the map
  // 3 - If the count reaches 0, removes the reference keyframe from the
  // map and the set of keyframes in the buffer

  write_lock_t lock(_mutex);
  if (_observableKeyframes.empty()) return;
  const Frame::shared_ptr& frame_ = _framesInBuffer[frameToEraseId_];
  const keyframe_id_t _referenceKeyframe = frame_->get_reference_keyframe_id();
  _observableKeyframes[_referenceKeyframe] -= 1;
  if (_observableKeyframes[_referenceKeyframe] <= 0) {
    _observableKeyframes.erase(_referenceKeyframe);
    _keyframesInBuffer.erase(_referenceKeyframe);
  }
}

// ---------------------------------------------------------------------------
// Ensure frame has depth data before keyframe upgrade
// ---------------------------------------------------------------------------
template <typename TransformationType>
void FramesBuffer<TransformationType>::__ensure_depth_map(
    const Frame::shared_ptr frame_) {
  using data_type::SensorData;
  using std::println;
  using std::views::reverse;

  // -------------------------------------------------------------------------
  // Lambda functions
  // Get image from the current frame
  auto _from_current_frame = [](vector_t<image_t>& bufferImages__,
                                const Frame::shared_ptr& frame__) -> void {
    const SensorData::shared_ptr& cameraData =
        SensorData::to_shared(frame__->get_camera_data());
    const image_t& image = cameraData->get_color_image();
    bufferImages__.push_back(image);
  };
  // Get images for inference from previous frames
  auto _from_frames = [&](vector_t<image_t>& bufferImages__,
                          const int maxImages__) -> void {
    // For the reverse method.
    int imagesForInference = 0;
    for (const auto& [_, frame] : reverse(_framesInBuffer)) {
      const SensorData::shared_ptr& cameraData =
          SensorData::to_shared(frame->get_camera_data());
      const image_t& image = cameraData->get_color_image();
      bufferImages__.push_back(image);
      imagesForInference++;
      if (imagesForInference == maxImages__) break;
    }
  };
  // Get images for inference from previous keyframes
  auto _from_keyframes = [&](vector_t<image_t>& bufferImages__,
                             const int maxImages__) -> void {
    int imagesForInference = 0;
    for (const auto& keyframeId : reverse(_keyframesInBuffer)) {
      output_t<keyframe_t::weak_ptr> keyframe = _map->at(keyframeId);
      if (keyframe.fail()) continue;
      const output_t<keyframe_t::shared_ptr> shared =
          keyframe_t::to_shared(keyframe);
      if (shared.fail()) continue;
      const image_t& image = shared->get_image();
      bufferImages__.push_back(image);
      imagesForInference++;
      if (imagesForInference == maxImages__) break;
    }
  };
  // -------------------------------------------------------------------------

  // This function:
  // 1 - Checks whether the frame already has depth data (RGB-D).
  //     If it has, do nothing
  // 2 - If it's a monocular camera, infer depth from images.
  //     We can use images from previous frames or from previous keyframes,
  // and we are always looking for up to 4 images--some depth inference method
  //     require more than 2 images for a relatively accurate depth inference.

  const SensorData::SensorType& sensorType = frame_->get_sensor_type();
  if (sensorType != SensorData::MONOCULAR) return;

  vector_t<image_t> images;

  // Add the current frame image as the first image for inference
  _from_current_frame(images, frame_);
  // Add the remaining images from either previous frames or keyframes
  if (__useRecentFramesForInference)
    _from_frames(images, 3);
  else
    _from_keyframes(images, 3);

  SensorData::weak_ptr data = frame_->get_camera_data();
  fail_t inference = SensorData::run_depth_inference(data, images);
  if (inference.fail()) {
    __throw__(
        "Failed to run depth inference for frame. Reason: {}" +
        std::string(inference));
  }
}

// ---------------------------------------------------------------------------
// Create keyframe data and push keyframe to map and graph
// ---------------------------------------------------------------------------
template <typename TransformationType>
bool FramesBuffer<TransformationType>::__keyframe_creation(
    const Frame::shared_ptr frame_) {
  using ObservationEdge =
      covisibility_graph::ObservationEdge<TransformationType>;
  using PointObservation =
      covisibility_graph::PointObservation<TransformationType>;
  using slam_mer::settings::sharedSettings;

  // -------------------------------------------------------------------------
  // Lambda functions
  auto _sanity_keyframe_created = [&](fail_t condition__) -> void {
    // A fail to push the keyframe to the map should not happen.
    if (condition__.fail())
      __throw__(
          "Failed to create keyframe for frame " +
          std::string(frame_->get_id()));
  };
  // Update the 3D to 3D matches
  auto _update3d3dMatches =
      [&](const point_id_t newPointId__,
          const point3d_t& newPoint__,
          const keyframe_id_t newKeyframeId__,
          const keyframe_id_t::vector& newKeyframeObservations__,
          dictionary_t<keyframe_id_t, vector_t<point3d_t::pair>>&
              keyframe3dMatches__) -> void {
    for (const keyframe_id_t& keyframeId : newKeyframeObservations__) {
      if (keyframeId == newKeyframeId__) continue;
      output_t<point3d_t> priorObservation =
          _covisibilityGraph->get_point_observation(newPointId__, keyframeId);
      if (priorObservation.fail()) continue;
      if (!keyframe3dMatches__.contains(keyframeId))
        keyframe3dMatches__[keyframeId] = vector_t<point3d_t::pair>();
      keyframe3dMatches__[keyframeId].push_back(
          point3d_t::pair(newPoint__, priorObservation));
    }
  };
  // insert to the map
  auto _insert_to_map = [this, frame_](
                            const size_t index__,
                            const point3d_t& point__,
                            point_id_t& pointId__) -> fail_t {
    const pose_t framePose = frame_->get_pose();
    const point3d_t pointInWorldCoordinates = framePose.transform(point__);
    output_t<pair_t<point_id_t, cell_id_t>> insertInfo =
        _map->insert(pointInWorldCoordinates);
    // Validate point creation. Points can fail to create if the cell is full
    if (insertInfo.fail()) {
      const std::string& errorMessage = insertInfo.get_error().get_message();
#ifdef SLAMMER_DEBUG
      log::warning(errorMessage, log::Module::TRACKING);
#endif
      return {fail_t::INSERTATION_FAILED, errorMessage};
    }
    pointId__ = insertInfo.get().first;
    frame_->update_mappoint_id(index__, pointId__);
    return fail_t::NO_FAIL;
  };
  // insert to the covisibility graph
  auto _insert_to_covisibility_graph =
      [this, frame_](
          const size_t index__,
          const point_id_t pointId__,
          const keyframe_id_t keyframeId__) -> void {
    const PointObservation observation(
        frame_->get_image_points()[index__],
        frame_->get_3d_points_camera_frame()[index__],
        frame_->get_image_descriptors()[index__],
        frame_->get_pose(),
        true);  // inline = true
    const ObservationEdge edge(
        pointId__,     // point Id
        keyframeId__,  // keyframe Id
        observation);  // observation
    _covisibilityGraph->insert(edge);
  };
  // -------------------------------------------------------------------------

  // This function:
  // - First makes sure that the per-pixel map for the particular frame
  //     exists as we need that for the mappoint part.
  // 2 - Upgrade the frame object from frame to keyframe.
  // 3 - Create the keyframe data objects and add them to the map.
  // 4 - If we are dealing with monocular cameras in particular, it is
  //     essential that the 3D scales of the depth estimation is fixed to the
  //     existing map, as it is usually one or two up to a scale factors
  //     off.
  // 5 - Create a matching between 3D to 3D points between the new keyframe
  //     and existing keyframes in the map. This is important for creating
  //     odometry measurements/edges between keyframes. (To be used in the
  //     covisibility graph).
  // 6 - For each 3D point in the new keyframe, create new observations
  //     in the covisibility graph. If necessary (and desired), create new
  //     mappoints in the map. Also for each 3D point, update the 3D to 3D
  //     matches structure, and use it later to create projection edges
  //     between keyframes and points and push it to the covisibility graph.

  {
    write_lock_t lock(__inKeyframeCreationMutex);
    __inKeyframeCreation = true;
  }

  // 1------------------------------------------------------------------------
  __ensure_depth_map(frame_);
  // 2------------------------------------------------------------------------
  frame_->setup_upgrade_to_keyframe();
  if (!frame_->valid_keyframe_upgrade()) {
    log::warning(
        "Keyframe creation failed for frame " + std::string(frame_->get_id()) +
            ": no 3D points in the frame",
        log::Module::TRACKING);
    return false;
  }

  // 3------------------------------------------------------------------------
  keyframe_t keyframeData(
      frame_->get_camera_data(),
      frame_->get_pose(),
      frame_->get_all_detected_image_points(),
      frame_->get_all_detected_image_descriptors());
  const pose_t& pose = frame_->get_pose();
  output_t<keyframe_id_t> newKeyframeId = _map->insert(pose, keyframeData);
  _sanity_keyframe_created(newKeyframeId);
  // 4------------------------------------------------------------------------
  __fix_point3d_scale(frame_, newKeyframeId);

  // 5------------------------------------------------------------------------
  dictionary_t<keyframe_id_t, vector_t<point3d_t::pair>> keyframe3dMatches;

  // 6------------------------------------------------------------------------
  const size_t& numPoints = frame_->get_3d_points_camera_frame().size();
  for (size_t index = 0; index < numPoints; ++index) {

    // Get 3d point and its ID
    const point3d_t& point = frame_->get_3d_points_camera_frame()[index];
    point_id_t pointId = frame_->get_mappoint_ids()[index];

    if (pointId == point_id_t::UNSET) {
      fail_t failInsert = _insert_to_map(index, point, pointId);
      if (failInsert.fail()) continue;
    }

    _insert_to_covisibility_graph(index, pointId, newKeyframeId);

    _update3d3dMatches(
        pointId,
        point,
        newKeyframeId,
        _covisibilityGraph->get_keyframe_ids(pointId),
        keyframe3dMatches);
  }

  //////////////////////////////
  // Create odometry connections
  //////////////////////////////

  // Next, we create odometry connections between keyframes
  // We have two possible ways of doing it:
  // 1. Run 3D registration between keyframe points
  // 2. Use keyframes current pose to compute a relative pose
  // Next, we define lambda function, one for each possibility
  // Use __odometryMeasurementsByRegistration to set which to use

  // -------------------------------------------------------------------------
  auto _odometryEstimationRegistration =
      [&](std::vector<point3d_t::pair> matches__) -> pose_t {
    // Get a separate vector for source and target matches
    point3d_t::vector sourcePoints;
    sourcePoints.reserve(matches__.size());
    point3d_t::vector targetPoints;
    targetPoints.reserve(matches__.size());
    for (const auto& pointPair : matches__) {
      sourcePoints.emplace_back(pointPair.first);
      targetPoints.emplace_back(pointPair.second);
    }
    // compute odometry using 3d registration
    pose_t odometry;

    const bool success =
        registration::compute_estimate(sourcePoints, targetPoints, odometry);
    if (!success) {
      log::warning(
          "Failed to compute odometry measurement between keyframes \
              using 3D registration for odometry estimation",
          log::Module::TRACKING);
    }
    return odometry;
  };
  auto _odometryEstimation = [&](const pose_t poseAToWorld__,
                                 const pose_t poseBToWorld__) -> pose_t {
    const pose_t poseWorldToB(poseBToWorld__.get_invert());
    const pose_t odometry = poseWorldToB * poseAToWorld__;
    return odometry;
  };
  auto _sanity_pose_exists = [](const fail_t& fail__,
                                const keyframe_id_t& keyframeId__) -> bool {
    if (fail__ != fail_t::NO_FAIL) {
      log::warning(
          "Failed to get keyframe " + std::string(keyframeId__) +
              " for odometry edge creation",
          log::Module::TRACKING);
      return false;
    };
    return true;
  };
  auto _pose_set = [](const pose_t& output__,
                      const keyframe_id_t& keyframeId__) -> bool {
    if (output__.is_unset()) {
      log::warning(
          "Failed to get pose for keyframe " + std::string(keyframeId__) +
              " for odometry edge creation",
          log::Module::TRACKING);
      return false;
    }
    return true;
  };
  auto _log_no_enough_matches = [](const keyframe_id_t& keyframeId__,
                                   const keyframe_id_t& newKeyframeId__,
                                   const size_t matchesSize__) -> void {
#ifdef LOG_LOCALIZATION
    log::message(
        "No odometry edge between keyframes " + keyframeId__.to_string() +
            " and " + newKeyframeId__->to_string() + ": only " +
            std::to_string(matchesSize__) + " 3D matches",
        log::TRACKING);
#endif
  };
  auto _log_creating_odometry_edge = [](const keyframe_id_t& keyframeId__,
                                        const keyframe_id_t& newKeyframeId__,
                                        const size_t matchesSize__) -> void {
#ifdef LOG_LOCALIZATION
    log::message(
        "Creating odometry edge between keyframes " +
            std::string(keyframeId__) + " and " + std::string(newKeyframeId__) +
            " based on " + std::to_string(matchesSize__) + " 3D matches",
        log::TRACKING);
#endif
  };
  auto _log_keyframe_created = [](const keyframe_id_t& newKeyframeId__,
                                  const frame_id_t& frameId__) -> void {
#ifdef LOG_LOCALIZATION
    log::message(
        "Created a new keyframe (" + std::string(newKeyframeId__) +
            ") using frame (id: " + std::string(frameId__) + ")",
        log::TRACKING);
#endif
  };
  // -------------------------------------------------------------------------

  // Finally, we need to iterate over all counter, and create observations
  // for each with a certain amount of count matches
  for (const auto& [keyframeId, matches] : keyframe3dMatches) {
    if (keyframeId != frame_->get_reference_keyframe_id()) {
      const int& minimumMatches =
          sharedSettings->odometry_edge_minimum_matches();
      if (matches.size() < static_cast<size_t>(minimumMatches)) {
        _log_no_enough_matches(keyframeId, newKeyframeId, matches.size());
        continue;
      }
    }
    _log_creating_odometry_edge(keyframeId, newKeyframeId, matches.size());

    // Compute odometry measurement
    pose_t odometry;
    if (__odometryMeasurementsByRegistration)
      odometry = _odometryEstimationRegistration(matches);
    else {
      output_t<pose_t> keyFramePose = _map->get(keyframeId);
      if (!_sanity_pose_exists(keyFramePose, keyframeId)) continue;
      if (!_pose_set(keyFramePose, keyframeId)) continue;
      odometry = _odometryEstimation(keyFramePose, frame_->get_pose());
    }

    // Create odometry edge
    const covisibility_graph::OdometryEdge odometryEdge(
        keyframeId,
        newKeyframeId,
        odometry);
    _covisibilityGraph->insert(odometryEdge);
  }

  // -------------------------------------------------------------------------
  // Update the mappoint neighbors structure
#ifdef SHOW_DENSE
  frame_->get_neighbors(newKeyframeId, _map->get_all_neighbors());
#endif

  // -------------------------------------------------------------------------
  // Update Localization
  // Save information that the frame was upgraded to keyframe
  frame_->set_upgraded_to_keyframe_id(newKeyframeId);
  // Update keyframes observations in the VO buffer
  // 1st remove the previous reference of the frame
  __remove_from_observable_keyframes(frame_->get_id());
  // 2nd update reference keyframe to the newly created keyframe
  frame_->set_reference_keyframe_id(newKeyframeId);
  // 3rd add the new frame reference
  __add_to_observable_keyframes(frame_->get_id());

  _log_keyframe_created(newKeyframeId, frame_->get_id());
  {
    write_lock_t lock(__inKeyframeCreationMutex);
    __inKeyframeCreation = false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Add a frame to the temporal localization buffer
// ---------------------------------------------------------------------------
template <typename TransformationType>
void FramesBuffer<TransformationType>::__add_frame_to_buffer(
    const Frame::shared_ptr frame_) {
  using slam_mer::settings::sharedSettings;

  // This function:
  // 1 - Checks if we need to remove old elements from the buffer to maintain
  //     its maximum allowed size
  // 2 - Adds the new frame to the buffer
  // 3 - Updates the keyframe observations structure
  // 4 - If the buffer is at is maximum allowed size, we remove the oldest
  //     frame before adding the new frame.

  const size_t& maximumBufferSize = sharedSettings->temporal_buffer_size();
  const size_t& bufferSize = _framesInBuffer.size();
  if (maximumBufferSize <= bufferSize) {
    // The std::map is ordered by keys, which means the first element
    // is the one with the lowest id, ie, the oldest
    const frame_id_t& frameToEraseId = _framesInBuffer.begin()->first;
    // Remove the references to keyframes that the frame we are going to
    // remove has. This includes updating the counting of how
    // many times the keyframes in the buffer are references to frames
    __remove_from_observable_keyframes(frameToEraseId);
    _framesInBuffer.erase(frameToEraseId);
  }

  const frame_id_t& frameId = frame_->get_id();
  _framesInBuffer.insert(std::make_pair(frameId, frame_));
  // Update keyframe observations (only if there is a reference keyframe)
  // we may not have a reference keyframe at the start, after a reset
  // or after an unsuccessful pose estimation
  if (frame_->get_reference_keyframe_id() != keyframe_id_t::UNSET)
    __add_to_observable_keyframes(frameId);
  return;
}

// ---------------------------------------------------------------------------
// Reset temporal localization buffer state
// ---------------------------------------------------------------------------
template <typename TransformationType>
void FramesBuffer<TransformationType>::reset_buffer() {
  _framesInBuffer.clear();
  _keyframesInBuffer.clear();
  _observableKeyframes.clear();
  _bufferData.clear();
}

// ---------------------------------------------------------------------------
// Query map points and descriptors for localization buffer
// ---------------------------------------------------------------------------
template <typename TransformationType>
void FramesBuffer<TransformationType>::query_3d_points(
    const pose_t& priorPose_) {
  using data_type::SensorData;
  using slam_mer::settings::sharedSettings;
  using std::println;
  using std::views::reverse;

  // --------------------------------------------------------------------------
  // lambda function -- used to add a mappoint to the buffer variables
  auto ___add_mappoint = [this](
                             const point_id_t pointId__,
                             point_id_t::set& pointsIdsInBuffer__,
                             const keyframe_id_t keyframeId__ =
                                 keyframe_id_t::UNSET) -> void {
    if (pointsIdsInBuffer__.contains(pointId__)) return;
    output_t<point3d_t::weak_ptr> point3d = _map->at(pointId__);
    if (point3d.fail()) return;
    output_t<descriptor_t> descriptor =
        _covisibilityGraph->get_descriptor_observation(
            pointId__,
            __previousFramePose);
    if (descriptor.fail()) return;
    keyframe_id_t::vector pointObservation =
        _covisibilityGraph->get_keyframe_ids(pointId__);
    if (pointObservation.empty()) return;
    _bufferData.push_back(point3d, descriptor, pointObservation, pointId__);
    pointsIdsInBuffer__.insert(pointId__);
    return;
  };
  // --------------------------------------------------------------------------

  point_id_t::set pointsIdsInBuffer;

  // This function:
  // 1 - Clears previous points and descriptors in the buffer
  // 2 - Performs a SPATIAL QUERY to get visible points from the map
  // 3 - Performs a TEMPORAL QUERY to get points from keyframes in the buffer
  //     (most recent keyframes first)

  _bufferData.clear();

  const Frame::shared_ptr latestFrame = _framesInBuffer.rbegin()->second;
  const SensorData::weak_ptr cameraData = latestFrame->get_camera_data();
  const auto intrinsicsOut = SensorData::get_intrinsics(cameraData);

  if (intrinsicsOut == fail_t::NO_FAIL) {
    const intrinsics_t& intrinsics = intrinsicsOut;
    const output_t<array2Size_t> imageSize =
        SensorData::get_image_size(cameraData);
    if (imageSize.fail()) {
      log::warning("Failed to get image size from camera data", log::TRACKING);
      return;
    }
    const cell_id_t::set visibleCells = _map->get_visible_cells(
        sharedSettings->spatial_query_cells(),
        intrinsics,
        imageSize,
        priorPose_);

    // Get points from cells
    point_id_t::set pointIdsRepresentation;
    for (const cell_id_t& cellId : visibleCells) {
      const point_id_t::vector pointsInCell =
          _map->get_points_in_cell_x(cellId);
      if (pointsInCell.empty()) continue;
      pointIdsRepresentation.insert(pointsInCell.begin(), pointsInCell.end());
    }
    for (const point_id_t& pointId : pointIdsRepresentation) {
      const auto updated = _covisibilityGraph->get_up_to_date_point_id(pointId);
      ___add_mappoint(updated, pointsIdsInBuffer);
    }
  }

  // Get points from the temporal buffer (keyframes in the buffer)
  for (const keyframe_id_t& keyframeBufferId : reverse(_keyframesInBuffer)) {
    // We ask the map for the point from the keyframes in the buffer
    point_id_t::vector pointsInKeyframe =
        _covisibilityGraph->get_point_ids(keyframeBufferId);
    // Go through all points and save the necessary data
    for (const point_id_t& id : pointsInKeyframe) {
      const auto updated = _covisibilityGraph->get_up_to_date_point_id(id);
      ___add_mappoint(updated, pointsIdsInBuffer, keyframeBufferId);
    }
  }
}

// ---------------------------------------------------------------------------
// Fix scale of frame 3D points using map correspondences
// ---------------------------------------------------------------------------
template <typename TransformationType>
void FramesBuffer<TransformationType>::__fix_point3d_scale(
    const Frame::shared_ptr frame_,
    const keyframe_id_t newKeyframeId_) {
  using data_type::SensorData;
  using registration::compute_best_scalar_s;
  using std::views::zip;

  auto _sanity_non_empty_points =
      [&](const point3d_t::vector& mapPoints__,
          const point3d_t::vector& framePoints__) -> bool {
    if (mapPoints__.empty() || framePoints__.empty()) {
      log::warning("No points to compute scale for the frame", log::TRACKING);
      return false;
    }
    return true;
  };

  // When dealing with a monocular camera, we run 3d registration between
  // matching 3d points of the frame and the map to fix the scale of the
  // points in the frame.
  // 1 - We double check the sensor type is monocular
  // 2 - We skip this step for the first keyframe because there are no
  //     points in the map yet.
  // 3 - We create two vectors of points: one from the frame and one from
  //     the map. The points from the map are transformed to the camera
  //     reference frame.
  // 4 - We compute the best scale using a closed form solution.
  // 5 - We apply the scale to the 3d points in the frame.

  const SensorData::SensorType& sensorType = frame_->get_sensor_type();
  if (sensorType != SensorData::MONOCULAR) return;

  if (newKeyframeId_ == keyframe_id_t(0)) return;

  const size_t& numberMappoints = frame_->get_mappoint_ids().size();
  point3d_t::vector framePoints;
  point3d_t::vector mapPoints;
  framePoints.reserve(numberMappoints);
  mapPoints.reserve(numberMappoints);
  const pose_t worldToCamera = frame_->get_pose().get_invert();
  for (const auto& [point, pointId] :
       zip(frame_->get_3d_points_camera_frame(), frame_->get_mappoint_ids())) {
    if (pointId == point_id_t::UNSET) continue;
    const output_t<point3d_t> point3d = _map->get(pointId);
    if (point3d.fail()) continue;
    mapPoints.emplace_back(worldToCamera.transform(point3d));
    framePoints.emplace_back(point);
  }

  if (!_sanity_non_empty_points(mapPoints, framePoints)) return;

  double scale = 1;
  try {
    scale = compute_best_scalar_s(mapPoints, framePoints);
  } catch (const std::exception& e) {
    __throw__(
        "Failed to compute scale for the frame: " + std::string(e.what()));
  }

  frame_->apply_scale_to_points3d(scale);
  return;
}

// ---------------------------------------------------------------------------
// Construct Localization instance
// ---------------------------------------------------------------------------
template <typename TransformationType>
Localization<TransformationType>::Localization(
    const map::Map::shared_ptr map_,
    const typename LocalizationTyped::CovisibilityPointer covisibilityGraph_)
    : _map(map_),
      _covisibilityGraph(covisibilityGraph_),
      _currentCameraWorldPose(pose_t::Identity()),
      _state(NOT_INITIALIZED) {
  _framesBuffer = FramesBufferTyped::make_unique(map_, covisibilityGraph_);
  return;
}

// ---------------------------------------------------------------------------
// Destroy Localization instance
// ---------------------------------------------------------------------------
template <typename TransformationType>
Localization<TransformationType>::~Localization() {

  // 1. Wait for keyframe creation to finish if it's running
  timer_t timer;
  while (__inKeyframeCreation)
    timer.sleep(duration_t::from_ms(10));

  std::println("[Localization] Stopped successfully.");
  return;
}

// ---------------------------------------------------------------------------
// Set visual-place-recognition database for relocalization
// ---------------------------------------------------------------------------
#ifdef VISUAL_PLACE_LOCALIZATION
template <typename TransformationType>
void Localization<TransformationType>::set_keyframe_database(
    const VisualPlaceRecognition::weak_ptr visualPlaceRecognition_) {
  _visualPlaceRecognition = visualPlaceRecognition_;
}
#endif  // end VISUAL_PLACE_LOCALIZATION

// ---------------------------------------------------------------------------
// Get all detected keypoints from latest frame
// ---------------------------------------------------------------------------
template <typename TransformationType>
point2d_t::vector Localization<TransformationType>::get_all_frame_keypoints() {
  return __allDetectedImagePoints;
}

// ---------------------------------------------------------------------------
// Get inlier keypoints from latest pose estimation
// ---------------------------------------------------------------------------
template <typename TransformationType>
point2d_t::vector Localization<TransformationType>::get_inlier_keypoints() {
  return __poseInlierKeypoints;
}

// ---------------------------------------------------------------------------
// Run localization for incoming sensor data
// ---------------------------------------------------------------------------
template <typename TransformationType>
void Localization<TransformationType>::run(
    data_type::SensorData::weak_ptr data_) {

  // -------------------------------------------------------------------------
  // Lambda Functions
  auto ___sanity_frameBuffer_not_empty = [this]() -> void {
    if (_framesBuffer->is_empty())
      __throw__(
          "Failed in Localization::Localization<data_type>::run \
            --> no data retrieved from the buffer");
  };
  auto ___sanity_tracking_status = [this]() -> void {
    if (_state != LocalizationState::NORMAL &&
        _state != LocalizationState::RELOCALIZATION &&
        _state != LocalizationState::NOT_INITIALIZED)
      __throw__("Localization state is invalid");
  };
  // -------------------------------------------------------------------------

#ifdef LOG_LOCALIZATION
  timer_t timer;
#endif

  // This is the main localization loop. This will be called each time
  // new sensor data is available.
  // 1 - Create a new Frame object from the incoming data
  // Depending on the current localization state, run different
  //     operations:
  // NORMAL:
  //   2.1 - Query 3D points from map that are likely to be seen in current.
  //   2.2 - Estimate the pose of the current frame
  //   2.3 - Add the current frame to the VO buffer
  // RELOCALIZATION:
  //   3.1 - Run relocalization to estimate the pose of the current frame
  //   3.2 - If relocalization is successful, reset the VO buffer
  //         and add the current frame to it
  // NOT_INITIALIZED:
  //   4.1 - Add the current frame to the VO buffer
  //   4.2 - Switch to NORMAL state
  // 5 - Save all detected image points for external use (visualization etc)

  ___sanity_tracking_status();

  // 1 ------------------------------------------------------------------------
  output_t<Frame::shared_ptr> frame = __create_frame(data_);
  if (frame.fail()) return;

  // Save all detected keypoints (for visualization)
  __allDetectedImagePoints = frame->get_all_detected_image_points();

  // 2 ------------------------------------------------------------------------
  if (_state == LocalizationState::NORMAL) {
    const auto& cameraWorldPose = this->get_camera_world_pose();
    _framesBuffer->query_3d_points(cameraWorldPose);
    ___sanity_frameBuffer_not_empty();
    __localize(frame);
    std::ignore = _framesBuffer->insert_new_frame(frame, _state);
    const bool trackingNormal = _framesBuffer->check_tracking_state();
    if (!trackingNormal) _state = LocalizationState::RELOCALIZATION;
    goto DONE;
  }

  // 3 ------------------------------------------------------------------------
  if (_state == LocalizationState::RELOCALIZATION) {
#ifdef VISUAL_PLACE_LOCALIZATION
    __relocalize(frame);
    const keyframe_id_t& keyframeId = frame->get_reference_keyframe_id();
    if (keyframeId != keyframe_id_t::UNSET) {
      _framesBuffer->reset_buffer();
      const bool wasKeyframeCreated =
          _framesBuffer->insert_new_frame(frame, _state, true);
      if (!wasKeyframeCreated) {
        _framesBuffer->reset_buffer();
        log::warning(
            "Failed to create the keyframe after relocalization. "
            "Staying in RELOCALIZATION state.",
            log::Module::TRACKING);
      } else
        _state = LocalizationState::NORMAL;
    }
#else
    log::warning(
        "Relocalization requested but not enabled. "
        "Compile with VISUAL_PLACE_LOCALIZATION=ON to enable it.",
        log::Module::TRACKING);
#endif  // VISUAL_PLACE_LOCALIZATION
    goto DONE;
  }

  // 4 ------------------------------------------------------------------------
  if (_state == LocalizationState::NOT_INITIALIZED) {
    const bool wasKeyframeCreated =
        _framesBuffer->insert_new_frame(frame, _state, true);
    if (!wasKeyframeCreated) {
      _framesBuffer->reset_buffer();
      log::warning(
          "Failed to create the first keyframe. "
          "Staying in NOT_INITIALIZED state.",
          log::Module::TRACKING);
    } else
      _state = LocalizationState::NORMAL;
    goto DONE;
  }

DONE:
#ifdef LOG_LOCALIZATION
  if (_state == LocalizationState::NORMAL)
    log::message("Running NORMAL", log::TRACKING);
  if (_state == LocalizationState::RELOCALIZATION)
    log::message("Running RELOCALIZATION", log::TRACKING);
  if (_state == LocalizationState::NOT_INITIALIZED)
    log::message("Running NOT_INITIALIZED", log::TRACKING);
  log::message("All took " + (std::string)timer.toc(), log::TRACKING);
#endif
}

// ---------------------------------------------------------------------------
// Create Frame object from sensor data
// ---------------------------------------------------------------------------
template <typename TransformationType>
output_t<Frame::shared_ptr> Localization<TransformationType>::__create_frame(
    data_type::SensorData::weak_ptr data_) {

  output_t<Frame::shared_ptr> output = fail_t(fail_t::NO_FAIL);

#ifdef LOG_LOCALIZATION
  timer_t timer;
#endif

  // This function:
  // 1 - Creates a new frame object from the incoming data
  // 2 - Checks if the frame was correctly created (features extracted
  //     and matches found). If not, returns a fail output.

  Frame::shared_ptr frame = Frame::make_shared(data_);

  if (!frame->check_frame_success())
    output = fail_t(fail_t::FAIL);
  else
    output = frame;

#ifdef LOG_LOCALIZATION
  if (output.fail())
    log::warning(
        "Frame " + std::string(frame->get_id()) + " failed to be created",
        log::Module::TRACKING);
  else
    log::message(
        "Created new frame with Id: " + std::string(frame->get_id()) +
            "; took " + (std::string)timer.toc(),
        log::TRACKING);
#endif

  return output;
}

// ---------------------------------------------------------------------------
// Localize current frame against buffered world data
// ---------------------------------------------------------------------------
template <typename TransformationType>
void Localization<TransformationType>::__localize(
    const Frame::shared_ptr frame_) {

  // Get data from the buffer
  const WorldData& _bufferData = _framesBuffer->get_world_data();

  // Run matching and absolute pose estimation
  std::vector<bool> inliers;
  match_t<index_t>::vector matches;
  const bool estimationSuccess =
      __pose_estimation(frame_, _bufferData, inliers, matches, true);

  if (!estimationSuccess) {
    log::warning("Pose estimation fail", log::Module::TRACKING);
    frame_->set_pose(_currentCameraWorldPose);
    return;
  }

  // If we reach this point, pose estimation was successful
  // we set the reference keyframe as the keyframe with the most inlier
  // points shared with the frame
  __select_reference_keyframe(frame_, _bufferData, inliers, matches);

#ifdef LOG_LOCALIZATION
  log::message(
      "Pose estimation: reference keyframe = " +
          std::string(frame_->get_reference_keyframe_id()),
      log::TRACKING);
#endif

  return;
}

// ---------------------------------------------------------------------------
// Attempt relocalization using visual place recognition
// ---------------------------------------------------------------------------
#ifdef VISUAL_PLACE_LOCALIZATION
template <typename TransformationType>
void Localization<TransformationType>::__relocalize(
    const Frame::shared_ptr frame_) {

  using data_type::SensorData;
  using slam_mer::settings::sharedSettings;
  using vpl::visual_place_recognition::query_results;

  // To be able to run relocalization, we need to lock the visual place
  // recognition module. If we fail to lock it, we cannot run
  // relocalization, so we log a warning and return.
  VisualPlaceRecognition::shared_ptr visualPlaceRecognition =
      _visualPlaceRecognition.lock();
  if (!visualPlaceRecognition) {
    log::warning(
        "Failed to lock visual place recognition module for relocalization",
        log::Module::TRACKING);
    return;
  }

  // -------------------------------------------------------------------------
  // Lambda functions

  // compute image descriptor for the frame
  auto _compute_image_descriptor =
      [visualPlaceRecognition](
          const Frame::shared_ptr frame__) -> std::vector<float> {
    std::vector<std::vector<float>> frameDescriptors =
        descriptor_t::to_float_vector(
            frame__->get_all_detected_image_descriptors());
    SensorData::weak_ptr sensorData = frame__->get_camera_data();
    if (auto shared = sensorData.lock()) {
      vpl::visual_place_recognition::Image vplImage =
          shared->get_color_image().convert_to_vpl_type();
      return visualPlaceRecognition->compute_descriptor(
          vplImage,
          frameDescriptors);
    } else {
      log::warning(
          "Failed to lock sensor data for frame " +
              std::string(frame__->get_id()),
          log::Module::TRACKING);
      return std::vector<float>();
    }
  };

  // query database for similar keyframes
  auto _query_database =
      [visualPlaceRecognition](
          std::vector<float> imageDescriptor__) -> query_results {
    return visualPlaceRecognition->query_database(
        keyframe_id_t::FAIL_TO_CREATE,
        imageDescriptor__,
        sharedSettings->relocalization_query_top_k());
  };

  // select candidates from query results
  auto _select_candidate_keyframes =
      [&](const query_results& queryResults__,
          const std::vector<float>& imageDescriptor__,
          keyframe_id_t::vector& candidateKeyframes__) {
        // For each of the query results, we need to check if the similarity
        // score is acceptable given the connected keyframes.

        for (const auto& [keyframeId, score] : queryResults__) {
          keyframe_id_t queryKeyframeId = keyframe_id_t(keyframeId);
          descriptor_t::vector connectedImageDescriptors;

          // First, get the connected keyframes of the query keyframe
          // We use the descriptors of these keyframes to compute a threshold
          // for the similarity score
          keyframe_id_t::vector connectedKeyframes =
              _covisibilityGraph->get_connection_ids(queryKeyframeId);
          for (const keyframe_id_t& id : connectedKeyframes) {
            output_t<keyframe_t::weak_ptr> connectedKeyframe = _map->at(id);
            if (connectedKeyframe.fail()) continue;
            const output_t<descriptor_t> connectedImageDescriptor =
                keyframe_t::get_image_descriptor(connectedKeyframe);
            if (connectedImageDescriptor.fail()) continue;
            connectedImageDescriptors.push_back(connectedImageDescriptor);
          }

          if (connectedImageDescriptors.empty()) continue;

          // Second, we compute a similarity score threshold from the
          // connected keyframes image descriptors
          double thresholdSimilarityScore =
              visualPlaceRecognition->compute_similarity_score(
                  imageDescriptor__,
                  descriptor_t::to_float_vector(connectedImageDescriptors),
                  sharedSettings->relocalization_similarity_score_factor());

#ifdef LOG_LOCALIZATION
          log::message(
              "Relocalization: query keyframe = " +
                  std::string(queryKeyframeId) + "; score " +
                  std::to_string(score) +
                  "; threshold = " + std::to_string(thresholdSimilarityScore),
              log::TRACKING);
#endif

          // Finally, we compare the similarity score obtained from the query
          // search
          // with the computed threshold
          if (!visualPlaceRecognition->valid_similarity_score(
                  score,
                  thresholdSimilarityScore)) {
#ifdef LOG_LOCALIZATION
            log::message(
                "Relocalization: query keyframe " +
                    std::string(queryKeyframeId) +
                    " failed to relocalize [similarity threshold criterion]",
                log::TRACKING);
#endif
            continue;
          }

          // If it passes similarity score check, save as candidate
          candidateKeyframes__.push_back(queryKeyframeId);
        }
        return;
      };

  // get data from the candidate keyframe
  auto _get_keyframe_data = [this](
                                const keyframe_id_t keyframeId__,
                                WorldData& keyframeData__) -> void {
    // Go through all points of the keyframe and save
    // 1. 3d points
    // 2. descriptors
    // 3. keyframe observations
    // 4. map point id
    point_id_t::vector candidatePointIds =
        _covisibilityGraph->get_point_ids(keyframeId__);

    output_t<keyframe_t::weak_ptr> keyframe = _map->at(keyframeId__);
    if (keyframe.fail()) return;
    output_t<pose_t::weak_ptr> keyframePose = keyframe_t::get_pose(keyframe);
    if (keyframePose.fail()) return;

    // Go through all points
    for (auto& pointId : candidatePointIds) {
      // Get and validate all data before sync
      output_t<descriptor_t> pointDescriptor =
          _covisibilityGraph->get_descriptor_observation(pointId, keyframePose);
      if (pointDescriptor.fail()) continue;
      output_t<point3d_t::weak_ptr> worldPoint = _map->at(pointId);
      if (worldPoint.fail()) continue;
      keyframe_id_t::vector observations =
          _covisibilityGraph->get_keyframe_ids(pointId);
      if (observations.empty()) continue;
      // Sync all
      keyframeData__
          .push_back(worldPoint, pointDescriptor, observations, pointId);
    }
    return;
  };

  // -------------------------------------------------------------------------
  // This function:
  // 1 - Computes the image descriptor for the current frame
  // 2 - Queries the keyframe database for similar keyframes
  // 3 - For each similar keyframe, checks if the similarity score is
  //     acceptable given the connected keyframes (ie, select candidates)
  // 4 - For each candidate keyframe, runs pose estimation
  // 5 - If pose estimation is successful, sets a pose and reference keyframe
  //     for the current frame and exits

  // Compare image descriptor with database
  std::vector<float> imageDescriptor = _compute_image_descriptor(frame_);
  query_results queryResults = _query_database(imageDescriptor);

  // Select keyframe candidates based on similarity score
  keyframe_id_t::vector candidateKeyframes;
  _select_candidate_keyframes(
      queryResults,
      imageDescriptor,
      candidateKeyframes);

  // For every candidate, we need to run pose estimation to check if
  // relocalization is successful.
  // As soon as we have one successfull pose estimation, we exit
  for (const auto& candidateId : candidateKeyframes) {

    // First we need to collect all the data for pose estimation
    WorldData keyframeData;
    _get_keyframe_data(candidateId, keyframeData);
    if (keyframeData.empty()) continue;

    // Run matching and absolute pose estimation
    std::vector<bool> inliers;
    match_t<index_t>::vector matches;
    const bool estimationSuccess =
        __pose_estimation(frame_, keyframeData, inliers, matches, false);

    if (!estimationSuccess) {
#ifdef LOG_LOCALIZATION
      log::message(
          "Relocalization: query keyframe " + std::string(candidateId) +
              " failed to relocalize",
          log::TRACKING);
#endif
      continue;
    }

    // If we reach this point, relocalization was successful
    // we set the reference keyframe as the keyframe with the most inlier
    // points shared with the frame
    __select_reference_keyframe(frame_, keyframeData, inliers, matches);

#ifdef LOG_LOCALIZATION
    log::message(
        "Relocalization: query keyframe " + std::string(candidateId) +
            " relocalized succesfully. Reference keyframe = " +
            std::string(frame_->get_reference_keyframe_id()),
        log::TRACKING);
#endif

    break;
  }
  return;
}
#endif  // end VISUAL_PLACE_LOCALIZATION

// ---------------------------------------------------------------------------
// Estimate frame pose from 2D-3D correspondences
// ---------------------------------------------------------------------------
template <typename TransformationType>
bool Localization<TransformationType>::__pose_estimation(
    const Frame::shared_ptr frame_,
    const WorldData& worldData_,
    std::vector<bool>& inliers_,
    match_t<index_t>::vector& matches_,
    const bool checkPoseLimits_) {
  using data_type::SensorData;
  using slam_mer::settings::sharedSettings;

  // update global variables. Save previous state and reset current
  __auxPreviousKeyframeReferenceCount = __auxKeyframeReferenceCount;
  __auxKeyframeReferenceCount.clear();
  __poseInlierKeypoints.clear();

  // Get data from the frame
  const point2d_t::vector& points2d = frame_->get_all_detected_image_points();
  const descriptor_t::vector& points2dDescriptors =
      frame_->get_all_detected_image_descriptors();

  // Get data from the world
  const vector_t<point3d_t::weak_ptr>& worldPoints = worldData_.points();
  const descriptor_t::vector& worldPointsDescriptors = worldData_.descriptors();

  // Run matching and absolute pose estimation
  // if we do not have camera intrinsic parameters, focal length is also
  // estimated and the principal point is the center of the image
  const SensorData::weak_ptr cameraData = frame_->get_camera_data();
  const bool knownFocalLength = SensorData::calibrated(cameraData);

  pose_t pose;
  bool estimationSuccess = pose_estimation::compute_absolute_pose(
      worldPoints,
      worldPointsDescriptors,
      points2d,
      points2dDescriptors,
      cameraData,
      pose,      // output variable
      inliers_,  // output variable
      matches_,  // output variable
      knownFocalLength,
      log::TRACKING);

  // Check pose variation limits
  if (checkPoseLimits_) {
    const array2D_t poseErrors =
        pose_t::computeError(pose.get_invert(), __previousFramePose);
    const array2D_t poseErrorsLimits = sharedSettings->pose_variation_limits();
    if (poseErrors[0] > poseErrorsLimits[0] ||
        poseErrors[1] > poseErrorsLimits[1]) {
      estimationSuccess = false;
    }
  }

  // If pose estimation was not successful, the global camera pose is kept
  // and the frame gets the current global pose
  if (!estimationSuccess) {
    frame_->set_pose(_currentCameraWorldPose);
    return false;
  }

  // Update global camera pose if the pose estimation was successful

  // Returned pose is from world to camera, invert to get camera position
  // in world coordinates
  pose.invert();

  // save pose of the previous frame
  __previousFramePose = _currentCameraWorldPose;

  // Update current frame world pose
  _currentCameraWorldPose = pose;
  frame_->set_pose(pose);

  return true;
}

// ---------------------------------------------------------------------------
// Select reference keyframe for current frame
// ---------------------------------------------------------------------------
template <typename TransformationType>
void Localization<TransformationType>::__select_reference_keyframe(
    const Frame::shared_ptr frame_,
    const WorldData& worldData_,
    const std::vector<bool>& inliers_,
    const match_t<index_t>::vector& matches_) {

  // This function:
  // Decides which keyframe is the reference keyframe based on the
  // number of inlier points shared with the current frame.

  const std::vector<keyframe_id_t::vector>& mapPointObservations =
      worldData_.observations();
  const point_id_t::vector& mapPointIds = worldData_.point_ids();

  // setup for the counting to obtain the reference keyframe
  __auxKeyframeReferenceCount.clear();
  for (size_t i = 0; i < __totalKeyframesCreated; i++)
    __auxKeyframeReferenceCount.emplace(keyframe_id_t(i), 0);

  // Run through the pose estimation inliers
  // If the point is an inlier, we:
  // 1. Set in the frame data that the point is already a map point
  // 2. Count the number of inlier points from each of the keyframes in
  // the buffer
  // 3. Save inliers to visualize on the localization image

  point2d_t::vector points2d = frame_->get_all_detected_image_points();
  descriptor_t::vector descriptors =
      frame_->get_all_detected_image_descriptors();

  int inlierCount = 0;
  for (size_t i = 0; i < inliers_.size(); i++) {
    // If the point is an outlier, we mark it as such in the current frame
    // and continue to the next point
    if (!inliers_[i]) {
      frame_->update_point_id(
          (size_t)matches_[i].first_element(),
          point_id_t::UNSET);
      continue;
    }

    // Set in the frame data that the point is already a map point
    frame_->update_point_id(
        (size_t)matches_[i].first_element(),
        mapPointIds[(size_t)matches_[i].second_element()]);

    // Count the number of inlier points from each of the keyframe
    keyframe_id_t::vector observations =
        mapPointObservations[(size_t)matches_[i].second_element()];
    // count observations
    for (keyframe_id_t::vector::iterator observations_it = observations.begin();
         observations_it != observations.end();
         observations_it++) {
      const keyframe_id_t keyframeId = *observations_it;
      __auxKeyframeReferenceCount[keyframeId] += 1;
    }

    // Update the descriptor of the map point with the new observation
    _covisibilityGraph->fit_new_descriptor(
        mapPointIds[(size_t)matches_[i].second_element()],
        descriptors[(size_t)matches_[i].first_element()],
        frame_->get_pose());

    // Save inliers to visualize on the localization image
    const size_t& matchIndex = static_cast<size_t>(matches_[i].first_element());
    const point2d_t& inlierKeypoint = points2d[matchIndex];
    __poseInlierKeypoints.push_back(inlierKeypoint);
    inlierCount++;
  }

  // Set the number of tracked points (used in keyframe creation decision)
  frame_->set_number_tracked_points(inlierCount);

  // Find which keyframe has the most points tracked.
  // That keyframe will be the reference keyframe
  keyframe_id_t _referenceKeyframe;
  int mostObservation = -1;
  for (const auto& [id, count] : __auxKeyframeReferenceCount) {
    if (count >= mostObservation) {
      mostObservation = count;
      _referenceKeyframe = id;
    }
  }

  // set the frame reference keyframe
  frame_->set_reference_keyframe_id(_referenceKeyframe);
  return;
}

#ifdef SLAM_MER_COMPILE_SIMILARITY3
template class FramesBuffer<gtsam::Similarity3>;
template class Localization<gtsam::Similarity3>;
#endif
#ifdef SLAM_MER_COMPILE_POSE3
template class FramesBuffer<gtsam::Pose3>;
template class Localization<gtsam::Pose3>;
#endif

}  // namespace localization
}  // namespace slam_mer
