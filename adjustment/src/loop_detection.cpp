/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file loop_detection.cpp
/// @brief Implements loop-detection execution and correction triggers.
/// @details Defines loop candidate validation and closure dispatch logic.
/// @author Pedro Miraldo
/// @author Valter Piedade

// Project includes
#include <keyframe.hpp>
#include <loop_detection.hpp>
#include <pose_estimation.hpp>
#include <settings.hpp>
#include <types/fail.hpp>
#include <types/point2d.hpp>
#include <types/point3d.hpp>

// Thirdparty libs
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>

namespace slam_mer {
namespace loop_detection {

#ifdef VISUAL_PLACE_LOCALIZATION

// ---------------------------------------------------------------------------
// Instantiate LoopDetection object
// ---------------------------------------------------------------------------
template <typename TransformationType>
LoopDetection<TransformationType>::LoopDetection(
    map::Map::shared_ptr map_,
    typename LoopDetectionTyped::CovisibilityGraphPtr covisibilityGraph_,
    VisualPlaceRecognition::weak_ptr visualPlaceRecognition_)
    : _map(map_),
      _covisibilityGraph(covisibilityGraph_),
      _visualPlaceRecognition(visualPlaceRecognition_) {}

// ---------------------------------------------------------------------------
// Destroy LoopDetection object
// ---------------------------------------------------------------------------
template <typename TransformationType>
LoopDetection<TransformationType>::~LoopDetection() {

  // 1. Wait until all edges are added
  timer_t timer;
  while (_covisibilityGraph->is_loop_queue_empty() == false)
    timer.sleep(duration_t::from_ms(10));

  // 2. Stop the thread that adds new edges to the graph
  if (_loopDetectionThread.joinable()) {
    _loopDetectionThread.request_stop();
    _loopDetectionThread.join();
  }
  std::println("[LoopDetection] Stopped successfully.");
  return;
}

// ---------------------------------------------------------------------------
// Start the loop closure worker thread
// ---------------------------------------------------------------------------
template <typename TransformationType>
void LoopDetection<TransformationType>::__start_thread() {
  if (_loopDetectionThread.joinable()) {
    _loopDetectionThread.request_stop();
    _loopDetectionThread.join();
  }
  _loopDetectionThread =
      std::jthread([this](std::stop_token token__) { __run(token__); });
}

// ---------------------------------------------------------------------------
// Run the loop closure module
// ---------------------------------------------------------------------------
template <typename TransformationType>
void LoopDetection<TransformationType>::__run(std::stop_token token_) {
  using modes::SystemMode;
  using slam_mer::settings::sharedSettings;

  auto ___log = [](const duration_t& duration__) -> void {
#ifdef LOG_TIME
    log::message(
        "Loop closure toc: " + std::string(duration__),
        log::LOOP_CLOSURE);
#endif
  };

  // This function:
  // 1 - Runs in a separate thread and continuously checks for new keyframes to
  //     evaluate for loop closure.
  // 2 - For each keyframe, it first adds it to the visual place recognition
  //     database.
  // 3 - Then, it queries the database for loop candidates and
  //     validates them geometrically.
  // 4 - The geometric validation consists of matching features between the
  //     current keyframe and each candidate keyframe, followed by pose
  //     estimation to verify the loop closure.
  // 5 - If a loop is validated, the map points of the current keyframe are
  //     merged with their correspondences in the candidate keyframe, and an
  //     odometry factor between both loop keyframes is added to the graph.

  timer_t timer;
  const int minimumFramesPerSecond = sharedSettings->loop_module_fps();

  while (token_.stop_requested() == false) {
    timer.tic();

    while (_covisibilityGraph->is_loop_queue_empty() == false) {

      const output_t<keyframe_id_t> evalKeyframeId = __get_next_keyframe();
      if (evalKeyframeId != fail_t::NO_FAIL) continue;

      const bool addedToDatabase = __add_to_database(evalKeyframeId);
      if (!addedToDatabase) continue;

      if (sharedSettings->operation_mode() != SystemMode::SLAM) continue;

      const keyframe_id_t::vector candidateIds = __detect_loop(evalKeyframeId);
      for (const keyframe_id_t& candidateKeyframeId : candidateIds)
        __validate_loop(evalKeyframeId, candidateKeyframeId);
    }

    timer.pace(duration_t::from_fps(minimumFramesPerSecond));
    ___log(timer.toc());
  }
}

// ---------------------------------------------------------------------------
// Gets the first loop detection candidate.
// ---------------------------------------------------------------------------
template <typename TransformationType>
output_t<keyframe_id_t>
LoopDetection<TransformationType>::__get_next_keyframe() {
  return _covisibilityGraph->next_loop_verification();
}

// ---------------------------------------------------------------------------
// Add keyframe data to visual place recognition database
// ---------------------------------------------------------------------------
template <typename TransformationType>
bool LoopDetection<TransformationType>::__add_to_database(
    const keyframe_id_t keyframeId_) {

  // ----------------------------------------------------------------------
  // Lambda functions
  auto __fail_vpr = [](const VisualPlaceRecognition::shared_ptr& ptr__,
                       const keyframe_id_t& keyframeId__) -> bool {
    if (!ptr__) {
      log::warning(
          "Loop Detection: Failed to lock visual place recognition module to "
          " add " +
              std::string(keyframeId__) + ".",
          log::LOOP_CLOSURE);
      return true;
    }
    return false;
  };
  // ----------------------------------------------------------------------

  // This function:
  // 1 - Gets the visual place recognition module and the keyframe from the map.
  // 2 - Gets the keyframe image and descriptor.
  // 3 - Adds the keyframe to the visual place recognition database.
  //     Depending on the visual place recognition approach either the
  //     keyframe image or the detected descriptors are needed, send both.
  // 4 - Saves the visual descriptor in the keyframe as well.

  const auto vpr = _visualPlaceRecognition.lock();
  if (__fail_vpr(vpr, keyframeId_)) return false;

  const auto keyframe = _map->at(keyframeId_);
  if (__fail_retrieve(keyframe, keyframeId_)) return false;

  const output_t<image_t> image = keyframe_t::get_color_image(keyframe);
  const auto descriptors = keyframe_t::get_descriptors(keyframe);
  if (image != fail_t::NO_FAIL) return false;
  if (__fail_retrieve(descriptors, keyframeId_)) return false;

  std::vector<float> imageDescriptor;
  const auto floatDescriptors = descriptor_t::to_float_vector(descriptors);
  const vpl::visual_place_recognition::Image vplImage =
      image.get().convert_to_vpl_type();
  vpr->add_to_database(vplImage, floatDescriptors, imageDescriptor);

  const auto descriptor = descriptor_t(imageDescriptor);
  const fail_t fail = keyframe_t::set_image_descriptor(keyframe, descriptor);
  if (fail != fail_t::NO_FAIL) return false;

  return true;
}

// ---------------------------------------------------------------------------
// Detects/finds loop candidates
// ---------------------------------------------------------------------------
template <typename TransformationType>
keyframe_id_t::vector LoopDetection<TransformationType>::__detect_loop(
    const keyframe_id_t keyframeId_) {
  using slam_mer::settings::sharedSettings;
  using vpl::visual_place_recognition::query_results;

  // -----------------------------------------------------------------------
  // Lambda functions
  auto __fail_vpr = [](const VisualPlaceRecognition::shared_ptr& ptr__,
                       const keyframe_id_t& keyframeId__) -> bool {
    if (!ptr__) {
      log::warning(
          "Loop Detection: Failed to lock visual place recognition module to "
          " detect loops for " +
              std::string(keyframeId__) + ".",
          log::LOOP_CLOSURE);
      return true;
    }
    return false;
  };
  auto ___log_running = [](const unsigned int& id__,
                           const double& score__,
                           const double& threshold__,
                           const bool& isConnected__) -> void {
#ifdef LOG_ADJUSTMENT
    log::message(
        "Database query: --keyframe id: " + std::to_string(id__) +
            "; --score: " + std::to_string(score__) +
            "; threshold: " + std::to_string(threshold__) +
            "; isConnected: " + std::to_string(isConnected__),
        log::LOOP_CLOSURE);
#endif
  };
  auto ___log_starting = [](const keyframe_id_t& keyframeId__) -> void {
#ifdef LOG_ADJUSTMENT
    log::message(
        "Loop Closure is checking for loops for " + std::string(keyframeId__),
        log::LOOP_CLOSURE);
#endif
  };
  auto ___log_finished =
      [](const keyframe_id_t::vector& potentialConnected__) -> void {
#ifdef LOG_ADJUSTMENT
    std::string candidateIds = "";
    for (keyframe_id_t candidate : potentialConnected__)
      candidateIds += candidate.to_string() + ", ";
    log::message("Candidates: " + candidateIds, log::LOOP_CLOSURE);
#endif
  };
  // -----------------------------------------------------------------------

  // This function:
  // 1 - Gets the visual place recognition module and the keyframe from the map.
  // 2 - Gets the keyframe image descriptor.
  // 3 - Queries the visual place recognition database for similar keyframes.
  // 4 - Validates the retrieved keyframes based on their similarity score and
  //     the threshold computed from the similarity scores of the connected
  //     keyframes. Also, already connected keyframes are not considered as
  //     loop candidates.
  // 5 - Returns the list of potential loop candidates.

  ___log_starting(keyframeId_);

  keyframe_id_t::vector potentialConnected;

  VisualPlaceRecognition::shared_ptr vpr = _visualPlaceRecognition.lock();
  if (__fail_vpr(vpr, keyframeId_)) return potentialConnected;

  const auto keyframe = _map->at(keyframeId_);
  __sanity_retrieve(keyframe, keyframeId_);

  const auto descriptor_out = keyframe_t::get_image_descriptor(keyframe);
  __sanity_retrieve(descriptor_out, keyframeId_);

  const descriptor_t& imageDescriptor = descriptor_out;
  const std::vector<float> floats = imageDescriptor.get_data();
  const auto key = keyframeId_.key();
  const auto topK = sharedSettings->loop_query_top_k();
  const query_results queryResults = vpr->query_database(key, floats, topK);

  descriptor_t::vector connectedDescriptors;
  const keyframe_id_t::vector connectedKeyframes =
      _covisibilityGraph->get_connection_ids(keyframeId_);
  for (const keyframe_id_t& id : connectedKeyframes) {

    if (id.key() > keyframeId_.key()) continue;

    const auto connected = _map->at(id);
    __sanity_retrieve(connected, id);

    const auto descriptor = keyframe_t::get_image_descriptor(connected);
    __sanity_retrieve(descriptor, id);

    connectedDescriptors.push_back(descriptor);
  }

  const auto allFloats = descriptor_t::to_float_vector(connectedDescriptors);
  const double similarityScoreFactor =
      sharedSettings->loop_similarity_score_factor();
  const double threshold =
      vpr->compute_similarity_score(floats, allFloats, similarityScoreFactor);

  for (const auto& [id, score] : queryResults) {

    const bool isConnected =
        std::find(connectedKeyframes.begin(), connectedKeyframes.end(), id) !=
        connectedKeyframes.end();

    ___log_running(id, score, threshold, isConnected);

    if (id == keyframeId_.key()) continue;

    if (isConnected) continue;

    const bool validScore = vpr->valid_similarity_score(score, threshold);
    if (!validScore) continue;

    potentialConnected.push_back(id);
  }

  ___log_finished(potentialConnected);

  return potentialConnected;
}

// ---------------------------------------------------------------------------
// Validates loop candidates
// ---------------------------------------------------------------------------
template <typename TransformationType>
void LoopDetection<TransformationType>::__validate_loop(
    const keyframe_id_t evalKeyframeId_,
    const keyframe_id_t candidateKeyframeId_) {
  using data_type::SensorData;
  using std::println;

  // -----------------------------------------------------------------------
  // Lambda functions
  auto ___log_failed_estimation =
      [](const keyframe_id_t& evalKeyframeId__,
         const keyframe_id_t& candidateKeyframeId__) -> void {
#ifdef LOG_ADJUSTMENT
    log::message(
        "[Loop Detection] NO LOOP between keyframes: " +
            evalKeyframeId__.to_string() + "<-> " +
            candidateKeyframeId__.to_string(),
        log::LOOP_CLOSURE);
#endif
  };
  auto ___log_successful_estimation =
      [](const keyframe_id_t& evalKeyframeId__,
         const keyframe_id_t& candidateKeyframeId__) -> void {
#ifdef LOG_ADJUSTMENT
    log::message(
        "[Loop Detection] found between keyframes: " +
            evalKeyframeId__.to_string() + "<-> " +
            candidateKeyframeId__.to_string(),
        log::LOOP_CLOSURE);
#endif
    println(
        "[Loop Detection] found between keyframes: {} <-> {}",
        std::string(evalKeyframeId__),
        std::string(candidateKeyframeId__));
  };
  // -----------------------------------------------------------------------

  // This function:
  // 1 - Gets the keyframe and its pose from the map for both the eval keyframe
  //     and the candidate keyframe.
  // 2 - Gets the 2D keypoints and descriptors of the eval keyframe and the 3D
  //     points and descriptors of the candidate keyframe (Sync).
  // 3 - Runs feature matching and absolute pose estimation between the
  //     eval keyframe and the candidate keyframe.
  // 4 - If the pose estimation is successful, it means that a loop closure
  //     has been found between the eval keyframe and the candidate keyframe, so
  //     we add the loop closure edge to the graph and merge the map points of
  //     both keyframes.
  // 5 - If the pose estimation fails, it means that no loop closure has been
  //     found between the eval keyframe and the candidate keyframe, so we just
  //     return.

  if (evalKeyframeId_ == candidateKeyframeId_)
    __throw__("loop closure between same keyframe!");

  const output_t<keyframe_t::weak_ptr> evalKeyframe = _map->at(evalKeyframeId_);
  const auto candidateKeyframe = _map->at(candidateKeyframeId_);
  __sanity_retrieve(evalKeyframe, evalKeyframeId_);
  __sanity_retrieve(candidateKeyframe, candidateKeyframeId_);

  const auto poseOutut = keyframe_t::get_pose(evalKeyframe);
  __sanity_retrieve(poseOutut, evalKeyframeId_);

  const pose_t evalKeyframePose = pose_t(poseOutut);
  if (__fail_set(evalKeyframePose, evalKeyframeId_)) return;

  const auto evalPointIds = _covisibilityGraph->get_point_ids(evalKeyframeId_);
  const size_t sizeEval = evalPointIds.size();
  descriptor_t::vector evalKeyframeDescriptorsSync;
  point2d_t::vector evalKeyframePointsImageSync;
  point_id_t::vector evalKeyframePointIdsSync;
  evalKeyframeDescriptorsSync.reserve(sizeEval);
  evalKeyframePointsImageSync.reserve(sizeEval);
  evalKeyframePointIdsSync.reserve(sizeEval);

  // 2
  for (const point_id_t& pointId : evalPointIds) {

    const auto pointDescriptor = _covisibilityGraph->get_descriptor_observation(
        pointId,
        evalKeyframePose);

    const auto point2D =
        _covisibilityGraph->get_keypoint_observation(pointId, evalKeyframeId_);

    if (__fail_retrieve(pointDescriptor, evalKeyframeId_)) continue;
    if (__fail_retrieve(point2D, pointId)) continue;

    evalKeyframePointsImageSync.emplace_back(point2D);
    evalKeyframeDescriptorsSync.emplace_back(pointDescriptor);
    evalKeyframePointIdsSync.emplace_back(pointId);
  }
  const auto evalCameraData = keyframe_t::get_camera_data(evalKeyframe);
  __sanity_retrieve(evalCameraData, evalKeyframeId_);

  // 2
  const auto candidatePointIds =
      _covisibilityGraph->get_point_ids(candidateKeyframeId_);
  const size_t sizeCandidate = candidatePointIds.size();
  descriptor_t::vector candidateKeyframeDescriptorsSync;
  vector_t<point3d_t::weak_ptr> candidateworldPointsSync;
  point_id_t::vector candidateKeyframePointIdsSync;
  candidateKeyframeDescriptorsSync.reserve(sizeCandidate);
  candidateworldPointsSync.reserve(sizeCandidate);
  candidateKeyframePointIdsSync.reserve(sizeCandidate);

  const auto candidateKeyframePose = keyframe_t::get_pose(candidateKeyframe);
  __sanity_retrieve(candidateKeyframePose, candidateKeyframeId_);

  for (const point_id_t& pointId : candidatePointIds) {

    const auto worldPoint = _map->at(pointId);
    if (__fail_retrieve(worldPoint, pointId)) continue;

    const output_t<descriptor_t> pointDescriptor =
        _covisibilityGraph->get_descriptor_observation(
            pointId,
            candidateKeyframePose);
    if (__fail_retrieve(pointDescriptor, pointId)) continue;

    candidateKeyframeDescriptorsSync.emplace_back(pointDescriptor);
    candidateworldPointsSync.emplace_back(worldPoint);
    candidateKeyframePointIdsSync.emplace_back(pointId);
  }

  // 3
  pose_t poseWorldToEvalCamera;
  std::vector<bool> inliers;
  match_t<index_t>::vector matches;
  const bool estimationSuccess = pose_estimation::compute_absolute_pose(
      candidateworldPointsSync,
      candidateKeyframeDescriptorsSync,
      evalKeyframePointsImageSync,
      evalKeyframeDescriptorsSync,
      evalCameraData,
      poseWorldToEvalCamera,  // output variable
      inliers,                // output variable
      matches,                // output variable
      true,                   // We don't want to use the uncalibrated pose
      log::LOOP_CLOSURE);

  // 4
  if (!estimationSuccess) {
    ___log_failed_estimation(evalKeyframeId_, candidateKeyframeId_);
    return;
  }
  ___log_successful_estimation(evalKeyframeId_, candidateKeyframeId_);

  _covisibilityGraph->close_loop(
      evalKeyframeId_,
      candidateKeyframeId_,
      poseWorldToEvalCamera,
      evalKeyframePointIdsSync,
      candidateKeyframePointIdsSync,
      matches,
      inliers);

  return;
}

#ifdef SLAM_MER_COMPILE_SIMILARITY3
template class LoopDetection<gtsam::Similarity3>;
#endif
#ifdef SLAM_MER_COMPILE_POSE3
template class LoopDetection<gtsam::Pose3>;
#endif

#endif  // end VISUAL_PLACE_LOCALIZATION

}  // namespace loop_detection
}  // namespace slam_mer
