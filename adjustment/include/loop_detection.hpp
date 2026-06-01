/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file loop_detection.hpp
/// @brief Loop-detection module API for recognizing and closing map loops.
/// @details Declares loop-detection interfaces that monitor keyframe history,
/// identify loop candidates, and trigger correction workflows through
/// covisibility-graph integration.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef LOOP_DETECTION_HPP
#define LOOP_DETECTION_HPP

#include <covisibility_graph.hpp>
#include <map.hpp>
#include <types/match.hpp>

#ifdef VISUAL_PLACE_LOCALIZATION
#include <vpl/visual_place_recognition.hpp>
#endif

namespace slam_mer {

/// loop_detection namespace.
/// This namespace contains the LoopDetection module class and any other
/// operation solely related to closing and/or correcting loops.
namespace loop_detection {

#ifdef VISUAL_PLACE_LOCALIZATION

using vpl::visual_place_recognition::VisualPlaceRecognition;

/// @brief Detects and validates loop closures for incoming keyframes.
/// @details The module receives keyframes scheduled for loop verification,
/// queries visual place recognition for loop candidates, validates each
/// candidate geometrically, and pushes validated loops to the covisibility
/// graph for correction.
template <typename TransformationType>
class LoopDetection {
public:
  /// Typed alias for the LoopDetection class template
  using LoopDetectionTyped = LoopDetection<TransformationType>;
  /// Auxiliary typedefs for covisibility graph point definition
  using __covisibilityTyped =
      covisibility_graph::CovisibilityGraph<TransformationType>;
  /// Shared pointer alias for the covisibility graph.
  using CovisibilityGraphPtr = __covisibilityTyped::shared_ptr;

  /// Typedef for a std::unique_ptr of type LoopDetection
  using unique_ptr = std::unique_ptr<LoopDetectionTyped>;
  /// Typedef for a std::shared_ptr of type LoopDetection
  using shared_ptr = std::shared_ptr<LoopDetectionTyped>;
  /// Typedef for a std::weak_ptr of type LoopDetection
  using weak_ptr = std::weak_ptr<LoopDetectionTyped>;

  /// @brief Create a new unique pointer to a LoopDetection object.
  /// @param map_ Slam system map.
  /// @param covisibilityGraph_ Slam system covisibility graph.
  /// @param visualPlaceRecognition_ Slam system database of keyframes.
  /// @return Unique pointer owning the created loop-detection module.
  [[nodiscard("Use returned unique_ptr to keep loop-detection ownership")]]
  static unique_ptr make_unique(
      const map::Map::shared_ptr map_,
      const CovisibilityGraphPtr covisibilityGraph_,
      const VisualPlaceRecognition::weak_ptr visualPlaceRecognition_) {
    unique_ptr loop(new LoopDetectionTyped(
        std::move(map_),
        std::move(covisibilityGraph_),
        std::move(visualPlaceRecognition_)));
    loop->__start_thread();
    return loop;
  }

  /// @brief Creates a new shared pointer to a LoopDetection object.
  /// @param map_ Slam system map.
  /// @param covisibilityGraph_ Slam system covisibility graph.
  /// @param visualPlaceRecognition_ Slam system database of keyframes.
  /// @return Shared pointer owning the created loop-detection module.
  [[nodiscard("Use returned shared_ptr to keep loop-detection ownership")]]
  static shared_ptr make_shared(
      map::Map::shared_ptr map_,
      CovisibilityGraphPtr covisibilityGraph_,
      VisualPlaceRecognition::shared_ptr visualPlaceRecognition_) {
    shared_ptr loop = shared_ptr(new LoopDetectionTyped(
        std::move(map_),
        std::move(covisibilityGraph_),
        std::move(visualPlaceRecognition_)));
    loop->__start_thread();
    return loop;
  }

  /// Destroy the LoopDetection module.
  ~LoopDetection();

  /// @brief Adds a keyframe for loop detection.
  /// This function appends the given keyframe ID to a list. All keyframes in
  /// that list will be sequentially verified for loops.
  /// @param keyframeId_ Id of the keyframe to be considered for loop detection.
  void add_to_loop_verification(const keyframe_id_t keyframeId_);

private:
  /// @brief Construct a Loop Closure module.
  /// @param map_ Slam system map.
  /// @param covisibilityGraph_ Slam system covisibility graph.
  /// @param visualPlaceRecognition_ Slam system database of keyframes.
  explicit LoopDetection(
      map::Map::shared_ptr map_,
      CovisibilityGraphPtr covisibilityGraph_,
      VisualPlaceRecognition::weak_ptr visualPlaceRecognition_);

  /// Starts the loop detection worker thread.
  void __start_thread();

  /// @brief Main loop-detection worker function.
  /// @param token_ Stop token used to terminate the worker thread.
  void __run(std::stop_token token_);

  /// @brief Gets the next keyframe to check for a loop.
  /// @return ID of the next keyframe to check for a loop.
  [[nodiscard("Use returned keyframe output to handle queue-empty failures")]]
  output_t<keyframe_id_t> __get_next_keyframe();

  /// @brief Adds a keyframe to the database for visual place recognition.
  /// @param keyframeId_ Id of the keyframe to add to the database.
  /// @return True if the keyframe was added successfully, false otherwise.
  [[nodiscard("Use returned bool to branch on database insertion status")]]
  bool __add_to_database(const keyframe_id_t keyframeId_);

  /// @brief Detects loop candidates (ie, keyframes similar to the input
  /// keyframe) using visual place recognition.
  /// @param keyframeId_ Id of the keyframe used to query loop candidates.
  /// @return List with the Ids of the keyframes most similar with the
  /// input keyframe.
  [[nodiscard("Use returned candidates to drive loop-validation attempts")]]
  keyframe_id_t::vector __detect_loop(const keyframe_id_t keyframeId_);

  /// @brief Validates loops candidates. Loop candidates are validated by
  /// 1) matching the eval keyframe with the candidate keyframe and
  /// verifying if there are enough matched points;
  /// 2) estimating a valid pose between both keyframes (uses threshold for
  /// eg, minimum inlier rate and/or minimum number of inliers).
  /// @param evalKeyframeId_ Id of the evaluation keyframe (the current keyframe
  /// that we are trying to find a loop).
  /// @param candidateKeyframeId_ Id of the loop candidate keyframe.
  void __validate_loop(
      const keyframe_id_t evalKeyframeId_,
      const keyframe_id_t candidateKeyframeId_);

  /// Pointer to the SLAM map
  map::Map::shared_ptr _map;
  /// Pointer to the covisibility graph
  CovisibilityGraphPtr _covisibilityGraph;
  /// Pointer to the BoW keyframe database
  VisualPlaceRecognition::weak_ptr _visualPlaceRecognition;
  /// This thread is used to run the loop detection module
  std::jthread _loopDetectionThread;
};

#endif  // end VISUAL_PLACE_LOCALIZATION

}  // namespace loop_detection
}  // namespace slam_mer

#endif  // LOOP_CLOSURE_HPP
