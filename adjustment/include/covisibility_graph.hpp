/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file covisibility_graph.hpp
/// @brief Covisibility-graph manager for backend optimization workflows.
/// @details Declares the `CovisibilityGraph` class and its public API for
/// ingesting keyframe/point observations, maintaining graph edges, and running
/// optimization-related updates for map consistency.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef COVISIBILITY_GRAPH_HPP
#define COVISIBILITY_GRAPH_HPP

// slam-mer includes
#include <covisibility_factors.hpp>
#include <descriptor_model.hpp>
#include <map.hpp>
#include <observation_edge.hpp>
#include <odometry_edge.hpp>
#include <types/id.hpp>
#include <types/match.hpp>
#include <types/point2d.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>
#include <types/queue.hpp>
#include <types/timeout_queue.hpp>

namespace slam_mer {
namespace covisibility_graph {

/// @brief Manages the covisibility graph used by backend optimization.
/// @details This class receives odometry and point-observation edges from
/// localization, converts them into GTSAM factors, runs optimization in a
/// worker thread, and synchronizes optimized results back to the map.
template <typename TransformationType>
class CovisibilityGraph {

public:
  /// Typedef to a std::shared_ptr of type CovisibilityGraph
  using shared_ptr = std::shared_ptr<CovisibilityGraph>;
  /// Typedef to a std::weak_ptr of type CovisibilityGraph
  using weak_ptr = std::weak_ptr<CovisibilityGraph>;

  /// @brief Allocates a new object of type CovisibilityGraph.
  /// @param map_ Slam system map.
  /// @return Pointer to the new object.
  [[nodiscard("Use returned shared_ptr to keep covisibility-graph ownership")]]
  static CovisibilityGraph::shared_ptr make_shared(
      const map::Map::shared_ptr map_) {
    CovisibilityGraph::shared_ptr graph(new CovisibilityGraph(std::move(map_)));
    graph->__start_thread();
    return graph;
  }

  /// CovisibilityGraph object destructor.
  ~CovisibilityGraph();

  /// @brief The covisibility graph module works in a separate thread.
  /// @param token_ Tell the thread to stop when requested.
  void run(std::stop_token token_);

  /// @brief Adds an odometry edge to a queue holding odometry edges.
  /// @param edge_ OdometryEdge with the new observation to be added.
  /// @return The size of the queue after adding the new edge.
  inline size_t insert(const OdometryEdge& edge_) {
    return _odometryEdges.push_back_safe(edge_);
  }

  /// @brief Adds an observation edge to a queue holding observation edges.
  /// @param edge_ ObservationEdge with the new observation to be added.
  /// @return The size of the queue after adding the new edge.
  inline size_t insert(const ObservationEdge<TransformationType>& edge_) {
    return _observationEdges.push_back_safe(edge_);
  }

  /// @brief Checks if the loop verification queue is empty.
  /// @return True if the loop verification queue is empty, false otherwise.
  [[nodiscard("Use returned bool to branch on loop-verification availability")]]
  inline bool is_loop_queue_empty() const {
    return _loopVerification.empty_safe();
  }

  /// @brief Gets the next keyframe Id from the loop verification queue.
  /// @return The next keyframe Id from the
  /// loop verification queue, or a fail if the queue is empty.
  [[nodiscard("Returns keyframe to handle empty loop-verification queue")]]
  inline output_t<keyframe_id_t> next_loop_verification() {
    return _loopVerification.pop_front_safe();
  }

  /// @brief Gets a 3d point observation (in a keyframe reference frame)
  /// for a given point ID and keyframe ID.
  /// If no keyframe Id is provided, the most recent observation is returned.
  /// @param pointId_ Map point ID to query.
  /// @param keyframeId_ Keyframe ID to query.
  /// @return The point observation.
  [[nodiscard("Use returned point output to handle observation lookup errors")]]
  output_t<point3d_t> get_point_observation(
      const point_id_t pointId_,
      const keyframe_id_t keyframeId_ = keyframe_id_t::UNSET);

  /// @brief Get a keypoint observation for a given point ID and keyframe ID.
  /// If no keyframe Id is provided, the most recent observation is returned.
  /// @param pointId_ Map point ID to query.
  /// @param keyframeId_ Keyframe ID to query.
  /// @return The keypoint observation.
  [[nodiscard("Returns keypoint output to handle observation lookup errors")]]
  output_t<point2d_t> get_keypoint_observation(
      const point_id_t pointId_,
      const keyframe_id_t keyframeId_ = keyframe_id_t::UNSET);

  /// @brief Get a descriptor for a given 3D point.
  /// @param pointId_ Map point ID to query.
  /// @param pose_ Pose to use for the descriptor prediction.
  /// @return The descriptor observation.
  [[nodiscard("Use returned descriptor output to handle retrieval failures")]]
  output_t<descriptor_t> get_descriptor_observation(
      const point_id_t pointId_,
      const pose_t pose_);

  /// @brief Get a descriptor for a given 3D point.
  /// @param pointId_ Map point ID to query.
  /// @param posePtr_ Pose to use for the descriptor prediction.
  /// @return The descriptor observation.
  [[nodiscard("Use returned descriptor output to handle retrieval failures")]]
  output_t<descriptor_t> get_descriptor_observation(
      const point_id_t pointId_,
      const pose_t::weak_ptr posePtr_);

  /// @brief Gets Ids of all points seen by the input keyframe Id.
  /// @param keyframeId_ Keyframe Id to query.
  /// @return Ids of all points seen by the keyframe.
  [[nodiscard("Use returned IDs to traverse keyframe-to-point connectivity")]]
  point_id_t::vector get_point_ids(const keyframe_id_t keyframeId_);

  /// @brief Gets Ids of all keyframes that observe the input point Id.
  /// @param pointId_ Point Id to query.
  /// @return Ids of all keyframes that observe the point.
  [[nodiscard("Use returned IDs to traverse point-to-keyframe connectivity")]]
  keyframe_id_t::vector get_keyframe_ids(const point_id_t pointId_);

  /// @brief Gets the Ids of all keyframes connected to the input keyframe Id.
  /// @param keyframeId_ Keyframe Id to query.
  /// @return Ids of all connected keyframes.
  [[nodiscard("Use returned IDs to inspect keyframe graph connectivity")]]
  keyframe_id_t::vector get_connection_ids(const keyframe_id_t keyframeId_);

  /// @brief Fits a new descriptor observation for a point.
  /// @param pointId_ Point Id to update the descriptor observation.
  /// @param descriptor_ New descriptor observation to fit into the model.
  /// @param pose_ Pose of the keyframe observing the point with the new
  /// descriptor. This is used to predict the expected descriptor and fit the
  /// new observation into the model.
  void fit_new_descriptor(
      const point_id_t pointId_,
      const descriptor_t& descriptor_,
      const pose_t pose_);

  /// @brief Fits a new descriptor observation for a point.
  /// @param pointId_ Point Id to update the descriptor observation.
  /// @param descriptor_ New descriptor observation to fit into the model.
  /// @param posePtr_ Pose of the keyframe observing the point with the new
  /// descriptor. This is used to predict the expected descriptor and fit the
  /// new observation into the model.
  void fit_new_descriptor(
      const point_id_t pointId_,
      const descriptor_t& descriptor_,
      const pose_t::weak_ptr posePtr_);

  /// @brief Given a point Id, gets the most up-to-date point Id.
  /// If the point was not merged before, the input id is returned unchanged.
  /// If the point was merged, the Id of the point it was merged into is
  /// returned
  /// @param pointId_ Input point Id to query. It is updated to the
  /// most up-to-date point Id.
  /// @return The most up-to-date point Id.
  [[nodiscard("Use returned point ID to avoid stale merged-point references")]]
  point_id_t get_up_to_date_point_id(const point_id_t pointId_);

  /// @brief Adds loop connection to the covisibility graph between two
  /// loop keyframes.
  /// @param evalKeyframeId_ Eval keyframe (the one we used to check for loops)
  /// @param loopKeyframeId_ Loop keyframe (the one already on the map)
  /// @param evalWorldToCamera_ Transform from world to camera for the eval
  ///   keyframe
  /// @param evalKeyframePointIds_ Eval keyframe point Ids (synced with
  /// loopKeyframePointIds_).
  /// @param loopKeyframePointIds_ Loop keyframe point Ids (synced with
  /// evalKeyframePointIds_).
  /// @param matches_ Matches between evalKeyframePointIds_ and
  /// loopKeyframePointIds_.
  /// @param inliers_ List with the inliers of the matches.
  void close_loop(
      const keyframe_id_t evalKeyframeId_,
      const keyframe_id_t loopKeyframeId_,
      const pose_t evalWorldToCamera_,
      const point_id_t::vector& evalKeyframePointIds_,
      const point_id_t::vector& loopKeyframePointIds_,
      match_t<index_t>::vector& matches_,
      std::vector<bool>& inliers_);

private:
  /// @brief CovisibilityGraph object constructor.
  /// @param map_ Slam system map.
  CovisibilityGraph(const map::Map::shared_ptr& map_);

  /// Starts the covisibility graph main threads.
  void __start_thread();

  /// @brief This function gets a new odometry edge to be processed.
  /// @return New edge to be processed, or fail_t::IS_EMPTY if none are queued.
  [[nodiscard("Use returned edge output to handle empty odometry queue")]]
  inline output_t<OdometryEdge> __pop_odometry_edge() {
    _odometryEdges.lock();
    if (_odometryEdges.empty()) {
      _odometryEdges.unlock();
      return fail_t(fail_t::IS_EMPTY, "No odometry edges available");
    }
    OdometryEdge edge = _odometryEdges.pop_front();
    _odometryEdges.unlock();
    return edge;
  }

  /// @brief This function gets a new observation edge to be processed.
  /// @return New edge to be processed, or fail_t::IS_EMPTY if none are queued.
  [[nodiscard("Use returned edge output to handle empty observation queue")]]
  inline output_t<ObservationEdge<TransformationType>>
  __pop_observation_edge() {
    _observationEdges.lock();
    if (_observationEdges.empty()) {
      _observationEdges.unlock();
      return fail_t(fail_t::IS_EMPTY, "No observation edges available");
    }
    ObservationEdge<TransformationType> edge = _observationEdges.pop_front();
    _observationEdges.unlock();
    return edge;
  }

  /// @brief Adds a keyframe Id to the loop verification queue.
  /// @param keyframeId_ Id of the keyframe to be added to the loop
  ///   verification.
  /// @return The size of the queue after adding the new keyframe Id.
  inline size_t __insert_to_loop_verification(const keyframe_id_t keyframeId_) {
    return _loopVerification.push_back_safe(keyframeId_);
  }

  /// @brief Gets a 3d point observation (in a keyframe reference frame)
  /// for a given point ID.
  /// @param pointId_ Map point ID to query.
  /// @return The point observation.
  [[nodiscard("Returns point output to handle internal observation failures")]]
  output_t<point3d_t> __get_point_observation(const point_id_t pointId_);

  /// @brief Gets a keypoint observation for a given point ID.
  /// @param pointId_ Map point ID to query.
  /// @return The keypoint observation.
  [[nodiscard("Returns keypoint handle internal observation failures")]]
  output_t<point2d_t> __get_keypoint_observation(const point_id_t pointId_);

  /// @brief Adds a new keyframe to the graph values.
  /// @param keyframeId_ ID of the keyframe to be added.
  /// @param keyframePtr_ Pointer to the keyframe (pose and data) as output_t.
  void __add_keyframe_to_values(
      const keyframe_id_t keyframeId_,
      const keyframe_t::weak_ptr keyframePtr_);

  /// @brief Adds a new point to the graph values.
  /// @param pointId_ ID of the point to be added.
  /// @param pointPtr_ Pointer to the point as output_t.
  void __add_point_to_values(
      const point_id_t pointId_,
      const point3d_t::weak_ptr pointPtr_);

  /// @brief Checks if a keyframe exists in the covisibility graph.
  /// @param symbol_ The GTSAM symbol representing the keyframe or point.
  /// @return True if the symbol exists in graph values, false otherwise.
  [[nodiscard("Use returned bool to avoid duplicate graph-value insertion")]]
  bool __exists(const gtsam::Symbol symbol_);

  /// @brief This function adds an odometry edge in the covisibility graph.
  /// @param edge_ The details of the odometry to be added.
  void __add_odometry_edge(const OdometryEdge& edge_);

  /// @brief Adds an observation between a keyframe and a point in graph.
  /// @param edge_ The details of the point observation to be added.
  void __add_observation_edge(const ObservationEdge<TransformationType>& edge_);

  /// @brief Adds a point observation to the graph.
  /// @param pointId_ Map point ID.
  /// @param keyframeId_ Keyframe ID.
  /// @param observationPtr_ Shared pointer to the PointObservation object.
  void __add_observation_to_graph(
      const point_id_t pointId_,
      const keyframe_id_t keyframeId_,
      const PointObservation<TransformationType>::shared_ptr observationPtr_);

  /// @brief Tries to add to the graph all uncalibrated projection factors that
  /// have been created without calibration, if a valid calibration is provided.
  /// @param calibration_ Intrinsics to be used in the projection factors.
  void __add_uncalibrated_factors(
      const gtsam::Cal3_S2::shared_ptr& calibration_);

  /// @brief Adds a prior factor to the graph for a keyframe.
  /// @param keyframeId_ ID of the keyframe to which the prior factor is added.
  /// @param pose_ Pose of the keyframe to be used in the prior factor.
  void __add_prior_factor(
      const keyframe_id_t keyframeId_,
      const TransformationType& pose_);

  /// @brief This function runs the optimization of the covisibility graph.
  /// @param token_ For stopping the thread when requested.
  void __optimize(std::stop_token token_);

  /// @brief Updates the points and keyframes in the map based on the
  /// optimized values after graph optimization.
  /// @param optimizedValues_ The optimized values after graph optimization.
  void __map_update(const gtsam::Values::shared_ptr optimizedValues_);

  /// @brief This function tries to update any point that failed
  /// to be updated to the map before. It uses a timeout mechanism to
  /// avoid keeping points that will never be updated.
  void __process_pending_points();

  /// Pointer to the map created in the slam system
  map::Map::shared_ptr _map;

  /// Queue with the odometry edges created in the Localization module
  /// that will be added to the covisibility graph
  OdometryEdge::queue _odometryEdges;
  /// Queue with the observations edges created in the Localization module
  /// that will be added to the covisibility graph
  ObservationEdge<TransformationType>::queue _observationEdges;
  /// Queue with the loop closure candidates to be verified. It contains the Ids
  /// of the keyframes that will be evaluated for loop closure
  keyframe_id_t::queue _loopVerification;

  /// This thread is used to insert new edges to the graph as they are
  /// created in the localization
  std::jthread _edgeInsertionThread;
  /// This thread is used to run the adjustments for the points and poses
  std::jthread _optimizationThread;

  /// Mutex for thread-safe access to the graph
  mutable mutex_t _mutex;
};

}  // namespace covisibility_graph
}  // namespace slam_mer

#endif  // COVISIBILITY_GRAPH_HPP
