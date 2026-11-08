/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file covisibility_graph.cpp
/// @brief Implements covisibility-graph construction and optimization flows.
/// @details Defines backend graph update and solver integration routines.
/// @author Pedro Miraldo
/// @author Valter Piedade

// Add our libraries
#include <covisibility_graph.hpp>
#include <factors_noise_model.hpp>
#include <settings.hpp>
#include <types/timeout_queue.hpp>

// Add the gtsam headers
#include <Eigen/src/Core/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>

// System includes
#include <keyframe.hpp>
#include <type_traits>
#include <types/pose.hpp>

namespace slam_mer {
namespace covisibility_graph {

// Local global variables
namespace {

/// The factor graph representing the covisibility graph
gtsam::NonlinearFactorGraph::shared_ptr __graph;
/// Initial values for the optimization
gtsam::Values::shared_ptr __initialEstimates;

using keyframeConnections = hash_t<keyframe_id_t, set_t<keyframe_id_t>>;
/// Mapping of keyframe IDs to their connected keyframe IDs
keyframeConnections __covisibilityConnections;

/// Flag that indicates if the map is anchored, ie, keyframe 0 has a prior
/// factor fixing its pose.
bool __isAnchored = false;

/// Set of symbols (keys) that exist in the graph
set_t<gtsam::Key> __graphKeys;

/// Mapping of merged points: point_id_t (key) merged into point_id_t (value)
hash_t<point_id_t, point_id_t> __mergedPoints;

/// List of points to be removed from the graph
point_id_t::set __pointsToBeRemoved;

/// Map keyframe scales
std::unordered_map<keyframe_id_t, double> __keyframeScales;

/// Dictionary of keyframes that have been optimized at least once.
set_t<keyframe_id_t> __optimizedKeyframeIds;

/// Flag to indicate if we should run a full graph optimization.
/// Usually set to true after a loop closure is detected, to optimize the
/// full graph and fix the drift.
bool __runFullOptimization = true;

/// List of projection factors without calibration. If a projection factor
/// cannot be created because the calibration is missing, it is stored here
/// and when the calibration becomes available, the factor is added to the
/// graph.
template <typename TransformationType>
vector_t<typename ProjectionFactor<TransformationType>::shared_ptr>
    __uncalibratedFactors;

/// Queue of points pending that could not be added to the graph
timeout_queue_t<pair_t<point_id_t, point3d_t>> __pendingPoints;

/// Mapping of point IDs to their descriptor models
hash_t<point_id_t, utils::DescriptorModel::shared_ptr> __pointDescriptors;

/// Mapping of point IDs to their descriptor models
template <typename TransformationType>
hash_t<
    keyframe_id_t,
    hash_t<
        point_id_t,
        typename PointObservation<TransformationType>::shared_ptr>>
    __keyframeObservations;

/// Mapping of point IDs to keyframe IDs and their observations
template <typename TransformationType>
hash_t<
    point_id_t,
    dictionary_t<
        keyframe_id_t,
        typename PointObservation<TransformationType>::shared_ptr>>
    __pointObservations;

// ---------------------------------------------------------------------------
// Working threads
// ---------------------------------------------------------------------------

/// Variable that says that we have enough data to run updates on the map.
std::atomic<bool> __hasEnoughData{false};

// ---------------------------------------------------------------------------
// Noise Models
// ---------------------------------------------------------------------------

/// Noise model for the point projection factor
gtsam::SharedNoiseModel __projectionNoise;
/// Noise model for the 3D to 3D distance factor
gtsam::SharedNoiseModel __distanceNoise;

}  // namespace

/// @brief Manages incremental ISAM2 updates for covisibility optimization.
/// @details Wraps update calls with exception handling, optional keyframe
/// clamping strategies, and factor-removal bookkeeping between iterations.
class GraphOptimizationManager {
public:
  /// @brief Constructor for GraphOptimizationManager. For now, we just
  /// need to keep the use similarity option.
  GraphOptimizationManager() {}

  /// @brief Run one guarded ISAM2 update cycle.
  /// @details Copies the current optimizer state, applies the requested update,
  /// and commits results only when optimization succeeds.
  /// @param isam_ ISAM2 optimizer instance to update.
  /// @param newFactors_ New factors queued for insertion.
  /// @param newValues_ New values queued for insertion.
  /// @param pointsToBeRemoved_ Set of point IDs scheduled for removal.
  /// @param map_ Map object used to retrieve fixed-window keyframes.
  /// @param fullGraphOptimization_ True to optimize the full graph, false to
  /// use the active fixed-window strategy.
  /// @return True when the update is successful, false otherwise.
  template <typename TransformationType>
  bool run_optimization(
      gtsam::ISAM2& isam_,
      const gtsam::NonlinearFactorGraph::shared_ptr newFactors_,
      const gtsam::Values::shared_ptr newValues_,
      set_t<point_id_t>& pointsToBeRemoved_,
      const map::Map::shared_ptr map_,
      const bool fullGraphOptimization_) {
    using gtsam::FactorIndices;
    using gtsam::IndeterminantLinearSystemException;
    using gtsam::ISAM2;
    using gtsam::NonlinearFactorGraph;
    using gtsam::Values;
    using gtsam::ValuesKeyDoesNotExist;
    using std::exception;

    // This function:
    // 1 - Keeps track of the current ISAM optimization (variable, graph, and
    //     values), in case the update fails;
    // 2 - Creates the keys to be removed and their priority order based on the
    //     points to be removed;
    // 3 - Runs the ISAM2 update in a try-catch block to catch any exceptions
    //     that may happen during the optimization;
    // 4 - If the update is successful, updates the original ISAM2 instance and
    //     resets the graph and values.

    ISAM2 isamCopy = isam_;
    const NonlinearFactorGraph copyGraph = *newFactors_;
    const Values copyValues = *newValues_;
    bool successfulUpdate = false;

    const keyframe_id_t::set fixedKeyframes =
        map_->get_localization_buffer_keyframes();

#ifdef LOG_ADJUSTMENT
    std::string keyframeIdsString;
    for (const keyframe_id_t& id : fixedKeyframes)
      keyframeIdsString += std::string(id) + " ";
    log::message("Most recent keyframes: " + keyframeIdsString, log::SLAM);
#endif

    try {
      _clamps = GraphOptimizationManager::__optimize<TransformationType>(
          isamCopy,
          fixedKeyframes,
          _clamps,
          copyGraph,
          copyValues,
          FactorIndices());
      successfulUpdate = true;
    } catch (const IndeterminantLinearSystemException& e_) {
      __indeterminant_system_exception(e_);
    } catch (const ValuesKeyDoesNotExist& e_) {
      __values_key_not_exist_exception(e_);
    } catch (const exception& e_) {
      __other_exception(e_);
    }

    if (successfulUpdate) {
      isam_ = std::move(isamCopy);
      newFactors_->resize(0);
      newValues_->clear();
    }
    return successfulUpdate;
  }

private:
  /// @brief Get the pose manifold dimension used in optimization.
  /// @return 6 for `gtsam::Pose3`, 7 for `gtsam::Similarity3`.
  /// @note If TransformationType is Pose3 dim is 6, if Similarity3 dim is 7.
  template <typename TransformationType>
  static constexpr int __pose_dimension() {
    if constexpr (std::is_same_v<TransformationType, gtsam::Pose3>) return 6;
    if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>)
      return 7;
    __throw__("TransformationType must be gtsam::Pose3 or gtsam::Similarity3");
  }

  /// @brief Handler for IndeterminantLinearSystem exceptions
  /// @note  If we have an indeterminant linear system, we log a warning with
  /// keys that are involved in the exception, which can help debugging
  /// the issue.
  /// @param e_ The exception object
  static void __indeterminant_system_exception(
      const gtsam::IndeterminantLinearSystemException& e_) {
    log::warning(
        "ISAM2 optimization failed due to an indeterminant linear system: " +
            std::string(e_.what()),
        log::SLAM);
  }

  /// @brief Handler for ValuesKeyDoesNotExist exceptions
  /// @note  This type of exception should never happen, throw an error.
  /// @param e_ The exception object
  static void __values_key_not_exist_exception(
      const gtsam::ValuesKeyDoesNotExist& e_) {
    __throw__("ISAM2 optimization failed: " + std::string(e_.what()));
  }

  /// @brief Handler for generic optimization exceptions
  /// @note  This type of exception can be several different things.
  /// Log the error in a warning message to help with debugging.
  /// @param e_ The exception object
  static void __other_exception(const std::exception& e_) {
    log::warning(
        "ISAM2 optimization failed: " + std::string(e_.what()),
        log::SLAM);
  }

  /// @brief Function that creates keysToRemove and removeOrder, which
  /// contain the keys to be removed from the graph and its priority order
  /// @param pointsToBeRemoved_ Set of point IDs to be removed from the graph
  /// @param keysToRemove_ Output list of gtsam::Key corresponding to the
  /// points to be removed
  /// @param removeOrder_ Output map of gtsam::Key to int indicating the
  /// priority order of removal (lower int = higher priority)
  static void __set_keys_to_remove(
      point_id_t::set& pointsToBeRemoved_,
      gtsam::FastList<gtsam::Key>& keysToRemove_,
      gtsam::FastMap<gtsam::Key, int>& removeOrder_) {
    using gtsam::Symbol;

    pointsToBeRemoved_.lock_read();
    for (const point_id_t& removeId : pointsToBeRemoved_) {
      const Symbol pointSymbol('p', removeId.key());
      removeOrder_[pointSymbol] = 0;
      keysToRemove_.push_back(pointSymbol);
    }
    pointsToBeRemoved_.clear();
    pointsToBeRemoved_.unlock_read();
  }

  /// @brief Optimizes the factor graph with the given parameters. If
  /// fixedWindow is empty, optimizes the whole graph. Otherwise,
  /// optimizes with the fixed window.
  /// @param isam_ ISAM2 optimizer object
  /// @param fixedWindow_ List of keyframe IDs to fix during optimization
  /// @param previousClamps_ List of factor indices corresponding to the clamps
  /// added in the previous optimization (to be removed before the next
  /// optimization)
  /// @param newFactors_ New factors to optimize
  /// @param newValues_ New values for the optimization
  /// @param factorsToRemove_ List of factor indices to remove from the graph
  /// before optimization (in addition to previousClamps_)
  /// @return List of factor indices corresponding to the clamps added in
  /// this optimization
  template <typename TransformationType>
  static gtsam::FactorIndices __optimize(
      gtsam::ISAM2& isam_,
      const keyframe_id_t::set& fixedWindow_ = keyframe_id_t::set(),
      const gtsam::FactorIndices& previousClamps_ = gtsam::FactorIndices(),
      const gtsam::NonlinearFactorGraph& newFactors_ =
          gtsam::NonlinearFactorGraph(),
      const gtsam::Values& newValues_ = gtsam::Values(),
      const gtsam::FactorIndices& factorsToRemove_ = gtsam::FactorIndices()) {
    using gtsam::FactorIndex;
    using gtsam::FactorIndices;

    // This function:
    // 1 - Setup factors that will be removed: old clamps + extra removals;
    // 2 - If fixedWindow is empty, optimize the whole graph. Otherwise,
    //     optimize with the fixed window by adding strong priors to the
    //     keyframes in the fixed window;
    // 3 - Return the indices of the clamps added in this optimization, to be
    //     removed in the next optimization.

    FactorIndices factorsToRemove = factorsToRemove_;
    for (const FactorIndex idx : previousClamps_)
      factorsToRemove.push_back(idx);

    if (fixedWindow_.empty())
      return __optimize_all(isam_, newFactors_, newValues_, factorsToRemove);

    return __optimize_rigid_window<TransformationType>(
        isam_,
        newFactors_,
        newValues_,
        fixedWindow_,
        factorsToRemove);
  }

  /// @brief Optimizes the graph with a fixed window by clamping the selected
  /// keyframes with constrained priors. Keyframes within the fixed window
  /// will remain fixed during optimization.
  /// @param isam_ ISAM2 optimizer object
  /// @param newFactors_ New factors to optimize
  /// @param newValues_ New values for the optimization
  /// @param fixedWindow_ List of keyframe IDs to fix during optimization
  /// @param factorsToRemove_ List of factor indices to remove from the graph
  /// before optimization
  /// @return List of factor indices corresponding to the clamps added
  /// to fix the window
  template <typename TransformationType>
  static gtsam::FactorIndices __optimize_fixed_window(
      gtsam::ISAM2& isam_,
      const gtsam::NonlinearFactorGraph& newFactors_,
      const gtsam::Values& newValues_,
      const keyframe_id_t::set& fixedWindow_,
      const gtsam::FactorIndices& factorsToRemove_) {
    using gtsam::FactorIndices;
    using gtsam::ISAM2Result;
    using gtsam::Key;
    using gtsam::NonlinearFactorGraph;
    using PriorFactor = PosePriorFactor<TransformationType>;
    using gtsam::SharedNoiseModel;
    using gtsam::Symbol;
    using gtsam::Values;
    using gtsam::noiseModel::Constrained;

    // This function:
    // 1 - Clamps the keyframes in the fixed window by adding prior factors
    //     with a very strong noise (constrained) to the newFactors;
    // 2 - Optimizes the graph with the new clamps and returns the indices of
    //     the clamps.
    // 3 - Retrives the indeces of the clamps added in this optimization and
    //     returns them.

    NonlinearFactorGraph newFactors = newFactors_;
    const Values estimate = isam_.calculateEstimate();

    const int dimension = __pose_dimension<TransformationType>();
    const SharedNoiseModel fixedPoseNoise = Constrained::All(dimension);

    size_t numClampsAdded = 0;
    for (const keyframe_id_t keyframeId : fixedWindow_) {
      const Symbol symbol('k', keyframeId.key());
      const Key key(symbol);
      TransformationType pose;

      if (estimate.exists(key))
        pose = estimate.at<TransformationType>(key);
      else if (newValues_.exists(key))
        pose = newValues_.at<TransformationType>(key);
      else
        continue;

      const auto strongFactor =
          std::make_shared<PriorFactor>(key, pose, fixedPoseNoise);

      newFactors.add(strongFactor);
      ++numClampsAdded;
    }

    const ISAM2Result result =
        isam_.update(newFactors, newValues_, factorsToRemove_);

    FactorIndices clampFactorIdxs;
    const size_t numExtra = newFactors.size() - numClampsAdded;
    for (size_t i = 0; i < numClampsAdded; ++i)
      clampFactorIdxs.push_back(result.newFactorsIndices[numExtra + i]);

    return clampFactorIdxs;
  }

  /// @brief Optimizes the graph with a rigid window by clamping the selected
  /// keyframes with rigid priors. Keyframes within the fixed window will
  /// have stronger constraints to (relative to each other) during optimization.
  /// @param isam_ ISAM2 optimizer object
  /// @param newFactors_ New factors to optimize
  /// @param newValues_ New values for the optimization
  /// @param fixedWindow_ List of keyframe IDs to fix during optimization
  /// @param factorsToRemove_ List of factor indices to remove from the graph
  /// before optimization
  /// @return List of factor indices corresponding to the clamps added
  /// to fix the window
  template <typename TransformationType>
  static gtsam::FactorIndices __optimize_rigid_window(
      gtsam::ISAM2& isam_,
      const gtsam::NonlinearFactorGraph& newFactors_,
      const gtsam::Values& newValues_,
      const keyframe_id_t::set& fixedWindow_,
      const gtsam::FactorIndices& factorsToRemove_) {
    using gtsam::FactorIndices;
    using gtsam::ISAM2Result;
    using gtsam::Key;
    using gtsam::NonlinearFactorGraph;
    using gtsam::SharedNoiseModel;
    using gtsam::Symbol;
    using gtsam::Values;
    using slam_mer::settings::sharedSettings;

    NonlinearFactorGraph newFactors = newFactors_;
    Values estimate = isam_.calculateEstimate();

    // Lambda function to get the poses from isam estimates or values
    auto ___get_pose = [&](const Key key__,
                           TransformationType& pose__) -> bool {
      if (estimate.exists(key__)) {
        pose__ = estimate.at<TransformationType>(key__);
        return true;
      }
      if (newValues_.exists(key__)) {
        pose__ = newValues_.at<TransformationType>(key__);
        return true;
      }
      return false;
    };

    // This function -- O(N^2):
    // 1 - Gets the poses of the keyframes in the fixed window from the ISAM2
    //     estimates or new values;
    // 2 - Computes the relative poses between the connected keyframes in
    //     fixed window.
    // 3 - Creates new factors, with strogger relative pose constrains, and
    //     add them to the graph.
    // 4 - Optimize, and collect the idexes of the optimized factors.
    // NOTE: If failed to query any of the poses, it will simple ignore and
    // not create the stronger edge.

    // Smaller weight => smaller sigmas => stronger constraint.
    // Tune in [0.01, 0.3] typically.
    const auto clampWeight =
        sharedSettings->covisibility_rigid_window_clamp_weight();
    const auto& clampNoiseModel =
        get_odometry_noise_model<TransformationType>(clampWeight);
    const SharedNoiseModel clampNoise = SharedNoiseModel(clampNoiseModel);
    size_t numClampsAdded = 0;

    for (const auto& keyframeIdA : fixedWindow_) {
      const Symbol symbolA('k', keyframeIdA.key());
      const Key keyA(symbolA);
      TransformationType poseA;
      if (!___get_pose(keyA, poseA)) continue;

      __covisibilityConnections.lock_read();
      set_t<keyframe_id_t> connected = __covisibilityConnections[keyframeIdA];
      __covisibilityConnections.unlock_read();

      for (const keyframe_id_t& keyframeIdB : connected) {
        const Symbol symbolB('k', keyframeIdB.key());
        const Key keyB(symbolB);
        TransformationType poseB;
        if (!___get_pose(keyB, poseB)) continue;

        const TransformationType relativePose = poseA.between(poseB);
        const auto newFactor = OdometryFactor<TransformationType>::make_shared(
            keyframeIdA,
            keyframeIdB,
            relativePose,
            clampNoise);
        newFactors.add(newFactor);
        ++numClampsAdded;
      }
    }

    const ISAM2Result result =
        isam_.update(newFactors, newValues_, factorsToRemove_);

    FactorIndices clampFactorIdxs;
    const size_t numExtra = newFactors.size() - numClampsAdded;
    for (size_t i = 0; i < numClampsAdded; ++i) {
      clampFactorIdxs.push_back(result.newFactorsIndices[numExtra + i]);
    }
    return clampFactorIdxs;
  }

  /// @brief Optimizes the whole graph (no fixed poses).
  /// @param isam_ ISAM2 optimizer object
  /// @param newFactors_ New factors to optimize
  /// @param newValues_ New values for the optimization
  /// @param factorsToRemove_ List of factor indices to remove from the graph
  /// before optimization
  /// @return Empty list (no clamps added)
  static gtsam::FactorIndices __optimize_all(
      gtsam::ISAM2& isam_,
      const gtsam::NonlinearFactorGraph& newFactors_,
      const gtsam::Values& newValues_,
      const gtsam::FactorIndices& factorsToRemove_) {
    using gtsam::FactorIndices;
    isam_.update(newFactors_, newValues_, factorsToRemove_);
    return FactorIndices{};  // no active clamps now
  }

  /// Index of the last optimization clamps, to be removed in the next
  /// optimization
  gtsam::FactorIndices _clamps;
};

// ---------------------------------------------------------------------------
// CovisibilityGraph object constructor.
// ---------------------------------------------------------------------------
template <typename TransformationType>
CovisibilityGraph<TransformationType>::CovisibilityGraph(
    const map::Map::shared_ptr& map_)
    : _map(map_) {
  using gtsam::NonlinearFactorGraph;
  using gtsam::Values;
  using gtsam::noiseModel::Diagonal;
  using slam_mer::settings::sharedSettings;

  // This Constructor:
  // 1 - Instantiate the factor graph, initial values, and map
  // 2 - Set the noise models for the projection and distance factors

  __graph = std::make_shared<NonlinearFactorGraph>();
  __initialEstimates = std::make_shared<Values>();

  Diagonal::shared_ptr projectionNoise = get_projection_noise_model();
  if (sharedSettings->use_robust_projection_factor())
    __projectionNoise = get_robust_projection_noise_model(projectionNoise);
  else
    __projectionNoise = projectionNoise;

  Diagonal::shared_ptr pointDistanceNoise = get_distance_noise_model();
  if (sharedSettings->use_robust_distance_factor())
    __distanceNoise = get_robust_distance_noise_model(pointDistanceNoise);
  else
    __distanceNoise = pointDistanceNoise;
}

// ---------------------------------------------------------------------------
// CovisibilityGraph object destructor
// ---------------------------------------------------------------------------
template <typename TransformationType>
CovisibilityGraph<TransformationType>::~CovisibilityGraph() {
  using std::println;

  timer_t timer;

  // 1. Wait until all edges are added
  while (_observationEdges.empty_safe() == false ||
         _odometryEdges.empty_safe() == false) {
    timer.sleep(duration_t::from_ms(10));
  }

  // 2. Stop the thread that adds new edges to the graph
  if (_edgeInsertionThread.joinable()) {
    _edgeInsertionThread.request_stop();
    _edgeInsertionThread.join();
  }

  // 3. Wait for 1 sec so that the optimization runs a few more times
  timer.sleep(duration_t::from_s(1));

  // 4. Stop the optimization thread
  if (_optimizationThread.joinable()) {
    _optimizationThread.request_stop();
    _optimizationThread.join();
  }

  // 5. Destructor to clean up the graph and values
  __graph.reset();
  __initialEstimates.reset();

  println("[CovisibilityGraph] Stopped successfully.");
}

// ---------------------------------------------------------------------------
// Start the threads
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__start_thread() {
  using modes::SystemMode;
  using slam_mer::settings::sharedSettings;

  // First, we launch the thread that will continuously add new odometry and
  // observation edges to the graph.
  if (_edgeInsertionThread.joinable()) {
    _edgeInsertionThread.request_stop();
    _edgeInsertionThread.join();
  }
  _edgeInsertionThread =
      std::jthread([this](std::stop_token token__) { this->run(token__); });

  // Secondly, we launch the thread that will run the optimization and
  // map update
  const SystemMode& mode = sharedSettings->operation_mode();
  if (mode == SystemMode::SLAM || mode == SystemMode::SLAM_WITHOUT_LOOP) {
    if (_optimizationThread.joinable()) {
      _optimizationThread.request_stop();
      _optimizationThread.join();
    }
    _optimizationThread = std::jthread(
        [this](std::stop_token token__) { this->__optimize(token__); });
  }
}

// ---------------------------------------------------------------------------
// Launch run thread
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::run(std::stop_token token_) {
  using modes::SystemMode;
  using slam_mer::settings::sharedSettings;
  using std::jthread;

  timer_t timer;
  const int minimumFramesPerSecond = sharedSettings->covisibility_module_fps();
  while (token_.stop_requested() == false) {
    timer.tic();

#pragma omp parallel sections num_threads(2)
    {
#pragma omp section
      {
        // Cycle through the observation edges, adding them to the graph
        while (true) {
          // Pop the observation edge from the queue
          const output_t<ObservationEdge<TransformationType>> edge =
              __pop_observation_edge();
          if (edge == fail_t::IS_EMPTY) break;
          if (edge != fail_t::NO_FAIL)
            __throw__("Unexpected error" + std::string(edge.get_error()));
          __add_observation_edge(edge);
        }
      }

#pragma omp section
      {
        // Cycle through the odometry edges, adding them to the graph
        while (true) {
          // Pop the odometry edge from the queue
          const output_t<OdometryEdge> edge = __pop_odometry_edge();
          if (edge == fail_t::IS_EMPTY) break;
          if (edge != fail_t::NO_FAIL)
            __throw__("Unexpected error" + std::string(edge.get_error()));
          __add_odometry_edge(edge);
        }
      }
    }

    // Wait to maintain the loop at the desired frequency
    timer.pace(duration_t::from_fps(minimumFramesPerSecond));
  }
}

// ---------------------------------------------------------------------------
// Add point to Values
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__add_point_to_values(
    const point_id_t pointId_,
    const point3d_t::weak_ptr pointPtr_) {
  using gtsam::Key;
  using gtsam::Point3;
  using gtsam::Symbol;

  // Ensure the point has a value on the graph
  const Symbol symbolPoint('p', pointId_.key());
  const point3d_t point(pointPtr_);
  if (__exists(symbolPoint) == false) {
    __sanity_set(point, pointId_);
    __graphKeys.insert_(Key(symbolPoint));
    const Eigen::Vector3d& eigenPoint = point;
    const Point3 gtsamPoint(eigenPoint);
    __initialEstimates->insert(symbolPoint, gtsamPoint);
  }
  return;
}

// ---------------------------------------------------------------------------
// Add keyframe to Values
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__add_keyframe_to_values(
    const keyframe_id_t keyframeId_,
    const keyframe_t::weak_ptr keyframePtr_) {
  using gtsam::Key;
  using gtsam::Symbol;

  // Ensure the keyframe has a value on the graph
  Symbol symbolKeyframe('k', keyframeId_.key());

  if (__exists(symbolKeyframe) == false) {
    // Add keyframe to the values
    __graphKeys.insert_(Key(symbolKeyframe));
    const pose_t::weak_ptr posePtr = keyframe_t::get_pose(keyframePtr_);
    const Eigen::Matrix4d poseMatrix = pose_t::get_matrix(posePtr);
    const TransformationType pose(poseMatrix);
    __initialEstimates->insert(symbolKeyframe, pose);
  }
  return;
}

// ---------------------------------------------------------------------------
// Add odometry edge to graph and update keyframe connectivity
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__add_odometry_edge(
    const OdometryEdge& edge_) {
  using Eigen::Matrix4d;
  using gtsam::SharedNoiseModel;
  using gtsam::noiseModel::Diagonal;
  using slam_mer::settings::sharedSettings;

  // ------------------------------------------------------------------------
  // Lambda functions
  auto ___get_poseType =
      [](const keyframe_t::weak_ptr& keyframe__) -> TransformationType {
    const output_t<pose_t::weak_ptr> posePtr = keyframe_t::get_pose(keyframe__);
    if (posePtr == fail_t::LOST_PTR || posePtr != fail_t::NO_FAIL)
      __throw__("Unexpected, cannot lock Keyframe");
    const output_t<Matrix4d> matrixPose = pose_t::get_matrix(posePtr);
    if (matrixPose == fail_t::LOST_PTR || matrixPose != fail_t::NO_FAIL)
      __throw__("Unexpected, cannot get matrix");
    return TransformationType(matrixPose.get());
  };
  auto ___get_weight = [](const keyframe_id_t& keyframeId1__,
                          const keyframe_id_t& keyframeId2__,
                          const bool fromLoopClosure__) -> double {
    double weight =
        keyframeId1__.key() > keyframeId2__.key()
            ? static_cast<double>(keyframeId1__.key() - keyframeId2__.key())
            : static_cast<double>(keyframeId2__.key() - keyframeId1__.key());
    weight = 1.0;  // (weight - 1) * 0.1 + 1.0;
    if (fromLoopClosure__) weight = sharedSettings->loop_edge_weight();
    return weight;
  };
  // ------------------------------------------------------------------------

  // This function:
  // 1 - Checks if the keyframes of the edge exist in the map.
  // 2 - Locks the graph for writing and ensures both keyframes have
  //     their poses on the graph values. If not, they are added.
  // 3 - If its the very first keyframe (keyframeId1 == 0), adds a prior
  //     factor to anchor the graph.
  // 4 - Sets the edge weight based on the distance between the keyframes and if
  // it is a loop closure edge.
  // 5 - Creates the odometry factor and adds it to the graph.
  // NOTE: We are not expecting keyframes to be deleted. So that throws error

  const keyframe_id_t keyframeId1 = edge_._keyframeA;
  const keyframe_id_t keyframeId2 = edge_._keyframeB;
  const bool fromLoopClosure = edge_._loopClosure;
  const pose_t odometry = edge_._odometry;

  if (keyframeId1 == keyframeId2)
    __throw__("Unexpected: Trying to add odometry between the same keyframe");

  {
    write_lock_t lock(_mutex);

    const output_t<keyframe_t::weak_ptr> keyframe1 = _map->at(keyframeId1);
    const output_t<keyframe_t::weak_ptr> keyframe2 = _map->at(keyframeId2);
    __sanity_retrieve(keyframe1, keyframeId1);
    __sanity_retrieve(keyframe2, keyframeId2);
    __add_keyframe_to_values(keyframeId1, keyframe1);
    __add_keyframe_to_values(keyframeId2, keyframe2);

    if (keyframeId1 == 0 && !__isAnchored) {
      const TransformationType pose = ___get_poseType(keyframe1);
      __add_prior_factor(keyframeId1, pose);
      __isAnchored = true;
    }

    double weight = ___get_weight(keyframeId1, keyframeId2, fromLoopClosure);
    const Diagonal::shared_ptr noiseModel =
        get_odometry_noise_model<TransformationType>(weight);
    const SharedNoiseModel noise =
        sharedSettings->use_robust_odometry_factor()
            ? SharedNoiseModel(get_robust_odometry_noise_model(noiseModel))
            : SharedNoiseModel(noiseModel);

    const Matrix4d& odometryMatrix = odometry.get_matrix();
    const typename OdometryFactor<TransformationType>::shared_ptr
        betweenFactor = OdometryFactor<TransformationType>::make_shared(
            keyframeId1,
            keyframeId2,
            TransformationType(odometryMatrix),
            noise);

    __graph->add(betweenFactor);
  }

  // Add to the interface for visualization
  _map->add_new_covisibility_connections_map(
      keyframe_id_t::pair(keyframeId1, keyframeId2));

  // Update the covisibility connections
  __covisibilityConnections.lock();
  __covisibilityConnections[keyframeId1].insert_(keyframeId2);
  __covisibilityConnections[keyframeId2].insert_(keyframeId1);
  __covisibilityConnections.unlock();
}

// ---------------------------------------------------------------------------
// Add point observation edge and create associated graph factors
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__add_observation_edge(
    const ObservationEdge<TransformationType>& edge_) {
  using gtsam::Cal3_S2;
  using gtsam::Point2;
  using gtsam::Point3;
  using slam_mer::settings::sharedSettings;
  using std::println;
  using utils::DescriptorModel;

  // ------------------------------------------------------------------------
  // Lambda functions
  auto ___compute_intrinsics =
      [](const output_t<intrinsics_t>& intrinsics_out__)
      -> Cal3_S2::shared_ptr {
    if (intrinsics_out__ == fail_t::LOST_PTR)
      __throw__("Unexpected: Lost pointer for intrinsics");
    if (intrinsics_out__ == fail_t::IS_EMPTY) return nullptr;
    const intrinsics_t intrinsics = intrinsics_out__;
    const auto [fx, fy, skew, cx, cy] = intrinsics.get_parameters();
    return std::make_shared<Cal3_S2>(fx, fy, skew, cx, cy);
  };
  auto ___compute_point2d = [](const point2d_t& point__) -> Point2 {
    const array2D_t xy = point__;
    return Point2(xy[0], xy[1]);
  };
  auto ___compute_point3d = [](const point3d_t& point__) -> Point3 {
    const array3D_t xyz = point__.to_3d_array();
    return Point3(xyz[0], xyz[1], xyz[2]);
  };
  auto ___add_point_observation =
      [](const point_id_t& pointId__,
         const keyframe_id_t& keyframeId__,
         const PointObservation<TransformationType>::shared_ptr
             observationPtr__) -> void {
    if (!__pointObservations<TransformationType>.contains(pointId__)) {
      using dic = dictionary_t<
          keyframe_id_t,
          typename PointObservation<TransformationType>::shared_ptr>;
      const dic newEntry({keyframeId__, observationPtr__});
      __pointObservations<TransformationType>.insert({pointId__, newEntry});
    }
    __pointObservations<TransformationType>[pointId__].insert_(
        {keyframeId__, observationPtr__});
  };
  auto ___add_keyframe_observation =
      [](const keyframe_id_t& keyframeId__,
         const point_id_t& pointId__,
         const typename PointObservation<TransformationType>::shared_ptr
             observationPtr__) -> void {
    if (!__keyframeObservations<TransformationType>.contains(keyframeId__)) {
      using dic = hash_t<
          point_id_t,
          typename PointObservation<TransformationType>::shared_ptr>;
      const dic newEntry({pointId__, observationPtr__});
      __keyframeObservations<TransformationType>.insert(
          {keyframeId__, newEntry});
    }
    __keyframeObservations<TransformationType>[keyframeId__].insert_(
        {pointId__, observationPtr__});
  };
  auto ___get_descriptor_info =
      [](const PointObservation<TransformationType>& observation__)
      -> std::tuple<descriptor_t, pose_t, point3d_t> {
    const output_t<descriptor_t> ouptut1 = observation__.get_descriptor();
    const output_t<pose_t> ouptut2 = observation__.get_camera_pose();
    const output_t<point3d_t> ouptut3 = observation__.get_point();
    if (ouptut1 != fail_t::NO_FAIL)
      __throw__("Unexpected: Fail to retrive descriptor from observation");
    if (ouptut2 != fail_t::NO_FAIL)
      __throw__("Unexpected: Fail to retrive camera pose from observation");
    if (ouptut3 != fail_t::NO_FAIL)
      __throw__("Unexpected: Fail to retrive 3D point from observation");
    return std::make_tuple(ouptut1, ouptut2, ouptut3);
  };
  auto ___setup_descriptor_model =
      [___get_descriptor_info, this](
          const point_id_t& pointId__,
          const PointObservation<TransformationType>& observation__) -> void {
    if (__pointDescriptors.contains(pointId__) == false) {
      auto [descriptor, cameraPose, point] =
          ___get_descriptor_info(observation__);
      DescriptorModel::shared_ptr descriptorModel =
          DescriptorModel::create(sharedSettings->descriptor_model_type());
      __pointDescriptors.insert({pointId__, descriptorModel});
      __pointDescriptors[pointId__]->fit(descriptor, cameraPose, point);
    }
  };
  // ------------------------------------------------------------------------

  // ------------------------------------------------------------------------
  // 2D projection and 3D distance factors
  // 1 - Get keyframe id and observation edge
  // 2 - Get the point id, and make sure we get the latest id if the point
  //     was/has been marked to be merged
  // 3 - Get camera calibration
  // 4 - Get 2D point observation
  // 5 - Get 3D point observation
  // 6 - Create projection and distance factors
  // 7 - If no calibration, and projection factor creted,
  //     add to uncalibrated factors list, to be added later.
  // 8 - Save the observation in the point and keyframe observations
  // dictionaries
  // 9 - Add the factors to the graph. We only add the point to the graph
  // 9.1 - at least 2 observations for the point, because before that the
  //       point is not fully observable and adding it creates a lot of
  //       noise in the graph
  // 10 - Finally, we add the calibration factors in the queue (notice that
  //      initially the camera may be running as uncalibrated, so we want
  //      to add the factors as soon as the calibration is available)

  // Make sure we have the most up-to-date point ID in case it was merged
  const keyframe_id_t keyframeId = edge_._keyframeId;
  const point_id_t pointId = this->get_up_to_date_point_id(edge_._pointId);
  const PointObservation observation = edge_._observation;

  const output_t<keyframe_t::weak_ptr> keyframe_out = _map->at(keyframeId);
  __sanity_retrieve(keyframe_out);

  const output_t<intrinsics_t> intrinsics_out =
      keyframe_t::get_intrinsics(keyframe_out);
  const Cal3_S2::shared_ptr calibration = ___compute_intrinsics(intrinsics_out);

  const output_t<point2d_t> point2d_out = observation.get_keypoint();
  const output_t<point3d_t> point3d_out = observation.get_point();
  __sanity_retrieve(point2d_out, pointId);
  __sanity_retrieve(point3d_out, pointId);

  const typename ProjectionFactor<TransformationType>::shared_ptr
      projectionFactor = ProjectionFactor<TransformationType>::make_shared(
          keyframeId,
          pointId,
          calibration,
          ___compute_point2d(point2d_out),
          __projectionNoise);

  const typename PointDistanceFactor<TransformationType>::shared_ptr
      distanceFactor = PointDistanceFactor<TransformationType>::make_shared(
          keyframeId,
          pointId,
          ___compute_point3d(point3d_out),
          __distanceNoise);

  if (!calibration && projectionFactor)
    __uncalibratedFactors<TransformationType>.push_back(projectionFactor);

  const typename PointObservation<TransformationType>::shared_ptr
      observationPtr = PointObservation<TransformationType>::make_shared(
          observation,
          projectionFactor,
          distanceFactor);

  // 8
  __pointObservations<TransformationType>.lock();
  __keyframeObservations<TransformationType>.lock();
  __pointDescriptors.lock();
  ___add_point_observation(pointId, keyframeId, observationPtr);
  ___add_keyframe_observation(keyframeId, pointId, observationPtr);
  ___setup_descriptor_model(pointId, observation);
  __pointDescriptors.unlock();
  __keyframeObservations<TransformationType>.unlock();
  __pointObservations<TransformationType>.unlock();

  // 9
  __pointObservations<TransformationType>[pointId].lock_read();
  const size_t& numberFactors =
      __pointObservations<TransformationType>[pointId].size();
  if (numberFactors == 2) {
    write_lock_t lock(_mutex);
    for (const auto& [keyframeId, observationsPtr] :
         __pointObservations<TransformationType>[pointId])
      __add_observation_to_graph(pointId, keyframeId, observationsPtr);
  } else if (numberFactors > 2) {
    write_lock_t lock(_mutex);
    __add_observation_to_graph(pointId, keyframeId, observationPtr);
  }
  __pointObservations<TransformationType>[pointId].unlock_read();

  // 10
  __add_uncalibrated_factors(calibration);
}

// ---------------------------------------------------------------------------
// Adds an observation to the graph.
// ---------------------------------------------------------------------------

template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__add_observation_to_graph(
    const point_id_t pointId_,
    const keyframe_id_t keyframeId_,
    const PointObservation<TransformationType>::shared_ptr observationPtr_) {

  // this function:
  // 1 - gets the variables from the map.
  // 2 - ensures the variables were loaded properly.
  // 3 - adds the variables to the graph values if they do not exist.
  // 4 - adds the factors to the graph (send message is fail to retrieve)

  output_t<keyframe_t::weak_ptr> keyframe_out = _map->at(keyframeId_);
  output_t<point3d_t::weak_ptr> point_out = _map->at(pointId_);
  __sanity_retrieve(keyframe_out, keyframeId_);
  __sanity_retrieve(point_out, pointId_);

  __add_keyframe_to_values(keyframeId_, keyframe_out);
  __add_point_to_values(pointId_, point_out);

  const auto observation_out = observationPtr_->get_distance_factor();
  __sanity_retrieve(observation_out);
  __graph->add(observation_out.get());

  const auto projection_out = observationPtr_->get_projection_factor();
  __sanity_retrieve(projection_out);
  if (projection_out->has_calibration()) __graph->add(projection_out.get());
}

// ---------------------------------------------------------------------------
// Adds uncalibrated factors
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__add_uncalibrated_factors(
    const gtsam::Cal3_S2::shared_ptr& calibration_) {
  using gtsam::Key;
  using gtsam::Symbol;

  // This function
  // 1 - Checks if there are uncalibrated factors and if calibration is
  //     available.
  // 2 - If so, it cycles through the uncalibrated factors, sets the
  //     calibration, and adds them to the graph if the point has at least
  //     2 observations.
  // 3 - If the point does not have at least 2 observations, it keeps the factor
  //     in the uncalibrated factors list to be added later.

  write_lock_t lock(_mutex);
  if (!calibration_ || __uncalibratedFactors<TransformationType>.empty())
    return;

  vector_t<typename ProjectionFactor<TransformationType>::shared_ptr>
      remainingFactors;
  for (const auto& factor : __uncalibratedFactors<TransformationType>) {
    const Key pointKey = factor->get_key();
    const Symbol pointSymbol(pointKey);
    const point_id_t pointId(pointSymbol.index());
    if (__pointObservations<TransformationType>[pointId].size_() < 2) {
      remainingFactors.push_back(factor);
    } else {
      factor->set_calibration(calibration_);
      __graph->add(factor);
    }
  }
  __uncalibratedFactors<TransformationType> = remainingFactors;
  return;
}

// ---------------------------------------------------------------------------
// Gets a point observation for a specified keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
output_t<point3d_t>
CovisibilityGraph<TransformationType>::get_point_observation(
    const point_id_t pointId_,
    const keyframe_id_t keyframeId_) {

  // This function:
  // 1 - Checks if the keyframeId is UNSET. If so, the point observation from
  //     the last added keyframe.
  // 2 - Looks for the observation in the queue first, if not found,
  //     looks for it in the covisibility graph.
  // 3 - If not found, returns fail.

  if (keyframeId_ == keyframe_id_t::UNSET)
    return __get_point_observation(pointId_);

  _observationEdges.lock_read();
  for (const ObservationEdge<TransformationType>& edge : _observationEdges) {
    const point_id_t& pointIdEdge = edge._pointId;
    const keyframe_id_t& keyframeIdEdge = edge._keyframeId;
    if (pointIdEdge == pointId_ && keyframeIdEdge == keyframeId_) {
      const point3d_t point = edge._observation.get_point();
      _observationEdges.unlock_read();
      return point;
    }
  }
  _observationEdges.unlock_read();

  // Then look for it in the covisibility graph
  __pointObservations<TransformationType>.lock_read();
  if (__pointObservations<TransformationType>.contains(pointId_)) {
    __pointObservations<TransformationType>[pointId_].lock_read();
    if (__pointObservations<TransformationType>[pointId_].contains(
            keyframeId_)) {
      const auto& observation =
          __pointObservations<TransformationType>[pointId_][keyframeId_];
      const point3d_t point = observation->get_point();
      __pointObservations<TransformationType>[pointId_].unlock_read();
      __pointObservations<TransformationType>.unlock_read();
      return point;
    }
    __pointObservations<TransformationType>[pointId_].unlock_read();
  }
  __pointObservations<TransformationType>.unlock_read();

  return fail_t::IS_EMPTY;
}

// ---------------------------------------------------------------------------
// [private] Gets a point observation for a specified keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
output_t<point3d_t>
CovisibilityGraph<TransformationType>::__get_point_observation(
    const point_id_t pointId_) {

  // This function:
  // 1 - Looks for the observation in the queue first, if not found,
  //     looks for it in the covisibility graph.
  // 2 - If not found, returns fail.

  // Look for the observation in the queue first
  _observationEdges.lock_read();
  for (const ObservationEdge<TransformationType>& edge : _observationEdges) {
    const point_id_t& pointIdEdge = edge._pointId;
    if (pointIdEdge == pointId_) {
      const PointObservation<TransformationType>& observation =
          edge._observation;
      const point3d_t point = observation.get_point();
      _observationEdges.unlock_read();
      return point;
    }
  }
  _observationEdges.unlock_read();

  __pointObservations<TransformationType>.lock_read();
  if (__pointObservations<TransformationType>.contains(pointId_)) {
    __pointObservations<TransformationType>[pointId_].lock_read();
    if (!__pointObservations<TransformationType>[pointId_].empty()) {
      const auto& observation =
          __pointObservations<TransformationType>[pointId_].rbegin()->second;
      const point3d_t point = observation->get_point();
      __pointObservations<TransformationType>[pointId_].unlock_read();
      __pointObservations<TransformationType>.unlock_read();
      return point;
    }
    __pointObservations<TransformationType>[pointId_].unlock_read();
  }
  __pointObservations<TransformationType>.unlock_read();

  return fail_t::IS_EMPTY;
}

// ---------------------------------------------------------------------------
// Gets a keypoint observation for a specified keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
output_t<point2d_t>
CovisibilityGraph<TransformationType>::get_keypoint_observation(
    const point_id_t pointId_,
    const keyframe_id_t keyframeId_) {

  // This function:
  // 1 - Checks if the keyframeId is UNSET. If so, the keypoint observation from
  //     the last added keyframe.
  // 2 - Looks for the observation in the queue first, if not found,
  //     looks for it in the covisibility graph.
  // 3 - If not found, returns fail.

  if (keyframeId_ == keyframe_id_t::UNSET)
    return __get_keypoint_observation(pointId_);

  // Queue first
  _observationEdges.lock_read();
  for (const ObservationEdge<TransformationType>& edge : _observationEdges) {
    if (edge._pointId == pointId_ && edge._keyframeId == keyframeId_) {
      _observationEdges.unlock_read();
      return edge._observation.get_keypoint();
    }
  }
  _observationEdges.unlock_read();

  // Then look for it in the covisibility graph
  __pointObservations<TransformationType>.lock_read();
  if (__pointObservations<TransformationType>.contains(pointId_)) {
    __pointObservations<TransformationType>[pointId_].lock_read();
    if (__pointObservations<TransformationType>[pointId_].contains(
            keyframeId_)) {
      const point2d_t keypoint =
          __pointObservations<TransformationType>[pointId_][keyframeId_]
          -> get_keypoint();
      __pointObservations<TransformationType>[pointId_].unlock_read();
      __pointObservations<TransformationType>.unlock_read();
      return keypoint;
    }
    __pointObservations<TransformationType>[pointId_].unlock_read();
  }
  __pointObservations<TransformationType>.unlock_read();

  // Since not found, return fail
  return fail_t::IS_EMPTY;
}

// ---------------------------------------------------------------------------
// [private] Gets a keypoint observation for a specified keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
output_t<point2d_t>
CovisibilityGraph<TransformationType>::__get_keypoint_observation(
    const point_id_t pointId_) {

  // This function:
  // 1 - Looks for the observation in the queue first, if not found,
  //     looks for it in the covisibility graph.
  // 2 - If not found, returns fail.

  // Queue first
  _observationEdges.lock_read();
  for (const ObservationEdge<TransformationType>& edge : _observationEdges) {
    if (edge._pointId == pointId_) {
      _observationEdges.unlock_read();
      return edge._observation.get_keypoint();
    }
  }
  _observationEdges.unlock_read();

  // Then look for it in the covisibility graph
  __pointObservations<TransformationType>.lock_read();
  if (__pointObservations<TransformationType>.contains(pointId_)) {
    __pointObservations<TransformationType>[pointId_].lock_read();
    if (!__pointObservations<TransformationType>[pointId_].empty()) {
      const auto& observation =
          __pointObservations<TransformationType>[pointId_].rbegin()->second;
      const point2d_t keypoint = observation->get_keypoint();
      __pointObservations<TransformationType>[pointId_].unlock_read();
      __pointObservations<TransformationType>.unlock_read();
      return keypoint;
    }
    __pointObservations<TransformationType>[pointId_].unlock_read();
  }
  __pointObservations<TransformationType>.unlock_read();
  return fail_t::IS_EMPTY;
}

// ---------------------------------------------------------------------------
// Fits a new descriptor observation for a point and keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::fit_new_descriptor(
    const point_id_t pointId_,
    const descriptor_t& descriptor_,
    const pose_t pose_) {
  using std::println;

  // This function will fit the descriptor model for the point with the new
  // observation. If the descriptor model does not exist, it will be created
  // and fitted with the new observation.

  __pointDescriptors.lock();
  if (__pointDescriptors.contains(pointId_)) {
    const output_t<point3d_t::weak_ptr> pointPtr = _map->at(pointId_);
    if (__fail_retrieve(pointPtr, pointId_)) goto END;
    const point3d_t point(pointPtr);
    if (__fail_set(point, pointId_)) goto END;
    __pointDescriptors[pointId_]->fit(descriptor_, pose_);
  }

END:
  __pointDescriptors.unlock();
  return;
}

// ---------------------------------------------------------------------------
// Fits a new descriptor observation for a point and keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::fit_new_descriptor(
    const point_id_t pointId_,
    const descriptor_t& descriptor_,
    const pose_t::weak_ptr posePtr_) {
  using std::println;

  const pose_t pose(posePtr_);
  if (__fail_set(pose)) return;
  this->fit_new_descriptor(pointId_, descriptor_, pose);
  return;
}

// ---------------------------------------------------------------------------
// Get a descriptor observation for a specified keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
output_t<descriptor_t>
CovisibilityGraph<TransformationType>::get_descriptor_observation(
    const point_id_t pointId_,
    const pose_t pose_) {
  using std::println;
  using utils::DescriptorModel;

  // This function:
  // 1 - Looks for the descriptor in the queue first, if not found,
  //     looks for it in the covisibility graph.
  // 2 - If not found, returns fail.

  __pointDescriptors.lock_read();
  if (__pointDescriptors.contains(pointId_)) {
    output_t<point3d_t::weak_ptr> pointWorldPtr = _map->at(pointId_);
    if (__fail_retrieve(pointWorldPtr, pointId_)) {
      __pointDescriptors.unlock_read();
      return fail_t::IS_EMPTY;
    }
    const point3d_t pointWorld(pointWorldPtr);
    if (__fail_set(pointWorld, pointId_)) {
      __pointDescriptors.unlock_read();
      return fail_t::IS_EMPTY;
    }
    DescriptorModel::shared_ptr descriptorModel = __pointDescriptors[pointId_];
    const auto descriptor = descriptorModel->predict(pose_, pointWorld);
    __pointDescriptors.unlock_read();
    return descriptor;
  }
  __pointDescriptors.unlock_read();

  _observationEdges.lock_read();
  for (const ObservationEdge<TransformationType>& edge : _observationEdges) {
    const point_id_t& pointIdEdge = edge._pointId;
    if (pointIdEdge == pointId_) {
      const auto descriptor = edge._observation.get_descriptor();
      _observationEdges.unlock_read();
      return descriptor;
    }
  }
  _observationEdges.unlock_read();
  return fail_t::IS_EMPTY;
}

// ---------------------------------------------------------------------------
// Get a descriptor observation for a specified keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
output_t<descriptor_t>
CovisibilityGraph<TransformationType>::get_descriptor_observation(
    const point_id_t pointId_,
    const pose_t::weak_ptr posePtr_) {
  using std::println;

  // Wrapper for get_descriptor_observation with explicit pose_t.

  const pose_t pose(posePtr_);
  if (!pose) {
    println(
        "[CovisibilityGraph] {}Warning{}: Failed to lock pose pointer when "
        "getting descriptor: {}.",
        yellow,
        reset,
        std::string(pointId_));
    return fail_t::IS_EMPTY;
  }
  return this->get_descriptor_observation(pointId_, pose);
}

// ---------------------------------------------------------------------------
// Gets IDs of all points observed by a keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
point_id_t::vector CovisibilityGraph<TransformationType>::get_point_ids(
    const keyframe_id_t keyframeId_) {

  // This function:
  // 1 - Checks if the keyframe has observations. If not, returns empty vector.
  // 2 - If so, returns the point IDs of all observations of the keyframe.

  point_id_t::vector ids;
  __keyframeObservations<TransformationType>.lock_read();

  if (__keyframeObservations<TransformationType>.contains(keyframeId_)) {
    __keyframeObservations<TransformationType>[keyframeId_].lock_read();
    ids.reserve(__keyframeObservations<TransformationType>[keyframeId_].size());
    for (const auto& [pointId, _] :
         __keyframeObservations<TransformationType>[keyframeId_])
      ids.emplace_back(pointId);
    __keyframeObservations<TransformationType>[keyframeId_].unlock_read();
  }

  __keyframeObservations<TransformationType>.unlock_read();
  return ids;
}

// ---------------------------------------------------------------------------
// Gets IDs of all keyframes observing a point
// ---------------------------------------------------------------------------
template <typename TransformationType>
keyframe_id_t::vector CovisibilityGraph<TransformationType>::get_keyframe_ids(
    const point_id_t pointId_) {

  // This function:
  // 1 - Checks if the point has observations. If not, returns empty vector.
  // 2 - If so, returns the keyframe IDs of all observations of the point.

  keyframe_id_t::vector ids;
  __pointObservations<TransformationType>.lock_read();

  if (__pointObservations<TransformationType>.contains(pointId_)) {
    __pointObservations<TransformationType>[pointId_].lock_read();
    ids.reserve(__pointObservations<TransformationType>[pointId_].size());
    for (const auto& [keyframeId, __] :
         __pointObservations<TransformationType>[pointId_])
      ids.emplace_back(keyframeId);
    __pointObservations<TransformationType>[pointId_].unlock_read();
  }

  __pointObservations<TransformationType>.unlock_read();
  return ids;
}

// ---------------------------------------------------------------------------
// Gets IDs of all keyframes connected to a keyframe
// ---------------------------------------------------------------------------
template <typename TransformationType>
keyframe_id_t::vector CovisibilityGraph<TransformationType>::get_connection_ids(
    const keyframe_id_t keyframeId_) {

  // This function:
  // 1 - Checks if the keyframe has connections. If not, returns empty vector.
  // 2 - If so, returns the keyframe IDs of all connections of the keyframe.

  keyframe_id_t::vector ids;
  __covisibilityConnections.lock_read();

  if (__covisibilityConnections.contains(keyframeId_)) {
    __covisibilityConnections[keyframeId_].lock_read();
    ids.reserve(__covisibilityConnections[keyframeId_].size());
    for (const keyframe_id_t& connectedId :
         __covisibilityConnections[keyframeId_])
      ids.emplace_back(connectedId);
    __covisibilityConnections[keyframeId_].unlock_read();
  }

  __covisibilityConnections.unlock_read();
  return ids;
}

// ---------------------------------------------------------------------------
// Gets the most up-to-date point ID in case it was merged
// ---------------------------------------------------------------------------
template <typename TransformationType>
point_id_t CovisibilityGraph<TransformationType>::get_up_to_date_point_id(
    const point_id_t pointId_) {

  // This function:
  // 1 - Checks if the point ID is in the __mergedPoints dictionary.
  //     If not, returns the point ID itself.
  // 2 - If so, returns the most up-to-date point ID.

  __mergedPoints.lock_read();
  point_id_t currentId = pointId_;
  while (__mergedPoints.contains(currentId))
    currentId = __mergedPoints[currentId];
  __mergedPoints.unlock_read();
  return currentId;
}

// ---------------------------------------------------------------------------
// Checks if a key exists in the graph
// ---------------------------------------------------------------------------
template <typename TransformationType>
bool CovisibilityGraph<TransformationType>::__exists(
    const gtsam::Symbol symbol_) {
  if (__graphKeys.contains_(gtsam::Key(symbol_))) return true;
  return false;
}

// ---------------------------------------------------------------------------
// Add a prior factor to the graph
// ---------------------------------------------------------------------------

template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__add_prior_factor(
    const keyframe_id_t keyframeId_,
    const TransformationType& pose_) {
  using PriorFactor = PosePriorFactor<TransformationType>;
  using gtsam::Symbol;
  using gtsam::noiseModel::Diagonal;

  Diagonal::shared_ptr noise = get_prior_pose_noise_model<TransformationType>();
  const Symbol symbol = Symbol('k', keyframeId_.key());
  PriorFactor priorFactor(symbol, pose_, noise);
  __graph->add(priorFactor);
}

// ---------------------------------------------------------------------------
// Optimization thread
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__optimize(std::stop_token token_) {
  using gtsam::ISAM2;
  using gtsam::ISAM2Params;
  using gtsam::Values;
  using slam_mer::settings::sharedSettings;

  // This function will do all the adjustment operations of SLAM:
  // 1 - Instanciate and ISAM optimizer with it's properties.
  // 2 - Loop at the desired frequency, and at each iteration:
  // 2.1 - Check if we have enough data to start optimizing. If not, skip
  //       optimization and wait for the next iteration.
  // 2.2 - If we have enough data, run the optimization
  // 2.3 - Update the map with the optimized values.
  // 2.4 - Pace the loop to run at the desired frequency.

  timer_t timer;
  GraphOptimizationManager optimizationManager;

  const int minimumFramesPerSecond = sharedSettings->covisibility_module_fps();
  const double& relinearize =
      sharedSettings->covisibility_relinearize_threshold();
  const int& skip = sharedSettings->covisibility_relinearize_skip();
  ISAM2Params parameters;
  parameters.relinearizeThreshold = relinearize;
  parameters.relinearizeSkip = skip;
  ISAM2 isam = ISAM2(parameters);
  uint64_t posteriorRevision = 0;

  while (token_.stop_requested() == false) {
    timer.tic();
    if (!__hasEnoughData.load()) {
      if (__graph->empty())
        __hasEnoughData.store(false);
      else
        __hasEnoughData.store(true);
    } else {
      bool isamUpdated = false;
      {
        write_lock_t lock(_mutex);
        isamUpdated = optimizationManager.run_optimization<TransformationType>(
            isam,
            __graph,
            __initialEstimates,
            __pointsToBeRemoved,
            _map,
            __runFullOptimization);
      }
      if (isamUpdated) {
        const auto values = std::make_shared<Values>(isam.calculateEstimate());
        // Extraction is report-only and outside the ingestion lock. A numeric
        // failure publishes unavailable for this revision without undoing estimates.
        _posterior.publish_after_update(
            isam,
            *values,
            ++posteriorRevision,
            [&] { __map_update(values); });
      }
    }
    timer.pace(duration_t::from_fps(minimumFramesPerSecond));
  }
}

// ---------------------------------------------------------------------------
// Update map points
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__map_update(
    const gtsam::Values::shared_ptr optimizedValues_) {
  using gtsam::Key;
  using gtsam::Point3;
  using gtsam::Symbol;
  using gtsam::Values;

  point_id_t::set updatedPoints;

  // ------------------------------------------------------------------------
  // Lambda functions
  auto ___sanity_keyframe_update =
      [&](const fail_t fail__, const keyframe_id_t& keyframeId__) -> void {
    if (fail__ != fail_t::NO_FAIL)
      __throw__(
          "Failed to update keyframe ID " + std::string(keyframeId__) +
          " in the map. Keyframes are not expected to fail to update since \
          they are always in the map.");
  };
  auto ___update_point = [&](const Symbol& symbol__,
                             const point3d_t& point__) -> void {
    const point_id_t id(symbol__.index());
    const fail_t fail = _map->update(id, point__);
    if (fail != fail_t::NO_FAIL)
      __pendingPoints.insert({id, point__});
    else
      updatedPoints.insert(id);
  };
  auto ___had_been_updated = [&](const point_id_t& pointId__) -> bool {
    if (updatedPoints.contains(pointId__)) return true;
    return false;
  };
  auto __sanity_check_point = [&](const point_id_t& pointId__) -> void {
    if (!__pointObservations<TransformationType>.contains(pointId__))
      __throw__(
          "__pointObservations<TransformationType> and "
          "__keyframeObservations<TransformationType> are out "
          "of sync");
  };
  auto ___non_single_observation_point =
      [&](const point_id_t& pointId__) -> bool {
    if (__pointObservations<TransformationType>[pointId__].size() >
        static_cast<size_t>(1))
      return true;
    return false;
  };
  auto ___point_in_map_fail = [&](const fail_t fail__,
                                  const point_id_t pointId__) -> bool {
    if (fail__ != fail_t::NO_FAIL) {
      log::message(
          "Point " + std::string(pointId__) +
              " [single observation] not updated in the map ",
          log::MAP);
      return true;
    }
    return false;
  };
  auto ___to_rotation_translation = [&](const TransformationType& pose__) {
    const rotation_t rotation(pose__.rotation().matrix());
    const translation_t translation = pose__.translation();
    return std::make_tuple(rotation, translation);
  };
  auto ___update_keyframe = [&](const Symbol& symbol__,
                                const TransformationType& pose__) {
    const keyframe_id_t keyframeId(symbol__.index());
    const auto [rotation, translation] = ___to_rotation_translation(pose__);
    double scale = 1.0;
    if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>)
      scale = pose__.scale();

    const output_t<keyframe_t::weak_ptr> priorKeyframe = _map->at(keyframeId);
    if (priorKeyframe != fail_t::NO_FAIL)
      __throw__(
          "Keyframe ID " + std::string(keyframeId) +
          " does not exist in the map.");

    const pose_t::weak_ptr priorPosePtr = keyframe_t::get_pose(priorKeyframe);
    const pose_t priorPose = pose_t(priorPosePtr);
    if (!priorPose) {
      __throw__(
          "Keyframe ID " + std::string(keyframeId) +
          " has no pose in the map.");
    }

    const auto [rotationPrior, translationPrior] =
        priorPose.get_rotation_translation();
    const rotation_t rotationPriorInv = rotationPrior.inverse();
    double scalePrior = 1.0;
    if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>)
      if (__keyframeScales.contains(keyframeId))
        scalePrior = __keyframeScales[keyframeId];

    // Update map points seen by the keyframe (only the ones with single
    // observations that are not yet in the graph)
    __pointObservations<TransformationType>.lock_read();
    __keyframeObservations<TransformationType>.lock_read();

    for (const auto& [pointId, __] :
         __keyframeObservations<TransformationType>[keyframeId]) {
      __sanity_check_point(pointId);
      if (___non_single_observation_point(pointId)) continue;
      const output_t<point3d_t::weak_ptr> pointWorldPtr = _map->at(pointId);
      if (___point_in_map_fail(pointWorldPtr, pointId)) continue;
      const point3d_t pointWorld(pointWorldPtr);
      point3d_t pointCamera =
          rotationPriorInv * (pointWorld - translationPrior);
      pointCamera.apply_scale(1.0 / scalePrior);
      pointCamera.apply_scale(scale);
      const point3d_t updatedPointWorld = rotation * pointCamera + translation;
      const Symbol pointSymbol = Symbol('p', pointId.key());
      ___update_point(pointSymbol, updatedPointWorld);
    }

    __keyframeObservations<TransformationType>.unlock_read();
    __pointObservations<TransformationType>.unlock_read();

    if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>)
      __keyframeScales[keyframeId] = scale;

    const fail_t fail = _map->update(keyframeId, pose_t(rotation, translation));
    ___sanity_keyframe_update(fail, keyframeId);
  };

  // Lambda function to send a keyframe to loop detection if it is the
  // first time it appears in the optimization
  // optimized
  auto ___send_to_loop_detection = [&](const keyframe_id_t& keyframeId__) {
#ifdef VISUAL_PLACE_LOCALIZATION
    if (!__optimizedKeyframeIds.contains_(keyframeId__)) {
      __optimizedKeyframeIds.insert_(keyframeId__);
      __insert_to_loop_verification(keyframeId__);
    }
#endif
  };
  // ------------------------------------------------------------------------

  // This function:
  // 1 - Iterate over the optimized values
  // 1.1 - We start by tracking the latest updated pose. This is necessary
  //       because there might be the change that some latest poses were in
  //       adjusting queues, but not optimized yet.
  // 1.2 - We update the pose on the map.
  // 1.3 - Send the trigger to the loop closure for consider this keyframe.
  // 2 - For the keyframes that are connected to the latest keyframe but were
  //     not yet updated in the covisibility graph:
  // 2.1 - we update their pose using the relative pose between the latest
  //       keyframe old and new poses.
  // 2.2 - We also update the points seen by those keyframes using the same
  //       relative pose.

  pair_t<keyframe_id_t, pose_t> latestKeyframeOldPose;
  pair_t<keyframe_id_t, pose_t> latestKeyframeNewPose;

  // 1
  for (const Values::ConstKeyValuePair& keyValue : *optimizedValues_) {
    const Key key = keyValue.key;
    const Symbol symbol(key);

    const char& code = symbol.chr();
    if (code == 'k') {

      const auto updatedPose = optimizedValues_->at<TransformationType>(key);

      // Get the keyframe ID old and new poses from the latest updated keyframe
      const keyframe_id_t keyframeId(symbol.index());
      const keyframe_id_t& latestKeyframeId = latestKeyframeOldPose.first;
      if (latestKeyframeId == keyframe_id_t::UNSET ||
          keyframeId > latestKeyframeId) {
        const output_t<keyframe_t::weak_ptr> keyframe = _map->at(keyframeId);
        __sanity_retrieve(keyframe, keyframeId);
        const pose_t::weak_ptr priorPosePtr = keyframe_t::get_pose(keyframe);
        __sanity_retrieve(priorPosePtr, keyframeId);
        const pose_t oldLatestPose(priorPosePtr);
        __sanity_pose(oldLatestPose, keyframeId);
        const auto [rotation, translation] =
            ___to_rotation_translation(updatedPose);
        const pose_t updatedLatestPose(rotation, translation);
        latestKeyframeOldPose = {keyframeId, oldLatestPose};
        latestKeyframeNewPose = {keyframeId, updatedLatestPose};
      }

      ___update_keyframe(symbol, updatedPose);
      ___send_to_loop_detection(keyframeId);
    }

    if (code == 'p') {
      const auto point = optimizedValues_->at<Point3>(key);
      const point3d_t updatedPoint3d = point3d_t(point);
      ___update_point(symbol, updatedPoint3d);
    }
  }

  // 2
  if (latestKeyframeOldPose.first != latestKeyframeNewPose.first)
    __throw__("Unexpected keyframe ID mismatch.");
  const keyframe_id_t& latestKeyframeId = latestKeyframeOldPose.first;
  if (latestKeyframeId == keyframe_id_t::UNSET) return;

  const auto newestKeyframeIds = _map->find_recent_keyframes(latestKeyframeId);
  const pose_t& oldPose = latestKeyframeOldPose.second;
  const pose_t& newPose = latestKeyframeNewPose.second;
  const pose_t oldPoseInv = oldPose.get_invert();
  const pose_t relativePoseAdjustment = oldPoseInv * newPose;
  for (const keyframe_id_t& keyframeId : newestKeyframeIds) {

    // Update Keyframe Pose
    const output_t<keyframe_t::weak_ptr> keyframe = _map->at(keyframeId);
    __sanity_retrieve(keyframe, keyframeId);
    const pose_t::weak_ptr keyframePosePtr = keyframe_t::get_pose(keyframe);
    __sanity_retrieve(keyframePosePtr, keyframeId);
    pose_t newestKeyframeOldPose = pose_t(keyframePosePtr);
    __sanity_pose(newestKeyframeOldPose, keyframeId);
    const pose_t newestKeyframeNewPose =
        newestKeyframeOldPose * relativePoseAdjustment;
    const fail_t fail = _map->update(keyframeId, newestKeyframeNewPose);
    ___sanity_keyframe_update(fail, keyframeId);

    // Update map points seen by the keyframe
    const point_id_t::vector pointIds = this->get_point_ids(keyframeId);
    newestKeyframeOldPose.invert();
    for (const point_id_t& pointId : pointIds) {
      // ignore if the point has already been updated
      if (___had_been_updated(pointId)) continue;
      const output_t<point3d_t::weak_ptr> pointWorldPtr = _map->at(pointId);
      if (pointWorldPtr != fail_t::NO_FAIL) {
        if (__mergedPoints.contains_(pointId))
          continue;
        else
          __throw__(pointWorldPtr.get_error());
      }
      const point3d_t pointWorld(pointWorldPtr);
      const point3d_t pointCamera = newestKeyframeOldPose * pointWorld;
      const point3d_t pointWorldUpdated = newestKeyframeNewPose * pointCamera;
      const Symbol pointSymbol = Symbol('p', pointId.key());
      ___update_point(pointSymbol, pointWorldUpdated);
    }
  }

  // Finally, we try to update any pending points that failed to update
  // in the map during the optimization.
  __process_pending_points();
}

// ---------------------------------------------------------------------------
// Process Pending Points
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::__process_pending_points() {
  using std::println;

  // --------------------------------------------------------------------------
  // Lambda functions
  auto ___still_in_map = [&](const fail_t fail__,
                             const point_id_t pointId__) -> void {
    if (fail__ == fail_t::NO_FAIL) return;
    if (fail__ == fail_t::NOT_FOUND) return;
    println(
        "{}WARNING{}: Failed to remove the point with id: {}.",
        yellow,
        reset,
        std::string(pointId__));
  };
  // --------------------------------------------------------------------------

  // Points that were not updated in the last optimization
  // are saved in __pendingPoints. We try to update them again here.
  // If they fail to be updated, they remain in the queue until they
  // are updated or time out:
  // 1 - Check if the point can be updated in the map. If so, update it and
  //     remove it. Clear all the points that were updated.
  // 2 - Next, we check for timeouts, i.e., in case points are in the queue
  //     for update for a long time, they probably got lost at some point,
  //     and should be removed from SLAM.
  // 2.1 - First we remove them from the observations
  // 2.2 - Finally, we tell the map to also ignore them, in case it is still
  //       there and the problem is somewhere else.

  vector_t<pair_t<point_id_t, point3d_t>> pulledOut;

  for (const auto& [pointId, point] : __pendingPoints) {
    const fail_t fail = _map->update(pointId, point);
    if (fail != fail_t::NO_FAIL) continue;
    pulledOut.push_back({pointId, point});
  }
  __pendingPoints.clear(pulledOut);

  for (const auto& [pointId, point] : __pendingPoints.timed_out()) {

    __pointObservations<TransformationType>.lock();
    __keyframeObservations<TransformationType>.lock();

    if (__pointObservations<TransformationType>.contains(pointId))
      __pointObservations<TransformationType>.erase(pointId);

    // FIX:
    // We dont need to go over all the keyframes. Before removing the
    // the point from the __pointObservations<TransformationType>, we can pull up all the
    // keyframes that are observing the point and only iterate over those.
    for (auto& [keyframeId, __] : __keyframeObservations<TransformationType>)
      if (__keyframeObservations<TransformationType>[keyframeId].contains(
              pointId))
        __keyframeObservations<TransformationType>[keyframeId].erase(pointId);

    __keyframeObservations<TransformationType>.unlock();
    __pointObservations<TransformationType>.unlock();

    const fail_t fail = _map->remove(pointId);
    ___still_in_map(fail, pointId);
  }

  __pendingPoints.clear_timed_out();
}

// ---------------------------------------------------------------------------
// Close Loop
// ---------------------------------------------------------------------------
template <typename TransformationType>
void CovisibilityGraph<TransformationType>::close_loop(
    const keyframe_id_t evalKeyframeId_,
    const keyframe_id_t loopKeyframeId_,
    const pose_t evalWorldToCamera_,
    const point_id_t::vector& evalKeyframePointIds_,
    const point_id_t::vector& loopKeyframePointIds_,
    match_t<index_t>::vector& matches_,
    std::vector<bool>& inliers_) {
  using covisibility_graph::OdometryEdge;
  using gtsam::Symbol;
  using std::println;

  // This function is doing two things:
  // 1 - Adds a new odometry edge that force the connection between
  //     two keyframes where the loop is detected. The relative point
  //     between these two cameras is computed among themselves, which
  //     adds a strong geometry prior for the loop.
  // 2 - Followed by the merge of 3D points that were marked to be the
  //     the same when the loop was detected.

  // -----------------------------------------------------------------------
  // Lambda functions

  const pose_t evalCameraToWorld = evalWorldToCamera_.get_invert();
  const output_t<pose_t> loopCameraToWorld_out = _map->get(loopKeyframeId_);
  __sanity_retrieve(loopCameraToWorld_out);
  const pose_t loopCameraToWorld = loopCameraToWorld_out;
  const pose_t loopWorldToCamera(loopCameraToWorld.get_invert());
  const pose_t evalToLoop = loopWorldToCamera * evalCameraToWorld;
  const OdometryEdge edge(loopKeyframeId_, evalKeyframeId_, evalToLoop, true);
  __add_odometry_edge(edge);

  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // Lambda functions
  auto ___same_point = [](const point_id_t& pointId1__,
                          const point_id_t& pointId2__,
                          const keyframe_id_t& evalKeyframeId__,
                          const keyframe_id_t& loopKeyframeId__) -> bool {
    if (pointId1__ == pointId2__) return true;
    println(
        "[CovisibilityGraph] {}Warning{}: Points {} and {} are being merged "
        "in the loop closure between keyframes {} and {}",
        yellow,
        reset,
        std::string(pointId1__),
        std::string(pointId2__),
        std::string(evalKeyframeId__),
        std::string(loopKeyframeId__));
    return false;
  };
  auto ___exists_points = [](const point_id_t& pointId1__,
                             const point_id_t& pointId2__) -> bool {
    __pointObservations<TransformationType>.lock_read();
    if (__pointObservations<TransformationType>.contains(pointId1__) &&
        __pointObservations<TransformationType>.contains(pointId2__)) {
      __pointObservations<TransformationType>.unlock_read();
      return true;
    }
    __pointObservations<TransformationType>.unlock_read();
    log::warning(
        "One of the points " + std::string(pointId1__) + " or " +
        std::string(pointId2__) +
        " to be merged does not exist "
        "in the graph. This may be due to a previous merge or a failed "
        "optimization. Skipping merge.");
    return false;
  };
  auto ___in_values = [this](const point_id_t& priorPointId__) -> void {
    // First ensure the priorPointId is in values
    // Also, if it only has one observation before the merge, the factor
    // associated to that observation is not in the graph yet, add it
    __pointObservations<TransformationType>[priorPointId__].lock_read();
    if (__pointObservations<TransformationType>[priorPointId__].size() ==
        static_cast<size_t>(1)) {
      // Get the only observation
      const auto& [keyframeId, observationPtr] =
          *__pointObservations<TransformationType>[priorPointId__].begin();
      // Add factors to the graph. This function will also ensure the
      // keyframe and point is in the values
      __add_observation_to_graph(priorPointId__, keyframeId, observationPtr);
    }
    __pointObservations<TransformationType>[priorPointId__].unlock_read();
  };
  auto ___update_factors =
      [](const point_id_t& evalPointId__,
         const point_id_t& loopPointId__,
         const keyframe_id_t& keyframeId__,
         const PointObservation<TransformationType>::shared_ptr&
             observationPtr__) -> void {
    // Update the projection and distance factors
    auto projectionFactorOutput = observationPtr__->get_projection_factor();
    if (!projectionFactorOutput.fail())
      observationPtr__->update_projection_factor_keys(
          loopPointId__,
          keyframeId__);

    auto distanceFactorOutput = observationPtr__->get_distance_factor();
    if (!distanceFactorOutput.fail())
      observationPtr__->update_distance_factor_keys(
          loopPointId__,
          keyframeId__);
  };
  auto ___update_observations =
      [](const keyframe_id_t keyframeId__,
         const point_id_t evalPointId__,
         const point_id_t loopPointId__,
         const PointObservation<TransformationType>::shared_ptr&
             observationPtr__) -> void {
    __keyframeObservations<TransformationType>[keyframeId__].lock();
    __keyframeObservations<TransformationType>[keyframeId__].erase(
        evalPointId__);
    __keyframeObservations<TransformationType>[keyframeId__].insert(
        {loopPointId__, observationPtr__});
    __keyframeObservations<TransformationType>[keyframeId__].unlock();
    __pointObservations<TransformationType>[loopPointId__].insert_(
        {keyframeId__, observationPtr__});
  };
  auto ___log_merge = [](const fail_t& fail__,
                         const point_id_t& evalPointId__,
                         const point_id_t& loopPointId__) -> void {
    if (fail__)
      log::warning(
          "[INFO] Unable to delete point ID: " + std::string(evalPointId__) +
              " that was merged into: " + std::string(loopPointId__) +
              " ==> point NOT deleted",
          log::MAP);
#ifdef LOG_ADJUSTMENT
    log::message(
        "Merged point " + std::string(evalPointId__) + " into point " +
            std::string(loopPointId__),
        log::MAP);
#endif
  };
  // -----------------------------------------------------------------------

  // This function:
  // 1 - Iteratee over all the matching candidates
  // 2 - Check if the ids of the points in the loop and eval keyframes are
  //     the same. If so, do nothing.
  // 3 - If not, we merge the points in the graph by:
  // 3.1 - We update the factors of the eval point to now point to the loop
  //       point. This is done in ___update_factors.
  // 3.2 - We update the observations of the keyframes that are observing the
  //       eval point to now observe the loop point instead. This is done
  //       in ___update_observations.
  // 3.3 - We add the pair of merged points to global::__mergedPoints, so
  //       we can keep track of the merges and get the most up-to-date
  //       point ID when needed.
  // 3.4 - Finally, we delete the eval point from the map, since it is
  //       now merged with the loop point.

  for (size_t i = 0; i < inliers_.size(); i++) {
    if (inliers_[i] == false) continue;  // If outlier do nothing

    // Get and validate point IDs
    const size_t first = static_cast<size_t>(matches_[i].first_element());
    const size_t second = static_cast<size_t>(matches_[i].second_element());
    const point_id_t evalPointId = evalKeyframePointIds_[first];
    const point_id_t loopPointId = loopKeyframePointIds_[second];
    if (___same_point(
            evalPointId,
            loopPointId,
            evalKeyframeId_,
            loopKeyframeId_))
      continue;

    if (!___exists_points(evalPointId, loopPointId)) continue;

    write_lock_t lock(_mutex);
    ___in_values(loopPointId);
    __pointObservations<TransformationType>[evalPointId].lock_read();

    for (const auto& [keyframeId, observationPtr] :
         __pointObservations<TransformationType>[evalPointId]) {

      ___update_factors(evalPointId, loopPointId, keyframeId, observationPtr);
      __add_keyframe_to_values(keyframeId, _map->at(keyframeId));
      ___update_observations(
          keyframeId,
          evalPointId,
          loopPointId,
          observationPtr);
    }
    __pointObservations<TransformationType>[evalPointId].unlock_read();
    __pointObservations<TransformationType>.erase_(evalPointId);

    __mergedPoints.insert_({evalPointId, loopPointId});
    const Symbol symbol('p', evalPointId.key());
    if (__exists(symbol)) __pointsToBeRemoved.insert_(evalPointId);

    // Delete the evalPoint point from the map
    fail_t fail = _map->remove(evalPointId);
    ___log_merge(fail, evalPointId, loopPointId);
  }

  // After merging the points, set flag to run a full optimization
  // to adjust the graph and fix the drift.
  // global::__runFullOptimization = true;

  return;
}

#ifdef SLAM_MER_COMPILE_SIMILARITY3
template class CovisibilityGraph<gtsam::Similarity3>;
#endif
#ifdef SLAM_MER_COMPILE_POSE3
template class CovisibilityGraph<gtsam::Pose3>;
#endif

}  // namespace covisibility_graph
}  // namespace slam_mer
