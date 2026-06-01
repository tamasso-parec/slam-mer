/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file observation_edge.hpp
/// @brief Observation-edge data structures for keyframe-point constraints.
/// @details Declares containers representing point observations in keyframes,
/// including descriptors and measurements used to build covisibility factors
/// and optimization edges in the adjustment backend.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef OBSERVATION_EDGE_HPP
#define OBSERVATION_EDGE_HPP

// SLAM-MER includes
#include <types/descriptor.hpp>
#include <types/id.hpp>
#include <types/mutex.hpp>
#include <types/output.hpp>
#include <types/point2d.hpp>
#include <types/pose.hpp>
#include <types/queue.hpp>
#include <types/vector.hpp>

#include <covisibility_factors.hpp>

namespace slam_mer {
namespace covisibility_graph {

/// @brief This class defines a point observation made by a keyframe.
/// It holds the 3D point in camera reference frame, the 2D keypoint,
/// descriptor, inlier flag, and optional factors.
template <typename TransformationType>
class PointObservation {

public:
  /// Typedef to a vector_t of type PointObservation.
  using vector = vector_t<PointObservation>;
  /// Typedef to a std::shared_ptr of type PointObservation.
  using shared_ptr = std::shared_ptr<PointObservation>;
  /// Typedef to a std::unique_ptr of type PointObservation.
  using unique_ptr = std::unique_ptr<PointObservation>;

  /// Typedef for the Typed projection factor
  using ProjectionFactorTyped = ProjectionFactor<TransformationType>;
  /// Typedef to the typed point distance factor
  using PointDistanceFactorTyped = PointDistanceFactor<TransformationType>;

  /// @brief Allocates a new PointObservation object.
  /// @param keypoint_ 2D observation of the keypoint in the image.
  /// @param point_ 3D position of the point in the image reference frame.
  /// @param descriptor_ Descriptor for the keypoint in the image.
  /// @param cameraPose_ Camera pose of the observation in world coordinates.
  /// @param inlier_ Flag to indicate if the point is an inlier.
  /// @param projectionFactor_ Pointer to a projection factor.
  /// @param distanceFactor_ Pointer to a distance factor.
  /// @return Shared pointer owning the new PointObservation object.
  [[nodiscard("Use returned shared_ptr to keep point-observation ownership")]]
  static PointObservation::shared_ptr make_shared(
      const point2d_t& keypoint_,
      const point3d_t& point_,
      const descriptor_t& descriptor_,
      const pose_t& cameraPose_,
      const bool inlier_ = true,
      const ProjectionFactorTyped::shared_ptr& projectionFactor_ = nullptr,
      const PointDistanceFactorTyped::shared_ptr& distanceFactor_ = nullptr) {
    return std::make_shared<PointObservation>(
        keypoint_,
        point_,
        descriptor_,
        cameraPose_,
        inlier_,
        projectionFactor_,
        distanceFactor_);
  }

  /// @brief Allocates a new PointObservation object.
  /// @param observation_ PointObservation object.
  /// @param projectionFactor_ Pointer to a projection factor.
  /// @param distanceFactor_ Pointer to a distance factor.
  /// @return Shared pointer owning the new PointObservation object.
  [[nodiscard("Use returned shared_ptr to keep point-observation ownership")]]
  static PointObservation::shared_ptr make_shared(
      const PointObservation& observation_,
      const ProjectionFactorTyped::shared_ptr& projectionFactor_ = nullptr,
      const PointDistanceFactorTyped::shared_ptr& distanceFactor_ = nullptr) {
    return std::make_shared<PointObservation>(
        observation_._keypoint,
        observation_._point,
        observation_._descriptor,
        observation_._cameraPose,
        observation_._inlier,
        projectionFactor_,
        distanceFactor_);
  }

  /// @brief Allocates a new PointObservation object.
  /// @param observation_ PointObservation object.
  /// @return Shared pointer owning the copied PointObservation object.
  [[nodiscard("Use returned shared_ptr to keep point-observation ownership")]]
  static PointObservation::shared_ptr make_shared(
      const PointObservation& observation_) {
    return std::make_shared<PointObservation>(
        observation_._keypoint,
        observation_._point,
        observation_._descriptor,
        observation_._cameraPose,
        observation_._inlier,
        observation_._projectionFactor,
        observation_._distanceFactor);
  }

  /// @brief Construct a new PointObservation object.
  /// @param keypoint_ 2D observation of the keypoint in the image.
  /// @param point_ 3D position of the point in the image reference frame.
  /// @param descriptor_ Descriptor for the keypoint in the image.
  /// @param cameraPose_ Camera pose of the observation in world coordinates.
  /// @param inlier_ Flag to indicate if the point is an inlier.
  /// @param projectionFactor_ Pointer to a projection factor.
  /// @param distanceFactor_ Pointer to a distance factor.
  PointObservation(
      const point2d_t& keypoint_,
      const point3d_t& point_,
      const descriptor_t& descriptor_,
      const pose_t& cameraPose_,
      const bool inlier_ = true,
      const ProjectionFactorTyped::shared_ptr& projectionFactor_ = nullptr,
      const PointDistanceFactorTyped::shared_ptr& distanceFactor_ = nullptr)
      : _keypoint(keypoint_),
        _point(point_),
        _descriptor(descriptor_),
        _cameraPose(cameraPose_),
        _inlier(inlier_),
        _projectionFactor(projectionFactor_),
        _distanceFactor(distanceFactor_) {}

  /// @brief Construct a PointObservation object.
  /// @param other_ Shared pointer to a PointObservation object.
  PointObservation(const PointObservation::shared_ptr& other_)
      : _keypoint(other_->_keypoint),
        _point(other_->_point),
        _descriptor(other_->_descriptor),
        _cameraPose(other_->_cameraPose),
        _inlier(other_->_inlier),
        _projectionFactor(other_->_projectionFactor),
        _distanceFactor(other_->_distanceFactor) {}

  /// @brief Construct a PointObservation object.
  /// @param other_ PointObservation object.
  PointObservation(const PointObservation& other_)
      : _keypoint(other_._keypoint),
        _point(other_._point),
        _descriptor(other_._descriptor),
        _cameraPose(other_._cameraPose),
        _inlier(other_._inlier),
        _projectionFactor(other_._projectionFactor),
        _distanceFactor(other_._distanceFactor) {}

  /// @brief Construct a default PointObservation object.
  PointObservation()
      : _keypoint(point2d_t()),
        _point(point3d_t()),
        _descriptor(descriptor_t()),
        _cameraPose(pose_t()),
        _inlier(false),
        _projectionFactor(nullptr),
        _distanceFactor(nullptr) {}

  /// @brief Destroy a PointObservation object.
  ~PointObservation() {}

  /// @brief Assignment operator for PointObservation.
  /// @param other_ PointObservation object.
  /// @return Reference to the updated observation.
  PointObservation& operator=(const PointObservation& other_) {
    if (this != &other_) {
      write_lock_t lockThis(_mutex);
      read_lock_t lockOther(other_._mutex);
      _keypoint = other_._keypoint;
      _point = other_._point;
      _descriptor = other_._descriptor;
      _cameraPose = other_._cameraPose;
      _inlier = other_._inlier;
      _projectionFactor = other_._projectionFactor;
      _distanceFactor = other_._distanceFactor;
    }
    return *this;
  }

  /// @brief Gets the 3D point observation (image reference frame).
  /// @return 3D point observation (image reference frame).
  [[nodiscard("Use returned point to handle empty-observation failures")]]
  output_t<point3d_t> get_point() const {
    read_lock_t lock(_mutex);
    if (_point == point3d_t()) return fail_t::IS_EMPTY;
    return _point;
  }

  /// @brief Gets the 2D observation of the keypoint in the image.
  /// @return 2D observation of the keypoint in the image.
  [[nodiscard("Use returned keypoint to handle empty-observation failures")]]
  output_t<point2d_t> get_keypoint() const {
    read_lock_t lock(_mutex);
    if (_keypoint == point2d_t()) {
      return fail_t::IS_EMPTY;
    }
    return _keypoint;
  }

  /// @brief Gets the descriptor for the point.
  /// @return Descriptor for the point.
  [[nodiscard("Use returned descriptor to handle empty-observation failures")]]
  output_t<descriptor_t> get_descriptor() const {
    read_lock_t lock(_mutex);
    if (descriptor_t::is_empty(_descriptor)) return fail_t::IS_EMPTY;
    return _descriptor;
  }

  /// @brief Gets the camera pose of the observation in world coordinates.
  /// @return Camera pose of the observation in world coordinates.
  [[nodiscard("Use returned pose to handle empty-observation failures")]]
  output_t<pose_t> get_camera_pose() const {
    read_lock_t lock(_mutex);
    if (_cameraPose.is_unset()) return fail_t::IS_EMPTY;
    return _cameraPose;
  }

  /// @brief Gets the projection factor for the observation.
  /// @return Projection factor for the observation.
  [[nodiscard("Use returned projection-factor to handle missing factors")]]
  output_t<typename ProjectionFactorTyped::shared_ptr> get_projection_factor()
      const {
    read_lock_t lock(_mutex);
    if (_projectionFactor == nullptr) return fail_t::IS_EMPTY;
    return _projectionFactor;
  }

  /// @brief Gets the distance factor for the observation.
  /// @return Distance factor for the observation.
  [[nodiscard("Use returned distance-factor to handle missing factors")]]
  output_t<typename PointDistanceFactorTyped::shared_ptr> get_distance_factor()
      const {
    read_lock_t lock(_mutex);
    if (_distanceFactor == nullptr) return fail_t::IS_EMPTY;
    return _distanceFactor;
  }

  /// @brief Updates the keys of the projection factor.
  /// @param pointId_ Point Id of the factor.
  /// @param keyframeId_ Keyframe Id of the factor.
  void update_projection_factor_keys(
      const point_id_t pointId_,
      const keyframe_id_t keyframeId_) {
    write_lock_t lock(_mutex);
    if (_projectionFactor == nullptr) return;
    _projectionFactor = ProjectionFactorTyped::make_shared(
        _projectionFactor,
        keyframeId_,
        pointId_);
    return;
  }

  /// @brief Updates the keys of the distance factor.
  /// @param pointId_ Point Id of the factor.
  /// @param keyframeId_ Keyframe Id of the factor.
  void update_distance_factor_keys(
      const point_id_t pointId_,
      const keyframe_id_t keyframeId_) {
    write_lock_t lock(_mutex);
    if (_distanceFactor == nullptr) return;
    _distanceFactor = PointDistanceFactorTyped::make_shared(
        _distanceFactor,
        keyframeId_,
        pointId_);
    return;
  }

  /// @brief Check if the edge is an inlier.
  /// @return True if the edge is an inlier, false otherwise.
  [[nodiscard("Use returned bool to branch on observation inlier status")]]
  bool is_inlier() const {
    read_lock_t lock(_mutex);
    return _inlier;
  }

  /// @brief Sets the inlier status of the point observation.
  /// @param inlier_ True to set the point as inlier, false otherwise.
  void set_inlier(bool inlier_) {
    write_lock_t lock(_mutex);
    _inlier = inlier_;
  }

private:
  /// 2D observation of the keypoint in the image.
  point2d_t _keypoint;
  /// 3D position of the point in the image reference frame.
  point3d_t _point;
  /// Descriptor for the keypoint in the image.
  descriptor_t _descriptor;
  /// Camera pose of the observation in world coordinates.
  pose_t _cameraPose;
  /// Flag to indicate if the point is an inlier.
  bool _inlier;
  /// Pointer to a projection factor.
  ProjectionFactorTyped::shared_ptr _projectionFactor;
  /// Pointer to a distance factor.
  PointDistanceFactorTyped::shared_ptr _distanceFactor;

  /// Mutex to protect access to the edge data.
  mutable mutex_t _mutex;
};

/// @brief This class defines the edge between a map point and a keyframe.
template <typename TransformationType>
class ObservationEdge {

public:
  /// Map point Id.
  point_id_t _pointId;
  /// Keyframe Id.
  keyframe_id_t _keyframeId;
  /// Observation data for the point in the keyframe.
  PointObservation<TransformationType> _observation;

  /// Typedef for a std::shared_ptr of ObservationEdge.
  using shared_ptr = std::shared_ptr<ObservationEdge>;
  /// Typedef for a vector_t of ObservationEdge.
  using vector = vector_t<ObservationEdge>;
  /// Typedef for a queue_t of ObservationEdge.
  using queue = queue_t<ObservationEdge>;

  /// @brief Construct an ObservationEdge object.
  /// @param pointId_ ID of the point being observed.
  /// @param keyframeId_ ID of the keyframe observing the point.
  /// @param observation_ Observation data for the point in the keyframe.
  ObservationEdge(
      const point_id_t& pointId_,
      const keyframe_id_t& keyframeId_,
      const PointObservation<TransformationType>& observation_)
      : _pointId(pointId_),
        _keyframeId(keyframeId_),
        _observation(observation_) {}

  /// @brief Construct an ObservationEdge object.
  /// @param other_ Shared pointer to an ObservationEdge object.
  ObservationEdge(const ObservationEdge::shared_ptr& other_)
      : _pointId(other_->_pointId),
        _keyframeId(other_->_keyframeId),
        _observation(other_->_observation) {}

  /// @brief Construct an ObservationEdge object.
  /// @param other_ ObservationEdge object.
  ObservationEdge(const ObservationEdge& other_)
      : _pointId(other_._pointId),
        _keyframeId(other_._keyframeId),
        _observation(other_._observation) {}

  /// @brief Construct a default ObservationEdge object.
  ObservationEdge()
      : _pointId(point_id_t::UNSET),
        _keyframeId(keyframe_id_t::UNSET),
        _observation(PointObservation<TransformationType>()) {}

  /// @brief Destroys an ObservationEdge object.
  ~ObservationEdge() {}

  /// @brief Assignment operator for ObservationEdge.
  /// @param other_ ObservationEdge object.
  /// @return Reference to the updated edge.
  ObservationEdge& operator=(const ObservationEdge& other_) {
    if (this != &other_) {
      _pointId = other_._pointId;
      _keyframeId = other_._keyframeId;
      _observation = other_._observation;
    }
    return *this;
  }
};

}  // namespace covisibility_graph
}  // namespace slam_mer

#endif  // OBSERVATION_EDGE_HPP
