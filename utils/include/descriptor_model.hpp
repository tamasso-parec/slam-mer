/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file descriptor_model.hpp
/// @brief Descriptor-model interfaces and concrete descriptor strategies.
/// @details Declares the abstract DescriptorModel API and concrete
/// implementations used to aggregate or update point descriptors over
/// time. This interface is consumed by mapping and localization
/// modules.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef DESCRIPTOR_MODEL_HPP
#define DESCRIPTOR_MODEL_HPP

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <types/descriptor.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>

namespace slam_mer {
namespace utils {

// ---------------------------------------------------------------------------
// Auxiliary functions
// ---------------------------------------------------------------------------

/// @brief Computes the query vector from the point and pose.
/// The query vector is the normalized vector from the point to the camera
/// center
/// in world coordinates.
/// @param point3d_ The 3D point for which to compute the query vector.
/// @param pose_ The pose of the camera observing the point.
/// @return The normalized query vector from the point to the camera center
/// in world coordinates.
Eigen::Vector3d __compute_query_vector(
    const point3d_t& point3d_,
    const pose_t& pose_);

// ---------------------------------------------------------------------------
// Base class for descriptor models
// ---------------------------------------------------------------------------

/// The DescriptorModel class is an abstract base class for different types of
/// descriptor models. A descriptor model is a model that can predict a
/// descriptor
/// for a given pose and point, and can be updated with new descriptors.
class DescriptorModel {
public:
  /// Types of descriptor models
  enum class Type {
    /// Descriptor is the last one that was added to the model
    LAST_SEEN,
    /// Descriptor is the medoid of the added descriptors
    MEDOID,
    /// Descriptor is the arithmetic average of the added descriptors
    AVERAGE,
    /// Descriptor is the weighted average of the added descriptors
    WEIGHTED_AVERAGE,
  };

  /// Typedef for shared pointer of type DescriptorModel
  typedef std::shared_ptr<DescriptorModel> shared_ptr;

  /// @brief Factory method that creates a descriptor model instance.
  /// @param type_ Descriptor-model type to instantiate.
  /// @return Shared pointer to the requested descriptor-model implementation.
  static DescriptorModel::shared_ptr create(const Type& type_);

  /// Virtual destructor for the DescriptorModel class
  virtual ~DescriptorModel() = default;

  /// @brief Predicts a descriptor for the provided pose and point.
  /// @param pose_ Camera pose used by the model.
  /// @param point_ 3D point used by the model.
  /// @return Predicted descriptor or an error state.
  virtual output_t<descriptor_t> predict(
      const pose_t& pose_,
      const point3d_t& point_) = 0;

  /// @brief Updates the model with one descriptor observation.
  /// @param descriptor_ Descriptor measurement to ingest.
  /// @param pose_ Camera pose associated with the observation.
  /// @param point_ 3D point associated with the observation.
  virtual void fit(
      const descriptor_t& descriptor_,
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) = 0;

protected:
  /// @brief Validates the input point and throws if it is unset.
  /// @param point_ Point to validate.
  void __point_sanity_check(const point3d_t& point_) {
    if (point_.is_unset())
      __throw__("Invalid point provided to WeightedAverage");
  }

  /// @brief Validates the input pose and throws if it is unset.
  /// @param pose_ Pose to validate.
  void __pose_sanity_check(const pose_t& pose_) {
    if (pose_.is_unset()) __throw__("Invalid pose provided to WeightedAverage");
  }

  /// The current descriptor of the model. This is updated by the fit method
  /// and returned by the predict method.
  descriptor_t _descriptor;
  /// The number of observations that have been added to the model.
  size_t _numObservations = 0;
};

// ---------------------------------------------------------------------------
// LastSeen descriptor model
// ---------------------------------------------------------------------------

/// The LastSeen descriptor model is a simple model that always returns the
/// last descriptor that was added to it.
class LastSeen : public DescriptorModel {
public:
  /// Typedef for shared pointer of type LastSeen
  typedef std::shared_ptr<LastSeen> shared_ptr;
  /// @brief Creates a shared `LastSeen` instance.
  /// @return Shared pointer to a new `LastSeen` object.
  static shared_ptr make_shared() { return std::make_shared<LastSeen>(); }

  /// Last Seen descriptor model constructor
  LastSeen() = default;
  /// Last Seen descriptor model destructor
  ~LastSeen() override = default;

  /// @brief Predicts the descriptor for a given pose and point.
  /// @param pose_ The pose for which to predict the descriptor
  /// (not used in this model).
  /// @param point_ The point for which to predict the descriptor
  /// (not used in this model).
  /// @return The last descriptor that was added to the model.
  inline output_t<descriptor_t> predict(
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) override {
    if (_numObservations == 0) return output_t<descriptor_t>(fail_t::IS_EMPTY);
    return output_t<descriptor_t>(_descriptor);
  }

  /// @brief Adds a new descriptor to the model.
  /// The current descriptor is updated to be the last one added.
  /// @param descriptor_ The new descriptor to add to the model.
  /// @param pose_ The pose of the keyframe that observed the descriptor
  /// (not used in this model).
  /// @param point_ The 3d point observation associated with the new descriptor
  /// (not used in this model).
  inline void fit(
      const descriptor_t& descriptor_,
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) override {
    _descriptor = descriptor_;
    _numObservations++;
  }
};

// ---------------------------------------------------------------------------
// Medoid descriptor model
// ---------------------------------------------------------------------------

/// The Medoid descriptor model computes the medoid of the added descriptors and
/// returns it as the current descriptor. The medoid is the descriptor that has
/// the smallest average distance to the other descriptors.
class Medoid : public DescriptorModel {
public:
  /// Typedef for shared pointer of type Medoid
  typedef std::shared_ptr<Medoid> shared_ptr;
  /// @brief Creates a shared `Medoid` instance.
  /// @return Shared pointer to a new `Medoid` object.
  static shared_ptr make_shared() { return std::make_shared<Medoid>(); }

  /// Medoid descriptor model constructor
  Medoid() = default;
  /// Medoid descriptor model destructor
  ~Medoid() override = default;

  /// @brief Predicts the descriptor for a given pose and point.
  /// @param pose_ The pose for which to predict the descriptor
  /// (not used in this model).
  /// @param point_ The point for which to predict the descriptor
  /// (not used in this model).
  /// @return The medoid descriptor of the added descriptors.
  inline output_t<descriptor_t> predict(
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) override {
    if (_numObservations == 0) return output_t<descriptor_t>(fail_t::IS_EMPTY);
    return output_t<descriptor_t>(_descriptor);
  }

  /// @brief Adds a new descriptor to the model and updates the
  /// current descriptor to be the medoid of the added descriptors.
  /// @param descriptor_ The new descriptor to add to the model.
  /// @param pose_ The pose of the keyframe that observed the descriptor
  /// (not used in this model).
  /// @param point_ The 3d point observation associated with the new descriptor
  /// (not used in this model).
  void fit(
      const descriptor_t& descriptor_,
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) override;

private:
  /// History of added descriptors
  vector_t<descriptor_t> _descriptorHistory;
  /// Cached unit-normalized descriptors for incremental medoid updates.
  std::vector<std::vector<float>> _unitDescriptorHistory;
  /// Running sum of squared distances per descriptor.
  std::vector<float> _squaredDistanceSums;
};

// ---------------------------------------------------------------------------
// Average descriptor model
// ---------------------------------------------------------------------------

/// The Average descriptor model computes the arithmetic average of the
/// added descriptors and returns it as the current descriptor.
class Average : public DescriptorModel {
public:
  /// Typedef for shared pointer of type Average
  typedef std::shared_ptr<Average> shared_ptr;
  /// @brief Creates a shared `Average` instance.
  /// @return Shared pointer to a new `Average` object.
  static shared_ptr make_shared() { return std::make_shared<Average>(); }

  /// Average descriptor model constructor
  Average() = default;
  /// Average descriptor model destructor
  ~Average() override = default;

  /// @brief Predicts the descriptor for a given pose and point.
  /// @param pose_ The pose for which to predict the descriptor
  /// (not used in this model).
  /// @param point_ The point for which to predict the descriptor
  /// (not used in this model).
  /// @return The arithmetic average descriptor of the added descriptors.
  inline output_t<descriptor_t> predict(
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) override {
    if (_numObservations == 0) return output_t<descriptor_t>(fail_t::IS_EMPTY);
    return output_t<descriptor_t>(_descriptor);
  }

  /// @brief Adds a new descriptor to the model.
  /// @param descriptor_ The new descriptor to add to the model.
  /// @param pose_ The pose of the keyframe that observed the descriptor
  /// (not used in this model).
  /// @param point_ The 3d point observation associated with the new descriptor
  /// (not used in this model).
  void fit(
      const descriptor_t& descriptor_,
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) override;

private:
  /// Sum of the added descriptors. This is used to compute the average
  /// descriptor efficiently without having to store all the descriptors
  /// in memory.
  std::vector<float> _sumDescriptor;
};

// ---------------------------------------------------------------------------
// Weighted average descriptor model
// ---------------------------------------------------------------------------

/// The Weighted Average descriptor model computes the weighted average of the
/// added descriptors and returns it as the current descriptor. The weight is
/// based on the angle between the view direction of the observation and the
/// view direction of the added descriptors.
class WeightedAverage : public DescriptorModel {
public:
  /// Typedef for shared pointer of type WeightedAverage
  typedef std::shared_ptr<WeightedAverage> shared_ptr;
  /// @brief Creates a shared `WeightedAverage` instance.
  /// @return Shared pointer to a new `WeightedAverage` object.
  static shared_ptr make_shared() {
    return std::make_shared<WeightedAverage>();
  }

  /// Weighted Average descriptor model constructor
  WeightedAverage() = default;
  /// Weighted Average descriptor model destructor
  ~WeightedAverage() override = default;

  /// @brief Predicts the descriptor for a given pose and point.
  /// @param pose_ The pose for which to predict the descriptor.
  /// @param point_ The point for which to predict the descriptor.
  /// @return The weighted average descriptor of the added descriptors.
  inline output_t<descriptor_t> predict(
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) override {
    __pose_sanity_check(pose_);
    __point_sanity_check(point_);

    // If no descriptors have been added, return fail
    if (_numObservations == 0) return output_t<descriptor_t>(fail_t::IS_EMPTY);

    // If only one descriptor has been added, return that descriptor
    if (_numObservations == 1) {
      _descriptor = _descriptorHistory.front();
      return output_t<descriptor_t>(_descriptor);
    }
    // otherwise, compute the weighted average of the descriptors based on the
    // query pose and point and return it
    __compute_weighted_average(point_, pose_);
    return output_t<descriptor_t>(_descriptor);
  }

  /// @brief Adds a new descriptor to the model.
  /// @param descriptor_ The new descriptor to add to the model.
  /// @param pose_ The pose of the keyframe that observed the descriptor.
  /// @param point_ The 3d point observation associated with the new descriptor.
  inline void fit(
      const descriptor_t& descriptor_,
      const pose_t& pose_ = pose_t(),
      const point3d_t& point_ = point3d_t()) override {
    // Save the new descriptor and its view vector in the history
    // No need to compute the weighted average here,
    // since it will be computed in the predict method depending on
    // the query pose and point.
    __pose_sanity_check(pose_);
    __point_sanity_check(point_);

    _descriptorHistory.push_back(descriptor_);
    const Eigen::Vector3d viewVector = __compute_query_vector(point_, pose_);
    _viewVectorHistory.push_back(viewVector);
    _numObservations++;
  }

private:
  /// @brief Computes the weighted average of the added descriptors.
  /// @param point_ The 3d point for which to compute the weighted average.
  /// @param pose_ The pose for which to compute the weighted average.
  void __compute_weighted_average(const point3d_t& point_, const pose_t& pose_);

  /// History of added descriptors
  vector_t<descriptor_t> _descriptorHistory;
  /// History of view vectors corresponding to the added descriptors
  vector_t<Eigen::Vector3d> _viewVectorHistory;
  /// The alpha parameter for the weighted average computation.
  double _alpha = 10.0;
};

}  // namespace utils
}  // namespace slam_mer

#endif  // DESCRIPTOR_MODEL_HPP
