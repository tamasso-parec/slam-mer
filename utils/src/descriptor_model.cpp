/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file descriptor_model.cpp
/// @brief Descriptor-model implementation and factory behavior.
/// @details Implements DescriptorModel creation and concrete descriptor-update
/// strategies. This translation unit contains runtime behavior behind
/// the public API.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <descriptor_model.hpp>
#include <types/array.hpp>
#include <types/output.hpp>
#include <types/translation.hpp>

namespace slam_mer {
namespace utils {

/// @brief A small constant for numerical stability when normalizing vectors.
constexpr double EPSILON = 1e-12;

// ---------------------------------------------------------------------------
// Auxiliary functions
// ---------------------------------------------------------------------------

/// @brief Computes the raw query vector from the point and pose.
/// The raw query vector is the vector from the point to the camera center
/// in world coordinates.
/// @param point3d_ The 3D point for which to compute the query vector.
/// @param pose_ The pose of the camera observing the point.
/// @return The raw query vector from the point to the camera center in world
/// coordinates.
Eigen::Vector3d __compute_raw_query_vector(
    const point3d_t& point3d_,
    const pose_t& pose_) {
  // World-space view direction: camera_center - point.
  const auto [_, translation] = pose_.get_rotation_translation();
  const array3D_t arrayPoint = point3d_;
  const array3D_t arrayTranslation = translation;
  const double& px = arrayPoint[0];
  const double& py = arrayPoint[1];
  const double& pz = arrayPoint[2];
  const double& tx = arrayTranslation[0];
  const double& ty = arrayTranslation[1];
  const double& tz = arrayTranslation[2];
  const Eigen::Vector3d worldPoint(px, py, pz);
  const Eigen::Vector3d cameraCenter(tx, ty, tz);
  return cameraCenter - worldPoint;
}

// ---------------------------------------------------------------------------
// Compute query vector and normalize it to compute the weighted average
// descriptor
// ---------------------------------------------------------------------------

Eigen::Vector3d __compute_query_vector(
    const point3d_t& point3d_,
    const pose_t& pose_) {
  Eigen::Vector3d queryVector = __compute_raw_query_vector(point3d_, pose_);
  const double norm = queryVector.norm();
  if (norm <= EPSILON) return Eigen::Vector3d::Zero();
  return queryVector / norm;
}

/// @brief Normalizes a descriptor vector to have unit L2 norm.
/// @param descriptorData_ The descriptor data to normalize. This is modified in
/// place to have unit L2 norm.
void __normalize_l2(std::vector<float>& descriptorData_) {
  double squaredNorm = 0.0;
  for (const float value : descriptorData_)
    squaredNorm += static_cast<double>(value) * static_cast<double>(value);

  const double norm = std::sqrt(squaredNorm);
  if (norm <= EPSILON) return;
  for (float& value : descriptorData_)
    value = static_cast<float>(static_cast<double>(value) / norm);
  return;
}

// ---------------------------------------------------------------------------
// Factory method to create a descriptor model based on a string identifier
// ---------------------------------------------------------------------------

/// @brief Creates a descriptor-model implementation for the requested type.
/// @param type_ Descriptor model type identifier.
/// @return Shared pointer to the corresponding descriptor-model instance.
DescriptorModel::shared_ptr DescriptorModel::create(
    const DescriptorModel::Type& type_) {
  if (type_ == DescriptorModel::Type::LAST_SEEN) {
    return LastSeen::make_shared();
  } else if (type_ == DescriptorModel::Type::MEDOID) {
    return Medoid::make_shared();
  } else if (type_ == DescriptorModel::Type::AVERAGE) {
    return Average::make_shared();
  } else if (type_ == DescriptorModel::Type::WEIGHTED_AVERAGE) {
    return WeightedAverage::make_shared();
  } else {
    __throw__("Unknown descriptor model type");
  }
}

// ---------------------------------------------------------------------------
// Compute the medoid descriptor from a set of descriptors.
// ---------------------------------------------------------------------------
void Medoid::fit(
    const descriptor_t& descriptor_,
    const pose_t& pose_,
    const point3d_t& point_) {

  const std::vector<float>& descriptorData = descriptor_.get_data();
  const size_t descriptorDimension = descriptorData.size();

  // Save the new descriptor in the history and increment the observations
  _descriptorHistory.push_back(descriptor_);
  _numObservations++;

  // Compute the unit-normalized descriptor and save it in the history for
  // incremental medoid updates
  _unitDescriptorHistory.emplace_back(descriptorDimension, 0.0f);
  std::vector<float>& newUnitDescriptor = _unitDescriptorHistory.back();
  double squaredNorm = 0.0;
  for (const float value : descriptorData) {
    squaredNorm += static_cast<double>(value) * static_cast<double>(value);
  }
  if (squaredNorm > EPSILON) {
    const float inverseNorm = static_cast<float>(1.0 / std::sqrt(squaredNorm));
    for (size_t valueId = 0; valueId < descriptorDimension; ++valueId) {
      newUnitDescriptor[valueId] = descriptorData[valueId] * inverseNorm;
    }
  }

  // Set up the squared distance sums for the new descriptor
  _squaredDistanceSums.push_back(0.0f);

  // If its the first observation, set the current descriptor to that descriptor
  const size_t descriptorCount = _numObservations;
  if (descriptorCount == 1) {
    _descriptor = descriptor_;
    return;
  }

  // Update the squared distance sums for each descriptor with the new
  // descriptor
  const size_t newDescriptorId = descriptorCount - 1;
  for (size_t descriptorId = 0; descriptorId < newDescriptorId;
       ++descriptorId) {
    // the unitDescriptor a previous descriptor
    const std::vector<float>& currentDescriptor =
        _unitDescriptorHistory[descriptorId];
    // compute the cosine similarity between a previous descriptor
    // and the new descriptor
    float cosineSimilarity = 0.0f;
    for (size_t valueId = 0; valueId < descriptorDimension; ++valueId) {
      cosineSimilarity +=
          currentDescriptor[valueId] * newUnitDescriptor[valueId];
    }
    cosineSimilarity = std::clamp(cosineSimilarity, -1.0f, 1.0f);
    // compute the squared distance from the cosine similarity
    // and update the squared distance sums for the previous and new descriptors
    const float squaredDistance =
        std::max(0.0f, 2.0f - 2.0f * cosineSimilarity);
    _squaredDistanceSums[descriptorId] += squaredDistance;
    _squaredDistanceSums[newDescriptorId] += squaredDistance;
  }

  // Find the descriptor with the smallest average squared distance to the other
  // descriptors and set it as the current descriptor
  float bestDistance = std::numeric_limits<float>::infinity();
  size_t bestIndex = 0;
  for (size_t descriptorId = 0; descriptorId < descriptorCount;
       ++descriptorId) {
    if (_squaredDistanceSums[descriptorId] < bestDistance) {
      bestDistance = _squaredDistanceSums[descriptorId];
      bestIndex = descriptorId;
    }
  }
  _descriptor = _descriptorHistory[bestIndex];
}

// ---------------------------------------------------------------------------
// Compute the arithmetic average descriptor from a set of descriptors.
// ---------------------------------------------------------------------------
void Average::fit(
    const descriptor_t& descriptor_,
    const pose_t& pose_,
    const point3d_t& point_) {

  // If its the first observation, initialize the sum descriptor
  // with the descriptor data and set the current descriptor
  // to that descriptor
  if (_numObservations == 0) {
    _descriptor = descriptor_;
    const std::vector<float>& descriptorData = descriptor_.get_data();
    _sumDescriptor = std::vector<float>(descriptorData);
    _numObservations++;
    return;
  }

  // Update the current descriptor by computing the average with the
  // new descriptor. Also save the sum of the descriptors element-wise
  std::vector<float> averageDescriptor(descriptor_.get_size(), 0.0f);
  _numObservations++;
  const float count = static_cast<float>(_numObservations);

  size_t idx = 0;
  const std::vector<float>& descriptorData = descriptor_.get_data();
  for (const float value : descriptorData) {
    // Update the sum of the descriptors element-wise
    _sumDescriptor[idx] += value;
    // Compute the average descriptor element-wise
    averageDescriptor[idx] = _sumDescriptor[idx] / count;
    idx++;
  }
  // Normalize the average descriptor to have unit L2 norm
  __normalize_l2(averageDescriptor);
  _descriptor = descriptor_t(averageDescriptor);
  return;
}

// ---------------------------------------------------------------------------
// Compute the weighted average descriptor from a set of descriptors
// based on the query pose and point. The weight is based on the angle
// between the view direction of the observation and the query vector.
// ---------------------------------------------------------------------------
void WeightedAverage::__compute_weighted_average(
    const point3d_t& point_,
    const pose_t& pose_) {

  const Eigen::Vector3d queryVector = __compute_query_vector(point_, pose_);
  const size_t descriptorCount = _descriptorHistory.size();
  std::vector<double> angles(descriptorCount, 0.0);

  double minimumAngle = std::numeric_limits<double>::infinity();
  for (size_t descriptorId = 0; descriptorId < descriptorCount;
       ++descriptorId) {
    const double dotProduct = std::clamp(
        _viewVectorHistory[descriptorId].dot(queryVector),
        -1.0,
        1.0);
    const double angle = std::acos(dotProduct);
    angles[descriptorId] = angle;
    minimumAngle = std::min(minimumAngle, angle);
  }

  std::vector<double> weights(descriptorCount, 0.0);
  double weightSum = 0.0;
  for (size_t descriptorId = 0; descriptorId < descriptorCount;
       ++descriptorId) {
    const double stabilizedAngle = angles[descriptorId] - minimumAngle;
    const double weight = std::exp(-_alpha * stabilizedAngle);
    weights[descriptorId] = weight;
    weightSum += weight;
  }

  if (weightSum <= EPSILON) {
    std::fill(weights.begin(), weights.end(), 1.0);
    weightSum = static_cast<double>(descriptorCount);
  }
  for (double& weight : weights) {
    weight /= weightSum;
  }

  const size_t descriptorSize = _descriptorHistory.front().get_size();
  std::vector<float> averageDescriptor(descriptorSize, 0.0f);
  for (size_t descriptorId = 0; descriptorId < descriptorCount;
       ++descriptorId) {
    const std::vector<float>& descriptorData =
        _descriptorHistory[descriptorId].get_data();
    for (size_t valueId = 0; valueId < descriptorSize; ++valueId) {
      averageDescriptor[valueId] += static_cast<float>(
          weights[descriptorId] * static_cast<double>(descriptorData[valueId]));
    }
  }
  // Normalize the average descriptor to have unit L2 norm
  __normalize_l2(averageDescriptor);
  _descriptor = descriptor_t(averageDescriptor);
}

}  // namespace utils
}  // namespace slam_mer
