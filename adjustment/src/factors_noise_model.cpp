/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file factors_noise_model.cpp
/// @brief Implements GTSAM noise-model builders for adjustment factors.
/// @details Defines robust and non-robust noise model factory routines.
/// @author Pedro Miraldo
/// @author Valter Piedade

// Slam-mer includes
#include <factors_noise_model.hpp>
#include <settings.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>

namespace slam_mer {
namespace covisibility_graph {

// ---------------------------------------------------------------------------
// Get the projection noise model object
// ---------------------------------------------------------------------------
gtsam::noiseModel::Diagonal::shared_ptr get_projection_noise_model() {
  using gtsam::noiseModel::Isotropic;
  using slam_mer::settings::sharedSettings;

  // Get noise parameters from settings
  double projectionNoiseSigma = sharedSettings->projection_factor_sigma();
  return Isotropic::Sigma(2, projectionNoiseSigma);
}

// ---------------------------------------------------------------------------
// Get the robust projection noise model object
// ---------------------------------------------------------------------------
gtsam::noiseModel::Robust::shared_ptr get_robust_projection_noise_model(
    const gtsam::noiseModel::Diagonal::shared_ptr projectionNoise_) {
  using gtsam::noiseModel::Robust;
  using gtsam::noiseModel::mEstimator::GemanMcClure;
  using gtsam::noiseModel::mEstimator::Huber;
  using modes::RobustLossType;
  using slam_mer::settings::sharedSettings;

  const RobustLossType loss =
      sharedSettings->robust_loss_for_projection_factor();
  const double shape =
      sharedSettings->robust_loss_shape_for_projection_factor();

  if (loss == RobustLossType::HUBER) {
    const Huber::shared_ptr huber = Huber::Create(shape);
    return Robust::Create(huber, projectionNoise_);
  }
  if (loss == RobustLossType::GEMAN_MCCLURE) {
    const GemanMcClure::shared_ptr gm = GemanMcClure::Create(shape);
    return Robust::Create(gm, projectionNoise_);
  }
  __throw__("get_robust_projection_noise_model: Unsupported loss type.");
}

// ---------------------------------------------------------------------------
// Get the distance noise model object
// ---------------------------------------------------------------------------
gtsam::noiseModel::Diagonal::shared_ptr get_distance_noise_model() {
  using gtsam::noiseModel::Isotropic;
  using slam_mer::settings::sharedSettings;

  double distanceNoiseSigma = sharedSettings->distance_factor_sigma();
  return Isotropic::Sigma(3, distanceNoiseSigma);
}

// ---------------------------------------------------------------------------
// Get the robust distance noise model object
// ---------------------------------------------------------------------------
gtsam::noiseModel::Robust::shared_ptr get_robust_distance_noise_model(
    const gtsam::noiseModel::Diagonal::shared_ptr distanceNoise_) {
  using gtsam::noiseModel::Robust;
  using gtsam::noiseModel::mEstimator::GemanMcClure;
  using gtsam::noiseModel::mEstimator::Huber;
  using modes::RobustLossType;
  using slam_mer::settings::sharedSettings;

  const RobustLossType loss = sharedSettings->robust_loss_for_distance_factor();
  const double shape = sharedSettings->robust_loss_shape_for_distance_factor();

  if (loss == RobustLossType::HUBER) {
    const Huber::shared_ptr huber = Huber::Create(shape);
    return Robust::Create(huber, distanceNoise_);
  }
  if (loss == RobustLossType::GEMAN_MCCLURE) {
    const GemanMcClure::shared_ptr gm = GemanMcClure::Create(shape);
    return Robust::Create(gm, distanceNoise_);
  }

  __throw__("get_robust_distance_noise_model: Unsupported robust loss type.");
}

// ---------------------------------------------------------------------------
// Get the odometry noise model object
// ---------------------------------------------------------------------------
template <typename TransformationType>
gtsam::noiseModel::Diagonal::shared_ptr get_odometry_noise_model(
    const double weight_) {
  using gtsam::Vector6;
  using gtsam::Vector7;
  using gtsam::noiseModel::Diagonal;
  using slam_mer::settings::sharedSettings;

  const double noiseRotation = sharedSettings->odometry_factor_rotation_sigma();
  const double noiseTranslation =
      sharedSettings->odometry_factor_translation_sigma();
  const double noiseScale = sharedSettings->odometry_factor_scale_sigma();

  Diagonal::shared_ptr noise;
  if constexpr (std::is_same_v<TransformationType, gtsam::Pose3>) {
    noise = Diagonal::Sigmas(Vector6(
        noiseRotation,
        noiseRotation,
        noiseRotation,
        noiseTranslation,
        noiseTranslation,
        noiseTranslation));
  } else if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>) {
    noise = Diagonal::Sigmas(Vector7(
        noiseRotation,
        noiseRotation,
        noiseRotation,
        noiseTranslation,
        noiseTranslation,
        noiseTranslation,
        noiseScale));
  } else {
    __throw__("get_odometry_noise_model: Unsupported TransformationType.");
  }

  return Diagonal::Sigmas(noise->sigmas() * weight_);
}

// ---------------------------------------------------------------------------
// Get the robust odometry noise model object
// ---------------------------------------------------------------------------
gtsam::noiseModel::Robust::shared_ptr get_robust_odometry_noise_model(
    const gtsam::noiseModel::Diagonal::shared_ptr odometryNoise_) {
  using gtsam::noiseModel::Robust;
  using gtsam::noiseModel::mEstimator::GemanMcClure;
  using gtsam::noiseModel::mEstimator::Huber;
  using modes::RobustLossType;
  using slam_mer::settings::sharedSettings;

  const RobustLossType loss = sharedSettings->robust_loss_for_odometry_factor();
  const double shape = sharedSettings->robust_loss_shape_for_odometry_factor();

  if (loss == RobustLossType::HUBER) {
    const Huber::shared_ptr huber = Huber::Create(shape);
    return Robust::Create(huber, odometryNoise_);
  }

  if (loss == RobustLossType::GEMAN_MCCLURE) {
    const GemanMcClure::shared_ptr gm = GemanMcClure::Create(shape);
    return Robust::Create(gm, odometryNoise_);
  }

  __throw__("get_robust_odometry_noise_model: Unsupported robust loss type.");
}

// ---------------------------------------------------------------------------
// Get the prior pose noise model object
// ---------------------------------------------------------------------------
template <typename TransformationType>
gtsam::noiseModel::Diagonal::shared_ptr get_prior_pose_noise_model() {
  using gtsam::Vector6;
  using gtsam::Vector7;
  using gtsam::noiseModel::Diagonal;

  Diagonal::shared_ptr priorPoseNoise;
  if constexpr (std::is_same_v<TransformationType, gtsam::Pose3>) {
    priorPoseNoise = Diagonal::Sigmas(Vector6::Constant(0.00001));
  } else if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>) {
    priorPoseNoise = Diagonal::Sigmas(Vector7::Constant(0.00001));
  } else {
    __throw__("get_odometry_noise_model: Unsupported TransformationType.");
  }

  return priorPoseNoise;
}

#ifdef SLAM_MER_COMPILE_SIMILARITY3
template gtsam::noiseModel::Diagonal::shared_ptr
get_odometry_noise_model<gtsam::Similarity3>(const double);
template gtsam::noiseModel::Diagonal::shared_ptr
get_prior_pose_noise_model<gtsam::Similarity3>();
#endif
#ifdef SLAM_MER_COMPILE_POSE3
template gtsam::noiseModel::Diagonal::shared_ptr
get_odometry_noise_model<gtsam::Pose3>(const double);
template gtsam::noiseModel::Diagonal::shared_ptr
get_prior_pose_noise_model<gtsam::Pose3>();
#endif

}  // namespace covisibility_graph
}  // namespace slam_mer
