/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file factors_noise_model.hpp
/// @brief Noise-model factory declarations for adjustment factors.
/// @details Declares builders for projection, distance, odometry, and prior
/// robust/non-robust GTSAM noise models used by the covisibility optimizer.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef FACTORS_NOISE_MODEL_HPP
#define FACTORS_NOISE_MODEL_HPP

// Third party includes
#include <gtsam/linear/NoiseModel.h>

namespace slam_mer {
namespace covisibility_graph {

/// @brief Build the baseline projection-factor noise model.
/// @return Diagonal noise model used by projection factors.
[[nodiscard("Use returned noise model to configure projection factors")]]
gtsam::noiseModel::Diagonal::shared_ptr get_projection_noise_model();

/// @brief Build a robust projection noise model from a base diagonal model.
/// @param projectionNoise_ Base projection noise model.
/// @return Robust noise model wrapping the projection residual.
[[nodiscard("Use returned robust model for projection outlier handling")]]
gtsam::noiseModel::Robust::shared_ptr get_robust_projection_noise_model(
    const gtsam::noiseModel::Diagonal::shared_ptr projectionNoise_);

/// @brief Build the baseline point-distance factor noise model.
/// @return Diagonal noise model used by point-distance factors.
[[nodiscard("Use returned noise model to configure point-distance factors")]]
gtsam::noiseModel::Diagonal::shared_ptr get_distance_noise_model();

/// @brief Build a robust point-distance noise model from a base diagonal model.
/// @param distanceNoise_ Base point-distance noise model.
/// @return Robust noise model wrapping the point-distance residual.
[[nodiscard("Use returned robust model for distance outlier handling")]]
gtsam::noiseModel::Robust::shared_ptr get_robust_distance_noise_model(
    const gtsam::noiseModel::Diagonal::shared_ptr distanceNoise_);

/// @brief Build the baseline odometry-factor noise model.
/// @param weight_ Scaling weight applied to odometry uncertainties.
/// @return Diagonal noise model used by odometry factors.
template <typename TransformationType>
[[nodiscard("Use returned noise model to configure odometry factors")]]
gtsam::noiseModel::Diagonal::shared_ptr get_odometry_noise_model(
    const double weight_ = 1.0);

/// @brief Build a robust odometry noise model from a base diagonal model.
/// @param odometryNoise_ Base odometry noise model.
/// @return Robust noise model wrapping the odometry residual.
[[nodiscard("Use returned robust model for odometry outlier handling")]]
gtsam::noiseModel::Robust::shared_ptr get_robust_odometry_noise_model(
    const gtsam::noiseModel::Diagonal::shared_ptr odometryNoise_);

/// @brief Build the prior-pose noise model for graph initialization.
/// @return Diagonal noise model used by prior pose factors.
template <typename TransformationType>
[[nodiscard("Use returned noise model when adding prior pose constraints")]]
gtsam::noiseModel::Diagonal::shared_ptr get_prior_pose_noise_model();

}  // namespace covisibility_graph
}  // namespace slam_mer

#endif  // FACTORS_NOISE_MODEL_HPP
