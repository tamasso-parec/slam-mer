/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file registration.hpp
/// @brief Point-cloud registration interfaces for rigid alignment.
/// @details Declares registration routines that estimate rotation, translation,
/// and optional scale from matched 3D points. Used by map alignment
/// and relocalization paths.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef REGISTRATION_HPP
#define REGISTRATION_HPP

// SLAM-MER includes
#include <types/point3d.hpp>
#include <types/pose.hpp>

namespace slam_mer {
namespace registration {

/// @brief Computes rigid registration between two matched 3D point sets.
/// @param sourcePoints_ Source points transformed by the estimated pose.
/// @param targetPoints_ Target points used as alignment reference.
/// @param estimate_ Output pose mapping `sourcePoints_` toward `targetPoints_`.
/// @param scale_ Output similarity scale estimated by the solver.
/// @param centroid_ Output centroids used during registration preprocessing.
/// @return `true` when the solver produces a valid estimate.
[[nodiscard(
    "Return value should be used to check if the estimation was \
    successful")]]
bool compute_estimate(
    const point3d_t::vector& sourcePoints_,
    const point3d_t::vector& targetPoints_,
    pose_t& estimate_,
    double& scale_,
    point3d_t::vector& centroid_);

/// @brief Computes rigid registration with default scale/centroid outputs.
/// @param sourcePoints_ Source points transformed by the estimated pose.
/// @param targetPoints_ Target points used as alignment reference.
/// @param estimate_ Output pose mapping `sourcePoints_` toward `targetPoints_`.
/// @return `true` when the solver produces a valid estimate.
[[nodiscard(
    "Return value should be used to check if the estimation was \
    successful")]]
inline bool compute_estimate(
    const point3d_t::vector& sourcePoints_,
    const point3d_t::vector& targetPoints_,
    pose_t& estimate_) {
  double scale = 1.0;
  point3d_t::vector centroid;
  return compute_estimate(
      sourcePoints_,
      targetPoints_,
      estimate_,
      scale,
      centroid);
}

/// @brief Computes the scalar `s` minimizing alignment error in a
/// similarity model.
/// @param sourcePoints_ Source points used in the scalar estimation.
/// @param targetPoints_ Target points used in the scalar estimation.
/// @return Best-fit scalar `s` for the provided correspondence sets.
[[nodiscard("Using the returned scale is likely the intended behavior")]]
double compute_best_scalar_s(
    const point3d_t::vector& sourcePoints_,
    const point3d_t::vector& targetPoints_);

}  // namespace registration
}  // namespace slam_mer

#endif  // REGISTRATION_HPP
