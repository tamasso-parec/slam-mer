/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file pose_estimation.hpp
/// @brief Absolute pose-estimation interfaces used by localization routines.
/// @details Declares APIs for estimating camera pose from 2D-3D correspondences
/// and for configuring estimation options. These functions connect
/// feature matching to geometric solvers.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef POSE_ESTIMATION_HPP
#define POSE_ESTIMATION_HPP

// SLAM-MER includes
#include <sensor_data.hpp>
#include <types/descriptor.hpp>
#include <types/match.hpp>
#include <types/point2d.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>

// Standard library includes
#include <vector>

namespace slam_mer {
namespace pose_estimation {

/// @brief Estimates camera absolute pose from unmatched 2D and 3D features.
/// @param worldPoints_ Candidate 3D map points in world coordinates.
/// @param worldPointsDescriptors_ Descriptors aligned with `worldPoints_`.
/// @param imagePoints_ 2D keypoints detected in the current frame.
/// @param imagePointsDescriptors_ Descriptors aligned with `imagePoints_`.
/// @param cameraData_ Sensor data handle with image/intrinsics metadata.
/// @param pose_ Output estimated camera pose.
/// @param inliers_ Output inlier mask from the final PnP solve.
/// @param matches_ Output descriptor matches between image and world features.
/// @param unknownFocalLength_ Enables focal-length estimation when `true`.
/// @param logModule_ Module label used by optional logging output.
/// @return `true` when pose estimation succeeds, otherwise `false`.
bool compute_absolute_pose(
    const vector_t<point3d_t::weak_ptr> worldPoints_,
    const descriptor_t::vector worldPointsDescriptors_,
    const point2d_t::vector imagePoints_,
    const descriptor_t::vector imagePointsDescriptors_,
    const data_type::SensorData::weak_ptr cameraData_,
    pose_t& pose_,
    std::vector<bool>& inliers_,
    match_t<index_t>::vector& matches_,
    const bool unknownFocalLength_ = false,
    const log::Module logModule_ = log::EMPTY);

/// @brief Solves PnP from already matched 2D-3D correspondences.
/// @param worldPoints_ Matched 3D points in world coordinates.
/// @param imagePoints_ Matched 2D points in image coordinates.
/// @param cameraData_ Sensor data handle with image/intrinsics metadata.
/// @param pose_ Output estimated camera pose.
/// @param inliers_ Output inlier mask aligned with input correspondences.
/// @param unknownFocalLength_ Enables focal-length estimation when `true`.
/// @param logModule_ Module label used by optional logging output.
/// @return `true` when PnP satisfies inlier thresholds, otherwise `false`.
bool solve_pnp(
    const point3d_t::vector& worldPoints_,
    const point2d_t::vector& imagePoints_,
    const data_type::SensorData::weak_ptr cameraData_,
    pose_t& pose_,
    std::vector<bool>& inliers_,
    const bool unknownFocalLength_ = false,
    const log::Module logModule_ = log::EMPTY);

}  // namespace pose_estimation
}  // namespace slam_mer

#endif  // POSE_ESTIMATION_HPP
