/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file rasterization_helper.hpp
/// @brief Geometry helpers for map visibility and projection operations.
/// @details Declares utility functions to compute 2D bounding-box IoU and to
/// project 3D axis-aligned boxes into image-space bounding boxes using camera
/// pose and intrinsics. These helpers are used by map-side culling logic.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef RASTERIZATION_HELPER_HPP
#define RASTERIZATION_HELPER_HPP

// Our includes
#include <types/camera.hpp>
#include <types/mutex.hpp>
#include <types/pose.hpp>

// Standard C++ libraries
#include <tuple>

namespace slam_mer {
namespace map {
namespace helper {

/// @brief Compute intersection-over-union (IoU) for two 2D bounding boxes.
/// @param boxA_ First box represented as `(xMin, yMin, xMax, yMax)`.
/// @param boxB_ Second box represented as `(xMin, yMin, xMax, yMax)`.
/// @param imageSize_ Optional image size `(width, height)` used for clipping.
/// @return IoU score in the closed interval `[0, 1]`.
[[nodiscard("Use returned IoU score for overlap and visibility filtering")]]
double iou(
    const array4D_t& boxA_,
    const array4D_t& boxB_,
    const array2Size_t& imageSize_ = array2Size_t{0, 0});

/// @brief Project a 3D axis-aligned bounding box to an image-space 2D box.
/// @param box3dMin_ Minimum world-space corner `(x, y, z)` of the 3D box.
/// @param box3dMax_ Maximum world-space corner `(x, y, z)` of the 3D box.
/// @param worldToCameraTransformation_ Pose that maps world coordinates to
/// camera coordinates.
/// @param intrinsics_ Camera intrinsics used for projection.
/// @return Tuple where:
/// first value is `true` when projection produced valid image points;
/// second value is image-space box `(xMin, yMin, xMax, yMax)`.
[[nodiscard("Use returned flag and box to validate and consume projection")]]
std::tuple<bool, array4D_t> project(
    const array3D_t& box3dMin_,
    const array3D_t& box3dMax_,
    const pose_t worldToCameraTransformation_,
    const intrinsics_t& intrinsics_);

}  // namespace helper
}  // namespace map
}  // namespace slam_mer

#endif  // RASTERIZATION_HELPER_HPP
