/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file image_processing.hpp
/// @brief Image-processing utilities for feature extraction and matching.
/// @details Declares the ImageProcessing class and related routines for feature
/// detection, description, matching, and robustness filtering. These
/// functions are core building blocks for visual front-end stages.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef IMG_PROCESS_H
#define IMG_PROCESS_H

// SLAM-MER includes
#include <types/descriptor.hpp>
#include <types/image.hpp>
#include <types/match.hpp>
#include <types/mutex.hpp>
#include <types/point2d.hpp>

namespace slam_mer {
namespace image_processing {

/// @brief Extracts 2D keypoints and descriptors from an input image.
/// @param image_ Input image used for feature extraction.
/// @param keypoints_ Output vector with detected keypoint coordinates.
/// @param descriptors_ Output vector with descriptors aligned with
/// `keypoints_`.
/// @return None.
void feature_detector(
    const image_t& image_,
    point2d_t::vector& keypoints_,
    descriptor_t::vector& descriptors_);

/// @brief Matches descriptors from two images using reciprocal nearest
/// neighbors.
/// @param descriptors_ Descriptor set from the first image.
/// @param descriptorsTemplate_ Descriptor set from the second image.
/// @return Vector with only the mutually consistent matches.
[[nodiscard("Using the returned matches is likely the intended behavior")]]
match_t<index_t>::vector feature_matcher_knn(
    const descriptor_t::vector& descriptors_,
    const descriptor_t::vector& descriptorsTemplate_);

/// @brief Computes the ratio between convex-hull areas of two keypoint sets.
/// @param points2d_1_ First keypoint set.
/// @param points2d_2_ Second keypoint set.
/// @return Coverage ratio `(area(points2d_1_) / area(points2d_2_))`.
[[nodiscard("Using returned coverage ratio is likely the intended behavior")]]
double compute_coverage_ratio(
    const point2d_t::vector& points2d_1_,
    const point2d_t::vector& points2d_2_);

/// @brief Counts the number of points in each cell of a grid.
/// @param keypoints_ Input keypoints in pixel coordinates.
/// @param imageSize_ Image size `(width, height)` used for grid projection.
/// @param rows_ Number of grid rows.
/// @param cols_ Number of grid columns.
/// @return Matrix with per-cell point counts.
[[nodiscard("Using the returned matrix is likely the intended behavior")]]
Eigen::MatrixXi count_points_per_cell(
    const point2d_t::vector& keypoints_,
    const array2Size_t& imageSize_,
    const int rows_,
    const int cols_);

}  // namespace image_processing
}  // namespace slam_mer

#endif  // IMG_PROCESS_H
