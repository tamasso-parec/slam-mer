/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file rasterization_helper.cpp
/// @brief Implements map-side rasterization and overlap helper routines.
/// @details Provides IoU and 3D-box projection implementations.
/// @author Pedro Miraldo
/// @author Valter Piedade

// Our includes
#include <rasterization_helper.hpp>

// Standard includes

namespace slam_mer {
namespace map {
namespace helper {

// ---------------------------------------------------------------------------
// Compute intersection-over-union between two 2D boxes
// ---------------------------------------------------------------------------
double iou(
    const array4D_t& boxA_,
    const array4D_t& boxB_,
    const array2Size_t& imageSize_) {
  using std::max;
  using std::min;

  const double imageWidth = static_cast<double>(imageSize_[0]);
  const double imageHeight = static_cast<double>(imageSize_[1]);

  const double xAMin = boxA_[0];
  const double yAMin = boxA_[1];
  const double xAMax = boxA_[2];
  const double yAMax = boxA_[3];

  const double xBMin = boxB_[0];
  const double yBMin = boxB_[1];
  const double xBMax = boxB_[2];
  const double yBMax = boxB_[3];

  // Verify if the boxes are valid, i.e., they project into the image.
  // For simplicity, I am just chaging if a corner goes outside
  // the image.
  // If image size is not provided, skip out-of-image logic
  if (imageWidth > 0 && imageHeight > 0) {

    // Creates a lambda function to check if a box is out of the image
    auto ___is_out_of_image = [&](const array4D_t& box__) {
      return box__[2] < 0 || box__[3] < 0 || box__[0] > imageWidth ||
             box__[1] > imageHeight;
    };

    const bool outA = ___is_out_of_image(boxA_);
    const bool outB = ___is_out_of_image(boxB_);

    // Options.
    // If one box is out of the image, return 0.
    // If both boxes are out of the image, return 1.
    if (outA && !outB) return 0.0;
    if (!outA && outB) return 0.0;
    if (outA && outB) return 1.0;
  }

  // Intersection rectangle
  const double xInterMin = max(xAMin, xBMin);
  const double yInterMin = max(yAMin, yBMin);
  const double xInterMax = min(xAMax, xBMax);
  const double yInterMax = min(yAMax, yBMax);

  // Intersection area
  const double interWidth = max(0.0, xInterMax - xInterMin);
  const double interHeight = max(0.0, yInterMax - yInterMin);
  const double intersection = interWidth * interHeight;

  // Areas of each box
  const double areaA = max(0.0, xAMax - xAMin) * max(0.0, yAMax - yAMin);
  const double areaB = max(0.0, xBMax - xBMin) * max(0.0, yBMax - yBMin);

  // Union
  const double unionArea = areaA + areaB - intersection;
  if (unionArea <= 0.0) return 0.0;

  return intersection / unionArea;
}

// ---------------------------------------------------------------------------
// Project a 3D axis-aligned box into an image-space 2D box
// ---------------------------------------------------------------------------
std::tuple<bool, array4D_t> project(
    const array3D_t& box3dMin_,
    const array3D_t& box3dMax_,
    const pose_t worldToCameraTransformation_,
    const intrinsics_t& intrinsics_) {
  using std::max;
  using std::min;
  using std::numeric_limits;

  // Generate the 8 corners of the 3D box
  array3D_t::vector corners;  // No need to reserve, only 8 points.
  for (int i = 0; i < 8; ++i) {
    corners.push_back(
        {(i & 1) ? box3dMax_[0] : box3dMin_[0],
         (i & 2) ? box3dMax_[1] : box3dMin_[1],
         (i & 4) ? box3dMax_[2] : box3dMin_[2]});
  }

  // Initialize bbox values
  // If no change, it will output an invalid bbox.
  double xMin = numeric_limits<double>::max();
  double yMin = numeric_limits<double>::max();
  double xMax = numeric_limits<double>::lowest();
  double yMax = numeric_limits<double>::lowest();

  // Iterate over each corner, project it, and update the bbox
  // If points any of the corners of the box is behind the camera,
  // we simply ignore it. This may lead to incorrect bboxes, but it's better
  // than returning invalid projections.
  for (const array3D_t& pointWorldFrame : corners) {
    // Transform to camera coordinates
    const point3d_t pointCameraFrame =
        worldToCameraTransformation_ * pointWorldFrame;

    // The projection function returns a tuple (valid, pointImage)
    // where valid indicates if the point is in front of the camera
    // and within the image bounds.
    const auto& [valid, pointImage] = intrinsics_.project(pointCameraFrame);
    if (!valid) continue;

    xMin = min(xMin, pointImage.at(0));
    yMin = min(yMin, pointImage.at(1));
    xMax = max(xMax, pointImage.at(0));
    yMax = max(yMax, pointImage.at(1));
  }

  // If no valid points, return an invalid bbox
  if (xMin == numeric_limits<double>::max() ||
      yMin == numeric_limits<double>::max() ||
      xMax == numeric_limits<double>::lowest() ||
      yMax == numeric_limits<double>::lowest()) {
    return std::make_tuple(false, array4D_t{xMin, yMin, xMax, yMax});
  } else {
    return std::make_tuple(true, array4D_t{xMin, yMin, xMax, yMax});
  }
}

}  // namespace helper
}  // namespace map
}  // namespace slam_mer
