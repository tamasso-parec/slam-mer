/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file frame.cpp
/// @brief Implements frame construction and feature extraction routines.
/// @details Defines `localization::Frame` runtime behavior.
/// @author Pedro Miraldo
/// @author Valter Piedade

// SLAM-MER includes
#include <frame.hpp>
#include <image_processing.hpp>
#include <types/timer.hpp>

namespace slam_mer {
namespace localization {

// Static variables initialization
namespace {
frame_id_t __nextId{0};
}  // namespace

// ---------------------------------------------------------------------------
// Construct frame and run feature detection
// ---------------------------------------------------------------------------
Frame::Frame(const data_type::SensorData::weak_ptr cameraData_) {
  using data_type::SensorData;
  using image_processing::feature_detector;
  using modes::FeatureType;
  using slam_mer::settings::sharedSettings;

#ifdef LOG_LOCALIZATION
  timer_t timer;
#endif

  // Setup some important attributes for this object:
  // 1 - Set the ID of the frame.
  // 2 - Set the camera data, which contains the image and some other relevant
  //     sensorial information for the frame.
  // 3 - Runs feature detection.
  // 4 - UNSETs all the detected points (will be assigned later).
  // 5 - Checks if the frame is created correctly.

  _id = frame_id_t(__nextId);
  ++__nextId;
  _minDepth = sharedSettings->frame_minimum_depth();
  _maxDepth = sharedSettings->frame_maximum_depth();

  // Lock the weak pointer to get a shared pointer
  const SensorData::shared_ptr locked = cameraData_.lock();
  if (!locked)
    __throw__("[FRAMES] Frames(.) Failed to lock cameraData_ weak pointer.");
  _cameraData = std::move(locked);

  if (sharedSettings->feature_type() == FeatureType::ALIKED) {
    feature_detector(
        _cameraData->get_color_image(),
        _allDetectedImagePoints,
        _allDetectedImageDescriptors);
  } else {
    feature_detector(
        _cameraData->get_grayscale_image(),
        _allDetectedImagePoints,
        _allDetectedImageDescriptors);
  }
  _allDetectedImagePointIds =
      vector_t<point_id_t>(_allDetectedImagePoints.size(), point_id_t::UNSET);
  _createdWithSuccess = __check_frame_success();

#ifdef LOG_LOCALIZATION
  log::message(
      "Time to run feature detection:  " + std::string(timer.toc()),
      log::TRACKING);
#endif
}

// ---------------------------------------------------------------------------
// Prepare frame data for keyframe upgrade with 3D points
// ---------------------------------------------------------------------------
void Frame::setup_upgrade_to_keyframe() {
  // Iterate though the detect points and save for each:
  // 0 - Check if the point is valid
  // 1 - 3d coordinates (with Color)
  // 2 - 2d keypoint
  // 3 - Descriptor
  // 4 - Id of the map point (can be UNSET)

  for (size_t iter = 0; iter < _allDetectedImagePoints.size(); iter++) {
    // get 3d point in camera frame
    const array2D_t& uv = _allDetectedImagePoints[iter];
    const int u = static_cast<int>(std::round(uv[0]));
    const int v = static_cast<int>(std::round(uv[1]));
    const point3d_t point3d = _cameraData->get_3d_point(u, v);

    // Filter wrong 3d points
    const array3D_t& xyz = point3d;
    // if monocular data, this means the point had low confidence
    if (xyz[0] == 0.0 && xyz[1] == 0.0 && xyz[2] == 0.0) continue;
    // Check for depth limits
    const double& depth = xyz[2];
    if (depth > _maxDepth || depth < _minDepth) continue;

    // Save synchronized data for each point
    // 3d coordinates and color
    _pointsCameraFrame.push_back(point3d);
    // 2d keypoint
    _pointsImage.push_back(_allDetectedImagePoints[iter]);
    // Descriptor
    _pointsImageDescriptors.push_back(_allDetectedImageDescriptors[iter]);
    // Id of the map point (can be UNSET is the point is not yet in the map)
    _pointsInMapIds.push_back(_allDetectedImagePointIds[iter]);
  }
  return;
}

// ---------------------------------------------------------------------------
// Apply scale factor to frame 3D points
// ---------------------------------------------------------------------------
void Frame::apply_scale_to_points3d(const double scale_) {
  // This function:
  // 1 - Apply a scale factor to the 3D points in the frame
  // 2 - Apply the scale to the entire point map saved in cameraData
  for (auto& point : _pointsCameraFrame)
    point.apply_scale(scale_);
#ifdef DEPTH_INFERENCE
  _cameraData->scale_pointmap(scale_);
#endif
}

// ---------------------------------------------------------------------------
// Compute point-neighbor structure for frame observations
// ---------------------------------------------------------------------------
void Frame::get_neighbors(
    const keyframe_id_t& keyframeId_,
    std::unordered_map<point_id_t, neighbor_t::vector>& neighbors_) const {

  using slam_mer::settings::sharedSettings;

  if (_pointsImage.size() == 0) {
    log::warning(
        "No neighbors computed for frame " + std::to_string(_id.key()),
        log::TRACKING);
    return;
  }

  // Query parameters
  const size_t& numberPoints = sharedSettings->neighbors_points_per_keyframe();
  const int& rangeSearch = sharedSettings->neighbors_search_radius();
  const double& distanceThreshold =
      sharedSettings->neighbors_distance_threshold();
  const double threshold = distanceThreshold * distanceThreshold;

#ifdef LOG_LOCALIZATION
  timer_t timer;
#endif

  // Get the image size
  const array2Size_t imageSize = _cameraData->get_image_size();
  const int imageWidth = static_cast<int>(imageSize[0]);
  const int imageHeight = static_cast<int>(imageSize[1]);

  // Lambda function to compute the squared distance between two 3D points
  auto ___distance = [](const array3D_t& a__, const array3D_t& b__) -> double {
    return (a__[0] - b__[0]) * (a__[0] - b__[0]) +
           (a__[1] - b__[1]) * (a__[1] - b__[1]) +
           (a__[2] - b__[2]) * (a__[2] - b__[2]);
  };

  // Iterate through all the points and get the neighbors
  for (size_t iter = 0; iter < numberPoints; iter++) {

    // Randomly select a point in the image. This is done by randomly
    // selecting a point in the image and then randomly selecting a
    // neighbor in a square of size rangeSearch x rangeSearch
    const int uDelta = rand() % rangeSearch;
    const int vDelta = rand() % rangeSearch;
    const size_t index = (size_t)rand() % _pointsImage.size();
    const array2D_t uv = _pointsImage[index].to_2d_array();
    const int u = static_cast<int>(std::round(uv[0])) + uDelta;
    const int v = static_cast<int>(std::round(uv[1])) + vDelta;

    // check if the neighbors are inside the image
    if (u < 0 || u >= imageWidth || v < 0 || v >= imageHeight) continue;

    // get the 3D point of the query point
    const point3d_t& queryPoint = _pointsCameraFrame[index];
    // get the corresponding id of the query point
    const point_id_t& queryId = _pointsInMapIds[index];
    // If the point is not in the map, we skip it
    if (queryId == point_id_t::UNSET) continue;
    // If the point does not exist in neighbors, we skip it
    if (!neighbors_.contains(queryId)) continue;
    // get the candidate 3D point of the neighbor
    const point3d_t& neighborPoint = _cameraData->get_3d_point(u, v);
    if (neighborPoint == point3d_t()) continue;

    // Compute the distance between the two points
    const array3D_t& queryArray = queryPoint.to_3d_array();
    const array3D_t& neighborArray = neighborPoint.to_3d_array();
    const double distanceSquared = ___distance(queryArray, neighborArray);

    // check if the point is at a distance smaller than the threshold.
    // If it is, we add it to the neighbors list, after getting the color
    // If this happends, then we should skip this point
    if (distanceSquared > threshold) continue;

    // If all went well, we add the point to the neighbors list
    // new neighbor id

    // 1st, we check how many neighbors the point has, if exists.
    // Otherwise, we assume it has zero neighbors.
    const size_t numberNeighbours =
        neighbors_.contains(queryId) ? neighbors_[queryId].size() : 0;

    // 2nd, we set the id of the new neighbor as the number of neighbors
    const point_id_t neighborId(numberNeighbours);

    // Compute the point as the relative position to the query point
    const point3d_t relativePoint = neighborPoint - queryPoint;

    // Create the new neighbor
    const neighbor_t newNeighbor(
        neighborId,
        keyframeId_,
        point3d_t::make_shared(relativePoint));

    // Push back the new neighbor
    neighbors_[queryId].push_back(newNeighbor);
  }

#ifdef LOG_LOCALIZATION
  log::message(
      "Time to compute the neighbors: " + std::string(timer.toc()),
      log::TRACKING);
#endif
  return;
}

}  // namespace localization
}  // namespace slam_mer
