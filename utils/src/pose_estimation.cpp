/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file pose_estimation.cpp
/// @brief Pose-estimation implementation using geometric solvers.
/// @details Implements absolute pose estimation workflows, solver setup, and
/// runtime parameter handling for localization. Bridges feature
/// correspondences with robust estimation backends.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <image_processing.hpp>
#include <pose_estimation.hpp>
#include <settings.hpp>
#include <types/timer.hpp>

// Thirdparty includes
#include <PoseLib/robust.h>

namespace slam_mer {
namespace pose_estimation {

namespace {

/// Camera parameters in PoseLib format
std::shared_ptr<poselib::Camera> __cameraPoselib;
/// PoseLib solver settings for absolute pose estimation
std::shared_ptr<poselib::AbsolutePoseOptions> __absolutePoseOptionsPoselib;
/// Vector with previous focal length estimates
std::vector<double> __estimatedFocalLengths;
/// Focal length estimated (mean of several measurements). Its value is -1.0
/// until enough measurements are obtained.
inline constexpr double UNSET_FOCAL_LENGTH = -1.0;
double __focalLength = UNSET_FOCAL_LENGTH;
}  // namespace

/// Initializes configuration parameters for absolute pose estimation.
void __initialize_absolute_pose_options() {
  using poselib::AbsolutePoseOptions;
  using poselib::BundleOptions;
  using poselib::RansacOptions;
  using slam_mer::settings::sharedSettings;

  RansacOptions ransacOptions;
  ransacOptions.max_iterations =
      sharedSettings->pnp_ransac_maximum_iterations();
  ransacOptions.min_iterations =
      sharedSettings->pnp_ransac_minimum_iterations();
  ransacOptions.dyn_num_trials_mult =
      sharedSettings->pnp_ransac_dyn_num_trials_mult();
  ransacOptions.success_prob = sharedSettings->pnp_ransac_success_prob();
  ransacOptions.seed = 0;
  ransacOptions.progressive_sampling = false;
  ransacOptions.max_prosac_iterations = 1000;
  ransacOptions.score_initial_model = false;

  BundleOptions bundleOptions;
  bundleOptions.max_iterations =
      sharedSettings->pnp_bundle_maximum_iterations();
  bundleOptions.loss_scale = 1.0;
  bundleOptions.loss_type = BundleOptions::LossType::CAUCHY;
  bundleOptions.gradient_tol = 1e-12;
  bundleOptions.step_tol = 1e-8;
  bundleOptions.initial_lambda = 1e-3;
  bundleOptions.min_lambda = 1e-10;
  bundleOptions.max_lambda = 1e10;
  bundleOptions.verbose = false;
  bundleOptions.refine_focal_length = false;
  bundleOptions.refine_extra_params = false;
  bundleOptions.refine_principal_point = false;

  // Pose estimation related options (includes ransac and refinement options)
  __absolutePoseOptionsPoselib = std::make_shared<AbsolutePoseOptions>();
  __absolutePoseOptionsPoselib->ransac = ransacOptions;
  __absolutePoseOptionsPoselib->bundle = bundleOptions;
}

/// @brief Converts a vector of `point3d_t` into PoseLib 3D points.
/// @param points3d_ Input 3D points in SLAM-MER representation.
/// @param points3dPoselib_ Output vector receiving PoseLib point objects.
void __convert_points3d_to_poselib(
    const point3d_t::vector& points3d_,
    std::vector<poselib::Point3D>& points3dPoselib_) {
  using poselib::Point3D;
  points3dPoselib_.reserve(points3d_.size());
  for (const auto& point_ : points3d_) {
    const array3D_t xyz = point_;
    const Point3D pointPoselib(xyz[0], xyz[1], xyz[2]);
    points3dPoselib_.emplace_back(pointPoselib);
  }
}

/// @brief Converts a vector of `point2d_t` into PoseLib 2D points.
/// @param points2d_ Input 2D points in SLAM-MER representation.
/// @param points2dPoselib_ Output vector receiving PoseLib point objects.
void __convert_points2d_to_poselib(
    const point2d_t::vector& points2d_,
    std::vector<poselib::Point2D>& points2dPoselib_) {
  using poselib::Point2D;
  points2dPoselib_.reserve(points2d_.size());
  for (const auto& point_ : points2d_) {
    const array2D_t uv = point_;
    const Point2D pointPoselib(uv[0], uv[1]);
    points2dPoselib_.emplace_back(pointPoselib);
  }
}

/// @brief Converts PoseLib inlier flags into a boolean mask.
/// @param inliersPoselib_ Input inlier vector returned by PoseLib.
/// @param inliers_ Output boolean mask aligned with correspondences.
void __convert_inliers_poselib_to_bool(
    const std::vector<char>& inliersPoselib_,
    std::vector<bool>& inliers_) {
  inliers_.reserve(inliersPoselib_.size());
  for (size_t i = 0; i < inliersPoselib_.size(); ++i)
    inliers_.emplace_back(inliersPoselib_[i] ? 1 : 0);
}

/// @brief Converts a PoseLib camera pose into `pose_t`.
/// @param posePoselib_ Input pose in PoseLib format.
/// @param outPose_ Output pose in SLAM-MER format.
void __convert_poselib_to_pose(
    const poselib::CameraPose& posePoselib_,
    pose_t& outPose_) {
  outPose_ = pose_t(posePoselib_.R(), posePoselib_.t);
}

/// @brief Updates camera intrinsics with a new focal length/principal point.
/// @param cameraData_ Sensor-data handle whose intrinsics are updated.
/// @param newFocalLength_ New focal length applied to both x and y axes.
/// @param newPrincipalPoint_ New principal point `{cx, cy}`.
void __update_intrinsics(
    const data_type::SensorData::weak_ptr cameraData_,
    const double newFocalLength_,
    const array2D_t newPrincipalPoint_) {
  using data_type::SensorData;
  using std::println;

  intrinsics_t cameraParameters(
      newFocalLength_,
      newFocalLength_,
      newPrincipalPoint_[0],
      newPrincipalPoint_[1]);
  fail_t fail = SensorData::set_intrinsics(cameraData_, cameraParameters);
  if (fail) {
    println(
        "[Pose Estimation] {}Warning{}: Failed to update intrinsics in "
        "SensorData.",
        yellow,
        reset);
  }
  return;
}

/// @brief Accumulates focal estimates and updates the global focal length.
/// @param estimatedFocalLength_ New focal-length estimate from one solve.
/// Once enough estimates are collected, the global focal length is set
/// to the median estimate.
void __update_focal_length(const double estimatedFocalLength_) {
  using slam_mer::settings::sharedSettings;

  // This function:
  // 1 - saves the estimated focal length
  // 2 - once we have a certain number of estimates, computes the median value
  //     of all estimates and assumes that is the focal length

  const int maxEstimates = sharedSettings->focal_length_estimates();
  size_t estimatedSize = __estimatedFocalLengths.size();

  const size_t maxSize = static_cast<size_t>(maxEstimates);
  if (estimatedSize == maxSize) return;

  __estimatedFocalLengths.push_back(estimatedFocalLength_);
  estimatedSize = __estimatedFocalLengths.size();

  if (estimatedSize == maxSize) {
    std::vector<double> sortedEstimates = __estimatedFocalLengths;
    std::sort(sortedEstimates.begin(), sortedEstimates.end());
    size_t n = sortedEstimates.size();
    if (n % 2 == 0) {
      __focalLength =
          (sortedEstimates[n / 2 - 1] + sortedEstimates[n / 2]) / 2.0;
    } else {
      __focalLength = sortedEstimates[n / 2];
    }
  }
  return;
}

// ----------------------------------------------------------------------------
// computes absolute pose of a camera
// ----------------------------------------------------------------------------
bool compute_absolute_pose(
    const vector_t<point3d_t::weak_ptr> worldPoints_,
    const descriptor_t::vector worldPointsDescriptors_,
    const point2d_t::vector imagePoints_,
    const descriptor_t::vector imagePointsDescriptors_,
    const data_type::SensorData::weak_ptr cameraData_,
    pose_t& pose_,
    std::vector<bool>& inliers_,
    match_t<index_t>::vector& matches_,
    const bool knownFocalLength_,
    const log::Module logModule_) {
  using image_processing::feature_matcher_knn;
  using slam_mer::settings::sharedSettings;

#ifdef LOG_POSE_ESTIMATION
  timer_t timer;
#endif

  const int requiredMatches = sharedSettings->pose_minimum_matches();
  int numMatches = 0;
  point2d_t::vector matched2dPoints;
  point3d_t::vector matched3dPoints;
  bool failSizeCheck = false;
  bool failMatchSizeCheck = false;
  bool estimationSuccess = false;

  // This function:
  // 1 - Checks the input data for validity
  // 2 - Matches the 2D and 3D descriptors
  // 3 - Extracts the matched 2D and 3D points
  // 4 - Calls the pose estimation function (solve_pnp)

  if (worldPoints_.size() != worldPointsDescriptors_.size() ||
      imagePoints_.size() != imagePointsDescriptors_.size() ||
      worldPoints_.size() == 0 || imagePoints_.size() == 0 ||
      worldPointsDescriptors_.size() == 0 ||
      imagePointsDescriptors_.size() == 0) {
    failSizeCheck = true;
    goto DONE;
  }

  matches_ =
      feature_matcher_knn(imagePointsDescriptors_, worldPointsDescriptors_);
  numMatches = static_cast<int>(matches_.size());
  if (numMatches < requiredMatches) {
    failMatchSizeCheck = true;
    goto DONE;
  }

  for (const match_t<index_t>& match : matches_) {
    const point3d_t point(worldPoints_[(size_t)match.second_element()]);
    const point2d_t pixel(imagePoints_[(size_t)match.first_element()]);
    if (point.is_unset()) continue;
    matched2dPoints.push_back(pixel);
    matched3dPoints.push_back(point);
  }

  estimationSuccess = solve_pnp(
      matched3dPoints,
      matched2dPoints,
      cameraData_,
      pose_,
      inliers_,
      knownFocalLength_,
      logModule_);

DONE:
#ifdef LOG_POSE_ESTIMATION
  log::message(
      "compute_absolute_pose inputs: world points = " +
          std::to_string(worldPointsDescriptors_.size()) +
          "; image points = " + std::to_string(imagePointsDescriptors_.size()),
      logModule_);

  if (failSizeCheck)
    log::message(
        "compute_absolute_pose input error. Ensure \
      worldPoints.size() != worldPointsDescriptors.size() || \
      imagePoints.size() != imagePointsDescriptors.size() || \
      worldPoints.size() == 0 || imagePoints.size() == 0 || \
      worldPointsDescriptors.size() == 0 || \
      imagePointsDescriptors.size() == 0",
        logModule_);

  if (failMatchSizeCheck)
    log::message(
        "Not enough matches to run solve_pnp (got " +
            std::to_string(numMatches) + " when at least " +
            std::to_string(requiredMatches) + " are required!",
        logModule_);

  if (!failSizeCheck && !failMatchSizeCheck)
    log::message(
        "compute_absolute_pose: matches found = " +
            std::to_string(matches_.size()) +
            "; matching time = " + (std::string)timer.toc(),
        logModule_);
#endif

  if (failSizeCheck || failMatchSizeCheck)
    return false;
  else
    return estimationSuccess;
}

// ----------------------------------------------------------------------------
// solves absolute pose of a camera given 2D-3D correspondences
// ----------------------------------------------------------------------------
bool solve_pnp(
    const point3d_t::vector& worldPoints_,
    const point2d_t::vector& imagePoints_,
    const data_type::SensorData::weak_ptr cameraData_,
    pose_t& pose_,
    std::vector<bool>& inliers_,
    const bool knownFocalLength_,
    const log::Module logModule_) {
  using data_type::SensorData;
  using poselib::Camera;
  using poselib::estimate_absolute_pose;
  using poselib::RansacStats;
  using slam_mer::settings::sharedSettings;
  using std::println;

#ifdef LOG_POSE_ESTIMATION
  timer_t timer;
#endif

  // Poselib pose estimation is an end to end solution for pose estimation
  // that is straightforward to use. The only thing that we need to do is
  // to convert the data to the poselib format and vice versa.
  // 1 - convert 2D and 3D points to poselib format
  // 2 - create and initialize poseLib structures and options
  // 3 - different options for calibrated and uncalibrated case
  // 4 - call poselib absolute pose estimation function
  // 5 - convert estimated pose and inliers back to our format
  // 6 - update focal length and intrinsics if needed
  // 7 - compute inlier ratio and return success flag

  // 1 ------------------------------------------------------------------------
  std::vector<poselib::Point3D> worldPointsPoselib;
  std::vector<poselib::Point2D> imagePointsPoseLib;
  __convert_points3d_to_poselib(worldPoints_, worldPointsPoselib);
  __convert_points2d_to_poselib(imagePoints_, imagePointsPoseLib);

  // 2 ------------------------------------------------------------------------
  const output_t<array2Size_t> imageSizeOutput =
      SensorData::get_image_size(cameraData_);
  if (imageSizeOutput.fail()) {
    println(
        "[Pose Estimation] {}Warning{}: solve_pnp() failed to get image size "
        "from SensorData.",
        yellow,
        reset);
    return false;
  }

  const array2Size_t& size = imageSizeOutput;
  const int& cols = size[0];
  const int& rows = size[1];
  std::unique_ptr<poselib::Image> posePoselib;
  posePoselib = std::make_unique<poselib::Image>();
  if (__absolutePoseOptionsPoselib == nullptr)
    __initialize_absolute_pose_options();

  // 3 ------------------------------------------------------------------------
  if (!knownFocalLength_) {
    if (__focalLength == UNSET_FOCAL_LENGTH) {
      __absolutePoseOptionsPoselib->estimate_focal_length = true;
      __absolutePoseOptionsPoselib->bundle.refine_focal_length = true;
      posePoselib->camera = Camera("PINHOLE", {}, cols, rows);
    } else {
      __absolutePoseOptionsPoselib->estimate_focal_length = false;
      __absolutePoseOptionsPoselib->bundle.refine_focal_length = false;
      const double f = __focalLength;
      const double cx = cols / 2.0;
      const double cy = rows / 2.0;
      posePoselib->camera = Camera("PINHOLE", {f, f, cx, cy}, cols, rows);
    }
  } else {
    __absolutePoseOptionsPoselib->estimate_focal_length = false;
    __absolutePoseOptionsPoselib->bundle.refine_focal_length = false;
    const output_t<intrinsics_t> intrinsicsOutput =
        SensorData::get_intrinsics(cameraData_);
    if (intrinsicsOutput == fail_t::LOST_PTR ||
        intrinsicsOutput == fail_t::IS_EMPTY) {
      println(
          "[Pose Estimation] {}Warning{}: solve_pnp() failed to get intrinsics "
          "from SensorData.",
          yellow,
          reset);
      return false;
    } else if (intrinsicsOutput != fail_t::NO_FAIL) {
      __throw__("Unexpected error when getting intrinsics from SensorData: ");
    }

    const intrinsics_t& params = intrinsicsOutput;
    const auto [fx, fy, _, cx, cy] = params.get_parameters();
    std::vector<double> cameraParameters = {fx, fy, cx, cy};
    posePoselib->camera = Camera("PINHOLE", cameraParameters, cols, rows);
  }

  // 4 ------------------------------------------------------------------------
  std::vector<char> inliersPoselib;
  RansacStats statistics = estimate_absolute_pose(
      imagePointsPoseLib,
      worldPointsPoselib,
      *__absolutePoseOptionsPoselib,
      posePoselib.get(),
      &inliersPoselib);

  // 5 ------------------------------------------------------------------------
  __convert_poselib_to_pose(posePoselib->pose, pose_);
  __convert_inliers_poselib_to_bool(inliersPoselib, inliers_);

  // 6 ------------------------------------------------------------------------
  if (!knownFocalLength_) {
    __update_focal_length(posePoselib->camera.focal());
    if (__focalLength != UNSET_FOCAL_LENGTH) {
      const double cx = cols / 2.0;
      const double cy = rows / 2.0;
      __update_intrinsics(cameraData_, __focalLength, {cx, cy});
    }
  }

  // 7 ------------------------------------------------------------------------
  const double& inlierRatioThreshold =
      sharedSettings->pose_minimum_inlier_ratio();
  const size_t& minInliers =
      static_cast<size_t>(sharedSettings->pose_minimum_inliers());
  const bool estimationSuccess =
      statistics.inlier_ratio > inlierRatioThreshold &&
      statistics.num_inliers > minInliers;

#ifdef LOG_POSE_ESTIMATION
  const size_t& numberInliers = statistics.num_inliers;
  const double& inlierRatio = statistics.inlier_ratio;
  log::message(
      "solve_pnp: Inliers = " + std::to_string(numberInliers) +
          "; Inlier rate = " + std::to_string(inlierRatio) +
          "; Time = " + std::string(timer.toc()),
      logModule_);
#endif

  return estimationSuccess;
}

}  // namespace pose_estimation
}  // namespace slam_mer
