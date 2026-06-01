/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file sensor_data.cpp
/// @brief Implements SensorData helper and reconstruction methods.
///
/// This file provides weak-pointer utility wrappers, depth-inference
/// integration, and per-pixel 3D reconstruction for RGB-D and monocular data.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

// SLAM-MER includes
#include <sensor_data.hpp>
#include <settings.hpp>

#ifdef DEPTH_INFERENCE
#include <depth_inference/depth_inference.hpp>
#endif

namespace slam_mer {
namespace data_type {

#ifdef DEPTH_INFERENCE
depth_inference::DepthInference::shared_ptr SensorData::_depthEstimator =
    nullptr;
#endif

// ---------------------------------------------------------------------------
// Get timestamp from weak SensorData pointer
// ---------------------------------------------------------------------------
output_t<time_stamp_t> SensorData::get_timestamp(const weak_ptr& ptr_) {
  if (auto shared = ptr_.lock())
    return shared->get_time_stamp();
  else
    return fail_t::LOST_PTR;
}

// ---------------------------------------------------------------------------
// Check calibration status from weak SensorData pointer
// ---------------------------------------------------------------------------
bool SensorData::calibrated(const weak_ptr& ptr_) {
  if (auto shared = ptr_.lock())
    return shared->calibrated();
  else
    return false;
}

// ---------------------------------------------------------------------------
// Get intrinsics from weak SensorData pointer
// ---------------------------------------------------------------------------
output_t<intrinsics_t> SensorData::get_intrinsics(const weak_ptr& ptr_) {
  if (auto shared = ptr_.lock()) {
    const intrinsics_t& intrinsics = shared->get_intrinsics();
    if (intrinsics.empty()) {
      log::warning(
          "SensorData::get_intrinsics: Intrinsics not set!!",
          log::DATALOADER);
      return fail_t::IS_EMPTY;
    } else {
      return intrinsics;
    }
  } else
    return fail_t::LOST_PTR;
}

// ---------------------------------------------------------------------------
// Set intrinsics through weak SensorData pointer
// ---------------------------------------------------------------------------
fail_t SensorData::set_intrinsics(
    const weak_ptr& ptr_,
    const intrinsics_t& intrinsics_) {
  if (auto shared = ptr_.lock()) {
    shared->set_intrinsics(intrinsics_);
    return fail_t::NO_FAIL;
  } else {
    return fail_t::LOST_PTR;
  }
}

// ---------------------------------------------------------------------------
// Get image size from weak SensorData pointer
// ---------------------------------------------------------------------------
output_t<array2Size_t> SensorData::get_image_size(const weak_ptr& ptr_) {
  if (auto shared = ptr_.lock())
    return shared->get_image_size();
  else
    return fail_t::LOST_PTR;
}

// ---------------------------------------------------------------------------
// Convert weak SensorData pointer to shared pointer
// ---------------------------------------------------------------------------
SensorData::shared_ptr SensorData::to_shared(const SensorData::weak_ptr& ptr_) {
  using std::println;

  shared_ptr shared = ptr_.lock();
  if (!shared)
    println(
        "[Sensor Data] {}Warning{}: Failed to convert SensorData weak to "
        "shared_ptr.",
        yellow,
        reset);
  return shared;
}

// ---------------------------------------------------------------------------
// Run depth inference through weak SensorData pointer
// ---------------------------------------------------------------------------
fail_t SensorData::run_depth_inference(
    const SensorData::weak_ptr& ptr_,
    vector_t<image_t>& images_) {
  if (auto shared = ptr_.lock()) {
    shared->run_depth_inference(images_);
    return fail_t::NO_FAIL;
  } else {
    return fail_t(
        fail_t::LOST_PTR,
        "Failed to lock SensorData weak_ptr pointer for depth inference");
  }
}

#ifdef DEPTH_INFERENCE

// ---------------------------------------------------------------------------
// Create depth estimator when depth inference is enabled
// ---------------------------------------------------------------------------
void SensorData::__create_depth_estimator() {
  depth_inference::DepthInference::Config config;
  config._method = depth_inference::MAST3R;
  config._cudaDeviceId =
      settings::sharedSettings->depth_inference_cuda_device_id();
  _depthEstimator = depth_inference::DepthInference::create(config);
}

#endif

// ---------------------------------------------------------------------------
// Run depth inference and store point/confidence maps
// ---------------------------------------------------------------------------
void SensorData::run_depth_inference(const vector_t<image_t>& images_) {

  // This function:
  // 1 - Checks if we are dealing with monocular data. Otherwise,
  //     log a warning, in case the client called this function by mistake.
  // 2 - Cheks if the depth inference modules was built as part of the
  //     projet (thirdparty).
  // 3 - If one tries to call this function, for monocular data, and the
  //     DEPTH_INFERENCE modules is not built, then throw an error.
  //     There is some code error.
  // 4 - If all goes well runs depth estimation and save the respective
  //     outputs.

  if (this->get_sensor_type() == SensorData::MONOCULAR) {
    if (images_.empty()) __throw__("No images provided for depth inference");

#ifdef DEPTH_INFERENCE
    if (_depthEstimator == nullptr) __create_depth_estimator();

    depth_inference::DepthInferenceOutput output;

    std::vector<depth_inference::Image> depthInferenceImages;
    depthInferenceImages.reserve(images_.size());
    for (const auto& img : images_)
      depthInferenceImages.push_back(img.convert_to_depth_inference_type());
    _depthEstimator->estimate_depth(depthInferenceImages, output);
    if (output.number_output() == 0)
      __throw__("No output from depth inference.");

    this->set_pointMap(output._pointmap[0]);
    this->set_pointMap_confidence(output._pointmapConfidence[0]);
#else
    __throw__(
        "Depth inference is not enabled. Please enable DEPTH_INFERENCE in "
        "CMake options.");
#endif
  } else {
    log::warning(
        "SensorDataTUM::run_depth_inference only defined for MONOCULAR",
        log::DATALOADER);
  }
}

// ---------------------------------------------------------------------------
// Reconstruct a 3D point from image coordinates
// ---------------------------------------------------------------------------
point3d_t SensorData::get_3d_point(int u_, int v_) {

  // Two cases can happen:
  // 1 - We can have an RGB-D sensor, and the depoth is basically obtaining
  //     by accessing the D channel and do the respective computations
  //     including color
  // 2 - If on the other hand we are working with monocular cameras, we
  //     we have s specific function. This requires depth inference. So,
  //     if the depth inference module is not compiled, it throws an error.

  const SensorType& type = this->get_sensor_type();
  if (type == SensorData::RGBD) {
    const intrinsics_t intrinsics = this->get_intrinsics();
    const image_t& depthImage = this->get_depth_image();
    const double depth = depthImage.ptr<float>(v_)[u_];
    const double z = depth / _depthScalingFactor;
    const auto [fx, fy, _, cx, cy] = intrinsics.get_parameters();
    const double x = (u_ - cx) * z / fx;
    const double y = (v_ - cy) * z / fy;

    const array3Uchar_t color = this->get_color_at(u_, v_);
    const double r = static_cast<double>(color[0]);
    const double g = static_cast<double>(color[1]);
    const double b = static_cast<double>(color[2]);
    return point3d_t(x, y, z, r, g, b);
  } else if (type == SensorData::MONOCULAR) {
#ifdef DEPTH_INFERENCE
    return this->get_3d_point_monocular(u_, v_);
#else
    __throw__(
        "Depth inference is not enabled. Please enable DEPTH_INFERENCE in "
        "CMake options.");
#endif
  } else {
    __throw__(
        "[slam_mer::dataloader::SensorDataEuroc::get_3d_point] \
        SensorData::SensorType set as UNDEFINED");
  }
  return point3d_t();
}

#ifdef DEPTH_INFERENCE

// ---------------------------------------------------------------------------
// Reconstruct monocular 3D point from image coordinates
// ---------------------------------------------------------------------------
point3d_t SensorData::get_3d_point_monocular(const int u_, const int v_) {
  using depth_inference::DepthInferenceOutput;
  using slam_mer::settings::sharedSettings;

  // Thus function:
  // 1 - Checks if there is a pointMap already created. If not, it creates
  //     it by running depth inference.
  // 2 - If there is a confidence map, it uses it to filter out low confidence
  //     points, by returning a point at (0,0,0).
  // 3 - If the confidence is good, it reconstructs the 3D point. If calibration
  //     is set, it computes X and Y from Z and intrinsics. Otherwise,
  //     it directly uses the X,Y,Z from the pointMap.
  // 4 - Finally, it gets the color for the point and returns the final 3D
  //     point.

  if (!_pointMap || _pointMap->empty()) {
    if (_depthEstimator == nullptr) __create_depth_estimator();

    DepthInferenceOutput output;
    depth_inference::Image depthInferenceImage =
        this->get_color_image().convert_to_depth_inference_type();
    _depthEstimator->estimate_depth({depthInferenceImage}, output);
    this->set_pointMap(output._pointmap[0]);
    this->set_pointMap_confidence(output._pointmapConfidence[0]);
  }

  if (!_pointMapConfidence || _pointMapConfidence->empty()) {
    log::warning(
        "No confidence map available for depth inference.",
        log::TRACKING);
  } else {
    const float depthConfidence = (*_pointMapConfidence)(v_, u_);
    if (depthConfidence < sharedSettings->depth_lowest_confidence())
      return point3d_t();  // return point at (0,0,0)
  }

  const std::array<float, 3> point = (*_pointMap)(v_, u_);
  double x, y, z;
  if (this->calibrated()) {
    const intrinsics_t intrinsics = this->get_intrinsics();
    const auto [fx, fy, _, cx, cy] = intrinsics.get_parameters();
    z = static_cast<double>(point[2]);
    x = (u_ - cx) * z / fx;
    y = (v_ - cy) * z / fy;
  } else {
    x = static_cast<double>(point[0]);
    y = static_cast<double>(point[1]);
    z = static_cast<double>(point[2]);
  }

  const array3Uchar_t color = this->get_color_at(u_, v_);
  const double r = static_cast<double>(color[0]);
  const double g = static_cast<double>(color[1]);
  const double b = static_cast<double>(color[2]);

  return point3d_t(x, y, z, r, g, b);
}

#endif

}  // namespace data_type
}  // namespace slam_mer
