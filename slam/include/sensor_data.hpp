/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file sensor_data.hpp
/// @brief Declares the SensorData container used by the SLAM pipeline.
/// @details Encapsulates camera observations together with calibration data,
/// distortion coefficients, and timing metadata used by tracking/mapping.
/// Supports monocular and RGB-D workflows, including optional depth inference
/// outputs when that module is enabled. Also provides weak-pointer helper
/// utilities for safer cross-module access in asynchronous code paths.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

// Standard C++ libraries
// #include <random>
#include <sys/types.h>
#include <cstddef>
#include <memory>

// SLAM-MER includes
#include <log.hpp>
#include <types/camera.hpp>
#include <types/id.hpp>
#include <types/image.hpp>
#include <types/mutex.hpp>
#include <types/point3d.hpp>
#include <types/time_stamp.hpp>
#include <types/vector.hpp>

#ifdef DEPTH_INFERENCE
#include <depth_inference/depth_inference.hpp>
#endif

namespace slam_mer {
namespace data_type {

/// @class SensorData
/// @brief Runtime container for one sensor sample and its calibration context.
/// @details Stores image/depth payloads plus intrinsics, distortion, and
/// timestamp metadata used by SLAM modules. Provides read/write helpers and
/// weak-pointer wrappers for safe use across asynchronous ownership boundaries.
class SensorData {
public:
  /// @brief Smart pointer alias for unique ownership.
  using unique_ptr = std::unique_ptr<SensorData>;
  /// @brief Smart pointer alias for shared ownership.
  using shared_ptr = std::shared_ptr<SensorData>;
  /// @brief Smart pointer alias for non-owning weak references.
  using weak_ptr = std::weak_ptr<SensorData>;

  /// @brief Returns the timestamp from a weak SensorData pointer.
  /// @param ptr_ Weak pointer to the sensor data.
  /// @return Stored timestamp, or `fail_t::LOST_PTR` if `ptr_` is expired.
  [[nodiscard("Use get_timestamp() to retrieve the timestamp")]]
  static output_t<time_stamp_t> get_timestamp(const weak_ptr& ptr_);

  /// @brief Checks whether the sensor data has camera intrinsics.
  /// @param ptr_ Weak pointer to the sensor data.
  /// @return `true` if intrinsics are set, `false` otherwise.
  [[nodiscard("Use has_intrinsics() to check for intrinsics")]]
  static bool calibrated(const weak_ptr& ptr_);

  /// @brief Returns camera intrinsics from a weak SensorData pointer.
  /// @param ptr_ Weak pointer to the sensor data.
  /// @return Camera intrinsics, or `fail_t::LOST_PTR` if `ptr_` is expired.
  [[nodiscard("Use get_intrinsics() to retrieve camera intrinsics")]]
  static output_t<intrinsics_t> get_intrinsics(const weak_ptr& ptr_);

  /// @brief Updates camera intrinsics through a weak SensorData pointer.
  /// @param ptr_ Weak pointer to the sensor data.
  /// @param intrinsics_ New intrinsics matrix to store.
  /// @return `fail_t::NO_FAIL` on success, `fail_t::LOST_PTR` otherwise.
  [[nodiscard("WARNING: Not Checking if successfully set intrinsics")]]
  static fail_t set_intrinsics(
      const weak_ptr& ptr_,
      const intrinsics_t& intrinsics_);

  /// @brief Returns image dimensions from a weak SensorData pointer.
  /// @param ptr_ Weak pointer to the sensor data.
  /// @return Image size as `{width, height}`, or `fail_t::LOST_PTR`.
  [[nodiscard("Use get_image_size() to retrieve image dimensions")]]
  static output_t<array2Size_t> get_image_size(const weak_ptr& ptr_);

  /// @brief Converts a weak SensorData pointer to a shared pointer.
  /// @param ptr_ Weak pointer to convert.
  /// @return Locked shared pointer, or `nullptr` if `ptr_` is expired.
  [[nodiscard("Use to_shared() to convert weak pointer to shared pointer")]]
  static shared_ptr to_shared(const weak_ptr& ptr_);

  /// @brief Runs depth inference through a weak SensorData pointer.
  /// @param ptr_ Weak pointer to the sensor data.
  /// @param images_ Input images consumed by the depth estimator.
  /// @return `fail_t::NO_FAIL` on success, `fail_t::LOST_PTR` otherwise.
  [[nodiscard("WARNING: Not Checking if successfully ran depth inference")]]
  static fail_t run_depth_inference(
      const weak_ptr& ptr_,
      vector_t<image_t>& images_);

#ifdef DEPTH_INFERENCE
  /// @brief Shared depth estimator instance used by `run_depth_inference`.
  static depth_inference::DepthInference::shared_ptr _depthEstimator;
#endif

  /// @brief Supported sensor modalities.
  enum SensorType {
    /// @brief Sensor type is unset.
    UNDEFINED,
    /// @brief Monocular camera data.
    MONOCULAR,
    /// @brief RGB-D camera data.
    RGBD
  };

  /// @brief Constructs an empty SensorData instance.
  SensorData() {}

  /// @brief Copy-construct sensor payloads and metadata.
  /// @param other_ Source sensor sample to copy.
  SensorData(const SensorData& other_)
      : _sensorType(other_._sensorType),
        _imageGrayscale(other_._imageGrayscale.clone()),
        _imageColor(other_._imageColor.clone()),
        _depthImage(other_._depthImage.clone()),
        _intrinsics(other_._intrinsics.clone()),
        _distortionCoefficients(other_._distortionCoefficients.clone()),
        _timeStamp(other_._timeStamp) {}

  /// @brief Virtual destructor.
  virtual ~SensorData() = default;

  /// @brief Return the configured sensor modality.
  /// @return Current `SensorType`.
  [[nodiscard("Use get_sensor_type() to retrieve the sensor modality")]]
  inline SensorType get_sensor_type() const {
    return _sensorType;
  }

  /// @brief Check whether a grayscale image is available.
  /// @return `true` when `_imageGrayscale` is not empty.
  [[nodiscard("Use has_gray_scale_image() to check for grayscale image")]]
  inline bool has_gray_scale_image() const {
    return !_imageGrayscale.empty();
  }

  /// @brief Access the stored grayscale image.
  /// @return Const reference to grayscale image payload.
  [[nodiscard("Use get_grayscale_image() to retrieve the grayscale image")]]
  inline const image_t& get_grayscale_image() {
    return _imageGrayscale;
  }

  /// @brief Check whether a color image is available.
  /// @return `true` when `_imageColor` is not empty.
  [[nodiscard("Use has_color_image() to check for color image")]]
  inline bool has_color_image() const {
    return !_imageColor.empty();
  }

  /// @brief Access the stored color image.
  /// @return Const reference to color image payload.
  [[nodiscard("Use get_color_image() to retrieve the color image")]]
  inline const image_t& get_color_image() {
    return _imageColor;
  }

  /// @brief Returns RGB color at integer pixel coordinates.
  /// @param u_ Horizontal pixel coordinate.
  /// @param v_ Vertical pixel coordinate.
  /// @return RGB color packed as `array3Uchar_t`.
  [[nodiscard("Use get_color_at() to retrieve color at pixel coordinates")]]
  inline array3Uchar_t get_color_at(const int u_, const int v_) const {
    const uint8_t* color = _imageColor.pixel(v_, u_);
    return array3Uchar_t({color[0], color[1], color[2]});
  }

  /// @brief Check whether a depth image is available.
  /// @return `true` when `_depthImage` is not empty.
  [[nodiscard("Use has_depth_image() to check for depth image")]]
  inline bool has_depth_image() const {
    return !_depthImage.empty();
  }

  /// @brief Access the stored depth image.
  /// @return Mutable reference to depth image payload.
  [[nodiscard("Use get_depth_image() to retrieve the depth image")]]
  inline image_t& get_depth_image() {
    return _depthImage;
  }

  /// @brief Check whether camera intrinsics are available.
  /// @return `true` when `_intrinsics` has been initialized.
  [[nodiscard("Use calibrated() to check for camera intrinsics")]]
  inline bool calibrated() const {
    return !_intrinsics.empty();
  }

  /// @brief Access the stored camera intrinsics.
  /// @return Const reference to intrinsic calibration matrix.
  [[nodiscard("Use get_intrinsics() to retrieve the camera intrinsics")]]
  inline const intrinsics_t& get_intrinsics() const {
    return _intrinsics;
  }

  /// @brief Check whether distortion coefficients are available.
  /// @return `true` when distortion coefficients are present.
  [[nodiscard("Use has_image_distortion() to check for distortion")]]
  inline bool has_image_distortion() const {
    return !this->_distortionCoefficients.empty();
  }

  /// @brief Access the stored distortion coefficients.
  /// @return Distortion coefficient vector.
  [[nodiscard("Use get_image_distortion() to retrieve distortion coeffs")]]
  inline distortion_t get_image_distortion() const {
    return this->_distortionCoefficients;
  }

  /// @brief Check whether a non-zero timestamp is available.
  /// @return `true` when timestamp is different from the default zero value.
  [[nodiscard("Use has_timeStamp() to check for non-zero timestamp")]]
  inline bool has_timeStamp() const {
    return _timeStamp != time_stamp_t(0.0);
  }

  /// @brief Access the stored timestamp.
  /// @return Timestamp associated with this sensor sample.
  [[nodiscard("Use get_time_stamp() to retrieve the timestamp")]]
  inline time_stamp_t get_time_stamp() const {
    return _timeStamp;
  }

  /// @brief Check whether stored payloads are consistent with sensor type.
  /// @return `true` when required image data is available for the configured
  /// sensor modality.
  [[nodiscard("Use is_valid() to check for sensor data consistency")]]
  inline bool is_valid() {
    // Monocular only requires rgb image
    if (_sensorType == SensorType::MONOCULAR) return this->has_color_image();
    // RGBD requires rgb image and depth image
    else if (_sensorType == SensorType::RGBD)
      return this->has_color_image() && this->has_depth_image();
    else
      log::warning("SensorType set to UNDEFINED in SensorData::is_valid()");
    return false;
  }

  /// @brief Return image size as `{width, height}`.
  /// @return Dimensions inferred from color image when present, otherwise
  /// grayscale image, or `{0,0}` if no image is available.
  [[nodiscard("Use get_image_size() to retrieve image dimensions")]]
  inline array2Size_t get_image_size() const {
    if (this->has_color_image()) {
      const size_t width = _imageColor.cols();
      const size_t height = _imageColor.rows();
      return array2Size_t{width, height};
    } else if (this->has_gray_scale_image()) {
      const size_t width = _imageGrayscale.cols();
      const size_t height = _imageGrayscale.rows();
      return array2Size_t{width, height};
    } else {
      log::warning(
          "SensorData::get_image_size: No image available!!",
          log::DATALOADER);
      return array2Size_t{0, 0};
    }
  }

  /// @brief Sets camera intrinsics.
  /// @param intrinsics_ Intrinsics matrix to store.
  inline void set_intrinsics(const intrinsics_t& intrinsics_) {
    _intrinsics = intrinsics_.clone();
  }

  /// @brief Returns a 3D point for the provided image pixel.
  /// @param u_ Horizontal pixel coordinate.
  /// @param v_ Vertical pixel coordinate.
  /// @return Reconstructed 3D point with color.
  [[nodiscard("Use get_3d_point() to retrieve the 3D point")]]
  virtual point3d_t get_3d_point(const int u_, const int v_);

#ifdef DEPTH_INFERENCE

  /// @brief Applies a scale factor to the entire point map.
  /// @param scale_ Scale factor.
  inline void scale_pointmap(const double scale_) {
    if (_pointMap) _pointMap->scale(scale_);
  }
#endif

  /// @brief Run depth inference and store estimated point/depth outputs.
  /// @param images_ Input image batch consumed by the depth estimator.
  void run_depth_inference(const vector_t<image_t>& images_);

protected:
  /// @brief Sets the sensor type.
  /// @param sensorType_ Sensor modality.
  inline void set_sensor_type(const SensorType sensorType_) {
    _sensorType = sensorType_;
  }

  /// @brief Stores a grayscale image.
  /// @param imageGrayscale_ Grayscale image.
  inline void set_gray_scale_image(const image_t& imageGrayscale_) {
    _imageGrayscale = imageGrayscale_.clone();
  }

  /// @brief Stores a color image.
  /// @param imageColor_ Color image.
  inline void set_color_image(const image_t& imageColor_) {
    _imageColor = imageColor_.clone();
  }

  /// @brief Stores a depth image.
  /// @param depthImage_ Depth image.
  inline void set_depth_image(const image_t& depthImage_) {
    _depthImage = depthImage_.clone();
  }

  /// @brief Stores distortion coefficients.
  /// @param distortionCoefficients_ Distortion coefficient vector.
  inline void set_image_distortion(
      const distortion_t& distortionCoefficients_) {
    this->_distortionCoefficients = distortionCoefficients_.clone();
  }

  /// @brief Sets the timestamp.
  /// @param timeStamp_ Timestamp value.
  inline void set_timeStamp(const time_stamp_t timeStamp_) {
    _timeStamp = timeStamp_;
  }

  /// @brief Sets the depth scaling factor used for RGB-D depth images.
  /// @param factor_ Depth scaling factor.
  inline void set_depth_scaling_factor(const double factor_) {
    _depthScalingFactor = factor_;
  }

private:
  /// @brief Sensor modality.
  SensorType _sensorType = SensorType::UNDEFINED;
  /// @brief Grayscale image.
  image_t _imageGrayscale;
  /// @brief Color image.
  image_t _imageColor;
  /// @brief Depth image.
  image_t _depthImage;
  /// @brief Camera intrinsics.
  intrinsics_t _intrinsics = intrinsics_t();
  /// @brief Camera distortion coefficients.
  distortion_t _distortionCoefficients;
  /// @brief Timestamp associated with this sensor sample.
  time_stamp_t _timeStamp = time_stamp_t(0.0);
  /// @brief Depth scaling factor for 16-bit depth images.
  double _depthScalingFactor = 5000.0;

#ifdef DEPTH_INFERENCE
  /// @brief Returns a 3D point for monocular data.
  /// @param u_ Horizontal pixel coordinate.
  /// @param v_ Vertical pixel coordinate.
  /// @return Reconstructed 3D point with color.
  [[nodiscard(
      "Use get_3d_point_monocular() to retrieve the 3D point for \
      monocular data")]]
  point3d_t get_3d_point_monocular(const int u_, const int v_);

  /// @brief Stores the point map estimated by depth inference.
  /// @param pointMap_ Point map to store.
  inline void set_pointMap(const depth_inference::PointMap& pointMap_) {
    using depth_inference::PointMap;
    _pointMap = PointMap::make_unique(pointMap_);
  }
  /// @brief Stores the confidence map estimated by depth inference.
  /// @param confidence_ Confidence map to store.
  inline void set_pointMap_confidence(
      const depth_inference::PointConfidenceMap& confidence_) {
    using depth_inference::PointConfidenceMap;
    _pointMapConfidence = PointConfidenceMap::make_unique(confidence_);
  }

  /// @brief Creates the depth estimator singleton when needed.
  void __create_depth_estimator();

  /// @brief Point map `[X, Y, Z]` in camera coordinates.
  depth_inference::PointMap::unique_ptr _pointMap = nullptr;
  /// @brief Per-point confidence values for `pointMap`.
  depth_inference::PointConfidenceMap::unique_ptr _pointMapConfidence = nullptr;
#endif

};  // class SensorData

}  // namespace data_type

/// @brief Validates a shared sensor output and throws on retrieval failure.
/// @param sensorOut_ Output wrapper holding a shared sensor pointer.
/// @param keyframeId_ Keyframe identifier used in the exception message.
inline void __sanity_retrieve(
    const output_t<data_type::SensorData::shared_ptr>& sensorOut_,
    const keyframe_id_t& keyframeId_) {
  if (sensorOut_ != fail_t::NO_FAIL)
    __throw__(
        "Unexpected: Empty pointer for sensor data " +
        std::string(keyframeId_));
}

/// @brief Validates a weak sensor output and throws on retrieval failure.
/// @param sensorOut_ Output wrapper holding a weak sensor pointer.
/// @param keyframeId_ Keyframe identifier used in the exception message.
inline void __sanity_retrieve(
    const output_t<data_type::SensorData::weak_ptr>& sensorOut_,
    const keyframe_id_t& keyframeId_) {
  if (sensorOut_ != fail_t::NO_FAIL)
    __throw__(
        "Unexpected: Empty pointer for sensor data " +
        std::string(keyframeId_));
}

/// @brief Validates a shared sensor output and throws on retrieval failure.
/// @param sensorOut_ Output wrapper holding a shared sensor pointer.
inline void __sanity_retrieve(
    const output_t<data_type::SensorData::shared_ptr>& sensorOut_) {
  if (sensorOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for sensor data");
}

/// @brief Validates a weak sensor output and throws on retrieval failure.
/// @param sensorOut_ Output wrapper holding a weak sensor pointer.
inline void __sanity_retrieve(
    const output_t<data_type::SensorData::weak_ptr>& sensorOut_) {
  if (sensorOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Empty pointer for sensor data");
}

/// @brief Logs a warning when shared sensor retrieval fails.
/// @param sensorOut_ Output wrapper holding a shared sensor pointer.
/// @param keyframeId_ Keyframe identifier used in the warning message.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use __fail_retrieve() to check for retrieval failure")]]
inline bool __fail_retrieve(
    const output_t<data_type::SensorData::shared_ptr>& sensorOut_,
    const keyframe_id_t& keyframeId_) {
  if (sensorOut_ != fail_t::NO_FAIL) {
    std::println(
        "{}[WARNING]{} Lost pointer for sensor data {}",
        yellow,
        reset,
        std::string(keyframeId_));
    return true;
  }
  return false;
}

/// @brief Logs a warning when weak sensor retrieval fails.
/// @param sensorOut_ Output wrapper holding a weak sensor pointer.
/// @param keyframeId_ Keyframe identifier used in the warning message.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use __fail_retrieve() to check for retrieval failure")]]
inline bool __fail_retrieve(
    const output_t<data_type::SensorData::weak_ptr>& sensorOut_,
    const keyframe_id_t& keyframeId_) {
  if (sensorOut_ != fail_t::NO_FAIL) {
    std::println(
        "{}[WARNING]{} Empty pointer for sensor data {}",
        yellow,
        reset,
        std::string(keyframeId_));
    return true;
  }
  return false;
}

/// @brief Logs a warning when weak sensor retrieval fails.
/// @param sensorOut_ Output wrapper holding a weak sensor pointer.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use __fail_retrieve() to check for retrieval failure")]]
inline bool __fail_retrieve(
    const output_t<data_type::SensorData::weak_ptr>& sensorOut_) {
  if (sensorOut_ != fail_t::NO_FAIL) {
    std::println("{}[WARNING]{} Empty pointer for sensor data", yellow, reset);
    return true;
  }
  return false;
}

}  // namespace slam_mer

#endif  // SENSOR_DATA_H
