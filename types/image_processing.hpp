/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file image_processing.hpp
/// @brief Configuration structs for image-feature extraction backends.
/// @details Collects runtime parameters for feature detectors and matchers such
/// as ALIKED and CudaSift. Keeping these options in typed structs
/// simplifies configuration loading and module interoperability.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_IMAGE_PROCESSING_HPP
#define TYPE_IMAGE_PROCESSING_HPP

#include <string>

namespace slam_mer {

/// @brief structure variable with all the important parameters for ALIKED.
struct AlikedParameters {
  /// You can use "aliked-n32", "aliked-n16", "aliked-n16rot", and "aliked-t16"
  std::string modelName = "aliked-n32";
  /// number of top keypoints (-1 for threshold-based selection, default: -1)
  int topK = 500;
  /// scores threshold (used when top_k is -1)
  float scoresTh = 0.2f;
  /// number of keypoints to keep (used when top_k is -1)
  int nLimit = 1000;
  /// CUDA device identifier used by ALIKED.
  int cudaDeviceId = 0;
};

/// @brief structure variable with all the important parameters for CudaSift.
struct CudaSiftParameters {
  /// Number of octaves in Gaussian pyramid
  int numOctaves = 3;
  /// Amount of initial Gaussian blurring in standard deviations
  double initBlur = 1.5;
  /// Threshold on difference of Gaussians for feature pruning
  double thresh = 1.5;
  /// Minimum acceptable scale to remove fine-scale features
  double minScale = 0.0;
  /// Whether to upscale image before extraction
  bool upScale = false;
};

}  // namespace slam_mer

#endif  // TYPE_IMAGE_PROCESSING_HPP
