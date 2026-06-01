/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file modes.hpp
/// @brief Enumerations for system operating modes and model choices.
/// @details Declares strongly named enum values used to switch pipeline
/// behavior, feature processing strategies, and matching backends.
/// These enums provide a shared contract across configuration and
/// runtime logic.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_MODES_HPP
#define TYPE_MODES_HPP

namespace slam_mer::modes {

/// System mode
enum SystemMode {
  /// Run the full SLAM system (localization, adjustment and loop closure)
  SLAM,
  /// Run only the localization and adjustment modules, without loop closure
  SLAM_WITHOUT_LOOP,
  /// Run only the localization module, without adjustment or loop closure
  LOCALIZATION
};

/// Type of features considered in this class
enum FeatureType {
  /// Scale-invariant feature transform (SIFT)
  SIFT,
  /// Attentive Local and Implicit Keypoint Detector (ALIKED)
  ALIKED
};

/// Type model used to run depth inference
enum DepthInferenceModel {
  /// DUSt3R: Geometric 3D Vision Made Easy
  DUST3R,
  /// Grounding Image Matching in 3D with MASt3R
  MAST3R,
  /// Depth Anything V2
  DEPTH_ANYTHING_V2,
  /// Visual Geometry Grounded Transformer
  VGGT
};

/// Type of robust loss function for noise models in map optimization
enum RobustLossType {
  /// Huber loss
  HUBER,
  /// Geman-McClure loss
  GEMAN_MCCLURE,
};

}  // namespace slam_mer::modes

#endif
