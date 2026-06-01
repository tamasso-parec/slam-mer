/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file config.hpp
/// @brief Defines the hierarchical configuration cache for the SLAM system.
///
/// This header declares `settings::Config`, the aggregate that stores all
/// runtime parameters used by the SLAM pipeline and its submodules. Each nested
/// struct mirrors one YAML section so values can be cached in strongly typed
/// fields after loading. The layout groups options by functional area
/// (front-end, localization, mapping, loop detection, and interface rendering).
/// Keeping this structure centralized avoids scattered string-based lookups.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki
#ifndef SLAM_CONFIG_HPP
#define SLAM_CONFIG_HPP

#include <array>
#include <cstddef>
#include <descriptor_model.hpp>
#include <string>
#include <types/modes.hpp>
#include <types/mutex.hpp>

#ifdef VISUAL_PLACE_LOCALIZATION
#include <vpl/visual_place_recognition.hpp>
#endif

namespace slam_mer {
namespace settings {

/// @brief Aggregates every configuration section loaded from YAML.
struct Config {
  /// @brief SLAM front-end settings.
  struct Slam {
    /// Requested runtime mode.
    modes::SystemMode _operationMode;

    /// @brief Feature extraction settings.
    struct Features {
      /// Feature extractor backend.
      modes::FeatureType _type;
      /// Enables RootSIFT normalization.
      bool _rootsift;
      /// Maximum number of extracted features.
      size_t _maximumFeaturesExtract;
      /// CUDA device identifier used by FAISS feature matching.
      int _faissCudaDeviceId = 0;
      /// Horizontal crop border in pixels.
      int _borderCutWidth;
      /// Vertical crop border in pixels.
      int _borderCutHeight;
    } _features;

    /// @brief Depth-inference settings.
    struct Inference {
      /// Depth estimator backend.
      modes::DepthInferenceModel _model;
      /// Path to the inference model.
      std::string _modelPath;
      /// Confidence threshold for valid depth.
      double _lowestConfidence;
      /// CUDA device identifier used by depth inference.
      int _cudaDeviceId = 0;
    } _inference;

    /// @brief ALIKED-specific settings.
    struct Aliked {
      /// ALIKED model variant name.
      std::string _modelName;
      /// Minimum ALIKED score.
      float _scoreTh;
      /// CUDA device identifier used by ALIKED.
      int _cudaDeviceId = 0;
    } _aliked;

    /// @brief SIFT/SiftCUDA-specific settings.
    struct Siftcuda {
      /// Number of pyramid octaves.
      int _numOctaves;
      /// Initial Gaussian blur.
      double _initBlur;
      /// Detector threshold.
      double _thresh;
      /// Minimum accepted scale.
      double _minScale;
      /// Enables image up-scaling before extraction.
      bool _upScale;
    } _sift;
  } _slam;

  /// @brief Localization module settings.
  struct Localization {
    /// @brief Input-frame quality constraints.
    struct Frame {
      /// Minimum features required in the frame.
      int _minimumFeatures;
      /// Minimum accepted depth value.
      double _minimumDepth;
      /// Maximum accepted depth value.
      double _maximumDepth;
    } _frame;

    /// @brief Point-query cache and spatial lookup settings.
    struct PointsQuery {
      /// Number of previous frames to keep.
      int _temporalBufferSize;
      /// Number of cells used for spatial queries.
      int _spatialQueryCells;
    } _pointsQuery;

    /// @brief Camera-pose estimation settings.
    struct PoseEstimation {
      /// Minimum 2D-3D matches required.
      int _minimumMatches;
      /// Iterations for focal estimation.
      int _focalLengthMaximumEstimates;
      /// Minimum accepted inlier ratio.
      double _minimumInlierRatio;
      /// Minimum accepted inlier count.
      int _minimumInliers;
      /// Maximum rotation delta.
      double _poseLimitsRotation;
      /// Maximum translation delta.
      double _poseLimitsTranslation;

      /// @brief RANSAC settings for PnP.
      struct Ransac {
        /// Maximum RANSAC iterations.
        int _maxIterations;
        /// Minimum RANSAC iterations.
        int _minIterations;
        /// Multiplier for dynamic trial count.
        double _dynNumTrialsMult;
        /// Target RANSAC success probability.
        double _successProb;
      } _ransac;

      /// @brief Local bundle-adjustment settings.
      struct Bundle {
        /// Maximum optimization iterations.
        int _maxIterations;
      } _bundle;
    } _pose_estimation;

    /// @brief Keyframe-creation policy settings.
    struct KeyframeCreation {
      /// Minimum frames between keyframes.
      int _framesInBetweenKeyframes;
      /// Enables tracked-point check.
      bool _checkMinimumTrackedPoints;
      /// Minimum tracked points threshold.
      int _minimumTrackedPoints;
      /// Enables image-coverage check.
      bool _checkImageCoverage;
      /// Minimum image coverage ratio.
      double _imageCoverageThreshold;
      /// Coverage grid row count.
      int _imageCoverageGridRows;
      /// Coverage grid column count.
      int _imageCoverageGridCols;
      /// Min points per grid cell.
      int _imageCoverageMinPointsPerGrid;
      /// Min valid grid cells.
      int _imageCoverageMinGridsFilled;
      /// Enables KL-divergence check.
      bool _checkKlDivergence;
      /// Maximum KL divergence.
      double _klDivergenceThreshold;
    } _keyframe_creation;

    /// @brief Relocalization settings.
    struct Relocalization {
#ifdef VISUAL_PLACE_LOCALIZATION
      /// Number of retrieval candidates.
      int _queryTopK;
      /// Similarity acceptance factor.
      double _similarityScoreFactor;
#endif  // VISUAL_PLACE_LOCALIZATION
    } _relocalization;
  } _localization;

  /// @brief Map and graph-optimization settings.
  struct Map {
    /// @brief Point-cloud representation settings.
    struct Representation {
      /// Half-size of each voxel cell.
      double _voxelHalfSize;
      /// Half-size for coarse map buckets.
      double _coarseHalfSize;
      /// Global map scale factor.
      double _scaleFactor;
    } _representation;

    /// @brief Point-visibility settings.
    struct Visibility {
      /// IoU threshold for visibility consistency.
      double _iouThreshold;
    } _visibility;

    /// @brief Spatial-neighborhood settings.
    struct Neighbors {
      /// Maximum points sampled per keyframe.
      size_t _pointsPerKeyframe;
      /// Distance threshold for neighbors.
      double _distanceThreshold;
      /// Neighborhood search radius.
      int _searchRadius;
    } _neighbors;

    /// @brief Covisibility-graph and optimization settings.
    struct Covisibility {
      /// Enables similarity-based optimization (Sim(3)).
      bool _useSimilarity;
      /// Target processing FPS for the module.
      int _moduleFps;
      /// Threshold for factor relinearization.
      double _relinearizeThreshold;
      /// Frames to skip between checks.
      int _relinearizeSkip;
      /// Clamp weight for rigid-window edges.
      double _rigidWindowClampWeight;
      /// Descriptor fusion mode.
      utils::DescriptorModel::Type _descriptorModel;

      /// @brief 2D projection-edge settings.
      struct ProjectionEdge {
        /// Measurement sigma.
        double _sigma;
        /// Enables robust loss.
        bool _robust;
        /// Robust loss function.
        modes::RobustLossType _loss;
        /// Robust loss shape parameter.
        double _shape;
      } _projectionEdge;

      /// @brief 3D distance-edge settings.
      struct DistanceEdge {
        /// Measurement sigma.
        double _sigma;
        /// Enables robust loss.
        bool _robust;
        /// Robust loss function.
        modes::RobustLossType _loss;
        /// Robust loss shape parameter.
        double _shape;
      } _distanceEdge;

      /// @brief Relative odometry-edge settings.
      struct OdometryEdge {
        /// Minimum matches for odometry factors.
        int _minimumMatches;
        /// Rotation sigma.
        double _sigmaRotation;
        /// Translation sigma.
        double _sigmaTranslation;
        /// Scale sigma.
        double _sigmaScale;
        /// Enables robust loss.
        bool _robust;
        /// Robust loss function.
        modes::RobustLossType _loss;
        /// Robust loss shape parameter.
        double _shape;
      } _odometryEdge;

      /// @brief Loop-closure edge settings.
      struct LoopEdge {
        /// Relative weight for loop edges.
        double _weight;
      } _loopEdge;
    } _covisibility;
  } _map;

  /// @brief Loop-detection settings.
  struct Loop {
#ifdef VISUAL_PLACE_LOCALIZATION
    /// Target processing FPS for loop detection.
    int _moduleFps;

    /// @brief Visual place-recognition settings.
    struct VisualPlaceRecognition {
      using VisualPlaceRecognitionMethod =
          vpl::visual_place_recognition::VisualPlaceRecognitionMethod;
      /// Retrieval backend.
      VisualPlaceRecognitionMethod _method;
      /// Path to ONNX retrieval model.
      std::string _onnxModel;
      /// Path to BoW vocabulary.
      std::string _bowVocabulary;
      /// CUDA device identifier used by VPL inference/database backends.
      int _cudaDeviceId = 0;
    } _visualPlaceRecognition;

    /// Number of retrieval candidates.
    int _queryTopK;
    /// Similarity acceptance factor.
    double _similarityScoreFactor;
#endif  // VISUAL_PLACE_LOCALIZATION
  } _loop;

  /// @brief Debug and visualization settings.
  struct Interface {
    /// @brief RGB colors used in 2D visualizations.
    struct Color {
      /// Default map-point color.
      std::array<unsigned int, 3> _point;
      /// Inlier-track color.
      std::array<unsigned int, 3> _inlier;
      /// All tracked points color.
      std::array<unsigned int, 3> _allPoints;
      /// Tracked features color.
      std::array<unsigned int, 3> _trackingFeatures;
      /// Overlay text color.
      std::array<unsigned int, 3> _text;
    } _color;

    /// @brief Generic drawing-size settings.
    struct Size {
      /// Default line thickness.
      int _thickness;
    } _size;

    /// @brief Point-size settings.
    struct Points {
      /// Sparse point size.
      float _sparseSize;
      /// Dense point size.
      float _denseSize;
    } _points;

    /// @brief Grid-cell rendering settings.
    struct Cells {
      /// Default cell border thickness.
      float _thickness;
      /// Highlighted cell border thickness.
      float _thicknessHighlighted;
      /// Default RGBA cell color.
      std::array<unsigned int, 4> _color;
      /// Highlight RGBA color.
      std::array<unsigned int, 4> _colorHighlighted;
    } _cells;

    /// @brief Covisibility-edge rendering settings.
    struct Covisibility {
      /// Covisibility edge thickness.
      float _thickness;
      /// Covisibility edge RGBA color.
      std::array<unsigned int, 4> _color;
    } _covisibility;
  } _interface;
};

}  // namespace settings
}  // namespace slam_mer

#endif  // SLAM_CONFIG_HPP
