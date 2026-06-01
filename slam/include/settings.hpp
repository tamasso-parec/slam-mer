/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file settings.hpp
/// @brief Declares `Settings`, the runtime YAML configuration loader/cache.
///
/// This header defines the API that loads project settings from YAML files and
/// exposes them through typed getters. The implementation merges a user file
/// with defaults so missing keys keep stable baseline values. It also maps
/// string tokens to strongly typed enums used by the rest of the system.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki
#ifndef SLAM_SETTINGS_HPP
#define SLAM_SETTINGS_HPP

#include <filesystem>
#include <string>

// Include third party libraries
#include <config.hpp>
#include <descriptor_model.hpp>
#include <log.hpp>
#include <types/array.hpp>
#include <types/image_processing.hpp>
#include <types/modes.hpp>

#ifdef VISUAL_PLACE_LOCALIZATION
#include <vpl/visual_place_recognition.hpp>
#endif

// third-party
#include <yaml-cpp/yaml.h>

namespace slam_mer {

namespace fs = std::filesystem;

/// @brief Loads and caches SLAM settings from YAML configuration files.
///
/// The loader merges a user-provided YAML file with the default settings and
/// exposes strongly typed getters for all modules.
class Settings {
public:
#ifdef VISUAL_PLACE_LOCALIZATION
  using VisualPlaceRecognitionMethod =
      vpl::visual_place_recognition::VisualPlaceRecognitionMethod;
#endif

  /// @brief Shared-pointer alias for a settings instance.
  typedef std::shared_ptr<Settings> shared_ptr;

  /// Static factory method to create a shared pointer to a `Settings` instance.
  /// @return Shared pointer to a new `Settings` instance.
  [[nodiscard("Use make_shared() to create a Settings instance")]]
  static Settings::shared_ptr make_shared() {
    return std::make_shared<Settings>();
  }

  /// @brief Construct a settings loader/cache with default-initialized values.
  Settings();

  /// @brief Destroy the settings object.
  ~Settings() {};

  /// @brief Load and merge settings from a YAML file path.
  /// @details Reads defaults, applies user overrides from `path_`, and updates
  /// typed cached fields used by getters.
  /// @param path_ Filesystem path to the YAML settings file.
  void load(const fs::path& path_);

  /// @brief String-path overload of `load(const fs::path&)`.
  /// @param path_ Path to the YAML settings file.
  void load(const std::string& path_) { this->load(fs::path(path_)); };

  /// ------------------------------------------------------------------------
  /// Getters for SLAM settings
  /// ------------------------------------------------------------------------

  /// @brief Returns the operation mode of the system.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline modes::SystemMode operation_mode() const {
    return this->_cfg._slam._operationMode;
  }

  /// @brief Returns the feature type used in SLAM.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline modes::FeatureType feature_type() const {
    return _cfg._slam._features._type;
  }

  /// @brief Returns the maximum number of features to extract.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline size_t maximum_features() const {
    return this->_cfg._slam._features._maximumFeaturesExtract;
  }

  /// @brief Returns the CUDA device used by FAISS feature matching.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int faiss_cuda_device_id() const {
    return this->_cfg._slam._features._faissCudaDeviceId;
  }

  /// @brief Returns the border cut width for feature extraction.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int border_cut_width() const {
    return this->_cfg._slam._features._borderCutWidth;
  }

  /// @brief Returns the border cut height for feature extraction.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int border_cut_height() const {
    return this->_cfg._slam._features._borderCutHeight;
  }

  /// @brief Returns whether RootSIFT is applied.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool apply_rootsift() const {
    return this->_cfg._slam._features._rootsift;
  }

  /// @brief Returns the depth inference model.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline modes::DepthInferenceModel depth_inference_model() const {
    return this->_cfg._slam._inference._model;
  }

  /// @brief Returns the path to the depth ONNX model.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::string path_to_depth_onnx_model() const {
    return this->_cfg._slam._inference._modelPath;
  }

  /// @brief Returns the lowest confidence threshold for depth inference.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double depth_lowest_confidence() const {
    return this->_cfg._slam._inference._lowestConfidence;
  }

  /// @brief Returns the CUDA device used by depth inference.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int depth_inference_cuda_device_id() const {
    return this->_cfg._slam._inference._cudaDeviceId;
  }

  /// @brief Returns the name of the ALIKED model.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::string aliked_model_name() const {
    return this->_cfg._slam._aliked._modelName;
  }

  /// @brief Returns the ALIKED score threshold.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline float aliked_scores_th() const {
    return this->_cfg._slam._aliked._scoreTh;
  }

  /// @brief Returns the CUDA device used by ALIKED.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int aliked_cuda_device_id() const {
    return this->_cfg._slam._aliked._cudaDeviceId;
  }

  /// @brief Returns the number of octaves for SIFT.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int sift_num_octaves() const {
    return this->_cfg._slam._sift._numOctaves;
  }

  /// @brief Returns the initial blur for SIFT.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double sift_init_blur() const {
    return this->_cfg._slam._sift._initBlur;
  }

  /// @brief Returns the peak threshold for SIFT.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double sift_peak_thresh() const {
    return this->_cfg._slam._sift._thresh;
  }

  /// @brief Returns the minimum scale for SIFT.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double sift_min_scale() const {
    return this->_cfg._slam._sift._minScale;
  }

  /// @brief Returns whether SIFT upscaling is enabled.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool sift_up_scale() const {
    return this->_cfg._slam._sift._upScale;
  }

  /// ------------------------------------------------------------------------
  /// Getters for Localization settings
  /// ------------------------------------------------------------------------

  /// @brief Returns the minimum number of features per frame.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int minimum_frame_features() const {
    return this->_cfg._localization._frame._minimumFeatures;
  }

  /// @brief Returns the minimum depth for frames.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double frame_minimum_depth() const {
    return this->_cfg._localization._frame._minimumDepth;
  }

  /// @brief Returns the maximum depth for frames.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double frame_maximum_depth() const {
    return this->_cfg._localization._frame._maximumDepth;
  }

  /// @brief Returns the temporal buffer size for point queries.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int temporal_buffer_size() const {
    return this->_cfg._localization._pointsQuery._temporalBufferSize;
  }

  /// @brief Returns the number of spatial query cells.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int spatial_query_cells() const {
    return this->_cfg._localization._pointsQuery._spatialQueryCells;
  }

  /// @brief Returns the minimum number of matches for pose estimation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int pose_minimum_matches() const {
    return this->_cfg._localization._pose_estimation._minimumMatches;
  }

  /// @brief Returns the maximum number of focal length estimates.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int focal_length_estimates() const {
    return this->_cfg._localization._pose_estimation
        ._focalLengthMaximumEstimates;
  }

  /// @brief Returns the minimum inlier ratio for pose estimation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double pose_minimum_inlier_ratio() const {
    return this->_cfg._localization._pose_estimation._minimumInlierRatio;
  }

  /// @brief Returns the minimum number of inliers for pose estimation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int pose_minimum_inliers() const {
    return this->_cfg._localization._pose_estimation._minimumInliers;
  }

  /// @brief Returns the pose variation limits (rotation and translation).
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline array2D_t pose_variation_limits() const {
    return {
        this->_cfg._localization._pose_estimation._poseLimitsRotation,
        this->_cfg._localization._pose_estimation._poseLimitsTranslation};
  }

  /// @brief Returns the maximum number of RANSAC iterations for PnP.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int pnp_ransac_maximum_iterations() const {
    return this->_cfg._localization._pose_estimation._ransac._maxIterations;
  }

  /// @brief Returns the minimum number of RANSAC iterations for PnP.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int pnp_ransac_minimum_iterations() const {
    return this->_cfg._localization._pose_estimation._ransac._minIterations;
  }

  /// @brief Returns the dynamic number of trials multiplier for RANSAC.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double pnp_ransac_dyn_num_trials_mult() const {
    return this->_cfg._localization._pose_estimation._ransac._dynNumTrialsMult;
  }

  /// @brief Returns the RANSAC success probability.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double pnp_ransac_success_prob() const {
    return this->_cfg._localization._pose_estimation._ransac._successProb;
  }

  /// @brief Returns the maximum number of bundle adjustment iterations for PnP.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int pnp_bundle_maximum_iterations() const {
    return this->_cfg._localization._pose_estimation._bundle._maxIterations;
  }

  /// @brief Returns the minimum number of frames between keyframes.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int minimum_frames_between_keyframes() const {
    return this->_cfg._localization._keyframe_creation
        ._framesInBetweenKeyframes;
  }

  /// @brief Returns whether to use the tracked points condition for keyframe creation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool use_tracked_points_condition() const {
    return this->_cfg._localization._keyframe_creation
        ._checkMinimumTrackedPoints;
  }

  /// @brief Returns the minimum number of tracked points for keyframe creation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int minimum_tracked_points() const {
    return this->_cfg._localization._keyframe_creation._minimumTrackedPoints;
  }

  /// @brief Returns whether to use the image coverage condition for keyframe creation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool use_image_coverage_condition() const {
    return this->_cfg._localization._keyframe_creation._checkImageCoverage;
  }

  /// @brief Returns the image coverage threshold for keyframe creation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double image_coverage_threshold() const {
    return this->_cfg._localization._keyframe_creation._imageCoverageThreshold;
  }

  /// @brief Returns the number of grid rows for image coverage.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int image_coverage_grid_rows() const {
    return this->_cfg._localization._keyframe_creation._imageCoverageGridRows;
  }

  /// @brief Returns the number of grid columns for image coverage.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int image_coverage_grid_cols() const {
    return this->_cfg._localization._keyframe_creation._imageCoverageGridCols;
  }

  /// @brief Returns the minimum number of points per grid for image coverage.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int image_coverage_minimum_points_per_grid() const {
    return this->_cfg._localization._keyframe_creation
        ._imageCoverageMinPointsPerGrid;
  }

  /// @brief Returns the minimum number of grids filled for image coverage.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int image_coverage_minimum_grids_filled() const {
    return this->_cfg._localization._keyframe_creation
        ._imageCoverageMinGridsFilled;
  }

  /// @brief Returns whether to use KL divergence condition for keyframe creation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool use_kl_divergence_condition() const {
    return this->_cfg._localization._keyframe_creation._checkKlDivergence;
  }

  /// @brief Returns the KL divergence threshold for keyframe creation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double kl_divergence_threshold() const {
    return this->_cfg._localization._keyframe_creation._klDivergenceThreshold;
  }

#ifdef VISUAL_PLACE_LOCALIZATION

  /// @brief Returns the top K for relocalization query.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int relocalization_query_top_k() const {
    return this->_cfg._localization._relocalization._queryTopK;
  }

  /// @brief Returns the similarity score factor for relocalization.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double relocalization_similarity_score_factor() const {
    return this->_cfg._localization._relocalization._similarityScoreFactor;
  }

#endif  // VISUAL_PLACE_LOCALIZATION

  /// ------------------------------------------------------------------------
  /// Getters for Map settings
  /// ------------------------------------------------------------------------

  /// @brief Returns the voxel half size for map representation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double map_voxel_half_size() const {
    return this->_cfg._map._representation._voxelHalfSize;
  }

  /// @brief Returns the coarse half size for map representation.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double map_coarse_half_size() const {
    return this->_cfg._map._representation._coarseHalfSize;
  }

  /// @brief Returns the map scale factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double map_scale_factor() const {
    return this->_cfg._map._representation._scaleFactor;
  }

  /// @brief Returns the IoU threshold for map visibility.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double map_visibility_iou_threshold() const {
    return this->_cfg._map._visibility._iouThreshold;
  }

  /// @brief Returns the number of neighbor points per keyframe.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline size_t neighbors_points_per_keyframe() const {
    return this->_cfg._map._neighbors._pointsPerKeyframe;
  }

  /// @brief Returns the search radius for neighbors.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int neighbors_search_radius() const {
    return this->_cfg._map._neighbors._searchRadius;
  }

  /// @brief Returns the distance threshold for neighbors.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double neighbors_distance_threshold() const {
    return this->_cfg._map._neighbors._distanceThreshold;
  }

  /// @brief Returns the descriptor model type.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline utils::DescriptorModel::Type descriptor_model_type() const {
    return this->_cfg._map._covisibility._descriptorModel;
  }

  /// @brief Returns whether covisibility uses similarity-based optimization.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool covisibility_use_similarity() const {
    return this->_cfg._map._covisibility._useSimilarity;
  }

  /// @brief Returns the covisibility module FPS.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int covisibility_module_fps() const {
    return this->_cfg._map._covisibility._moduleFps;
  }

  /// @brief Returns the relinearize threshold for covisibility.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double covisibility_relinearize_threshold() const {
    return this->_cfg._map._covisibility._relinearizeThreshold;
  }

  /// @brief Returns the relinearize skip value for covisibility.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int covisibility_relinearize_skip() const {
    return this->_cfg._map._covisibility._relinearizeSkip;
  }

  /// @brief Returns the rigid window clamp weight for covisibility.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double covisibility_rigid_window_clamp_weight() const {
    return this->_cfg._map._covisibility._rigidWindowClampWeight;
  }

  /// @brief Returns the sigma value for the projection factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double projection_factor_sigma() const {
    return this->_cfg._map._covisibility._projectionEdge._sigma;
  }

  /// @brief Returns whether to use a robust projection factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool use_robust_projection_factor() const {
    return this->_cfg._map._covisibility._projectionEdge._robust;
  }

  /// @brief Returns the robust loss type for the projection factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline modes::RobustLossType robust_loss_for_projection_factor() const {
    return this->_cfg._map._covisibility._projectionEdge._loss;
  }

  /// @brief Returns the robust loss shape for the projection factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double robust_loss_shape_for_projection_factor() const {
    return this->_cfg._map._covisibility._projectionEdge._shape;
  }

  /// @brief Returns the sigma value for the distance factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double distance_factor_sigma() const {
    return this->_cfg._map._covisibility._distanceEdge._sigma;
  }

  /// @brief Returns whether to use a robust distance factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool use_robust_distance_factor() const {
    return this->_cfg._map._covisibility._distanceEdge._robust;
  }

  /// @brief Returns the robust loss type for the distance factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline modes::RobustLossType robust_loss_for_distance_factor() const {
    return this->_cfg._map._covisibility._distanceEdge._loss;
  }

  /// @brief Returns the robust loss shape for the distance factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double robust_loss_shape_for_distance_factor() const {
    return this->_cfg._map._covisibility._distanceEdge._shape;
  }

  /// @brief Returns the minimum number of matches for the odometry edge.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int odometry_edge_minimum_matches() const {
    return this->_cfg._map._covisibility._odometryEdge._minimumMatches;
  }

  /// @brief Returns the rotation sigma for the odometry factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double odometry_factor_rotation_sigma() const {
    return this->_cfg._map._covisibility._odometryEdge._sigmaRotation;
  }

  /// @brief Returns the translation sigma for the odometry factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double odometry_factor_translation_sigma() const {
    return this->_cfg._map._covisibility._odometryEdge._sigmaTranslation;
  }

  /// @brief Returns the scale sigma for the odometry factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double odometry_factor_scale_sigma() const {
    return this->_cfg._map._covisibility._odometryEdge._sigmaScale;
  }

  /// @brief Returns whether to use a robust odometry factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline bool use_robust_odometry_factor() const {
    return this->_cfg._map._covisibility._odometryEdge._robust;
  }

  /// @brief Returns the robust loss type for the odometry factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline modes::RobustLossType robust_loss_for_odometry_factor() const {
    return this->_cfg._map._covisibility._odometryEdge._loss;
  }

  /// @brief Returns the robust loss shape for the odometry factor.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double robust_loss_shape_for_odometry_factor() const {
    return this->_cfg._map._covisibility._odometryEdge._shape;
  }

  /// @brief Returns the loop edge weight.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double loop_edge_weight() const {
    return this->_cfg._map._covisibility._loopEdge._weight;
  }

  /// ------------------------------------------------------------------------
  /// Getters for Loop settings
  /// ------------------------------------------------------------------------

#ifdef VISUAL_PLACE_LOCALIZATION

  /// @brief Returns the loop module FPS.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int loop_module_fps() const {
    return this->_cfg._loop._moduleFps;
  }

  /// @brief Returns the visual place recognition method.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline VisualPlaceRecognitionMethod visual_place_recognition_method() const {
    return this->_cfg._loop._visualPlaceRecognition._method;
  }

  /// @brief Returns the path to the visual place recognition ONNX model.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::string path_to_visual_place_recognition_onnx_model() const {
    return this->_cfg._loop._visualPlaceRecognition._onnxModel;
  }

  /// @brief Returns the path to the BoW vocabulary.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::string path_to_bow_vocabulary() const {
    return this->_cfg._loop._visualPlaceRecognition._bowVocabulary;
  }

  /// @brief Returns the CUDA device used by visual place recognition.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int visual_place_recognition_cuda_device_id() const {
    return this->_cfg._loop._visualPlaceRecognition._cudaDeviceId;
  }

  /// @brief Returns the top K for loop query.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int loop_query_top_k() const {
    return this->_cfg._loop._queryTopK;
  }

  /// @brief Returns the similarity score factor for loop detection.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline double loop_similarity_score_factor() const {
    return this->_cfg._loop._similarityScoreFactor;
  }

#endif

  /// ------------------------------------------------------------------------
  /// Getters for Interface settings
  /// ------------------------------------------------------------------------

  /// @brief Returns the default point color for the interface image.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::array<unsigned int, 3> interface_image_default_point_color()
      const {
    return this->_cfg._interface._color._point;
  }

  /// @brief Returns the inlier point color for the interface image.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::array<unsigned int, 3> interface_image_inlier_point_color()
      const {
    return this->_cfg._interface._color._inlier;
  }

  /// @brief Returns the color for all points in the interface image.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::array<unsigned int, 3> interface_image_all_point_color() const {
    return this->_cfg._interface._color._allPoints;
  }

  /// @brief Returns the tracking feature color for the interface image.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::array<unsigned int, 3> interface_image_tracking_color() const {
    return this->_cfg._interface._color._trackingFeatures;
  }

  /// @brief Returns the text color for the interface image.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::array<unsigned int, 3> interface_image_text_color() const {
    return this->_cfg._interface._color._text;
  }

  /// @brief Returns the line thickness for the interface image.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline int interface_image_line_thickness() const {
    return this->_cfg._interface._size._thickness;
  }

  /// @brief Returns the sparse point size for the interface.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline float interface_sparse_point_size() const {
    return this->_cfg._interface._points._sparseSize;
  }

  /// @brief Returns the dense point size for the interface.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline float interface_dense_point_size() const {
    return this->_cfg._interface._points._denseSize;
  }

  /// @brief Returns the cell thickness for the interface.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline float interface_cell_thickness() const {
    return this->_cfg._interface._cells._thickness;
  }

  /// @brief Returns the highlighted cell thickness for the interface.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline float interface_cell_highlighted_thickness() const {
    return this->_cfg._interface._cells._thicknessHighlighted;
  }

  /// @brief Returns the cell color for the interface.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::array<unsigned int, 4> interface_cell_color() const {
    return this->_cfg._interface._cells._color;
  }

  /// @brief Returns the highlighted cell color for the interface.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::array<unsigned int, 4> interface_cell_highlighted_color() const {
    return this->_cfg._interface._cells._colorHighlighted;
  }

  /// @brief Returns the covisibility line thickness for the interface.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline float interface_covisibility_line_thickness() const {
    return this->_cfg._interface._covisibility._thickness;
  }

  /// @brief Returns the covisibility line color for the interface.
  /// @return Configured value loaded from the merged settings files.
  [[nodiscard("Use returned setting value.")]]
  inline std::array<unsigned int, 4> interface_covisibility_line_color() const {
    return this->_cfg._interface._covisibility._color;
  }

private:
  /// Path to the settings files
  fs::path _path;
  /// All cached settings in a single structure
  settings::Config _cfg;
  /// Keep YAML::Node for backward compatibility and debugging
  YAML::Node _node;
};

namespace settings {

/// @brief Process-wide shared Settings instance.
extern slam_mer::Settings::shared_ptr sharedSettings;

}  // namespace settings

}  // namespace slam_mer

#endif  // SLAM_SETTINGS_HPP
