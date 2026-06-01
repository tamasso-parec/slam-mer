/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file settings.cpp
/// @brief Implements settings loading and YAML merge helpers.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki

#include <settings.hpp>
namespace slam_mer {

namespace {

/// @brief Flatten a YAML node into `key = value` text entries.
/// @details Traverses maps and sequences recursively and returns a list of
/// dotted-path string entries useful for debug printing.
/// @param node_ YAML node to flatten.
/// @param prefix_ Current key prefix used during recursive traversal.
/// @return Flattened list of key-value strings.
std::vector<std::string> yamlToStringList(
    const YAML::Node& node_,
    const std::string& prefix_ = "") {
  std::vector<std::string> out;

  if (node_.IsScalar()) {  // leaf node
    out.push_back(prefix_ + " = \"" + node_.as<std::string>() + "\"");
  } else if (node_.IsSequence()) {
    std::stringstream ss;
    ss << "[";

    for (size_t i = 0; i < node_.size(); i++) {
      if (i > 0) ss << ", ";      // store comma between elements
      if (node_[i].IsScalar()) {  // leaf node
        ss << node_[i].as<std::string>();
      } else {
        ss << node_[i];
      }
    }
    ss << "]";

    out.push_back(prefix_ + " = \"" + ss.str() + "\"");
  } else if (node_.IsMap()) {
    for (auto it : node_) {
      const std::string key = it.first.as<std::string>();
      // extract recursively by keeping parent keys as prefix
      const std::string new_prefix =
          prefix_.empty() ? key : prefix_ + "." + key;
      const auto child = yamlToStringList(it.second, new_prefix);
      out.insert(out.end(), child.begin(), child.end());
    }
  }

  return out;
}

/// @brief Recursively merge two YAML mapping nodes.
/// @details For overlapping keys, scalar/sequence values from `source_`
/// overwrite `target_`; nested maps are merged recursively.
/// @param source_ Source node providing overriding values.
/// @param target_ Target node providing baseline values.
/// @return Merged YAML node.
YAML::Node mergeYamlNodes(
    const YAML::Node& source_,
    const YAML::Node& target_) {
  // check validity of source and target nodes
  const bool isSourceValid =
      (source_.IsMap() && !source_.IsNull() && source_.size() > 0);
  const bool isTargetValid =
      (target_.IsMap() && !target_.IsNull() && target_.size() > 0);

  // trivial cases
  if (!isSourceValid && !isTargetValid) return YAML::Node();
  if (!isSourceValid) return target_;
  if (!isTargetValid) return source_;

  // recursive merge
  YAML::Node result = YAML::Clone(target_);
  for (const auto& kv : source_) {
    const std::string& key = kv.first.as<std::string>();
    const YAML::Node& sourceValue = kv.second;

    // check if key exists in target
    if (target_[key]) {
      const YAML::Node targetValue = target_[key];
      // if both source and target values are maps, merge them recursively
      if (sourceValue.IsMap() && targetValue.IsMap()) {
        result[key] = mergeYamlNodes(sourceValue, targetValue);
      } else {
        // otherwise, overwrite target value with source value
        result[key] = sourceValue;
      }
    } else {
      // key doesn't exist in target, add it from source
      result[key] = YAML::Clone(sourceValue);
    }
  }
  return result;
}

}  // namespace

namespace {

using utils::DescriptorModel;
#ifdef VISUAL_PLACE_LOCALIZATION
using VisualPlaceRecognitionMethod =
    vpl::visual_place_recognition::VisualPlaceRecognitionMethod;
#endif

/// Mapping from string to SystemMode
const std::unordered_map<std::string, modes::SystemMode> __systemModeMap = {
    {"SLAM", modes::SystemMode::SLAM},
    {"SLAM_WITHOUT_LOOP", modes::SystemMode::SLAM_WITHOUT_LOOP},
    {"LOCALIZATION", modes::SystemMode::LOCALIZATION}};

/// Mapping from string to FeatureType
const std::unordered_map<std::string, modes::FeatureType> __featureMap = {
    {"SIFT", modes::FeatureType::SIFT},
    {"ALIKED", modes::FeatureType::ALIKED}};

/// Mapping from string to DepthInferenceModel
const std::unordered_map<std::string, modes::DepthInferenceModel> __modelMap = {
    {"DUST3R", modes::DepthInferenceModel::DUST3R},
    {"MAST3R", modes::DepthInferenceModel::MAST3R},
    {"DEPTH_ANYTHING_V2", modes::DepthInferenceModel::DEPTH_ANYTHING_V2},
    {"VGGT", modes::DepthInferenceModel::VGGT}};

/// Mapping from string to RobustLossType
const std::unordered_map<std::string, modes::RobustLossType> __robustLossMap = {
    {"HUBER", modes::RobustLossType::HUBER},
    {"GEMAN_MCCLURE", modes::RobustLossType::GEMAN_MCCLURE}};

/// Mapping from string to DescriptorModel::Type
const std::unordered_map<std::string, DescriptorModel::Type>
    __descriptorModelMap = {
        {"LAST_SEEN", DescriptorModel::Type::LAST_SEEN},
        {"MEDOID", DescriptorModel::Type::MEDOID},
        {"AVERAGE", DescriptorModel::Type::AVERAGE},
        {"WEIGHTED_AVERAGE", DescriptorModel::Type::WEIGHTED_AVERAGE}};

#ifdef VISUAL_PLACE_LOCALIZATION
/// Mapping from string to VisualPlaceRecognitionMethod
const std::unordered_map<std::string, VisualPlaceRecognitionMethod>
    __visualPlaceRecognitionMap = {
        {"BOW", VisualPlaceRecognitionMethod::BOW},
        {"MEGALOC", VisualPlaceRecognitionMethod::MEGALOC}};
#endif

}  // namespace

// ---------------------------------------------------------------------------
// Initialize settings cache with default YAML file
// ---------------------------------------------------------------------------

Settings::Settings() : _node(YAML::LoadFile("settings/default.yaml")) {}

// ---------------------------------------------------------------------------
// Load user settings and merge them with default configuration
// ---------------------------------------------------------------------------

void Settings::load(const fs::path& path_) {
  _path = path_;

#ifdef LOG_SETTINGS
  log::message("Loading settings (at " + path_.string() + " )", log::SETTINGS);
#endif

  // Load YAML file and merge with default config
  {
    const YAML::Node config = YAML::LoadFile(path_.string());
    _node = mergeYamlNodes(config, _node);
  }

  // ------------------------------------------------------------------------
  // Cache all settings into structures
  // ------------------------------------------------------------------------

  try {

    // Slam settings
    {
      _cfg._slam._operationMode =
          __systemModeMap.at(_node["Slam"]["Mode"].as<std::string>());
      _cfg._slam._features._type =
          __featureMap.at(_node["Slam"]["Features"]["Type"].as<std::string>());
      _cfg._slam._features._rootsift =
          _node["Slam"]["Features"]["RootSIFT"].as<bool>();
      _cfg._slam._features._maximumFeaturesExtract =
          _node["Slam"]["Features"]["MaximumFeaturesExtract"].as<size_t>();
      _cfg._slam._features._faissCudaDeviceId =
          _node["Slam"]["Features"]["FaissCudaDeviceId"]
              ? _node["Slam"]["Features"]["FaissCudaDeviceId"].as<int>()
              : 0;
      _cfg._slam._features._borderCutWidth =
          _node["Slam"]["Features"]["BorderCutWidth"].as<int>();
      _cfg._slam._features._borderCutHeight =
          _node["Slam"]["Features"]["BorderCutHeight"].as<int>();

      _cfg._slam._inference._model =
          __modelMap.at(_node["Slam"]["Inference"]["Model"].as<std::string>());
      _cfg._slam._inference._modelPath =
          _node["Slam"]["Inference"]["ModelPath"].as<std::string>();
      _cfg._slam._inference._lowestConfidence =
          _node["Slam"]["Inference"]["LowerConfidence"].as<double>();
      _cfg._slam._inference._cudaDeviceId =
          _node["Slam"]["Inference"]["CudaDeviceId"]
              ? _node["Slam"]["Inference"]["CudaDeviceId"].as<int>()
              : 0;

      _cfg._slam._aliked._modelName =
          _node["Slam"]["Aliked"]["ModelName"].as<std::string>();
      _cfg._slam._aliked._scoreTh =
          _node["Slam"]["Aliked"]["ScoreTh"].as<float>();
      _cfg._slam._aliked._cudaDeviceId =
          _node["Slam"]["Aliked"]["CudaDeviceId"]
              ? _node["Slam"]["Aliked"]["CudaDeviceId"].as<int>()
              : 0;

      _cfg._slam._sift._numOctaves =
          _node["Slam"]["Sift"]["NumOctaves"].as<int>();
      _cfg._slam._sift._initBlur =
          _node["Slam"]["Sift"]["InitBlur"].as<double>();
      _cfg._slam._sift._thresh = _node["Slam"]["Sift"]["Thresh"].as<double>();
      _cfg._slam._sift._minScale =
          _node["Slam"]["Sift"]["MinScale"].as<double>();
      _cfg._slam._sift._upScale = _node["Slam"]["Sift"]["UpScale"].as<bool>();
    }

    // Localization settings
    {
      _cfg._localization._frame._minimumFeatures =
          _node["Localization"]["Frame"]["MinimumFeatures"].as<int>();
      _cfg._localization._frame._minimumDepth =
          _node["Localization"]["Frame"]["MinDepth"].as<double>();
      _cfg._localization._frame._maximumDepth =
          _node["Localization"]["Frame"]["MaxDepth"].as<double>();

      _cfg._localization._pointsQuery._temporalBufferSize =
          _node["Localization"]["PointsQuery"]["TemporalBufferSize"].as<int>();
      _cfg._localization._pointsQuery._spatialQueryCells =
          _node["Localization"]["PointsQuery"]["SpatialCellsQuery"].as<int>();

      _cfg._localization._pose_estimation._minimumMatches =
          _node["Localization"]["PoseEstimation"]["MinimumMatches"].as<int>();
      _cfg._localization._pose_estimation._focalLengthMaximumEstimates =
          _node["Localization"]["PoseEstimation"]["FocalLengthMaximumEstimates"]
              .as<int>();
      _cfg._localization._pose_estimation._minimumInlierRatio =
          _node["Localization"]["PoseEstimation"]["InlierRatioThreshold"]
              .as<double>();
      _cfg._localization._pose_estimation._minimumInliers =
          _node["Localization"]["PoseEstimation"]["MinimumInliers"].as<int>();
      _cfg._localization._pose_estimation._poseLimitsRotation =
          _node["Localization"]["PoseEstimation"]["PoseLimitsRotation"]
              .as<double>();
      _cfg._localization._pose_estimation._poseLimitsTranslation =
          _node["Localization"]["PoseEstimation"]["PoseLimitsTranslation"]
              .as<double>();

      _cfg._localization._pose_estimation._ransac._maxIterations =
          _node["Localization"]["PoseEstimation"]["Ransac"]["MaximumIterations"]
              .as<int>();
      _cfg._localization._pose_estimation._ransac._minIterations =
          _node["Localization"]["PoseEstimation"]["Ransac"]["MinimumIterations"]
              .as<int>();
      _cfg._localization._pose_estimation._ransac._dynNumTrialsMult =
          _node["Localization"]["PoseEstimation"]["Ransac"]
               ["DynamicNumTrialsMultiplier"]
                   .as<double>();
      _cfg._localization._pose_estimation._ransac._successProb =
          _node["Localization"]["PoseEstimation"]["Ransac"]
               ["SuccessProbability"]
                   .as<double>();
      _cfg._localization._pose_estimation._bundle._maxIterations =
          _node["Localization"]["PoseEstimation"]["Bundle"]["MaximumIterations"]
              .as<int>();

      _cfg._localization._keyframe_creation._framesInBetweenKeyframes =
          _node["Localization"]["KeyframeCreation"]["FramesInBetweenKeyframes"]
              .as<int>();
      _cfg._localization._keyframe_creation._checkMinimumTrackedPoints =
          _node["Localization"]["KeyframeCreation"]["CheckMinimumTrackedPoints"]
              .as<bool>();
      _cfg._localization._keyframe_creation._minimumTrackedPoints =
          _node["Localization"]["KeyframeCreation"]["MinimumTrackedPoints"]
              .as<int>();
      _cfg._localization._keyframe_creation._checkImageCoverage =
          _node["Localization"]["KeyframeCreation"]["CheckImageCoverage"]
              .as<bool>();
      _cfg._localization._keyframe_creation._imageCoverageThreshold =
          _node["Localization"]["KeyframeCreation"]["ImageCoverageThreshold"]
              .as<double>();
      _cfg._localization._keyframe_creation._imageCoverageGridRows =
          _node["Localization"]["KeyframeCreation"]["GridRows"].as<int>();
      _cfg._localization._keyframe_creation._imageCoverageGridCols =
          _node["Localization"]["KeyframeCreation"]["GridCols"].as<int>();
      _cfg._localization._keyframe_creation._imageCoverageMinPointsPerGrid =
          _node["Localization"]["KeyframeCreation"]["MinimumPointsPerCell"]
              .as<int>();
      _cfg._localization._keyframe_creation._imageCoverageMinGridsFilled =
          _node["Localization"]["KeyframeCreation"]["MinimumCellsCovered"]
              .as<int>();
      _cfg._localization._keyframe_creation._checkKlDivergence =
          _node["Localization"]["KeyframeCreation"]["CheckKLDivergence"]
              .as<bool>();
      _cfg._localization._keyframe_creation._klDivergenceThreshold =
          _node["Localization"]["KeyframeCreation"]["KLDivergenceThreshold"]
              .as<double>();

#ifdef VISUAL_PLACE_LOCALIZATION
      _cfg._localization._relocalization._queryTopK =
          _node["Localization"]["Relocalization"]["QueryTopK"].as<int>();
      _cfg._localization._relocalization._similarityScoreFactor =
          _node["Localization"]["Relocalization"]["SimilarityScoreFactor"]
              .as<double>();
#endif  // VISUAL_PLACE_LOCALIZATION
    }

    // Map settings
    {
      const YAML::Node covisibilityNode = _node["Map"]["Covisibility"];

      _cfg._map._representation._voxelHalfSize =
          _node["Map"]["Representation"]["VoxelHalfSize"].as<double>();
      _cfg._map._representation._coarseHalfSize =
          _node["Map"]["Representation"]["CoarseHalfSize"].as<double>();
      _cfg._map._representation._scaleFactor =
          _node["Map"]["Representation"]["ScaleFactor"].as<double>();
      _cfg._map._visibility._iouThreshold =
          _node["Map"]["Visibility"]["IoUThreshold"].as<double>();

      _cfg._map._neighbors._pointsPerKeyframe =
          _node["Map"]["Neighbors"]["PointsPerKeyframe"].as<size_t>();
      _cfg._map._neighbors._searchRadius =
          _node["Map"]["Neighbors"]["SearchRadius"].as<int>();
      _cfg._map._neighbors._distanceThreshold =
          _node["Map"]["Neighbors"]["DistanceThreshold"].as<double>();

      _cfg._map._covisibility._useSimilarity =
          _node["Map"]["Covisibility"]["UseSimilarity"].as<bool>();
      _cfg._map._covisibility._descriptorModel = __descriptorModelMap.at(
          covisibilityNode["Descriptors"]["Model"].as<std::string>());

      _cfg._map._covisibility._projectionEdge._sigma =
          covisibilityNode["ProjectionEdge"]["Sigma"].as<double>();
      _cfg._map._covisibility._projectionEdge._robust =
          covisibilityNode["ProjectionEdge"]["Robust"].as<bool>();
      _cfg._map._covisibility._projectionEdge._loss = __robustLossMap.at(
          covisibilityNode["ProjectionEdge"]["Loss"].as<std::string>());
      _cfg._map._covisibility._projectionEdge._shape =
          covisibilityNode["ProjectionEdge"]["Shape"].as<double>();

      _cfg._map._covisibility._distanceEdge._sigma =
          covisibilityNode["DistanceEdge"]["Sigma"].as<double>();
      _cfg._map._covisibility._distanceEdge._robust =
          covisibilityNode["DistanceEdge"]["Robust"].as<bool>();
      _cfg._map._covisibility._distanceEdge._loss = __robustLossMap.at(
          covisibilityNode["DistanceEdge"]["Loss"].as<std::string>());
      _cfg._map._covisibility._distanceEdge._shape =
          covisibilityNode["DistanceEdge"]["Shape"].as<double>();

      _cfg._map._covisibility._odometryEdge._minimumMatches =
          covisibilityNode["OdometryEdge"]["MinMatches"].as<int>();
      _cfg._map._covisibility._odometryEdge._sigmaRotation =
          covisibilityNode["OdometryEdge"]["SigmaRotation"].as<double>();
      _cfg._map._covisibility._odometryEdge._sigmaTranslation =
          covisibilityNode["OdometryEdge"]["SigmaTranslation"].as<double>();
      _cfg._map._covisibility._odometryEdge._sigmaScale =
          covisibilityNode["OdometryEdge"]["SigmaScale"].as<double>();
      _cfg._map._covisibility._odometryEdge._robust =
          covisibilityNode["OdometryEdge"]["Robust"].as<bool>();
      _cfg._map._covisibility._odometryEdge._loss = __robustLossMap.at(
          covisibilityNode["OdometryEdge"]["Loss"].as<std::string>());
      _cfg._map._covisibility._odometryEdge._shape =
          covisibilityNode["OdometryEdge"]["Shape"].as<double>();
      _cfg._map._covisibility._loopEdge._weight =
          covisibilityNode["LoopEdge"]["Weight"].as<double>();
      _cfg._map._covisibility._moduleFps =
          covisibilityNode["ModuleFps"].as<int>();
      _cfg._map._covisibility._relinearizeThreshold =
          covisibilityNode["RelinearizeThreshold"].as<double>();
      _cfg._map._covisibility._relinearizeSkip =
          covisibilityNode["RelinearizeSkip"].as<int>();
      _cfg._map._covisibility._rigidWindowClampWeight =
          covisibilityNode["RigidWindowClampWeight"].as<double>();
    }

    // Loop settings
    {
#ifdef VISUAL_PLACE_LOCALIZATION
      _cfg._loop._moduleFps = _node["Loop"]["ModuleFps"].as<int>();

      _cfg._loop._visualPlaceRecognition
          ._method = __visualPlaceRecognitionMap.at(
          _node["Loop"]["VisualPlaceRecognition"]["Method"].as<std::string>());
      _cfg._loop._visualPlaceRecognition._onnxModel =
          _node["Loop"]["VisualPlaceRecognition"]["OnnxModel"]
              .as<std::string>();
      _cfg._loop._visualPlaceRecognition._bowVocabulary =
          _node["Loop"]["VisualPlaceRecognition"]["BowVocabulary"]
              .as<std::string>();
      _cfg._loop._visualPlaceRecognition._cudaDeviceId =
          _node["Loop"]["VisualPlaceRecognition"]["CudaDeviceId"]
              ? _node["Loop"]["VisualPlaceRecognition"]["CudaDeviceId"]
                    .as<int>()
              : 0;
      _cfg._loop._queryTopK = _node["Loop"]["QueryTopK"].as<int>();
      _cfg._loop._similarityScoreFactor =
          _node["Loop"]["SimilarityScoreFactor"].as<double>();
#endif  // VISUAL_PLACE_LOCALIZATION
    }

    // Interface settings
    {
      _cfg._interface._color._point = _node["Interface"]["Color"]["Point"]
                                          .as<std::array<unsigned int, 3>>();
      _cfg._interface._color._inlier = _node["Interface"]["Color"]["Inlier"]
                                           .as<std::array<unsigned int, 3>>();
      _cfg._interface._color._allPoints =
          _node["Interface"]["Color"]["AllPoints"]
              .as<std::array<unsigned int, 3>>();
      _cfg._interface._color._trackingFeatures =
          _node["Interface"]["Color"]["TrackingFeatures"]
              .as<std::array<unsigned int, 3>>();
      _cfg._interface._color._text =
          _node["Interface"]["Color"]["Text"].as<std::array<unsigned int, 3>>();

      _cfg._interface._size._thickness =
          _node["Interface"]["Size"]["Thickness"].as<int>();

      _cfg._interface._points._sparseSize =
          _node["Interface"]["Points"]["SparseSize"].as<float>();
      _cfg._interface._points._denseSize =
          _node["Interface"]["Points"]["DenseSize"].as<float>();

      _cfg._interface._cells._thickness =
          _node["Interface"]["Cells"]["Thickness"].as<float>();
      _cfg._interface._cells._thicknessHighlighted =
          _node["Interface"]["Cells"]["HighlightedThickness"].as<float>();
      _cfg._interface._cells._color = _node["Interface"]["Cells"]["Color"]
                                          .as<std::array<unsigned int, 4>>();
      _cfg._interface._cells._colorHighlighted =
          _node["Interface"]["Cells"]["HighlightedColor"]
              .as<std::array<unsigned int, 4>>();

      _cfg._interface._covisibility._thickness =
          _node["Interface"]["Covisibility"]["Thickness"].as<float>();
      _cfg._interface._covisibility._color =
          _node["Interface"]["Covisibility"]["Color"]
              .as<std::array<unsigned int, 4>>();
    }

  } catch (const std::out_of_range& e) {
    __throw__("Invalid setting in YAML file: " + std::string(e.what()));
  } catch (const YAML::Exception& e) {
    __throw__("YAML parsing error: " + std::string(e.what()));
  }

  // Validation
  if (_cfg._slam._features._type == modes::FeatureType::SIFT) {
#ifndef WITH_CUDA
    __throw__("To use SIFT, compile with WITH_CUDA=ON");
#endif  // WITH_CUDA
  } else if (_cfg._slam._features._type == modes::FeatureType::ALIKED) {
#ifndef WITH_ALIKED
    __throw__("To use ALIKED, compile with WITH_ALIKED=ON");
#endif  // WITH_ALIKED
  }

#ifdef LOG_SETTINGS
  // print all settings
  log::message("Settings...", log::SETTINGS);
  auto settingsList = yamlToStringList(_node);
  for (const auto& setting : settingsList) {
    log::message("  -> " + setting, log::SETTINGS);
  }
#endif  // LOG_SETTINGS

  return;
}

}  // namespace slam_mer
