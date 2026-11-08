/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file slam.cpp
/// @brief Implements the main SLAM pipeline facade (`slam_mer::Slam`).
/// @author Pedro Miraldo and Valter Piedade
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki
/// @author Lalit Manam

#include <print>
#include <slam.hpp>

#include <localization.hpp>
#include <thread>

// Thirdparty headers
// Add the gtsam headers for pose definitions
// Maybe there is a better way to do this, but for now we need to include
// them here to avoid circular dependencies
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>

#if !defined(SLAM_MER_COMPILE_POSE3) && !defined(SLAM_MER_COMPILE_SIMILARITY3)
#error "At least one SLAM transformation instantiation must be compiled"
#endif

#ifdef VISUAL_PLACE_LOCALIZATION
#endif

// Standard C++ headers

namespace slam_mer {

namespace settings {
slam_mer::Settings::shared_ptr sharedSettings =
    slam_mer::Settings::make_shared();
}

namespace {
// ---------------------------------------------------------------------------
// Global atributes
/// Keep track of the latest input image
image_t __latestImage;
/// Keep track of the latest gray scaled image
image_t __latestGrayImage;
/// Keep track of the latest input depth image
image_t __latestDepthImage;
/// timer_t for measuring the time of the SLAM pipeline
timer_t __timerImageAdd;
/// Frames per second (default 10)
double __localizationFps;
/// Pointer to the MAP, and all it's functionalities
map::Map::shared_ptr __map;

// Pointer to the localization module
std::variant<
    localization::Localization<gtsam::Pose3>::shared_ptr,
    localization::Localization<gtsam::Similarity3>::shared_ptr>
    __localization;

/// Pointer to the covisibility graph
std::variant<
    covisibility_graph::CovisibilityGraph<gtsam::Pose3>::shared_ptr,
    covisibility_graph::CovisibilityGraph<gtsam::Similarity3>::shared_ptr>
    __covisibilityGraph;

/// Keeps track of the sensor data used to create keyframes
hash_t<keyframe_id_t, data_type::SensorData::shared_ptr> __sensorDataKeyframes;

#ifdef VISUAL_PLACE_LOCALIZATION
/// Keyframe Database
vpl::visual_place_recognition::VisualPlaceRecognition::shared_ptr
    __visualPlaceRecognition;
/// Pointer to the loop detection module
std::variant<
    loop_detection::LoopDetection<gtsam::Pose3>::shared_ptr,
    loop_detection::LoopDetection<gtsam::Similarity3>::shared_ptr>
    __loopDetection;

#endif  // end VISUAL_PLACE_LOCALIZATION

}  // namespace

std::shared_ptr<const covisibility_graph::GaussianPosterior>
Slam::get_posterior() const {
  return std::visit(
      [](const auto& graph_)
          -> covisibility_graph::GaussianPosterior::shared_ptr {
        return graph_ ? graph_->posterior() : nullptr;
      },
      __covisibilityGraph);
}

// ---------------------------------------------------------------------------
// Construct SLAM system and initialize all runtime modules
// ---------------------------------------------------------------------------
Slam::Slam(const std::string& settingsPath_) {
  using map::Map;
  using slam_mer::settings::sharedSettings;
  using std::jthread;
  using std::println;
#ifdef VISUAL_PLACE_LOCALIZATION
  using vpl::visual_place_recognition::VisualPlaceRecognition;
#endif

  // When creating the SLAM system, we need to do the following steps:
  // 1. Load settings
  // 2. Instantiate the map
  // 3. Instantiate the database for visual place recognition (if enabled)
  // 4. Instantiate the adjustment module (covisibility graph)
  // 5. Launch the map adjustment thread
  // 6. Instantiate the loop detection module (if VPL enabled)
  // 7. Launch the loop detection thread (if VPL enabled)
  // 8. Instantiate the localization module

  // Load settings
  if (!settingsPath_.empty()) sharedSettings->load(settingsPath_);

  // Instantiate the map
  __map = Map::make_shared();

  // Instantiate the database for visual place recognition
#ifdef VISUAL_PLACE_LOCALIZATION
  VisualPlaceRecognition::Config config;
  config._method = sharedSettings->visual_place_recognition_method();
  config._cudaDeviceId =
      sharedSettings->visual_place_recognition_cuda_device_id();
  __visualPlaceRecognition = VisualPlaceRecognition::create(config);
#endif

  const bool useSimilarity = sharedSettings->covisibility_use_similarity();
#if defined(SLAM_MER_COMPILE_POSE3) && defined(SLAM_MER_COMPILE_SIMILARITY3)
  if (useSimilarity)
    _type = transformation_t::Similarity3;
  else
    _type = transformation_t::Pose3;
#elif defined(SLAM_MER_COMPILE_SIMILARITY3)
  if (!useSimilarity) {
    __throw__(
        "[SLAM] SLAM_TRANSFORMATION_INSTANTIATIONS=SIMILARITY3 "
        "but settings request SE(3)");
  }
  _type = transformation_t::Similarity3;
#elif defined(SLAM_MER_COMPILE_POSE3)
  if (useSimilarity) {
    __throw__(
        "[SLAM] SLAM_TRANSFORMATION_INSTANTIATIONS=POSE3 "
        "but settings request SIM(3)");
  }
  _type = transformation_t::Pose3;
#endif

  switch (_type) {
#ifdef SLAM_MER_COMPILE_POSE3
  case transformation_t::Pose3:
    println("[SLAM] Using SE(3) for covisibility");
    __instanciate_modules<gtsam::Pose3>();
    break;
#endif
#ifdef SLAM_MER_COMPILE_SIMILARITY3
  case transformation_t::Similarity3:
    println("[SLAM] Using SIM(3) for covisibility");
    __instanciate_modules<gtsam::Similarity3>();
    break;
#endif
  default:
    __throw__("[SLAM] Using UNDEFINED transformation type for covisibility");
  }
}

// ---------------------------------------------------------------------------
// Instanciate the SLAM modules (localization, covisibility graph, and
// loop detection) based on the transformation type
// ---------------------------------------------------------------------------
template <typename transformation_t>
void Slam::__instanciate_modules() {
  using covisibility_graph::CovisibilityGraph;
  using CovisibilityGraphType = CovisibilityGraph<transformation_t>;
  using CovisibilityGraphPtr = CovisibilityGraphType::shared_ptr;
  using localization::Localization;
  using LocalizationType = Localization<transformation_t>;
#ifdef VISUAL_PLACE_LOCALIZATION
  using loop_detection::LoopDetection;
  using LoopDetectionType = LoopDetection<transformation_t>;
  using vpl::visual_place_recognition::VisualPlaceRecognition;
#endif

  // Instantiate the loop detection module
  __covisibilityGraph = CovisibilityGraphType::make_shared(__map);
  auto& graph = std::get<CovisibilityGraphPtr>(__covisibilityGraph);
  // Instantiate the loop detection module
#ifdef VISUAL_PLACE_LOCALIZATION
  __loopDetection =
      LoopDetectionType::make_shared(__map, graph, __visualPlaceRecognition);
#endif
  // Instantiate the localization module
  __localization = LocalizationType::make_shared(__map, graph);
#ifdef VISUAL_PLACE_LOCALIZATION
  using LocalizationPtr = LocalizationType::shared_ptr;
  // Note that __visualPlaceRecognition is sent as weak_ptr
  auto& localization = std::get<LocalizationPtr>(__localization);
  localization->set_keyframe_database(__visualPlaceRecognition);
#endif
}

// ---------------------------------------------------------------------------
// Shutdown SLAM system and release all runtime modules
// ---------------------------------------------------------------------------
Slam::~Slam() {
  using std::println;

  println("[SLAM] Sending stop signal...");
  switch (_type) {
#ifdef SLAM_MER_COMPILE_POSE3
  case transformation_t::Pose3:
    println("[SLAM] Using SE(3) for covisibility");
    __terminate_modules<gtsam::Pose3>();
    break;
#endif
#ifdef SLAM_MER_COMPILE_SIMILARITY3
  case transformation_t::Similarity3:
    println("[SLAM] Using SIM(3) for covisibility");
    __terminate_modules<gtsam::Similarity3>();
    break;
#endif
  default:
    __throw__("[SLAM] Using UNDEFINED transformation type for covisibility");
  }

  println("[SLAM] Stopped successfully.");
}

// ---------------------------------------------------------------------------
// Instanciate the SLAM modules (localization, covisibility graph, and
// loop detection) based on the transformation type
// ---------------------------------------------------------------------------
template <typename transformation_t>
void Slam::__terminate_modules() {
  using covisibility_graph::CovisibilityGraph;
  using CovisibilityGraphType = CovisibilityGraph<transformation_t>;
  using CovisibilityGraphPtr = CovisibilityGraphType::shared_ptr;
  ;
  using localization::Localization;
  using LocalizationType = Localization<transformation_t>;
  using LocalizationPtr = LocalizationType::shared_ptr;
#ifdef VISUAL_PLACE_LOCALIZATION
  using loop_detection::LoopDetection;
  using LoopDetectionType = LoopDetection<transformation_t>;
  using LoopDetectionPtr = LoopDetectionType::shared_ptr;
  using vpl::visual_place_recognition::VisualPlaceRecognition;
#endif

  // 1. Stop Localization
  // 2. Stop Loop detection (if enabled)
  // 3. Stop covisibility graph and map optimization
  // 4. Cleanup memory
  // 5. Stops the map

  // 1 -------------------------------------
  auto& localization = std::get<LocalizationPtr>(__localization);
  localization.reset();

  // 2 -------------------------------------
#ifdef VISUAL_PLACE_LOCALIZATION
  auto& loop = std::get<LoopDetectionPtr>(__loopDetection);
  loop.reset();
#endif

  // 3 -------------------------------------
  auto& graph = std::get<CovisibilityGraphPtr>(__covisibilityGraph);
  graph.reset();

  // 4 -------------------------------------
#ifdef VISUAL_PLACE_LOCALIZATION
  __visualPlaceRecognition->shutdown();
#endif

  // 5 -------------------------------------
  __map.reset();
}

// ---------------------------------------------------------------------------
// Add a new sensor frame to the SLAM pipeline
// ---------------------------------------------------------------------------
void Slam::add(data_type::SensorData::weak_ptr data_) {
  switch (_type) {
#ifdef SLAM_MER_COMPILE_POSE3
  case transformation_t::Pose3:
    __add<gtsam::Pose3>(data_);
    break;
#endif
#ifdef SLAM_MER_COMPILE_SIMILARITY3
  case transformation_t::Similarity3:
    __add<gtsam::Similarity3>(data_);
    break;
#endif
  default:
    __throw__("[SLAM] Using UNDEFINED transformation type for covisibility");
  }
}

// ---------------------------------------------------------------------------
// Add a new sensor frame to the SLAM pipeline
// ---------------------------------------------------------------------------
template <typename transformation_t>
void Slam::__add(data_type::SensorData::weak_ptr data_) {
  using localization::Localization;
  using LocalizationType = Localization<transformation_t>;
  using LocalizationPtr = LocalizationType::shared_ptr;

  // Get the localization module based on the transformation type
  auto& localization = std::get<LocalizationPtr>(__localization);

  // Add the new sensor data to the SLAM system
  // 1 - Pass the sensor data to the localization module
  // 2 - Counts the time taken to process the sensor data

  __timerImageAdd.tic();
  localization->run(data_);
  const duration_t elapsedTime = __timerImageAdd.toc();
  const double computeTime = elapsedTime.fps();
  __localizationFps = computeTime;
}

// ---------------------------------------------------------------------------
// Get current camera pose estimate from localization
// ---------------------------------------------------------------------------
pair_t<rotation_t, translation_t> Slam::get_camera_pose() const {
  switch (_type) {
#ifdef SLAM_MER_COMPILE_POSE3
  case transformation_t::Pose3:
    return __get_camera_pose<gtsam::Pose3>();
#endif
#ifdef SLAM_MER_COMPILE_SIMILARITY3
  case transformation_t::Similarity3:
    return __get_camera_pose<gtsam::Similarity3>();
#endif
  default:
    __throw__("[SLAM] Using UNDEFINED transformation type for covisibility");
  }
}

// ---------------------------------------------------------------------------
// Get current camera pose estimate from localization
// ---------------------------------------------------------------------------
template <typename transformation_t>
pair_t<rotation_t, translation_t> Slam::__get_camera_pose() const {
  using localization::Localization;
  using LocalizationType = Localization<transformation_t>;
  using LocalizationPtr = LocalizationType::shared_ptr;

  // Get the localization module based on the transformation type
  auto& localization = std::get<LocalizationPtr>(__localization);
  const pose_t pose = localization->get_camera_world_pose();
  const auto [rotation, translation] = pose.get_rotation_translation();
  return {rotation, translation};
}

// ---------------------------------------------------------------------------
// Get current map size counters
// ---------------------------------------------------------------------------
std::tuple<size_t, size_t, size_t, size_t, size_t> Slam::get_map_size() const {
  return __map->size();
}

// ---------------------------------------------------------------------------
// Collect image/debug visualization data for external consumers
// ---------------------------------------------------------------------------
void Slam::get_image_visualization_data(
    data_type::SensorData::shared_ptr data_,
    utils::ImageDebugData& outputData_) {

  switch (_type) {
#ifdef SLAM_MER_COMPILE_POSE3
  case transformation_t::Pose3:
    __get_image_visualization_data<gtsam::Pose3>(data_, outputData_);
    break;
#endif
#ifdef SLAM_MER_COMPILE_SIMILARITY3
  case transformation_t::Similarity3:
    __get_image_visualization_data<gtsam::Similarity3>(data_, outputData_);
    break;
#endif
  default:
    __throw__("[SLAM] Using UNDEFINED transformation type for covisibility");
  }
}

// ---------------------------------------------------------------------------
// Collect image/debug visualization data for external consumers
// ---------------------------------------------------------------------------
template <typename transformation_t>
void Slam::__get_image_visualization_data(
    data_type::SensorData::shared_ptr data_,
    utils::ImageDebugData& outputData_) {
  using localization::Localization;
  using LocalizationType = Localization<transformation_t>;
  using LocalizationPtr = LocalizationType::shared_ptr;

  // Get the localization module based on the transformation type
  auto& localization = std::get<LocalizationPtr>(__localization);

  outputData_._depthImage = data_->get_depth_image().clone();
  outputData_._image = data_->get_color_image().clone();

  if (localization->is_lost()) {
    log::error("Entering Relocalization");
  }

#ifdef DEBUG
  if (localization->is_lost()) {
    outputData_._allFeatures = localization->get_all_frame_keypoints();
  } else {
    outputData_._allFeatures = localization->get_all_frame_keypoints();
    outputData_._inliers = localization->get_inlier_keypoints();
  }
  outputData_._grayscaleImage = data_->get_grayscale_image().clone();
  outputData_._fps = floor(__localizationFps);
  const auto
      [numKeyframes,
       numMappoints,
       numQueryCells,
       numClosestCells,
       numTotalCells] = __map->size();
  outputData_._numMappoints = numMappoints;
  outputData_._numKeyframes = numKeyframes;
  outputData_._numCells = numTotalCells;
  outputData_._numQueryCells = numQueryCells;
  outputData_._numClosestCells = numClosestCells;
#endif
}

// ---------------------------------------------------------------------------
// Get current SLAM fps
// ---------------------------------------------------------------------------
double Slam::get_frames_per_second() {
  return __localizationFps;
}

// ---------------------------------------------------------------------------
// Add IMU sample (placeholder path while IMU integration is pending)
// ---------------------------------------------------------------------------
void Slam::add(std::vector<double> imu_) {
#ifdef LOG_SLAM
  log::warning("IMU pre-integration not implemented yet");
#endif
  return;
}

}  // namespace slam_mer
