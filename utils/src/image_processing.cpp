/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file image_processing.cpp
/// @brief Image-processing implementation for feature pipeline operations.
/// @details Implements feature extraction, matching, and filtering routines
/// declared in image_processing.hpp. This file contains core front-end
/// processing logic.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <image_processing.hpp>
#include <settings.hpp>
#include <types/timer.hpp>

#ifdef WITH_ALIKED
#include <lightglue/feature/aliked.hpp>
#endif

// Thirdpary includes
#include <faiss/gpu/GpuIndexFlat.h>
#include <faiss/gpu/StandardGpuResources.h>

// Standard library includes
#include <cfloat>

#ifdef WITH_CUDA
#include <cudaSift.h>
#endif

namespace slam_mer {
namespace image_processing {

using FaissIndexType = faiss::gpu::GpuIndexFlatL2;

class FaissGpuResourcePool {
public:
  /// Number of pooled GPU resource/index slots.
  static constexpr int _resource_num = 4;
  /// @brief Returns the singleton pool instance.
  /// @return Reference to the global FAISS GPU resource pool.
  static FaissGpuResourcePool& instance() {
    static FaissGpuResourcePool pool;
    return pool;
  }

  /// @brief Initializes pooled FAISS GPU resources for a descriptor dimension.
  /// @param dimension_ Descriptor dimension to configure for pooled indices.
  /// @param cudaDeviceId_ CUDA device identifier for FAISS indices.
  /// @return Pointer to this pool instance.
  FaissGpuResourcePool* initialize(int dimension_, int cudaDeviceId_) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_initialized && _dim == dimension_ && _device == cudaDeviceId_)
      return this;

    for (int i = 0; i < _resource_num; ++i)
      _gpu_indices[i].reset();

    for (int i = 0; i < _resource_num; ++i) {
      if (!_gpu_resources[i]) {
        _gpu_resources[i] =
            std::make_unique<faiss::gpu::StandardGpuResources>();
      }
      if (!_gpu_indices[i]) {
        faiss::gpu::GpuIndexFlatConfig config;
        config.device = cudaDeviceId_;
        _gpu_indices[i] = std::make_unique<FaissIndexType>(
            _gpu_resources[i].get(),
            dimension_,
            config);
      }
      _in_use[i] = false;
    }
    _dim = dimension_;
    _device = cudaDeviceId_;
    _initialized = true;
    return this;
  }

  /// @brief Acquires one free pool slot.
  /// @return Slot index, or `-1` when all slots are busy.
  int acquire() {
    std::lock_guard<std::mutex> lock(_mutex);
    for (int i = 0; i < _resource_num; ++i) {
      if (!_in_use[i]) {
        _in_use[i] = true;
        return i;
      }
    }
    return -1;
  }

  /// @brief Releases a previously acquired pool slot.
  /// @param idx_ Slot index to release.
  void release(int idx_) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (idx_ >= 0 && idx_ < _resource_num) _in_use[idx_] = false;
  }
  /// @brief Returns the FAISS GPU index associated with a slot.
  /// @param idx_ Slot index.
  /// @return Pointer to the FAISS index in the slot.
  FaissIndexType* get_index(int idx_) { return _gpu_indices[idx_].get(); }
  /// @brief Returns the descriptor dimension currently configured in the pool.
  /// @return Descriptor dimension.
  int dim() const { return _dim; }

private:
  /// @brief Constructs an empty pool (singleton internal use).
  FaissGpuResourcePool()
      : _in_use{}, _dim(0), _device(0), _initialized(false) {}
  /// @brief Destroys indices/resources in safe order.
  ~FaissGpuResourcePool() {
    for (int i = 0; i < _resource_num; ++i) {
      _gpu_indices[i].reset();
      _gpu_resources[i].reset();
    }
  }
  /// Deleted copy constructor (singleton).
  FaissGpuResourcePool(const FaissGpuResourcePool&) = delete;
  /// Deleted copy assignment operator (singleton).
  FaissGpuResourcePool& operator=(const FaissGpuResourcePool&) = delete;

  /// Pooled FAISS GPU resources.
  std::array<std::unique_ptr<faiss::gpu::StandardGpuResources>, _resource_num>
      _gpu_resources;
  /// Pooled FAISS L2 indices.
  std::array<std::unique_ptr<FaissIndexType>, _resource_num> _gpu_indices;
  /// Slot usage flags.
  std::array<bool, _resource_num> _in_use;
  /// Active descriptor dimension.
  int _dim;
  /// Active CUDA device identifier.
  int _device;
  /// Whether the pool has been initialized for dim_.
  bool _initialized;
  /// Mutex protecting pool state.
  std::mutex _mutex;
};

class FaissGpuResourceHandle {
public:
  /// @brief Acquires a FAISS GPU slot for the requested descriptor dimension.
  /// @param dim_ Descriptor dimension expected by the caller.
  /// @param cudaDeviceId_ CUDA device identifier for FAISS indices.
  FaissGpuResourceHandle(int dim_, int cudaDeviceId_)
      : _pool(FaissGpuResourcePool::instance()), _idx(-1) {
    // Ensure pool is initialized for this dimension
    _idx = _pool.initialize(dim_, cudaDeviceId_)->acquire();
    if (_idx < 0) {
      __throw__("No free FAISS GPU resources available");
    }
    // Always reset index before use
    _pool.get_index(_idx)->reset();
  }
  /// @brief Releases the acquired slot back to the pool.
  ~FaissGpuResourceHandle() { _pool.release(_idx); }

  /// @brief Returns the currently acquired FAISS GPU index.
  /// @return Pointer to the active FAISS index.
  FaissIndexType* index() { return _pool.get_index(_idx); }

private:
  /// Reference to the global FAISS resource pool.
  FaissGpuResourcePool& _pool;
  /// Acquired slot index.
  int _idx;
};

#ifdef WITH_ALIKED
std::shared_ptr<::lightglue::feature::ALIKED> aliked_model = nullptr;
#endif  // WITH_ALIKED

#ifdef WITH_CUDA

/// @brief This a a version of CUDA SIFT
/// @param image_ Original image.
/// @param keypoints_ Output vector of detected keypoints.
/// @param descriptors_ Output vector of corresponding descriptors.
void __compute_cuda_sift_features(
    const image_t& image_,
    point2d_t::vector& keypoints_,
    descriptor_t::vector& descriptors_) {
  using slam_mer::settings::sharedSettings;

  // Reserve memory space for a whole bunch of SIFT features.
  SiftData siftData;
  int maxKeypoints = sharedSettings->maximum_features();
  InitSiftData(siftData, maxKeypoints, true, true);

  // Convert image to floating point.
  image_t imgFloat = image_.convert_to(image_t::Type::F32);

  CudaImage img;
  const size_t rows = image_.rows();
  const size_t cols = image_.cols();
  const int alignUp = iAlignUp(cols, 128);
  float* iamgeData = imgFloat.ptr<float>();
  img.Allocate(cols, rows, alignUp, false, NULL, iamgeData);
  // Download image from host to device
  img.Download();

  // Extract SIFT features
  ExtractSift(
      siftData,
      img,
      sharedSettings->sift_num_octaves(),
      sharedSettings->sift_init_blur(),
      sharedSettings->sift_peak_thresh(),
      sharedSettings->sift_min_scale(),
      sharedSettings->sift_up_scale());
  keypoints_.reserve(siftData.numPts);
  descriptors_.reserve(siftData.numPts);
  // #pragma omp parallel for
  for (int i = 0; i < siftData.numPts; i++) {
    // Convert SiftData to point2d_t
    keypoints_.emplace_back(
        point2d_t(siftData.h_data[i].xpos, siftData.h_data[i].ypos));
    // Convert SiftData to descriptor_t
    std::vector<float> data(
        siftData.h_data[i].data,
        siftData.h_data[i].data + 128);
    descriptors_.emplace_back(descriptor_t(data));
  }

  auto saturate_cast = [](auto value__, auto out_type_tag__) {
    using T = decltype(out_type_tag__);
    return static_cast<T>(std::clamp(
        value__,
        static_cast<decltype(value__)>(std::numeric_limits<T>::lowest()),
        static_cast<decltype(value__)>(std::numeric_limits<T>::max())));
  };

  // Normalization similar to descriptors
  int len = 128;
  for (int i = 0; i < siftData.numPts; i++) {
    float nrm2 = 0.0;
    std::vector<float> descriptorData = descriptors_[i].get_data();
    for (int k = 0; k < len; k++) {
      nrm2 += descriptorData[k] * descriptorData[k];
    }
    float thr = std::sqrt(nrm2) * 0.2f;
    nrm2 = 0.0;
    for (int j = 0; j < len; j++) {
      float val = std::min(descriptorData[j], thr);
      descriptorData[j] = val;
      nrm2 += val * val;
    }
    nrm2 = 512.f / std::max(std::sqrt(nrm2), FLT_EPSILON);
    for (int k = 0; k < len; k++) {
      descriptorData[k] = saturate_cast(descriptorData[k] * nrm2, uint8_t());
    }
    descriptors_[i] = descriptor_t(descriptorData);
  }
  FreeSiftData(siftData);
}
#endif  //WITH_CUDA

#ifdef WITH_ALIKED

/// @brief Extract ALIKED features using the ALIKED neural network
/// @param image_ Original image.
/// @param keypoints_ Output vector of detected keypoints.
/// @param descriptors_ Output vector of corresponding descriptors.
void __compute_aliked_features(
    const image_t& image_,
    point2d_t::vector& keypoints_,
    descriptor_t::vector& descriptors_) {
  using slam_mer::settings::sharedSettings;

  if (aliked_model == nullptr) {
    const std::string device =
        "cuda:" + std::to_string(sharedSettings->aliked_cuda_device_id());

    aliked_model = std::make_shared<::lightglue::feature::ALIKED>(
        sharedSettings->aliked_model_name(),
        device,
        sharedSettings->maximum_features(),
        sharedSettings->aliked_scores_th(),
        sharedSettings->maximum_features());
    aliked_model->eval();
  }

  if (image_.empty()) {
    log::error(
        "Input image is empty for ALIKED feature extraction",
        log::IMAGE_PROCESSING);
    exit(1);
  }

  torch::Tensor kpts_cpu, desc_cpu;
  {
    torch::NoGradGuard no_grad;
    // cv::Mat imgRgb = slam_mer::as_cv_mat(image_);
    // const auto& pred = aliked_model->run(imgRgb.data, imgRgb.rows, imgRgb.cols);
    const auto& pred =
        aliked_model->run(image_.data(), image_.rows(), image_.cols());
    kpts_cpu = pred.at("keypoints").contiguous().cpu().clone();
    desc_cpu = pred.at("descriptors").contiguous().cpu().clone();
  }

  if (kpts_cpu.dim() < 2 || kpts_cpu.size(0) == 0) {
    log::warning(
        "ALIKED detected no keypoints in the image",
        log::IMAGE_PROCESSING);
    exit(1);
  }

  // Convert to KeyPoints
  const int n_kpts = kpts_cpu.size(0);
  keypoints_.reserve(n_kpts);
  const float* kpts_data = kpts_cpu.data_ptr<float>();
  for (int i = 0; i < n_kpts; ++i) {
    keypoints_.emplace_back(point2d_t(kpts_data[i * 2], kpts_data[i * 2 + 1]));
  }

  // Convert to descriptors
  const int desc_rows = desc_cpu.size(0);
  const int desc_cols = desc_cpu.size(1);
  float* ptr = desc_cpu.data_ptr<float>();
  descriptors_.reserve(desc_rows);

  for (int r = 0; r < desc_rows; r++) {
    std::vector<float> descriptorsData(
        ptr + r * desc_cols,
        ptr + (r + 1) * desc_cols);
    descriptors_.emplace_back(descriptor_t(descriptorsData));
  }
  return;
}
#endif  // WITH_ALIKED

/// @brief Some heuristic function for removing points from the
/// borders of the image.
/// @param keypoints_ Image keypoints.
/// @param descriptors_ Image descriptors.
/// @param rows_ Number of rows in the image.
/// @param cols_ Number of columns in the image.
void __remove_points_from_image_borders(
    point2d_t::vector& keypoints_,
    descriptor_t::vector& descriptors_,
    int rows_,
    int cols_) {
  using slam_mer::settings::sharedSettings;

  // Define the area to be considered in the image where
  // features can be extracted and used for localization
  const int minHeight = sharedSettings->border_cut_height();
  const int maxHeight = static_cast<int>(rows_ - minHeight);
  const int minWidth = sharedSettings->border_cut_width();
  const int maxWidth = static_cast<int>(cols_ - minWidth);

  // go over the stored keypoints, keeping only the ones inside the
  // considered area
  point2d_t::vector subImageKeyPoints;
  descriptor_t::vector subImageDescriptors;
  for (size_t i = 0; i < keypoints_.size(); i++) {
    std::array<double, 2> point2d = keypoints_[i].to_2d_array();
    // Check x axis
    if (static_cast<int>(point2d[0]) > minWidth &&
        static_cast<int>(point2d[0]) < maxWidth) {
      // Check y axis
      if (static_cast<int>(point2d[1]) > minHeight &&
          static_cast<int>(point2d[1]) < maxHeight) {
        subImageKeyPoints.push_back(keypoints_[i]);
        subImageDescriptors.push_back(descriptors_[i]);
      }
    }
  }
  keypoints_ = subImageKeyPoints;
  descriptors_ = subImageDescriptors;
  return;
}

// ------------------------------------------------------------------------
// Detect features in an image
// ------------------------------------------------------------------------
void feature_detector(
    const image_t& image_,
    point2d_t::vector& keypoints_,
    descriptor_t::vector& descriptors_) {
  using modes::FeatureType;
  using slam_mer::settings::sharedSettings;

  // This function is used to extract features from an image. It is used in
  // the localization module and others to extract features from the images.
  // We have multiple options. For now, we are using SIFT (namely CudaSIFT)
  // and ALIKED. Other options can be added.

  // Extracts features and compute descriptors
  switch (sharedSettings->feature_type()) {

  // For SIFT, we have one option: CudaSIFT (GPU). This is implemented
  // using a CMAKE_WITH_CUDA flag, so the compiler only includes the
  // necessary code.
  case FeatureType::SIFT: {

#ifdef WITH_CUDA
    // If we are using CUDA, we use the GPU implementation.
    __compute_cuda_sift_features(image_, keypoints_, descriptors_);
#else
    __throw__("CudaSIFT requires the compile option WITH_CUDA=ON");
#endif  // WITH_CUDA

    // Apply RootSIFT
    if (sharedSettings->apply_rootsift()) {
      // Perform L1 normalization on each descriptor
      for (size_t i = 0; i < descriptors_.size(); ++i) {
        descriptors_[i].normalize_l1();
        descriptors_[i].sqrt();
      }
    }
    break;
  }

  // ALIKED is a deep learning-based feature detector that provides
  // high accuracy and real-time performance with GPU acceleration.
  case FeatureType::ALIKED: {

#ifdef WITH_ALIKED
    __compute_aliked_features(image_, keypoints_, descriptors_);
#else
    __throw__("ALIKED requires the compile option WITH_ALIKED=ON");
#endif  // WITH_ALIKED

    break;
  }

  default:
    __throw__("Invalid Feature Type");
  }

  // For many reasons that include a short localization and the fact that
  // these are less accurate, we dont want to use features that are very
  // to the borders of the image. The following function removes them.
  const size_t rows = image_.rows();
  const size_t cols = image_.cols();
  __remove_points_from_image_borders(keypoints_, descriptors_, rows, cols);
  return;
}

// ------------------------------------------------------------------------
// Brute Force Matching
// ------------------------------------------------------------------------

/// @brief Matches descriptors from two images using FAISS GPU brute-force.
/// @param descriptors1_ Descriptors from the first image.
/// @param descriptors2_ Descriptors from the second image.
/// @param matchMap_ Output map of matches (index in descriptors1_ to index in
/// descriptors2_).
void __feature_matcher_faiss(
    const descriptor_t::vector& descriptors1_,
    const descriptor_t::vector& descriptors2_,
    std::map<unsigned int, unsigned int>& matchMap_) {
  timer_t timer;

  timer.tic();
  using slam_mer::settings::sharedSettings;

  const int dim = descriptors2_.at(0).get_size();
  const int cudaDeviceId = sharedSettings->faiss_cuda_device_id();

  // Ensure pool is initialized and acquire a resource
  FaissGpuResourceHandle handle(dim, cudaDeviceId);
  FaissIndexType* faiss_index = handle.index();
  // Add database descriptors to index
  const std::vector<float>& flattenDescriptors2 =
      descriptor_t::flatten_descriptors(descriptors2_);

  const std::vector<float>& flattenDescriptors1 =
      descriptor_t::flatten_descriptors(descriptors1_);

  faiss_index->add(descriptors2_.size(), flattenDescriptors2.data());
  int k = 1;  // get top-1 match
  std::vector<faiss::idx_t> indices(descriptors1_.size() * k);
  std::vector<float> distances(descriptors1_.size() * k);
  faiss_index->search(
      descriptors1_.size(),
      flattenDescriptors1.data(),
      k,
      distances.data(),
      indices.data());
  // Convert to DMatch format if you want
  for (size_t i = 0; i < descriptors1_.size(); i++) {
    matchMap_[i] = indices[i];
  }
}

// ------------------------------------------------------------------------
// Get intersection between matches from image 1 to 2 and 2 to 1
// ------------------------------------------------------------------------

/// @brief Computes the intersection of matches from image 1 to 2 and 2 to 1.
/// @param matchesOneTwo_ Matches from image 1 to 2.
/// @param matchesTwoOne_ Matches from image 2 to 1.
/// @param matches_ Output vector of intersecting matches.
void __set_match_intersection(
    const std::map<unsigned int, unsigned int>& matchesOneTwo_,
    const std::map<unsigned int, unsigned int>& matchesTwoOne_,
    match_t<index_t>::vector& matches_) {

  // Make sure that the matches vector is empty
  if (!matches_.empty()) matches_.clear();
  // reserve memory
  matches_.reserve(std::min(matchesOneTwo_.size(), matchesTwoOne_.size()));

  // Save only intersecting matches
  for (const auto& match : matchesOneTwo_) {
    std::map<unsigned int, unsigned int>::const_iterator it =
        matchesTwoOne_.find(match.second);
    if (it != matchesTwoOne_.end() && it->second == match.first) {
      matches_.emplace_back(
          match_t<index_t>(index_t(match.first), index_t(match.second)));
    }
  }
  return;
}

// ------------------------------------------------------------------------
// Matches descriptors from two images
// ------------------------------------------------------------------------
match_t<index_t>::vector feature_matcher_knn(
    const descriptor_t::vector& descriptors_,
    const descriptor_t::vector& descriptorsTemplate_) {

  std::map<unsigned int, unsigned int> matchesOneTwo;
  std::map<unsigned int, unsigned int> matchesTwoOne;

  // We compute matches from image A to B and B to A in parallel
#pragma omp parallel sections
  {
#pragma omp section
    __feature_matcher_faiss(descriptors_, descriptorsTemplate_, matchesOneTwo);

#pragma omp section
    __feature_matcher_faiss(descriptorsTemplate_, descriptors_, matchesTwoOne);
  }

  // Only the intersecting matches are returned
  match_t<index_t>::vector matches;
  __set_match_intersection(matchesOneTwo, matchesTwoOne, matches);
  return matches;
}

// ----------------------------------------------------------------------------
// Computes image coverage ratio
// ----------------------------------------------------------------------------

/// @brief Computes the area of the convex hull formed by a set of 2D points.
/// @param points2d_ Current points on the image.
/// @return Area of the convex hull.
double __compute_convex_hull_area(const point2d_t::vector& points2d_) {
  using std::println;
  println(
      "[Image Processing] {}Warning{}: Convex hull area computation is not "
      "implemented yet.",
      yellow,
      reset);
  return 0.0;
}

/// @brief Computes image coverage ratio. Its the ratio between the
/// area of a hull created with two set of image keypoints.
/// @param points2d_1_ First set of points on the image.
/// @param points2d_2_ Second set of points on the image.
/// @return Coverage ratio = (area of the 1st set / area of the 2nd set)
double compute_coverage_ratio(
    const point2d_t::vector& points2d_1_,
    const point2d_t::vector& points2d_2_) {

  const double hullArea_1 = __compute_convex_hull_area(points2d_1_);
  const double hullArea_2 = __compute_convex_hull_area(points2d_2_);
  const double coverageRatio = hullArea_1 / hullArea_2;
  return coverageRatio;
}

// ----------------------------------------------------------------------------
// Count points per cell in a image grid
// ----------------------------------------------------------------------------
Eigen::MatrixXi count_points_per_cell(
    const point2d_t::vector& keypoints_,
    const array2Size_t& imageSize_,
    const int rows_,
    const int cols_) {

  Eigen::MatrixXi counts(rows_, cols_);
  counts.setZero();  // initialize counts to 0

  for (const point2d_t& keypoint : keypoints_) {
    const array2D_t pixel = keypoint.to_2d_array();
    int row = (pixel[1] * rows_) / imageSize_[1];
    int col = (pixel[0] * cols_) / imageSize_[0];
    row = std::clamp(row, 0, rows_ - 1);
    col = std::clamp(col, 0, cols_ - 1);
    counts(row, col) += 1;
  }
  return counts;
}

}  // namespace image_processing
}  // namespace slam_mer
