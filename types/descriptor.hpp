/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file descriptor.hpp
/// @brief Descriptor container utilities and retrieval validation helpers.
/// @details Defines descriptor storage and normalization-related operations
/// used in feature pipelines. Also provides typed retrieval and
/// fail-check helpers to keep error propagation explicit and
/// consistent.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_DESCRIPTOR_HPP
#define TYPE_DESCRIPTOR_HPP

// Our includes
#include <types/id.hpp>
#include <types/output.hpp>

// Standard includes
#include <cmath>
#include <print>
#include <vector>

namespace slam_mer {

/// @brief Represents a feature descriptor as a float vector.
///
/// The descriptor values can be normalized and converted to packed formats for
/// matching or model input.
class descriptor_t {

public:
  /// Typedef for a vector of descriptors
  using vector = std::vector<descriptor_t>;

  /// @brief Flattens descriptors into a contiguous float buffer.
  /// @param descriptors_ Descriptor list appended sequentially into one buffer.
  /// @return Flat vector containing all descriptor values in order.
  static std::vector<float> flatten_descriptors(
      const descriptor_t::vector& descriptors_) {
    std::vector<float> flat;
    for (const auto& descriptor : descriptors_) {
      const auto data = descriptor.get_data();
      flat.insert(flat.end(), data.begin(), data.end());
    }
    return flat;
  }

  /// @brief Convert a vector of descriptors to a
  ///   std::vector<std::vector<float>>.
  /// @param descriptors_ Descriptor list converted into nested float vectors.
  /// @return list of descriptors
  [[nodiscard("Converting descriptors to float vector.")]]
  static std::vector<std::vector<float>> to_float_vector(
      const descriptor_t::vector& descriptors_) {
    std::vector<std::vector<float>> result;
    result.reserve(descriptors_.size());
    for (const auto& descriptor : descriptors_) {
      result.push_back(descriptor.get_data());
    }
    return result;
  }

  /// @brief Check if a descriptor is empty (all elements are zero).
  /// @param descriptor_ Descriptor instance tested for stored values.
  /// @return true if all elements are zero, false otherwise.
  [[nodiscard("Checking if descriptor is empty.")]]
  static bool is_empty(const descriptor_t& descriptor_) {
    return descriptor_._data.empty();
  }

  /// Default constructor
  descriptor_t() {}

  /// @brief Construct a new Pointdescriptor_t object.
  /// @param data_ Descriptor values copied into internal storage.
  descriptor_t(const std::vector<float>& data_) : _data(data_) {}

  /// @brief Copy constructor
  /// @param other_ Descriptor instance copied into this object.
  descriptor_t(const descriptor_t& other_) { _data = other_._data; }

  /// @brief Construct a new descriptor_t object from a pointer to data
  /// @param data_ Pointer to the first float value of the descriptor buffer.
  /// @param size_ Number of float values read from `data_`.
  /// and its size.
  descriptor_t(float* data_, const size_t size_) {
    _data = std::vector<float>(data_, data_ + size_);
    return;
  }

  /// @brief Check if this descriptor is empty (all elements are zero).
  /// @return `true` when no descriptor values are stored.
  [[nodiscard("Checking if descriptor is empty.")]]
  inline bool is_empty() const {
    return _data.empty();
  }

  /// @brief Get the description object.
  /// @return with the descriptor data.
  [[nodiscard("Retrieving descriptor data.")]]
  const inline std::vector<float>& get_data() const {
    return _data;
  }

  /// @brief Get the size of the descriptor (number of elements).
  /// @return Number of float elements in this descriptor.
  [[nodiscard("Retrieving descriptor size.")]]
  inline size_t get_size() const {
    return _data.size();
  }

  /// @brief Normalize the descriptor using L2 norm.
  void normalize_l2() {
    const int descriptorSize = static_cast<int>(_data.size());
    float norm = 0.0f;

    for (int i = 0; i < descriptorSize; ++i)
      norm += _data[i] * _data[i];
    norm = std::sqrt(norm);
    for (int i = 0; i < descriptorSize; ++i)
      _data[i] /= norm;
    return;
  }

  /// @brief Normalize the descriptor using L1 norm.
  void normalize_l1() {
    const int descriptorSize = static_cast<int>(_data.size());
    float norm = 0.0f;
    for (int i = 0; i < descriptorSize; ++i)
      norm += _data[i];
    for (int i = 0; i < descriptorSize; ++i)
      _data[i] /= norm;
    return;
  }

  /// @brief Apply square root to each element of the descriptor.
  void sqrt() {
    using std::sqrt;
    const int descriptorSize = static_cast<int>(_data.size());
    for (int i = 0; i < descriptorSize; ++i)
      _data[i] = sqrt(_data[i]);
    return;
  }

  /// @brief Print the descriptor values to the console.
  void print() const {
    using std::print;
    print("descriptor_t: ");
    for (const auto& value : _data)
      print("{} ", value);
    print("\n");
  }

private:
  /// descriptor_t data, using a set of floats
  std::vector<float> _data;
};

// ----------------------------------------------------------------------------
// Sanity and failure checks for descriptors
// ----------------------------------------------------------------------------

/// @brief Validates descriptor retrieval for a keyframe and throws on failure.
/// @param descriptor__ Output wrapper containing the descriptor.
/// @param keyframeId__ Keyframe identifier used in the error message.
inline void __sanity_retrieve(
    const output_t<descriptor_t>& descriptor__,
    const keyframe_id_t& keyframeId__) {
  if (descriptor__ != fail_t::NO_FAIL)
    __throw__(
        "Keyframe " + std::string(keyframeId__) +
        " does not have a valid descriptor for loop detection.");
}

/// @brief Validates descriptor retrieval for a point and throws on failure.
/// @param descriptor__ Output wrapper containing the descriptor.
/// @param pointId__ Point identifier used in the error message.
inline void __sanity_retrieve(
    const output_t<descriptor_t>& descriptor__,
    const point_id_t& pointId__) {
  if (descriptor__ != fail_t::NO_FAIL)
    __throw__(
        "Point " + std::string(pointId__) +
        " does not have a valid descriptor for loop detection.");
}

/// @brief Validates descriptor retrieval and throws on failure.
/// @param descriptor__ Output wrapper containing the descriptor.
inline void __sanity_retrieve(const output_t<descriptor_t>& descriptor__) {
  if (descriptor__ != fail_t::NO_FAIL)
    __throw__("Failed to retrieve descriptor.");
}

/// @brief Logs a warning when keyframe descriptor retrieval fails.
/// @param descriptor__ Output wrapper containing the descriptor.
/// @param keyframeId__ Keyframe identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to chec if keyframe descriptor retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<descriptor_t>& descriptor__,
    const keyframe_id_t& keyframeId__) {
  using std::println;
  if (descriptor__ != fail_t::NO_FAIL) {
    println(
        "{}[WARNING]{} Keyframe {} \
            does not have a valid descriptor.",
        yellow,
        reset,
        std::string(keyframeId__));
    return true;
  }
  return false;
}

/// @brief Logs a warning when point descriptor retrieval fails.
/// @param descriptor__ Output wrapper containing the descriptor.
/// @param pointId__ Point identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if point descriptor retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<descriptor_t>& descriptor__,
    const point_id_t& pointId__) {
  using std::println;
  if (descriptor__ != fail_t::NO_FAIL) {
    println(
        "{}[WARNING]{} Point {} \
            does not have a valid descriptor.",
        yellow,
        reset,
        std::string(pointId__));
    return true;
  }
  return false;
}

/// @brief Logs a warning when descriptor retrieval fails.
/// @param descriptor__ Output wrapper containing the descriptor.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if descriptor retrieval failed.")]]
inline bool __fail_retrieve(const output_t<descriptor_t>& descriptor__) {
  using std::println;
  if (descriptor__ != fail_t::NO_FAIL) {
    println("{}[WARNING]{} failed to .", yellow, reset);
    return true;
  }
  return false;
}

/// @brief Logs a warning when descriptor-vector retrieval fails.
/// @param descriptor__ Output wrapper containing the descriptor vector.
/// @param keyframeId__ Keyframe identifier used in the warning.
/// @return `true` if retrieval failed, `false` otherwise.
[[nodiscard("Use to check if descriptor vector retrieval failed.")]]
inline bool __fail_retrieve(
    const output_t<descriptor_t::vector>& descriptor__,
    const keyframe_id_t& keyframeId__) {
  using std::println;
  if (descriptor__ != fail_t::NO_FAIL) {
    println(
        "{}[WARNING]{} failed to retrieve \
            the vector of descriptor {}.",
        yellow,
        reset,
        std::string(keyframeId__));
    return true;
  }
  return false;
}

}  // namespace slam_mer

#endif  // TYPE_DESCRIPTOR_HPP
