/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file image.hpp
/// @brief Defines the image_t class for pixel-interleaved, row-major image
/// buffers.
/// @details Provides a shared-ownership image type with runtime element type,
/// deep/shallow copy semantics, resizing, type conversion, and grayscale
/// conversion.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki
#ifndef TYPES_IMAGE_T_HPP
#define TYPES_IMAGE_T_HPP

#include <types/output.hpp>

#ifdef VISUAL_PLACE_LOCALIZATION
#include <vpl/visual_place_recognition.hpp>
#endif

#ifdef DEPTH_INFERENCE
#include <depth_inference/depth_inference.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace slam_mer {

/**
 * @brief Pixel-interleaved, row-major image buffer with shared ownership.
 *
 * Copying an Image shares the underlying buffer via shared_ptr (shallow copy,
 * analogous to cv::Mat reference counting). Use clone() for an independent
 * deep copy.
 *
 * The element type is expressed as a runtime tag (Type::U8 or Type::F32).
 */
class image_t {
public:
  // -------------------------------------------------------------------------
  // Element type
  // -------------------------------------------------------------------------

  enum class Type : uint8_t {
    U8,   ///< uint8_t elements — 8 bits per channel
    F32,  ///< float elements   — 32 bits per channel
  };

  // -------------------------------------------------------------------------
  // Construction
  // -------------------------------------------------------------------------

  /// @brief Creates an empty image with no allocated pixel buffer.
  /// @return Empty `image_t` instance.
  image_t() = default;

  /// @brief Creates an image and allocates a zero-initialized pixel buffer.
  /// @param rows_ Image height in pixels.
  /// @param cols_ Image width in pixels.
  /// @param channels_ Number of interleaved channels per pixel (for example 1
  /// for grayscale or 3 for RGB/BGR).
  /// @param type_ Element datatype used per channel.
  /// @return Initialized `image_t` with owned storage.
  image_t(size_t rows_, size_t cols_, size_t channels_, Type type_ = Type::U8)
      : _rows(rows_),
        _cols(cols_),
        _channels(channels_),
        _type(type_),
        _step(cols_ * channels_ * bytes_per_elem(type_)) {
    const size_t bytes = rows_ * _step;
    _data = std::shared_ptr<uint8_t[]>(new uint8_t[bytes]{});
  }

  /// @brief Creates a zero-filled image with the requested geometry.
  /// @param rows_ Image height in pixels.
  /// @param cols_ Image width in pixels.
  /// @param channels_ Number of channels per pixel.
  /// @param type_ Element datatype used per channel.
  /// @return Zero-initialized `image_t`.
  [[nodiscard("Use to get a zero-filled image with the requested geometry.")]]
  static image_t
  zeros(size_t rows_, size_t cols_, size_t channels_, Type type_ = Type::U8) {
    return image_t(rows_, cols_, channels_, type_);  // zero-initialized
  }

  /// @brief Copy-constructs an image with shared underlying storage.
  /// @param other_ Source image to copy.
  /// @return New `image_t` sharing the same buffer.
  image_t(const image_t& other_) = default;
  /// @brief Move-constructs an image by transferring ownership metadata.
  /// @param other_ Source image to move from.
  /// @return New `image_t` that takes ownership of the source state.
  image_t(image_t&& other_) = default;
  /// @brief Copy-assigns this image with shared underlying storage.
  /// @param other_ Source image to copy.
  /// @return Reference to this image.
  image_t& operator=(const image_t& other_) = default;
  /// @brief Move-assigns this image by transferring ownership metadata.
  /// @param other_ Source image to move from.
  /// @return Reference to this image.
  image_t& operator=(image_t&& other_) = default;

  // -------------------------------------------------------------------------
  // Deep copy
  // -------------------------------------------------------------------------

  /// @brief Creates a deep copy with an independent pixel buffer.
  /// @return Deep-copied image.
  [[nodiscard("Use to get a deep-copied image with independent pixel buffer.")]]
  image_t clone() const {
    if (empty()) return image_t{};
    image_t result(_rows, _cols, _channels, _type);
    std::memcpy(result._data.get(), _data.get(), size_bytes());
    return result;
  }

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------

  /// @brief Checks whether the image has a valid buffer and dimensions.
  /// @return `true` when the image has no usable pixel data.
  [[nodiscard("Checking if image has no usable pixel data.")]]
  bool empty() const noexcept {
    return !_data || _rows == 0 || _cols == 0;
  }

  // -------------------------------------------------------------------------
  // Dimensions
  // -------------------------------------------------------------------------

  /// @brief Returns the image height.
  /// @return Number of rows (pixels).
  [[nodiscard("Getting the number of rows (pixels).")]]
  size_t rows() const noexcept {
    return _rows;
  }
  /// @brief Returns the image width.
  /// @return Number of columns (pixels).
  [[nodiscard("Getting the number of columns (pixels).")]]
  size_t cols() const noexcept {
    return _cols;
  }
  /// @brief Returns channel count per pixel.
  /// @return Number of interleaved channels.
  [[nodiscard("Getting the number of interleaved channels.")]]
  size_t channels() const noexcept {
    return _channels;
  }
  /// @brief Returns the per-channel element datatype.
  /// @return Runtime datatype tag.
  [[nodiscard("Getting the per-channel element datatype.")]]
  Type type() const noexcept {
    return _type;
  }

  // -------------------------------------------------------------------------
  // Size helpers
  // -------------------------------------------------------------------------

  /// @brief Returns the number of pixels in the image plane.
  /// @return `rows() * cols()`.
  [[nodiscard("Getting the total number of pixels in the image plane.")]]
  size_t total() const noexcept {
    return _rows * _cols;
  }

  /// @brief Returns the number of bytes used by one pixel.
  /// @return Channel count multiplied by bytes per channel element.
  [[nodiscard("Getting the number of bytes used by one pixel.")]]
  size_t elem_size() const noexcept {
    return _channels * bytes_per_elem(_type);
  }

  /// @brief Returns total allocated payload size in bytes.
  /// @return `total() * elem_size()`.
  [[nodiscard("Getting the total allocated payload size in bytes.")]]
  size_t size_bytes() const noexcept {
    return total() * elem_size();
  }

  // -------------------------------------------------------------------------
  // Raw data access
  // -------------------------------------------------------------------------

  /// @brief Returns mutable pointer to the first byte of image storage.
  /// @return Raw mutable byte pointer.
  [[nodiscard("Getting a mutable pointer to the first byte of image storage.")]]
  uint8_t* data() noexcept {
    return _data.get();
  }
  /// @brief Returns read-only pointer to the first byte of image storage.
  /// @return Raw const byte pointer.
  [[nodiscard("Getting a read-only pointer to the first byte of image.")]]
  const uint8_t* data() const noexcept {
    return _data.get();
  }

  /// @brief Returns a typed mutable pointer to the first element of a row.
  /// @param row_ Row index from which pointer arithmetic starts.
  /// @return Pointer typed as `T*` into the internal buffer.
  template <typename T = uint8_t>
  [[nodiscard("Getting a typed mutable pointer to the first element of row.")]]
  T* ptr(size_t row_ = 0) noexcept {
    return reinterpret_cast<T*>(_data.get() + row_ * _step);
  }

  /// @brief Returns a typed read-only pointer to the first element of a row.
  /// @param row_ Row index from which pointer arithmetic starts.
  /// @return Const pointer typed as `const T*` into the internal buffer.
  template <typename T = uint8_t>
  [[nodiscard("Getting a typed read-only pointer to first element of a row.")]]
  const T* ptr(size_t row_ = 0) const noexcept {
    return reinterpret_cast<const T*>(_data.get() + row_ * _step);
  }

  /// @brief Returns pointer to the first channel byte of a pixel.
  /// @param row_ Pixel row index.
  /// @param col_ Pixel column index.
  /// @return Pointer to pixel start (`channel 0`) in interleaved storage.
  [[nodiscard("Getting a pointer to the first channel byte of a pixel.")]]
  const uint8_t* pixel(const size_t& row_, const size_t& col_) const noexcept {
    return _data.get() + row_ * _step + col_ * elem_size();
  }

  // -------------------------------------------------------------------------
  // Type conversion
  // -------------------------------------------------------------------------

  /// @brief Converts this image to another per-channel element datatype.
  /// @param new_type_ Target element datatype.
  /// @return Converted image with same geometry and channel count.
  [[nodiscard("Converting this image to another per-channel element.")]]
  image_t convert_to(Type new_type_) const {
    if (empty()) return image_t{};
    if (_type == new_type_) return clone();

    image_t result(_rows, _cols, _channels, new_type_);
    const size_t n = total() * _channels;

    if (_type == Type::U8 && new_type_ == Type::F32) {
      const uint8_t* src = data();
      float* dst = result.ptr<float>(0);
      for (size_t i = 0; i < n; ++i)
        dst[i] = static_cast<float>(src[i]);
    } else {
      // F32 → U8
      const float* src = ptr<float>(0);
      uint8_t* dst = result.data();
      for (size_t i = 0; i < n; ++i) {
        const float v = src[i];
        dst[i] = (v <= 0.0f)     ? uint8_t{0}
                 : (v >= 255.0f) ? uint8_t{255}
                                 : static_cast<uint8_t>(v);
      }
    }
    return result;
  }

  /// @brief Swaps channel 0 and 2 for each pixel.
  /// @return Deep copy with BGR<->RGB channel order swap applied when possible.
  [[nodiscard("Swapping channel 0 and 2 for each pixel.")]]
  image_t bgr2rgb() const {
    if (empty()) return image_t{};
    image_t result = clone();
    if (_channels < 3) return result;

    const size_t bpe = bytes_per_elem(_type);
    const size_t px = _channels * bpe;

    for (size_t r = 0; r < _rows; ++r) {
      uint8_t* row = result._data.get() + r * _step;
      for (size_t c = 0; c < _cols; ++c) {
        uint8_t* p = row + c * px;
        for (size_t b = 0; b < bpe; ++b)
          std::swap(p[b], p[2 * bpe + b]);
      }
    }
    return result;
  }

  /// @brief Converts this image to single-channel grayscale.
  /// @return Deep-copied grayscale image.
  [[nodiscard("Converting this image to single-channel grayscale.")]]
  image_t to_grayscale() const {
    if (empty()) return image_t{};
    if (_channels == 1) return clone();

    image_t result(_rows, _cols, 1, _type);

    if (_type == Type::U8) {
      for (size_t r = 0; r < _rows; ++r) {
        const uint8_t* src = ptr(r);
        uint8_t* dst = result.ptr(r);
        for (size_t c = 0; c < _cols; ++c) {
          const uint8_t* px = src + c * _channels;
          dst[c] = static_cast<uint8_t>(
              0.299 * px[0] + 0.587 * px[1] + 0.114 * px[2]);
        }
      }
    } else {
      for (size_t r = 0; r < _rows; ++r) {
        const float* src = ptr<float>(r);
        float* dst = result.ptr<float>(r);
        for (size_t c = 0; c < _cols; ++c) {
          const float* px = src + c * _channels;
          dst[c] = 0.299f * px[0] + 0.587f * px[1] + 0.114f * px[2];
        }
      }
    }
    return result;
  }

#ifdef VISUAL_PLACE_LOCALIZATION

  /// @brief Converts this image to VPL library image representation.
  /// @return Deep-copied VPL image payload.
  [[nodiscard("Converting this image to VPL library image representation.")]]
  inline vpl::visual_place_recognition::Image convert_to_vpl_type() const {

    if (empty()) return vpl::visual_place_recognition::Image();
    const std::size_t bytes = size_bytes();
    std::vector<std::uint8_t> imageData(bytes);
    std::memcpy(imageData.data(), _data.get(), bytes);
    return vpl::visual_place_recognition::Image(
        _cols,
        _rows,
        _channels,
        std::move(imageData));
  }

#endif  // VISUAL_PLACE_LOCALIZATION

#ifdef DEPTH_INFERENCE

  /// @brief Converts this image to depth-inference library image representation.
  /// @return Deep-copied depth-inference image payload.
  [[nodiscard("Converting this image to depth-inference library image.")]]
  inline depth_inference::Image convert_to_depth_inference_type() const {

    if (empty()) return depth_inference::Image();
    const std::size_t bytes = size_bytes();
    std::vector<std::uint8_t> imageData(bytes);
    std::memcpy(imageData.data(), _data.get(), bytes);
    return depth_inference::Image(
        _cols,
        _rows,
        _channels,
        std::move(imageData));
  }
#endif  // DEPTH_INFERENCE

  /// @brief Loads an image from disk.
  /// @param path_ Input file path (`.png`, `.jpg`, or `.jpeg`).
  /// @param grayscale_ `true` to force single-channel grayscale loading.
  /// @return Decoded image in `Type::U8`.
  /// @throws std::runtime_error If decoding fails or format is unsupported.
  [[nodiscard("Loading an image from disk.")]]
  static image_t read_image(const std::string& path_, bool grayscale_ = false);

  /// @brief Loads a 16-bit grayscale depth PNG into `Type::F32`.
  /// @param path_ Input file path (`.png`).
  /// @return Decoded depth image in `Type::F32` with 1 channel.
  /// @throws std::runtime_error If the file is not a 16-bit grayscale PNG or
  /// decoding fails.
  [[nodiscard("Loading a 16-bit depth PNG from disk.")]]
  static image_t read_depth_image(const std::string& path_);

  /// @brief Resizes this image by uniform scale using bilinear interpolation.
  /// @param scaleFactor_ Positive scale factor applied to width and height.
  /// @return Resized image preserving channel count and datatype.
  /// @throws std::runtime_error If image is empty or scale is non-positive.
  [[nodiscard("Resizing this image by a uniform scale using bilinear int.")]]
  image_t resize(double scaleFactor_) const {
    if (empty()) __throw__("image_t::resize: image is empty");
    if (scaleFactor_ <= 0.0)
      __throw__("image_t::resize: scaleFactor must be > 0");

    const size_t out_rows = std::max(
        size_t{1},
        static_cast<size_t>(std::lround(_rows * scaleFactor_)));
    const size_t out_cols = std::max(
        size_t{1},
        static_cast<size_t>(std::lround(_cols * scaleFactor_)));

    image_t result(out_rows, out_cols, _channels, _type);

    const double row_scale = static_cast<double>(_rows) / out_rows;
    const double col_scale = static_cast<double>(_cols) / out_cols;

    for (size_t oy = 0; oy < out_rows; ++oy) {
      const double sy = (static_cast<double>(oy) + 0.5) * row_scale - 0.5;
      const long fy = static_cast<long>(std::floor(sy));
      const size_t y0 = static_cast<size_t>(
          std::clamp(fy, 0L, static_cast<long>(_rows) - 1L));
      const size_t y1 = static_cast<size_t>(
          std::clamp(fy + 1L, 0L, static_cast<long>(_rows) - 1L));
      const double wy = sy - static_cast<double>(fy);

      for (size_t ox = 0; ox < out_cols; ++ox) {
        const double sx = (static_cast<double>(ox) + 0.5) * col_scale - 0.5;
        const long fx = static_cast<long>(std::floor(sx));
        const size_t x0 = static_cast<size_t>(
            std::clamp(fx, 0L, static_cast<long>(_cols) - 1L));
        const size_t x1 = static_cast<size_t>(
            std::clamp(fx + 1L, 0L, static_cast<long>(_cols) - 1L));
        const double wx = sx - static_cast<double>(fx);

        if (_type == Type::U8) {
          const uint8_t* p00 = ptr(y0) + x0 * _channels;
          const uint8_t* p01 = ptr(y0) + x1 * _channels;
          const uint8_t* p10 = ptr(y1) + x0 * _channels;
          const uint8_t* p11 = ptr(y1) + x1 * _channels;
          uint8_t* out = result.ptr(oy) + ox * _channels;
          for (size_t c = 0; c < _channels; ++c) {
            const double v = (1.0 - wy) * ((1.0 - wx) * p00[c] + wx * p01[c]) +
                             wy * ((1.0 - wx) * p10[c] + wx * p11[c]);
            out[c] = static_cast<uint8_t>(std::clamp(std::lround(v), 0L, 255L));
          }
        } else {
          const float* p00 = ptr<float>(y0) + x0 * _channels;
          const float* p01 = ptr<float>(y0) + x1 * _channels;
          const float* p10 = ptr<float>(y1) + x0 * _channels;
          const float* p11 = ptr<float>(y1) + x1 * _channels;
          float* out = result.ptr<float>(oy) + ox * _channels;
          for (size_t c = 0; c < _channels; ++c) {
            out[c] = static_cast<float>(
                (1.0 - wy) * ((1.0 - wx) * p00[c] + wx * p01[c]) +
                wy * ((1.0 - wx) * p10[c] + wx * p11[c]));
          }
        }
      }
    }
    return result;
  }

private:
  /// @brief Returns byte size of one channel element for a datatype.
  /// @param type_ Channel datatype tag.
  /// @return Number of bytes per channel element.
  static constexpr size_t bytes_per_elem(Type type_) noexcept {
    return (type_ == Type::F32) ? sizeof(float) : sizeof(uint8_t);
  }

  /// Number of image rows (height in pixels).
  size_t _rows = 0;
  /// Number of image columns (width in pixels).
  size_t _cols = 0;
  /// Number of interleaved channels per pixel.
  size_t _channels = 0;
  /// Runtime channel datatype.
  Type _type = Type::U8;
  /// Shared pixel buffer stored as raw bytes.
  std::shared_ptr<uint8_t[]> _data;
  /// Stride in bytes between consecutive rows.
  size_t _step = 0;
};

}  // namespace slam_mer

#endif  // TYPES_IMAGE_T_HPP
