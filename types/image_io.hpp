/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file image_io.hpp
/// @brief Implements image reading functions for PNG and JPEG formats.
/// @details Provides functions to read images from disk into image_t objects,
/// supporting grayscale and color modes, with error handling for file and
/// format issues.
/// @author Pedro Miraldo
/// @author Valter Piedade
/// @author Masashi Yamazaki
#ifndef IMAGE_IO_HPP
#define IMAGE_IO_HPP

#include <types/image.hpp>
#include <types/output.hpp>

#include <cctype>
#include <csetjmp>
#include <cstring>
#include <filesystem>
#include <string>

#include <jpeglib.h>
#include <png.h>

namespace slam_mer {
namespace {

/// @brief Returns the lowercase file extension of a path.
/// @param path_ Input filesystem path.
/// @return Lowercase extension including the leading dot (for example
/// `".png"`).
inline std::string __lower_ext(const std::string& path_) {
  std::string ext = std::filesystem::path(path_).extension().string();
  for (auto& c : ext)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return ext;
}

// ---------------------------------------------------------------------------
// PNG

/// @brief Reads a PNG image from disk and converts it to project `image_t`.
/// @param path_ PNG file path.
/// @param grayscale_ `true` to output one grayscale channel, `false` for
/// three RGB channels.
/// @return Decoded image in `image_t::Type::U8`.
/// @throws std::runtime_error If the file cannot be opened or decoding fails.
[[nodiscard("Loading a PNG image from disk.")]]
inline image_t read_png(const std::string& path_, bool grayscale_) {
  FILE* fp = fopen(path_.c_str(), "rb");
  if (!fp) __throw__("read_image: cannot open '" + path_ + "'");

  unsigned char sig[8];
  if (fread(sig, 1, 8, fp) != 8 || png_sig_cmp(sig, 0, 8)) {
    fclose(fp);
    __throw__("read_image: not a PNG file: '" + path_ + "'");
  }

  png_structp png =
      png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png) {
    fclose(fp);
    __throw__("read_image: png_create_read_struct failed");
  }

  png_infop info = png_create_info_struct(png);
  if (!info) {
    png_destroy_read_struct(&png, nullptr, nullptr);
    fclose(fp);
    __throw__("read_image: png_create_info_struct failed");
  }

  // libpng error recovery via setjmp/longjmp
  if (setjmp(png_jmpbuf(png))) {
    png_destroy_read_struct(&png, &info, nullptr);
    fclose(fp);
    __throw__("read_image: PNG decode error in '" + path_ + "'");
  }

  png_init_io(png, fp);
  png_set_sig_bytes(png, 8);
  png_read_info(png, info);

  const png_byte colorType = png_get_color_type(png, info);
  const png_byte bitDepth = png_get_bit_depth(png, info);

  // Normalize to 8-bit per channel
  if (bitDepth == 16) png_set_strip_16(png);
  if (colorType == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
  if (colorType == PNG_COLOR_TYPE_GRAY && bitDepth < 8)
    png_set_expand_gray_1_2_4_to_8(png);
  if (png_get_valid(png, info, PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png);
  // Drop alpha
  if (colorType & PNG_COLOR_MASK_ALPHA) png_set_strip_alpha(png);

  const bool srcIsGray =
      (colorType == PNG_COLOR_TYPE_GRAY ||
       colorType == PNG_COLOR_TYPE_GRAY_ALPHA);

  if (srcIsGray && !grayscale_) png_set_gray_to_rgb(png);
  if (!srcIsGray && grayscale_) png_set_rgb_to_gray(png, 1, -1.0, -1.0);

  png_read_update_info(png, info);

  const uint32_t width = png_get_image_width(png, info);
  const uint32_t height = png_get_image_height(png, info);
  const size_t channels = grayscale_ ? 1u : 3u;

  image_t result(height, width, channels, image_t::Type::U8);

  std::vector<png_bytep> rows(height);
  for (uint32_t r = 0; r < height; ++r)
    rows[r] = result.ptr(r);

  png_read_image(png, rows.data());
  png_destroy_read_struct(&png, &info, nullptr);
  fclose(fp);

  return result;
}

// ---------------------------------------------------------------------------
// JPEG

/// Custom error manager that throws instead of calling exit().
struct JpegErrorMgr {
  /// JPEG error manager used by libjpeg callbacks.
  struct jpeg_error_mgr _base;  // must be first member
  /// Jump buffer used to recover from libjpeg fatal decode errors.
  std::jmp_buf _jmpbuf;
};

/// @brief Reads a JPEG image from disk and converts it to project `image_t`.
/// @param path_ JPEG file path.
/// @param grayscale_ `true` to output one grayscale channel, `false` for
/// three RGB channels.
/// @return Decoded image in `image_t::Type::U8`.
/// @throws std::runtime_error If the file cannot be opened or decoding fails.
[[nodiscard("Loading a JPEG image from disk.")]]
inline image_t read_jpeg(const std::string& path_, bool grayscale_) {
  FILE* fp = fopen(path_.c_str(), "rb");
  if (!fp) __throw__("read_image: cannot open '" + path_ + "'");

  struct jpeg_decompress_struct cinfo;
  JpegErrorMgr jerr_;

  cinfo.err = jpeg_std_error(&jerr_._base);
  jerr_._base.error_exit = [](j_common_ptr cinfo_) {
    auto* err = reinterpret_cast<JpegErrorMgr*>(cinfo_->err);
    longjmp(err->_jmpbuf, 1);
  };

  if (setjmp(jerr_._jmpbuf)) {
    jpeg_destroy_decompress(&cinfo);
    fclose(fp);
    __throw__("read_image: JPEG decode error in '" + path_ + "'");
  }

  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, fp);
  jpeg_read_header(&cinfo, TRUE);

  cinfo.out_color_space = grayscale_ ? JCS_GRAYSCALE : JCS_RGB;

  jpeg_start_decompress(&cinfo);

  const size_t width = cinfo.output_width;
  const size_t height = cinfo.output_height;
  const size_t channels = static_cast<size_t>(cinfo.output_components);

  image_t result(height, width, channels, image_t::Type::U8);

  while (cinfo.output_scanline < cinfo.output_height) {
    uint8_t* row = result.ptr(cinfo.output_scanline);
    jpeg_read_scanlines(&cinfo, &row, 1);
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(fp);

  return result;
}

/// @brief Reads a 16-bit grayscale PNG depth map and converts to F32.
/// @param path_ PNG file path.
/// @return Decoded depth image in `image_t::Type::F32` with 1 channel.
/// @throws std::runtime_error If the file cannot be opened, is not grayscale,
/// or decoding fails.
[[nodiscard("Loading a 16-bit depth PNG from disk.")]]
inline image_t read_depth_png(const std::string& path_) {
  FILE* fp = fopen(path_.c_str(), "rb");
  if (!fp) __throw__("read_depth_image: cannot open '" + path_ + "'");

  unsigned char sig[8];
  if (fread(sig, 1, 8, fp) != 8 || png_sig_cmp(sig, 0, 8)) {
    fclose(fp);
    __throw__("read_depth_image: not a PNG file: '" + path_ + "'");
  }

  png_structp png =
      png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png) {
    fclose(fp);
    __throw__("read_depth_image: png_create_read_struct failed");
  }

  png_infop info = png_create_info_struct(png);
  if (!info) {
    png_destroy_read_struct(&png, nullptr, nullptr);
    fclose(fp);
    __throw__("read_depth_image: png_create_info_struct failed");
  }

  // libpng error recovery via setjmp/longjmp
  if (setjmp(png_jmpbuf(png))) {
    png_destroy_read_struct(&png, &info, nullptr);
    fclose(fp);
    __throw__("read_depth_image: PNG decode error in '" + path_ + "'");
  }

  png_init_io(png, fp);
  png_set_sig_bytes(png, 8);
  png_read_info(png, info);

  const png_byte colorType = png_get_color_type(png, info);
  const png_byte bitDepth = png_get_bit_depth(png, info);
  const uint32_t width = png_get_image_width(png, info);
  const uint32_t height = png_get_image_height(png, info);

  // Verify it's a grayscale image
  if (colorType != PNG_COLOR_TYPE_GRAY) {
    png_destroy_read_struct(&png, &info, nullptr);
    fclose(fp);
    __throw__(
        "read_depth_image: expected grayscale PNG, got color type " +
        std::to_string(colorType));
  }

  // Read 16-bit depth without stripping
  if (bitDepth == 16) {
    png_set_swap(
        png);  // Ensure big-endian 16-bit values are in host byte order
  } else if (bitDepth != 16) {
    png_destroy_read_struct(&png, &info, nullptr);
    fclose(fp);
    __throw__(
        "read_depth_image: expected 16-bit grayscale PNG, got " +
        std::to_string(bitDepth) + "-bit");
  }

  png_read_update_info(png, info);

  // Allocate full buffer for all uint16 values (not just one row!)
  std::vector<uint16_t> temp_buffer(static_cast<size_t>(width) * height);
  std::vector<png_bytep> rows(height);

  // Each row must point to a DIFFERENT part of the buffer
  for (uint32_t r = 0; r < height; ++r) {
    rows[r] = reinterpret_cast<png_bytep>(
        temp_buffer.data() + static_cast<size_t>(r) * width);
  }

  png_read_image(png, rows.data());

  // Create output image as F32
  image_t result(height, width, 1, image_t::Type::F32);
  float* result_ptr = result.ptr<float>(0);

  // Convert uint16 array to float and store in result
  for (size_t i = 0; i < static_cast<size_t>(width) * height; ++i) {
    result_ptr[i] = static_cast<float>(temp_buffer[i]);
  }

  png_destroy_read_struct(&png, &info, nullptr);
  fclose(fp);

  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// Public static factory

/// @brief Loads an image file (`.png`, `.jpg`, `.jpeg`) into `image_t`.
/// @param path_ Input file path.
/// @param grayscale_ `true` to request grayscale output, `false` for color.
/// @return Decoded `image_t` in `Type::U8`.
/// @throws std::runtime_error If format is unsupported or decoding fails.
[[nodiscard("Loading an image from disk.")]]
inline image_t image_t::read_image(const std::string& path_, bool grayscale_) {
  const std::string ext = __lower_ext(path_);

  image_t result;
  if (ext == ".png") {
    result = read_png(path_, grayscale_);
  } else if (ext == ".jpg" || ext == ".jpeg") {
    result = read_jpeg(path_, grayscale_);
  } else {
    __throw__(
        "read_image: unsupported format '" + ext + "' in '" + path_ +
        "' (supported: .png, .jpg, .jpeg)");
  }

  // libpng and libjpeg decode to RGB, which matches the old OpenCV pipeline
  // (cv::imread BGR → cv::COLOR_BGR2RGB → RGB).  No channel swap needed.

  return result;
}

/// @brief Loads a 16-bit grayscale depth PNG into `image_t::Type::F32`.
/// @param path_ Input depth PNG file path.
/// @return Decoded depth image in `Type::F32` with 1 channel (height × width × 1).
/// @throws std::runtime_error If the file is not a 16-bit grayscale PNG or
/// decoding fails.
[[nodiscard("Loading a 16-bit depth PNG from disk.")]]
inline image_t image_t::read_depth_image(const std::string& path_) {
  const std::string ext = __lower_ext(path_);

  if (ext == ".png") return read_depth_png(path_);

  __throw__(
      "read_depth_image: unsupported format '" + ext + "' in '" + path_ +
      "' (supported: .png)");
}

}  // namespace slam_mer

#endif  // IMAGE_IO_HPP
