// Copyright 2023 ETH Zurich and Janusch Patas
// Apache-2.0
// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
// SPDX-License-Identifier: AGPL-3.0-or-later

/**
 * @file aliked.hpp
 * @brief ALIKED feature extractor implementation
 *
 * This file provides the C++ implementation of ALIKED (Accurate and Lightweight
 * Keypoint Detector), a neural network-based method for fast and accurate
 * keypoint detection and feature descriptor extraction.
 */

#pragma once

#include "internal/blocks.hpp"
#include "internal/input_padder.hpp"
#include <torch/torch.h>

#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <unordered_map>

namespace lightglue {
    namespace feature {

        /**
         * @struct AlikedConfig
         * @brief Configuration parameters for ALIKED models
         *
         * Defines the channel numbers for convolutional layers and other
         * hyperparameters for each model variant.
         */
        struct AlikedConfig {
            int c1;  ///< Number of channels in block 1
            int c2;  ///< Number of channels in block 2
            int c3;  ///< Number of channels in block 3
            int c4;  ///< Number of channels in block 4
            int dim; ///< Dimension of feature descriptors
            int K;   ///< DKD parameter
            int M;   ///< Multi-scale parameter
        };

        // Forward declarations
        /**
         * @class DKD
         * @brief Deformable Kernel Deconvolution module
         */
        class DKD;

        /**
         * @class SDDH
         * @brief Sub-pixel Descriptor Deconvolution Head module
         */
        class SDDH;

        namespace internal {
            /**
             * @class ConvBlock
             * @brief Convolutional block
             */
            class ConvBlock;

            /**
             * @class ResBlock
             * @brief Residual block
             */
            class ResBlock;
        } // namespace internal

        /**
         * @brief Map of predefined ALIKED model configurations
         *
         * Available model variants:
         * - aliked-t16: Lightweight version (Tiny)
         * - aliked-n16: Standard version (Normal, 16 scales)
         * - aliked-n16rot: Rotation-invariant version (Normal, 16 scales)
         * - aliked-n32: Standard version (Normal, 32 scales)
         */
        inline const std::unordered_map<std::string_view, AlikedConfig> ALIKED_CFGS = {
            {"aliked-t16", {8, 16, 32, 64, 64, 3, 16}},
            {"aliked-n16", {16, 32, 64, 128, 128, 3, 16}},
            {"aliked-n16rot", {16, 32, 64, 128, 128, 3, 16}},
            {"aliked-n32", {16, 32, 64, 128, 128, 3, 32}}};

        /**
         * @class ALIKED
         * @brief Main class for ALIKED feature extractor
         *
         * Inherits from PyTorch's nn::Module and provides functionality to extract
         * keypoints and feature descriptors from images.
         *
         * @example
         * @code
         * ALIKED detector("aliked-n32", "cuda");
         * // img_rgb: RGB, HWC layout, uint8, H*W*3 bytes
         * auto result = detector.run(img_rgb_ptr, height, width);
         * auto keypoints = result["keypoints"];
         * auto descriptors = result["descriptors"];
         * @endcode
         */
        class ALIKED : public torch::nn::Module {
        public:
            /**
             * @brief ALIKED constructor
             *
             * @param model_name Model name to use (default: "aliked-n32")
             *                   Available: "aliked-t16", "aliked-n16", "aliked-n16rot", "aliked-n32"
             * @param device Computation device (default: "cuda")
             *               Can specify "cuda" or "cpu"
             * @param top_k Number of top-k keypoints to extract (default: -1 for all)
             * @param scores_th Threshold for keypoint scores (default: 0.2)
             * @param n_limit Maximum number of keypoints to extract (default: 20000)
             */
            explicit ALIKED(std::string_view model_name = "aliked-n32",
                            std::string_view device = "cuda", int top_k = -1,
                            float scores_th = 0.2, int n_limit = 20000);

            /**
             * @brief Extract dense feature map (rvalue reference version)
             *
             * Extracts dense feature maps and score maps from the image.
             * Uses move semantics for efficient tensor operations.
             *
             * @param image Input image tensor [B, C, H, W]
             * @return std::tuple<torch::Tensor, torch::Tensor>
             *         Tuple of (feature map, score map)
             */
            std::tuple<torch::Tensor, torch::Tensor> extract_dense_map(
                torch::Tensor image) &&;

            /**
             * @brief Forward pass (rvalue reference version)
             *
             * Extracts keypoints, descriptors, and scores from the image.
             * Uses move semantics.
             *
             * @param image Input image tensor [B, C, H, W]
             * @return torch::Dict<std::string, torch::Tensor>
             *         Dictionary containing "keypoints", "descriptors", "scores"
             */
            torch::Dict<std::string, torch::Tensor> forward(torch::Tensor image) &&;

            /**
             * @brief Forward pass (lvalue reference version)
             *
             * Extracts keypoints, descriptors, and scores from the image.
             *
             * @param image Input image tensor [B, C, H, W]
             * @return torch::Dict<std::string, torch::Tensor>
             *         Dictionary containing "keypoints", "descriptors", "scores"
             */
            torch::Dict<std::string, torch::Tensor> forward(const torch::Tensor& image) &;

            /**
             * @brief Extract features from OpenCV image
             *
             * Accepts raw RGB pixel data and extracts keypoints and descriptors.
             * Internally performs tensor conversion and normalization.
             *
             * @param img_rgb Pointer to RGB image data (HWC layout, uint8, H*W*3 bytes)
             * @param height  Image height in pixels
             * @param width   Image width in pixels
             * @return torch::Dict<std::string, torch::Tensor>
             *         Dictionary containing "keypoints", "descriptors", "scores"
             */
            torch::Dict<std::string, torch::Tensor> run(
                const uint8_t* img_rgb, int height, int width);

        private:
            /**
             * @brief Initialize network layers
             *
             * Constructs each layer of the network based on the specified model name.
             *
             * @param model_name Model name
             */
            void init_layers(std::string_view model_name);

            /**
             * @brief Load pretrained weights
             *
             * Loads pretrained parameters for the specified model.
             *
             * @param model_name Model name
             */
            void load_weights(std::string_view model_name);

            /**
             * @brief Load parameters from file
             *
             * Loads parameters from a PyTorch .pt file.
             *
             * @param pt_pth Path to parameter file
             */
            void load_parameters(std::string_view pt_pth);

            /**
             * @brief Read bytes from file
             *
             * Reads the contents of the specified file as a byte array.
             *
             * @param filename File name
             * @return std::vector<char> File contents
             */
            static std::vector<char> get_the_bytes(std::string_view filename);

            // Network layers
            torch::nn::AvgPool2d pool2_{nullptr};         ///< 2x2 average pooling layer
            torch::nn::AvgPool2d pool4_{nullptr};         ///< 4x4 average pooling layer
            std::shared_ptr<internal::ConvBlock> block1_; ///< First convolutional block
            std::shared_ptr<internal::ResBlock> block2_;  ///< Second residual block
            std::shared_ptr<internal::ResBlock> block3_;  ///< Third residual block
            std::shared_ptr<internal::ResBlock> block4_;  ///< Fourth residual block
            torch::nn::Conv2d conv1_{nullptr};            ///< First convolution layer
            torch::nn::Conv2d conv2_{nullptr};            ///< Second convolution layer
            torch::nn::Conv2d conv3_{nullptr};            ///< Third convolution layer
            torch::nn::Conv2d conv4_{nullptr};            ///< Fourth convolution layer
            torch::nn::Sequential score_head_{nullptr};   ///< Score prediction head

            std::shared_ptr<DKD> dkd_;        ///< Deformable Kernel Deconvolution module
            std::shared_ptr<SDDH> desc_head_; ///< Descriptor head module

            torch::Device device_; ///< Computation device (CPU or CUDA)
            int dim_{};            ///< Feature descriptor dimension
        };

    } // namespace feature
} // namespace lightglue
