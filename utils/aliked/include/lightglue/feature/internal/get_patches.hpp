// Copyright 2023 ETH Zurich and Janusch Patas
// Apache-2.0
// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <torch/torch.h>

namespace lightglue {
    namespace ops {

        /**
         * CUDA operations for patch extraction
         */
        torch::Tensor get_patches_forward(const torch::Tensor& map, torch::Tensor& points, int64_t radius);
        torch::Tensor get_patches_backward(const torch::Tensor& d_patches, torch::Tensor& points, int64_t H, int64_t W);

    } // namespace ops
} // namespace lightglue
