// Copyright 2023 ETH Zurich and Janusch Patas
// Apache-2.0
// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

namespace lightglue {
    namespace ops {

#define CUDA_1D_KERNEL_LOOP_T(i, n, index_t)                           \
    for (index_t i = (blockIdx.x * blockDim.x) + threadIdx.x; i < (n); \
         i += (blockDim.x * gridDim.x))

#define CUDA_1D_KERNEL_LOOP(i, n) CUDA_1D_KERNEL_LOOP_T(i, n, int)

        template <typename integer>
        constexpr __host__ __device__ inline integer ceil_div(integer n, integer m) {
            return (n + m - 1) / m;
        }

    } // namespace ops
} // namespace lightglue
