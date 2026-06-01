// Copyright 2023 ETH Zurich and Janusch Patas
// Apache-2.0
// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once
#include <torch/torch.h>

#include <array>
#include <span>

namespace lightglue {
    namespace feature {
        namespace internal {

            /**
             * InputPadder utility class for padding input tensors
             * Ensures input dimensions are divisible by a specified value
             */
            class InputPadder {
            public:
                InputPadder(int h, int w, int div_by = 8)
                    : ht_(h),
                      wd_(w) {
                    int pad_ht = (((ht_ / div_by) + 1) * div_by - ht_) % div_by;
                    int pad_wd = (((wd_ / div_by) + 1) * div_by - wd_) % div_by;

                    pad_ = {pad_wd / 2, pad_wd - pad_wd / 2,
                            pad_ht / 2, pad_ht - pad_ht / 2};
                }

                // Move semantics for pad operation
                torch::Tensor pad(torch::Tensor x) &&;
                torch::Tensor pad(const torch::Tensor& x) &;

                // Move semantics for unpad operation
                [[maybe_unused]] torch::Tensor unpad(torch::Tensor x) &&;
                torch::Tensor unpad(const torch::Tensor& x) &;

                void setPadding(std::span<const int> padding);
                std::span<const int> getPadding() const { return std::span<const int>(pad_); }

            private:
                int ht_;
                int wd_;
                std::array<int, 4> pad_;
            };

        } // namespace internal
    } // namespace feature
} // namespace lightglue
