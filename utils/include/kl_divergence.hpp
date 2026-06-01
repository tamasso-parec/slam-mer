/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file kl_divergence.hpp
/// @brief KL-divergence helpers for histogram-style SLAM statistics.
/// @details Declares overloads to compute Kullback-Leibler divergence for
/// dictionary-backed and vector-backed histogram representations.
/// Optional normalization and smoothing are supported.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef KL_DISTANCE_HPP
#define KL_DISTANCE_HPP

#include <types/dictionary.hpp>
#include <types/id.hpp>
#include <vector>

namespace slam_mer {
namespace statistics {

/// @brief Compute KL(P || Q) for two histograms P and Q.
/// @param P_ Histogram counts for distribution `P`, indexed by keyframe ID.
/// @param Q_ Histogram counts for distribution `Q`, indexed by keyframe ID.
/// @param normalize_ Enables conversion of counts into probability vectors.
/// @param epsilon_ Nonnegative Laplace-smoothing factor added per bin.
/// @return Kullback-Leibler divergence value `KL(P || Q)`.
float kl_divergence(
    const dictionary_t<keyframe_id_t, int>& P_,
    const dictionary_t<keyframe_id_t, int>& Q_,
    bool normalize_ = true,
    float epsilon_ = 1e-12);

/// @brief Compute KL(P || Q) for two histograms P and Q.
/// @param P_ Input vector for distribution `P`; modified when smoothing or
/// normalization is enabled.
/// @param Q_ Input vector for distribution `Q`; modified when smoothing or
/// normalization is enabled.
/// @param normalize_ Enables in-place conversion to probability vectors.
/// @param epsilon_ Nonnegative Laplace-smoothing factor added per element.
/// @return Kullback-Leibler divergence value `KL(P || Q)`.
float kl_divergence(
    std::vector<float>& P_,
    std::vector<float>& Q_,
    bool normalize_ = true,
    float epsilon_ = 1e-12);

}  // namespace statistics
}  // namespace slam_mer

#endif  // KL_DISTANCE_HPP
