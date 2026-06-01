/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file kl_divergence.cpp
/// @brief KL-divergence computation implementations for histogram data.
/// @details Implements numeric KL-divergence routines for both dictionary and
/// vector histogram inputs, including normalization controls. Used for
/// statistical comparison tasks in the pipeline.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <cmath>
#include <kl_divergence.hpp>
#include <numeric>

namespace slam_mer {
namespace statistics {

// ----------------------------------------------------------------------------
// Converts dictionary_t<keyframe_id_t, int> to vectors and computes KL(P || Q)
// ----------------------------------------------------------------------------
float kl_divergence(
    const dictionary_t<keyframe_id_t, int>& countMapA_,
    const dictionary_t<keyframe_id_t, int>& countMapB_,
    bool normalize_,
    float epsilon_) {

  std::vector<float> countVectorA(countMapA_.size(), 0.0f);
  std::vector<float> countVectorB(countMapB_.size(), 0.0f);

  for (const auto& [key, value] : countMapA_) {
    const size_t& index = static_cast<size_t>(key.key());
    countVectorA.at(index) = static_cast<float>(value);
  }

  for (const auto& [key, value] : countMapB_) {
    const size_t& index = static_cast<size_t>(key.key());
    countVectorB.at(index) = static_cast<float>(value);
  }

  return kl_divergence(countVectorA, countVectorB, normalize_, epsilon_);
}

// ----------------------------------------------------------------------------
// Compute KL(P || Q) for two histograms P and Q.
// ----------------------------------------------------------------------------
float kl_divergence(
    std::vector<float>& P_,
    std::vector<float>& Q_,
    bool normalize_,
    float epsilon_) {

  // -------------------------------------------------------------------------
  // Lambda function
  auto ___sum_vec = [](const std::vector<float>& v__) {
    return std::accumulate(v__.begin(), v__.end(), 0.0f);
  };
  // -------------------------------------------------------------------------

  if (P_.size() != Q_.size() || P_.empty() || Q_.empty())
    __throw__("P and Q must have the same nonzero size.");
  if (epsilon_ < 0.0) __throw__("epsilon must be >= 0");

  if (epsilon_ > 0.0) {
    for (size_t i = 0; i < P_.size(); ++i) {
      P_[i] += epsilon_;
      Q_[i] += epsilon_;
    }
  }

  if (normalize_) {
    const float sp = ___sum_vec(P_);
    const float sq = ___sum_vec(Q_);
    if (!(sp > 0.0) || !(sq > 0.0))
      __throw__("Sums must be > 0 after smoothing/normalization.");
    for (float& x : P_)
      x /= sp;
    for (float& x : Q_)
      x /= sq;
  }

  float kl = 0.0;
  for (size_t i = 0; i < P_.size(); ++i) {
    const float pi = P_[i];
    const float qi = Q_[i];
    if (pi == 0.0) continue;  // 0 * log(0/qi) := 0
    if (qi == 0.0) return std::numeric_limits<float>::infinity();
    kl += pi * std::log(pi / qi);
  }
  return kl;
}

}  // namespace statistics
}  // namespace slam_mer
