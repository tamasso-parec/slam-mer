/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef SLAM_MER_GAUSSIAN_POSTERIOR_HPP
#define SLAM_MER_GAUSSIAN_POSTERIOR_HPP

#include <gtsam/nonlinear/ISAM2.h>
#include <Eigen/SparseCore>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace slam_mer::covisibility_graph {

/// @brief Owned, immutable numeric representation of one maintained backend Gaussian.
/// @details Covariance is in additive tangent coordinates at anchors(), not at
/// estimates(). All keys in a query are marginalized jointly, in caller order.
/// No optimizer pointers or Bayes-tree query caches are retained. See
/// doc/online_posterior.md for semantics, limits and publication guarantees.
class GaussianPosterior {
public:
  using shared_ptr = std::shared_ptr<const GaussianPosterior>;
  enum class Status { available, empty, unsupported, unavailable };
  enum class Coordinates { anchor_tangent, estimate_tangent };
  struct Query {
    uint64_t epoch = 0;
    Coordinates coordinates = Coordinates::anchor_tangent;
    gtsam::Vector mean;
    uint64_t revision = 0;
    Status status = Status::unavailable;
    std::string message;
    gtsam::KeyVector keys;
    std::vector<size_t> dimensions;
    gtsam::Matrix covariance;
  };

  /// @brief Capture numeric conditionals and matching values on the optimizer thread.
  /// @param isam_ Accepted optimizer state; never retained.
  /// @param estimates_ The values used by the backend map update.
  /// @param revision_ Monotonic accepted-update identity within this provider.
  /// @param epoch_ Provider identity; zero is reserved for standalone captures.
  /// @return Snapshot for Pose3/Similarity3/Point3, unsupported for other value
  /// types, or unavailable on numeric extraction failure.
  static shared_ptr capture(
      const gtsam::ISAM2& isam_,
      const gtsam::Values& estimates_,
      uint64_t revision_,
      uint64_t epoch_ = 0);

  /// @brief Joint marginal in caller key order; duplicates/missing keys are errors.
  Query joint_covariance(const gtsam::KeyVector& keys_) const;
  /// @brief First-order covariance transported through Retract to estimates().
  /// @details Returns the affine mean J*(gaussian_mean-estimate_delta) as well
  /// as J*P*J^T. Uses analytic GTSAM Jacobians of
  /// Local(estimate, Retract(anchor,delta+e)); supports Pose3, Sim(3) and Point3.
  /// Preserves every cross term.
  Query joint_covariance_at_estimate(const gtsam::KeyVector& keys_) const;
  /// @brief Process-local provider identity (zero for standalone captures).
  uint64_t epoch() const { return _epoch; }
  /// @brief Accepted backend update within this epoch.
  uint64_t revision() const { return _revision; }
  /// @brief Availability of the complete retained Gaussian.
  Status status() const { return _status; }
  /// @brief Diagnostic reason when posterior extraction is unavailable.
  const std::string& message() const { return _message; }
  /// @brief Owned optimizer linearization anchors for this revision.
  const gtsam::Values& anchors() const { return _anchors; }
  /// @brief Owned values supplied to map commit for this revision.
  const gtsam::Values& estimates() const { return _estimates; }
  /// @brief Complete optimum of the Gaussian in anchor coordinates (not wildfire delta).
  const gtsam::VectorValues& gaussian_mean() const { return _mean; }
  /// @brief Optimizer delta corresponding to estimates(), possibly wildfire truncated.
  const gtsam::VectorValues& estimate_delta() const { return _delta; }
  /// @brief Sparse factor-array storage; excludes Values, cache and overhead.
  size_t factor_bytes() const;
  /// @brief Wall time spent in capture, excluding map commit and queries.
  double capture_seconds() const { return _captureSeconds; }
  /// @brief Time when numeric capture completed, for measuring publication age.
  std::chrono::steady_clock::time_point captured_at() const {
    return _capturedAt;
  }

private:
  enum class VariableType { pose3, similarity3, point3 };
  struct Block {
    size_t offset, dimension;
    VariableType type;
  };
  GaussianPosterior() = default;
  uint64_t _epoch = 0, _revision = 0;
  Status _status = Status::unavailable;
  std::string _message;
  gtsam::Values _anchors, _estimates;
  gtsam::VectorValues _mean, _delta;
  Eigen::SparseMatrix<double> _r;
  std::map<gtsam::Key, Block> _blocks;
  std::chrono::steady_clock::time_point _capturedAt;
  double _captureSeconds = 0;
  // Only query results mutate, serialized independently from optimizer/map locks.
  mutable std::mutex _cacheMutex;
  mutable std::map<gtsam::KeyVector, Query> _cache;
  mutable size_t _cacheBytes = 0;
};

/// @brief Single-writer publication slot; readers retain old snapshots across updates.
class PosteriorPublication {
public:
  PosteriorPublication();
  uint64_t epoch() const { return _epoch; }
  /// @brief Capture, commit map values, then atomically publish the matching state.
  /// @details Extraction failure still invokes commit_. If commit_ throws the old
  /// publication remains readable. Called only by the optimizer thread.
  template <typename Commit>
  void publish_after_update(
      const gtsam::ISAM2& isam_,
      const gtsam::Values& estimates_,
      uint64_t revision_,
      Commit&& commit_) {
    const auto previous = latest();
    if (previous && revision_ <= previous->revision())
      throw std::invalid_argument("Stale posterior completion");
    const auto snapshot =
        GaussianPosterior::capture(isam_, estimates_, revision_, _epoch);
    commit_();
    publish(snapshot);
  }
  /// @brief Publish a fully constructed revision; reject stale attachment.
  void publish(GaussianPosterior::shared_ptr snapshot_);
  /// @brief Null before the first accepted update. Returned ownership survives replacement.
  GaussianPosterior::shared_ptr latest() const;

private:
  const uint64_t _epoch;
  mutable std::mutex _mutex;
  GaussianPosterior::shared_ptr _latest;
};
}  // namespace slam_mer::covisibility_graph
#endif
