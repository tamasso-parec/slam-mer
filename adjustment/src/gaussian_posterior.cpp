/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gaussian_posterior.hpp>
#include <sim3_factor_derivatives.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace slam_mer::covisibility_graph {
namespace {
class UnsupportedVariable : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};
}  // namespace

GaussianPosterior::shared_ptr GaussianPosterior::capture(
    const gtsam::ISAM2& isam_,
    const gtsam::Values& estimates_,
    uint64_t revision_,
    uint64_t epoch_) {
  auto result = std::shared_ptr<GaussianPosterior>(new GaussianPosterior);
  const auto start = std::chrono::steady_clock::now();
  result->_revision = revision_;
  result->_epoch = epoch_;
  try {
    result->_estimates = estimates_;
    result->_anchors = isam_.getLinearizationPoint();
    result->_delta = isam_.getDelta();
    if (!result->_anchors.retract(result->_delta).equals(estimates_, 1e-9))
      throw std::invalid_argument("Estimates do not match optimizer delta");
    if (isam_.roots().empty()) {
      result->_status = Status::empty;
      result->_message = "No backend Gaussian variables";
    } else {
      // Children first ensures each frontal precedes all separator variables.
      std::vector<gtsam::GaussianConditional::shared_ptr> conditionals;
      // Iterative traversal avoids stack overflow on deep elimination trees.
      std::vector<gtsam::ISAM2::sharedClique> pending(
          isam_.roots().begin(),
          isam_.roots().end());
      while (!pending.empty()) {
        const auto clique = pending.back();
        pending.pop_back();
        conditionals.push_back(clique->conditional());
        pending.insert(
            pending.end(),
            clique->children.begin(),
            clique->children.end());
      }
      std::reverse(conditionals.begin(), conditionals.end());
      size_t dimension = 0;
      for (const auto& conditional : conditionals) {
        for (auto key = conditional->beginFrontals();
             key != conditional->endFrontals();
             ++key) {
          const size_t dim = conditional->getDim(key);
          if (!result->_anchors.exists(*key) ||
              result->_anchors.at(*key).dim() != dim)
            throw std::runtime_error("Gaussian and anchor dimensions disagree");
          const auto& value = result->_anchors.at(*key);
          VariableType type;
          if (dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&value))
            type = VariableType::pose3;
          else if (
              const auto sim =
                  dynamic_cast<const gtsam::GenericValue<gtsam::Similarity3>*>(
                      &value)) {
            if (!(sim->value().scale() > 0) ||
                !std::isfinite(sim->value().scale()))
              throw std::runtime_error("Invalid Similarity3 scale");
            type = VariableType::similarity3;
          } else if (dynamic_cast<const gtsam::GenericValue<gtsam::Point3>*>(
                         &value))
            type = VariableType::point3;
          else
            throw UnsupportedVariable(
                "Posterior supports Pose3, Similarity3 and Point3 values");
          if (!result->_blocks.emplace(*key, Block{dimension, dim, type})
                   .second)
            throw std::runtime_error("Duplicate Gaussian frontal");
          dimension += dim;
        }
      }
      if (result->_blocks.size() != result->_anchors.size())
        throw std::runtime_error("Gaussian and anchor key sets disagree");
      if (dimension > size_t(std::numeric_limits<int>::max()))
        throw std::length_error("Gaussian exceeds sparse index range");
      std::vector<Eigen::Triplet<double>> entries;
      gtsam::Vector rhs(dimension);
      for (const auto& conditional : conditionals) {
        const size_t row =
            result->_blocks.at(*conditional->beginFrontals()).offset;
        const auto model = conditional->get_model();
        const gtsam::Vector sigmas =
            model ? model->sigmas() : gtsam::Vector::Ones(conditional->rows());
        if (!sigmas.allFinite() || (sigmas.array() <= 0).any())
          throw std::runtime_error("Constrained or invalid conditional noise");
        rhs.segment(row, conditional->rows()) =
            conditional->d().array() / sigmas.array();
        for (auto key = conditional->begin(); key != conditional->end();
             ++key) {
          const auto block = result->_blocks.at(*key);
          const gtsam::Matrix a = conditional->getA(key);
          for (Eigen::Index j = 0; j < a.cols(); ++j)
            for (Eigen::Index i = 0; i < a.rows(); ++i) {
              const double value = a(i, j) / sigmas(i);
              if (!std::isfinite(value))
                throw std::runtime_error("Nonfinite factor");
              if (value != 0) {
                if (row + i > block.offset + j)
                  throw std::runtime_error("Nontriangular Gaussian factor");
                entries.emplace_back(row + i, block.offset + j, value);
              }
            }
        }
      }
      result->_r.resize(dimension, dimension);
      result->_r.setFromTriplets(entries.begin(), entries.end());
      result->_r.makeCompressed();
      for (size_t i = 0; i < dimension; ++i)
        if (!std::isfinite(result->_r.coeff(i, i)) ||
            result->_r.coeff(i, i) == 0)
          throw std::runtime_error(
              "Singular Gaussian factor; no regularization applied");
      const gtsam::Vector mean =
          result->_r.triangularView<Eigen::Upper>().solve(rhs);
      if (!mean.allFinite())
        throw std::runtime_error("Nonfinite Gaussian mean");
      for (const auto& [key, block] : result->_blocks)
        result->_mean.insert(key, mean.segment(block.offset, block.dimension));
      result->_status = Status::available;
    }
  } catch (const UnsupportedVariable& error_) {
    result->_status = Status::unsupported;
    result->_message = error_.what();
  } catch (const std::exception& error_) {
    result->_status = Status::unavailable;
    result->_message = error_.what();
    result->_r.resize(0, 0);
    result->_mean = gtsam::VectorValues();
  }
  result->_capturedAt = std::chrono::steady_clock::now();
  result->_captureSeconds =
      std::chrono::duration<double>(result->_capturedAt - start).count();
  return result;
}

GaussianPosterior::Query GaussianPosterior::joint_covariance(
    const gtsam::KeyVector& keys_) const {
  Query result;
  result.revision = _revision;
  result.epoch = _epoch;
  result.keys = keys_;
  result.status = _status;
  result.message = _message;
  if (_status != Status::available) return result;
  try {
    std::set<gtsam::Key> seen;
    size_t dimension = 0;
    for (const auto key : keys_) {
      if (!seen.insert(key).second)
        throw std::invalid_argument("Duplicate query key");
      const auto block = _blocks.find(key);
      if (block == _blocks.end())
        throw std::out_of_range("Missing backend key");
      result.dimensions.push_back(block->second.dimension);
      dimension += block->second.dimension;
    }
    // Bound temporary RHS and cached results independently. No graph-size cutoff.
    if (dimension > 256 ||
        (dimension &&
         size_t(_r.rows()) > (64u << 20) / sizeof(double) / dimension))
      throw std::length_error(
          "Query exceeds 256 scalars or 64 MiB solve workspace");
    {
      std::lock_guard<std::mutex> lock(_cacheMutex);
      const auto found = _cache.find(keys_);
      if (found != _cache.end()) return found->second;
    }
    result.mean = _mean.vector(keys_);
    gtsam::Matrix selected = gtsam::Matrix::Zero(_r.rows(), dimension);
    size_t column = 0;
    for (const auto key : keys_) {
      const auto block = _blocks.at(key);
      selected.block(block.offset, column, block.dimension, block.dimension)
          .setIdentity();
      column += block.dimension;
    }
    // H = R'R, so E'H^-1 E = (R'^-1 E)'(R'^-1 E).
    _r.transpose().triangularView<Eigen::Lower>().solveInPlace(selected);
    result.covariance = selected.transpose() * selected;
    if (!result.covariance.allFinite())
      throw std::runtime_error("Nonfinite covariance");
    const size_t bytes = (dimension * dimension + dimension) * sizeof(double) +
                         keys_.size() * (sizeof(gtsam::Key) + sizeof(size_t));
    {
      std::lock_guard<std::mutex> lock(_cacheMutex);
      if (_cache.size() >= 32 || _cacheBytes + bytes > (8u << 20)) {
        _cache.clear();
        _cacheBytes = 0;
      }
      if (_cache.emplace(keys_, result).second) _cacheBytes += bytes;
    }
  } catch (const std::exception& error_) {
    result.status = Status::unavailable;
    result.message = error_.what();
    result.covariance.resize(0, 0);
    result.mean.resize(0);
  }
  return result;
}

GaussianPosterior::Query GaussianPosterior::joint_covariance_at_estimate(
    const gtsam::KeyVector& keys_) const {
  Query result = joint_covariance(keys_);
  result.coordinates = Coordinates::estimate_tangent;
  if (result.status != Status::available) return result;
  try {
    gtsam::Matrix transform =
        gtsam::Matrix::Zero(result.covariance.rows(), result.covariance.cols());
    size_t offset = 0;
    for (const auto key : keys_) {
      const size_t dim = _blocks.at(key).dimension;
      const auto delta = _delta.at(key);
      if (_blocks.at(key).type == VariableType::pose3) {
        const auto& anchor = _anchors.at<gtsam::Pose3>(key);
        const auto& estimate = _estimates.at<gtsam::Pose3>(key);
        gtsam::Matrix66 retractJacobian, localJacobian;
        const auto retracted = anchor.retract(delta, {}, retractJacobian);
        estimate.localCoordinates(retracted, {}, localJacobian);
        transform.block<6, 6>(offset, offset) = localJacobian * retractJacobian;
      } else if (_blocks.at(key).type == VariableType::similarity3) {
        transform.block<7, 7>(offset, offset) = sim3_retract_jacobian(delta);
      } else if (_blocks.at(key).type == VariableType::point3) {
        transform.block<3, 3>(offset, offset).setIdentity();
      } else
        throw std::runtime_error("Unsupported variable coordinate type");
      offset += dim;
    }
    result.mean = transform * (result.mean - _delta.vector(keys_));
    result.covariance = transform * result.covariance * transform.transpose();
    if (!result.mean.allFinite() || !result.covariance.allFinite())
      throw std::runtime_error("Nonfinite transported Gaussian");
  } catch (const std::exception& error_) {
    result.status = Status::unavailable;
    result.message = error_.what();
    result.covariance.resize(0, 0);
    result.mean.resize(0);
  }
  return result;
}

size_t GaussianPosterior::factor_bytes() const {
  if (_r.rows() == 0) return 0;
  return _r.nonZeros() * (sizeof(double) + sizeof(int)) +
         (_r.outerSize() + 1) * sizeof(int);
}
PosteriorPublication::PosteriorPublication()
    : _epoch([] {
        static std::atomic<uint64_t> next{1};
        return next.fetch_add(1);
      }()) {}
void PosteriorPublication::publish(GaussianPosterior::shared_ptr snapshot_) {
  if (!snapshot_) throw std::invalid_argument("Null posterior publication");
  std::lock_guard<std::mutex> lock(_mutex);
  if (snapshot_->epoch() != _epoch)
    throw std::invalid_argument("Posterior epoch mismatch");
  if (_latest && snapshot_->revision() <= _latest->revision())
    throw std::invalid_argument("Stale posterior publication");
  _latest = std::move(snapshot_);
}
GaussianPosterior::shared_ptr PosteriorPublication::latest() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _latest;
}
}  // namespace slam_mer::covisibility_graph
