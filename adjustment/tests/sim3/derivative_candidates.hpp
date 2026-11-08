/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @brief Experimental derivative repairs; not selected by the production backend.
#pragma once
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/slam/PriorFactor.h>
#include <covisibility_factors.hpp>

namespace slam_mer::validation {
using Sim3 = gtsam::Similarity3;

/// @brief Differentiate the patched inverse point action in its actual right chart.
/// @details Only first derivatives of Retract at zero are required; the chart
/// need not be the group's exponential for these Jacobians to be valid.
inline void camera_jacobians(
    const Sim3& pose_,
    const gtsam::Point3& point_,
    gtsam::OptionalMatrixType poseJacobian_,
    gtsam::OptionalMatrixType pointJacobian_) {
  const gtsam::Point3 camera = pose_.inverse().transformFrom(point_);
  if (poseJacobian_) {
    poseJacobian_->resize(3, 7);
    *poseJacobian_ << gtsam::skewSymmetric(camera), -gtsam::Matrix3::Identity(),
        -camera;
  }
  if (pointJacobian_)
    *pointJacobian_ = pose_.rotation().matrix().transpose() / pose_.scale();
}

class DistanceCandidate : public covisibility_graph::PointDistanceFactor<Sim3> {
public:
  using Base = covisibility_graph::PointDistanceFactor<Sim3>;
  using Base::Base;
  explicit DistanceCandidate(const Base& other_) : Base(other_) {}
  gtsam::Vector evaluateError(
      const Sim3& pose_,
      const gtsam::Point3& point_,
      gtsam::OptionalMatrixType h1_,
      gtsam::OptionalMatrixType h2_) const override {
    camera_jacobians(pose_, point_, h1_, h2_);
    return Base::evaluateError(pose_, point_, nullptr, nullptr);
  }
};

class ProjectionCandidate : public covisibility_graph::ProjectionFactor<Sim3> {
public:
  using Base = covisibility_graph::ProjectionFactor<Sim3>;
  using Base::Base;
  explicit ProjectionCandidate(const Base& other_) : Base(other_) {}
  gtsam::Vector evaluateError(
      const Sim3& pose_,
      const gtsam::Point3& point_,
      gtsam::OptionalMatrixType h1_,
      gtsam::OptionalMatrixType h2_) const override {
    if (h1_ || h2_) {
      const auto camera = pose_.inverse().transformFrom(point_);
      const double z = camera.z();
      gtsam::Matrix23 normalized;
      normalized << 1 / z, 0, -camera.x() / (z * z), 0, 1 / z,
          -camera.y() / (z * z);
      gtsam::Matrix22 intrinsics;
      this->_intrinsics->uncalibrate(
          gtsam::Point2(camera.x() / z, camera.y() / z),
          nullptr,
          intrinsics);
      gtsam::Matrix a, b;
      camera_jacobians(pose_, point_, h1_ ? &a : nullptr, h2_ ? &b : nullptr);
      if (h1_) *h1_ = intrinsics * normalized * a;
      if (h2_) *h2_ = intrinsics * normalized * b;
    }
    return Base::evaluateError(pose_, point_, nullptr, nullptr);
  }
};

/// @brief Independent numerical oracle for either two-variable visual factor.
template <typename Base>
class NumericalVisual : public Base {
public:
  using Base::Base;
  explicit NumericalVisual(const Base& other_) : Base(other_) {}
  gtsam::Vector evaluateError(
      const Sim3& pose_,
      const gtsam::Point3& point_,
      gtsam::OptionalMatrixType h1_,
      gtsam::OptionalMatrixType h2_) const override {
    const auto residual = Base::evaluateError(pose_, point_, nullptr, nullptr);
    if (h1_) {
      h1_->resize(residual.size(), 7);
      for (int j = 0; j < 7; ++j) {
        gtsam::Vector7 step = gtsam::Vector7::Zero();
        step(j) = 1e-6;
        h1_->col(j) = (Base::evaluateError(
                           pose_.retract(step),
                           point_,
                           nullptr,
                           nullptr) -
                       Base::evaluateError(
                           pose_.retract(-step),
                           point_,
                           nullptr,
                           nullptr)) /
                      2e-6;
      }
    }
    if (h2_) {
      h2_->resize(residual.size(), 3);
      for (int j = 0; j < 3; ++j) {
        gtsam::Vector3 step = gtsam::Vector3::Zero();
        step(j) = 1e-6;
        h2_->col(j) =
            (Base::evaluateError(pose_, point_ + step, nullptr, nullptr) -
             Base::evaluateError(pose_, point_ - step, nullptr, nullptr)) /
            2e-6;
      }
    }
    return residual;
  }
};

/// @brief Correct both the stale adjoint and omitted Local derivative numerically.
/// @details Both Jacobians require 28 perturbed residual evaluations plus the
/// nominal evaluation. This is a validation candidate, without an online budget claim.
class BetweenCandidate : public covisibility_graph::OdometryFactor<Sim3> {
public:
  using Base = covisibility_graph::OdometryFactor<Sim3>;
  using Base::Base;
  explicit BetweenCandidate(const Base& other_) : Base(other_) {}
  gtsam::Vector evaluateError(
      const Sim3& a_,
      const Sim3& b_,
      gtsam::OptionalMatrixType h1_,
      gtsam::OptionalMatrixType h2_) const override {
    auto residual = [&](const Sim3& first_, const Sim3& second_) {
      return Base::evaluateError(first_, second_, nullptr, nullptr);
    };
    for (int side = 0; side < 2; ++side) {
      auto output = side == 0 ? h1_ : h2_;
      if (!output) continue;
      output->resize(7, 7);
      for (int j = 0; j < 7; ++j) {
        gtsam::Vector7 step = gtsam::Vector7::Zero();
        step(j) = 1e-6;
        output->col(j) = side == 0 ? (residual(a_.retract(step), b_) -
                                      residual(a_.retract(-step), b_)) /
                                         2e-6
                                   : (residual(a_, b_.retract(step)) -
                                      residual(a_, b_.retract(-step))) /
                                         2e-6;
      }
    }
    return residual(a_, b_);
  }
};

class PriorCandidate : public gtsam::PriorFactor<Sim3> {
public:
  using Base = gtsam::PriorFactor<Sim3>;
  using Base::Base;
  explicit PriorCandidate(const Base& other_) : Base(other_) {}
  gtsam::Vector evaluateError(const Sim3& pose_, gtsam::OptionalMatrixType h_)
      const override {
    if (h_) {
      h_->resize(7, 7);
      for (int j = 0; j < 7; ++j) {
        gtsam::Vector7 step = gtsam::Vector7::Zero();
        step(j) = 1e-6;
        h_->col(j) = (Base::evaluateError(pose_.retract(step), nullptr) -
                      Base::evaluateError(pose_.retract(-step), nullptr)) /
                     2e-6;
      }
    }
    return Base::evaluateError(pose_, nullptr);
  }
};
}  // namespace slam_mer::validation
