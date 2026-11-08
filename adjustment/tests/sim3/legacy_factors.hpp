/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @brief Historical derivative fixture, never used by the production backend.
#pragma once
#include <covisibility_factors.hpp>

namespace slam_mer::validation::legacy {
using Sim3 = gtsam::Similarity3;

/// @brief Retain the pre-correction inverse/forward Jacobian chain for comparisons.
class Distance : public covisibility_graph::PointDistanceFactor<Sim3> {
public:
  using Base = covisibility_graph::PointDistanceFactor<Sim3>;
  using Base::Base;
  explicit Distance(const Base& other_) : Base(other_) {}
  gtsam::Vector evaluateError(
      const Sim3& pose_,
      const gtsam::Point3& point_,
      gtsam::OptionalMatrixType h1_,
      gtsam::OptionalMatrixType h2_) const override {
    gtsam::Matrix inverse, forward, point;
    const auto inverted = pose_.inverse(inverse);
    inverted.transformFrom(
        point_,
        h1_ ? &forward : nullptr,
        h2_ ? &point : nullptr);
    if (h1_) *h1_ = forward * inverse;
    if (h2_) *h2_ = point;
    return Base::evaluateError(pose_, point_, nullptr, nullptr);
  }
};

class Projection : public covisibility_graph::ProjectionFactor<Sim3> {
public:
  using Base = covisibility_graph::ProjectionFactor<Sim3>;
  using Base::Base;
  explicit Projection(const Base& other_) : Base(other_) {}
  gtsam::Vector evaluateError(
      const Sim3& pose_,
      const gtsam::Point3& point_,
      gtsam::OptionalMatrixType h1_,
      gtsam::OptionalMatrixType h2_) const override {
    gtsam::Matrix inverse, forward, point;
    const auto inverted = pose_.inverse(inverse);
    const auto camera = inverted.transformFrom(
        point_,
        h1_ ? &forward : nullptr,
        h2_ ? &point : nullptr);
    const double z = camera.z();
    gtsam::Matrix23 normalized;
    normalized << 1 / z, 0, -camera.x() / (z * z), 0, 1 / z,
        -camera.y() / (z * z);
    gtsam::Matrix22 intrinsics;
    this->_intrinsics->uncalibrate(
        gtsam::Point2(camera.x() / z, camera.y() / z),
        nullptr,
        intrinsics);
    if (h1_) *h1_ = intrinsics * normalized * forward * inverse;
    if (h2_) *h2_ = intrinsics * normalized * point;
    return Base::evaluateError(pose_, point_, nullptr, nullptr);
  }
};

class Between : public covisibility_graph::OdometryFactor<Sim3> {
public:
  using Base = covisibility_graph::OdometryFactor<Sim3>;
  using Base::Base;
  explicit Between(const Base& other_) : Base(other_) {}
  gtsam::Vector evaluateError(
      const Sim3& first_,
      const Sim3& second_,
      gtsam::OptionalMatrixType h1_,
      gtsam::OptionalMatrixType h2_) const override {
    return gtsam::BetweenFactor<Sim3>::evaluateError(first_, second_, h1_, h2_);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<Between>(*this);
  }
};
}  // namespace slam_mer::validation::legacy
