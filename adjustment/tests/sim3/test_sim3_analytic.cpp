/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
#include <covisibility_factors.hpp>
#include <interface_rerun.hpp>
#include <settings.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace slam_mer;
using namespace slam_mer::covisibility_graph;
using Sim3 = gtsam::Similarity3;

namespace {
int checks = 0;
double worstLog = 0, worstFactor = 0;

void check(
    const Matrix& actual_,
    const Matrix& expected_,
    double tolerance_,
    const char* message_) {
  ++checks;
  const double error =
      (actual_ - expected_).norm() / std::max(1., expected_.norm());
  if (!std::isfinite(error) || error > tolerance_) {
    std::cerr << message_ << ": relative error " << error << ", tolerance "
              << tolerance_ << '\n';
    throw std::runtime_error(message_);
  }
}

/// @brief Independent five-point differences, used only by the tests.
template <typename Residual>
Matrix77
difference(const Sim3& pose_, const Residual& residual_, double step_ = 1e-5) {
  Matrix77 result;
  for (int j = 0; j < 7; ++j) {
    Vector7 delta = Vector7::Zero();
    delta(j) = step_;
    result.col(j) = (-residual_(pose_.retract(2 * delta)) +
                     8 * residual_(pose_.retract(delta)) -
                     8 * residual_(pose_.retract(-delta)) +
                     residual_(pose_.retract(-2 * delta))) /
                    (12 * step_);
  }
  return result;
}

void check_log(const Sim3& pose_, double tolerance_, double step_ = 1e-5) {
  Matrix analytic;
  check(
      sim3_logmap(pose_, &analytic),
      Sim3::Logmap(pose_),
      0,
      "Logmap value changed");
  const Matrix numeric = difference(
      pose_,
      [](const Sim3& value_) { return Sim3::Logmap(value_); },
      step_);
  worstLog = std::max(
      worstLog,
      (analytic - numeric).norm() / std::max(1., numeric.norm()));
  if ((analytic - numeric).norm() > tolerance_ * std::max(1., numeric.norm())) {
    std::cerr << "omega=" << Rot3::Logmap(pose_.rotation()).transpose()
              << " log scale=" << std::log(pose_.scale()) << " step=" << step_
              << "\nanalytic:\n"
              << analytic << "\nnumeric:\n"
              << numeric << '\n';
  }
  check(analytic, numeric, tolerance_, "Logmap derivative");
}

void identity_and_limits() {
  Matrix jacobian;
  sim3_logmap(Sim3(), &jacobian);
  check(jacobian, Matrix77::Identity(), 1e-14, "Identity log derivative");
  const Vector3 translation(.3, -.8, 1.2);
  for (double lambda : {-1.2, -.1, -1e-5, 0., 1e-5, .1, 1.2}) {
    const Sim3 pose(Rot3(), translation, std::exp(lambda));
    const double effectiveLambda = std::log(pose.scale());
    const bool small = effectiveLambda * effectiveLambda <= 1e-9;
    const double p =
        small ? 1 - effectiveLambda / 2 + effectiveLambda * effectiveLambda / 6
              : -std::expm1(-effectiveLambda) / effectiveLambda;
    const double dp = small
                          ? -.5 + effectiveLambda / 3
                          : (std::exp(-effectiveLambda) - p) / effectiveLambda;
    const auto logarithm = sim3_logmap(pose, &jacobian);
    check(
        logarithm.segment<3>(3),
        translation / p,
        1e-12,
        "Pure-scale log value");
    check(
        jacobian.block<3, 3>(3, 3),
        std::exp(lambda) / p * Matrix3::Identity(),
        1e-12,
        "Pure-scale translation block");
    check(
        jacobian.block<3, 1>(3, 6),
        -dp / (p * p) * translation,
        1e-12,
        "Pure-scale derivative");
  }
  // Both small-scale branches, and both sides of SO(3)'s polynomial threshold.
  // Do not difference across a branch boundary or the rotation pi cut.
  for (double angle : {0., 1e-5, .0008, .0012, .3, 1.7, 3.12})
    for (double lambda : {-1.2, -.02, -.0002, -2e-5, 0., 2e-5, .0002, .02, 1.2})
      check_log(
          Sim3(
              Rot3::Expmap(Vector3(angle, 0, 0)),
              translation,
              std::exp(lambda)),
          3e-6,
          1e-6);
  const std::vector<Vector3> axes{
      Vector3::UnitX(),
      Vector3::UnitY(),
      Vector3::UnitZ(),
      Vector3(.2, -.8, .5).normalized()};
  for (const auto& axis : axes)
    for (double angle : {-3.141, -3.12, -3.08, 3.08, 3.12, 3.141})
      for (double lambda : {-.4, 0., .4})
        check_log(
            Sim3(Rot3::Expmap(angle * axis), translation, std::exp(lambda)),
            1e-6,
            1e-5);
}

void factor_and_adjoint_trials() {
  std::mt19937 generator(3872026);
  std::normal_distribution<double> normal;
  auto vector = [&]() {
    return Vector3(normal(generator), normal(generator), normal(generator));
  };
  const auto noise = noiseModel::Isotropic::Sigma(7, .2);
  for (int trial = 0; trial < 160; ++trial) {
    const Sim3 first(
        Rot3::Expmap(.4 * vector()),
        vector(),
        std::exp(.4 * normal(generator)));
    const Sim3 second(
        Rot3::Expmap(.4 * vector()),
        vector(),
        std::exp(.4 * normal(generator)));
    const Sim3 measured(
        Rot3::Expmap(.4 * vector()),
        .4 * vector(),
        std::exp(.4 * normal(generator)));
    check_log(first, 5e-7);
    // Conjugation at identity checks the physical adjoint independently of Logmap.
    const auto conjugate = [&](const Sim3& delta_) {
      return Sim3::Logmap(first * delta_ * first.inverse());
    };
    check(
        sim3_adjoint(first),
        difference(Sim3(), conjugate),
        1e-8,
        "Physical adjoint");
    OdometryFactor<Sim3> between(
        keyframe_id_t(0),
        keyframe_id_t(1),
        measured,
        noise);
    gtsam::BetweenFactor<Sim3> native(
        Symbol('k', 0),
        Symbol('k', 1),
        measured,
        noise);
    Matrix a, b, only;
    const Vector value = between.evaluateError(first, second, &a, &b);
    check(
        value,
        native.evaluateError(first, second),
        0,
        "Between value changed");
    const Matrix na = difference(first, [&](const Sim3& v_) {
      return native.evaluateError(v_, second);
    });
    const Matrix nb = difference(second, [&](const Sim3& v_) {
      return native.evaluateError(first, v_);
    });
    worstFactor = std::max(
        {worstFactor,
         (a - na).norm() / std::max(1., na.norm()),
         (b - nb).norm() / std::max(1., nb.norm())});
    check(a, na, 5e-7, "Between first block");
    check(b, nb, 5e-7, "Between second block");
    between.evaluateError(first, second, &only, nullptr);
    check(only, a, 0, "Between optional first block");
    between.evaluateError(first, second, nullptr, &only);
    check(only, b, 0, "Between optional second block");
    PosePriorFactor<Sim3> prior(Symbol('k', 0), measured, noise);
    gtsam::PriorFactor<Sim3> nativePrior(Symbol('k', 0), measured, noise);
    check(
        prior.evaluateError(first, &a),
        nativePrior.evaluateError(first),
        0,
        "Prior value changed");
    const Matrix np = difference(first, [&](const Sim3& v_) {
      return nativePrior.evaluateError(v_);
    });
    worstFactor =
        std::max(worstFactor, (a - np).norm() / std::max(1., np.norm()));
    check(a, np, 5e-7, "Prior derivative");
    // Joint directional derivatives exercise both blocks together at nonzero error.
    Vector7 da, db;
    for (int j = 0; j < 7; ++j) {
      da(j) = .1 * normal(generator);
      db(j) = .1 * normal(generator);
    }
    constexpr double step = 1e-5;
    const Vector directional = (native.evaluateError(
                                    first.retract(step * da),
                                    second.retract(step * db)) -
                                native.evaluateError(
                                    first.retract(-step * da),
                                    second.retract(-step * db))) /
                               (2 * step);
    between.evaluateError(first, second, &a, &b);
    check(a * da + b * db, directional, 5e-7, "Joint between direction");
  }
}
}  // namespace

int main() {
  try {
    if (!settings::sharedSettings)
      throw std::runtime_error("Missing production settings owner");
    (void)interface::InterfaceRerun::get_instance();
    identity_and_limits();
    factor_and_adjoint_trials();
    std::cout
        << checks
        << " analytic checks passed; worst relative log/factor FD discrepancy "
        << worstLog << '/' << worstFactor << '\n';
  } catch (const std::exception& error_) {
    std::cerr << error_.what() << '\n';
    return 1;
  }
}
