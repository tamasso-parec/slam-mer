/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
#include <sim3_factor_derivatives.hpp>

#include <gtsam/geometry/SO3.h>

#include <cmath>

namespace slam_mer::covisibility_graph {
namespace {

/// @brief Stable derivative of sum_n (-lambda)^n/(n+order)! near zero.
/// @details This convergent analytic series avoids cancellation in the closed
/// forms for P', beta' and mu'. Eighteen terms suffice for |lambda| < 0.1.
double kernel_scale_derivative(double lambda_, int order_) {
  double factorial = 1;
  for (int j = 2; j <= order_ + 1; ++j)
    factorial *= j;
  double term = -1 / factorial;
  double sum = term;
  for (int n = 1; n < 18; ++n) {
    term *= -lambda_ * (n + 1) / (n * (n + order_ + 1));
    sum += term;
  }
  return sum;
}

/// @brief Differentiate the pinned rotation logarithm, including its pi approximation.
/// @details The matrix implementation reports a standard inverse Jacobian even
/// where its value uses an approximate pi branch. Differentiate that value.
gtsam::Matrix3 rotation_logmap_derivative(const gtsam::Rot3& rotation_) {
#ifdef GTSAM_USE_QUATERNIONS
  gtsam::Matrix3 jacobian;
  gtsam::Rot3::Logmap(rotation_, jacobian);
  return jacobian;
#else
  const gtsam::Matrix3 matrix = rotation_.matrix();
  gtsam::Matrix3 jacobian;
  if (matrix.trace() + 1 < 1e-3) {
    const int a = matrix(2, 2) > matrix(1, 1) && matrix(2, 2) > matrix(0, 0)
                      ? 2
                      : (matrix(1, 1) > matrix(0, 0) ? 1 : 0);
    const int b = (a + 1) % 3, c = (a + 2) % 3;
    gtsam::Vector3 q;
    q(a) = 2 + 2 * matrix(a, a);
    q(b) = matrix(b, a) + matrix(a, b);
    q(c) = matrix(c, a) + matrix(a, c);
    const double w = matrix(c, b) - matrix(b, c);
    const double norm = std::sqrt(q.squaredNorm() + w * w);
    const double sign = w < 0 ? -1 : 1;
    const double magnitude = std::acos(-1.) - 2 * sign * w / norm;
    const double coefficient = .5 * sign / std::sqrt(q(a));
    for (int j = 0; j < 3; ++j) {
      const gtsam::Matrix3 d =
          matrix * gtsam::skewSymmetric(gtsam::Vector3::Unit(j));
      gtsam::Vector3 dq;
      dq(a) = 2 * d(a, a);
      dq(b) = d(b, a) + d(a, b);
      dq(c) = d(c, a) + d(a, c);
      const double dw = d(c, b) - d(b, c);
      const double dnorm = (q.dot(dq) + w * dw) / norm;
      const double dmagnitude =
          -2 * sign * (dw / norm - w * dnorm / (norm * norm));
      jacobian.col(j) =
          coefficient *
          (magnitude * dq + (dmagnitude - .5 * magnitude * dq(a) / q(a)) * q);
    }
  } else {
    const double traceMinusThree = matrix.trace() - 3;
    double magnitude, derivative;
    if (traceMinusThree < -1e-6) {
      const double theta = std::acos((matrix.trace() - 1) / 2);
      const double sine = std::sin(theta);
      magnitude = theta / (2 * sine);
      derivative = -(sine - theta * std::cos(theta)) / (4 * sine * sine * sine);
    } else {
      magnitude =
          .5 - traceMinusThree / 12 + traceMinusThree * traceMinusThree / 60;
      derivative = -1. / 12 + traceMinusThree / 30;
    }
    const gtsam::Vector3 skew(
        matrix(2, 1) - matrix(1, 2),
        matrix(0, 2) - matrix(2, 0),
        matrix(1, 0) - matrix(0, 1));
    for (int j = 0; j < 3; ++j) {
      const gtsam::Matrix3 d =
          matrix * gtsam::skewSymmetric(gtsam::Vector3::Unit(j));
      const gtsam::Vector3 dskew(
          d(2, 1) - d(1, 2),
          d(0, 2) - d(2, 0),
          d(1, 0) - d(0, 1));
      jacobian.col(j) = magnitude * dskew + derivative * d.trace() * skew;
    }
  }
  return jacobian;
#endif
}

/// @brief V and derivatives of the pinned Similarity3.cpp LocalV formulas.
/// @details Radial derivatives mean (d/dtheta)/theta. The branch thresholds and
/// polynomial values deliberately match the dependency; changing that kernel
/// would change the residual model and is outside this derivative correction.
struct Sim3ChartKernel {
  gtsam::Matrix3 value;
  gtsam::Matrix3 scaleDerivative;
  double q, r, radialQ, radialR;

  Sim3ChartKernel(const gtsam::so3::DexpFunctor& rotation_, double lambda_) {
    const double lambda2 = lambda_ * lambda_;
    double p, dp, dq, dr;
    // The dependency's near-zero B/C/E values are truncated polynomials. Use
    // their exact polynomial derivatives rather than DexpFunctor's higher
    // order derivative approximations.
    const double radialB = rotation_.nearZero ? -1. / 12 : rotation_.dB();
    const double radialC = rotation_.nearZero ? -1. / 60 : rotation_.dC();
    const double radialE = rotation_.nearZero ? -1. / 360 : rotation_.dE();
    const double c = rotation_.C(), e = rotation_.E();
    if (lambda2 > 1e-9) {
      const double exponential = std::exp(-lambda_);
      p = (1 - exponential) / lambda_;
      const double beta = (exponential - 1 + lambda_) / lambda2;
      const double mu =
          (1 - lambda_ + .5 * lambda2 - exponential) / (lambda2 * lambda_);
      const double alpha = 1 / (1 + rotation_.theta2 / lambda2);
      const double complement = 1 - alpha;
      const double radialAlpha = -2 * alpha * alpha / lambda2;
      const double scaleAlpha = 2 * alpha * complement / lambda_;
      double dbeta, dmu;
      if (std::abs(lambda_) < .1) {
        dp = kernel_scale_derivative(lambda_, 1);
        dbeta = kernel_scale_derivative(lambda_, 2);
        dmu = kernel_scale_derivative(lambda_, 3);
      } else {
        dp = (exponential - p) / lambda_;
        dbeta = (p - 2 * beta) / lambda_;
        dmu = (beta - 3 * mu) / lambda_;
      }
      const double baseQ = rotation_.B - lambda_ * c;
      const double baseR = c - lambda_ * e;
      q = alpha * beta + complement * baseQ;
      r = alpha * mu + complement * baseR;
      radialQ = radialAlpha * (beta - baseQ) +
                complement * (radialB - lambda_ * radialC);
      radialR = radialAlpha * (mu - baseR) +
                complement * (radialC - lambda_ * radialE);
      dq = scaleAlpha * (beta - baseQ) + alpha * dbeta - complement * c;
      dr = scaleAlpha * (mu - baseR) + alpha * dmu - complement * e;
    } else {
      p = 1 - lambda_ / 2 + lambda2 / 6;
      dp = -.5 + lambda_ / 3;
      q = rotation_.B - lambda_ * c;
      r = c - lambda_ * e;
      radialQ = radialB - lambda_ * radialC;
      radialR = radialC - lambda_ * radialE;
      dq = -c;
      dr = -e;
    }
    value = p * gtsam::Matrix3::Identity() + q * rotation_.W + r * rotation_.WW;
    scaleDerivative =
        dp * gtsam::Matrix3::Identity() + dq * rotation_.W + dr * rotation_.WW;
  }
};
}  // namespace

gtsam::Vector7 sim3_logmap(
    const gtsam::Similarity3& pose_,
    gtsam::OptionalMatrixType jacobian_) {
  const gtsam::Vector7 logarithm = gtsam::Similarity3::Logmap(pose_);
  if (!jacobian_) return logarithm;

  const gtsam::Vector3 omega = logarithm.head<3>();
  const gtsam::Vector3 rho = logarithm.segment<3>(3);
  const gtsam::so3::DexpFunctor rotation(omega);
  const Sim3ChartKernel kernel(rotation, logarithm(6));
  const gtsam::Matrix3 inverseV = kernel.value.inverse();
  const gtsam::Matrix3 rotationLog =
      rotation_logmap_derivative(pose_.rotation());
  gtsam::Matrix3 translationRotation;
  // Differentiate V*rho=t: d rho = V^-1 (d t - d V*rho).
  for (int j = 0; j < 3; ++j) {
    const gtsam::Matrix3 direction =
        gtsam::skewSymmetric(gtsam::Vector3::Unit(j));
    const gtsam::Matrix3 derivative =
        kernel.q * direction +
        kernel.r * (direction * rotation.W + rotation.W * direction) +
        omega(j) * (kernel.radialQ * rotation.W + kernel.radialR * rotation.WW);
    translationRotation.col(j) = -inverseV * derivative * rho;
  }
  jacobian_->setZero(7, 7);
  jacobian_->block<3, 3>(0, 0) = rotationLog;
  jacobian_->block<3, 3>(3, 0) = translationRotation * rotationLog;
  // The actual right chart has dt = s R dv and d(log s) = d lambda at zero.
  jacobian_->block<3, 3>(3, 3) =
      inverseV * pose_.scale() * pose_.rotation().matrix();
  jacobian_->block<3, 1>(3, 6) = -inverseV * kernel.scaleDerivative * rho;
  (*jacobian_)(6, 6) = 1;
  return logarithm;
}
gtsam::Matrix77 sim3_retract_jacobian(const gtsam::Vector7& delta_) {
  const gtsam::Vector3 omega = delta_.head<3>();
  const gtsam::Vector3 rho = delta_.segment<3>(3);
  const gtsam::so3::DexpFunctor rotation(omega);
  const Sim3ChartKernel kernel(rotation, delta_(6));
  const gtsam::Matrix3 rotationMatrix = gtsam::Rot3::Expmap(omega).matrix();
  const gtsam::Matrix3 translationToLocal =
      std::exp(-delta_(6)) * rotationMatrix.transpose();
  gtsam::Matrix77 jacobian = gtsam::Matrix77::Zero();
  jacobian.block<3, 3>(0, 0) = rotation.rightJacobian();
#ifndef GTSAM_USE_QUATERNIONS
  if (rotation.nearZero) {
    // Differentiate the actual truncated Rodrigues polynomial in this branch.
    for (int j = 0; j < 3; ++j) {
      const gtsam::Matrix3 direction =
          gtsam::skewSymmetric(gtsam::Vector3::Unit(j));
      const gtsam::Matrix3 derivative =
          rotation.A * direction +
          rotation.B * (direction * rotation.W + rotation.W * direction) -
          omega(j) * (rotation.W / 3 + rotation.WW / 12);
      const gtsam::Matrix3 local = rotationMatrix.transpose() * derivative;
      jacobian.block<3, 1>(0, j) = .5 * gtsam::Vector3(
                                            local(2, 1) - local(1, 2),
                                            local(0, 2) - local(2, 0),
                                            local(1, 0) - local(0, 1));
    }
  }
#endif
  for (int j = 0; j < 3; ++j) {
    const gtsam::Matrix3 direction =
        gtsam::skewSymmetric(gtsam::Vector3::Unit(j));
    const gtsam::Matrix3 derivative =
        kernel.q * direction +
        kernel.r * (direction * rotation.W + rotation.W * direction) +
        omega(j) * (kernel.radialQ * rotation.W + kernel.radialR * rotation.WW);
    jacobian.block<3, 1>(3, j) = translationToLocal * derivative * rho;
  }
  jacobian.block<3, 3>(3, 3) = translationToLocal * kernel.value;
  jacobian.block<3, 1>(3, 6) =
      translationToLocal * kernel.scaleDerivative * rho;
  jacobian(6, 6) = 1;
  return jacobian;
}

}  // namespace slam_mer::covisibility_graph
