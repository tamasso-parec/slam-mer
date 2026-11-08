/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @brief Factor derivatives for the repository's patched physical-translation Sim(3).
#ifndef SLAM_MER_SIM3_FACTOR_DERIVATIVES_HPP
#define SLAM_MER_SIM3_FACTOR_DERIVATIVES_HPP

#include <gtsam/geometry/Similarity3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace slam_mer::covisibility_graph {

/// @brief Evaluate the existing inverse action with its actual right-chart derivatives.
/// @details For pc = R^T(P-t)/s, J_pose = [skew(pc), -I, -pc] and
/// J_point = R^T/s. This bypasses the inconsistent upstream adjoint/Jacobians
/// retained by the repository's Similarity3 patch, without changing its values.
/// @param pose_ Camera-to-world similarity using physical translation t.
/// @param point_ World point.
/// @param poseJacobian_ Optional 3x7 right-tangent derivative.
/// @param pointJacobian_ Optional 3x3 world-coordinate derivative.
/// @return Camera-coordinate point, evaluated by the existing patched action.
inline gtsam::Point3 sim3_camera_point(
    const gtsam::Similarity3& pose_,
    const gtsam::Point3& point_,
    gtsam::OptionalMatrixType poseJacobian_ = nullptr,
    gtsam::OptionalMatrixType pointJacobian_ = nullptr) {
  const auto camera = pose_.inverse().transformFrom(point_);
  if (poseJacobian_) {
    poseJacobian_->resize(3, 7);
    *poseJacobian_ << gtsam::skewSymmetric(camera), -gtsam::Matrix3::Identity(),
        -camera;
  }
  if (pointJacobian_)
    *pointJacobian_ = pose_.rotation().matrix().transpose() / pose_.scale();
  return camera;
}

/// @brief Physical-translation adjoint in the actual right-tangent coordinates.
/// @param pose_ Similarity whose conjugation is differentiated at identity.
/// @return Adjoint of (R,t,s), with blocks [R,0,0; [t]R,sR,-t; 0,0,1].
inline gtsam::Matrix77 sim3_adjoint(const gtsam::Similarity3& pose_) {
  const auto rotation = pose_.rotation().matrix();
  const auto translation = pose_.translation();
  gtsam::Matrix77 adjoint = gtsam::Matrix77::Zero();
  adjoint.block<3, 3>(0, 0) = rotation;
  adjoint.block<3, 3>(3, 0) = gtsam::skewSymmetric(translation) * rotation;
  adjoint.block<3, 3>(3, 3) = pose_.scale() * rotation;
  adjoint.block<3, 1>(3, 6) = -translation;
  adjoint(6, 6) = 1;
  return adjoint;
}

/// @brief Evaluate the pinned Logmap with its analytic right-chart derivative.
/// @details Differentiates the pinned piecewise V kernel, including its small
/// log-scale polynomial, rather than assuming the standard group exponential.
/// The rotation logarithm must be differentiable (away from its pi branch cut).
/// No finite differences, perturbation evaluations or regularization are used.
/// @param pose_ Similarity at which to evaluate the existing Logmap.
/// @param jacobian_ Optional 7x7 derivative under pose_.retract(delta).
/// @return Unchanged GTSAM Logmap value.
gtsam::Vector7 sim3_logmap(
    const gtsam::Similarity3& pose_,
    gtsam::OptionalMatrixType jacobian_ = nullptr);

/// @brief Analytic transport from additive chart coordinates to a local tangent.
/// @details Differentiates Local(Expmap(delta_), Expmap(delta_ + e)) at e=0
/// for the actual patched chart. Left composition by any anchor cancels. This
/// is not a standard Sim(3) exponential Jacobian: the pinned V kernel is used.
/// @param delta_ Optimizer delta at which to transport, ordered (w,v,log scale).
/// @return 7x7 derivative preserving rotation/translation/scale cross terms.
gtsam::Matrix77 sim3_retract_jacobian(const gtsam::Vector7& delta_);

}  // namespace slam_mer::covisibility_graph
#endif  // SLAM_MER_SIM3_FACTOR_DERIVATIVES_HPP
