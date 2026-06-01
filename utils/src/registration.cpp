/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file registration.cpp
/// @brief Rigid registration implementation for matched 3D points.
/// @details Implements alignment routines that compute transform and scale from
/// correspondence sets. Used to align point subsets across frames or
/// maps.
/// @author Pedro Miraldo
/// @author Valter Piedade

#include <registration.hpp>

// Thirdparty includes

namespace slam_mer {
namespace registration {

namespace {

/// Inputs of the registration solver
using RegistrationInput = std::tuple<
    Eigen::MatrixXd,
    Eigen::MatrixXd,
    std::vector<Eigen::Vector3d>,
    double>;
/// Outputs of the registration solver
using RegistrationOutput = std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>;

}  // namespace

namespace utils {

/// @brief Converts a vector of point3d_t to an Eigen::MatrixXd
/// @param points3d_ Input 3D points in SLAM-MER representation.
/// @param points3dEigen_ Output 3xN matrix populated with point coordinates.
void __convert_points3d_to_eigen(
    const point3d_t::vector& points3d_,
    Eigen::MatrixXd& points3dEigen_) {
  using Eigen::MatrixXd;

  points3dEigen_ = MatrixXd::Zero(3, points3d_.size());
  for (size_t i = 0; i < points3d_.size(); ++i) {
    const array3D_t pointArray = points3d_[i];
    points3dEigen_(0, i) = pointArray[0];
    points3dEigen_(1, i) = pointArray[1];
    points3dEigen_(2, i) = pointArray[2];
  }
}

/// @brief Prepares the inputs for the registration solver
/// @param points3dSource_ Source-point matrix used in registration.
/// @param points3dTarget_ Target-point matrix used in registration.
/// @param input_ Output tuple with centered/scaled solver inputs.
void __prepare_inputs(
    const Eigen::MatrixXd& points3dSource_,
    const Eigen::MatrixXd& points3dTarget_,
    RegistrationInput& input_) {
  using Eigen::MatrixXd;
  using Eigen::Vector3d;

  const size_t dataSize = points3dSource_.cols();
  double scale = 0.0;
  double max_scale = 0.0;
  MatrixXd sourcePoints = points3dSource_;
  MatrixXd targetPoints = points3dTarget_;
  std::vector<Vector3d> centroids;

  /////////////////////////////////
  // Compute mean of points3dSource
  Vector3d meanSourcePoints = Vector3d::Zero();
  for (size_t i = 0; i < dataSize; i++) {
    Vector3d p(points3dSource_.col(i));
    meanSourcePoints = meanSourcePoints + p;
  }
  meanSourcePoints = meanSourcePoints / dataSize;
  centroids.push_back(meanSourcePoints);
  for (size_t i = 0; i < dataSize; i++) {
    sourcePoints(0, i) -= meanSourcePoints(0);
    sourcePoints(1, i) -= meanSourcePoints(1);
    sourcePoints(2, i) -= meanSourcePoints(2);
  }
  // compute scale
  for (size_t i = 0; i < dataSize; i++) {
    Vector3d p(sourcePoints.col(i));
    // because we extract mean in the previous stage.
    double temp = p.squaredNorm();
    if (temp > max_scale) max_scale = temp;
  }
  scale = max_scale;

  /////////////////////////////////
  // Compute mean of points3dTarget
  Vector3d meanTargetPoints = Vector3d::Zero();
  // max_scale = 0.0f;
  for (size_t i = 0; i < dataSize; i++) {
    Vector3d p(points3dTarget_.col(i));
    meanTargetPoints = meanTargetPoints + p;
  }
  meanTargetPoints = meanTargetPoints / dataSize;
  centroids.push_back(meanTargetPoints);
  for (size_t i = 0; i < dataSize; i++) {
    targetPoints(0, i) -= meanTargetPoints(0);
    targetPoints(1, i) -= meanTargetPoints(1);
    targetPoints(2, i) -= meanTargetPoints(2);
  }
  // compute scale
  for (size_t i = 0; i < dataSize; i++) {
    Vector3d p(targetPoints.col(i));
    double temp = p.squaredNorm();
    if (temp > max_scale) max_scale = temp;
  }
  if (max_scale > scale) scale = max_scale;

  // for (size_t i = 0; i < dataSize; i++) {
  //   sourcePoints(0, i) /= scale;
  //   sourcePoints(1, i) /= scale;
  //   sourcePoints(2, i) /= scale;

  //   targetPoints(0, i) /= scale;
  //   targetPoints(1, i) /= scale;
  //   targetPoints(2, i) /= scale;
  // }

  input_ = std::make_tuple(sourcePoints, targetPoints, centroids, 1);
  return;
}

}  // namespace utils

/// @brief Solves the 3D registration problem
/// @param input_ Preprocessed solver inputs (source, target, centroids, scale).
/// @param output_ Output tuple with rotation, translation, and scale.
/// @return `true` when a solution is produced successfully.
bool __solve_registration(
    const RegistrationInput& input_,
    RegistrationOutput& output_) {
  using Eigen::ComputeFullU;
  using Eigen::ComputeFullV;
  using Eigen::JacobiSVD;
  using Eigen::Matrix3d;
  using Eigen::MatrixXd;
  using Eigen::Vector3d;
  using Eigen::VectorXd;

  MatrixXd X, Y;
  std::vector<Vector3d> centroids;
  double inputScale;
  std::tie(X, Y, centroids, inputScale) = input_;

  const int dataSize = X.cols();
  const int dataDim = X.rows();
  const VectorXd weights = VectorXd::Constant(dataSize, 1, 1);
  double cn = weights.sum();

  VectorXd mu_x =
      ((Y.array().rowwise()) * (weights.transpose().array())).rowwise().sum() /
      cn;

  VectorXd mu_y =
      ((X.array().rowwise()) * (weights.transpose().array())).rowwise().sum() /
      cn;

  Matrix3d sig_xy =
      ((X.array().rowwise()) * (weights.transpose().array())).matrix() *
          (Y.transpose()) / cn -
      mu_y * (mu_x.transpose());

  // Computing SVD
  JacobiSVD<Matrix3d> svd(sig_xy, ComputeFullU | ComputeFullV);

  VectorXd S = VectorXd::Ones(dataDim);
  if (((svd.matrixU() * svd.matrixV()).determinant()) < 0) {
    S(dataDim - 1) = -1;
  }

  // Compute rotation
  MatrixXd M = MatrixXd::Identity(dataDim + 1, dataDim + 1);
  M.block(0, 0, dataDim, dataDim).noalias() =
      svd.matrixU() * S.asDiagonal() * (svd.matrixV().transpose());

  double scale;
  if (false) {
    // Compute scale
    double sig_x2 =
        weights.dot(((((Y.colwise() - mu_x).array().square()).colwise().sum())
                         .matrix())) /
        cn;
    scale = (svd.singularValues().dot(S)) / sig_x2;
  } else {
    scale = 1.0;
  }
  M.col(dataDim).head(dataDim) =
      mu_y - scale * M.topLeftCorner(dataDim, dataDim) * mu_x;

  const Matrix3d rotation = M.block<3, 3>(0, 0);
  Vector3d translation = M.block<3, 1>(0, 3);
  translation =
      -rotation * centroids[1] + translation * inputScale + centroids[0];

  output_ = std::make_tuple(rotation, translation, scale);
  return true;
}

// ----------------------------------------------------------------------------
// Computes registration between two sparse vector of matches between point
// clouds
// ----------------------------------------------------------------------------
bool compute_estimate(
    const point3d_t::vector& sourcePoints_,
    const point3d_t::vector& targetPoints_,
    pose_t& estimate_,
    double& scale_,
    point3d_t::vector& centroid_) {
  using Eigen::Matrix3d;
  using Eigen::MatrixXd;
  using Eigen::Vector3d;

  // convert from point3d_t to Eigen::MatrixXd
  MatrixXd points3dSource;
  MatrixXd points3dTarget;
  utils::__convert_points3d_to_eigen(sourcePoints_, points3dSource);
  utils::__convert_points3d_to_eigen(targetPoints_, points3dTarget);

  // preprocess and solve registration
  RegistrationInput inputs;
  RegistrationOutput outputs;
  utils::__prepare_inputs(points3dTarget, points3dSource, inputs);
  const bool estimationSuccess = __solve_registration(inputs, outputs);

  // Sets the output
  const auto [rotation, translation, scaleOutput] = outputs;
  const std::vector<Vector3d> centroidEigen = std::get<2>(inputs);
  estimate_ = pose_t(rotation, translation);
  scale_ = scaleOutput;
  centroid_.push_back(point3d_t(centroidEigen[0]));
  centroid_.push_back(point3d_t(centroidEigen[1]));

  return estimationSuccess;
}

// ----------------------------------------------------------------------------
// Computes the best scalar 's' to align sourcePoints to targetPoints using
// similarity transformation.
// ----------------------------------------------------------------------------
double compute_best_scalar_s(
    const point3d_t::vector& sourcePoints_,
    const point3d_t::vector& targetPoints_) {

  if (sourcePoints_.size() != targetPoints_.size())
    __throw__("Mismatched number of x and y vectors.");

  double numerator = 0.0;
  double denominator = 0.0;
  for (size_t i = 0; i < sourcePoints_.size(); ++i) {
    const array3D_t sourcePoint = sourcePoints_[i];
    const array3D_t targetPoint = targetPoints_[i];
    numerator += (sourcePoint[0] * targetPoint[0]) +
                 (sourcePoint[1] * targetPoint[1]) +
                 (sourcePoint[2] * targetPoint[2]);
    denominator += (targetPoint[0] * targetPoint[0]) +
                   (targetPoint[1] * targetPoint[1]) +
                   (targetPoint[2] * targetPoint[2]);
  }

  if (denominator == 0.0) __throw__("Denominator is zero — cannot compute s.");
  return numerator / denominator;
}

}  // namespace registration
}  // namespace slam_mer
