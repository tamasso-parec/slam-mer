/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <atomic>
#include <gaussian_posterior.hpp>
#include <iostream>
#include <random>
#include <thread>

using namespace gtsam;
using namespace slam_mer::covisibility_graph;
using Status = GaussianPosterior::Status;
static size_t checks = 0;
void check(bool value_, const char* message_) {
  ++checks;
  if (!value_) throw std::runtime_error(message_);
}
void close(const Matrix& a_, const Matrix& b_, double tolerance_ = 1e-9) {
  check(a_.rows() == b_.rows() && a_.cols() == b_.cols(), "shape mismatch");
  check(
      (a_ - b_).norm() <= tolerance_ * std::max(1., b_.norm()),
      "numeric mismatch");
}
GaussianPosterior::shared_ptr capture(ISAM2& isam_, uint64_t revision_) {
  return GaussianPosterior::capture(
      isam_,
      isam_.calculateEstimate(),
      revision_);
}
// Independent information assembly from the maintained conditionals, not from
// a fresh nonlinear linearization. Dense inverse only in tiny reference tests.
Matrix maintained_reference(const ISAM2& isam_, const Ordering& order_) {
  GaussianFactorGraph graph;
  std::set<const void*> seen;
  for (const auto& [key, clique] : isam_.nodes())
    if (seen.insert(clique.get()).second)
      graph.push_back(clique->conditional());
  const auto hessian = graph.hessian(order_);
  return hessian.first.ldlt().solve(
      Matrix::Identity(hessian.first.rows(), hessian.first.rows()));
}
void linear_tests() {
  // Three coupled Point3 variables with an explicitly assembled full-rank A,b.
  Matrix a = Matrix::Identity(9, 9);
  a.block<3, 3>(0, 3) = 0.4 * Matrix3::Identity();
  a.block<3, 3>(0, 6) = -0.2 * Matrix3::Identity();
  a.block<3, 3>(3, 6) = 0.7 * Matrix3::Identity();
  Vector b = Vector::LinSpaced(9, -0.3, 0.6);
  std::vector<std::pair<Key, Matrix>> terms{
      {1, a.leftCols(3)},
      {2, a.middleCols(3, 3)},
      {3, a.rightCols(3)}};
  auto factor = std::make_shared<JacobianFactor>(terms, b);
  NonlinearFactorGraph graph;
  graph.push_back(std::make_shared<LinearContainerFactor>(factor));
  Values values;
  for (Key key : {1, 2, 3})
    values.insert(key, Point3::Zero().eval());
  ISAM2 isam;
  isam.update(graph, values);
  const auto estimate = isam.calculateEstimate();
  auto snapshot = capture(isam, 1);
  check(snapshot->status() == Status::available, "linear capture");
  const Matrix covariance =
      (a.transpose() * a).ldlt().solve(Matrix::Identity(9, 9));
  close(snapshot->joint_covariance({1, 2, 3}).covariance, covariance);
  close(
      snapshot->gaussian_mean().vector(Ordering{1, 2, 3}),
      a.fullPivLu().solve(b));
  for (Key i : {1, 2, 3}) {
    close(
        snapshot->joint_covariance({i}).covariance,
        covariance.block(3 * (i - 1), 3 * (i - 1), 3, 3));
    close(
        snapshot->joint_covariance({i}).covariance,
        isam.marginalCovariance(i));
    for (Key j : {1, 2, 3})
      if (i != j) {
        auto joint = snapshot->joint_covariance({i, j});
        close(
            joint.covariance.topRightCorner(3, 3),
            covariance.block(3 * (i - 1), 3 * (j - 1), 3, 3));
        close(joint.covariance, joint.covariance.transpose());
        check(
            joint.keys == KeyVector({i, j}) &&
                joint.dimensions == std::vector<size_t>({3, 3}),
            "query order");
      }
  }
  close(
      isam.calculateEstimate().localCoordinates(estimate).vector(),
      Vector::Zero(9));
  check(
      snapshot->joint_covariance({9}).status == Status::unavailable,
      "missing key");
  check(
      snapshot->joint_covariance({1, 1}).status == Status::unavailable,
      "duplicate key");
  check(
      snapshot->joint_covariance({}).covariance.size() == 0,
      "empty selection");
  isam.update();
  close(
      capture(isam, 2)->joint_covariance({3, 1, 2}).covariance,
      snapshot->joint_covariance({3, 1, 2}).covariance);
  PosteriorPublication publication;
  check(!publication.latest(), "initial publication");
  publication.publish(
      GaussianPosterior::capture(isam, estimate, 1, publication.epoch()));
  std::atomic<bool> stop{false};
  std::atomic<bool> valid{true};
  std::thread reader([&] {
    while (!stop) {
      auto current = publication.latest();
      auto result = current->joint_covariance({2, 1});
      if (result.revision != current->revision() ||
          result.status != Status::available)
        valid = false;
    }
  });
  for (uint64_t revision = 2; revision < 30; ++revision)
    publication.publish(
        GaussianPosterior::capture(
            isam,
            isam.calculateEstimate(),
            revision,
            publication.epoch()));
  stop = true;
  reader.join();
  check(valid, "concurrent publication");
  close(snapshot->joint_covariance({1, 2, 3}).covariance, covariance);
  bool rejected = false;
  try {
    publication.publish(
        GaussianPosterior::capture(
            isam,
            isam.calculateEstimate(),
            1,
            publication.epoch()));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  check(rejected, "stale attachment");
  // Factor replacement changes covariance; old snapshot stays readable.
  NonlinearFactorGraph additional;
  additional.add(
      PriorFactor<Point3>(
          1,
          Point3(1, 2, 3),
          noiseModel::Isotropic::Sigma(3, 0.2)));
  isam.update(additional, Values());
  auto replaced = capture(isam, 30);
  close(
      replaced->joint_covariance({1, 2, 3}).covariance,
      maintained_reference(isam, Ordering{1, 2, 3}));
  check(
      (replaced->joint_covariance({1}).covariance -
       snapshot->joint_covariance({1}).covariance)
              .norm() > 0.1,
      "cache revision invalidation");
  isam.update(NonlinearFactorGraph(), Values(), FactorIndices{1});
  close(capture(isam, 31)->joint_covariance({1, 2, 3}).covariance, covariance);
}

void pose_tests() {
  ISAM2Params parameters;
  parameters.relinearizeSkip = 1;
  parameters.relinearizeThreshold = 0.01;
  ISAM2 isam(parameters);
  NonlinearFactorGraph graph;
  const Pose3 p0(Rot3::RzRyRx(0.1, -0.1, 0.2), Point3(0.2, 0.1, 0));
  const Pose3 p1(Rot3::RzRyRx(-0.1, 0.2, 0.3), Point3(1, 0.2, 0.1));
  const Point3 point(0.5, 0.4, 4), point2(1, 0.6, 5);
  auto calibration = std::make_shared<Cal3_S2>(300, 300, 0, 320, 240);
  graph.add(PriorFactor<Pose3>(10, p0, noiseModel::Isotropic::Sigma(6, 0.1)));
  graph.add(
      BetweenFactor<Pose3>(
          10,
          11,
          p0.between(p1),
          noiseModel::Isotropic::Sigma(6, 0.2)));
  graph.add(
      PriorFactor<Point3>(20, point, noiseModel::Isotropic::Sigma(3, 0.3)));
  graph.add(
      PriorFactor<Point3>(21, point2, noiseModel::Isotropic::Sigma(3, 0.4)));
  for (const auto& [key, pose] :
       std::vector<std::pair<Key, Pose3>>{{10, p0}, {11, p1}})
    for (const auto& [pointKey, value] :
         std::vector<std::pair<Key, Point3>>{{20, point}, {21, point2}}) {
      const auto measurement =
          PinholeCamera<Cal3_S2>(pose, *calibration).project(value);
      graph.add(
          GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
              measurement,
              noiseModel::Robust::Create(
                  noiseModel::mEstimator::Huber::Create(1.2),
                  noiseModel::Isotropic::Sigma(2, 1)),
              key,
              pointKey,
              calibration));
    }
  Values values;
  values.insert(10, p0.retract(Vector6::Constant(0.02)));
  values.insert(11, p1.retract(Vector6::Constant(-0.03)));
  values.insert(20, point + Point3(0.1, -0.1, 0.2));
  values.insert(21, point2 + Point3(-0.1, 0.1, 0.1));
  isam.update(graph, values);
  auto snapshot = capture(isam, 1);
  check(snapshot->status() == Status::available, "pose capture");
  const Ordering ordering{10, 11, 20, 21};
  close(
      snapshot->joint_covariance(ordering).covariance,
      maintained_reference(isam, ordering),
      1e-8);
  // This comparison is a fresh linearization at initial anchors, valid on the
  // first update only. It is deliberately separate from maintained_reference.
  auto fresh = graph.linearize(snapshot->anchors())->hessian(ordering).first;
  close(
      snapshot->joint_covariance(ordering).covariance,
      fresh.ldlt().solve(Matrix::Identity(18, 18)),
      1e-8);
  const auto atEstimateInformation =
      graph.linearize(snapshot->estimates())->hessian(ordering).first;
  const Matrix atEstimateCovariance =
      atEstimateInformation.ldlt().solve(Matrix::Identity(18, 18));
  check(
      (atEstimateCovariance - snapshot->joint_covariance(ordering).covariance)
              .norm() > 1e-6,
      "fresh estimated-state Gaussian must remain distinct from maintained "
      "Gaussian");
  for (Key key : ordering)
    close(
        snapshot->joint_covariance({key}).covariance,
        isam.marginalCovariance(key),
        1e-8);
  auto transported = snapshot->joint_covariance_at_estimate(ordering);
  check(transported.status == Status::available, "coordinate transport");
  close(transported.covariance, transported.covariance.transpose());
  // Independent central differences check the analytic coordinate Jacobian,
  // including pose-point and pose-pose cross blocks.
  Matrix numerical = Matrix::Identity(18, 18);
  size_t offset = 0;
  for (Key key : KeyVector{10, 11}) {
    const auto anchor = snapshot->anchors().at<Pose3>(key);
    const auto estimate = snapshot->estimates().at<Pose3>(key);
    const auto delta = snapshot->estimate_delta().at(key);
    for (int j = 0; j < 6; ++j) {
      Vector plus = delta, minus = delta;
      plus(j) += 1e-6;
      minus(j) -= 1e-6;
      numerical.block(offset, offset + j, 6, 1) =
          (estimate.localCoordinates(anchor.retract(plus)) -
           estimate.localCoordinates(anchor.retract(minus))) /
          2e-6;
    }
    offset += 6;
  }
  const auto raw = snapshot->joint_covariance(ordering);
  close(
      transported.covariance,
      numerical * raw.covariance * numerical.transpose(),
      1e-8);
  close(
      transported.mean,
      numerical * (raw.mean - snapshot->estimate_delta().vector(ordering)),
      1e-8);
  // Monte Carlo validates the *joint* retraction transport at small noise.
  const Matrix base = snapshot->joint_covariance({10, 20}).covariance;
  const Matrix target =
      snapshot->joint_covariance_at_estimate({10, 20}).covariance;
  const Matrix l = base.llt().matrixL();
  std::mt19937 random(9182);
  std::normal_distribution<double> normal;
  Matrix empirical = Matrix::Zero(9, 9);
  Vector sum = Vector::Zero(9);
  const auto anchor = snapshot->anchors().at<Pose3>(10);
  const auto estimate = snapshot->estimates().at<Pose3>(10);
  const auto delta = snapshot->estimate_delta().at(10);
  for (int i = 0; i < 30000; ++i) {
    Vector z(9);
    for (int j = 0; j < 9; ++j)
      z(j) = normal(random);
    Vector sample = 0.001 * l * z;
    sample.head(6) =
        estimate.localCoordinates(anchor.retract(delta + sample.head(6)));
    sum += sample;
    empirical += sample * sample.transpose();
  }
  empirical =
      (empirical / 30000 - (sum / 30000) * (sum / 30000).transpose()) / 1e-6;
  close(empirical, target, 0.025);
  // Relative-pose and inverse Jacobians use GTSAM's right local coordinates.
  const auto firstPose = snapshot->estimates().at<Pose3>(10);
  const auto secondPose = snapshot->estimates().at<Pose3>(11);
  Matrix66 h1, h2, inverseH;
  const auto relative = firstPose.between(secondPose, h1, h2);
  const auto inverse = firstPose.inverse(inverseH);
  Matrix relativeJ(6, 12);
  relativeJ << h1, h2;
  Matrix numericRelative(6, 12), numericInverse(6, 6);
  for (int j = 0; j < 12; ++j) {
    Vector6 step = Vector6::Zero();
    step(j % 6) = 1e-6;
    const Pose3 plus = j < 6 ? firstPose.retract(step).between(secondPose)
                             : firstPose.between(secondPose.retract(step));
    const Pose3 minus = j < 6 ? firstPose.retract(-step).between(secondPose)
                              : firstPose.between(secondPose.retract(-step));
    numericRelative.col(j) =
        (relative.localCoordinates(plus) - relative.localCoordinates(minus)) /
        2e-6;
    if (j < 6)
      numericInverse.col(j) =
          (inverse.localCoordinates(firstPose.retract(step).inverse()) -
           inverse.localCoordinates(firstPose.retract(-step).inverse())) /
          2e-6;
  }
  close(relativeJ, numericRelative, 1e-8);
  close(inverseH, numericInverse, 1e-8);
  const Matrix poseJoint =
      snapshot->joint_covariance_at_estimate({10, 11}).covariance;
  close(
      relativeJ * poseJoint * relativeJ.transpose(),
      numericRelative * poseJoint * numericRelative.transpose(),
      1e-8);
  // World point into an uncertain camera frame: retain the pose-point terms.
  Matrix36 poseH;
  Matrix33 pointH;
  const auto worldPoint = snapshot->estimates().at<Point3>(20);
  firstPose.transformTo(worldPoint, poseH, pointH);
  Matrix frameJ(3, 9);
  frameJ << poseH, pointH;
  Matrix numericFrame(3, 9);
  for (int j = 0; j < 9; ++j) {
    if (j < 6) {
      Vector6 step = Vector6::Zero();
      step(j) = 1e-6;
      numericFrame.col(j) = (firstPose.retract(step).transformTo(worldPoint) -
                             firstPose.retract(-step).transformTo(worldPoint)) /
                            2e-6;
    } else {
      Vector3 step = Vector3::Zero();
      step(j - 6) = 1e-6;
      numericFrame.col(j) = (firstPose.transformTo(Point3(worldPoint + step)) -
                             firstPose.transformTo(Point3(worldPoint - step))) /
                            2e-6;
    }
  }
  close(frameJ, numericFrame, 1e-8);
  close(
      frameJ * target * frameJ.transpose(),
      numericFrame * target * numericFrame.transpose(),
      1e-8);
  isam.update();
  auto next = capture(isam, 2);
  close(
      next->joint_covariance(ordering).covariance,
      maintained_reference(isam, ordering),
      1e-8);
  // Point ID addition: old publication never acquires the new key.
  NonlinearFactorGraph extra;
  extra.add(
      PriorFactor<Point3>(
          22,
          Point3(2, 3, 4),
          noiseModel::Isotropic::Sigma(3, 2)));
  Values newValues;
  newValues.insert(22, Point3(0, 0, 0));
  isam.update(extra, newValues);
  check(
      snapshot->joint_covariance({22}).status == Status::unavailable,
      "old ID set");
  close(
      capture(isam, 3)->joint_covariance({22}).covariance,
      4 * Matrix3::Identity());
}

void edge_tests() {
  ISAM2 empty;
  check(capture(empty, 1)->status() == Status::empty, "empty graph");
  check(capture(empty, 1)->factor_bytes() == 0, "empty factor storage");
  // Audit patch: scale column disagrees with the actual right retraction.
  Similarity3 sim(Rot3(), Point3(1, 2, 3), 2.5);
  Matrix37 analytic;
  sim.transformFrom(Point3(2, 1, 4), analytic);
  Vector7 step = Vector7::Zero();
  step(6) = 1e-6;
  Vector3 numeric = (sim.retract(step).transformFrom(Point3(2, 1, 4)) -
                     sim.retract(-step).transformFrom(Point3(2, 1, 4))) /
                    2e-6;
  check((analytic.col(6) - numeric).norm() > 1., "Sim3 patch audit");
  ISAM2 simIsam;
  NonlinearFactorGraph simGraph;
  simGraph.add(
      PriorFactor<Similarity3>(1, sim, noiseModel::Isotropic::Sigma(7, 0.1)));
  Values simInitial;
  simInitial.insert(1, sim);
  simIsam.update(simGraph, simInitial);
  const auto simPosterior =
      GaussianPosterior::capture(simIsam, simIsam.calculateEstimate(), 1);
  check(simPosterior->factor_bytes() > 0, "Sim3 retained factor storage");
  check(
      simPosterior->status() == Status::available &&
          simPosterior->estimates().size() == 1,
      "Sim3 Gaussian available");
  close(
      simPosterior->joint_covariance({1}).covariance,
      .01 * Matrix77::Identity());
  close(
      simPosterior->joint_covariance_at_estimate({1}).covariance,
      .01 * Matrix77::Identity());
  // Gauge failure is GTSAM's explicit error, never a damped finite posterior.
  bool singular = false;
  try {
    ISAM2 isam;
    NonlinearFactorGraph graph;
    graph.add(
        BetweenFactor<Point3>(
            1,
            2,
            Point3(1, 0, 0),
            noiseModel::Isotropic::Sigma(3, 1)));
    Values values;
    values.insert(1, Point3(0, 0, 0));
    values.insert(2, Point3(1, 0, 0));
    isam.update(graph, values);
    singular = capture(isam, 1)->status() != Status::available;
  } catch (const IndeterminantLinearSystemException&) {
    singular = true;
  }
  check(singular, "gauge handling");
  // Weak but anchored geometry is retained, without a hidden covariance floor.
  NonlinearFactorGraph weak;
  weak.add(
      PriorFactor<Point3>(
          1,
          Point3(0, 0, 0),
          noiseModel::Isotropic::Sigma(3, 1e6)));
  Values value;
  value.insert(1, Point3(0, 0, 0));
  ISAM2 isam;
  isam.update(weak, value);
  close(
      capture(isam, 1)->joint_covariance({1}).covariance,
      1e12 * Matrix3::Identity());
}

void publication_path_tests() {
  ISAM2 isam;
  NonlinearFactorGraph graph;
  graph.add(
      PriorFactor<Point3>(
          1,
          Point3(1, 2, 3),
          noiseModel::Isotropic::Sigma(3, 2)));
  Values initial;
  initial.insert(1, Point3(0, 0, 0));
  isam.update(graph, initial);
  const auto estimates = isam.calculateEstimate();
  PosteriorPublication publication;
  Values committed;
  publication.publish_after_update(isam, estimates, 1, [&] {
    check(!publication.latest(), "publication precedes map commit");
    committed = estimates;
  });
  const auto first = publication.latest();
  check(first->epoch() == publication.epoch(), "epoch identity");
  check(first->estimates().equals(committed), "publication estimate identity");
  const auto joint = first->joint_covariance({1});
  check(
      joint.epoch == first->epoch() && joint.revision == 1,
      "result identity");
  close(joint.mean, first->gaussian_mean().at(1));
  const auto moved = first->joint_covariance_at_estimate({1});
  check(
      moved.coordinates == GaussianPosterior::Coordinates::estimate_tangent,
      "coordinate identity");
  close(moved.mean, Vector3::Zero());
  // Covariance failure must neither prevent a valid commit nor attach old covariance.
  NonlinearFactorGraph unsupported;
  unsupported.add(
      PriorFactor<Point2>(2, Point2(1, 2), noiseModel::Isotropic::Sigma(2, 1)));
  Values addition;
  addition.insert(2, Point2(1, 2));
  isam.update(unsupported, addition);
  const auto nextEstimates = isam.calculateEstimate();
  publication.publish_after_update(isam, nextEstimates, 2, [&] {
    committed = nextEstimates;
  });
  check(
      committed.equals(nextEstimates),
      "unsupported extraction still commits estimates");
  check(
      publication.latest()->status() == Status::unsupported,
      "unavailable publication");
  check(publication.latest()->revision() == 2, "failure revision");
  check(first->status() == Status::available, "retained publication lifetime");
  Values wrong;
  wrong.insert(1, Point3(9, 9, 9));
  check(
      GaussianPosterior::capture(isam, wrong, 3)->status() ==
          Status::unavailable,
      "mismatched estimate rejected");
  bool epochRejected = false;
  try {
    publication.publish(capture(isam, 3));
  } catch (const std::invalid_argument&) {
    epochRejected = true;
  }
  check(epochRejected, "cross-epoch attachment rejected");
  // A failed update never calls the completion path, leaving the old state intact.
  bool updateFailed = false;
  try {
    NonlinearFactorGraph invalid;
    invalid.add(
        PriorFactor<Point3>(
            9,
            Point3(0, 0, 0),
            noiseModel::Isotropic::Sigma(3, 1)));
    ISAM2 trial = isam;
    trial.update(invalid, Values());
  } catch (const std::exception&) {
    updateFailed = true;
  }
  check(updateFailed, "invalid update must fail");
  check(
      publication.latest()->revision() == 2,
      "failed update publication lifetime");
  GaussianPosterior::shared_ptr orphan;
  {
    ISAM2 temporary;
    temporary.update(graph, initial);
    orphan = capture(temporary, 1);
  }
  close(orphan->joint_covariance({1}).covariance, 4 * Matrix3::Identity());
  check(orphan->estimates().size() == 1, "snapshot outlives optimizer");
  // A new backend/provider starts a new epoch and has no stale map association.
  PosteriorPublication rebuilt;
  check(
      rebuilt.epoch() != publication.epoch() && !rebuilt.latest(),
      "rebuild epoch");
  rebuilt.publish_after_update(isam, nextEstimates, 1, [] {});
  check(rebuilt.latest()->revision() == 1, "rebuild revision");
  // Explicit query admission bound, exercised without allocating a large inverse.
  ISAM2 large;
  NonlinearFactorGraph independent;
  Values independentValues;
  KeyVector keys;
  for (Key key = 0; key < 90; ++key) {
    independent.add(
        PriorFactor<Point3>(
            key,
            Point3(0, 0, 0),
            noiseModel::Isotropic::Sigma(3, 1)));
    independentValues.insert(key, Point3(0, 0, 0));
    keys.push_back(key);
  }
  large.update(independent, independentValues);
  const auto bounded = capture(large, 1);
  for (const auto key : keys)
    close(bounded->joint_covariance({key}).covariance, Matrix3::Identity());
  close(bounded->joint_covariance({0}).covariance, Matrix3::Identity());
  check(
      capture(large, 1)->joint_covariance(keys).status == Status::unavailable,
      "workspace bound");
}

int main(int argc_, char** argv_) {
  try {
    const std::string selected = argc_ > 1 ? argv_[1] : "all";
    if (selected == "all" || selected == "linear") linear_tests();
    if (selected == "all" || selected == "pose") pose_tests();
    if (selected == "all" || selected == "edge") edge_tests();
    if (selected == "all" || selected == "publication")
      publication_path_tests();
    std::cout << checks << " checks passed\n";
  } catch (const std::exception& error_) {
    std::cerr << "FAIL: " << error_.what() << '\n';
    return 1;
  }
}
