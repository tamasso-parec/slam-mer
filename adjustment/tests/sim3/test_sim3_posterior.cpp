/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <algorithm>
#include <atomic>
#include <covisibility_factors.hpp>
#include <gaussian_posterior.hpp>
#include <interface_rerun.hpp>
#include <iostream>
#include <random>
#include <set>
#include <settings.hpp>
#include <stdexcept>
#include <thread>

using namespace gtsam;
using namespace slam_mer;
using namespace slam_mer::covisibility_graph;
using Sim3 = gtsam::Similarity3;
using Status = GaussianPosterior::Status;
namespace {
size_t checks = 0;
void require(bool condition_, const char* message_) {
  ++checks;
  if (!condition_) throw std::runtime_error(message_);
}
void close(
    const Matrix& actual_,
    const Matrix& expected_,
    double tolerance_ = 1e-9) {
  require(
      actual_.rows() == expected_.rows() && actual_.cols() == expected_.cols(),
      "Shape mismatch");
  require(
      (actual_ - expected_).norm() <=
          tolerance_ * std::max(1., expected_.norm()),
      "Numerical mismatch");
}
void relative_close(
    const Matrix& actual_,
    const Matrix& expected_,
    double tolerance_) {
  require(
      (actual_ - expected_).norm() <= tolerance_ * expected_.norm(),
      "Relative covariance mismatch");
}
GaussianPosterior::shared_ptr capture(ISAM2& solver_, uint64_t revision_) {
  const auto snapshot = GaussianPosterior::capture(
      solver_,
      solver_.calculateEstimate(),
      revision_);
  if (snapshot->status() != Status::available)
    throw std::runtime_error(snapshot->message());
  return snapshot;
}

struct LinearFixture {
  ISAM2 solver;
  Ordering order{
      Symbol('k', 0),
      Symbol('k', 1),
      Symbol('p', 0),
      Symbol('p', 1)};
  Matrix a;
  Vector b;
  LinearFixture() : a(Matrix::Identity(20, 20)), b(20) {
    std::mt19937 generator(91482026);
    std::normal_distribution<double> normal;
    for (int i = 0; i < 20; ++i)
      for (int j = i + 1; j < 20; ++j)
        a(i, j) = .12 * normal(generator);
    a *= 100;
    b = a * Vector::LinSpaced(20, -.1, .12);
    std::vector<std::pair<Key, Matrix>> terms;
    terms.emplace_back(order[0], a.leftCols(7));
    terms.emplace_back(order[1], a.middleCols(7, 7));
    terms.emplace_back(order[2], a.middleCols(14, 3));
    terms.emplace_back(order[3], a.rightCols(3));
    NonlinearFactorGraph graph;
    graph.push_back(
        std::make_shared<LinearContainerFactor>(
            std::make_shared<JacobianFactor>(terms, b)));
    Values anchors;
    anchors.insert(
        order[0],
        Sim3(Rot3::RzRyRx(.2, -.3, .1), Point3(1., -.2, .4), 2.5));
    anchors.insert(
        order[1],
        Sim3(Rot3::RzRyRx(-.1, .2, .3), Point3(-.3, .8, .2), .4));
    anchors.insert(order[2], Point3(.2, -.4, 4.));
    anchors.insert(order[3], Point3(.7, .3, 5.));
    solver.update(graph, anchors);
  }
};

Matrix numeric_transport(
    const GaussianPosterior& snapshot_,
    const KeyVector& order_) {
  size_t dimension = 0;
  for (const auto key : order_)
    dimension += snapshot_.anchors().at(key).dim();
  Matrix jacobian = Matrix::Identity(dimension, dimension);
  size_t offset = 0;
  for (const auto key : order_) {
    const size_t dim = snapshot_.anchors().at(key).dim();
    if (dim == 7) {
      const auto anchor = snapshot_.anchors().at<Sim3>(key);
      const auto estimate = snapshot_.estimates().at<Sim3>(key);
      const auto delta = snapshot_.estimate_delta().at(key);
      for (int j = 0; j < 7; ++j) {
        Vector plus = delta, minus = delta;
        plus(j) += 1e-6;
        minus(j) -= 1e-6;
        jacobian.block(offset, offset + j, 7, 1) =
            (estimate.localCoordinates(anchor.retract(plus)) -
             estimate.localCoordinates(anchor.retract(minus))) /
            2e-6;
      }
    }
    offset += dim;
  }
  return jacobian;
}

void linear_tests() {
  LinearFixture fixture;
  const auto snapshot = capture(fixture.solver, 1);
  const Matrix information = fixture.a.transpose() * fixture.a;
  const Matrix covariance = information.ldlt().solve(Matrix::Identity(20, 20));
  const Vector mean = fixture.a.colPivHouseholderQr().solve(fixture.b);
  const auto joint = snapshot->joint_covariance(fixture.order);
  relative_close(joint.covariance, covariance, 1e-11);
  close(joint.mean, mean, 1e-12);
  const int offsets[]{0, 7, 14, 17}, dims[]{7, 7, 3, 3};
  for (int i = 0; i < 4; ++i) {
    relative_close(
        snapshot->joint_covariance({fixture.order[i]}).covariance,
        fixture.solver.marginalCovariance(fixture.order[i]),
        1e-10);
    for (int j = 0; j < 4; ++j)
      if (i != j) {
        const auto pair =
            snapshot->joint_covariance({fixture.order[j], fixture.order[i]});
        require(
            pair.dimensions ==
                std::vector<size_t>{size_t(dims[j]), size_t(dims[i])},
            "Caller dimensions/order");
        relative_close(
            pair.covariance.topRightCorner(dims[j], dims[i]),
            covariance.block(offsets[j], offsets[i], dims[j], dims[i]),
            1e-10);
        close(pair.covariance, pair.covariance.transpose(), 1e-14);
      }
  }
  require(
      joint.covariance.block<3, 3>(14, 17).norm() > 1e-6,
      "Nontrivial landmark correlation");
  require(
      joint.covariance.block<7, 1>(0, 13).norm() > 1e-6,
      "Nontrivial scale cross correlation");
  const KeyVector reversed{
      fixture.order[3],
      fixture.order[1],
      fixture.order[2],
      fixture.order[0]};
  const auto raw = snapshot->joint_covariance(reversed);
  const auto moved = snapshot->joint_covariance_at_estimate(reversed);
  const Matrix transport = numeric_transport(*snapshot, reversed);
  relative_close(
      moved.covariance,
      transport * raw.covariance * transport.transpose(),
      1e-8);
  close(
      moved.mean,
      transport * (raw.mean - snapshot->estimate_delta().vector(reversed)),
      1e-10);
  require(
      moved.revision == raw.revision && moved.epoch == raw.epoch &&
          moved.coordinates == GaussianPosterior::Coordinates::estimate_tangent,
      "Compatible transport identity");
  require(
      snapshot->joint_covariance({Symbol('p', 99)}).status ==
          Status::unavailable,
      "Missing key");
  require(
      snapshot->joint_covariance({fixture.order[0], fixture.order[0]}).status ==
          Status::unavailable,
      "Duplicate key");
  close(snapshot->joint_covariance(reversed).covariance, raw.covariance, 0);
  // The selected pair's marginal differs from conditioning on the other variables.
  require(
      (covariance.topLeftCorner(7, 7) -
       information.topLeftCorner(7, 7).inverse())
              .norm() > 1e-6,
      "Marginalization must preserve eliminated-variable uncertainty");
}

void geometry_tests() {
  for (double scale : {.4, 1., 2.5})
    for (double sigma : {-.3, 0., .4})
      for (double angle : {0., .0004, .2, 3.12}) {
        const Sim3 anchor(
            Rot3::RzRyRx(.2, -.1, .3),
            Point3(2., -.4, .7),
            scale);
        Vector7 delta;
        delta << angle, -.1 * angle, .2 * angle, .3, -.2, .15, sigma;
        const auto estimate = anchor.retract(delta);
        Matrix77 numeric;
        for (int j = 0; j < 7; ++j) {
          Vector7 e = Vector7::Zero();
          e(j) = 1e-6;
          numeric.col(j) =
              (estimate.localCoordinates(anchor.retract(delta + e)) -
               estimate.localCoordinates(anchor.retract(delta - e))) /
              2e-6;
        }
        close(sim3_retract_jacobian(delta), numeric, 2e-7);
      }
  close(sim3_retract_jacobian(Vector7::Zero()), Matrix77::Identity(), 1e-14);
  LinearFixture fixture;
  const auto snapshot = capture(fixture.solver, 1);
  const auto joint = snapshot->joint_covariance_at_estimate(fixture.order);
  const auto first = snapshot->estimates().at<Sim3>(fixture.order[0]);
  const auto second = snapshot->estimates().at<Sim3>(fixture.order[1]);
  const auto point = snapshot->estimates().at<Point3>(fixture.order[2]);
  Matrix a, b;
  sim3_camera_point(first, point, &a, &b);
  Matrix camera = Matrix::Zero(3, 20);
  camera.leftCols(7) = a;
  camera.middleCols(14, 3) = b;
  Matrix numericCamera = Matrix::Zero(3, 20);
  Matrix77 numericInverse;
  Matrix relative = Matrix::Zero(7, 20), numericRelative = relative;
  const auto inverse = first.inverse(), between = first.between(second);
  relative.leftCols(7) = -sim3_adjoint(between.inverse());
  relative.middleCols(7, 7).setIdentity();
  for (int j = 0; j < 7; ++j) {
    Vector7 e = Vector7::Zero();
    e(j) = 1e-6;
    numericCamera.col(j) = (first.retract(e).inverse().transformFrom(point) -
                            first.retract(-e).inverse().transformFrom(point)) /
                           2e-6;
    numericInverse.col(j) =
        (inverse.localCoordinates(first.retract(e).inverse()) -
         inverse.localCoordinates(first.retract(-e).inverse())) /
        2e-6;
    numericRelative.col(j) =
        (between.localCoordinates(first.retract(e).between(second)) -
         between.localCoordinates(first.retract(-e).between(second))) /
        2e-6;
    numericRelative.col(7 + j) =
        (between.localCoordinates(first.between(second.retract(e))) -
         between.localCoordinates(first.between(second.retract(-e)))) /
        2e-6;
  }
  for (int j = 0; j < 3; ++j) {
    Vector3 e = Vector3::Zero();
    e(j) = 1e-6;
    numericCamera.col(14 + j) = (first.inverse().transformFrom(point + e) -
                                 first.inverse().transformFrom(point - e)) /
                                2e-6;
  }
  close(camera, numericCamera, 1e-8);
  close(-sim3_adjoint(first), numericInverse, 1e-8);
  close(relative, numericRelative, 1e-8);
  relative_close(
      camera * joint.covariance * camera.transpose(),
      numericCamera * joint.covariance * numericCamera.transpose(),
      1e-8);
  relative_close(
      relative * joint.covariance * relative.transpose(),
      numericRelative * joint.covariance * numericRelative.transpose(),
      1e-8);
  // World translation covariance is sR * P_vv * (sR)^T, not the local vv block.
  Matrix translation = Matrix::Zero(3, 20);
  translation.block<3, 3>(0, 3) = first.scale() * first.rotation().matrix();
  require(
      (translation * joint.covariance * translation.transpose() -
       joint.covariance.block<3, 3>(3, 3))
              .norm() > 1e-6,
      "Physical translation coordinates must include scale");
  // Joint Monte Carlo propagation keeps every pose/scale/landmark cross term.
  const auto raw = snapshot->joint_covariance(fixture.order);
  const Matrix lower = raw.covariance.llt().matrixL();
  Matrix empirical = Matrix::Zero(20, 20);
  Vector sum = Vector::Zero(20);
  std::mt19937 generator(73812026);
  std::normal_distribution<double> normal;
  constexpr int samples = 20000;
  for (int i = 0; i < samples; ++i) {
    Vector z(20);
    for (int j = 0; j < 20; ++j)
      z(j) = normal(generator);
    Vector sample = .02 * lower * z;
    for (int pose = 0; pose < 2; ++pose) {
      const auto key = fixture.order[pose];
      sample.segment<7>(7 * pose) =
          snapshot->estimates().at<Sim3>(key).localCoordinates(
              snapshot->anchors().at<Sim3>(key).retract(
                  snapshot->estimate_delta().at(key) +
                  sample.segment<7>(7 * pose)));
    }
    sum += sample;
    empirical.noalias() += sample * sample.transpose();
  }
  empirical =
      empirical / samples - (sum / samples) * (sum / samples).transpose();
  relative_close(empirical, .0004 * joint.covariance, .05);
}

struct MaintainedReference {
  Matrix covariance;
  Vector mean;
};
MaintainedReference maintained_reference(
    const ISAM2& solver_,
    const Ordering& order_) {
  GaussianFactorGraph factors;
  std::set<const void*> seen;
  for (const auto& entry : solver_.nodes())
    if (seen.insert(entry.second.get()).second)
      factors.push_back(entry.second->conditional());
  const auto hessian = factors.hessian(order_);
  return {
      hessian.first.ldlt().solve(
          Matrix::Identity(hessian.first.rows(), hessian.first.rows())),
      hessian.first.ldlt().solve(hessian.second)};
}

void wildfire_mean_tests() {
  ISAM2Params parameters;
  parameters.enableRelinearization = false;
  parameters.optimizationParams = ISAM2GaussNewtonParams(1.);
  ISAM2 solver(parameters);
  NonlinearFactorGraph graph;
  Values values;
  FastMap<Key, int> groups;
  std::vector<Sim3> poses;
  Ordering order;
  const auto noise = noiseModel::Isotropic::Sigma(7, .2);
  for (int i = 0; i < 6; ++i) {
    const Key key = Symbol('k', i);
    order.push_back(key);
    groups[key] = i;
    poses.emplace_back(
        Rot3::RzRyRx(.02 * i, -.01 * i, .03 * i),
        Point3(.2 * i, .03 * i, 0),
        .7 + .2 * i);
    values.insert(key, poses.back());
    if (i)
      graph.add(
          OdometryFactor<Sim3>(
              keyframe_id_t(i - 1),
              keyframe_id_t(i),
              poses[i - 1].between(poses[i]),
              noise));
    else
      graph.add(PosePriorFactor<Sim3>(key, poses.back(), noise));
  }
  solver.update(graph, values, {}, groups);
  solver.calculateEstimate();
  NonlinearFactorGraph addition;
  addition.add(
      PosePriorFactor<Sim3>(
          order.back(),
          poses.back().retract(Vector7::Constant(.01)),
          noise));
  solver.update(addition, Values());
  const auto snapshot = capture(solver, 2);
  const auto reference = maintained_reference(solver, order);
  const auto raw = snapshot->joint_covariance(order);
  close(raw.mean, reference.mean, 1e-9);
  const Vector difference = raw.mean - snapshot->estimate_delta().vector(order);
  require(
      difference.norm() > 1e-6,
      "Wildfire fixture did not truncate the estimate delta");
  const auto moved = snapshot->joint_covariance_at_estimate(order);
  const Matrix transport = numeric_transport(*snapshot, order);
  require(
      moved.mean.norm() > 1e-6,
      "Transport silently replaced Gaussian mean by optimizer estimate");
  close(moved.mean, transport * difference, 1e-9);
  relative_close(
      moved.covariance,
      transport * raw.covariance * transport.transpose(),
      1e-8);
}

void incremental_tests() {
  ISAM2Params parameters;
  parameters.relinearizeSkip = 1;
  parameters.relinearizeThreshold = .001;
  ISAM2 solver(parameters), baseline(parameters);
  const Key k0 = Symbol('k', 0), k1 = Symbol('k', 1), p0 = Symbol('p', 0),
            p1 = Symbol('p', 1);
  const std::vector<Sim3> poses{
      Sim3(Rot3::RzRyRx(.1, -.2, .05), Point3(.1, -.2, .3), 2.5),
      Sim3(Rot3::RzRyRx(-.1, .1, .2), Point3(.8, .1, .2), .4)};
  const std::vector<Point3> points{Point3(.3, .2, 4.), Point3(.8, -.1, 5.)};
  const auto robust = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.345),
      noiseModel::Isotropic::Sigma(3, .08));
  const auto noise = noiseModel::Isotropic::Sigma(7, .2);
  const auto calibration = std::make_shared<Cal3_S2>(300, 310, 0, 320, 240);
  NonlinearFactorGraph graph;
  Values initial;
  graph.add(PosePriorFactor<Sim3>(k0, poses[0], noise));
  graph.add(
      OdometryFactor<Sim3>(
          keyframe_id_t(0),
          keyframe_id_t(1),
          poses[0].between(poses[1]),
          noise));
  for (int i = 0; i < 2; ++i) {
    initial.insert(
        Symbol('k', i),
        poses[i].retract(Vector7::Constant(.015 * (i ? -1 : 1))));
    initial.insert(Symbol('p', i), Point3(points[i] + Vector3(.06, -.04, .03)));
    graph.add(
        PriorFactor<Point3>(
            Symbol('p', i),
            points[i],
            noiseModel::Isotropic::Sigma(3, 1)));
    for (int j = 0; j < 2; ++j) {
      const auto camera = poses[j].inverse().transformFrom(points[i]);
      graph.push_back(
          PointDistanceFactor<Sim3>::make_shared(
              keyframe_id_t(j),
              point_id_t(i),
              camera + Vector3(.02, -.03, .01),
              robust));
      graph.push_back(
          slam_mer::covisibility_graph::ProjectionFactor<Sim3>::make_shared(
              keyframe_id_t(j),
              point_id_t(i),
              calibration,
              calibration->uncalibrate(
                  Point2(camera.x() / camera.z(), camera.y() / camera.z())) +
                  Point2(1, -2),
              noiseModel::Isotropic::Sigma(2, 1)));
    }
  }
  solver.update(graph, initial);
  baseline.update(graph, initial);
  auto first = capture(solver, 1);
  const Ordering order{k0, k1, p0, p1};
  const auto fresh = graph.linearize(first->anchors())->hessian(order);
  relative_close(
      first->joint_covariance(order).covariance,
      fresh.first.ldlt().solve(Matrix::Identity(20, 20)),
      1e-8);
  close(
      first->gaussian_mean().vector(order),
      fresh.first.ldlt().solve(fresh.second),
      1e-8);
  const auto atEstimate = graph.linearize(first->estimates())->hessian(order);
  require(
      (atEstimate.first.ldlt().solve(Matrix::Identity(20, 20)) -
       first->joint_covariance_at_estimate(order).covariance)
              .norm() > 1e-6,
      "Fresh nonlinear linearization must be distinguished");
  const Matrix old = first->joint_covariance({k0, p0}).covariance;
  PosteriorPublication publication;
  publication.publish_after_update(solver, solver.calculateEstimate(), 1, [] {
  });
  std::atomic<bool> valid{true};
  std::jthread reader([&](std::stop_token token_) {
    while (!token_.stop_requested()) {
      auto snapshot = publication.latest();
      const auto query = snapshot->joint_covariance_at_estimate({k0, k1, p0});
      if (query.status != Status::available ||
          query.epoch != snapshot->epoch() ||
          query.revision != snapshot->revision())
        valid = false;
      std::this_thread::yield();
    }
  });
  for (uint64_t revision = 2; revision <= 8; ++revision) {
    NonlinearFactorGraph additions;
    Values values;
    FactorIndices removals;
    if (revision == 3) {
      additions.add(
          PosePriorFactor<Sim3>(
              k1,
              poses[1],
              noiseModel::Isotropic::Sigma(7, .05)));
    } else if (revision == 4) {
      // Replace a supported factor in the real nonlinear graph.
      removals.push_back(1);
      additions.add(
          OdometryFactor<Sim3>(
              keyframe_id_t(0),
              keyframe_id_t(1),
              poses[0].between(poses[1]),
              noise));
    } else if (revision == 5) {
      // Replace a point ID by removing all its factors, then adding a new key.
      const auto& factors = solver.getFactorsUnsafe();
      for (size_t i = 0; i < factors.size(); ++i)
        if (factors[i] && std::find(
                              factors[i]->keys().begin(),
                              factors[i]->keys().end(),
                              p1) != factors[i]->keys().end())
          removals.push_back(i);
      const Key newPoint = Symbol('p', 99);
      values.insert(newPoint, points[1]);
      additions.add(
          PriorFactor<Point3>(
              newPoint,
              points[1],
              noiseModel::Isotropic::Sigma(3, .5)));
      additions.push_back(
          PointDistanceFactor<Sim3>::make_shared(
              keyframe_id_t(0),
              point_id_t(99),
              poses[0].inverse().transformFrom(points[1]),
              robust));
    }
    baseline.update(additions, values, removals);
    solver.update(additions, values, removals);
    const auto estimates = solver.calculateEstimate();
    require(
        estimates.equals(baseline.calculateEstimate(), 1e-12),
        "Reporting changed fixed-graph estimates");
    publication.publish_after_update(solver, estimates, revision, [&] {
      require(
          publication.latest()->revision() == revision - 1,
          "Publication must follow commit");
    });
    const auto snapshot = publication.latest();
    const Ordering currentOrder(snapshot->anchors().keys());
    const auto reference = maintained_reference(solver, currentOrder);
    const auto result = snapshot->joint_covariance(currentOrder);
    relative_close(result.covariance, reference.covariance, 1e-8);
    close(result.mean, reference.mean, 1e-8);
    for (const auto key : currentOrder)
      relative_close(
          snapshot->joint_covariance({key}).covariance,
          solver.marginalCovariance(key),
          1e-8);
    require(
        estimates.equals(solver.calculateEstimate(), 1e-12),
        "Query changed estimates");
    if (revision >= 5) {
      require(
          snapshot->joint_covariance({p1}).status == Status::unavailable,
          "Removed point ID still present");
      require(
          first->joint_covariance({Symbol('p', 99)}).status ==
              Status::unavailable,
          "Old snapshot acquired new point ID");
    }
  }
  reader.request_stop();
  reader.join();
  require(valid, "Concurrent Sim3 query failed");
  close(first->joint_covariance({k0, p0}).covariance, old, 0);
  require(
      (publication.latest()->joint_covariance({k0, p0}).covariance - old)
              .norm() > 1e-6,
      "Revision cache invalidation");
  const auto retained = publication.latest();
  bool failed = false;
  try {
    NonlinearFactorGraph invalid;
    invalid.add(PosePriorFactor<Sim3>(Symbol('k', 999), Sim3(), noise));
    ISAM2 guarded = solver;
    guarded.update(invalid, Values());
  } catch (const std::exception&) {
    failed = true;
  }
  require(
      failed && publication.latest() == retained,
      "Failed update must retain previous snapshot");
}

void edge_tests() {
  const Key k0 = Symbol('k', 0), k1 = Symbol('k', 1);
  const std::vector<Sim3> poses{
      Sim3(Rot3(), Point3(.1, 0, 0), 2.5),
      Sim3(Rot3(), Point3(.5, .1, 0), .4)};
  const auto noise = noiseModel::Isotropic::Sigma(7, .2);
  // Gauge failure is explicit; weak anchored directions retain their magnitude.
  ISAM2 weak;
  NonlinearFactorGraph weakGraph;
  Values weakValues;
  weakGraph.add(
      PosePriorFactor<Sim3>(
          k0,
          poses[0],
          noiseModel::Isotropic::Sigma(7, 1e6)));
  weakValues.insert(k0, poses[0]);
  weak.update(weakGraph, weakValues);
  relative_close(
      capture(weak, 1)->joint_covariance({k0}).covariance,
      1e12 * Matrix77::Identity(),
      1e-10);
  bool singular = false;
  try {
    ISAM2 free;
    NonlinearFactorGraph freeGraph;
    Values freeValues;
    freeGraph.add(
        OdometryFactor<Sim3>(
            keyframe_id_t(0),
            keyframe_id_t(1),
            poses[0].between(poses[1]),
            noise));
    freeValues.insert(k0, poses[0]);
    freeValues.insert(k1, poses[1]);
    free.update(freeGraph, freeValues);
    singular = GaussianPosterior::capture(free, free.calculateEstimate(), 1)
                   ->status() != Status::available;
  } catch (const IndeterminantLinearSystemException&) {
    singular = true;
  }
  require(singular, "Sim3 gauge freedom was regularized");
  // A seven-dimensional Euclidean value is not a Similarity3 merely because
  // its dimension agrees. Unsupported coordinate types must stay explicit.
  ISAM2 wrongType;
  NonlinearFactorGraph unsupported;
  Values unsupportedValues;
  const Vector7 zero = Vector7::Zero();
  unsupported.add(PriorFactor<Vector7>(k0, zero, noise));
  unsupportedValues.insert(k0, zero);
  wrongType.update(unsupported, unsupportedValues);
  require(
      GaussianPosterior::capture(wrongType, wrongType.calculateEstimate(), 1)
              ->status() == Status::unsupported,
      "Dimension was incorrectly used to infer a Similarity3 value");
  for (double scale : {0., -.4}) {
    ISAM2 invalid;
    NonlinearFactorGraph linear;
    Values values;
    const auto factor =
        std::make_shared<JacobianFactor>(k0, Matrix77::Identity(), zero);
    linear.push_back(std::make_shared<LinearContainerFactor>(factor));
    values.insert(k0, Sim3(Rot3(), Point3(1, 2, 3), scale));
    invalid.update(linear, values);
    require(
        GaussianPosterior::capture(invalid, invalid.calculateEstimate(), 1)
                ->status() == Status::unavailable,
        "Invalid scale produced available covariance");
  }
}
}  // namespace

int main(int argc_, char** argv_) {
  try {
    if (!settings::sharedSettings)
      throw std::runtime_error("Missing production settings owner");
    (void)interface::InterfaceRerun::get_instance();
    const std::string selected = argc_ > 1 ? argv_[1] : "all";
    if (selected == "all" || selected == "linear") linear_tests();
    if (selected == "all" || selected == "geometry") geometry_tests();
    if (selected == "all" || selected == "mean") wildfire_mean_tests();
    if (selected == "all" || selected == "incremental") incremental_tests();
    if (selected == "all" || selected == "edge") edge_tests();
    std::cout << checks << " Sim3 posterior checks passed\n";
  } catch (const std::exception& error_) {
    std::cerr << error_.what() << '\n';
    return 1;
  }
}
