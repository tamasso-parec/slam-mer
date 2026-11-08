/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @brief Production custom-factor and real backend/map publication checks.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Cholesky>
#include <atomic>
#include <chrono>
#include <covisibility_graph.hpp>
#include <interface_rerun.hpp>
#include <iostream>
#include <random>
#include <sensor_data.hpp>
#include <settings.hpp>
#include <stdexcept>
#include <thread>

using namespace slam_mer;
using namespace slam_mer::covisibility_graph;
using Clock = std::chrono::steady_clock;

void require(bool value_, const char* message_) {
  if (!value_) throw std::runtime_error(message_);
}
void close(
    const gtsam::Matrix& actual_,
    const gtsam::Matrix& expected_,
    double tolerance_ = 1e-7) {
  require(
      actual_.rows() == expected_.rows() && actual_.cols() == expected_.cols(),
      "Matrix dimensions disagree");
  require(
      (actual_ - expected_).norm() <=
          tolerance_ * std::max(1., expected_.norm()),
      "Numerical reference mismatch");
}

void factor_tests() {
  using namespace gtsam;
  std::mt19937 random(6429);
  std::normal_distribution<double> normal;
  const auto calibration = std::make_shared<Cal3_S2>(300, 310, 0, 320, 240);
  for (int sample = 0; sample < 24; ++sample) {
    Vector6 tangent;
    for (int j = 0; j < 6; ++j)
      tangent(j) = .2 * normal(random);
    const Pose3 pose = Pose3::Expmap(tangent);
    const Point3 point = pose.transformFrom(Point3(.3, -.2, 4.));
    const auto noise = noiseModel::Isotropic::Sigma(3, .1);
    PointDistanceFactor<Pose3> distance(
        keyframe_id_t(0),
        point_id_t(0),
        Point3(.1, .2, 4.1),
        noise);
    slam_mer::covisibility_graph::ProjectionFactor<Pose3> projection(
        keyframe_id_t(0),
        point_id_t(0),
        calibration,
        Point2(320, 240),
        noiseModel::Isotropic::Sigma(2, 1.));
    auto verify = [&](const auto& factor_) {
      Matrix poseJacobian, pointJacobian;
      const auto error =
          factor_.evaluateError(pose, point, &poseJacobian, &pointJacobian);
      Matrix numericalPose(error.size(), 6), numericalPoint(error.size(), 3);
      for (int j = 0; j < 6; ++j) {
        Vector6 step = Vector6::Zero();
        step(j) = 1e-6;
        numericalPose.col(j) =
            (factor_
                 .evaluateError(pose.retract(step), point, nullptr, nullptr) -
             factor_
                 .evaluateError(pose.retract(-step), point, nullptr, nullptr)) /
            2e-6;
      }
      for (int j = 0; j < 3; ++j) {
        Vector3 step = Vector3::Zero();
        step(j) = 1e-6;
        numericalPoint.col(j) =
            (factor_.evaluateError(pose, point + step, nullptr, nullptr) -
             factor_.evaluateError(pose, point - step, nullptr, nullptr)) /
            2e-6;
      }
      close(poseJacobian, numericalPose, 1e-7);
      close(pointJacobian, numericalPoint, 1e-7);
    };
    verify(distance);
    verify(projection);
    // Pose3 preserves the pinned Between/Prior approximation, including at
    // nonzero residual. The Sim(3) correction must not change this estimator.
    const auto other = pose.retract(Vector6::Constant(.1));
    const auto measured = pose.between(other).retract(Vector6::Constant(.03));
    const auto poseNoise = noiseModel::Isotropic::Sigma(6, .1);
    BetweenFactor<Pose3> nativeBetween(
        Symbol('k', 0),
        Symbol('k', 1),
        measured,
        poseNoise);
    OdometryFactor<Pose3> productionBetween(
        keyframe_id_t(0),
        keyframe_id_t(1),
        measured,
        poseNoise);
    Matrix nativeA, nativeB, productionA, productionB;
    close(
        nativeBetween.evaluateError(pose, other, &nativeA, &nativeB),
        productionBetween
            .evaluateError(pose, other, &productionA, &productionB),
        1e-13);
    close(nativeA, productionA, 1e-13);
    close(nativeB, productionB, 1e-13);
    close(
        productionBetween.evaluateError(pose, other),
        nativeBetween.evaluateError(pose, other),
        1e-13);
    PriorFactor<Pose3> nativePrior(Symbol('k', 0), other, poseNoise);
    PosePriorFactor<Pose3> productionPrior(Symbol('k', 0), other, poseNoise);
    close(
        nativePrior.evaluateError(pose, &nativeA),
        productionPrior.evaluateError(pose, &productionA),
        1e-13);
    close(nativeA, productionA, 1e-13);
  }
  // Real repository factors and robust weights, with a nonzero RHS.
  ISAM2Params params;
  params.relinearizeSkip = 1;
  params.relinearizeThreshold = .001;
  ISAM2 isam(params);
  NonlinearFactorGraph graph;
  Values initial;
  Ordering order;
  const auto robust = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.345),
      noiseModel::Isotropic::Sigma(3, .05));
  for (int i = 0; i < 2; ++i) {
    const Key key = Symbol('k', i);
    const Pose3 pose(Rot3::RzRyRx(.1, -.1, .05 * i), Point3(i * .5, .1, .2));
    initial.insert(key, pose.retract(Vector6::Constant(.02)));
    graph.add(
        PriorFactor<Pose3>(key, pose, noiseModel::Isotropic::Sigma(6, .2)));
    order.push_back(key);
  }
  for (int i = 0; i < 2; ++i) {
    const Key key = Symbol('p', i);
    const Point3 point(.2 * i, .3, 4.);
    initial.insert(key, Point3(point + Vector3(.1, -.05, .1)));
    graph.add(
        PriorFactor<Point3>(key, point, noiseModel::Isotropic::Sigma(3, .4)));
    order.push_back(key);
    for (int pose = 0; pose < 2; ++pose) {
      const auto anchor = initial.at<Pose3>(Symbol('k', pose));
      graph.push_back(
          PointDistanceFactor<Pose3>::make_shared(
              keyframe_id_t(pose),
              point_id_t(i),
              anchor.transformTo(point) + Vector3(.02, -.03, .01),
              robust));
      graph.push_back(
          slam_mer::covisibility_graph::ProjectionFactor<Pose3>::make_shared(
              keyframe_id_t(pose),
              point_id_t(i),
              calibration,
              PinholeCamera<Cal3_S2>(anchor, *calibration).project(point) +
                  Point2(2, -1),
              noiseModel::Isotropic::Sigma(2, 1.)));
    }
  }
  isam.update(graph, initial);
  const auto snapshot =
      GaussianPosterior::capture(isam, isam.calculateEstimate(), 1);
  require(
      snapshot->status() == GaussianPosterior::Status::available,
      "Custom-factor capture failed");
  const auto information = graph.linearize(initial)->hessian(order);
  const Matrix reference =
      information.first.ldlt().solve(Matrix::Identity(18, 18));
  close(snapshot->joint_covariance(order).covariance, reference, 1e-8);
  close(
      snapshot->gaussian_mean().vector(order),
      information.first.ldlt().solve(information.second),
      1e-8);
  for (const Key key : order)
    close(
        snapshot->joint_covariance({key}).covariance,
        isam.marginalCovariance(key),
        1e-8);
  isam.update();
  auto next = GaussianPosterior::capture(isam, isam.calculateEstimate(), 2);
  for (const Key key : order)
    close(
        next->joint_covariance({key}).covariance,
        isam.marginalCovariance(key),
        1e-8);
  // Report-only extraction and queries must not change identical fixed-graph
  // estimates through subsequent selective relinearizations.
  ISAM2 baseline = isam;
  for (uint64_t revision = 3; revision < 8; ++revision) {
    baseline.update();
    isam.update();
    const auto expected = baseline.calculateEstimate();
    const auto estimates = isam.calculateEstimate();
    const auto reporting =
        GaussianPosterior::capture(isam, estimates, revision);
    require(
        reporting->joint_covariance(order).status ==
            GaussianPosterior::Status::available,
        "Report-only custom-factor query failed");
    require(
        expected.equals(isam.calculateEstimate(), 1e-12),
        "Report-only extraction changed fixed-graph estimates");
  }
  std::cout << "48 custom-factor Jacobian checks and robust joint Gaussian "
               "references passed\n";
}

class TestSensor : public data_type::SensorData {
public:
  TestSensor() {
    set_intrinsics(intrinsics_t(300, 300, 320, 240));
    set_timeStamp(time_stamp_t(uint64_t(1000000)));
  }
};

template <typename TransformationType>
GaussianPosterior::shared_ptr wait_for(
    const std::shared_ptr<CovisibilityGraph<TransformationType>>& graph_,
    const gtsam::KeyVector& keys_,
    uint64_t after_ = 0) {
  const auto deadline = Clock::now() + std::chrono::seconds(10);
  std::string reason;
  while (Clock::now() < deadline) {
    auto snapshot = graph_->posterior();
    if (snapshot && snapshot->revision() > after_) {
      if (snapshot->joint_covariance(keys_).status ==
          GaussianPosterior::Status::available)
        return snapshot;
      reason = snapshot->message();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw std::runtime_error("Backend publication timeout: " + reason);
}

template <typename TransformationType>
void worker_tests(const std::string& settings_) {
  settings::sharedSettings->load(settings_);
  auto map = map::Map::make_shared();
  auto sensor = std::make_shared<TestSensor>();
  const gtsam::Key k0 = gtsam::Symbol('k', 0), k1 = gtsam::Symbol('k', 1);
  const auto pose0 = pose_t::Identity();
  gtsam::Pose3 pose1(gtsam::Rot3(), gtsam::Point3(.5, 0, 0));
  keyframe_t frame0(sensor, pose0, {}, {});
  keyframe_t frame1(sensor, pose_t(pose1.matrix()), {}, {});
  const auto id0 = map->insert(pose0, frame0).get();
  const auto id1 = map->insert(pose_t(pose1.matrix()), frame1).get();
  require(id0.key() == 0 && id1.key() == 1, "Fresh map key IDs");
  keyframe_id_t::ordered buffer;
  buffer.insert_(id0);
  buffer.insert_(id1);
  map->set_localization_buffer_keyframes(buffer);
  auto graph = CovisibilityGraph<TransformationType>::make_shared(map);
  require(!graph->posterior(), "Unexpected initial publication");
  graph->insert(OdometryEdge(id0, id1, pose_t(pose1.matrix())));
  auto first = wait_for(graph, {k0, k1});
  const auto original = first->joint_covariance({k0, k1}).covariance;
  auto repeated = wait_for(graph, {k0, k1}, first->revision());
  close(repeated->joint_covariance({k0, k1}).covariance, original, 1e-7);
  // Add a real map point with two observations: only then does it enter iSAM2.
  const gtsam::Point3 point(.2, .1, 4.);
  const auto pointId = map->insert(point3d_t(point)).get().first;
  const gtsam::Key p0 = gtsam::Symbol('p', pointId.key());
  const descriptor_t descriptor(std::vector<float>(128, .1f));
  auto observe = [&](keyframe_id_t id_, const gtsam::Pose3& pose_) {
    const auto camera = pose_.transformTo(point);
    PointObservation<TransformationType> observation(
        point2d_t(
            300 * camera.x() / camera.z() + 320,
            300 * camera.y() / camera.z() + 240),
        point3d_t(camera),
        descriptor,
        pose_t(pose_.matrix()));
    graph->insert(
        ObservationEdge<TransformationType>(pointId, id_, observation));
  };
  observe(id0, gtsam::Pose3());
  observe(id1, pose1);
  auto withPoint = wait_for(graph, {k0, k1, p0}, repeated->revision());
  require(
      first->joint_covariance({p0}).status ==
          GaussianPosterior::Status::unavailable,
      "Old publication acquired a new ID");
  std::atomic<bool> valid{true};
  std::jthread reader([&](std::stop_token token_) {
    while (!token_.stop_requested()) {
      auto current = graph->posterior();
      const auto result = current->joint_covariance_at_estimate({k0, p0});
      if (result.status != GaussianPosterior::Status::available ||
          result.epoch != current->epoch() ||
          result.revision != current->revision())
        valid = false;
      std::this_thread::yield();
    }
  });
  // Release the active restraint and then reinstate it while readers query.
  map->set_localization_buffer_keyframes(keyframe_id_t::ordered{});
  auto released = wait_for(graph, {k0, k1, p0}, withPoint->revision() + 1);
  map->set_localization_buffer_keyframes(buffer);
  auto restored = wait_for(graph, {k0, k1, p0}, released->revision() + 1);
  reader.request_stop();
  reader.join();
  require(valid, "Concurrent backend reader failed");
  require(
      released->joint_covariance({k1}).covariance.trace() >
          restored->joint_covariance({k1}).covariance.trace(),
      "Active window restraint missing");
  // The zero-residual map commit must agree with the snapshot, without reading
  // a mutable map during a changing estimate.
  const auto finalPose =
      restored->estimates().template at<TransformationType>(k1);
  // The map stores physical rotation/translation, whereas the snapshot keeps
  // the complete similarity. Do not use Similarity3::matrix() for conversion.
  close(
      map->get(id1).get().get_matrix().topLeftCorner<3, 3>(),
      finalPose.rotation().matrix(),
      1e-7);
  close(
      map->get(id1).get().get_matrix().topRightCorner<3, 1>(),
      finalPose.translation(),
      1e-7);
  close(
      map->get(pointId).get().to_eigen(),
      restored->estimates().template at<gtsam::Point3>(p0),
      1e-7);
  graph.reset();
  map.reset();
  close(first->joint_covariance({k0, k1}).covariance, original);
  require(
      restored->joint_covariance({k0, p0}).status ==
          GaussianPosterior::Status::available,
      "Snapshot did not survive actual backend teardown");
  std::cout << "Real backend worker/map commit, restraints, new IDs, "
               "concurrent queries and teardown passed\n";
}

int main(int argc_, char** argv_) {
  try {
    // Link the production logging sink without starting interface workers/viewer.
    (void)interface::InterfaceRerun::get_instance();
    if (argc_ > 1 && std::string(argv_[1]) == "factors")
      factor_tests();
    else if (argc_ == 3 && std::string(argv_[1]) == "worker")
      worker_tests<gtsam::Pose3>(argv_[2]);
    else if (argc_ == 3 && std::string(argv_[1]) == "worker-sim3")
      worker_tests<gtsam::Similarity3>(argv_[2]);
    else
      throw std::invalid_argument(
          "Usage: test_backend_posterior factors | worker|worker-sim3 "
          "SETTINGS");
  } catch (const std::exception& error_) {
    std::cerr << error_.what() << '\n';
    return 1;
  }
}
