/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <chrono>
#include <fstream>
#include <interface_rerun.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <settings.hpp>
#include "derivative_candidates.hpp"
#include "legacy_factors.hpp"

using namespace gtsam;
using namespace slam_mer;
using namespace slam_mer::covisibility_graph;
using namespace slam_mer::validation;
using Distance = legacy::Distance;
using Projection = legacy::Projection;
using Between = legacy::Between;

void require(bool condition_, const char* message_) {
  if (!condition_) throw std::runtime_error(message_);
}
void check_close(
    const Matrix& actual_,
    const Matrix& expected_,
    double tolerance_ = 1e-6) {
  require(
      actual_.rows() == expected_.rows() && actual_.cols() == expected_.cols(),
      "Shape mismatch");
  require(
      (actual_ - expected_).norm() <=
          tolerance_ * std::max(1., expected_.norm()),
      "Derivative mismatch");
}

void trace(const std::string& output_) {
  const auto calibration = std::make_shared<Cal3_S2>(300, 310, 0, 320, 240);
  const auto noise = noiseModel::Isotropic::Sigma(3, .05);
  std::ofstream log(output_);
  log << "scale,translation,distance_rotation_error,distance_translation_error,"
         "distance_scale_error,projection_rotation_error,projection_scale_"
         "error,between_zero_error,between_nonzero_error,repaired_distance_"
         "error,repaired_projection_error\n";
  for (double scale : {.4, 1., 2.5})
    for (double translation : {0., 1.}) {
      const Sim3 pose(
          Rot3::RzRyRx(.2, -.1, .3),
          Point3(translation, -.3 * translation, .2 * translation),
          scale);
      const Point3 point = pose.transformFrom(Point3(.3, -.1, 4.));
      Distance distance(
          keyframe_id_t(0),
          point_id_t(0),
          Point3(.2, .2, 4.1),
          noise);
      DistanceCandidate repairedDistance(
          keyframe_id_t(0),
          point_id_t(0),
          Point3(.2, .2, 4.1),
          noise);
      NumericalVisual<Distance> numericDistance(
          keyframe_id_t(0),
          point_id_t(0),
          Point3(.2, .2, 4.1),
          noise);
      Projection projection(
          keyframe_id_t(0),
          point_id_t(0),
          calibration,
          Point2(320, 240),
          noiseModel::Isotropic::Sigma(2, 1));
      ProjectionCandidate repairedProjection(
          keyframe_id_t(0),
          point_id_t(0),
          calibration,
          Point2(320, 240),
          noiseModel::Isotropic::Sigma(2, 1));
      NumericalVisual<Projection> numericProjection(
          keyframe_id_t(0),
          point_id_t(0),
          calibration,
          Point2(320, 240),
          noiseModel::Isotropic::Sigma(2, 1));
      Matrix a, b, n, m, r, s, pa, pb, pn, pm, pr, ps;
      const auto value = distance.evaluateError(pose, point, &a, &b);
      numericDistance.evaluateError(pose, point, &n, &m);
      check_close(
          repairedDistance.evaluateError(pose, point, &r, &s),
          value,
          1e-12);
      check_close(r, n);
      check_close(s, m);
      PointDistanceFactor<Sim3> productionDistance(
          keyframe_id_t(0),
          point_id_t(0),
          Point3(.2, .2, 4.1),
          noise);
      Matrix productionA, productionB;
      check_close(
          productionDistance
              .evaluateError(pose, point, &productionA, &productionB),
          value,
          1e-12);
      check_close(productionA, n);
      check_close(productionB, m);
      productionDistance.evaluateError(pose, point, nullptr, &productionB);
      check_close(productionB, m);
      productionDistance.evaluateError(pose, point, &productionA, nullptr);
      check_close(productionA, n);
      check_close(
          projection.evaluateError(pose, point, &pa, &pb),
          repairedProjection.evaluateError(pose, point, &pr, &ps),
          1e-12);
      numericProjection.evaluateError(pose, point, &pn, &pm);
      check_close(pr, pn);
      check_close(ps, pm);
      covisibility_graph::ProjectionFactor<Sim3> productionProjection(
          keyframe_id_t(0),
          point_id_t(0),
          calibration,
          Point2(320, 240),
          noiseModel::Isotropic::Sigma(2, 1));
      productionProjection
          .evaluateError(pose, point, &productionA, &productionB);
      check_close(productionA, pn);
      check_close(productionB, pm);
      const auto camera = pose.inverse().transformFrom(point);
      // Independently predicted exact error of the current distance Jacobian.
      check_close(a.col(6), -scale * camera, 1e-9);
      check_close(n.col(6), -camera, 1e-7);
      const Matrix expectedRotation =
          skewSymmetric(camera) +
          (1 / scale - 1) *
              skewSymmetric(pose.rotation().unrotate(pose.translation()));
      check_close(a.leftCols(3), expectedRotation, 1e-9);
      require(pa.col(6).norm() < 1e-9, "Projection scale must cancel");
      // A common world-origin translation leaves this residual and the right
      // pose tangent unchanged. The current Jacobian violates that invariance.
      const Vector3 shift(20, -5, 2);
      const Sim3 shifted(pose.rotation(), pose.translation() + shift, scale);
      Matrix shiftedLegacy, shiftedPoint, shiftedRepair, shiftedRepairPoint;
      check_close(
          distance.evaluateError(
              shifted,
              point + shift,
              &shiftedLegacy,
              &shiftedPoint),
          value,
          1e-10);
      productionDistance.evaluateError(
          shifted,
          point + shift,
          &shiftedRepair,
          &shiftedRepairPoint);
      check_close(shiftedRepair, r, 1e-10);
      check_close(
          shiftedLegacy.leftCols(3) - a.leftCols(3),
          (1 / scale - 1) * skewSymmetric(pose.rotation().unrotate(shift)),
          1e-9);
      const Sim3 second(Rot3::RzRyRx(-.1, .2, .1), Point3(.8, -.3, .2), 1.6);
      const auto measured = pose.between(second);
      Between zero(
          keyframe_id_t(0),
          keyframe_id_t(1),
          measured,
          noiseModel::Isotropic::Sigma(7, .1));
      BetweenCandidate zeroOracle(
          keyframe_id_t(0),
          keyframe_id_t(1),
          measured,
          noiseModel::Isotropic::Sigma(7, .1));
      Matrix za, zb, zn, zm, ea, eb, en, em;
      zero.evaluateError(pose, second, &za, &zb);
      zeroOracle.evaluateError(pose, second, &zn, &zm);
      const auto perturbed = second.retract(Vector7::Constant(.07));
      zero.evaluateError(pose, perturbed, &ea, &eb);
      auto legacyClone = std::dynamic_pointer_cast<Between>(zero.clone());
      require(
          bool(legacyClone),
          "Historical fixture clone changed the derivative version");
      legacyClone->evaluateError(pose, perturbed, &productionA, &productionB);
      check_close(productionA, ea, 1e-12);
      check_close(productionB, eb, 1e-12);
      zeroOracle.evaluateError(pose, perturbed, &en, &em);
      OdometryFactor<Sim3> productionBetween(
          keyframe_id_t(0),
          keyframe_id_t(1),
          measured,
          noiseModel::Isotropic::Sigma(7, .1));
      productionBetween
          .evaluateError(pose, perturbed, &productionA, &productionB);
      check_close(productionA, en, 1e-7);
      check_close(productionB, em, 1e-7);
      check_close(
          productionBetween.evaluateError(pose, perturbed),
          zeroOracle.evaluateError(pose, perturbed, nullptr, nullptr),
          1e-12);
      productionBetween.evaluateError(pose, perturbed, nullptr, &productionB);
      check_close(productionB, em, 1e-7);
      auto cloned = std::dynamic_pointer_cast<OdometryFactor<Sim3>>(
          productionBetween.clone());
      require(bool(cloned), "Cloning sliced corrected odometry derivatives");
      cloned->evaluateError(pose, perturbed, &productionA, nullptr);
      check_close(productionA, en, 1e-7);
      PosePriorFactor<Sim3> productionPrior(
          Symbol('k', 0),
          measured,
          noiseModel::Isotropic::Sigma(7, .1));
      PriorCandidate priorOracle(
          Symbol('k', 0),
          measured,
          noiseModel::Isotropic::Sigma(7, .1));
      Matrix priorNumerical;
      check_close(
          productionPrior.evaluateError(pose, &productionA),
          priorOracle.evaluateError(pose, &priorNumerical),
          1e-12);
      check_close(productionA, priorNumerical, 1e-7);
      check_close(
          productionPrior.evaluateError(pose),
          priorOracle.evaluateError(pose, nullptr),
          1e-12);
      require(
          bool(std::dynamic_pointer_cast<PosePriorFactor<Sim3>>(
              productionPrior.clone())),
          "Cloning sliced corrected prior derivatives");
      log << scale << ',' << translation << ','
          << (a.leftCols(3) - n.leftCols(3)).norm() << ','
          << (a.middleCols(3, 3) - n.middleCols(3, 3)).norm() << ','
          << (a.col(6) - n.col(6)).norm() << ','
          << (pa.leftCols(3) - pn.leftCols(3)).norm() << ','
          << (pa.col(6) - pn.col(6)).norm() << ','
          << (za - zn).norm() + (zb - zm).norm() << ','
          << (ea - en).norm() + (eb - em).norm() << ',' << (r - n).norm() << ','
          << (pr - pn).norm() << '\n';
    }
  std::cout
      << "Factor trace and independent inverse-action identities passed\n";
}

Vector true_gradient(
    const NonlinearFactorGraph& graph_,
    const Values& values_) {
  Vector gradient(values_.dim());
  size_t offset = 0;
  for (const auto key : values_.keys()) {
    const size_t dimension = values_.at(key).dim();
    for (size_t j = 0; j < dimension; ++j) {
      VectorValues increment;
      for (const auto other : values_.keys())
        increment.insert(other, Vector::Zero(values_.at(other).dim()));
      increment.at(key)(j) = 1e-6;
      const double plus = graph_.error(values_.retract(increment));
      increment.at(key)(j) = -1e-6;
      const double minus = graph_.error(values_.retract(increment));
      gradient(offset + j) = (plus - minus) / 2e-6;
    }
    offset += dimension;
  }
  return gradient;
}

enum class Mode { legacy, visual, repaired, oracle, production };
NonlinearFactorGraph problem(
    Mode mode_,
    const std::vector<Sim3>& truth_,
    const std::vector<Point3>& points_,
    bool robust_) {
  NonlinearFactorGraph graph;
  auto distanceNoise = noiseModel::Isotropic::Sigma(3, .05);
  SharedNoiseModel robust =
      robust_ ? SharedNoiseModel(
                    noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345),
                        distanceNoise))
              : SharedNoiseModel(distanceNoise);
  const auto calibration = std::make_shared<Cal3_S2>(300, 310, 0, 320, 240);
  if (mode_ == Mode::production)
    graph.add(
        PosePriorFactor<Sim3>(
            Symbol('k', 0),
            truth_[0],
            noiseModel::Isotropic::Sigma(7, 1e-4)));
  else if (mode_ == Mode::repaired || mode_ == Mode::oracle)
    graph.add(PriorCandidate(
        Symbol('k', 0),
        truth_[0],
        noiseModel::Isotropic::Sigma(7, 1e-4)));
  else
    graph.add(
        PriorFactor<Sim3>(
            Symbol('k', 0),
            truth_[0],
            noiseModel::Isotropic::Sigma(7, 1e-4)));
  for (size_t i = 1; i < truth_.size(); ++i) {
    const auto measured = truth_[i - 1].between(truth_[i]);
    if (mode_ == Mode::production)
      graph.add(
          OdometryFactor<Sim3>(
              keyframe_id_t(i - 1),
              keyframe_id_t(i),
              measured,
              noiseModel::Isotropic::Sigma(7, .08)));
    else if (mode_ == Mode::repaired || mode_ == Mode::oracle)
      graph.add(BetweenCandidate(
          keyframe_id_t(i - 1),
          keyframe_id_t(i),
          measured,
          noiseModel::Isotropic::Sigma(7, .08)));
    else
      graph.add(Between(
          keyframe_id_t(i - 1),
          keyframe_id_t(i),
          measured,
          noiseModel::Isotropic::Sigma(7, .08)));
  }
  for (size_t j = 0; j < points_.size(); ++j) {
    graph.add(
        PriorFactor<Point3>(
            Symbol('p', j),
            points_[j],
            noiseModel::Isotropic::Sigma(3, 1.)));
    for (size_t i = 0; i < truth_.size(); ++i) {
      const auto camera = truth_[i].inverse().transformFrom(points_[j]);
      Point3 measured = camera;
      if (robust_)
        measured += Point3(
            .005 * std::sin(i + j),
            -.008 * std::cos(i + 2 * j),
            .006 * std::sin(2 * i + j));
      if (mode_ == Mode::production)
        graph.add(
            PointDistanceFactor<Sim3>(
                keyframe_id_t(i),
                point_id_t(j),
                measured,
                robust));
      else if (mode_ == Mode::oracle)
        graph.add(
            NumericalVisual<Distance>(
                keyframe_id_t(i),
                point_id_t(j),
                measured,
                robust));
      else if (mode_ != Mode::legacy)
        graph.add(DistanceCandidate(
            keyframe_id_t(i),
            point_id_t(j),
            measured,
            robust));
      else
        graph.add(Distance(keyframe_id_t(i), point_id_t(j), measured, robust));
      const auto pixel = calibration->uncalibrate(
          Point2(camera.x() / camera.z(), camera.y() / camera.z()));
      const auto projectionNoise = noiseModel::Isotropic::Sigma(2, 1.);
      if (mode_ == Mode::production)
        graph.add(
            covisibility_graph::ProjectionFactor<Sim3>(
                keyframe_id_t(i),
                point_id_t(j),
                calibration,
                pixel,
                projectionNoise));
      else if (mode_ == Mode::oracle)
        graph.add(
            NumericalVisual<Projection>(
                keyframe_id_t(i),
                point_id_t(j),
                calibration,
                pixel,
                projectionNoise));
      else if (mode_ != Mode::legacy)
        graph.add(ProjectionCandidate(
            keyframe_id_t(i),
            point_id_t(j),
            calibration,
            pixel,
            projectionNoise));
      else
        graph.add(Projection(
            keyframe_id_t(i),
            point_id_t(j),
            calibration,
            pixel,
            projectionNoise));
    }
  }
  return graph;
}

void optimization_trials(const std::string& output_) {
  std::ofstream log(output_);
  log << std::setprecision(12);
  log << "trial,robust,mode,initial_cost,gradient_relative_error,final_cost,"
         "final_gradient_norm,iterations,milliseconds,pose_error,isam_failed,"
         "isam_final_cost,isam_final_gradient_norm,isam_milliseconds\n";
  std::mt19937 random(8462026);
  std::normal_distribution<double> normal;
  for (int trial = 0; trial < 12; ++trial) {
    const bool robust = trial >= 6;
    const std::vector<Sim3> truth{
        Sim3(Rot3::RzRyRx(.1, -.1, .1), Point3(.3, -.2, .1), .7),
        Sim3(Rot3::RzRyRx(.15, -.2, .2), Point3(.7, .1, .2), 1.4),
        Sim3(Rot3::RzRyRx(-.1, .15, -.1), Point3(1., .2, .3), 2.1)};
    std::vector<Point3> points;
    for (int j = 0; j < 8; ++j)
      points.emplace_back(.25 * (j % 4) - .3, .2 * (j / 4) - .1, 5. + .2 * j);
    Values initial;
    for (size_t i = 0; i < truth.size(); ++i) {
      Vector7 offset;
      for (int j = 0; j < 7; ++j)
        offset(j) = (j < 3 ? .12 : .25) * normal(random);
      initial.insert(Symbol('k', i), i ? truth[i].retract(offset) : truth[i]);
    }
    for (size_t j = 0; j < points.size(); ++j)
      initial.insert(
          Symbol('p', j),
          Point3(
              points[j] + Point3(
                              .2 * normal(random),
                              .2 * normal(random),
                              .2 * normal(random))));
    const auto reference = problem(Mode::legacy, truth, points, robust);
    const auto gradient = true_gradient(reference, initial);
    double oracleCost = -1, repairedCost = -1, productionCost = -1;
    for (const auto mode :
         {Mode::legacy,
          Mode::visual,
          Mode::repaired,
          Mode::oracle,
          Mode::production}) {
      const auto graph = problem(mode, truth, points, robust);
      require(
          std::abs(graph.error(initial) - reference.error(initial)) < 1e-10,
          "Candidate changed residual objective");
      const auto linear = graph.linearize(initial)->hessian();
      const double gradientError =
          (-linear.second - gradient).norm() / std::max(1., gradient.norm());
      if (mode == Mode::repaired || mode == Mode::oracle ||
          mode == Mode::production)
        require(gradientError < 2e-6, "Repaired graph gradient is incorrect");
      LevenbergMarquardtParams parameters;
      parameters.maxIterations = 100;
      parameters.relativeErrorTol = 1e-10;
      parameters.absoluteErrorTol = 1e-10;
      const auto start = std::chrono::steady_clock::now();
      LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
      const auto result = optimizer.optimize();
      const double ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - start)
                            .count();
      const double finalCost = reference.error(result);
      if (mode == Mode::repaired) repairedCost = finalCost;
      if (mode == Mode::oracle) oracleCost = finalCost;
      if (mode == Mode::production) productionCost = finalCost;
      double error = 0;
      for (size_t i = 0; i < truth.size(); ++i)
        error += truth[i]
                     .localCoordinates(result.at<Sim3>(Symbol('k', i)))
                     .squaredNorm();
      bool incrementalFailed = false;
      double incrementalCost = std::numeric_limits<double>::quiet_NaN();
      double incrementalGradient = std::numeric_limits<double>::quiet_NaN();
      Values incrementalValues;
      const auto incrementalStart = std::chrono::steady_clock::now();
      try {
        ISAM2Params params;
        params.relinearizeSkip = 1;
        params.relinearizeThreshold = .01;
        ISAM2 incremental(params);
        incremental.update(graph, initial);
        incrementalValues = incremental.calculateEstimate();
        for (int step = 0; step < 40; ++step) {
          incremental.update();
          incrementalValues = incremental.calculateEstimate();
        }
        incrementalCost = reference.error(incrementalValues);
        incrementalFailed = !std::isfinite(incrementalCost);
      } catch (const std::exception& error_) {
        incrementalFailed = true;
        std::cerr << "iSAM2 failed trial " << trial << " mode " << int(mode)
                  << ": " << error_.what() << '\n';
      }
      const double incrementalMilliseconds =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - incrementalStart)
              .count();
      if (mode == Mode::production)
        require(!incrementalFailed, "Production iSAM2 regression failed");
      if (!incrementalFailed)
        incrementalGradient =
            true_gradient(reference, incrementalValues).norm();
      log << trial << ',' << robust << ',' << int(mode) << ','
          << reference.error(initial) << ',' << gradientError << ','
          << finalCost << ',' << true_gradient(reference, result).norm() << ','
          << optimizer.iterations() << ',' << ms << ',' << std::sqrt(error)
          << ',' << incrementalFailed << ',' << incrementalCost << ','
          << incrementalGradient << ',' << incrementalMilliseconds << '\n';
      log.flush();
    }
    require(
        std::abs(oracleCost - productionCost) < 1e-7,
        "Production and numerical oracle disagree on optimum");
    require(
        std::abs(oracleCost - repairedCost) < 1e-7,
        "Analytic repair and numerical oracle disagree on optimum");
  }
  std::cout << "60 fixed-objective batch comparisons passed; iSAM2 outcomes "
               "recorded separately\n";
}

void window_costs(const std::string& output_) {
  std::ofstream log(output_);
  log << "poses,clamps,update,mode,milliseconds\n";
  for (int count : {3, 6, 10}) {
    std::vector<Sim3> poses;
    Values initial;
    for (int i = 0; i < count; ++i) {
      poses.emplace_back(
          Rot3::RzRyRx(.01 * i, -.005 * i, .02 * i),
          Point3(.1 * i, .03 * i, -.02 * i),
          std::exp(.02 * i));
      initial.insert(Symbol('k', i), poses.back());
    }
    ISAM2Params parameters;
    parameters.relinearizeSkip = 1;
    parameters.relinearizeThreshold = .01;
    ISAM2 solvers[3]{ISAM2(parameters), ISAM2(parameters), ISAM2(parameters)};
    FactorIndices previous[3];
    Matrix referenceCovariance[3];
    const auto noise = noiseModel::Isotropic::Sigma(7, .01);
    for (int mode = 0; mode < 3; ++mode) {
      NonlinearFactorGraph graph;
      if (mode == 2)
        graph.add(PriorCandidate(
            Symbol('k', 0),
            poses[0],
            noiseModel::Isotropic::Sigma(7, 1e-5)));
      else if (mode == 1)
        graph.add(
            PosePriorFactor<Sim3>(
                Symbol('k', 0),
                poses[0],
                noiseModel::Isotropic::Sigma(7, 1e-5)));
      else
        graph.add(
            PriorFactor<Sim3>(
                Symbol('k', 0),
                poses[0],
                noiseModel::Isotropic::Sigma(7, 1e-5)));
      for (int j = 1; j < count; ++j) {
        const auto measured = poses[j - 1].between(poses[j]);
        if (mode == 2)
          graph.add(BetweenCandidate(
              keyframe_id_t(j - 1),
              keyframe_id_t(j),
              measured,
              noise));
        else if (mode == 1)
          graph.add(
              OdometryFactor<Sim3>(
                  keyframe_id_t(j - 1),
                  keyframe_id_t(j),
                  measured,
                  noise));
        else
          graph.add(
              Between(keyframe_id_t(j - 1), keyframe_id_t(j), measured, noise));
      }
      solvers[mode].update(graph, initial);
    }
    // Regenerate directed window pairs and remove the preceding window on
    // every update. Alternate mode order and omit five warmup iterations.
    for (int update = 0; update < 80; ++update)
      for (int order = 0; order < 3; ++order) {
        const int mode = (update + order) % 3;
        const auto start = std::chrono::steady_clock::now();
        NonlinearFactorGraph clamps;
        for (int a = 0; a < count; ++a)
          for (int b = 0; b < count; ++b)
            if (a != b) {
              const auto measured = poses[a].between(poses[b]);
              if (mode == 2)
                clamps.add(BetweenCandidate(
                    keyframe_id_t(a),
                    keyframe_id_t(b),
                    measured,
                    noise));
              else if (mode == 1)
                clamps.add(
                    OdometryFactor<Sim3>(
                        keyframe_id_t(a),
                        keyframe_id_t(b),
                        measured,
                        noise));
              else
                clamps.add(Between(
                    keyframe_id_t(a),
                    keyframe_id_t(b),
                    measured,
                    noise));
            }
        ISAM2 guarded = solvers[mode];
        const auto result = guarded.update(clamps, Values(), previous[mode]);
        solvers[mode] = std::move(guarded);
        previous[mode] = result.newFactorsIndices;
        const auto estimate = solvers[mode].calculateEstimate();
        const double milliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count();
        require(
            initial.equals(estimate, 1e-8),
            "Zero-residual window update changed its known solution");
        const Matrix covariance =
            solvers[mode].marginalCovariance(Symbol('k', count - 1));
        if (referenceCovariance[mode].size() == 0)
          referenceCovariance[mode] = covariance;
        else
          require(
              (covariance - referenceCovariance[mode]).norm() <=
                  1e-5 * referenceCovariance[mode].norm(),
              "Window replacement accumulated information");
        if (update >= 5)
          log << count << ',' << count * (count - 1) << ',' << update << ','
              << mode << ',' << milliseconds << '\n';
      }
  }
  std::cout << "Window replacement timings and stationary estimates verified\n";
}

int main(int argc_, char** argv_) {
  try {
    require(
        bool(settings::sharedSettings),
        "Missing production settings owner");
    (void)interface::InterfaceRerun::get_instance();
    if (argc_ != 3)
      throw std::invalid_argument(
          "Usage: test_sim3_derivatives trace|optimize|window OUTPUT.csv");
    if (std::string(argv_[1]) == "trace")
      trace(argv_[2]);
    else if (std::string(argv_[1]) == "optimize")
      optimization_trials(argv_[2]);
    else if (std::string(argv_[1]) == "window")
      window_costs(argv_[2]);
    else
      throw std::invalid_argument("Unknown test mode");
  } catch (const std::exception& error_) {
    std::cerr << error_.what() << '\n';
    return 1;
  }
}
