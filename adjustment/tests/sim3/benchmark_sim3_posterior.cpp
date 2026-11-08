/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @brief Bounded identical-graph Sim(3) reporting cost and estimate invariance.
#include <sys/resource.h>
#include <algorithm>
#include <chrono>
#include <covisibility_factors.hpp>
#include <fstream>
#include <gaussian_posterior.hpp>
#include <interface_rerun.hpp>
#include <iostream>
#include <settings.hpp>

using namespace gtsam;
using namespace slam_mer;
using namespace slam_mer::covisibility_graph;
using Sim3 = gtsam::Similarity3;
using Clock = std::chrono::steady_clock;
double milliseconds(Clock::time_point start_) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start_)
      .count();
}

int main(int argc_, char** argv_) {
  try {
    if (!settings::sharedSettings)
      throw std::runtime_error("Missing production settings owner");
    (void)interface::InterfaceRerun::get_instance();
    if (argc_ != 3)
      throw std::invalid_argument(
          "Usage: benchmark_sim3_posterior paired|baseline|report OUTPUT.csv");
    const std::string mode = argv_[1];
    if (mode != "paired" && mode != "baseline" && mode != "report")
      throw std::invalid_argument("Unknown benchmark mode");
    const bool base = mode != "report", reporting = mode != "baseline";
    std::ofstream output(argv_[2]);
    if (!output) throw std::runtime_error("Cannot open benchmark output");
    output << "update,variables,baseline_ms,report_update_ms,capture_ms,cold_"
              "ms,cached_ms,transport_ms,age_ms,factor_bytes\n";
    ISAM2Params parameters;
    parameters.relinearizeSkip = 1;
    parameters.relinearizeThreshold = .01;
    ISAM2 baseline(parameters), solver(parameters);
    GaussianPosterior::shared_ptr old;
    FactorIndices previous[2];
    const auto calibration = std::make_shared<Cal3_S2>(300, 310, 0, 320, 240);
    const auto robust = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345),
        noiseModel::Isotropic::Sigma(3, .1));
    std::vector<Sim3> poses;
    for (int update = 0; update < 40; ++update) {
      const Key poseKey = Symbol('k', update);
      poses.emplace_back(
          Rot3::RzRyRx(.001 * update, -.002 * update, .003 * update),
          Point3(.03 * update, .005 * update, 0),
          .6 * std::exp(.02 * update));
      NonlinearFactorGraph graph;
      Values values;
      values.insert(poseKey, poses.back().retract(Vector7::Constant(.001)));
      if (!update)
        graph.add(
            PosePriorFactor<Sim3>(
                poseKey,
                poses[0],
                noiseModel::Isotropic::Sigma(7, .03)));
      else
        graph.add(
            OdometryFactor<Sim3>(
                keyframe_id_t(update - 1),
                keyframe_id_t(update),
                poses[update - 1].between(poses[update]),
                noiseModel::Isotropic::Sigma(7, .1)));
      for (int j = 0; j < 40; ++j) {
        const Key pointKey = Symbol('p', j);
        const Point3 point(.08 * (j % 8), .08 * (j / 8), 4. + .02 * j);
        if (!update) {
          values.insert(pointKey, Point3(point + Vector3(.001, -.001, .002)));
          graph.add(
              PriorFactor<Point3>(
                  pointKey,
                  point,
                  noiseModel::Isotropic::Sigma(3, 2)));
        }
        const auto camera = poses.back().inverse().transformFrom(point);
        graph.push_back(
            PointDistanceFactor<Sim3>::make_shared(
                keyframe_id_t(update),
                point_id_t(j),
                camera,
                robust));
        graph.push_back(
            slam_mer::covisibility_graph::ProjectionFactor<Sim3>::make_shared(
                keyframe_id_t(update),
                point_id_t(j),
                calibration,
                calibration->uncalibrate(
                    Point2(camera.x() / camera.z(), camera.y() / camera.z())),
                noiseModel::Isotropic::Sigma(2, 1)));
      }
      const size_t persistent = graph.size();
      for (int a = std::max(0, update - 9); a <= update; ++a)
        for (int b = std::max(0, update - 9); b <= update; ++b)
          if (a != b)
            graph.add(
                OdometryFactor<Sim3>(
                    keyframe_id_t(a),
                    keyframe_id_t(b),
                    poses[a].between(poses[b]),
                    noiseModel::Isotropic::Sigma(7, .01)));
      double baselineTime = 0, reportTime = 0;
      Values baselineEstimate, estimate;
      // Alternate measurement order to reduce a fixed first-run cache bias.
      for (int pass = 0; pass < 2; ++pass) {
        const int side = (update + pass) % 2;
        if ((side == 0 && !base) || (side == 1 && !reporting)) continue;
        auto& optimizer = side == 0 ? baseline : solver;
        const auto start = Clock::now();
        ISAM2 guarded = optimizer;
        const auto result = guarded.update(graph, values, previous[side]);
        optimizer = std::move(guarded);
        auto currentEstimate = optimizer.calculateEstimate();
        const double elapsed = milliseconds(start);
        previous[side].assign(
            result.newFactorsIndices.begin() + persistent,
            result.newFactorsIndices.end());
        if (side == 0) {
          baselineTime = elapsed;
          baselineEstimate = std::move(currentEstimate);
        } else {
          reportTime = elapsed;
          estimate = std::move(currentEstimate);
        }
      }
      double captureTime = 0, coldTime = 0, cachedTime = 0, transportTime = 0,
             age = 0;
      size_t bytes = 0;
      if (reporting) {
        auto start = Clock::now();
        const auto snapshot =
            GaussianPosterior::capture(solver, estimate, update + 1);
        captureTime = milliseconds(start);
        if (snapshot->status() != GaussianPosterior::Status::available)
          throw std::runtime_error(snapshot->message());
        const KeyVector keys{
            Symbol('k', 0),
            poseKey,
            Symbol('p', 0),
            Symbol('p', 39)};
        // At the first update only one unique pose exists.
        const KeyVector selection =
            update ? keys : KeyVector{poseKey, Symbol('p', 0), Symbol('p', 39)};
        start = Clock::now();
        const auto cold = snapshot->joint_covariance(selection);
        coldTime = milliseconds(start);
        start = Clock::now();
        const auto cached = snapshot->joint_covariance(selection);
        cachedTime = milliseconds(start);
        start = Clock::now();
        const auto moved = snapshot->joint_covariance_at_estimate(selection);
        transportTime = milliseconds(start);
        if (cold.status != GaussianPosterior::Status::available ||
            cached.status != cold.status || moved.status != cold.status)
          throw std::runtime_error("Benchmark query unavailable");
        if (base && !baselineEstimate.equals(estimate, 1e-12))
          throw std::runtime_error(
              "Reporting changed identical-graph estimates");
        if (!estimate.equals(solver.calculateEstimate(), 1e-12))
          throw std::runtime_error("Query changed estimates");
        if (old && old->joint_covariance({Symbol('k', 0)}).status !=
                       GaussianPosterior::Status::available)
          throw std::runtime_error("Retained snapshot invalidated");
        bytes = snapshot->factor_bytes();
        age = milliseconds(snapshot->captured_at());
        old = snapshot;
      }
      output << update << ',' << 40 + update + 1 << ',' << baselineTime << ','
             << reportTime << ',' << captureTime << ',' << coldTime << ','
             << cachedTime << ',' << transportTime << ',' << age << ',' << bytes
             << '\n';
    }
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::cout << "mode=" << mode
              << " updates=40 poses=40 points=40 peak_rss_kib="
              << usage.ru_maxrss << '\n';
  } catch (const std::exception& error_) {
    std::cerr << error_.what() << '\n';
    return 1;
  }
}
