/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <sys/resource.h>
#include <algorithm>
#include <fstream>
#include <gaussian_posterior.hpp>
#include <iostream>

using namespace gtsam;
using namespace slam_mer::covisibility_graph;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start_) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start_)
      .count();
}
void report(const char* name_, std::vector<double> samples_) {
  if (samples_.empty()) return;
  std::sort(samples_.begin(), samples_.end());
  auto percentile = [&](double p_) {
    return samples_.at(size_t(std::ceil(p_ * samples_.size())) - 1);
  };
  std::cout << name_ << " ms p50=" << percentile(.5)
            << " p95=" << percentile(.95) << " p99=" << percentile(.99)
            << " max=" << samples_.back() << '\n';
}
int main(int argc_, char** argv_) {
  const std::string mode = argc_ > 1 ? argv_[1] : "paired";
  const bool runBaseline = mode != "--report-only";
  const bool runReport = mode != "--baseline-only";
  ISAM2Params params;
  params.relinearizeSkip = 1;
  params.relinearizeThreshold = 0.01;
  ISAM2 baseline(params), reporting(params);
  auto calibration = std::make_shared<Cal3_S2>(300, 300, 0, 320, 240);
  std::vector<double> baselineTimes, reportTimes, publicationTimes, coldTimes,
      cachedTimes, ageTimes;
  size_t maxBytes = 0;
  GaussianPosterior::shared_ptr old;
  for (int update = 0; update < 80; ++update) {
    NonlinearFactorGraph graph;
    Values values;
    const Key poseKey = Symbol('k', update);
    Pose3 pose(Rot3(), Point3(.03 * update, 0, 0));
    values.insert(poseKey, pose.retract(Vector6::Constant(.001)));
    if (!update)
      graph.add(
          PriorFactor<Pose3>(
              poseKey,
              pose,
              noiseModel::Isotropic::Sigma(6, .01)));
    else
      graph.add(
          BetweenFactor<Pose3>(
              Symbol('k', update - 1),
              poseKey,
              Pose3(Rot3(), Point3(.03, 0, 0)),
              noiseModel::Isotropic::Sigma(6, .05)));
    // 120 persistent landmarks, each observed from every pose: deliberately
    // bounded, increasingly dense visual graph, not a recorded sequence.
    for (int j = 0; j < 120; ++j) {
      const Key pointKey = Symbol('p', j);
      Point3 point(.05 * (j % 12), .05 * (j / 12), 4. + .01 * j);
      if (!update) {
        values.insert(pointKey, point + Point3(.001, -.001, .001));
        graph.add(
            PriorFactor<Point3>(
                pointKey,
                point,
                noiseModel::Isotropic::Sigma(3, 2)));
      }
      const auto measurement =
          PinholeCamera<Cal3_S2>(pose, *calibration).project(point);
      graph.add(
          GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
              measurement,
              noiseModel::Isotropic::Sigma(2, 1),
              poseKey,
              pointKey,
              calibration));
    }
    auto start = Clock::now();
    // Include the backend's existing guarded-update copy in both paths.
    Values baselineEstimate;
    if (runBaseline) {
      ISAM2 baselineCopy = baseline;
      baselineCopy.update(graph, values);
      baseline = std::move(baselineCopy);
      baselineEstimate = baseline.calculateEstimate();
      baselineTimes.push_back(elapsed(start));
    }
    if (!runReport) continue;
    start = Clock::now();
    ISAM2 reportCopy = reporting;
    reportCopy.update(graph, values);
    reporting = std::move(reportCopy);
    const auto reportEstimate = reporting.calculateEstimate();
    const auto queryStart = Clock::now();
    auto snapshot =
        GaussianPosterior::capture(reporting, reportEstimate, update + 1);
    publicationTimes.push_back(elapsed(queryStart));
    reportTimes.push_back(elapsed(start));
    if (snapshot->status() != GaussianPosterior::Status::available ||
        (runBaseline && !baselineEstimate.equals(reportEstimate, 1e-12)))
      return 1;
    maxBytes = std::max(maxBytes, snapshot->factor_bytes());
    const KeyVector keys{
        poseKey,
        Symbol('p', update % 120),
        Symbol('p', (update + 1) % 120)};
    start = Clock::now();
    const auto cold = snapshot->joint_covariance(keys);
    coldTimes.push_back(elapsed(start));
    if (cold.status != GaussianPosterior::Status::available) return 2;
    start = Clock::now();
    snapshot->joint_covariance(keys);
    cachedTimes.push_back(elapsed(start));
    ageTimes.push_back(elapsed(snapshot->captured_at()));
    if (old && old->joint_covariance({Symbol('k', 0)}).status !=
                   GaussianPosterior::Status::available)
      return 3;
    old = std::move(snapshot);
  }
  report("baseline update+copy+estimate", baselineTimes);
  report("reporting update+copy+estimate+capture", reportTimes);
  report("capture", publicationTimes);
  report("cold pose+two points joint", coldTimes);
  report("cached joint", cachedTimes);
  report("age at first query completion", ageTimes);
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  std::cout << "maximum sparse factor bytes=" << maxBytes
            << " process peak RSS KiB=" << usage.ru_maxrss
            << " (paired mode includes both optimizers)\n";
  std::cout << "80 updates, 80 poses, 120 points; paired mode checks estimates "
               "at 1e-12\n";
}
