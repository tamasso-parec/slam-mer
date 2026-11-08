/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @brief Bounded headless production replay with a concurrent posterior reader.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/inference/Symbol.h>
#include <sys/resource.h>
#include <Eigen/Eigenvalues>
#include <atomic>
#include <chrono>
#include <dataloader_euroc.hpp>
#include <fstream>
#include <gaussian_posterior.hpp>
#include <interface_rerun.hpp>
#include <iomanip>
#include <iostream>
#include <slam.hpp>
#include <thread>

using namespace slam_mer;
using namespace slam_mer::covisibility_graph;
using Clock = std::chrono::steady_clock;
double elapsed_ms(Clock::time_point start_) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start_)
      .count();
}

/// @brief Tear down inference before static CUDA destruction, including on errors.
struct InferenceShutdown {
  ~InferenceShutdown() {
#ifdef DEPTH_INFERENCE
    if (data_type::SensorData::_depthEstimator) {
      try {
        data_type::SensorData::_depthEstimator->shutdown();
        data_type::SensorData::_depthEstimator.reset();
      } catch (const std::exception& error_) {
        std::cerr << "Inference shutdown failed: " << error_.what() << '\n';
      }
    }
#endif
  }
};

int main(int argc_, char** argv_) {
  try {
    // Link the production logging sink without starting interface workers/viewer.
    (void)interface::InterfaceRerun::get_instance();
    if (argc_ != 5 && argc_ != 6)
      throw std::invalid_argument(
          "Usage: validate_euroc_posterior SEQUENCE SETTINGS FRAMES OUTPUT_DIR "
          "[no-queries]");
    const bool query = argc_ == 5;
    if (!query && std::string(argv_[5]) != "no-queries")
      throw std::invalid_argument("Unknown validation mode");
    std::filesystem::create_directories(argv_[4]);
    const std::filesystem::path output(argv_[4]);
    std::ofstream frames(output / "frames.csv"),
        snapshots(output / "snapshots.csv");
    frames << "frame,add_ms,total_ms,keyframes,points\n";
    snapshots << "epoch,revision,status,poses,points,factor_bytes,capture_ms,"
                 "cold_ms,cached_ms,transport_ms,age_before_ms,age_after_ms\n";
    InferenceShutdown inferenceShutdown;
    // Settings must be loaded before sensor creation, as in the ordinary runner.
    auto slam = Slam::make_unique(argv_[2]);
    dataloader::DataloaderEuroc loader(argv_[1], std::stoull(argv_[3]));
    std::atomic<size_t> observed{0}, available{0}, failures{0};
    GaussianPosterior::shared_ptr retained;
    std::jthread reader([&](std::stop_token token_) {
      try {
        uint64_t last = 0, epoch = 0;
        while (!token_.stop_requested()) {
          const auto current = slam->get_posterior();
          if (!current || current->revision() == last) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
          }
          if ((epoch && current->epoch() != epoch) ||
              current->revision() <= last)
            throw std::runtime_error("Publication identities regressed");
          epoch = current->epoch();
          last = current->revision();
          ++observed;
          gtsam::KeyVector poses, points;
          for (auto key : current->estimates().keys()) {
            if (gtsam::Symbol(key).chr() == 'k')
              poses.push_back(key);
            else if (gtsam::Symbol(key).chr() == 'p')
              points.push_back(key);
          }
          const double ageBefore = elapsed_ms(current->captured_at());
          double cold = 0, cached = 0, transport = 0;
          if (current->status() == GaussianPosterior::Status::available) {
            ++available;
            if (query) {
              gtsam::KeyVector keys;
              if (!poses.empty()) keys.push_back(poses.front());
              if (poses.size() > 1) keys.push_back(poses.back());
              if (!points.empty()) keys.push_back(points.front());
              if (points.size() > 1) keys.push_back(points.back());
              auto start = Clock::now();
              const auto joint = current->joint_covariance(keys);
              cold = elapsed_ms(start);
              start = Clock::now();
              const auto repeated = current->joint_covariance(keys);
              cached = elapsed_ms(start);
              start = Clock::now();
              const auto moved = current->joint_covariance_at_estimate(keys);
              transport = elapsed_ms(start);
              if (joint.status != GaussianPosterior::Status::available ||
                  repeated.status != joint.status ||
                  moved.status != joint.status || joint.revision != last ||
                  joint.epoch != epoch ||
                  (joint.covariance - repeated.covariance).norm() != 0 ||
                  !moved.covariance.allFinite() ||
                  (joint.covariance - joint.covariance.transpose()).norm() >
                      1e-9 * std::max(1., joint.covariance.norm()))
                throw std::runtime_error("Joint query/identity check failed");
              if (joint.covariance.rows() &&
                  Eigen::SelfAdjointEigenSolver<gtsam::Matrix>(joint.covariance)
                          .eigenvalues()
                          .minCoeff() < -1e-9)
                throw std::runtime_error("Selected covariance is indefinite");
              if (retained &&
                  retained->status() == GaussianPosterior::Status::available) {
                const auto oldKeys = retained->estimates().keys();
                if (!oldKeys.empty() &&
                    retained->joint_covariance({oldKeys.front()}).status !=
                        GaussianPosterior::Status::available)
                  throw std::runtime_error("Old snapshot became unreadable");
              }
            }
          } else {
            ++failures;
            std::cerr << "Posterior unavailable at revision " << last << ": "
                      << current->message() << '\n';
          }
          snapshots << epoch << ',' << last << ',' << int(current->status())
                    << ',' << poses.size() << ',' << points.size() << ','
                    << current->factor_bytes() << ','
                    << 1000 * current->capture_seconds() << ',' << cold << ','
                    << cached << ',' << transport << ',' << ageBefore << ','
                    << elapsed_ms(current->captured_at()) << '\n';
          retained = current;
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      } catch (const std::exception& error_) {
        ++failures;
        std::cerr << "Reader failed: " << error_.what() << '\n';
      }
    });
    size_t count = 0;
    while (true) {
      const auto start = Clock::now();
      auto sample = loader.get_data();
      if (!sample) break;
      if (!sample->is_valid()) throw std::runtime_error("Invalid EuRoC sample");
      const auto addStart = Clock::now();
      slam->add(sample);
      const double add = elapsed_ms(addStart);
      const auto [keyframes, points, queryCells, closestCells, cells] =
          slam->get_map_size();
      frames << count++ << ',' << add << ',' << elapsed_ms(start) << ','
             << keyframes << ',' << points << '\n';
      // Bound playback to the native 20 Hz; slow frames are never dropped.
      std::this_thread::sleep_until(start + std::chrono::milliseconds(50));
      // The first inference can consume the playback interval. Let its queued
      // observations reach ingestion before asking localization for a map buffer.
      // This explicit 100 ms bootstrap grace is excluded from frame timings.
      if (count == 1)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Allow queued edges to be ingested; this is a fixed grace period, not a
    // claimed backend-drained barrier. Revisions continue for unchanged updates.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    reader.request_stop();
    reader.join();
    const auto last = slam->get_posterior();
    std::ofstream estimates(output / "backend_estimates.txt");
    estimates << std::setprecision(17);
    if (last) {
      for (const auto key : last->estimates().keys()) {
        estimates << gtsam::Symbol(key) << ' ';
        if (gtsam::Symbol(key).chr() == 'k') {
          if (settings::sharedSettings->covisibility_use_similarity()) {
            const auto pose = last->estimates().at<gtsam::Similarity3>(key);
            estimates << pose.rotation().rpy().transpose() << ' '
                      << pose.translation().transpose() << ' ' << pose.scale();
          } else {
            const auto pose = last->estimates().at<gtsam::Pose3>(key);
            estimates << pose.rotation().rpy().transpose() << ' '
                      << pose.translation().transpose();
          }
        } else
          estimates << last->estimates().at<gtsam::Point3>(key).transpose();
        estimates << '\n';
      }
    }
    slam.reset();
    if (retained &&
        retained->status() == GaussianPosterior::Status::available && query) {
      const auto keys = retained->estimates().keys();
      if (!keys.empty() && retained->joint_covariance({keys.front()}).status !=
                               GaussianPosterior::Status::available)
        throw std::runtime_error("Snapshot did not survive pipeline teardown");
    }

    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::ofstream summary(output / "summary.txt");
    summary << "frames=" << count << " observed=" << observed
            << " available=" << available << " query_failures=" << failures
            << " peak_rss_kib=" << usage.ru_maxrss << '\n';
    std::cout << "Frames=" << count << " observed=" << observed
              << " available=" << available << " failures=" << failures
              << " peak_rss_kib=" << usage.ru_maxrss << '\n';
    if (failures || (query && (!observed || !available))) return 1;
    return 0;
  } catch (const std::exception& error_) {
    std::cerr << error_.what() << '\n';
    return 1;
  }
}
