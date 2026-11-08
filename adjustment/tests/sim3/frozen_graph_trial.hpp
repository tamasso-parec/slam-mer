/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @brief Validation-only frozen real-graph experiment, run after the worker stops.
#pragma once
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "derivative_candidates.hpp"
#include "legacy_factors.hpp"

namespace slam_mer::validation {
class FrozenGraphTrial {
public:
  /// @brief Capture a small accepted real graph once; no output or optimization here.
  void capture(const gtsam::ISAM2& isam_) {
    const char* path = std::getenv("SLAM_MER_SIM3_TRIAL");
    if (!path || !_values.empty() || isam_.getLinearizationPoint().size() < 150)
      return;
    _path = path;
    _values = isam_.calculateEstimate();
    for (const auto& factor : isam_.getFactorsUnsafe()) {
      if (!factor) continue;
      using Distance = covisibility_graph::PointDistanceFactor<Sim3>;
      using Projection = covisibility_graph::ProjectionFactor<Sim3>;
      using Between = covisibility_graph::OdometryFactor<Sim3>;
      if (auto value = std::dynamic_pointer_cast<Distance>(factor)) {
        _graphs[0].add(legacy::Distance(*value));
        _graphs[1].add(DistanceCandidate(*value));
        _graphs[2].add(DistanceCandidate(*value));
        _graphs[3].add(NumericalVisual<Distance>(*value));
      } else if (auto value = std::dynamic_pointer_cast<Projection>(factor)) {
        _graphs[0].add(legacy::Projection(*value));
        _graphs[1].add(ProjectionCandidate(*value));
        _graphs[2].add(ProjectionCandidate(*value));
        _graphs[3].add(NumericalVisual<Projection>(*value));
      } else if (auto value = std::dynamic_pointer_cast<Between>(factor)) {
        _graphs[0].add(legacy::Between(*value));
        _graphs[1].add(legacy::Between(*value));
        _graphs[2].add(BetweenCandidate(*value));
        _graphs[3].add(BetweenCandidate(*value));
      } else if (
          auto value =
              std::dynamic_pointer_cast<gtsam::PriorFactor<Sim3>>(factor)) {
        _graphs[0].add(gtsam::PriorFactor<Sim3>(*value));
        _graphs[1].add(gtsam::PriorFactor<Sim3>(*value));
        _graphs[2].add(PriorCandidate(*value));
        _graphs[3].add(PriorCandidate(*value));
      } else
        throw std::runtime_error("Unsupported factor in frozen graph trial");
    }
  }

  /// @brief Compare the copied nonlinear problems after ingestion/optimization stop.
  /// @details No result is written back to the map or optimizer. Run times here
  /// are diagnostic batch costs, not backend or trajectory measurements.
  void run() const {
    if (_values.empty()) return;
    std::ofstream output(_path);
    output << std::setprecision(12);
    output << "mode,variables,scalars,factors,initial_cost,gradient_relative_"
              "error,final_cost,final_gradient_norm,iterations,milliseconds\n";
    const auto initialGradient = __gradient(_values);
    for (size_t mode = 0; mode < 4; ++mode) {
      const auto& graph = _graphs[mode];
      const double initialCost = _graphs[0].error(_values);
      if (std::abs(graph.error(_values) - initialCost) >
          1e-9 * std::max(1., initialCost))
        throw std::runtime_error(
            "Frozen candidate changed the nonlinear objective");
      const auto linear = graph.linearize(_values)->hessian();
      const double discrepancy = (-linear.second - initialGradient).norm() /
                                 std::max(1., initialGradient.norm());
      const auto start = std::chrono::steady_clock::now();
      gtsam::LevenbergMarquardtParams parameters;
      parameters.maxIterations = 80;
      parameters.relativeErrorTol = 1e-9;
      parameters.absoluteErrorTol = 1e-9;
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, _values, parameters);
      const auto result = optimizer.optimize();
      const double milliseconds = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - start)
                                      .count();
      output << mode << ',' << _values.size() << ',' << _values.dim() << ','
             << graph.size() << ',' << initialCost << ',' << discrepancy << ','
             << _graphs[0].error(result) << ',' << __gradient(result).norm()
             << ',' << optimizer.iterations() << ',' << milliseconds << '\n';
      output.flush();
    }
    std::cout << "Frozen Sim3 graph trial saved to " << _path << '\n';
  }

private:
  gtsam::Vector __gradient(const gtsam::Values& values_) const {
    gtsam::Vector result(values_.dim());
    size_t index = 0;
    const auto keys = values_.keys();
    gtsam::VectorValues increment;
    for (const auto key : keys)
      increment.insert(key, gtsam::Vector::Zero(values_.at(key).dim()));
    for (const auto key : keys) {
      for (size_t j = 0; j < values_.at(key).dim(); ++j) {
        increment.at(key)(j) = 1e-6;
        const double plus = _graphs[0].error(values_.retract(increment));
        increment.at(key)(j) = -1e-6;
        const double minus = _graphs[0].error(values_.retract(increment));
        increment.at(key)(j) = 0;
        result(index++) = (plus - minus) / 2e-6;
      }
    }
    return result;
  }
  gtsam::Values _values;
  gtsam::NonlinearFactorGraph _graphs[4];
  std::string _path;
};
}  // namespace slam_mer::validation
