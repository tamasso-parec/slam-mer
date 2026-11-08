#!/usr/bin/env python3
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Instrument a disposable source copy; never required by the production build.

The baseline branch skips only posterior capture/publication. Both branches keep
all estimation/map operations and the same timing/logging code. The source diff
is written next to the instrumented file for review. No estimator settings change.
"""
import argparse
import difflib
import hashlib
import pathlib
import re

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--reference", type=pathlib.Path, required=True)
parser.add_argument("--target", type=pathlib.Path, required=True)
args = parser.parse_args()
reference = args.reference.resolve()
target = args.target.resolve()
if reference == target:
    raise SystemExit("Target must be a disposable source copy, not the reference")
source = reference.read_text()
result = source.replace("#include <covisibility_graph.hpp>",
                        "#include <covisibility_graph.hpp>\n#include <fstream>\n#include <cstdlib>", 1)
marker = "  uint64_t posteriorRevision = 0;"
assert result.count(marker) == 1
result = result.replace(marker, marker + '''
  // Validation-only instrumentation; generated in a disposable checkout.
  const bool validationBaseline = std::getenv("SLAM_MER_VALIDATION_BASELINE") != nullptr;
  std::ofstream validationLog;
  if (const auto path = std::getenv("SLAM_MER_VALIDATION_TRACE")) validationLog.open(path);
  validationLog << "revision,variables,factor_slots,lock_wait_ms,guarded_update_ms,estimate_ms,publication_ms,capture_ms,map_commit_ms,status,factor_bytes\\n";
  using ValidationClock = std::chrono::steady_clock;
  auto validationMilliseconds = [](auto start_, auto end_) {
    return std::chrono::duration<double, std::milli>(end_ - start_).count();
  };
''', 1)
marker = '''      bool isamUpdated = false;
      {
        write_lock_t lock(_mutex);
        isamUpdated = optimizationManager.run_optimization<TransformationType>('''
assert result.count(marker) == 1
result = result.replace(marker, '''      bool isamUpdated = false;
      const auto validationLockStart = ValidationClock::now();
      auto validationUpdateStart = validationLockStart;
      {
        write_lock_t lock(_mutex);
        validationUpdateStart = ValidationClock::now();
        isamUpdated = optimizationManager.run_optimization<TransformationType>(''', 1)
marker = '''      if (isamUpdated) {
        const auto values = std::make_shared<Values>(isam.calculateEstimate());'''
assert result.count(marker) == 1
result = result.replace(marker, '''      const auto validationUpdateEnd = ValidationClock::now();
      if (isamUpdated) {
        const auto values = std::make_shared<Values>(isam.calculateEstimate());
        const auto validationEstimateEnd = ValidationClock::now();
        double validationMapMilliseconds = 0;
        auto validationCommit = [&] {
          const auto start = ValidationClock::now();
          __map_update(values);
          validationMapMilliseconds = validationMilliseconds(start, ValidationClock::now());
        };''', 1)
# Formatting hooks may wrap the arguments differently. Match the call rather
# than one particular line layout, while requiring exactly one replacement.
matches = list(re.finditer(
    r"(?m)^ {8}_posterior\.publish_after_update\(\s*isam,\s*\*values,"
    r"\s*\+\+posteriorRevision,\s*\[&\] \{ __map_update\(values\); \}\);", result))
assert len(matches) == 1
marker = matches[0].group(0)
result = result.replace(marker, '''        if (validationBaseline) {
          ++posteriorRevision;
          validationCommit();
        } else {
          _posterior.publish_after_update(
              isam, *values, ++posteriorRevision,
              validationCommit);
        }
        const auto validationPublicationEnd = ValidationClock::now();
        const auto validationSnapshot = _posterior.latest();
        validationLog << posteriorRevision << ',' << values->size() << ','
            << isam.getFactorsUnsafe().size() << ','
            << validationMilliseconds(validationLockStart, validationUpdateStart) << ','
            << validationMilliseconds(validationUpdateStart, validationUpdateEnd) << ','
            << validationMilliseconds(validationUpdateEnd, validationEstimateEnd) << ','
            << validationMilliseconds(validationEstimateEnd, validationPublicationEnd) << ','
            << (validationSnapshot ? validationSnapshot->capture_seconds() * 1000 : 0) << ','
            << validationMapMilliseconds << ','
            << (validationSnapshot ? int(validationSnapshot->status()) : -1) << ','
            << (validationSnapshot ? validationSnapshot->factor_bytes() : 0) << '\\n';''', 1)
target.write_text(result)
diff = ''.join(difflib.unified_diff(source.splitlines(True), result.splitlines(True),
                                  fromfile=str(reference), tofile=str(target)))
target.with_suffix('.validation.diff').write_text(diff)
print(f"Reference SHA256: {hashlib.sha256(source.encode()).hexdigest()}")
print(f"Instrumented SHA256: {hashlib.sha256(result.encode()).hexdigest()}")
print(f"Review diff: {target.with_suffix('.validation.diff')}")
