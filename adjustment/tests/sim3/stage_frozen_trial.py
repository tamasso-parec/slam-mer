#!/usr/bin/env python3
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Add a validation-only frozen-graph hook to a disposable backend source copy."""
import argparse
import difflib
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--reference", type=Path, required=True)
parser.add_argument("--target", type=Path, required=True)
args = parser.parse_args()
if args.reference.resolve() == args.target.resolve():
    raise SystemExit("Use a disposable target, not the reference")
source = args.reference.read_text()
result = source.replace('#include <covisibility_graph.hpp>', '#include <covisibility_graph.hpp>\n#include "../tests/sim3/frozen_graph_trial.hpp"', 1)
start = result.index('void CovisibilityGraph<TransformationType>::__optimize(std::stop_token token_)')
end = result.index('// Update map points', start)
part = result[start:end]
part = part.replace('  uint64_t posteriorRevision = 0;', '  uint64_t posteriorRevision = 0;\n  slam_mer::validation::FrozenGraphTrial derivativeTrial;', 1)
marker = '            [&] { __map_update(values); });'
assert part.count(marker) == 1
part = part.replace(marker, marker + '\n        if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>)\n          derivativeTrial.capture(isam);', 1)
marker = '    timer.pace(duration_t::from_fps(minimumFramesPerSecond));\n  }\n}'
assert part.count(marker) == 1
part = part.replace(marker, marker[:-1] + '  if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>)\n    derivativeTrial.run();\n}', 1)
result = result[:start] + part + result[end:]
args.target.write_text(result)
args.target.with_suffix('.frozen-trial.diff').write_text(''.join(difflib.unified_diff(source.splitlines(True), result.splitlines(True), fromfile=str(args.reference), tofile=str(args.target))))
