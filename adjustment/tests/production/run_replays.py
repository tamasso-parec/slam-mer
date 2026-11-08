#!/usr/bin/env python3
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Run serial bounded baseline/reporting/query replays in a configured container."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--executable", type=Path, required=True)
parser.add_argument("--sequence", type=Path, required=True)
parser.add_argument("--settings", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--frames", type=int, default=100)
parser.add_argument("--repeats", type=int, default=2)
parser.add_argument("--working-directory", type=Path, default=Path("/opt/slam-mer_examples"))
args = parser.parse_args()
if args.frames < 1 or args.repeats < 1:
    raise SystemExit("Frame/repeat counts must be positive")
args.output.mkdir(parents=True, exist_ok=True)
records = []
for repeat in range(1, args.repeats + 1):
    modes = ["baseline", "reporting", "queries"]
    if repeat % 2 == 0:
        modes.reverse()
    for mode in modes:
        directory = args.output / f"{mode}-{repeat}"
        directory.mkdir(exist_ok=True)
        environment = dict(os.environ)
        environment.update(OMP_NUM_THREADS="2", SLAM_MER_FAISS_TEMP_MEMORY_MB="64",
                           SLAM_MER_VALIDATION_TRACE=str(directory / "backend.csv"))
        environment.pop("SLAM_MER_VALIDATION_BASELINE", None)
        if mode == "baseline":
            environment["SLAM_MER_VALIDATION_BASELINE"] = "1"
        command = [str(args.executable), str(args.sequence), str(args.settings),
                   str(args.frames), str(directory)]
        if mode != "queries":
            command.append("no-queries")
        start = time.monotonic()
        print(f"Starting {mode}-{repeat}", flush=True)
        with (directory / "run.log").open("w") as log:
            completed = subprocess.run(command, env=environment, cwd=args.working_directory,
                                       stdout=log, stderr=subprocess.STDOUT)
        record = {"mode": mode, "repeat": repeat, "elapsed_seconds": time.monotonic() - start,
                  "returncode": completed.returncode, "command": command}
        records.append(record)
        (args.output / "runs.json").write_text(json.dumps(records, indent=2))
        if completed.returncode:
            raise SystemExit(f"Replay failed; inspect {directory / 'run.log'}")
        print(f"Finished {mode}-{repeat} in {record['elapsed_seconds']:.2f} s", flush=True)
