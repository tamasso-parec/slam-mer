#!/usr/bin/env python3
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Run one installed derivative version serially; preserve raw per-run timings."""
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
parser.add_argument("--label", choices=("legacy", "corrected"), required=True)
parser.add_argument("--frames", type=int, default=300)
parser.add_argument("--repeats", type=int, default=2)
args = parser.parse_args()
if args.frames < 1 or args.repeats < 1:
    raise SystemExit("Frame and repeat counts must be positive")
args.output.mkdir(parents=True, exist_ok=True)
records = []
for repeat in range(1, args.repeats + 1):
    directory = args.output / f"{args.label}-{repeat}"
    directory.mkdir(exist_ok=True)
    environment = dict(os.environ)
    environment.update(OMP_NUM_THREADS="2", SLAM_MER_FAISS_TEMP_MEMORY_MB="64",
                       SLAM_MER_VALIDATION_TRACE=str(directory / "backend.csv"))
    environment.pop("SLAM_MER_VALIDATION_BASELINE", None)
    environment.pop("SLAM_MER_SIM3_TRIAL", None)
    command = [str(args.executable), str(args.sequence), str(args.settings), str(args.frames), str(directory)]
    print(f"Starting {directory.name}", flush=True)
    start = time.monotonic()
    with (directory / "run.log").open("w") as log:
        result = subprocess.run(command, env=environment, cwd="/opt/slam-mer_examples", stdout=log, stderr=subprocess.STDOUT)
    records.append(dict(repeat=repeat, returncode=result.returncode,
                        elapsed_seconds=time.monotonic() - start, command=command))
    (args.output / f"{args.label}-runs.json").write_text(json.dumps(records, indent=2))
    if result.returncode:
        raise SystemExit(f"Failed: inspect {directory / 'run.log'}")
    print(f"Completed {directory.name}", flush=True)
