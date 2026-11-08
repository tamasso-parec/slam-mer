#!/usr/bin/env python3
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Pool bounded Sim(3) posterior measurements; report nearest-rank quantiles."""
import argparse
import csv
import json
import math
from pathlib import Path
import re

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("root", type=Path)
args = parser.parse_args()


def statistics(values):
    values = sorted(values)
    return {name: values[math.ceil(q * len(values)) - 1]
            for name, q in (("p50", .5), ("p95", .95), ("p99", .99), ("maximum", 1.))}


def read_rows(paths):
    return [row for path in paths for row in csv.DictReader(path.open())]


def summarize(rows):
    result = {"samples": len(rows)}
    if rows:
        result["milliseconds"] = {key: statistics([float(row[key]) for row in rows])
                                  for key in rows[0] if key.endswith("_ms")}
        for key in ("variables", "factor_slots", "factor_bytes", "poses", "points"):
            if key in rows[0]:
                result["max_" + key] = max(int(row[key]) for row in rows)
        if "status" in rows[0]:
            result["status_counts"] = {status: sum(row["status"] == status for row in rows)
                                       for status in sorted({row["status"] for row in rows})}
    return result


report = {}
paired = read_rows(sorted(args.root.glob("paired-*.csv")))
report["paired"] = summarize(paired)
if paired:
    report["paired"]["milliseconds"]["report_total_ms"] = statistics(
        [float(row["report_update_ms"]) + float(row["capture_ms"]) for row in paired])
report["benchmark_peak_rss_kib"] = {}
for mode in ("baseline", "report"):
    path = args.root / (mode + ".log")
    if path.exists():
        report["benchmark_peak_rss_kib"][mode] = int(re.search(r"peak_rss_kib=(\d+)", path.read_text())[1])
for mode in ("baseline", "reporting", "queries"):
    directories = sorted((args.root / "replays").glob(mode + "-*"))
    result = {name: summarize(read_rows([d / (name + ".csv") for d in directories]))
              for name in ("backend", "frames", "snapshots")}
    result["runs"] = []
    for directory in directories:
        run = {"directory": directory.name,
               "summary": (directory / "summary.txt").read_text().strip()}
        scales = [float(line.split()[-1]) for line in (directory / "backend_estimates.txt").read_text().splitlines()
                  if line.startswith("k")]
        if scales:
            run["final_pose_scale_range"] = [min(scales), max(scales)]
        result["runs"].append(run)
    report[mode] = result
print(json.dumps(report, indent=2))
