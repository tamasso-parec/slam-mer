#!/usr/bin/env python3
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Summarize raw validation CSVs with nearest-rank percentiles, retaining maxima."""
import argparse
import csv
import json
import math
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("directories", type=Path, nargs="+")
args = parser.parse_args()
report = {}
for directory in args.directories:
    record = {}
    for name in ("backend", "frames", "snapshots"):
        path = directory / (name + ".csv")
        if not path.exists():
            continue
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        metrics = {"samples": len(rows)}
        if rows:
            for key in rows[0]:
                if key.endswith("_ms"):
                    values = sorted(float(row[key]) for row in rows)
                    metrics[key] = {f"p{int(p * 100)}": values[math.ceil(p * len(values)) - 1]
                                    for p in (.5, .95, .99)}
                    metrics[key]["maximum"] = values[-1]
            for key in ("variables", "factor_slots", "factor_bytes", "poses", "points"):
                if key in rows[0]:
                    metrics["max_" + key] = max(int(row[key]) for row in rows)
            if "status" in rows[0]:
                metrics["status_counts"] = {status: sum(row["status"] == status for row in rows)
                                             for status in sorted({row["status"] for row in rows})}
        record[name] = metrics
    summary = directory / "summary.txt"
    if summary.exists():
        record["summary"] = summary.read_text().strip()
    report[str(directory)] = record
print(json.dumps(report, indent=2))
