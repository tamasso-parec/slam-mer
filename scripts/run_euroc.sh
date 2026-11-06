#!/usr/bin/env bash
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

set -euo pipefail
if [[ $# -lt 2 ]]; then
  echo "Usage: bash scripts/run_euroc.sh DATASET_ROOT SEQUENCE [runner options]" >&2
  echo "Example: bash scripts/run_euroc.sh /path/to/datasets/EuroC MH_01_easy" >&2
  exit 1
fi
dataset_root="$(realpath -- "$1")"
sequence="$2"
shift 2
if [[ ! -d "${dataset_root}" ]]; then
  echo "Dataset directory does not exist: ${dataset_root}" >&2
  exit 1
fi
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${SLAM_MER_OUTPUT_DIR:-${repo_root}/output/euroc/${sequence}/$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p -- "${output_dir}"
output_dir="$(realpath -- "${output_dir}")"
echo "Results: ${output_dir}"
docker run --rm --gpus all --user "$(id -u):$(id -g)" \
  --env HOME=/tmp --env SLAM_MER_RERUN_PATH=/output/recording.rrd \
  --env "SLAM_MER_FAISS_TEMP_MEMORY_MB=${SLAM_MER_FAISS_TEMP_MEMORY_MB:-64}" \
  --mount "type=bind,source=${dataset_root},target=/datasets,readonly" \
  --mount "type=bind,source=${output_dir},target=/output" \
  "${SLAM_MER_IMAGE:-slam-mer:datasets}" \
  /opt/slam-mer_examples/build/slam-mer_euroc \
  --dataset /datasets --sequence "${sequence}" "$@" 2>&1 | tee "${output_dir}/run.log"
