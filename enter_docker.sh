#!/usr/bin/env bash
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
options=(--rm --gpus all --interactive)
if [[ -t 0 && -t 1 ]]; then
  options+=(--tty)
fi
options+=(--mount "type=bind,source=${repo_root},target=/workspace")
dataset_root="${SLAM_MER_DATASETS_DIR:-${repo_root}/../unseen_v_slam/datasets}"
if [[ -d "${dataset_root}" ]]; then
  dataset_root="$(realpath -- "${dataset_root}")"
  options+=(--mount "type=bind,source=${dataset_root},target=/datasets,readonly")
fi
if [[ $# -eq 0 ]]; then
  set -- bash
fi
exec docker run "${options[@]}" "${SLAM_MER_IMAGE:-slam-mer:datasets}" "$@"
