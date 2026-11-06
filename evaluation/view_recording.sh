#!/usr/bin/env bash
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -gt 0 ]]; then
  recording="$1"
  shift
else
  recording="$(python3 - "${repo_root}/output/euroc" <<'PY'
from pathlib import Path
import sys

recordings = list(Path(sys.argv[1]).glob('**/recording.rrd'))
if not recordings:
    sys.exit('No recordings found. Pass a recording path or run a sequence first.')
latest = max(recordings, key=lambda path: path.stat().st_mtime)
recovered = latest.with_name('recording-viewer.rrd')
print(recovered if recovered.is_file() else latest)
PY
)"
fi
if [[ ! -f "${recording}" ]]; then
  echo "Recording not found: ${recording}" >&2
  exit 1
fi
echo "Opening: ${recording}"
exec uv run --no-project --python 3.12 --with rerun-sdk==0.25.1 python \
  "${repo_root}/evaluation/view_recording.py" "${recording}" \
  --memory-limit "${SLAM_MER_VIEWER_MEMORY_LIMIT:-4GB}" "$@"
