<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# Evaluation

This directory holds post-processing tools and qualitative and quantitative
evaluation utilities. Dataset execution remains in `scripts/`; generated
results remain in `output/`.

## Visualize a recording

From the repository root:

```bash
./evaluation/view_recording.sh
./evaluation/view_recording.sh /path/to/recording.rrd
```

Without a path, the launcher selects the latest EuRoC run under
`output/euroc/`. It prefers `recording-viewer.rrd` when a recovered copy exists
beside the original recording. It uses Rerun 0.25.1 with a 4 GB memory limit;
set `SLAM_MER_VIEWER_MEMORY_LIMIT` to adjust that limit.

The 3D view uses a visualization-only world frame: X forward, Y left,
Z up. The original optical SLAM frame remains X right, Y down, Z forward.
Rerun applies one fixed basis change between these frames. The map points,
camera poses, `trajectory.txt`, and PLY exports are not rewritten or rotated.
This is a convention change, not an estimate of gravity or geographic heading.

The launcher adds a small temporary metadata overlay and an explicit viewer
layout, so this also works with existing recordings without rerunning SLAM.
The recording file itself is opened read-only. New recordings also contain
the `scene/world` frame directly. Select this frame as a 3D view's origin if
opening a new recording without the launcher.

Check the frame mapping and preservation of logged geometry with:

```bash
uv run --no-project --python 3.12 --with rerun-sdk==0.25.1 \
  python -m unittest discover -s evaluation -p 'test_world_frame.py' -v
```

## Evaluation inputs

Runs produce a TUM-format camera trajectory (`trajectory.txt`), a full final
point cloud (`<sequence>.ply`), a Rerun recording, and `run.log`.
Quantitative evaluation tools have not been added yet. When adding EuRoC
trajectory evaluation, account for the camera/body extrinsics, timestamp
association, and monocular scale alignment.
