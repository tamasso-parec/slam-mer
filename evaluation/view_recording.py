# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

"""Open a recording in a visualization-only, Z-up world frame."""

import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile

import rerun as rr
import rerun.blueprint as rrb
import rerun_cli


def recording_identity(path, viewer):
    """Read only the stream header; do not load the recording into memory."""
    process = subprocess.Popen(
        [viewer, "rrd", "print", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    pattern = re.compile(
        r'store_id:\s*StoreId\(\s*Recording,\s*'
        r'("(?:\\.|[^"\\])*")\s*,\s*("(?:\\.|[^"\\])*")',
        re.DOTALL,
    )
    header = ""
    try:
        for line in process.stdout:
            header += line
            if match := pattern.search(header):
                return tuple(json.loads(value) for value in match.groups())
            if len(header) > 65536:
                break
    finally:
        # `rrd print` otherwise continues decoding the entire input. Its header
        # already provides the IDs needed to attach a tiny overlay recording.
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        process.stdout.close()
    raise ValueError(f"Cannot read recording identity from {path}")


def prepare_view(recording, directory, viewer):
    """Create metadata and a blueprint, without rewriting the input recording."""
    application_id, recording_id = recording_identity(recording, viewer)
    overlay = directory / "world-frame.rrd"
    blueprint_path = directory / "world-frame.rbl"
    stream = rr.RecordingStream(application_id, recording_id=recording_id)
    stream.save(overlay)
    # Parent-from-child: columns are the world's X, Y, Z axes expressed in
    # optical SLAM coordinates. This is the same basis logged by the C++ SDK.
    optical_from_world = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
    for optical, world in [("scene", "scene/world"), ("world", "world/visualization")]:
        stream.log(optical, rr.ViewCoordinates.RDF, static=True)
        stream.log(world, rr.ViewCoordinates.FLU, static=True)
        stream.log(
            world,
            rr.Transform3D(mat3x3=optical_from_world, axis_length=0.5),
            static=True,
        )
    stream.flush()
    stream.disconnect()

    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="scene/world",
                contents=["/scene/**", "/world/**"],
                name="World (X forward, Y left, Z up)",
            ),
            rrb.Spatial2DView(origin="scene/camera/image", name="Camera"),
            column_shares=[3, 1],
        ),
        auto_layout=False,
        auto_views=False,
    )
    blueprint.save(application_id, str(blueprint_path))
    return overlay, blueprint_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--memory-limit", default="4GB")
    parser.add_argument("--prepare-only", type=Path, metavar="DIRECTORY")
    arguments, viewer_arguments = parser.parse_known_args()
    recording = arguments.recording.resolve(strict=True)
    viewer = Path(rerun_cli.__file__).with_name("rerun")
    if not viewer.is_file():
        raise RuntimeError("Rerun executable not found in the SDK environment")
    viewer = str(viewer)
    if arguments.prepare_only:
        arguments.prepare_only.mkdir(parents=True, exist_ok=True)
        for artifact in prepare_view(recording, arguments.prepare_only, viewer):
            print(artifact)
        return 0
    with tempfile.TemporaryDirectory(prefix="slam-mer-rerun-") as temporary:
        overlay, blueprint = prepare_view(recording, Path(temporary), viewer)
        return subprocess.call(
            [viewer, "--memory-limit", arguments.memory_limit,
             str(recording), str(overlay), str(blueprint), *viewer_arguments]
        )


if __name__ == "__main__":
    raise SystemExit(main())
