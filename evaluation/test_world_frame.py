# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

"""Verify the world-frame overlay changes viewing without changing data."""

import hashlib
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
import rerun as rr
import rerun.dataframe as rdf
import rerun_cli

from view_recording import prepare_view


class WorldFrameTest(unittest.TestCase):
    def test_overlay_preserves_geometry_and_expresses_forward_left_up(self):
        viewer = str(Path(rerun_cli.__file__).with_name("rerun"))
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            original = directory / "original.rrd"
            stream = rr.RecordingStream("test_world", recording_id="test_world_frame")
            stream.save(original)
            stream.log("scene", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
            stream.log(
                "scene/point_cloud", rr.Points3D([[1, 2, 3], [-4, 5, 6]]),
                static=True,
            )
            stream.log(
                "scene/camera", rr.Transform3D(translation=[1, -2, 3]),
                static=True,
            )
            stream.flush()
            stream.disconnect()
            before = hashlib.sha256(original.read_bytes()).digest()
            overlay, blueprint = prepare_view(original, directory, viewer)
            self.assertEqual(before, hashlib.sha256(original.read_bytes()).digest())

            subprocess.run([viewer, "rrd", "verify", str(overlay), str(blueprint)],
                           check=True, capture_output=True)
            merged = directory / "merged.rrd"
            subprocess.run(
                [viewer, "rrd", "route", "-o", str(merged),
                 str(original), str(overlay)],
                check=True, capture_output=True,
            )
            source = rdf.load_recording(str(original))
            result = rdf.load_recording(str(merged))
            for entity in ["scene/point_cloud", "scene/camera"]:
                expected = source.view(index=None, contents=entity).select().read_all()
                actual = result.view(index=None, contents=entity).select().read_all()
                self.assertTrue(expected.equals(actual), entity)

            frame = result.view(index=None, contents="scene/world").select().read_all()
            matrix = next(name for name in frame.column_names if "mat3x3" in name)
            columns = frame.column(matrix)[0].as_py()[0]
            optical_from_world = np.asarray(columns).reshape(3, 3, order="F")
            world_from_optical = optical_from_world.T
            np.testing.assert_allclose(world_from_optical @ [0, 0, 1], [1, 0, 0])
            np.testing.assert_allclose(world_from_optical @ [1, 0, 0], [0, -1, 0])
            np.testing.assert_allclose(world_from_optical @ [0, 1, 0], [0, 0, -1])
            self.assertEqual(np.linalg.det(world_from_optical), 1)


if __name__ == "__main__":
    unittest.main()
