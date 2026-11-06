# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

"""Download the public v1.0.0 model releases without Git metadata or gh login."""

import fnmatch
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from urllib.request import Request, urlopen


def download_models(repository, destination, patterns):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    request = Request(
        f"https://api.github.com/repos/slam-mer/{repository}/releases/tags/v1.0.0",
        headers={"Accept": "application/vnd.github+json", "User-Agent": "slam-mer-docker"},
    )
    with urlopen(request, timeout=60) as response:
        assets = json.load(response)["assets"]

    for pattern in patterns:
        matches = sorted(
            (asset for asset in assets if fnmatch.fnmatchcase(asset["name"], pattern)),
            key=lambda asset: asset["name"],
        )
        if not matches:
            raise RuntimeError(f"No release assets match {repository}: {pattern}")
        # Stream multipart archives into one temporary file, then remove it in
        # this layer so the image retains only extracted model files.
        with tempfile.TemporaryDirectory() as temporary:
            archive = Path(temporary) / "model.tar.gz"
            with archive.open("wb") as output:
                for asset in matches:
                    print(f"Downloading {repository}/{asset['name']}", flush=True)
                    subprocess.run(
                        ["curl", "-fL", "--retry", "5", "--output",
                         str(Path(temporary) / "part"), asset["browser_download_url"]],
                        check=True,
                    )
                    with (Path(temporary) / "part").open("rb") as part:
                        shutil.copyfileobj(part, output)
            subprocess.run(["tar", "-xzf", str(archive), "-C", str(destination)], check=True)


if __name__ == "__main__":
    download_models(
        "depth_inference", "thirdparty/depth_inference/assets",
        ["mast3r_onnx_model.tar.gz.part-*", "dust3r_onnx_model.tar.gz.part-*",
         "depth_anything_v2_onnx_model.tar.gz"],
    )
    download_models(
        "visual_place_localization",
        "thirdparty/visual_place_localization/assets/visual_place_recognition",
        ["megaloc_onnx_model.tar.gz"],
    )
