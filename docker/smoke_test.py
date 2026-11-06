# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

"""Check GPU access, installed full-build libraries, and packaged model files."""

import argparse
import ctypes
import subprocess
from pathlib import Path


def check_full_image(require_gpu=True):
    root = Path(__file__).resolve().parent.parent
    if require_gpu:
        runtime = ctypes.CDLL("libcudart.so.12")
        count = ctypes.c_int()
        status = runtime.cudaGetDeviceCount(ctypes.byref(count))
        if status != 0 or count.value == 0:
            raise RuntimeError(f"CUDA device discovery failed: status={status}, devices={count.value}")
        print(f"CUDA devices available: {count.value}", flush=True)

    libraries = list((root / "build/install/lib").glob("*.so"))
    if not libraries:
        raise RuntimeError("No installed SLAM-MER shared libraries found")
    for library in libraries:
        # Core libraries use application-owned settings and interface globals.
        # Check their dependency paths without requiring a SLAM application.
        dependencies = subprocess.check_output(["ldd", str(library)], text=True)
        if "not found" in dependencies:
            raise RuntimeError(f"Unresolved dependencies for {library}:\n{dependencies}")
    print(f"Resolved dependencies for {len(libraries)} SLAM-MER libraries", flush=True)

    for module, library_name in [
        ("depth_inference", "libdepth_inference.so"),
        ("visual_place_localization", "libvisual_place_recognition.so"),
    ]:
        directory = root / "thirdparty" / module
        ctypes.CDLL(str(directory / "build/install/lib" / library_name))
        onnx = directory / "thirdparty/onnxruntime-linux-x64-gpu-1.22.0/lib"
        # Provider plugins are initialized by ONNX Runtime, not by plain dlopen.
        dependencies = subprocess.check_output(
            ["ldd", str(onnx / "libonnxruntime_providers_cuda.so")], text=True
        )
        if "not found" in dependencies:
            raise RuntimeError(f"Unresolved ONNX CUDA dependencies:\n{dependencies}")
        if not list((directory / "assets").rglob("*.onnx")):
            raise RuntimeError(f"No ONNX model files found for {module}")
        print(f"{module}: library, CUDA provider, and model files available", flush=True)

    if not (root / "utils/aliked/models/aliked-n32.pt").is_file():
        raise RuntimeError("ALIKED model weights are missing")
    # ALIKED is statically linked into image_processing; require its Torch
    # dependency so a reduced core image cannot pass this full-image check.
    ctypes.CDLL(str(root / "thirdparty/libtorch/lib/libtorch_cuda.so"))
    print("ALIKED weights and CUDA LibTorch available", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-gpu", action="store_true", help="Check libraries and models without GPU discovery")
    check_full_image(require_gpu=not parser.parse_args().skip_gpu)
