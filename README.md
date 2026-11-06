<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# SLAM-MER

A modern visual SLAM framework for map creation, localization, and rapid prototyping. It is a C++ visual SLAM library organized around modular mapping, localization, adjustment, and visualization components. It is designed to be flexible, with optional modules for depth inference and visual place localization.

> **Note:** This codebase is still under active development. If you find any issues or would like to discuss anything, please open an issue or contact Pedro Miraldo.

## Usage

### Download

Clone the repository and initialize submodules:
```bash
# Clone SLAM-MER
git clone https://github.com/slam-mer/slam-mer
# Go to the repository directory
cd slam-mer
# Sync optional module URLs (auto-selects private repos when this repository
# name ends with "_private")
./scripts/sync_optional_modules.sh
# Initialize and update submodules
git submodule update --init --recursive
```

### Compilation

For simplicity, we handle installation and configuration for this repository with `mise`. For detailed instructions, check the online documentation.

The code has been tested on Ubuntu 24.04 with CUDA 12.8, `CUDA_ARCH=86`, and `TORCH_CUDA_ARCH_LIST=8.6`. To simplify compilation, we use `mise` tasks. Please make sure `mise` is installed on your machine. Check and update `mise.toml` accordingly, especially the CUDA version and architecture. Then run:

```bash
# Set up trusted configuration
mise trust
# Install tools
mise install
# Create local Python environment
uv sync
```

Optional Python extras:

```bash
# Development tooling (pre-commit)
uv sync --extra dev
# Documentation tooling (Sphinx, Breathe, Exhale)
uv sync --extra docs
```

To install and register pre-commit hooks:

```bash
mise run setup_precommit
```

It is also recommended to install and configure [GitHub CLI](https://docs.github.com/en/github-cli/github-cli/quickstart) for downloading ONNX models from GitHub Releases.

Run the following commands from the root of `slam-mer` to build from scratch:

```bash
# Install system dependencies
mise run install_dependencies
# Build third-party dependencies
mise run build_thirdparties
# Configure SLAM-MER extensions
mise run configure_modules
# Build extensions
mise run build_modules
# Build SLAM-MER libraries
mise run build # or: mise run debug
```

To compile the core project without building or linking the depth inference and visual place localization modules, skip `mise run configure_modules` and `mise run build_modules`, then run:

```bash
mise run build_thirdparties
mise run build_without_depth_vpl
```


For the complete list of configurable CMake build flags, see [Compilation Options](COMPILATION_OPTIONS.md).

### Docker build (Ubuntu 22.04 hosts)

The supplied [Dockerfile](Dockerfile) provides Ubuntu 24.04, GCC 14, CUDA
12.8, and cuDNN without changing the host toolchain. The native setup is not
portable to Ubuntu 22.04 as written: `mise.toml` requires `/usr/bin/g++-14`
and `/usr/local/cuda-12.8`, and `install_dependencies` uses NVIDIA's Ubuntu
24.04 package repository. GCC 11 cannot compile the project's C++23
`std::println` calls. Do not run that dependency installer unchanged on 22.04.

Initialize submodules as described above, then build from the repository root:

```bash
docker build --target full --build-arg CUDA_ARCH=89 -t slam-mer:full .
docker run --rm -it --gpus all slam-mer:full
# Check GPU access, shared libraries, CUDA providers, and model files:
docker run --rm --gpus all slam-mer:full mise exec -- python docker/smoke_test.py
```

Set `CUDA_ARCH` for your GPU (`89` for RTX 40-series, `86` for RTX 30-series).
The `full` target enables ALIKED/LightGlue, depth inference, and visual place
localization, alongside CUDA SIFT and GPU FAISS. It downloads CUDA-enabled
LibTorch 2.9.0, ONNX Runtime 1.22.0, and the MAST3R, DUST3R, Depth Anything V2,
and MegaLoc model releases. ALIKED weights are included in the source checkout.
Public model downloads do not require GitHub login. Libraries are installed at
`/opt/slam-mer/build/install` inside the image. Build without `--gpus all`;
GPU access is needed at runtime. The host needs a working NVIDIA driver and
the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
to run GPU containers. Docker does not use the host CUDA toolkit.

If Docker reports `could not select device driver ... with capabilities: [[gpu]]`,
install and configure the host NVIDIA runtime with the supplied script:

```bash
# Run on the host after builds finish; this restarts Docker.
sudo bash docker/setup_nvidia_runtime.sh
docker run --rm --gpus all slam-mer:full nvidia-smi
```

Allow tens of GB of free disk space for the CUDA image, dependencies, and
build artifacts. The full profile also downloads LibTorch and ONNX models.
The container limits Ninja to two jobs by default; use
`--build-arg BUILD_JOBS=4` to change this during the build. Host build directories and virtual environments
are excluded from the Docker context to avoid incompatible cached paths.

For the smaller CI profile without ALIKED, depth inference, or visual place
localization, use `--target core -t slam-mer:core` instead. Both targets share
cached third-party dependencies. The default `datasets` target also adds the
example applications and the EuRoC runner described below.

For an interactive development environment, build the tools image and clone a fresh checkout
inside a named volume. This keeps container build files separate from host
builds and preserves Git metadata for model downloads:

```bash
docker build --target devel -t slam-mer:devel .
docker run --rm -it --gpus all \
  -v slam-mer-work:/work -w /work slam-mer:devel

# Inside the container:
git clone --recurse-submodules https://github.com/slam-mer/slam-mer.git
cd slam-mer
mise trust
mise install
uv sync
# Match CUDA_ARCH and TORCH_CUDA_ARCH_LIST in mise.toml to your GPU.
mise run build_thirdparties
mise run configure_modules
mise run build_modules
mise run build
```

System dependencies are already installed in this image, so skip
`mise run install_dependencies`. If GitHub release downloads require
authentication, run `gh auth login` interactively inside the container.

To control which transformation templates are compiled, use `-DSLAM_TRANSFORMATION_INSTANTIATIONS=ALL|POSE3|SIMILARITY3` (default: `ALL`).

What each step does:

1. `./scripts/sync_optional_modules.sh`
   Sets optional module submodule URLs based on the current repository naming:
   repositories ending with `_private` use private optional module repos.

2. `git submodule update --init --recursive`
   Initializes third-party submodules used by the project, including `thirdparty/gtsam`, `thirdparty/PoseLib`, `thirdparty/faiss`, `thirdparty/depth_inference`, and `thirdparty/visual_place_localization`.

3. `mise run install_dependencies`
   Installs Ubuntu packages used by the standard build, including Ninja, GCC/G++, CUDA Toolkit 12.8, Boost, GTSAM prerequisites, OpenBLAS/LAPACK, YAML-CPP, Eigen3, TBB, libpng, libjpeg-turbo, and cuDNN when available.

4. `mise run build_thirdparties`
   Builds and installs core dependencies required by SLAM-MER:
   - GTSAM (`thirdparty/gtsam/build/install`)
   - Rerun SDK (`thirdparty/rerun_cpp_sdk/build/install`)
   - PoseLib (`thirdparty/PoseLib/build/install`)
   - FAISS with GPU support (`thirdparty/faiss/build/install`)

5. `mise run configure_modules`
   Prepares the module repositories used by SLAM-MER by running their local `mise trust`, `mise install`, and `uv sync` workflows:
   - `thirdparty/depth_inference`
   - `thirdparty/visual_place_localization`

6. `mise run build_modules`
   Builds the two optional modules used by the default `mise run build` profile:
   - `depth_inference`: downloads ONNX Runtime and depth-model assets, then builds and installs to `thirdparty/depth_inference/build/install`
   - `visual_place_localization`: downloads ONNX Runtime and MegaLoc assets, builds FAISS for this module, then builds and installs to `thirdparty/visual_place_localization/build/install`

7. `mise run build`
   Downloads `thirdparty/libtorch` when missing, then configures and builds SLAM-MER libraries with CMake + Ninja and installs them to `build/install`. This task enables ALIKED, depth inference, and visual place localization by default.

### Examples

Examples are maintained in a separate project:
- https://github.com/slam-mer/slam-mer_examples

Please follow the instructions in that repository to build and run examples. If you find any issues, please let us know.

#### EuRoC from an existing dataset folder

To enter the existing dataset container from this repository, run:

```bash
./enter_docker.sh
```

This opens Bash in `/opt/slam-mer_examples`, with GPU access. The host project
is mounted at `/workspace`, and the sibling `unseen_v_slam/datasets` folder
is mounted read-only at `/datasets` when present. Override the dataset folder
with `SLAM_MER_DATASETS_DIR` or the image with `SLAM_MER_IMAGE`. Type `exit`
to leave the container.

The `datasets` Docker target builds the upstream examples at a pinned revision
and adds `slam-mer_euroc`. It reads the original EuRoC layout directly:
`DATASET_ROOT/SEQUENCE/mav0/cam0/{data.csv,sensor.yaml,data/}`.

```bash
docker build --target datasets --build-arg CUDA_ARCH=89 -t slam-mer:datasets .
bash scripts/run_euroc.sh /path/to/datasets/EuroC MH_01_easy
```

For the existing unseen_v_slam datasets on this machine:

```bash
bash scripts/run_euroc.sh \
  /home/tom/Documents/PhD/unseen_v_slam/datasets/EuroC MH_01_easy
```

The launcher mounts input data read-only. It writes `trajectory.txt` (TUM
format), a point cloud, and `recording.rrd` under
`output/euroc/SEQUENCE/<UTC timestamp>/`, owned by your host user. Set
`SLAM_MER_OUTPUT_DIR` to choose another output directory and `SLAM_MER_IMAGE`
to select another dataset image. Open `recording.rrd` in Rerun Viewer 0.25.1
afterward; no viewer is required inside Docker.
The launcher also saves terminal output to `run.log` in the same directory.
Use the host viewer shortcut to open the latest result with the matching
viewer version and a 4 GB memory limit:

```bash
./evaluation/view_recording.sh
# Or choose a specific recording:
./evaluation/view_recording.sh /path/to/recording.rrd
```

The shortcut prefers `recording-viewer.rrd` when a recovered copy is present
next to the latest `recording.rrd`. Set `SLAM_MER_VIEWER_MEMORY_LIMIT` to adjust
the viewer's memory budget. For file recordings, dense-map visualization is
sampled to at most 50,000 points at 1 Hz, camera images use lossless PNG, and
unchanged images are not logged repeatedly. This bounds recording overhead;
the PLY export retains the full final map.

Post-processing and evaluation utilities live in [evaluation/](evaluation/README.md).
The viewer uses an additional Z-up world frame for display; SLAM geometry
and trajectory/PLY exports retain their original optical coordinates.

The launcher limits each feature-matching FAISS scratch buffer to 64 MB to
leave GPU memory for the neural networks. Override
`SLAM_MER_FAISS_TEMP_MEMORY_MB` to tune this reservation; zero disables the
preallocated scratch buffers. Direct library users retain FAISS defaults
unless they set this variable.

The runner uses cam0 as a monocular camera, undistorts with `sensor.yaml`,
and scales the image and intrinsics together for inference. Capture times
come from `data.csv`, converted from nanoseconds to SLAM-MER's microsecond
timestamp representation. Stereo and IMU streams are not consumed. Output
poses describe cam0; EuRoC body-frame ground truth requires the corresponding
`T_BS` transform when evaluating them.

Check decoding without running SLAM, or restrict a trial run:

```bash
bash scripts/run_euroc.sh /path/to/datasets/EuroC MH_01_easy --check 1 --max-frames 5
bash scripts/run_euroc.sh /path/to/datasets/EuroC MH_01_easy --max-frames 100
```

Direct sequence, `mav0`, and `cam0` directory paths are also accepted in place
of the dataset root. The parent `datasets` folder is accepted when it contains
a `EuroC` subfolder, as in unseen_v_slam. To save recordings when calling an
example directly, set `SLAM_MER_RERUN_PATH` to an output `.rrd` path.

### Documentation

We also provide a Sphinx website with useful information about the code structure, class documentation, and contribution workflow. Use the following instructions to generate the documentation:

```bash
mise run install_docs_dependencies
mise run build_docs
```

If you prefer managing Python docs packages with `uv`, install the docs extra before building (with `doxygen` available on your system):

```bash
uv sync --extra docs
mise run build_docs
```

## Citation

If you use SLAM-MER in your research, please cite:

```bibtex
@inproceedings{merl2026revisiting,
  author = {Piedade, Valter and Manam, Lalit and Yamazaki, Masashi and Miraldo, Pedro},
  title = {Revisiting Monocular SLAM with Spatio-Temporal Scene Modeling},
  booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
  year = {2026}
}
```

## Contact

<b>Pedro Miraldo</b>
Senior Principal Research Scientist
Mitsubishi Electric Research Laboratories
miraldo@merl.com


<b>Valter Piedade</b>
PhD Student in Electrical and Computer Engineering
Instituto Superior Tecnico, Lisbon
(MERL consultant)

<b>Masashi Yamazaki</b>
Advanced Technology Center
Mitsubishi Electric Corporation

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for our policy on contributions.

## License

Released under the `AGPL-3.0-or-later` license, as described in [LICENSE.md](LICENSE.md).

All files, except those listed in [LICENSE-THIRD-PARTY.md](LICENSE-THIRD-PARTY.md), have:

```
Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL).

SPDX-License-Identifier: AGPL-3.0-or-later
```
