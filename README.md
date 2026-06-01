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
