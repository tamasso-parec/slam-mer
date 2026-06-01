.. Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
.. SPDX-License-Identifier: AGPL-3.0-or-later

.. _installation:

Installation
============

This page provides the recommended installation workflow, matching the
procedure in the project ``README.md``.

For detailed task descriptions, manual CMake notes, and debugging guidance, see
:doc:`detailed_instalation`.

Download
--------

Clone the repository and initialize submodules:

.. code-block:: bash

  # Clone SLAM-MER
  git clone <slam-mer-repository-url>
  # Go to the repository directory
  cd slam-mer
  # Sync optional module URLs before initializing submodules
  ./scripts/sync_optional_modules.sh
  # Initialize and update submodules
  git submodule update --init --recursive

Setup
-----

The code was tested on Ubuntu 24.04 with CUDA 12.8, ``CUDA_ARCH=86``, and
``TORCH_CUDA_ARCH_LIST=8.6``. To simplify compilation, SLAM-MER uses ``mise``
tasks. Check and update ``mise.toml`` accordingly, especially the CUDA version
and architecture.

Run the following from the repository root:

.. code-block:: bash

  # Set up trusted configuration
  mise trust
  # Install tools
  mise install
  # Create venv and install dependencies
  uv sync

Build
-----

Build from scratch using the standard workflow:

.. code-block:: bash

  # Install system dependencies
  mise run install_dependencies
  # Build third-party dependencies
  mise run build_thirdparties
  # Configure SLAM-MER extensions
  mise run configure_modules
  # Build extensions
  mise run build_modules
  # Build SLAM-MER libraries
  mise run build

What Each Step Does
-------------------

1. ``./scripts/sync_optional_modules.sh``
   Sets optional module submodule URLs based on the current repository naming:
   repositories ending with ``_private`` use private optional module repos.
2. ``git submodule update --init --recursive``
   Initializes third-party submodules used by the project, including
   ``thirdparty/gtsam``, ``thirdparty/PoseLib``, ``thirdparty/faiss``,
   ``thirdparty/depth_inference``, and
   ``thirdparty/visual_place_localization``.
3. ``mise run install_dependencies``
   Installs Ubuntu packages used by the standard build, including Ninja,
   GCC/G++, CUDA Toolkit 12.8, Boost, GTSAM prerequisites, OpenBLAS/LAPACK,
   YAML-CPP, Eigen3, TBB, libpng, libjpeg-turbo, and cuDNN when available.
4. ``mise run build_thirdparties``
   Builds and installs core dependencies required by SLAM-MER:

   - GTSAM (``thirdparty/gtsam/build/install``)
   - Rerun SDK (``thirdparty/rerun_cpp_sdk/build/install``)
   - PoseLib (``thirdparty/PoseLib/build/install``)
   - FAISS with GPU support (``thirdparty/faiss/build/install``)
5. ``mise run configure_modules``
   Prepares module repositories used by SLAM-MER by running their local
   ``mise trust``, ``mise install``, and ``uv sync`` workflows:

   - ``thirdparty/depth_inference``
   - ``thirdparty/visual_place_localization``
6. ``mise run build_modules``
   Builds the optional modules used by the default ``mise run build`` profile:

   - ``depth_inference``: downloads ONNX Runtime and depth-model assets, then
     builds and installs to ``thirdparty/depth_inference/build/install``
   - ``visual_place_localization``: downloads
     ONNX Runtime and MegaLoc assets, builds FAISS for this module, then builds
     and installs to ``thirdparty/visual_place_localization/build/install``
7. ``mise run build``
   Downloads ``thirdparty/libtorch`` when missing, then configures and builds
   SLAM-MER libraries with CMake + Ninja and installs them to
   ``build/install``. This task enables ALIKED, depth inference, and visual
   place localization by default.

To control which transformation templates are compiled, pass
``-DSLAM_TRANSFORMATION_INSTANTIATIONS=ALL|POSE3|SIMILARITY3`` when
configuring CMake (default: ``ALL``).

References
----------

- Compilation flags: top-level ``COMPILATION_OPTIONS.md``
- Examples repository: `slam-mer_examples <https://github.com/slam-mer/slam-mer_examples>`_
- Detailed manual setup and extra notes: :doc:`detailed_instalation`
