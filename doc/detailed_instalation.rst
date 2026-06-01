.. Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
.. SPDX-License-Identifier: AGPL-3.0-or-later

.. _detailed_instalation:
.. _detailed_installation:

Detailed Installation
=====================

This page expands the recommended installation workflow from :doc:`install`.
It lists the shell commands, environment values, package names, and CMake
options used by the repository tasks so the build configuration is visible. The
values are expanded from the top-level and module-local ``mise.toml`` files,
but the steps below use direct commands.

-------------
Dependencies
-------------

The code was implemented and tested with
`Ubuntu 24.04 <https://releases.ubuntu.com/noble/>`_, CUDA 12.8,
``CUDA_ARCH=86``, and ``TORCH_CUDA_ARCH_LIST=8.6``. Adjust the CUDA paths and
architecture values for your GPU before building on a different machine.

The standard tool versions used by the repository are:

- CMake 3.26
- Python 3.12.12
- uv
- clang-format 21.1.8, for development formatting checks

Set the CUDA and runtime environment from the repository root:

.. code-block:: bash

  export CUDA_HOME=/usr/local/cuda-12.8
  export CUDA_VERSION=12.8
  export CUDA_ARCH=86
  export TORCH_CUDA_ARCH_LIST=8.6
  export RERUN_VERSION=0.25.1
  export ONNX_VERSION=1.22.0
  export ASAN_OPTIONS=protect_shadow_gap=0
  export PATH="${CUDA_HOME}/bin:${PATH}"
  export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH:-}"
  export CPLUS_INCLUDE_PATH="${CUDA_HOME}/targets/x86_64-linux/include:${CPLUS_INCLUDE_PATH:-}"
  export LIBRARY_PATH="${CUDA_HOME}/targets/x86_64-linux/lib:${LIBRARY_PATH:-}"

Install the CUDA repository keyring and standard system dependencies:

.. code-block:: bash

  wget -qO /tmp/cuda-keyring.deb \
    https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
  sudo dpkg -i /tmp/cuda-keyring.deb
  rm -f /tmp/cuda-keyring.deb

  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    ninja-build \
    g++-14 gcc-14 \
    gcc-9 g++-9 \
    git libssl-dev \
    libboost-all-dev \
    libgoogle-glog-dev \
    cuda-toolkit-12-8 \
    libgflags-dev \
    libatlas-base-dev \
    libopenblas-dev \
    liblapack-dev \
    libyaml-cpp-dev \
    libeigen3-dev \
    libtbb-dev \
    libpng-dev \
    libjpeg-turbo8-dev \
    wget unzip

  if apt-cache show libcudnn9-cuda-12 >/dev/null 2>&1; then
    sudo apt-get install -y libcudnn9-cuda-12
  elif apt-cache show cudnn9-cuda-12 >/dev/null 2>&1; then
    sudo apt-get install -y cudnn9-cuda-12
  fi

Install OpenCV when building the visual place localization module with its
OpenCV-dependent components:

.. code-block:: bash

  sudo apt-get install -y libopencv-dev

--------------------
Clone the Repository
--------------------

Clone the repository and download the third-party libraries:

.. code-block:: bash

  git clone <slam-mer-repository-url>
  cd slam-mer
  ./scripts/sync_optional_modules.sh
  git submodule update --init --recursive

------------------------
Python Tool Environment
------------------------

Create the Python environment from the repository root:

.. code-block:: bash

  uv sync

Use optional Python extras when needed:

.. code-block:: bash

  uv sync --extra dev
  uv sync --extra docs

The ``dev`` extra installs pre-commit. The ``docs`` extra installs Sphinx,
the Read the Docs theme, Breathe, and Exhale.

-----------------------------
Compile Third-Party Libraries
-----------------------------

Build GTSAM from source:

.. code-block:: bash

  if [ ! -d "thirdparty/gtsam/build/install" ]; then
    if git -C thirdparty/gtsam apply --unidiff-zero --reverse --check \
        ../gtsam_similarity3.patch >/dev/null 2>&1; then
      echo "thirdparty/gtsam_similarity3.patch already applied."
    elif git -C thirdparty/gtsam apply --unidiff-zero --check \
        ../gtsam_similarity3.patch; then
      git -C thirdparty/gtsam apply --unidiff-zero ../gtsam_similarity3.patch
      echo "Applied thirdparty/gtsam_similarity3.patch."
    else
      echo "Failed to apply thirdparty/gtsam_similarity3.patch cleanly." >&2
      exit 1
    fi
  else
    echo "GTSAM install exists, skipping patch application."
  fi

  cmake -B thirdparty/gtsam/build -S thirdparty/gtsam \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_SUPPORT_NESTED_DISSECTION=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_WITH_TBB=ON \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_LIBRARY_TYPE=SHARED \
        -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
        -DCMAKE_INSTALL_PREFIX=./thirdparty/gtsam/build/install \
        -DCMAKE_INSTALL_MESSAGE=NEVER \
        -GNinja
  ninja -C thirdparty/gtsam/build
  ninja -C thirdparty/gtsam/build install

Build Rerun SDK 0.25.1:

.. code-block:: bash

  if [ ! -d "thirdparty/rerun_cpp_sdk" ]; then
    wget -O thirdparty/rerun_cpp_sdk.zip \
      "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip"
  fi

  if [ -f "thirdparty/rerun_cpp_sdk.zip" ]; then
    unzip -o thirdparty/rerun_cpp_sdk.zip -d thirdparty
    rm -f thirdparty/rerun_cpp_sdk.zip
  fi

  cmake -B thirdparty/rerun_cpp_sdk/build -S thirdparty/rerun_cpp_sdk \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=thirdparty/rerun_cpp_sdk/build/install \
        -DCUDA_NVRTC_SHORTHASH=IGNORE \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_MESSAGE=NEVER \
        -GNinja
  ninja -C thirdparty/rerun_cpp_sdk/build rerun_sdk
  ninja -C thirdparty/rerun_cpp_sdk/build install

Build PoseLib:

.. code-block:: bash

  cmake -B thirdparty/PoseLib/build -S thirdparty/PoseLib \
        -DBUILD_SHARED_LIBS=ON \
        -DWITH_BENCHMARK=OFF \
        -DBUILD_TESTS=OFF \
        -DPYTHON_PACKAGE=OFF \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=thirdparty/PoseLib/build/install \
        -DCMAKE_INSTALL_MESSAGE=NEVER \
        -GNinja
  ninja -C thirdparty/PoseLib/build
  ninja -C thirdparty/PoseLib/build install

Build FAISS with GPU support:

.. code-block:: bash

  cmake -B thirdparty/faiss/build -S thirdparty/faiss \
        -DBUILD_SHARED_LIBS=ON \
        -DFAISS_ENABLE_GPU=ON \
        -DFAISS_ENABLE_MKL=OFF \
        -DBLA_VENDOR=OpenBLAS \
        -DFAISS_OPT_LEVEL=avx2 \
        -DCUDAToolkit_ROOT=/usr/local/cuda-${CUDA_VERSION} \
        -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} \
        -DBUILD_TESTING=OFF \
        -DFAISS_ENABLE_PYTHON=OFF \
        -DFAISS_ENABLE_C_API=OFF \
        -DFAISS_ENABLE_EXTRAS=OFF \
        -DFAISS_USE_LTO=OFF \
        -DCMAKE_CUDA_COMPILER=/usr/local/cuda-${CUDA_VERSION}/bin/nvcc \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=thirdparty/faiss/build/install \
        -DCMAKE_INSTALL_MESSAGE=NEVER \
        -GNinja
  ninja -C thirdparty/faiss/build
  ninja -C thirdparty/faiss/build install

----------------------
Optional SLAM Modules
----------------------

The optional modules live in ``thirdparty/depth_inference`` and
``thirdparty/visual_place_localization``. Both modules use CUDA 12.8 and ONNX
Runtime 1.22.0. The module-local configuration files currently list
``CUDA_ARCH=89``; keep the top-level ``CUDA_ARCH=86`` value above unless you
are intentionally changing the target GPU architecture.

Prepare and build the depth inference module:

.. code-block:: bash

  cd thirdparty/depth_inference
  uv sync

  if [ -d "thirdparty/onnxruntime-linux-x64-gpu-${ONNX_VERSION}" ]; then
    echo "ONNX Runtime already installed."
  else
    wget -O "thirdparty/onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz" \
      "https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz"
    tar -xvf "thirdparty/onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz" -C thirdparty/
    rm "thirdparty/onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz"
  fi

  if [ -d assets/mast3r_onnx_model ]; then
    echo "MAST3R assets already installed."
  else
    gh release download v0.1.0 -p "mast3r_onnx_model.tar.gz.part-*" -D assets/
    cat assets/mast3r_onnx_model.tar.gz.part-* > assets/mast3r_onnx_model.tar.gz
    tar -xvf assets/mast3r_onnx_model.tar.gz -C assets/
    rm -f assets/mast3r_onnx_model.tar.gz*
  fi

  if [ -d assets/dust3r_onnx_model ]; then
    echo "DUSt3R assets already installed."
  else
    gh release download v0.1.0 -p "dust3r_onnx_model.tar.gz.part-*" -D assets/
    cat assets/dust3r_onnx_model.tar.gz.part-* > assets/dust3r_onnx_model.tar.gz
    tar -xvf assets/dust3r_onnx_model.tar.gz -C assets/
    rm -f assets/dust3r_onnx_model.tar.gz
  fi

  if [ -d assets/depth_anything_v2_onnx_model ]; then
    echo "Depth Anything V2 assets already installed."
  else
    gh release download v0.1.0 -p "depth_anything_v2_onnx_model.tar.gz" -D assets/
    tar -xvf assets/depth_anything_v2_onnx_model.tar.gz -C assets/
    rm -f assets/depth_anything_v2_onnx_model.tar.gz
  fi

  cmake -B build -S . \
        -DCMAKE_BUILD_TYPE=Debug \
        -DBUILD_SHARED=ON \
        -DBUILD_TESTS=OFF \
        -DCMAKE_INSTALL_PREFIX="${PWD}/build/install" \
        -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
        -GNinja
  ninja -C build
  ninja -C build install
  cd ../..

Prepare and build the visual place localization module. This module reuses the
top-level FAISS installation through ``Faiss_DIR``:

.. code-block:: bash

  cd thirdparty/visual_place_localization
  uv sync

  archive="thirdparty/onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz"
  install_dir="thirdparty/onnxruntime-linux-x64-gpu-${ONNX_VERSION}"

  cleanup_archive() {
    rm -f "${archive}"
  }
  trap cleanup_archive EXIT

  if [ -f "${install_dir}/include/onnxruntime_cxx_api.h" ] && \
     [ -f "${install_dir}/lib/libonnxruntime.so" ]; then
    echo "ONNX Runtime already installed."
  else
    if [ -d "${install_dir}" ]; then
      echo "Removing incomplete ONNX Runtime install at ${install_dir}."
      rm -rf "${install_dir}"
    fi

    mkdir -p thirdparty
    rm -f "${archive}"
    df -h thirdparty

    wget -O "${archive}" \
      "https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz"
    df -h thirdparty

    tar -xzf "${archive}" -C thirdparty/
    df -h thirdparty
  fi

  if [ -d assets/visual_place_recognition/megaloc_onnx_model ]; then
    echo "MegaLoc assets already installed."
  else
    gh release download v0.1.0 -p "megaloc_onnx_model.tar.gz" \
      --dir assets/visual_place_recognition/
    tar -xvf assets/visual_place_recognition/megaloc_onnx_model.tar.gz \
      -C assets/visual_place_recognition/
    rm -f assets/visual_place_recognition/megaloc_onnx_model.tar.gz
  fi

  cmake -B build -S . \
        -DCMAKE_BUILD_TYPE=Release \
        -DUSE_GPU_DATABASE=ON \
        -DBUILD_TESTS=OFF \
        -DBUILD_SHARED=ON \
        -DFaiss_DIR="${PWD}/../faiss/build/install/share/faiss" \
        -DCMAKE_INSTALL_PREFIX="${PWD}/build/install" \
        -GNinja
  ninja -C build
  ninja -C build install
  cd ../..

-------------------
LibTorch and ALIKED
-------------------

ALIKED is compiled from ``utils/aliked`` during the main SLAM-MER build when
``WITH_ALIKED=ON``. Download LibTorch 2.9.0+cu128 before enabling ALIKED:

.. code-block:: bash

  if [ -d "thirdparty/libtorch" ]; then
    echo "libtorch already present: thirdparty/libtorch"
  else
    archive="libtorch-shared-with-deps-2.9.0+cu128.zip"
    url="https://download.pytorch.org/libtorch/cu128/libtorch-shared-with-deps-2.9.0%2Bcu128.zip"

    cd thirdparty
    wget -O "${archive}" "${url}"
    unzip "${archive}"
    rm -f "${archive}"
    cd ..
  fi

--------------------------------
Compile the Visual-SLAM Pipeline
--------------------------------

After compiling the third-party libraries and optional modules, configure and
install the full SLAM-MER Release build:

.. code-block:: bash

  cmake -B build -S . \
        -DCMAKE_BUILD_TYPE=Release \
        -DDEPTH_INFERENCE:BOOL=ON \
        -DVISUAL_PLACE_LOCALIZATION:BOOL=ON \
        -DCMAKE_INSTALL_PREFIX=./build/install \
        -GNinja
  ninja -C build install

For a Debug build with verbose logging enabled:

.. code-block:: bash

  cmake -B build -S . \
        -DCMAKE_BUILD_TYPE=Debug \
        -DLOG_TERMINAL:BOOL=OFF \
        -DLOG_INTERFACE:BOOL=ON \
        -DLOG_CUDA:BOOL=ON \
        -DLOG_TIME:BOOL=ON \
        -DLOG_LOCALIZATION:BOOL=ON \
        -DLOG_POSE_ESTIMATION:BOOL=ON \
        -DLOG_SETTINGS:BOOL=ON \
        -DLOG_SLAM:BOOL=ON \
        -DLOG_ADJUSTMENT:BOOL=ON \
        -DLOG_REPRESENTATION:BOOL=ON \
        -DDEPTH_INFERENCE:BOOL=ON \
        -DVISUAL_PLACE_LOCALIZATION:BOOL=ON \
        -DCMAKE_INSTALL_PREFIX=./build/install \
        -GNinja
  ninja -C build install

To build only the core libraries without depth inference or visual place
localization:

.. code-block:: bash

  cmake -B build -S . \
        -DCMAKE_BUILD_TYPE="${SLAM_MER_BUILD_TYPE:-Release}" \
        -DDEPTH_INFERENCE:BOOL=OFF \
        -DVISUAL_PLACE_LOCALIZATION:BOOL=OFF \
        -DWITH_ALIKED:BOOL="${SLAM_MER_WITH_ALIKED:-ON}" \
        -DCMAKE_INSTALL_PREFIX=./build/install \
        -GNinja
  ninja -C build install

Set ``SLAM_MER_WITH_ALIKED=OFF`` for the reduced build when
``thirdparty/libtorch`` is unavailable. For the complete CMake option list, see
the top-level ``COMPILATION_OPTIONS.md``.

To control which transformation templates are compiled, use
``-DSLAM_TRANSFORMATION_INSTANTIATIONS=ALL|POSE3|SIMILARITY3``
(default: ``ALL``).

If the Rerun viewer has GUI backend issues, for example in WSL2, start the web
viewer in advance:

.. code-block:: bash

  rerun --web-viewer

Or run the matching Rerun Python package directly:

.. code-block:: bash

  uvx --from rerun-sdk==X.XX.X rerun --web-viewer

Runnable examples, dataset download instructions, and experiment-specific
executables are maintained outside this library repository:

.. code-block:: text

  https://github.com/slam-mer/slam-mer_examples

This repository currently provides the SLAM-MER libraries and documentation. It
does not contain the old dataset download scripts that were previously
documented here.

----------
Evaluation
----------

When an example writes a TUM-format trajectory, it can be evaluated with
`evo <https://github.com/MichaelGrupp/evo>`_. Install evo with:

.. code-block:: bash

  pip install evo

For absolute pose errors, use the command by replacing the filenames:

.. code-block:: bash

  evo_ape tum <groundtruth_file> <output_file> -as

-------------
Documentation
-------------

Install documentation system packages:

.. code-block:: bash

  sudo apt update
  sudo apt install -y \
    python3-sphinx \
    python3-sphinx-rtd-theme \
    doxygen \
    python3-exhale \
    python3-breathe

Build the full Sphinx and Doxygen documentation:

.. code-block:: bash

  uv sync --extra docs
  rm -rf doc/_build doc/api doc/doxygen/_docs
  cd doc/doxygen
  doxygen Doxyfile.cfg
  cd ..
  make html
  cd ..

For quick prose-only rebuilds that skip Doxygen regeneration, use:

.. code-block:: bash

  cd doc
  SLAM_MER_DOCS_FAST=1 make html
  cd ..

The generated HTML is written to ``doc/_build/html/index.html``.

Register pre-commit and pre-push hooks from the repository root:

.. code-block:: bash

  uv sync --extra dev
  uv run pre-commit install --install-hooks --hook-type pre-commit --hook-type pre-push

After this step, pre-commit runs on every ``git commit`` and pre-push checks
run on every ``git push``.

------------------
Debugging the Code
------------------

To enable `Address Sanitizer (ASan) <https://github.com/google/sanitizers/wiki/addresssanitizer>`_,
build the project with ``-DUSE_ASAN=ON``. Libraries that use prebuilt LibTorch
binaries, such as ALIKED, must not be built with ASan due to GCC
incompatibilities.

Build the project with ASan enabled using the following commands:

.. code-block:: bash

  cmake -B build -S . \
        -DCMAKE_BUILD_TYPE=Debug \
        -DWITH_ALIKED=OFF \
        -DUSE_ASAN=ON \
        -DDEPTH_INFERENCE=OFF \
        -DVISUAL_PLACE_LOCALIZATION=OFF \
        -DCMAKE_INSTALL_PREFIX=./build/install \
        -GNinja
  ninja -C build install

To use `Compute Sanitizer <https://docs.nvidia.com/compute-sanitizer/ComputeSanitizer/index.html>`_
for CUDA memory error detection, build with ``-DUSE_ASAN=OFF``, because ASan and
Compute Sanitizer cannot be used together.

Build with Compute Sanitizer support:

.. code-block:: bash

  cmake -B build -S . \
        -DCMAKE_BUILD_TYPE=Debug \
        -DWITH_ALIKED=ON \
        -DUSE_ASAN=OFF \
        -DCMAKE_INSTALL_PREFIX=./build/install \
        -GNinja
  ninja -C build install

Then run the executable from the examples repository with Compute Sanitizer:

.. code-block:: bash

  compute-sanitizer <example-executable>

In Release builds, most ``LOG_*`` CMake options are forced off by
``cmake/Options.cmake``. Use a Debug build when verbose runtime logging is
needed.
