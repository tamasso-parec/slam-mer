#!/usr/bin/env bash
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

set -euo pipefail

# Configure directly: the host mise workflow also synchronizes Git remotes and
# downloads assets with authenticated gh. Neither is needed in an image build.
cmake -S thirdparty/depth_inference -B thirdparty/depth_inference/build -GNinja \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED=ON -DBUILD_TESTS=OFF \
  -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
  -DCMAKE_INSTALL_PREFIX="${PWD}/thirdparty/depth_inference/build/install"
cmake --build thirdparty/depth_inference/build --target install

cmake -S thirdparty/visual_place_localization \
  -B thirdparty/visual_place_localization/build -GNinja \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED=ON -DBUILD_TESTS=OFF \
  -DUSE_GPU_DATABASE=ON \
  -DFaiss_DIR="${PWD}/thirdparty/faiss/build/install/share/faiss" \
  -DCMAKE_INSTALL_PREFIX="${PWD}/thirdparty/visual_place_localization/build/install"
cmake --build thirdparty/visual_place_localization/build --target install
