# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04 AS devel

ARG MISE_VERSION=2026.11.1
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git gh sudo build-essential gcc-14 g++-14 ninja-build \
    libssl-dev libboost-all-dev libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libopenblas-dev liblapack-dev libyaml-cpp-dev \
    libeigen3-dev libtbb-dev libpng-dev libjpeg-turbo8-dev wget unzip \
    && rm -rf /var/lib/apt/lists/*
RUN curl -fsSL "https://github.com/jdx/mise/releases/download/v${MISE_VERSION}/mise-v${MISE_VERSION}-linux-x64.tar.gz" \
    | tar -xz --strip-components=2 -C /usr/local/bin mise/bin/mise

# Install the same tool versions as the README before copying source files.
WORKDIR /opt/slam-mer-tools
COPY mise.toml ./
COPY .mise/ .mise/
RUN mise trust && mise install

ENV PATH="/root/.local/share/mise/shims:/usr/local/cuda-12.8/bin:${PATH}" \
    CC=/usr/bin/gcc-14 \
    CXX=/usr/bin/g++-14 \
    CUDAHOSTCXX=/usr/bin/g++-14 \
    CMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-14 \
    CUDA_HOME=/usr/local/cuda-12.8 \
    CUDA_VERSION=12.8

# Existing mise tasks invoke Ninja directly. Bound their memory usage as well.
COPY docker/ninja /usr/local/bin/ninja
RUN chmod +x /usr/local/bin/ninja
ENV BUILD_JOBS=2
WORKDIR /opt/slam-mer
CMD ["bash"]

FROM devel AS models
WORKDIR /opt/slam-mer-tools
COPY docker/download_models.py /tmp/download_models.py
RUN mise exec -- python /tmp/download_models.py

FROM devel AS libtorch
WORKDIR /opt/slam-mer-tools
RUN mkdir -p thirdparty && mise run ensure_libtorch

FROM devel AS dependencies
# RTX 40-series uses 89; use 86 for RTX 30-series, for example.
ARG CUDA_ARCH=89
ARG BUILD_JOBS=2
ENV CUDA_ARCH=${CUDA_ARCH} BUILD_JOBS=${BUILD_JOBS}
COPY mise.toml ./
COPY .mise/ .mise/
COPY thirdparty/gtsam/ thirdparty/gtsam/
COPY thirdparty/gtsam_similarity3.patch thirdparty/
COPY thirdparty/PoseLib/ thirdparty/PoseLib/
COPY thirdparty/faiss/ thirdparty/faiss/
# mise's checked-in [env] would otherwise override the Docker build argument.
RUN printf '[env]\nCUDA_ARCH = "%s"\n' "${CUDA_ARCH}" > mise.local.toml \
    && test -f thirdparty/gtsam/CMakeLists.txt \
    && test -f thirdparty/PoseLib/CMakeLists.txt \
    && test -f thirdparty/faiss/CMakeLists.txt \
    && mise trust && mise trust mise.local.toml \
    && mise run build_thirdparties

FROM dependencies AS core
ENV SLAM_MER_WITH_ALIKED=OFF
COPY . .
RUN mise run build_without_depth_vpl

FROM dependencies AS extensions
RUN apt-get update && apt-get install -y --no-install-recommends libopencv-dev \
    && rm -rf /var/lib/apt/lists/*
COPY thirdparty/depth_inference/ thirdparty/depth_inference/
COPY thirdparty/visual_place_localization/ thirdparty/visual_place_localization/
RUN mise -C thirdparty/depth_inference trust \
    && mise -C thirdparty/depth_inference run configure_onnx \
    && mise -C thirdparty/visual_place_localization trust \
    && mise -C thirdparty/visual_place_localization run configure_onnx
COPY --from=models /opt/slam-mer-tools/thirdparty/depth_inference/assets/ \
    thirdparty/depth_inference/assets/
COPY --from=models /opt/slam-mer-tools/thirdparty/visual_place_localization/assets/ \
    thirdparty/visual_place_localization/assets/
COPY docker/build_modules.sh docker/build_modules.sh
RUN mise exec -- bash docker/build_modules.sh

FROM extensions AS full
COPY . .
RUN printf '[env]\nCUDA_ARCH = "%s"\nTORCH_CUDA_ARCH_LIST = "%s.%s"\n' \
    "${CUDA_ARCH}" "$((CUDA_ARCH / 10))" "$((CUDA_ARCH % 10))" > mise.local.toml \
    && mise trust mise.local.toml
COPY --from=libtorch /opt/slam-mer-tools/thirdparty/libtorch/ thirdparty/libtorch/
RUN mise run build && mise exec -- python docker/smoke_test.py --skip-gpu

FROM full AS datasets
ARG EXAMPLES_REVISION=02da2e8899968e877bed2f36a46beb2fff972b64
RUN git init /opt/slam-mer_examples \
    && git -C /opt/slam-mer_examples remote add origin https://github.com/slam-mer/slam-mer_examples.git \
    && git -C /opt/slam-mer_examples fetch --depth 1 origin "${EXAMPLES_REVISION}" \
    && git -C /opt/slam-mer_examples checkout --detach FETCH_HEAD
COPY examples/euroc/ /opt/slam-mer/examples/euroc/
RUN git -C /opt/slam-mer_examples apply \
      /opt/slam-mer/examples/euroc/stop_visualization_before_export.patch \
    && mise exec -- cmake -S examples/euroc -B /opt/slam-mer_examples/build -GNinja \
      -DCMAKE_BUILD_TYPE=Release \
    && mise exec -- cmake --build /opt/slam-mer_examples/build -j "${BUILD_JOBS}" \
    && mise exec -- ctest --test-dir /opt/slam-mer_examples/build --output-on-failure \
    && chmod a+w /opt/slam-mer_examples
WORKDIR /opt/slam-mer_examples
