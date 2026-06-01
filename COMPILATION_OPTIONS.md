<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# SLAM-MER Compilation Options

This document lists the CMake cache options used by the SLAM-MER library.

## Main CMake Options

| Option | Type | Default | Description |
| --- | --- | --- | --- |
| `WITH_CUDA` | `BOOL` | `ON` | Build with CUDA support. |
| `SLAM_TRANSFORMATION_INSTANTIATIONS` | `STRING` | `ALL` | Select template instantiations: `ALL`, `POSE3`, or `SIMILARITY3`. |
| `HIDE_WARNINGS` | `BOOL` | `ON` | Ignore deprecated warnings (`-Wno-deprecated`). |
| `HIDE_CUDA_NVCC_WARNINGS` | `BOOL` | `ON` | Hide noisy NVCC warnings for CUDA targets. |
| `CUDA_VERSION` | `STRING` | `12.8` (or `$ENV{CUDA_VERSION}`) | CUDA version used by the build. |
| `CUDA_ARCH` | `STRING` | `86` (or `$ENV{CUDA_ARCH}`) | CUDA compute architecture used for GPU builds. |
| `WITH_ALIKED` | `BOOL` | `ON` | Enable ALIKED and LightGlue integration (requires CUDA + LibTorch). |
| `VISUAL_PLACE_LOCALIZATION` | `BOOL` | `OFF` | Enable Visual Place Localization module integration. |
| `DEPTH_INFERENCE` | `BOOL` | `OFF` | Enable Depth Inference module integration. |
| `LOG_FILE` | `STRING` | `"slam"` | Base log file name. |
| `LOG_TERMINAL` | `BOOL` | `ON` | Print logs to terminal. |
| `LOG_INTERFACE` | `BOOL` | `ON` | Enable interface-level logs. |
| `LOG_CUDA` | `BOOL` | `OFF` | Enable CUDA-SIFT logs. |
| `LOG_TIME` | `BOOL` | `OFF` | Enable timing logs. |
| `LOG_LOCALIZATION` | `BOOL` | `OFF` | Enable extra localization logs. |
| `LOG_POSE_ESTIMATION` | `BOOL` | `OFF` | Enable extra pose estimation logs. |
| `LOG_SETTINGS` | `BOOL` | `OFF` | Enable settings-load logs. |
| `LOG_SLAM` | `BOOL` | `OFF` | Enable SLAM module logs. |
| `LOG_ADJUSTMENT` | `BOOL` | `ON` | Enable covisibility/loop-closure adjustment logs. |
| `LOG_REPRESENTATION` | `BOOL` | `OFF` | Enable representation module logs. |
| `USE_ASAN` | `BOOL` | `OFF` | Enable AddressSanitizer (requires `WITH_ALIKED=OFF`). |
| `SHOW_DENSE` | `BOOL` | `OFF` | Show dense map points in visualization. |

## Notes and Constraints

- `SLAM_TRANSFORMATION_INSTANTIATIONS` controls which template instantiations are compiled (`ALL`, `POSE3`, `SIMILARITY3`).
- If `WITH_ALIKED=ON` and `WITH_CUDA=OFF`, CUDA is automatically enabled.
- If `WITH_ALIKED=ON`, `thirdparty/libtorch` must exist or configuration fails.
- If `WITH_ALIKED=ON`, `TORCH_CUDA_ARCH_LIST` is derived from `CUDA_ARCH` (for example, `86` -> `8.6`).
- `USE_ASAN=ON` is not compatible with `WITH_ALIKED=ON`.
- In Release builds, `LOG_TERMINAL`, `LOG_CUDA`, `LOG_TIME`, `LOG_LOCALIZATION`, `LOG_POSE_ESTIMATION`, `LOG_SETTINGS`, `LOG_SLAM`, `LOG_ADJUSTMENT`, and `LOG_REPRESENTATION` are forced `OFF`; `LOG_INTERFACE` is not forced off.
- In Release builds, `SHOW_DENSE` is forced `ON`.

## Default `mise run build` Profile

The default `mise run build` task configures CMake with:

```bash
-DCMAKE_BUILD_TYPE=Release
-DDEPTH_INFERENCE=ON
-DVISUAL_PLACE_LOCALIZATION=ON
-DCMAKE_INSTALL_PREFIX=./build/install
```

## Reduced `mise run build_without_depth_vpl` Profile

The reduced build task skips the depth inference and visual place localization
integrations:

```bash
-DCMAKE_BUILD_TYPE=${SLAM_MER_BUILD_TYPE:-Release}
-DDEPTH_INFERENCE=OFF
-DVISUAL_PLACE_LOCALIZATION=OFF
-DWITH_ALIKED=${SLAM_MER_WITH_ALIKED:-ON}
-DCMAKE_INSTALL_PREFIX=./build/install
```

Set `SLAM_MER_WITH_ALIKED=OFF` when building on machines that do not have
`thirdparty/libtorch`.

## Example Custom Configuration

```bash
cmake -B build -S . \
  -DCMAKE_BUILD_TYPE=Release \
  -DWITH_ALIKED=OFF \
  -DUSE_ASAN=ON \
  -DDEPTH_INFERENCE=OFF \
  -DVISUAL_PLACE_LOCALIZATION=OFF \
  -DCMAKE_INSTALL_PREFIX=./build/install \
  -GNinja
```
