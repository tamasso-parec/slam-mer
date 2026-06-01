<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# License Exceptions and Third-Party Notices

Files `utils/cudaSift/*` were adapted from [GitHub: Celebrandil/CudaSift](https://github.com/Celebrandil/CudaSift):
```
Copyright (c) 2017 Marten Bjorkman

MIT License
```

Third-party libraries under `thirdparty/*`:

### Light_Glue_CPP
GitHub: [MrNeRF/Light_Glue_CPP](https://github.com/MrNeRF/Light_Glue_CPP)
Description: C++ implementation of LightGlue and ALIKED using LibTorch and CUDA. It provides local feature extraction and matching for robust geometric correspondence generation.
Folder: `thirdparty/Light_Glue_CPP`
Copyright and license:
```
Copyright 2023 ETH Zurich

Apache-2.0
```

### PoseLib
GitHub: [PoseLib/PoseLib](https://github.com/PoseLib/PoseLib)
Description: Collection of minimal solvers and robust estimators for camera geometry. It is used to estimate absolute and relative pose from 2D-3D and 2D-2D correspondences.
Folder: `thirdparty/PoseLib`
Copyright and license:
```
Copyright (c) 2020, Viktor Larsson

BSD-3-Clause
```

### faiss
GitHub: [facebookresearch/faiss](https://github.com/facebookresearch/faiss)
Description: Library for fast similarity search and clustering of dense vectors. It supports CPU and GPU backends for large-scale nearest-neighbor retrieval.
Folder: `thirdparty/faiss`
Copyright and license:
```
Copyright (c) Facebook, Inc. and its affiliates.

MIT License
```

### gtsam
GitHub: [borglab/gtsam](https://github.com/borglab/gtsam)
Description: Factor-graph optimization library for SLAM, robotics, and structure-from-motion. It provides nonlinear solvers for smoothing, mapping, and probabilistic inference.
Folder: `thirdparty/gtsam`
Copyright and license:
```
Copyright (c) 2010, Georgia Tech Research Corporation

BSD-3-Clause
```

### libtorch
GitHub: [pytorch/pytorch](https://github.com/pytorch/pytorch)
Description: C++ distribution of the PyTorch runtime and tensor library. It enables neural network inference and tensor operations directly from C++ modules.
Folder: `thirdparty/libtorch`
Copyright and license:
```
Copyright (c) PyTorch and Caffe2 contributors (multiple holders; see upstream LICENSE)

BSD-3-Clause
```

### rerun_cpp_sdk
GitHub: [rerun-io/rerun](https://github.com/rerun-io/rerun)
Description: C++ SDK for logging and visualizing time-series and spatial data in Rerun. It is used to inspect trajectories, landmarks, and debugging signals during SLAM execution.
Folder: `thirdparty/rerun_cpp_sdk`
Copyright and license:
```
Copyright (c) 2022 Rerun Technologies AB

MIT OR Apache-2.0
```

### depth_inference
GitHub: [slam-mer/depth_inference](https://github.com/slam-mer/depth_inference)
Description: C++ ONNX Runtime module for monocular depth estimation. It outputs depth, confidence, and point-map data used as depth priors in the SLAM pipeline.
Folder: `thirdparty/depth_inference`
Copyright and license:
```
Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL).

SPDX-License-Identifier: AGPL-3.0-or-later
```

### visual_place_localization
GitHub: [slam-mer/visual_place_localization](https://github.com/slam-mer/visual_place_localization)
Description: C++ visual place recognition module with BoW and MegaLoc backends. It supports database construction and nearest-neighbor retrieval for localization and loop candidate search.
Folder: `thirdparty/visual_place_localization`
Copyright and license:
```
Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL).

SPDX-License-Identifier: AGPL-3.0-or-later
```
