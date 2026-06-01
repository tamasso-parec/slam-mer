.. Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
.. SPDX-License-Identifier: AGPL-3.0-or-later

SLAM-MER: :u:`M`\itsubishi :u:`E`\lectric :u:`R`\esearch framework for visual :u:`SLAM`
========================================================================================================


About
-----

This documentation is generated for SLAM-MER, a Visual Simultaneous
Localization and Mapping (Visual-SLAM) framework developed at Mitsubishi
Electric Research Laboratories (MERL).

Many legacy SLAM codebases are difficult to maintain, debug, and extend for
new research directions. With this in mind, we implemented SLAM-MER as a
modern, low-latency, modular Visual-SLAM pipeline that explores recent
advances in deep learning and optimization while keeping strong geometric
foundations.
In particular, we integrate vision and depth priors into the framework to
improve the robustness, accuracy, and efficiency of the SLAM system.
The pipeline is implemented in C++ and is designed to run in indoor and outdoor
environments. The current code uses Eigen, CudaSift, GTSAM, PoseLib, FAISS,
Rerun, libpng, libjpeg-turbo, optional LibTorch/ALIKED support, and optional
ONNX Runtime-based modules.

The project is under active development, and we continue to add features and
improve the documentation to make integration and extension easier.


Repositories
------------

The source tree is split into the main framework and optional module
repositories:

- :u:`Main framework`: ``slam-mer``.
  This repository contains the core C++ Visual-SLAM pipeline, integration
  code, build configuration, and main documentation.
- :u:`Depth module`: ``thirdparty/depth_inference``.
  This repository provides the standalone depth estimation module (ONNX Runtime)
  used as an optional depth prior in SLAM-MER.
- :u:`Visual place localization module`: ``thirdparty/visual_place_localization``.
  This repository provides the standalone visual place recognition/localization
  module used by SLAM-MER for retrieval and relocalization workflows.
- :u:`Examples and experiments repository`:
  `slam-mer_examples <https://github.com/slam-mer/slam-mer_examples>`_.
  This repository contains runnable examples and experiment-style scripts that
  demonstrate how to use and evaluate the framework.

To install and use the framework, see :doc:`install` and :doc:`examples`. For
ROS integration notes, see :doc:`ros_wrapper`.


.. toctree::
  :maxdepth: 1

  install
  detailed_instalation
  examples
  depth_inference
  visual_place_recognition
  ros_wrapper
  code_structure
  api/library_root
  authors


.. _horizontal-line:


.. figure:: figures/merl_logo.jpg
  :alt: MERL logo
  :figclass: left
  :width: 40%


This project is supported by the Mitsubishi Electric Research Laboratories
(MERL), 201 Broadway, Cambridge, Massachusetts.
