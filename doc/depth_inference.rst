.. Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
.. SPDX-License-Identifier: AGPL-3.0-or-later

Depth Inference
===============

This page summarizes the depth inference module used by SLAM-MER.

--------------------
Repository location
--------------------

- ``thirdparty/depth_inference``

-----------
Description
-----------

The depth inference repository provides a standalone C++ library for monocular
depth estimation using ONNX Runtime. The active backend in the current module
is MAST3R; other model names exist in shared configuration enums but are not
active integration paths in the module.

Main outputs include:

- Point maps
- Point-map confidence maps

----------------------
Usage in the framework
----------------------

In SLAM-MER, this module is used as an optional source of keyframe-local depth
priors. For monocular input, the current SLAM-MER integration creates a MAST3R
depth estimator and stores point-map and confidence-map outputs used during
keyframe creation.

-----------------------
Build and integration
-----------------------

To build it as part of the SLAM-MER workflow:

.. code-block:: bash

  mise run configure_modules
  mise run build_modules

This configures and builds ``thirdparty/depth_inference`` together with the
other optional modules.

For standalone installation and module-specific options, see the repository
README in ``thirdparty/depth_inference``.
