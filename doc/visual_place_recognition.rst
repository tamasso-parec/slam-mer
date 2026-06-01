.. Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
.. SPDX-License-Identifier: AGPL-3.0-or-later

Visual Place Recognition
========================

This page summarizes the visual place recognition/localization module.
It is a standalone C++ module that provides visual place recognition (VPR)
capabilities, i.e., matching a query image against a database of previously
seen places and retrieving the most similar image(s). SLAM-MER uses those
retrieved candidates for loop validation and relocalization pose estimation.

--------------------
Repository location
--------------------

- ``thirdparty/visual_place_localization``

-----------
Description
-----------

For SLAM, this repository provides a common visual-place-recognition interface
for image retrieval, along with implementations of multiple VPR backends.
Current backends include:

- Bag-of-Words (BoW)-based retrieval with OpenCV descriptors and FAISS search
- MegaLoc global descriptor retrieval with ONNX Runtime and FAISS

----------------------
Usage in the framework
----------------------

In SLAM-MER, this module supports loop-candidate retrieval and relocalization
workflows by matching current observations against a database of previously
seen places. Candidate validation and camera-pose estimation are handled by the
SLAM-MER geometric pose-estimation code.

---------------------
Build and integration
---------------------

This module is optional but strongly recommended for visual SLAM applications,
as it provides robust place recognition capabilities that improve loop-closure
detection and relocalization performance. To build it as part of the
SLAM-MER workflow with the required options enabled, run the following
commands from the root of the SLAM-MER project:

.. code-block:: bash

  mise run configure_modules
  mise run build_modules

This configures and builds ``thirdparty/visual_place_localization`` together
with the other optional modules.

For standalone installation details and backend-specific options, see the
repository README in ``thirdparty/visual_place_localization``.
