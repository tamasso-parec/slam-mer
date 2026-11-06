#!/usr/bin/env bash
# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
# SPDX-License-Identifier: AGPL-3.0-or-later

# Host setup from NVIDIA's Container Toolkit installation guide:
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
# Run after builds finish: restarting Docker interrupts running builds.
set -euo pipefail

if [[ "$(id -u)" != 0 ]]; then
  echo "Run with sudo: sudo bash docker/setup_nvidia_runtime.sh" >&2
  exit 1
fi

apt-get update
apt-get install -y --no-install-recommends ca-certificates curl gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  > /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt-get update
apt-get install -y nvidia-container-toolkit
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker
