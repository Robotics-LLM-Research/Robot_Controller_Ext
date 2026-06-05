#!/usr/bin/env bash

export VK_ICF_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

GPU_ID="${1:-0}"

cd ~/isaacsim || exit 1

./isaac-sim.sh \
    --/renderer/multiGpu/enabled=false \
    --/renderer/activeGpu="${GPU_ID}"