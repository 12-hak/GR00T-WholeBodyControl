#!/usr/bin/env bash
set -euo pipefail
CUDA_NVCC="/mnt/d/python/GR00T-WholeBodyControl/.venv_sim/lib/python3.10/site-packages/nvidia/cuda_nvcc/include"
cp -r "${CUDA_NVCC}/crt" "${HOME}/deps/cuda/include/"
if [ -d "${CUDA_NVCC}/cooperative_groups" ]; then
  cp -r "${CUDA_NVCC}/cooperative_groups" "${HOME}/deps/cuda/include/"
fi
test -f "${HOME}/deps/cuda/include/crt/host_defines.h"

if [ -d "${HOME}/deps/cuda/lib" ] && [ ! -d "${HOME}/deps/cuda/lib64" ]; then
  ln -sfn lib "${HOME}/deps/cuda/lib64"
fi

if [ ! -f "${HOME}/deps/cppzmq/zmq.hpp" ]; then
  cd "${HOME}/deps"
  curl -sL https://github.com/zeromq/cppzmq/archive/refs/tags/v4.10.0.tar.gz -o cppzmq.tgz
  tar xzf cppzmq.tgz
  mv cppzmq-4.10.0 cppzmq
fi
