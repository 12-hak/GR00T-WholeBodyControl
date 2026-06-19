#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CUDA_ROOT="${HOME}/deps/cuda"

MSGPACK_ROOT="${HOME}/deps/msgpack-cxx-6.1.0"

export PATH="${HOME}/.local/bin:/usr/bin:/bin"
export TensorRT_ROOT="${HOME}/deps/TensorRT-10.13.3.9"
export onnxruntime_ROOT="${HOME}/deps/onnxruntime"
export CUDAToolkit_ROOT="${CUDA_ROOT}"
export CUDA_HOME="${CUDA_ROOT}"
export LD_LIBRARY_PATH="${TensorRT_ROOT}/lib:${onnxruntime_ROOT}/lib:${CUDA_ROOT}/lib:${HOME}/deps/zeromq/lib:/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="${HOME}/deps/zeromq/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export CMAKE_PREFIX_PATH="${HOME}/deps/zeromq:${HOME}/deps/gtest:${CMAKE_PREFIX_PATH:-}"

test -f "${CUDA_ROOT}/include/cuda_runtime.h"
test -f "${TensorRT_ROOT}/include/NvInfer.h"
test -f "${MSGPACK_ROOT}/include/msgpack.hpp"

bash "${REPO_ROOT}/install_scripts/fix_cuda_headers_wsl.sh"

cd "${REPO_ROOT}/gear_sonic_deploy"
rm -rf build
mkdir -p build
cd build
cmake -S .. -B . \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DMSGPACK_INCLUDE_DIR="${MSGPACK_ROOT}/include" \
  -DZMQ_INCLUDE_DIR="${HOME}/deps/zeromq/include" \
  -DZMQ_LIBRARY="${HOME}/deps/zeromq/lib/libzmq.so" \
  -DCMAKE_CXX_FLAGS="-I${HOME}/deps/cppzmq -I${HOME}/deps/json/include -I${MSGPACK_ROOT}/include"
cmake --build . -j"$(nproc)"
