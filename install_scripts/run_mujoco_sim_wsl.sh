#!/usr/bin/env bash
# Run MuJoCo sim2sim on WSL (Ubuntu 22.04).
# Terminal 1: bash install_scripts/run_mujoco_sim_wsl.sh sim
# Terminal 2: bash install_scripts/run_mujoco_sim_wsl.sh deploy
# Terminal 2 (Quest teleop): bash install_scripts/run_mujoco_sim_wsl.sh deploy-zmq

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-sim}"

export PATH="${HOME}/.local/bin:/usr/bin:/bin"
export TensorRT_ROOT="${HOME}/deps/TensorRT-10.13.3.9"
export onnxruntime_ROOT="${HOME}/deps/onnxruntime"
export CUDA_HOME="${HOME}/deps/cuda"
export LD_LIBRARY_PATH="${TensorRT_ROOT}/lib:${onnxruntime_ROOT}/lib:${CUDA_HOME}/lib:${HOME}/deps/zeromq/lib:/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"

case "$MODE" in
  sim)
    bash "${REPO_ROOT}/install_scripts/fix_wslg_display.sh" || true
    cd "${REPO_ROOT}"
    source .venv_sim/bin/activate
    exec python gear_sonic/scripts/run_sim_loop.py
    ;;
  deploy|deploy-zmq)
    bash "${REPO_ROOT}/install_scripts/fix_unitree_libs_wsl.sh"
    cd "${REPO_ROOT}/gear_sonic_deploy"
    sed -i 's/\r$//' deploy.sh scripts/setup_env.sh 2>/dev/null || true
    export MSGPACK_ROOT="${HOME}/deps/msgpack-cxx-6.1.0"
    export CMAKE_PREFIX_PATH="${HOME}/deps/gtest:${HOME}/deps/zeromq:${CMAKE_PREFIX_PATH:-}"
    if [[ "$MODE" == "deploy-zmq" ]]; then
      DEPLOY_ARGS=(sim --input-type zmq_manager)
    else
      DEPLOY_ARGS=(sim --input-type keyboard)
    fi
    if [[ "${NO_PLANNER:-}" == "1" ]]; then
      DEPLOY_ARGS+=(--no-planner)
    else
      PLANNER_REPO="${REPO_ROOT}/gear_sonic_deploy/planner/target_vel/V2"
      PLANNER_ONNX="${PLANNER_REPO}/planner_sonic.onnx"
      PLANNER_TRT="${PLANNER_REPO}/planner_planner_sonic.trt"
      PLANNER_CACHE="${HOME}/gear_sonic_cache/planner/V2"
      mkdir -p "${PLANNER_CACHE}"

      if [[ -f "${PLANNER_TRT}" ]]; then
        # Reuse TRT already built next to the repo ONNX (fast startup)
        DEPLOY_ARGS+=(--planner "${PLANNER_ONNX}")
        echo "Using cached planner TRT: ${PLANNER_TRT}"
      else
        # First-time build: ONNX on WSL ext4 (much faster than /mnt/d)
        if [[ ! -f "${PLANNER_CACHE}/planner_sonic.onnx" ]]; then
          echo "Copying planner ONNX to WSL disk for first TRT build..."
          cp "${PLANNER_ONNX}" "${PLANNER_CACHE}/planner_sonic.onnx"
        fi
        DEPLOY_ARGS+=(--planner "${PLANNER_CACHE}/planner_sonic.onnx")
      fi
    fi
    exec bash deploy.sh "${DEPLOY_ARGS[@]}"
    ;;
  *)
    echo "Usage: $0 [sim|deploy|deploy-zmq]"
    exit 1
    ;;
esac
