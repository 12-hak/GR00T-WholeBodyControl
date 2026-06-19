#!/usr/bin/env bash
# Minimal teleop venv for Quest 3 (no Pico / XRoboToolkit SDK).
# Usage: bash install_scripts/install_quest3_teleop_wsl.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="${GEAR_SONIC_TELEOP_VENV:-${HOME}/.venv_gear_sonic_teleop}"
cd "$REPO_ROOT"

export PATH="${HOME}/.local/bin:/usr/bin:/bin:${PATH:-}"

if ! command -v uv &>/dev/null; then
  echo "[INFO] Installing uv..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
  # shellcheck disable=SC1091
  [[ -f "$HOME/.local/bin/env" ]] && source "$HOME/.local/bin/env"
  export PATH="${HOME}/.local/bin:${PATH}"
fi

echo "[INFO] Installing Python 3.10 via uv..."
uv python install 3.10
MANAGED_PY="$(uv python find --no-project 3.10)"

rm -rf "${VENV_DIR}"
uv venv "${VENV_DIR}" --python "$MANAGED_PY" --prompt gear_sonic_teleop
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

echo "[INFO] Installing gear_sonic[teleop] (pyzmq, pin, websockets)..."
uv pip install -e "gear_sonic[teleop]"

echo ""
echo "Done. Venv: ${VENV_DIR}"
echo "Run: bash install_scripts/run_quest3_teleop_wsl.sh"
