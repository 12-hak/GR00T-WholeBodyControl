#!/usr/bin/env bash
# Quest 3 arm teleop for GR00T sim2sim (WSL).
# Terminal 1: install_scripts/run_mujoco_sim_wsl.sh sim
# Terminal 2: install_scripts/run_mujoco_sim_wsl.sh deploy-zmq
# Terminal 3: install_scripts/run_quest3_teleop_wsl.sh

set -euo pipefail

echo "[Quest teleop] starting..."

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

VENV_DIR="${GEAR_SONIC_TELEOP_VENV:-${HOME}/.venv_gear_sonic_teleop}"

if [[ ! -f "${VENV_DIR}/bin/activate" ]]; then
  echo "Creating teleop venv at ${VENV_DIR}..."
  bash "${REPO_ROOT}/install_scripts/install_quest3_teleop_wsl.sh"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
uv pip install -q websockets 2>/dev/null || pip install -q websockets

_extract_ip() {
  echo "$1" | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -1 || true
}

_get_windows_lan_ip() {
  if command -v ipconfig.exe >/dev/null 2>&1; then
    ipconfig.exe 2>/dev/null | tr -d '\r' | grep -i "IPv4" \
      | grep -viE '169\.254|192\.168\.56\.' \
      | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' \
      | grep -vE '^(127\.|169\.254\.|192\.168\.56\.)' \
      | head -1 || true
  fi
}

if [[ -n "${QUEST_HOST_IP:-}" ]]; then
  WIN_IP="${QUEST_HOST_IP}"
else
  WIN_IP="$(_get_windows_lan_ip)"
fi
if [[ -z "${WIN_IP}" ]]; then
  WIN_IP="$(powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "${REPO_ROOT}/install_scripts/get_quest_lan_ip.ps1" 2>/dev/null | tr -d '\r' || true)"
  WIN_IP="$(_extract_ip "$WIN_IP")"
fi
if [[ -z "${WIN_IP}" ]]; then
  WIN_IP="$(powershell.exe -NoLogo -NoProfile -Command "(Get-NetIPAddress -AddressFamily IPv4 | Where-Object { \$_.IPAddress -match '^192\.168\.1\.' } | Select-Object -First 1).IPAddress" 2>/dev/null | tr -d '\r' || true)"
  WIN_IP="$(_extract_ip "$WIN_IP")"
fi
if [[ -z "${WIN_IP}" ]]; then
  WIN_IP="192.168.1.235"
fi

echo ""
echo "LAN IP for Quest: ${WIN_IP}"
echo "If Quest cannot connect, run ONCE as Admin in PowerShell:"
echo "  powershell -ExecutionPolicy Bypass -File install_scripts/setup_quest3_ports.ps1"
echo ""
echo "Quest browser URL (HTTPS — required for WebXR on Quest):"
echo "  https://${WIN_IP}:8766/webxr_client.html?host=${WIN_IP}"
echo ""
echo "On Quest: accept the certificate warning (Advanced -> Proceed) then Enter VR"
echo ""
echo "Order: MuJoCo 9 -> press s HERE (see 'Policy started') -> v -> Enter VR on Quest"
echo "Keys: c=quick calib | j=joint wizard | s=start | v=arms | o=stop"
echo "Full guide: docs/quest3_teleop.md"
echo "Tip: focus THIS terminal before pressing keys (not PowerShell / deploy window)"
echo ""

exec python gear_sonic/scripts/quest3_manager_server.py --host-ip "${WIN_IP}" "$@"
