#!/usr/bin/env bash
# Fix WSLg invisible GUI windows ([WARN:COPY MODE] in taskbar).
# See: https://github.com/microsoft/wslg/issues/1456

set -euo pipefail

if [ -d /mnt/shared_memory ] && mountpoint -q /mnt/shared_memory 2>/dev/null; then
  exit 0
fi

if grep -q "rdp_allocate_shared_memory: Failed to open" /mnt/wslg/weston.log 2>/dev/null; then
  echo "WSLg shared-memory error detected (invisible GUI / [WARN:COPY MODE])."
fi

if [ ! -d /mnt/shared_memory ]; then
  if sudo -n mkdir -p /mnt/shared_memory 2>/dev/null; then
    :
  else
    echo ""
    echo "MuJoCo viewer needs WSLg shared memory. Run ONE of these, then retry:"
    echo ""
    echo "  PowerShell (recommended):  wsl --shutdown"
    echo "  Then open a new terminal and run the sim again."
    echo ""
    echo "  Or in WSL (needs sudo password):"
    echo "    sudo mkdir -p /mnt/shared_memory"
    echo "    sudo mount -t tmpfs tmpfs /mnt/shared_memory"
    echo ""
    exit 1
  fi
fi

if ! mountpoint -q /mnt/shared_memory 2>/dev/null; then
  if ! sudo -n mount -t tmpfs tmpfs /mnt/shared_memory 2>/dev/null; then
    echo "Could not mount /mnt/shared_memory without sudo. Run: wsl --shutdown (from PowerShell)"
    exit 1
  fi
fi

echo "WSLg display fix applied (/mnt/shared_memory mounted)."
