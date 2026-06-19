#!/usr/bin/env bash
set -euo pipefail
LIB="$(cd "$(dirname "${BASH_SOURCE[0]}")/../gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64" && pwd)"
for base in libddsc libddscxx; do
  if [ -f "${LIB}/${base}.so" ]; then
    cp -f "${LIB}/${base}.so" "${LIB}/${base}.so.0"
    echo "Fixed ${base}.so.0 ($(stat -c%s "${LIB}/${base}.so.0") bytes)"
  fi
done
