#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${MOWEN_VENDOR_WORKSPACE_ROOT:-${1:-}}"

if [[ -z "${WORKSPACE_ROOT}" ]]; then
  echo "MOWEN_VENDOR_WORKSPACE_ROOT is not set and no workspace path argument was provided" >&2
  exit 2
fi
if [[ ! -d "${WORKSPACE_ROOT}" ]]; then
  echo "Vendor workspace root does not exist: ${WORKSPACE_ROOT}" >&2
  exit 2
fi
if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_ROOT}/devel/setup.bash"
elif [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_ROOT}/install/setup.bash"
else
  echo "Vendor workspace is missing devel/setup.bash and install/setup.bash: ${WORKSPACE_ROOT}" >&2
  exit 2
fi
exec roslaunch ruikang_recon_baseline mowen_vendor_sidecar.launch "${@:2}"
