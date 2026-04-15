#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${1:-$(mktemp -d /tmp/ruikang_noetic_verify_XXXXXX)}"
CATKIN_WS="${WORKSPACE_ROOT}/catkin_ws"
SRC_DIR="${CATKIN_WS}/src"
PACKAGE_LINK="${SRC_DIR}/$(basename "${PACKAGE_ROOT}")"

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[verify] missing required command: $1" >&2
    exit 2
  fi
}

require_command python3
require_command catkin_make
require_command rostest
require_command roscore

if [ -f "/opt/ros/noetic/setup.bash" ]; then
  source /opt/ros/noetic/setup.bash
else
  echo "[verify] /opt/ros/noetic/setup.bash not found" >&2
  exit 2
fi

python3 "${PACKAGE_ROOT}/tools/validate_repository_hygiene.py"
python3 "${PACKAGE_ROOT}/tools/validate_launch_contracts.py"
python3 "${PACKAGE_ROOT}/tools/validate_profile_contracts.py"
python3 "${PACKAGE_ROOT}/tools/validate_static_python_contracts.py"
python3 "${PACKAGE_ROOT}/tools/validate_third_party_governance.py"
python3 "${PACKAGE_ROOT}/tools/validate_field_asset_release.py"
if [ -z "${RUIKANG_VENDOR_WORKSPACE_ROOT:-}" ]; then
  RUIKANG_VENDOR_WORKSPACE_ROOT="$(python3 "${PACKAGE_ROOT}/tools/provision_vendor_workspace_fixture.py")"
fi
python3 "${PACKAGE_ROOT}/tools/validate_managed_vendor_bundle.py" --workspace-root "${RUIKANG_VENDOR_WORKSPACE_ROOT}" --require-external-workspace

mkdir -p "${SRC_DIR}"
rm -rf "${PACKAGE_LINK}"
ln -s "${PACKAGE_ROOT}" "${PACKAGE_LINK}"

pushd "${CATKIN_WS}" >/dev/null
catkin_make
catkin_make tests
catkin_make run_tests_ruikang_recon_baseline

for launch_file in   tests/demo_profile_smoke.test   tests/baseline_profile_smoke.test   tests/baseline_contract_smoke.test   tests/baseline_deploy_smoke.test   tests/baseline_deploy_feedback_timeout_smoke.test   tests/baseline_integration_smoke.test   tests/baseline_integration_namespace_smoke.test   tests/dynamic_schema_integration_smoke.test   tests/dynamic_schema_integration_namespace_smoke.test   tests/dynamic_schema_mismatch_smoke.test   tests/reference_field_runtime_smoke.test   tests/field_runtime_smoke.test; do
  echo "[verify] rostest ${launch_file}"
  rostest ruikang_recon_baseline "${PACKAGE_ROOT}/${launch_file}"
done

TEST_RESULTS_DIR="${CATKIN_WS}/build/test_results"
if [ -d "${TEST_RESULTS_DIR}" ]; then
  python3 -m unittest discover -s "${PACKAGE_ROOT}/tests" -p 'test_*.py'
  find "${TEST_RESULTS_DIR}" -type f \( -name '*.xml' -o -name '*.txt' \) -print | sort
fi
popd >/dev/null

echo "[verify] workspace: ${CATKIN_WS}"
echo "[verify] completed static validators + catkin_make + explicit rostest smoke suite"
