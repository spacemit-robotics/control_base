#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/control/base/${SROBOTIS_TEST_NAME:-chassis-rpmsg-hardware-smoke}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/chassis_rpmsg_hardware_smoke.log"
build_dir="$artifact_dir/build"

ctrl_dev="${CHASSIS_RPMSG_CTRL_DEV:-/dev/rpmsg_ctrl0}"
data_dev="${CHASSIS_RPMSG_DATA_DEV:-/dev/rpmsg0}"
service_name="${CHASSIS_RPMSG_SERVICE_NAME:-rpmsg:motor_ctrl}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] ctrl_dev=$ctrl_dev"
    echo "[info] data_dev=$data_dev"
    echo "[info] service_name=$service_name"

    if [[ ! -e "$ctrl_dev" ]]; then
        echo "[error] RPMsg control device does not exist: $ctrl_dev" >&2
        exit 1
    fi
    if [[ ! -e "$data_dev" ]]; then
        echo "[error] RPMsg data device does not exist: $data_dev" >&2
        exit 1
    fi

    cc -D_DEFAULT_SOURCE -std=c99 -Wall -Wextra -pedantic \
        -I"$module_root/include" \
        -I"$module_root/src" \
        "$module_root/src/chassis_core.c" \
        "$module_root/src/drivers/drv_rpmsg_esos.c" \
        "$module_root/test/test_chassis_rpmsg.c" \
        -pthread -lm \
        -o "$build_dir/test_chassis_rpmsg"

    timeout --signal=INT --kill-after=5s 8s \
        "$build_dir/test_chassis_rpmsg" \
        --ctrl "$ctrl_dev" \
        --data "$data_dev" \
        --service "$service_name"
} | tee "$log_file"

grep -q "RPMsg chassis created and started" "$log_file"
grep -q "Done" "$log_file"
