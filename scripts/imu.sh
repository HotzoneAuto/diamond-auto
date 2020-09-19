#!/usr/bin/env bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/imu.out"
    CMD="cyber_launch start /apollo/modules/drivers/imu/launch/imu.launch"
    NUM_PROCESSES="$(pgrep -c -f "/apollo/modules/drivers/imu/dag/imu.dag")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
#       eval "${CMD}"
    fi
}

function stop() {
    eval "nohup cyber_launch stop /apollo/modules/drivers/imu/launch/imu.launch < /dev/null 2>&1 &"
}

function run() {
    case $1 in
        start)
            start
            ;;
        stop)
            stop
            ;;
        *)
            start
            ;;
    esac
}

run "$1"

