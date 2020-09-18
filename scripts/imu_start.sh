DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

# TODO(all): This is just an initial commit. Velodyne 64 and 16 share lots of
# things. We need to select their processes precisely in 'pgrep', 'pkill' and
# the monitor module.
function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/imu.out"
    CMD="roslaunch imu imu.launch"
    NUM_PROCESSES="$(pgrep -c -f "imu")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    pkill -SIGTERM -f imu.launch
}

# run command_name module_name
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

