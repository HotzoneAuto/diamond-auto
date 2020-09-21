#!/usr/bin/env bash

   DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

   cd "${DIR}/.."

   source "${DIR}/apollo_base.sh"

  function up() {
      eval "${APOLLO_BIN_PREFIX}/modules/canbus/tools/motor_vol_up_node  \
        --log_dir=${APOLLO_ROOT_DIR}/data/log"
  }

  function down() {
      eval "${APOLLO_BIN_PREFIX}/modules/canbus/tools/motor_vol_down_node  \
        --log_dir=${APOLLO_ROOT_DIR}/data/log"
  }


  function stop() {
      pkill -SIGKILL -f motor_vol_up_node
      pkill -SIGKILL -f motor_vol_down_node
  }

  # run command_name module_name
  function run() {
      case $1 in
          up)
              up
              ;;
          down)
              down
              ;;
          stop)
              stop
              ;;
          *)
              up
              ;;
      esac
  }

  run "$1"

