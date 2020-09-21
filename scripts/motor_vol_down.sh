#!/usr/bin/env bash

   DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
   
   cd "${DIR}/.."
   
   source "${DIR}/apollo_base.sh"
   
  function start() {
      eval "${APOLLO_BIN_PREFIX}/modules/canbus/tools/canbus_stop/canbus_stop  \
        --log_dir=${APOLLO_ROOT_DIR}/data/log"
  }
  
  function stop() {
      pkill -SIGKILL -f canbus_stop
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
