 #!/usr/bin/env bash
   
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
   
    cd "${DIR}/.."
   
    source "${DIR}/apollo_base.sh"
   
    function start() {
       bash "${APOLLO_ROOT_DIR}/scripts/lcd_display.sh"
       bash "${APOLLO_ROOT_DIR}/scripts/rfid.sh"
       bash "${APOLLO_ROOT_DIR}/scripts/wheel.sh"
       bash "${APOLLO_ROOT_DIR}/scripts/parking_brake.sh"
       bash "${APOLLO_ROOT_DIR}/scripts/canbus.sh"
    }
  
    function stop() {
       eval "bash ${APOLLO_ROOT_DIR}/scripts/lcd_display.sh stop"
       eval "bash ${APOLLO_ROOT_DIR}/scripts/rfid.sh stop"
       eval "bash ${APOLLO_ROOT_DIR}/scripts/wheel.sh stop"
       eval "bash ${APOLLO_ROOT_DIR}/scripts/parking_brake.sh stop"
       eval "bash ${APOLLO_ROOT_DIR}/scripts/canbus.sh stop"
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
 
