 #!/usr/bin/env bash
   
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
   
    cd "${DIR}/.."
   
    source "${DIR}/apollo_base.sh"
   
    function start() {
       LOG_LCD_DISPLAY="${APOLLO_ROOT_DIR}/data/log/lcd_display.out"
       CMD_LCD_DISPLAY="cyber_launch start /apollo/modules/lcd/launch/lcd_display.launch"
       NUM_PROCESSES_LCD_DISPLAY="$(pgrep -c -f "modules/lcd/dag/rfid.dag")"
       if [ "${NUM_PROCESSES_LCD_DISPLAY}" -eq 0 ]; then
           eval "nohup ${CMD_LCD_DISPLAY} </dev/null >${LOG_LCD_DISPLAY} 2>&1 &"
       fi
       

       LOG_RFID="${APOLLO_ROOT_DIR}/data/log/rfid.out"
       CMD_RFID="cyber_launch start /apollo/modules/lcd/launch/lcd_display.launch"
       NUM_PROCESSES_RFID="$(pgrep -c -f "modules/lcd/dag/rfid.dag")"
       if [ "${NUM_PROCESSES_RFID}" -eq 0 ]; then
           eval "nohup ${CMD_RFID} </dev/null >${LOG_RFID} 2>&1 &"
       fi

       LOG_WHEEL="${APOLLO_ROOT_DIR}/data/log/wheel.out"
       CMD_WHEEL="cyber_launch start /apollo/modules/drivers/wheel/launch/wheel.launch"
       NUM_PROCESSES_WHEEL="$(pgrep -c -f "modules/drivers/wheel/dag/wheel_angle_*.dag")"
       if [ "${NUM_PROCESSES_WHEEL}" -eq 0 ]; then
           eval "nohup ${CMD_WHEEL} </dev/null >${LOG_WHEEL} 2>&1 &"
       fi

       LOG_CANBUS="${APOLLO_ROOT_DIR}/data/log/canbus.out"
       CMD_CANBUS="cyber_launch start /apollo/modules/canbus/launch/canbus.launch"
       NUM_PROCESSES_CANBUS="$(pgrep -c -f "/apollo/modules/canbus/dag/canbus.dag")"
       if [ "${NUM_PROCESSES_CANBUS}" -eq 0 ]; then
           eval "nohup ${CMD_CANBUS} </dev/null >${LOG_CANBUS} 2>&1 &" 
       fi

       LOG_CONTROL="${APOLLO_ROOT_DIR}/data/log/control.out"
       CMD_CONTROL="cyber_launch start /apollo/modules/control/launch/control.launch"
       NUM_PROCESSES_CONTROL="$(pgrep -c -f "modules/control/dag/control.dag")"
       if [ "${NUM_PROCESSES_CONTROL}" -eq 0 ]; then
           eval "nohup ${CMD_CONTROL} </dev/null >${LOG_CONTROL} 2>&1 &"
       fi
    }
  
    function stop() {
       eval "nohup cyber_launch stop /apollo/modules/lcd/launch/lcd_display.launch < /dev/null 2>&1 &"
       eval "nohup cyber_launch stop /apollo/modules/drivers/rfid/launch/rfid.launch < /dev/null 2>&1 &"
       eval "nohup cyber_launch stop /apollo/modules/drivers/wheel/launch/wheel.launch < /dev/null 2>&1 &"
       eval "nohup cyber_launch stop /apollo/modules/canbus/launch/canbus.launch < /dev/null 2>&1 &"
       eval "nohup cyber_launch stop /apollo/modules/control/launch/control.launch < /dev/null 2>&1 &"
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

