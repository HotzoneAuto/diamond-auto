#include "modules/canbus/tools/component_motorVolUp/mortor_vol_up.h"
#include "modules/canbus/vehicle/diamond/diamond_message_manager.h"
#include <iostream>
using namespace std;

bool mortor_vol_up::Init() {
  AINFO << "Commontest component init";
  cout << "commontest component init"  << endl;
  return true;
}

bool mortor_vol_up::Proc(const std::shared_ptr<apollo::canbus::ChassisDetail>& msg) {

  apollo::canbus::ChassisDetail chassis_detail;
  chassis_detail = *msg;
  if (!chassis_detail.diamond().has_id_0x1818d0f3()) {
    AINFO << "empty chassis detail, waiting.....";
    std::this_thread::sleep_for(5s);
    chassis_detail.Clear();
//    message_manager_->GetSensorData(&chassis_detail);
  }

  // 1. check error flag
  if (chassis_detail.diamond().id_0x0c0ba7f0().dwmcuerrflg() != 0) {
    AERROR << "SetMotorVoltageUp flag check Error:"
           << chassis_detail.diamond().id_0x0c0ba7f0().dwmcuerrflg();
    return false;
  }
  // 2. Tell BMS you can release voltage now
  std::string cmd1 = "cansend can0 0CFFF3A7#0001000000000000";
  const int ret1 = std::system(cmd1.c_str());
  if (ret1 == 0) {
    AINFO << "BMS message send SUCCESS: " << cmd1;
  } else {
    AERROR << "BMS message send FAILED(" << ret1 << "): " << cmd1;
  }

  if (chassis_detail.diamond().id_0x1818d0f3().has_bybatnegrlysts() != false or
      chassis_detail.diamond().id_0x1818d0f3().bybatnegrlysts() == 1) {
    if (chassis_detail.diamond().id_0x1818d0f3().bybatinsrerr() != 0) {
      AERROR << "1818d0f3 bybatinsrerr REEOR!!";
      return false;
    }
    // 3. K2 up
    std::string cmd2 = "cansend can0 00AA5701#1000000000000000";
    const int ret2 = std::system(cmd2.c_str());
    if (ret2 == 0) {
      AINFO << "K2 up message send SUCCESS: " << cmd2;
    } else {
      AERROR << "K2 up message send FAILED(" << ret2 << "): " << cmd2;
    }
    std::this_thread::sleep_for(5s);
    chassis_detail.Clear();
//    message_manager_->GetSensorData(&chassis_detail);
    if (std::abs(chassis_detail.diamond().id_0x1818d0f3().fbatvolt() -
                 chassis_detail.diamond().id_0x0c09a7f0().fmotvolt()) < 25) {
      // 4. K1 up
      std::string cmd3 = "cansend can0 00AA5701#1100000000000000";
      const int ret3 = std::system(cmd3.c_str());
      if (ret3 == 0) {
        AINFO << "K1 up can message send SUCCESS: " << cmd3;
      } else {
        AERROR << "K1 up message send FAILED(" << ret3 << "): " << cmd3;
      }
      std::this_thread::sleep_for(3s);
      // 5. K2 down
      std::string cmd4 = "cansend can0 00AA5701#0100000000000000";
      const int ret4 = std::system(cmd4.c_str());
      std::this_thread::sleep_for(3s);
      if (ret4 == 0) {
        AINFO << "K2 down message send SUCCESS: " << cmd4;
      } else {
        AERROR << "K2 down message send FAILED(" << ret4 << "): " << cmd4;
      }
      // 6.Done
    } else if (std::abs(chassis_detail.diamond().id_0x1818d0f3().fbatvolt() -
                        chassis_detail.diamond().id_0x0c09a7f0().fmotvolt()) >
               25) {
      AERROR << "diff > 25, K2 down";
      std::string cmd5 = "cansend can0 00AA5701#0000000000000000";
      const int ret = std::system(cmd5.c_str());
      if (ret == 0) {
        AINFO << "K2 down message send SUCCESS: " << cmd5;
      } else {
        AERROR << "K2 down message send FAILED(" << ret << "): " << cmd5;
      }
    }
  }
  return true;
}

