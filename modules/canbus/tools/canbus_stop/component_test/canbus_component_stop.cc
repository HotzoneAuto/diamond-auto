#include "modules/canbus/tools/canbus_stop/component_test/canbus_component_stop.h"
#include <iostream>
using namespace std;

bool canbus_component_stop::Init() {
  AINFO << "Commontest component init";
  cout << "commontest component init" << endl;
  return true;
}

bool canbus_component_stop::Proc(
    const std::shared_ptr<apollo::canbus::ChassisDetail>& msg) {
  if (std::abs(msg->diamond().id_0x1818d0f3().fbatcur()) < 5 and
      std::abs(msg->diamond().id_0x0c08a7f0().fmotcur())) {
    std::string cmd = "cansend can0 00AA5701#0000000000000000";
    const int ret = std::system(cmd.c_str());
    if (ret == 0) {
      AINFO << "Battery K1 down can message send SUCCESS: " << cmd;
    } else {
      AERROR << "Battery K1 down can message send FAILED(" << ret
             << "): " << cmd;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::string cmd1 = "cansend can0 0CFFF3A7#0002000000000000";
    const int ret1 = std::system(cmd1.c_str());
    if (ret1 == 0) {
      AINFO << "BMS message send SUCCESS: " << cmd1;
    } else {
      AERROR << "BMS message send FAILED(" << ret1 << "): " << cmd1;
    }
  }

  return true;
}
