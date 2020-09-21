#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"
#include "cyber/cyber.h"

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"

#include "modules/canbus/vehicle/diamond/protocol/id_0x00aa5701.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0cfff3a7.h"

void MessageCallback(
    const std::shared_ptr<apollo::canbus::ChassisDetail>& msg) {

  if(msg->diamond().id_0x0c09a7f0().fmotvolt() < 10) {
    AINFO << "in motor_vol_down function: motor_vol < 10";
    std::cout << "in motor_vol_down function: motor_vol < 10"  << std::endl;
    apollo::cyber::AsyncShutdown();
    return;
  }

  if (std::abs(msg->diamond().id_0x1818d0f3().fbatcur()) < 5 and
      std::abs(msg->diamond().id_0x0c08a7f0().fmotcur()) < 5) {
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
}

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("canbus_stop_listener");
  // create listener
  auto listener = listener_node->CreateReader<apollo::canbus::ChassisDetail>(
      "/diamond/canbus/chassis_detail", MessageCallback);

  apollo::cyber::WaitForShutdown();

  return 0;
}
