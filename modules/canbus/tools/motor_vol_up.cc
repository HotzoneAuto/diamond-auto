#include "cyber/cyber.h"

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/common/adapters/adapter_gflags.h"

static bool k2_on = false;

void MessageCallback(
    const std::shared_ptr<apollo::canbus::ChassisDetail>& msg) {
  auto diamond = msg->diamond();

  if (!diamond.has_id_0x1818d0f3()) {
    AINFO << "empty chassis detail, Skip this frame.";
    return;
  }
  if (diamond.id_0x0c09a7f0().fmotvolt() > 630) {
    AINFO << "Motor voltage have been done. Shutdown myself, Bye.";
    apollo::cyber::AsyncShutdown();
    return;
  }

  if (diamond.id_0x0c09a7f0().fmotvolt() > 618) {
    k2_on = true;
  }

  // 1. check error flag
  if (diamond.id_0x0c0ba7f0().dwmcuerrflg() != 0) {
    AERROR << "SetMotorVoltageUp flag check Error:"
           << diamond.id_0x0c0ba7f0().dwmcuerrflg();
    return;
  }

  // 2. Tell BMS you can release voltage now
  if (!k2_on) {
    std::string cmd1 = "cansend can0 0CFFF3A7#0001000000000000";
    const int ret1 = std::system(cmd1.c_str());
    if (ret1 == 0) {
      AINFO << "BMS message send SUCCESS: " << cmd1;
    } else {
      AERROR << "BMS message send FAILED(" << ret1 << "): " << cmd1;
    }
  }

  if (!diamond.id_0x1818d0f3().bybatinsrerr()) {
    ADEBUG << "Meet error for field 1818d0f3 bybatinsrerr";
  }

  if (!diamond.id_0x1818d0f3().bybatnegrlysts()) {
    ADEBUG << "Meet Error for field 1818d0f3 bybatnegrlysts";
  }

  // 3. K2 up
  if (!k2_on) {
    sleep(2);
    std::string cmd2 = "cansend can0 00AA5701#1000000000000000";
    const int ret2 = std::system(cmd2.c_str());
    if (ret2 == 0) {
      AINFO << "K2 up message send SUCCESS: " << cmd2;
      k2_on = true;
      std::this_thread::sleep_for(std::chrono::seconds(6));
      return;
    } else {
      AERROR << "K2 up message send FAILED(" << ret2 << "): " << cmd2;
    }
  }

  if (std::abs(diamond.id_0x1818d0f3().fbatvolt() -
               diamond.id_0x0c09a7f0().fmotvolt()) < 25) {
    // 4. K1 up
    std::string cmd3 = "cansend can0 00AA5701#1100000000000000";
    const int ret3 = std::system(cmd3.c_str());
    if (ret3 == 0) {
      AINFO << "K1 up can message send SUCCESS: " << cmd3;
    } else {
      AERROR << "K1 up message send FAILED(" << ret3 << "): " << cmd3;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // 5. K2 down
    std::string cmd4 = "cansend can0 00AA5701#0100000000000000";
    const int ret4 = std::system(cmd4.c_str());
    if (ret4 == 0) {
      AINFO << "K2 down message send SUCCESS: " << cmd4;
    } else {
      AERROR << "K2 down message send FAILED(" << ret4 << "): " << cmd4;
    }
    // 6.Done
  } else {
    AERROR << "diff > 25, K2 down";
    std::string cmd5 = "cansend can0 00AA5701#0000000000000000";
    const int ret = std::system(cmd5.c_str());
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (ret == 0) {
      AINFO << "K2 down message send SUCCESS: " << cmd5;
    } else {
      AERROR << "K2 down message send FAILED(" << ret << "): " << cmd5;
    }
  }
}

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("motor_vol_up_listener");
  // create listener
  auto listener = listener_node->CreateReader<apollo::canbus::ChassisDetail>(
      FLAGS_chassis_detail_topic, MessageCallback);

  apollo::cyber::WaitForShutdown();

  return 0;
}
