#include <memory>
#include "cyber/cyber.h"
#include "modules/canbus/proto/chassis.pb.h"

void MessageCallback(
    const std::shared_ptr<apollo::canbus::Chassis>& msg) {
  AINFO << "speed_mps-> " << msg->speed_mps();
  AINFO << "driving_mode->" << msg->driving_mode();
  AINFO << "vehicle_id" << msg->vehicle_id().vin();
}

int main(int argc, char* argv[]) {
  // init cyber framework   
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("listener");
  // create listener
  // /diamond/canbus/chassis
  auto listener =
      listener_node->CreateReader<apollo::canbus::Chassis>(
          "/diamond/canbus/chassis", MessageCallback);
  AINFO << listener ;
  apollo::cyber::WaitForShutdown();
  return 0;
}
