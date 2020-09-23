#pragma once
#include <cmath>
#include <memory>
#include "cyber/component/component.h"
#include "modules/common/util/uart.h"
#include "modules/drivers/parking/proto/parking_brake_conf.pb.h"
namespace apollo{
namespace drivers{
namespace parking{

using apollo::cyber::Component;
using apollo::cyber::Writer;

class ParkingComponet:public Component<>{
private:
    ParkingDeviceConf device_conf_;
    std::unique_ptr<Uart> device_ = nullptr;
    std::future<void> async_action_;

public:
  bool Init() override;
  void Proc();
};
CYBER_REGISTER_COMPONENT(ParkingComponet)
}//parking
}//drivers
}//apollo