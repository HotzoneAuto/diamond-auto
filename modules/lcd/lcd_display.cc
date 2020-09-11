#include "modules/lcd/lcd_display.h"

#include "math.h"

namespace apollo {
namespace drivers {
namespace lcd {
bool LcdComponet::Init() {
  device_ = std::make_unique<Uart>("ttyUSB0");
  // Uart device set option
  device_->SetOpt(9600, 8, 'N', 1);

  // /diamond/canbus/chassis
  chassis_reader_ = node_->CreateReader<Chassis>(
      "/diamond/canbus/chassis",
      [this](const std::shared_ptr<Chassis>& chassis) {
        chassis_.CopyFrom(*chassis);
      });
  // AINFO << listener;
  async_action_ = cyber::Async(&LcdComponet::Proc, this);
  // async_action_=std::async(std::launch::async,&LcdComponet::Proc,this);
  return true;
}
bool LcdComponet::Proc() {
  while (!apollo::cyber::IsShutdown()) {
    unsigned char vehicle_id[8] = {0x01, 0x06, 0x00, 0x00,
                                   0x00, 0x01, 0x48, 0x0A};
    int results = device_->Write(vehicle_id, 8);
    AINFO << "results->" << results;
    // unsigned char c0[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD8, 0x0A};

    // unsigned char c1[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x01, 0x19, 0xCA};

    // unsigned char c2[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x02, 0x59, 0xCB};

    // unsigned char c3[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x03, 0x98, 0x0B};

    // unsigned char c4[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x04, 0xD9, 0xC9};

    // unsigned char c5[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x05, 0x18, 0x09};

    // unsigned char c6[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x06, 0x58, 0x08};

    // unsigned char c7[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x07, 0x99, 0xC8};

    // unsigned char c8[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x08, 0xD9, 0xCC};
  }
  return true;
}
}  // namespace lcd
}  // namespace drivers
}  // namespace apollo