#include <cmath>
#include <memory>
#include "cyber/cyber.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/util/uart.h"

void MessageCallback(const std::shared_ptr<apollo::canbus::Chassis>& msg) {
  int result;
  std::unique_ptr<Uart> device_ = nullptr;
  device_ = std::make_unique<Uart>("ttyUSB0");
  device_->SetOpt(9600, 8, 'N', 1);
  unsigned char vehicle_id[8] = {0x01, 0x06, 0x00, 0x00,
                                 0x00, 0x01, 0x48, 0x0A};
  int results = device_->Write(vehicle_id, 8);
  AINFO << "vehicle_id->" << msg->vehicle_id().vin();
  AINFO << "results->" << results;
  AINFO << "speed_mps-> " << msg->speed_mps();
  int speed = round(3.6 * msg->speed_mps());

  unsigned char c0[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD8, 0x0A};

  unsigned char c1[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x01, 0x19, 0xCA};

  unsigned char c2[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x02, 0x59, 0xCB};

  unsigned char c3[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x03, 0x98, 0x0B};

  unsigned char c4[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x04, 0xD9, 0xC9};

  unsigned char c5[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x05, 0x18, 0x09};

  unsigned char c6[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x06, 0x58, 0x08};

  unsigned char c7[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x07, 0x99, 0xC8};

  unsigned char c8[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x08, 0xD9, 0xCC};

  switch (speed) {
    case 0:
      result = device_->Write(c0, 8);
      AINFO << "Current speed->" << result;
      break;
    case 1:
      result = device_->Write(c1, 8);
      AINFO << "Current speed->" << result;
      break;
    case 2:
      result = device_->Write(c2, 8);
      AINFO << "Current speed->" << result;
      break;
    case 3:
      result = device_->Write(c3, 8);
      AINFO << "Current speed->" << result;
      break;
    case 4:
      result = device_->Write(c4, 8);
      AINFO << "Current speed->" << result;
      break;
    case 5:
      result = device_->Write(c5, 8);
      AINFO << "Current speed->" << result;
      break;
    case 6:
      result = device_->Write(c6, 8);
      AINFO << "Current speed->" << result;
      break;
    case 7:
      result = device_->Write(c7, 8);
      AINFO << "Current speed->" << result;
      break;
    default:
      result = device_->Write(c8, 8);
      AINFO << "Current speed->" << result;
      break;
  }
  AINFO << "driving_mode->" << msg->driving_mode();
  if (msg->driving_mode() == 0 or msg->driving_mode() == 1) {
    unsigned char driving_mode[8] = {0x01, 0x06, 0x00, 0x02,
                                     0x00, 0x01, 0xE9, 0xCA};
    device_->Write(driving_mode, 8);
  } else if (msg->driving_mode() == 1 or msg->driving_mode() == 2 or
             msg->driving_mode() == 3) {
    unsigned char driving_mode[8] = {0x01, 0x06, 0x00, 0x02,
                                     0x00, 0x00, 0x28, 0x0A};
    device_->Write(driving_mode, 8);
  }
  // electricity_quantity
  /*
  unsigned char eq0[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x00, 0xC8, 0x0B};

  unsigned char eq1[8] = {0x01, 0x06, 0x00, 0x04,
                      0x00, 0x01, 0x09, 0xCB};

  unsigned char eq5[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x05, 0x08, 0x08};

  unsigned char eq10[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x0A, 0x48, 0x0C};

  unsigned char eq15[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x0F, 0x88, 0x0F};

  unsigned char eq20[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x14, 0xC8, 0x04};

  unsigned char eq25[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x19, 0x09, 0xC1};

  unsigned char eq30[0] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x1E, 0x48, 0x03};

  unsigned char eq35[0] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x23, 0x89, 0xD2};

  unsigned char eq40[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x28, 0xC8, 0x15};

  unsigned char eq41[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x29, 0x09, 0xD5};

  unsigned char eq42[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x2A, 0x49, 0xD4};

  unsigned char eq43[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x2B, 0x88, 0x14};

  unsigned char eq44[0] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x2C, 0xC9, 0xD6};

  unsigned char eq45[0] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x2D, 0x08, 0x16};

  unsigned char eq46[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x2E, 0x48, 0x17};

  unsigned char eq47[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x2F, 0x89, 0xD7};

  unsigned char eq48[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x30, 0x09, 0xDF};

  unsigned char eq49[8] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x31, 0x09, 0xDF};
  unsigned char eq50[0] = {0x01, 0x06, 0x00, 0x04,
                        0x00, 0x32, 0x49, 0xDE};
  */
  AINFO << "bat_percentage->" << msg->bat_percentage();
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("lcd_display");
  // create listener
  // /diamond/canbus/chassis
  auto listener = listener_node->CreateReader<apollo::canbus::Chassis>(
      "/diamond/canbus/chassis", MessageCallback);
  AINFO << listener;
  apollo::cyber::WaitForShutdown();
  return 0;
}
