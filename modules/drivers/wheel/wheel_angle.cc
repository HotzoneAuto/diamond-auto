#include "modules/drivers/wheel/wheel_angle.h"
#include "cyber/time/rate.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace drivers {
namespace wheel {
using apollo::cyber::Rate;

WheelAngleComponent::WheelAngleComponent() {}

std::string WheelAngleComponent::Name() const { return "wheel"; }

bool WheelAngleComponent::Init() {
  if (!GetProtoConfig(&device_conf_)) {
    AERROR << "Unable to load wheel angle conf file: " << ConfigFilePath();
    return false;
  }

  ADEBUG << "Device conf:" << device_conf_.ShortDebugString();

  device_ = std::make_unique<Uart>(device_conf_.device_id().c_str());

  // Uart front device set option
  device_->SetOpt(9600, 8, 'N', 1);
  wheel_angle_writer_ =
      node_->CreateWriter<WheelAngle>(device_conf_.output_channel());

  async_action_ = cyber::Async(&WheelAngleComponent::Action, this);
  return true;
}

void WheelAngleComponent::Action() {
  WheelAngle angle;
  common::util::FillHeader(node_->Name(), &angle);
  Rate rate(100.0);
  while (!apollo::cyber::IsShutdown()) {
    int count = 0;
    static char buffer[10];
    static char buf;

    // Send read Data message
    unsigned char cmd[8] = {0x01, 0x03, 0x10, 0x00, 0x00, 0x02, 0xC0, 0xCB};
    int result = device_->Write(cmd, 8);
    ADEBUG << "CalWheelAngle command send result:" << result;

    count = 0;
    double wheel_angle = 0.0;
    std::memset(buffer, 0, 10);
    for (count = 0; count < 9; count++) {
      int ret = device_->Read(&buf, 1);
      ADEBUG << "READ RETURN :" << ret;
      if (ret == 1) {
        buffer[count] = buf;
      } else {
        std::memset(buffer, 0, 10);
        break;
      }
      if (count == 8) {
        // auto header = angle.mutable_header();
        // header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
        // header->set_frame_id("wheel_angle");

        if (buffer[5] == 0xFF && buffer[6] == 0xFF) {
          wheel_angle =
              0.1 * (-(pow(2, 16) - 1) + (static_cast<int>(buffer[3]) * 256 +
                                          (static_cast<int>(buffer[4])) - 1));
        } else if (buffer[5] == 0x00 && buffer[6] == 0x00) {
          wheel_angle = 0.1 * (static_cast<int>(buffer[3]) * 256 +
                               static_cast<int>(buffer[4]));
        }
      }
    }
    angle.set_value(wheel_angle);
    ADEBUG << angle.DebugString();
    wheel_angle_writer_->Write(angle);
    rate.Sleep();
  }
}

WheelAngleComponent::~WheelAngleComponent() {
  device_ = nullptr;
  async_action_.wait();
}

}  // namespace wheel
}  // namespace drivers
}  // namespace apollo
