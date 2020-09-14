#include "modules/drivers/wheel/wheel_angle.h"
namespace apollo {
namespace drivers {
namespace wheel {

WheelAngleComponent::WheelAngleComponent() {}

bool WheelAngleComponent::Init() {
  if (!GetProtoConfig(&front_device_conf_)) {
    AERROR << "Unable to load front wheel angle conf file: " << ConfigFilePath();
    return false;
  }

  ADEBUG << "Front device conf:" << front_device_conf_.ShortDebugString();

  front_device_ = std::make_unique<Uart>(front_device_conf_.device_id().c_str());

  // Uart front device set option
  front_device_->SetOpt(9600, 8, 'N', 1);

  front_wheel_angle_writer_ =
      node_->CreateWriter<WheelAngle>(front_device_conf_.output_channel());

  if (!GetProtoConfig(&rear_device_conf_)) {
    AERROR << "Unable to load rear wheel angle conf file: " << ConfigFilePath();
    return false;
  }

  ADEBUG << "Rear device conf:" << rear_device_conf_.ShortDebugString();

  rear_device_ = std::make_unique<Uart>(rear_device_conf_.device_id().c_str());

  // Uart rear device set option
  rear_device_->SetOpt(9600, 8, 'N', 1);

  rear_wheel_angle_writer_ =
      node_->CreateWriter<WheelAngle>(rear_device_conf_.output_channel());

  async_action_ = cyber::Async(&WheelAngleComponent::Action, this);
  return true;
}

void WheelAngleComponent::Action() {
  WheelAngle front_angle;
  WheelAngle rear_angle;
  while (!apollo::cyber::IsShutdown()) {
    double front_wheel_angle = CalWheelAngle(front_device_, front_angle);
    doubel rear_wheel_angle = CalWheelAngle(rear_device_, rear_angle);
    front_angle.set_front_wheel_angle(front_wheel_angle);
    rear_angle.set_rear_wheel_angle(rear_wheel_angle);
    ADEBUG << front_angle.DebugString();
    ADEBUG << rear_angle.DebugString();
    front_wheel_angle_writer_->Write(front_angle);
    rear_wheel_angle_writer_->Write(rear_angle);
  }
}

WheelAngleComponent::~WheelAngleComponent() {
  front_device_ = nullptr;
  rear_device_ = nullptr;
  async_action_.wait();
}

double WheelAngleComponent::CalWheelAngle(
      std::unique_ptr<Uart> device_, WheelAngle & angle) {
  int count = 0;
  static char buffer[10];
  static char buf;

  // Send read Data message
  unsigned char cmd[8] = {0x01, 0x03, 0x10, 0x00, 0x00, 0x02, 0xC0, 0xCB};
  int result = device_->Write(cmd, 8);
  ADEBUG << "CalWheelAngle command send result:" << result;

  count = 0;
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
    ADEBUG << "buf:" << buf << " count:" << count;
    if (count == 8) {
      ADEBUG << "count == 8";
      auto header = angle.mutable_header();
      header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
      header->set_frame_id("wheel_angle");

      AINFO << buffer[3] << buffer[4];
      double wheel_angle = 0.0;

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
  return wheel_angle;
}

}  // namespace wheel
}  // namespace drivers
}  // namespace apollo
