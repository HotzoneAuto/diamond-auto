#include "modules/drivers/wheel/wheel_angle.h"
namespace apollo {
namespace drivers {
namespace wheel {

WheelAngleComponent::WheelAngleComponent() {}

bool WheelAngleComponent::Init() {
  if (!GetProtoConfig(&device_conf_)) {
    AERROR << "Unable to load rfid conf file: " << ConfigFilePath();
    return false;
  }

  ADEBUG << "Device conf:" << device_conf_.ShortDebugString();

  device_ = std::make_unique<Uart>(device_conf_.device_id().c_str());

  // Uart device set option
  device_->SetOpt(9600, 8, 'N', 1);

  wheel_angle_writer_ =
      node_->CreateWriter<WheelAngle>(device_conf_.output_channel());

  async_action_ = cyber::Async(&WheelAngleComponent::Action, this);
  return true;
}

void WheelAngleComponent::Action() {}

WheelAngleComponent::~WheelAngleComponent() {}

}  // namespace wheel
}  // namespace drivers
}  // namespace apollo
