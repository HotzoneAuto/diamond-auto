#pragma once

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/uart.h"
#include "modules/drivers/proto/wheelangle.pb.h"
#include "modules/drivers/wheel/proto/device_conf.pb.h"

namespace apollo {
namespace drivers {
namespace wheel {

using apollo::cyber::Writer;
using apollo::drivers::WheelAngle;

class WheelAngleComponent : public apollo::cyber::Component<> {
 public:
  WheelAngleComponent();
  ~WheelAngleComponent();
  bool Init() override;
  void Action();

 private:
  WheelDeviceConf device_conf_;
  std::unique_ptr<Uart> device_ = nullptr;
  std::shared_ptr<Writer<WheelAngle>> wheel_angle_writer_ = nullptr;
  std::future<void> async_action_;

    // TODO: calibration, unit: mm
  double wheel_diameter = 620;
};

CYBER_REGISTER_COMPONENT(WheelAngleComponent)

}  // namespace wheel
}  // namespace drivers
}  // namespace apollo
