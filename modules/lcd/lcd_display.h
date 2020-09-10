#pragma once
#include <cmath>
#include <memory>
#include "cyber/cyber.h"
#include "modules/common/util/uart.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/canbus/proto/chassis.pb.h"
namespace apollo {
namespace drivers {
namespace lcd {

using apollo::cyber::Component;

using apollo::cyber::Writer;

using apollo::canbus::Chassis;

class LcdComponet : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  bool Proc();

 private:
  Chassis chassis_;
  //  // TODO(all): auto config by udev
  std::unique_ptr<Uart> device_ = nullptr;

  std::future<bool> async_action_;

  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
};
CYBER_REGISTER_COMPONENT(LcdComponet)
}  // namespace lcd
}  // namespace drivers
}  // namespace apollo