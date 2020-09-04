#pragma once

#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/common/adapters/adapter_gflags.h"


namespace apollo {
namespace control {

using apollo::cyber::Component;
using apollo::canbus::Chassis;
using apollo::control::ControlCommand;

class ControlComponent: public Component<> {
public:
  // ControlComponent();
  bool Init() override;
  void GenerateCommand();
  ~ControlComponent();

private:
  Chassis chassis_;

  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::Magnetic>> magnetic_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::RFID>> rfid_reader_;
  std::shared_ptr<apollo::cyber::Writer<apollo::canbus::Chassis>> chassis_writer_;

  std::shared_ptr<apollo::cyber::Writer<ControlCommand>> control_cmd_writer_;

  std::future<void> async_action_;
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo
