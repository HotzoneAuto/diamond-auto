#pragma once

#include <memory>
#include <string>

#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/timer/timer.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/drivers/proto/rfid.pb.h"
#include "modules/drivers/proto/wheelangle.pb.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::control::ControlCommand;
using apollo::control::ControlConf;
using apollo::control::PadMessage;
using apollo::drivers::RFID;
using apollo::drivers::WheelAngle;

class ControlComponent final : public apollo::cyber::TimerComponent {
 public:
  ControlComponent();
  std::string Name() const;
  ~ControlComponent();

 private:
  bool Init() override;
  bool Proc() override;
  double PidSpeed();
  Chassis chassis_;
  ControlConf control_conf_;
  RFID rfid_;
  PadMessage pad_msg_;
  WheelAngle wheel_angle_;
  bool pad_received_ = false;

  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<apollo::drivers::RFID>> rfid_reader_;
  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_;
  std::shared_ptr<cyber::Reader<PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<WheelAngle>> wheel_angle_reader_;

  double manual_front_wheel_target = 0;
  double manual_rear_wheel_target = 0;

  std::future<void> async_action_;
  double pid_int = 0;
  double pid_e_pre = 0;
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo
