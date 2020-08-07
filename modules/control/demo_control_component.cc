#include "modules/control/demo_control_component.h"

#include <string>
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "math.h"

namespace apollo {
namespace control {

using apollo::cyber::Rate;
using apollo::cyber::Time;

bool ControlComponent::Init() {
  // Reader
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        chassis_.CopyFrom(*chassis);
      });

  // create Writer
  control_cmd_writer_ = node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);

  // compute control message in aysnc
  async_action_ = cyber::Async(&ControlComponent::GenerateCommand, this);

  return true;
}

// write to channel
void ControlComponent::GenerateCommand() {
  auto cmd = std::make_shared<ControlCommand>();

  // frequency, TODO: reset
  Rate rate(20.0);

  while (true) {
    float lat_dev_mgs =
        5;  // TODO: lateral_dev_mgs need to changed according to MGS module.
    if (lat_dev_mgs < -4.5 || lat_dev_mgs > 4.5) {
      cmd->set_steering_target(10);
    } else {
      cmd->set_steering_target(0);
    }
    control_cmd_writer_->Write(cmd);
    rate.Sleep();
  }
}

ControlComponent::~ControlComponent() {
  // back chassis handle
  async_action_.wait();
}

}  // namespace control
}  // namespace apollo
