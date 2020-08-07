#pragma once

#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/common/global_gflags.h"


namespace apollo {
namespace control {
using apollo::cyber::Component;

class ControlComponent: public Component<>{
public:
  ControlComponent();
  bool ControlComponent::Init() override;
  void ControlComponent::GenerateCommand();
  ~ControlComponent();

  Chassis chassis_;
  
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo