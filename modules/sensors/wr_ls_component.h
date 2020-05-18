#pragma once

#include <map>
#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/node/node.h"

#include "modules/common/global_gflags.h"
#include "modules/sensors/proto/laser_scan.pb.h"

namespace apollo {
namespace sensors {

using apollo::sensors::wr_ls;
using apollo::control::Chassis;
using apollo::cyber::Component;
using apollo::sensors::LaserScan;

class WRLSComponent : public Component<> {
 public:
  bool Init() override;
  void InitDeviceAndSensor();
  void Run();
  ~WRLSComponent();

 private:


};

CYBER_REGISTER_COMPONENT(WRLSComponent)
}  // namespace sensors
}  // namespace apollo
