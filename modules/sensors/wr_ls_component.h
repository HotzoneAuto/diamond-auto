#pragma once

#include <map>
#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/node/node.h"

#include "modules/common/global_gflags.h"
#include "modules/sensors/proto/laser_scan.pb.h"

#include "modules/sensors/wr_ls/wr_ls1207de_parser.h"
#include "modules/sensors/wr_ls/wr_ls_common_tcp.h"


namespace apollo {
namespace sensors {

using apollo::control::Chassis;
using apollo::cyber::Component;
using apollo::sensors::LaserScan;
using apollo::sensors::wr_ls;

class WRLSComponent : public Component<> {
 public:
  bool Init() override;
  int Run();
  ~WRLSComponent();

 private:
   /*Setup TCP connection and attempt to connect/reconnect*/
  wr_ls::CWrLsCommon *pWrLs = nullptr;
  wr_ls::CWrLs1207DEParser *pParser = nullptr;
};

CYBER_REGISTER_COMPONENT(WRLSComponent)
}  // namespace sensors
}  // namespace apollo
