#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"
#include "cyber/cyber.h"

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"

#include "modules/canbus/vehicle/diamond/protocol/id_0x00aa5701.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0cfff3a7.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class mortor_vol_up : public Component<apollo::canbus::ChassisDetail> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::canbus::ChassisDetail>& msg) override;
};
CYBER_REGISTER_COMPONENT(mortor_vol_up)
