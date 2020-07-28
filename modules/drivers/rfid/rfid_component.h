
#pragma once
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/common/util/uart.h"
#include "modules/drivers/proto/rfid.pb.h"

namespace apollo {
namespace drivers {
namespace rfid {

using apollo::cyber::Component;

class RfidComponent : public Component<> {
 public:
  bool Init() override;
  void Action();
  ~RfidComponent();

 private:
  Uart device_ = Uart("ttyUSB0");
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::RFID>> rfid_writer_ = nullptr;

std::future<void> async_action_;
  // atomic flag for action
  //   std::atomic<bool> action_ready_ = {false};
};

CYBER_REGISTER_COMPONENT(RfidComponent)

}  // namespace rfid
}  // namespace drivers
}  // namespace apollo