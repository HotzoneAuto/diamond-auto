
#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/uart.h"
#include "modules/drivers/proto/rfid.pb.h"

namespace apollo {
namespace drivers {
namespace rfid {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::drivers::RFID;

class RfidComponent : public apollo::cyber::Component<> {
 public:
  RfidComponent();
  std::string Name() const;
  bool Init() override;
  void Action();
  bool Check();
  ~RfidComponent();

 private:
  // TODO(all): config by udev or sudo usermod -aG dialout $USER
  Uart device_ = Uart("ttyUSB1");
  std::shared_ptr<Writer<RFID>> rfid_writer_ = nullptr;

  std::future<void> async_action_;
  // atomic flag for action
  //   std::atomic<bool> action_ready_ = {false};
};

CYBER_REGISTER_COMPONENT(RfidComponent)

}  // namespace rfid
}  // namespace drivers
}  // namespace apollo
