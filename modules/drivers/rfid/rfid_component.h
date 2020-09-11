
#pragma once
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/uart.h"
#include "modules/drivers/proto/rfid.pb.h"
#include "modules/drivers/rfid/proto/rfid_device_conf.pb.h"

namespace apollo {
namespace drivers {
namespace rfid {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::drivers::RFID;

class RfidComponent : public Component<> {
 public:
  RfidComponent();
  std::string Name() const;
  bool Init() override;
  void Action();
  bool Check();
  ~RfidComponent();

 private:
  RFIDDeviceConf device_conf_;

  // TODO(all): auto config by udev
  std::unique_ptr<Uart> device_ = nullptr;

  std::shared_ptr<Writer<RFID>> rfid_writer_ = nullptr;

  std::future<void> async_action_;
};

CYBER_REGISTER_COMPONENT(RfidComponent)

}  // namespace rfid
}  // namespace drivers
}  // namespace apollo
