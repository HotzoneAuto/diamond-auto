
#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/uart.h"
#include "modules/drivers/proto/magnetic.pb.h"

namespace apollo {
namespace drivers {
namespace magnetic {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::drivers::Magnetic;

class MagneticComponent : public Component<> {
 public:
  bool Init() override;
  void Action();
  bool Check();
  ~MagneticComponent();

 private:
  // TODO(wangying):
  // 1. config by udev
  // 2. or sudo usermod -aG dialout $USER
  Uart device_mgs_front = Uart("ttyUSB0"); 
  Uart device_mgs_rear = Uart("ttyUSB1"); 

  std::shared_ptr<Writer<Magnetic>> magnetic_writer_ = nullptr;

  std::future<void> async_action_;
};

CYBER_REGISTER_COMPONENT(MagneticComponent)

}  // namespace magnetic
}  // namespace drivers
}  // namespace apollo
