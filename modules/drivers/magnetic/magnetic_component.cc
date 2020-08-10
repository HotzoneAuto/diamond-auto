
#include "modules/drivers/magnetic/magnetic_component.h"

#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace drivers {
namespace magnetic {

bool MagneticComponent::Init() {
  char str = 0x03;
  uint8_t out;
  stringToHex(&str, &out);
  AINFO << out;
  // Uart device set option
  device_.SetOpt(9600, 8, 'N', 1);

  // Publish rfid station data
  rfid_writer_ = node_->CreateWriter<RFID>(FLAGS_rfid_topic);

  // Async read
  async_action_ = cyber::Async(&MagneticComponent::Action, this);
  return true;
}

// TODO()CHECK
bool MagneticComponent::Check() {}

void MagneticComponent::Action() {
  int count = 0;
  static char buffer[13];
  static char buf;
}

MagneticComponent::~MagneticComponent() {
  AINFO << "MagneticComponent::~MagneticComponent()";
}

}  // namespace magnetic
}  // namespace drivers
}  // namespace apollo
