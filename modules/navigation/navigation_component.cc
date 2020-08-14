
#include "modules/drivers/rfid/rfid_component.h"

#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace navigation {

float distance(int b) {
  float c;
  if (b == 0x0001)
    c = 8.5;
  else if (b == 0x8000)
    c = -8.5;
  else if (b == 0x0003)
    c = 7.5;
  else if (b == 0xC000)
    c = -7.5;
  else if (b == 0x0007)
    c = 6.5;
  else if (b == 0xE000)
    c = -6.5;
  else
    c = 0;
  return c;
}

bool NavigationComponent::Init() {
  // Init Reader
  magnetic_reader_ = node_->CreateReader<Magnetic>(
      FLAGS_magnetic_channel,
      [this](const std::shared_ptr<Magnetic>& magnetic) {
        ADEBUG << "Received Magnetic message. run callback.";
        magnetic_.Clear();
        magnetic_.CopyFrom(*magnetic);
      });

  // Init Writer
  navigation_writer_ = node_->CreateWriter<Distance>(FLAGS_navigation_channel);

  // Async read
  async_action_ = cyber::Async(&RfidComponent::Action, this);

  return true;
}

void NavigationComponent::Action() {
  Distance distance;
  auto header = distance.mutable_header();
  header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
  header->set_frame_id("rfid");

  auto mag = magnetic_.channel();

  distance.set_distance(distance(mag));
}

NavigationComponent::~NavigationComponent() { AINFO << "~NavigationComponent"; }

}  // namespace navigation
}  // namespace apollo
