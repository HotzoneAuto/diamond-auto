
#include "modules/navigation/navigation_component.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace navigation {

std::string NavigationComponent::Name() const { return "Navigation"; }

bool NavigationComponent::Init() {

  // front rfid Reader
  rfid_front_reader_ = node_->CreateReader<RFID>(
      FLAGS_rfid_front_topic, [this](const std::shared_ptr<RFID>& rfid_front) {
        rfid_front_.CopyFrom(*rfid_front);
      });

  // rear rfid Reader
  rfid_rear_reader_ = node_->CreateReader<RFID>(
      FLAGS_rfid_rear_topic, [this](const std::shared_ptr<RFID>& rfid_rear) {
        rfid_rear_.CopyFrom(*rfid_rear);
      });


    // create Writer
  navigation_writer_ =
      node_->CreateWriter<Navigation>(FLAGS_routing_request_topic);

  return true;
}

void NavigationComponent::Action() {
  auto nav = std::make_shared<Navigation>();
  // belef from apriltag or rfid
  nav->set_start_point(49);
  nav->set_end_point(50);

  common::util::FillHeader(node_->Name(), nav.get());
  navigation_writer_->Write(nav);

}

NavigationComponent::~NavigationComponent() { AINFO << "~NavigationComponent"; }

}  // namespace navigation
}  // namespace apollo
