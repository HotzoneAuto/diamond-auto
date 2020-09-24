
#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/drivers/proto/rfid.pb.h"
#include "modules/navigation/proto/navigation.pb.h"

namespace apollo {
namespace navigation {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::RFID;
using apollo::navigation::Navigation;

class NavigationComponent : public Component<> {
 public:
  bool Init() override;
  void Action();
  ~NavigationComponent();
  std::string Name() const;

 private:
  std::shared_ptr<Writer<Navigation>> navigation_writer_ = nullptr;

  std::shared_ptr<cyber::Reader<apollo::drivers::RFID>> rfid_front_reader_;
  std::shared_ptr<cyber::Reader<apollo::drivers::RFID>> rfid_rear_reader_;

  RFID rfid_front_;
  RFID rfid_rear_;
  std::future<void> async_action_;
};

CYBER_REGISTER_COMPONENT(NavigationComponent)

}  // namespace navigation
}  // namespace apollo
