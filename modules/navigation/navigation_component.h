
#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/drivers/proto/magnetic.pb.h"
#include "modules/navigation/proto/navigation.pb.h"

namespace apollo {
namespace navigation {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::Magnetic;
using apollo::navigation::Distance;

class NavigationComponent : public Component<> {
 public:
  bool Init() override;
  void Action();
  ~NavigationComponent();

 private:
  std::shared_ptr<Writer<Distance>> navigation_writer_ = nullptr;
  std::shared_ptr<Reader<Magnetic>> magnetic_reader_ = nullptr;

  Magnetic magnetic_;

  std::future<void> async_action_;
};

CYBER_REGISTER_COMPONENT(NavigationComponent)

}  // namespace navigation
}  // namespace apollo
