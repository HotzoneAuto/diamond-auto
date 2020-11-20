#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace ipcamera {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
class IpCameraComponent : public Component<> {
  public:
    bool Init() override;
    ~IpCameraComponent();
  private:
    void run();
    std::future<void> async_result_;
    std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(IpCameraComponent)
}  // namespace ipcamera
}  // namespace drivers
}  // namespace apoll