
#include "modules/drivers/ipcamera/ipcompress_component.h"
namespace apollo {
namespace drivers {
namespace ipcamera {
    bool IpComressComponent::Init(){
        AINFO << "Init";
        return false;
    }
    void IpCameraComponent::run(){
        AINFO << "run";
        return false;
    }
CameraComponent::~CameraComponent() {
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo