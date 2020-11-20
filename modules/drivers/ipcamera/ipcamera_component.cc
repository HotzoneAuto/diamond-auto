#include "modules/drivers/ipcamera/ipcamera_component.h"
namespace apollo {
namespace drivers {
namespace ipcamera {
    bool IpCameraComponent::Init(){
        AINFO << "Init";
        return false;
    }
    void IpCameraComponent::run(){
        AINFO << "run";
        // return false;
    }
IpCameraComponent::~IpCameraComponent() {
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo