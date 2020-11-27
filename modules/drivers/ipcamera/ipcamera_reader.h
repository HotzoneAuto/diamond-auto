#pragma once
#include <memory>

#include "cyber/cyber.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cyber/component/component.h"
#include "modules/drivers/ipcamera/proto/ipcamera_conf.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
namespace apollo {
namespace drivers {
namespace ipcamera {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::drivers::Image;

class IpcameraReader : public Component<> {
 public:
  ~IpcameraReader();
  bool Init() override;
  void Proc(const std::shared_ptr<Image>& image);

 private:
  Image ipcamera_front_;
  IpcameraDeviceConf device_conf_;
  std::shared_ptr<Reader<Image>> ipcamera_writer_ = nullptr;
  std::future<void> async_action_;
  std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(IpcameraReader)
}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo
