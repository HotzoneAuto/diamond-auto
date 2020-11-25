#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "modules/drivers/ipcamera/ipcamera_main.h"
#include "modules/drivers/proto/sensor_image.pb.h"

#include "modules/drivers/ipcamera/proto/ipcamera_conf.pb.h"

namespace apollo {
namespace drivers {
namespace ipcamera {

using apollo::cyber::Component;
using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::Writer;
using apollo::drivers::Image;

class IpCameraComponent : public Component<> {
 public:
  bool Init() override;
  void run();
  ~IpCameraComponent();

 private:
  bool isSuccess=false;
  IpcameraDeviceConf device_conf_;
  IStreamSourcePtr m_streamSptr;
  CFrame frame;
  ICameraPtr cameraSptr;
  IStreamSourcePtr streamPtr;
  std::future<void> async_result_;
  std::shared_ptr<Writer<Image>> parking_writer_ = nullptr;
  std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(IpCameraComponent)
}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo