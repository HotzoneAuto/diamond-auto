
#include "modules/drivers/ipcamera/ipcamera_reader.h"

namespace apollo {
namespace drivers {
namespace ipcamera {

bool IpcameraReader::Init() {
  // if (!GetProtoConfig(&device_conf_)) {
  //   AERROR << "Unable to load rfid conf file: " << ConfigFilePath();
  //   return false;
  // }
  // Publish rfid station data
  // rfid_writer_ = node_->CreateReader<Image>(device_conf_.output_channel());
  ipcamera_writer_ = node_->CreateReader<Image>(
      "/diamond/sensor/ipcamera/front",
      [this](const std::shared_ptr<Image>& ipcamera_front) {
        ipcamera_front_.CopyFrom(*ipcamera_front);
      });
  // Async read
  async_action_ = cyber::Async(&IpcameraReader::Proc, this);
  return true;
}

void IpcameraReader::Proc() {
  while (!apollo::cyber::IsShutdown()) {
    AINFO << "ipcamera_front_.width=" << ipcamera_front_.width();
    AINFO << "ipcamera_front_.height=" << ipcamera_front_.height();
    cv::Mat img(1280 * 1080, 3, CV_8UC3);

    // cv::imshow("test",img);
    // cv::waitKey(1);
  }
}

}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo
