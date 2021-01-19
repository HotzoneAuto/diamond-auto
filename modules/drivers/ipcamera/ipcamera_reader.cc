
#include "modules/drivers/ipcamera/ipcamera_reader.h"

namespace apollo {
namespace drivers {
namespace ipcamera {

bool IpcameraReader::Init() {
  if (!GetProtoConfig(&device_conf_)) {
    AERROR << "Unable to load IpcameraReader conf file: " << ConfigFilePath();
    return false;
  }
  // Publish IpcameraReader station data
  ipcamera_writer_ = node_->CreateReader<Image>(
      device_conf_.output_channel(),
      [this](const std::shared_ptr<Image>& image) { this->Proc(image); });
  // Async read
  // async_action_ = cyber::Async(&IpcameraReader::Proc, this);
  return true;
}

void IpcameraReader::Proc(const std::shared_ptr<Image>& image) {
  cv::Mat new_image;
  running_.exchange(true);
  new_image = cv::Mat(static_cast<int>(image->height()),
                      static_cast<int>(image->width()), CV_8UC3,
                      (uint8_t*)image->data().c_str());
  cv::imshow("test", new_image);
  cv::waitKey(1);
}

IpcameraReader::~IpcameraReader() {
  if (running_.load()) {
    running_.exchange(false);
    // async_result_.wait();
  }
}

}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo
