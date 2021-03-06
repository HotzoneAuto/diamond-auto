#include "modules/drivers/ipcamera/ipcamera_component.h"
namespace apollo {
namespace drivers {
namespace ipcamera {

bool IpCameraComponent::Init() {
  if (!GetProtoConfig(&device_conf_)) {
    AERROR << "Unable to load rfid conf file: " << ConfigFilePath();
    return false;
  }
  // 发现设备
  // discovery device
  CSystem &systemObj = CSystem::getInstance();
  TVector<ICameraPtr> vCameraPtrList;
  bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
  if (!isDiscoverySuccess) {
    AINFO "discovery device fail.\n";
    return 0;
  }

  if (vCameraPtrList.size() == 0) {
    AINFO "no devices.\n";
    return 0;
  }
  // 打印相机基本信息（序号,类型,制造商信息,型号,序列号,用户自定义ID,IP地址）
  // print camera info (index,Type,vendor name, model,serial
  // number,DeviceUserID,IP Address) displayDeviceInfo(vCameraPtrList); int
  // int cameraIndex = selectDevice(vCameraPtrList.size());
  cameraSptr = vCameraPtrList[0];

  // GigE相机时，连接前设置相机Ip与网卡处于同一网段上
  // When a GigE camera is connected, set the camera IP to be on the same
  // network segment as the network card
  if (ICamera::typeGige == cameraSptr->getType()) {
    if (Ipcamera::autoSetCameraIP(cameraSptr) != 0) {
      AINFO "set camera Ip failed.\n";
    }
  }

  // 连接相机
  // connect camera
  if (!cameraSptr->connect()) {
    AINFO "connect cameral failed.\n";
    return 0;
  }

  // 设置相机为连续取流模式
  // set camera to continue grab mode
  Ipcamera::setGrabMode(cameraSptr, true);

  // 创建流对象
  // create acquisitioncontrol object
  streamPtr = systemObj.createStreamSource(cameraSptr);
  if (NULL == streamPtr) {
    AINFO "create stream obj  fail.\r\n";
    return false;
  }
  // 开始取图
  // start grabbing
  bool isStartGrabbingSuccess = streamPtr->startGrabbing();
  if (!isStartGrabbingSuccess) {
    AINFO "StartGrabbing  fail.\n";
  }
  parking_writer_ = node_->CreateWriter<Image>(device_conf_.output_channel());
  async_result_ = cyber::Async(&IpCameraComponent::run, this);
  return true;
}

void IpCameraComponent::run() {
  running_.exchange(true);
  while (!cyber::IsShutdown()) {
    isSuccess = streamPtr->getFrame(frame, 300);
    if (!isSuccess) {
      AERROR << "getFrame  fail";
    }

    // 判断帧的有效性
    // Judge the validity of frame
    bool isValid = frame.valid();
    if (!isValid) {
      AERROR << "frame is invalid!";
    }
    // 7.EN:convert to BRG24(Color Camera)    CN:转换为BRG24格式(彩色相机)
    int nBGRBufferSize = frame.getImageWidth() * frame.getImageHeight() * 3;
    uint8_t *pBGRbuffer = new uint8_t[nBGRBufferSize];

    // 7-1.EN:set param     CN:转换参数
    openParam.width = frame.getImageWidth();
    openParam.height = frame.getImageHeight();
    openParam.paddingX = frame.getImagePadddingX();
    openParam.paddingY = frame.getImagePadddingY();
    openParam.dataSize = frame.getImageSize();
    openParam.pixelForamt = frame.getImagePixelFormat();

    // 7-2.EN:convert to BRG24    CN:转换为BRG24格式
    const void *pImage = frame.getImage();
    status = IMGCNV_ConvertToBGR24((unsigned char *)pImage, &openParam,
                                   pBGRbuffer, &nBGRBufferSize);

    // 7-3.EN:convert to Opencv   CN:转换到Opencv格式
    cv::Mat image = cv::Mat(frame.getImageHeight(), frame.getImageWidth(),
                            CV_8UC3, (uint8_t *)pBGRbuffer);

    // 7-4.EN:display       CN:显示
    // cv::imshow("test", image);
    // cv::waitKey(1);
    auto pb_image = std::make_shared<Image>();
    pb_image->set_width(frame.getImageWidth());
    pb_image->set_height(frame.getImageHeight());
    pb_image->set_data(image.data, nBGRBufferSize);
    parking_writer_->Write(pb_image);
  }
}

IpCameraComponent::~IpCameraComponent() {
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
    // 停止相机拉流
    // stop camera grabbing
    streamPtr->stopGrabbing();

    // 修改相机曝光时间
    // Modify camera exposure time
    // modifyCamralExposureTime(systemObj, cameraSptr);

    // 断开设备
    // disconnect device
    if (!cameraSptr->disConnect()) {
      AINFO "disConnect camera failed\n";
    }
  }
}

}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo
